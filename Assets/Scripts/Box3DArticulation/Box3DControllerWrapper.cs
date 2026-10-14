using System;
using Box3D;
using Box3D.Hybrid;
using MxMGameplay;
using Unity.Mathematics;
using UnityEngine;
using PopcronGizmos = Popcron.Gizmos;

namespace Box3DArticulation
{
    /// <summary>
    /// Drop-in Box3D replacement for UnityControllerWrapper + PhysX
    /// CharacterController. Implements the same GenericControllerWrapper
    /// seam, so MxMRootMotionApplicator needs no changes: Move() receives
    /// a displacement and moves the TRANSFORM with collide-and-slide; the
    /// transform stays the single authority every consumer already reads
    /// (trajectory generator, stream tap, god forces, coupling).
    ///
    /// Collision model: Box3D's kinematic mover toolkit (the same one the
    /// FootStabilizer uses), per move step:
    ///   CollideMover -> collision planes at the current pose,
    ///   SolvePlanes  -> corrected delta (slide + depenetration push),
    ///   CastMover    -> capsule sweep of that delta (anti-tunneling),
    ///   apply skin-short of the sweep fraction, iterate (max bounces).
    /// The capsule tilts with the character: its segment runs along the
    /// smoothed surface up and its center rides the transform's frame
    /// (center offset rotates with the transform), so on slopes the query
    /// shape matches the leaned character instead of a vertical stick.
    ///
    /// Step-up (stepOffset) and slope limits are NOT implemented (flat
    /// prototype floors); MaxStepHeight is stored but unused. Grounded =
    /// a small post-move down-sweep. The transform's up aligns to a blend of
    /// -World.GravityVector and the predicted averaged surface normal (from
    /// the post-sweep collide pass); the collision capsule leans with it.
    ///
    /// World presence (optional): assign a Kinematic Box3DBody with a
    /// Box3DCapsuleShape child so other dynamic bodies can collide with the
    /// character. Kinematic bodies are pushed from their Transform by
    /// Box3DWorld each step, so the capsule follows this transform with no
    /// extra wiring. Keep the capsule child's layer unticked in Collision Mask
    /// so the mover queries never hit it; also filter it against the rig
    /// bodies if they overlap.
    /// </summary>
    public sealed class Box3DControllerWrapper : GenericControllerWrapper
    {
        [Header("Capsule (world-vertical, like Unity's CC)")]
        [SerializeField, Tooltip("Capsule radius (m). Keep equal to the optional Box3DCapsuleShape child.")]
        private float radius = 0.3f;
        [SerializeField, Tooltip("Total capsule height including caps (m).")]
        private float height = 1.8f;
        [SerializeField, Tooltip("Capsule center offset from the transform position (m; in the transform's own frame - rotates with the character's tilt). Character pivots at the feet, so the default is half the height upwards.")]
        private Vector3 center = new Vector3(0f, 0.9f, 0f); // height 1.8 / 2
        [SerializeField, Tooltip("Collision skin (m): sweeps stop this far short of a hit so numerical jitter never re-penetrates.")]
        private float skinWidth = 0.02f;
        [SerializeField, Range(1, 5), Tooltip("Max collide-and-slide iterations per move.")]
        private int maxBounces = 3;
        [SerializeField, Tooltip("Extra sweep distance below the capsule sole for the grounded test (m).")]
        private float groundExtra = 0.05f;

        [Header("World")]
        [SerializeField, Tooltip("Layers the character collides with (the mover queries' MaskBits). Default: Everything - untick the optional world-presence capsule child's layer so the character never collides with itself.")]
        private LayerMask collisionMask = ~0;
        [SerializeField, Tooltip("Optional kinematic Box3DBody whose capsule shape gives the character world presence (other bodies collide with it). Follows this transform automatically. Keep its layer unticked in Collision Mask.")]
        private Box3DBody body;

        [Header("Slope alignment (gravity-referenced)")]
        [SerializeField, Range(0f, 1f), Tooltip("Authority of the trajectory's nearest NEW surface over the current under-foot support. 0 = ignore prediction, hold current support; 1 = full predicted tilt. No-evidence frames (standing still, airborne) always hold - no gravity erosion.")]
        private float upPredictionBlend = 1f;
        [SerializeField, Tooltip("Lookahead time (s) for tilt prediction: one extra sweep + collide pass along the move direction, sampling the surface ahead of the current step. Sweep distance = speed * time, capped. 0 = off.")]
        private float lookaheadTime = 0.15f;
        [SerializeField, Tooltip("Cap on the lookahead sweep distance (m) so high speed cannot query absurd ranges.")]
        private float maxLookaheadDistance = 1.5f;
        [SerializeField, Tooltip("Minimum lookahead distance (m): time-based distance shrinks with speed, so slow walking would sample centimeters ahead. The lookahead never samples less than this floor (unless capped by maxLookaheadDistance).")]
        private float minLookaheadDistance = 0.4f;
        [SerializeField, Range(1, 4), Tooltip("Slide/redirect iterations INSIDE the lookahead: the virtual trajectory is integrated through the lookahead distance with up to this many collide-and-slide redirects, so it bends around corners instead of sampling one point.")]
        private int lookaheadIterations = 2;
        [SerializeField, Tooltip("Cast velocity smoothing rate (1/s): the lookahead sweeps along a low-passed measured velocity instead of the raw per-step delta, so direction changes ramp and brief root-motion gaps keep leading with residual speed (inertia-like). 0 = raw delta.")]
        private float castVelRate = 10f;
        // Lookahead-only: when the virtual sweep blocks, the virtual capsule
        // advances this far INTO the obstacle so the follow-up CollideMover
        // finds overlap (it reports overlap-only, no speculative margin -
        // verified by MoverLookaheadTests). Same scale as skinWidth.
        private const float PenetrationProbe = 0.02f;
        // Cast speed below this = standing still: skip the lookahead entirely
        // (no meaningful approach direction; see the gating in MoveCapsule).
        private const float StillSpeedEpsilon = 0.05f;
        [SerializeField, Tooltip("Up-vector blend rate (1/s) toward the predicted up. Higher = snappier slope alignment.")]
        private float upBlendRate = 8f;
        [SerializeField, Tooltip("Contacts whose facet normal dots less than this with the current up are walls and excluded from the up average.")]
        private float upMinGroundDot = 0.3f;

        private bool m_collisionEnabled = true;
        private Vector3 m_velocity;
        private Vector3 m_cachedMove;
        private Quaternion m_cachedRot = Quaternion.identity;
        private readonly MoverHit[] _hits = new MoverHit[8]; // gizmo scratch (no per-frame alloc)
        private int _hitCount;
        private float3 _targetUp = new float3(0f, 1f, 0f);   // blend(gravityUp, predicted surface up)
        private float3 _smoothUp = new float3(0f, 1f, 0f);   // low-passed, drives transform tilt
        private float _moveEfficiency = 1f;                  // applied / intended move length (trajectory gating)
        private float3 _castVel = new float3(0f, 0f, 0f);    // low-passed measured velocity (inertia-aware cast)
        private readonly float3[] _lookPts = new float3[5];  // gizmo: predicted trajectory waypoints (no alloc)
        private int _lookCount;
        private float _lookArc;                              // predicted polyline arc length (lead parameterization)
        private float3 _lookGroundOffset;                    // gizmo: capsule center -> current ground contact

        [Header("Debug (Popcron GL)")]
        [SerializeField, Tooltip("Draw the mover capsule, the last collide pass contacts and the ground probe through Popcron.Gizmos.")]
        private bool drawGizmos = true;

        private World World => Box3DWorld.Instance != null ? Box3DWorld.Instance.World : default;

        // --- GenericControllerWrapper ---------------------------------------

        /// <summary>Capsule center in world space: the transform's own frame
        /// carries the center offset. The transform pivots at the feet and
        /// tilts with the slope, so a world-axes offset would leave the capsule
        /// hanging off the character's central axis on ramps (offset along the
        /// ramp growing with the tilt).</summary>
        private float3 CenterWorld()
        {
            return (float3)(transform.rotation * center)
                + (float3)transform.position;
        }

        public override bool IsGrounded
        {
            get
            {
                var w = World;
                if (!w.IsValid) return false;
                return ProbeGround(CenterWorld());
            }
        }

        public override bool ApplyGravityWhenGrounded => false;

        public override Vector3 Velocity => m_velocity;

        /// <summary>Low-passed surface up driving the transform tilt (for
        /// consumers that want to align to the same slope reference).</summary>
        public float3 SmoothUp => _smoothUp;

        /// <summary>Waypoint count / accessor for the last predicted trajectory
        /// (diagnostics + EditMode tests; the gizmo polyline uses the same).</summary>
        public int LookPointCount => _lookCount;
        public float3 GetLookPoint(int i) => _lookPts[i];

        /// <summary>Applied / intended move length of the last Move(), lightly
        /// smoothed: ~1 on free motion, drops when collide-and-slide blocks or
        /// shortens the motion. Lets downstream prediction (god forces) gate
        /// anticipated velocities by what the trajectory can actually do.</summary>
        public float MoveEfficiency => _moveEfficiency;

        /// <summary>Displacement along the last predicted trajectory after
        /// aheadTime seconds of travel at the cast velocity: follows the
        /// collide-and-slide redirects (up ramps, along walls) instead of a
        /// straight clip-velocity integral. False when no trajectory was built
        /// this step (idle / no lookahead).</summary>
        public bool TryGetPredictedLead(float aheadTime, out float3 offset)
        {
            offset = default;
            if (_lookCount < 2) return false;
            float speed = math.length(_castVel);
            if (speed < 1e-4f) return false;
            float arc = math.min(speed * math.max(aheadTime, 0f), _lookArc);
            float3 p = _lookPts[0];
            for (int i = 1; i < _lookCount && arc > 1e-8f; i++)
            {
                float3 seg = _lookPts[i] - p;
                float len = math.length(seg);
                if (len > 1e-8f)
                {
                    if (len >= arc) { p += seg * (arc / len); arc = 0f; break; }
                    arc -= len;
                }
                // else: duplicate waypoint (zero-progress block) - skip.
                p = _lookPts[i];
            }
            offset = p - _lookPts[0];
            return true;
        }

        public override float MaxStepHeight { get; set; } = 0.3f; // not implemented (flat prototype floors)

        public override float Height { get => height; set => height = value; }
        public override Vector3 Center { get => center; set => center = value; }
        public override float Radius { get => radius; set => radius = value; }

        public override bool CollisionEnabled
        {
            get => m_collisionEnabled;
            set
            {
                if (m_collisionEnabled == value) return;
                m_collisionEnabled = value;
                // Same semantics as UnityControllerWrapper disabling the CC:
                // no world presence while "disabled" (Move falls back to a free
                // translate).
                if (body != null && body.Body.IsValid)
                {
                    if (value) body.Body.Enable();
                    else body.Body.Disable();
                }
            }
        }

        public override void Initialize() { }

        public override Vector3 GetCachedMoveDelta() => m_cachedMove;
        public override Quaternion GetCachedRotDelta() => m_cachedRot;

        public override void Move(Vector3 a_move) => ApplyMove(a_move);

        public override void MoveAndRotate(Vector3 a_move, Quaternion a_rotDelta)
        {
            transform.rotation *= a_rotDelta;
            m_cachedRot = a_rotDelta;
            ApplyMove(a_move);
        }

        public override void Rotate(Quaternion a_rotDelta)
        {
            transform.rotation *= a_rotDelta;
            m_cachedRot = a_rotDelta;
        }

        public override void SetPosition(Vector3 a_position)
        {
            transform.position = a_position;
            m_velocity = Vector3.zero;
        }

        public override void SetRotation(Quaternion a_rotation) => transform.rotation = a_rotation;

        public override void SetPositionAndRotation(Vector3 a_position, Quaternion a_rotation)
        {
            transform.SetPositionAndRotation(a_position, a_rotation);
            m_velocity = Vector3.zero;
        }

        // --- collide-and-slide (Box3D mover toolkit) --------------------------

        private void ApplyMove(Vector3 a_move)
        {
            Vector3 start = transform.position;
            float dt = Time.inFixedTimeStep ? Time.fixedDeltaTime : Time.deltaTime;

            if (!m_collisionEnabled || !World.IsValid)
            {
                transform.position = start + a_move; // CC-disabled path: free translate
                _targetUp = _smoothUp = GravityUp(); // no contacts: snap to gravity up
            }
            else
            {
                // Capsule origin = transform + center offset IN THE
                // TRANSFORM'S FRAME (tilted with the character).
                float3 origin = CenterWorld();
                origin = MoveCapsule(origin, (float3)a_move, dt);
                transform.position = (Vector3)(origin
                    - (float3)(transform.rotation * center));
            }

            ApplyUpBlend(dt); // slope alignment: up = blend(gravity, predicted surface)
            if (dt > 1e-5f) m_velocity = (transform.position - start) / dt;
            m_cachedMove = transform.position - start;

            // Trajectory gating for downstream prediction: how much of the
            // intended motion survived collide-and-slide (1 = free, 0 = wall).
            float appliedLen = m_cachedMove.magnitude;
            float wantLen = a_move.magnitude;
            if (wantLen > 1e-5f)
                _moveEfficiency = math.lerp(_moveEfficiency, appliedLen / wantLen, 0.3f);
        }

        /// <summary>Collide-and-slide loop over the mover toolkit: gather planes
        /// at the current pose, solve the remaining delta against them (slide +
        /// depenetration), sweep the result skin-short, iterate. The loop's last
        /// collide pass is the CURRENT-pose baseline; a virtual lookahead
        /// trajectory provides the ENDPOINT support. Contacts are also mirrored
        /// into _hits for the gizmo pass.</summary>
        private float3 MoveCapsule(float3 origin, float3 delta, float dt)
        {
            Capsule cap = CapsuleFor(_smoothUp);
            QueryFilter filter = MoverFilter();
            Span<CollisionPlane> planes = stackalloc CollisionPlane[8];
            Span<MoverHit> curHits = stackalloc MoverHit[8];
            Span<MoverHit> aheadHits = stackalloc MoverHit[8];
            float3 remaining = delta;
            float3 curSum = new float3(0f, 0f, 0f);
            int curN = 0;
            for (int iter = 0; iter < maxBounces; iter++)
            {
                int count = World.CollideMover(origin, cap, filter, planes, curHits);
                // Accumulate up-facing normals across ALL bounce iterations:
                // in a V-valley/corner each pass contacts a different walkable
                // face, and only the union gives the correct bisected up.
                int hitCount = math.min(count, curHits.Length);
                float best = float.MaxValue;
                for (int i = 0; i < hitCount; i++)
                {
                    float3 nn = curHits[i].Normal;
                    if (math.dot(nn, _smoothUp) < upMinGroundDot) continue;
                    curSum += nn;
                    curN++;
                    // Gizmo anchor: the lowest (most under-foot) contact point.
                    float d = math.dot(curHits[i].Point - origin, _smoothUp);
                    if (d < best) { best = d; _lookGroundOffset = curHits[i].Point - origin; }
                }
                float3 solved = count > 0
                    ? Mover.SolvePlanes(remaining, planes).Delta
                    : remaining;
                float len = math.length(solved);
                if (len < 1e-6f) break;

                float frac = World.CastMover(origin, cap, solved, filter);
                // Stop skin-short of the hit so the next CollideMover keeps a
                // stable separating plane instead of re-penetrating.
                float take = math.max(0f, math.min(frac, 1f)
                    - math.min(skinWidth / len, 0.5f));
                float3 step = solved * take;
                origin += step;
                remaining -= step;
                if (math.lengthsq(remaining) < 1e-10f) break;
            }

            // Tilt lookahead: integrate a short VIRTUAL trajectory from the
            // cast velocity, up to lookaheadIterations collide-and-slide
            // redirect slices inside the prediction window, so the sampled
            // path bends around obstacles and drops off ledges. The predicted
            // support is the nearest NEW surface the path contacts. The
            // redirect IS the relevance filter - surfaces the path never
            // lands on don't count.
            float dtc = math.max(dt, 1e-5f);
            _castVel = math.lerp(_castVel, delta / dtc,
                castVelRate > 0f ? 1f - math.exp(-castVelRate * dt) : 1f);
            float speed = math.length(_castVel);
            float lookDist = math.max(minLookaheadDistance,
                math.min(speed * lookaheadTime, maxLookaheadDistance));
            // Two distinct references, never merged:
            // - HOLD anchor (what to keep when the trajectory finds nothing):
            //   moving → _smoothUp (it follows _targetUp, so persistent
            //   evidence converges; a measured-contact average can be pinned
            //   by stale overlap - ramp-base contacts while already on flat -
            //   and freeze the target at a lerp fraction). Standing → measured
            //   average (nothing follows; resting contacts are the truth).
            // - DUPLICATE filter reference ("is this contact a repeat of what
            //   I stand on"): only measured contacts, only while they exist.
            //   Filtering against the smoothed up goes circular (stalls the
            //   residual tilt inside the band); floating → no filtering.
            bool moving = lookDist > 1e-3f && speed >= StillSpeedEpsilon;
            float3 measuredAvg = curN > 0
                ? math.normalizesafe(curSum / curN, _smoothUp)
                : _smoothUp;
            float3 baselineAvg = moving ? _smoothUp : measuredAvg;
            float3 bestN;
            if (moving)
            {
                var traj = IntegrateTrajectory(World, cap, filter, origin,
                    _castVel, GravityVector(), measuredAvg, curN > 0, planes, aheadHits);
                bestN = traj.BestNormal;
            }
            else
            {
                // Standing still (or no budget): no approach direction. The
                // stale cast direction would sample surfaces the character is
                // NOT moving toward (e.g. the flat floor past a ramp edge) and
                // let them override the under-foot support - upright creep.
                _lookCount = 0;
                _lookArc = 0f;
                bestN = new float3(0f, 0f, 0f);
            }

            _targetUp = PredictedUp(baselineAvg, bestN);

            // Re-seat flush: the sweep clamp leaves the resting pose skinWidth
            // short of the support (motion anti-jitter - keep that), but the
            // bias must not survive into the RESTING pose: the rig's own body
            // colliders rest flush, so on a slope the gap points along the
            // slope normal and shears the skinned mesh off the capsule.
            // Closing it along the support normal makes the mover seat exactly
            // where the character's colliders do; next frame's overlap-only
            // collide + depenetration absorbs the hair of contact, same as it
            // does for any genuine penetration.
            if (curN > 0)
                origin += math.normalizesafe(curSum / curN) * skinWidth;
            return origin;
        }

        /// <summary>Predicted trajectory integration result.</summary>
        public struct TrajectoryResult
        {
            /// <summary>Nearest new support normal along the approach (zero =
            /// the trajectory touched no non-baseline surface).</summary>
            public float3 BestNormal;
            /// <summary>Distance along the approach to that support.</summary>
            public float BestDistance;
        }

        /// <summary>Virtual collide-and-slide trajectory with gravity: the
        /// prediction window splits into lookaheadIterations time slices.
        /// Supported slices (contacts at the slice start) sweep straight along
        /// the carried velocity - root motion holds height, the ground cancels
        /// gravity. Air slices go ballistic (inertia + accumulated gravity) so
        /// the path drops off ledges and sags into the surface below - that
        /// sag is what discovers the landing support. Public for EditMode
        /// ground-truth tests (MoverLookaheadTests) and diagnostics; also
        /// fills the _lookPts/_lookCount/_lookArc gizmo/lead state.
        /// planes/hits: scratch buffers (8 capacity recommended).</summary>
        public TrajectoryResult IntegrateTrajectory(World world, in Capsule cap,
            QueryFilter filter, float3 origin, float3 castVel, float3 gravity,
            float3 measuredAvg, bool dupFilterActive,
            Span<CollisionPlane> planes, Span<MoverHit> hits)
        {
            var result = new TrajectoryResult
            {
                BestNormal = new float3(0f, 0f, 0f),
                BestDistance = float.MaxValue,
            };
            float speed = math.length(castVel);
            if (speed < 1e-4f) { _lookCount = 0; _lookArc = 0f; return result; }

            float lookDist = math.max(minLookaheadDistance,
                math.min(speed * lookaheadTime, maxLookaheadDistance));
            float3 dir = castVel / speed;
            float winT = math.min(lookDist / speed, lookaheadTime);
            float dtSeg = winT / math.max(lookaheadIterations, 1);
            float3 v = castVel;
            float3 vOrigin = origin;
            _lookPts[0] = vOrigin;
            _lookCount = 1;
            for (int iter = 0; iter < lookaheadIterations; iter++)
            {
                // This pass's contacts join the nearest-in-approach
                // selection (walls and baseline duplicates skipped) and
                // classify the slice: contacts = supported (hug the
                // ground), none = airborne (ballistic).
                int count = world.CollideMover(vOrigin, cap, filter, planes, hits);
                TrackNearestAhead(hits, count, origin, dir, _smoothUp,
                    upMinGroundDot, measuredAvg, dupFilterActive, ref result.BestNormal, ref result.BestDistance);

                float3 segTarget;
                if (count > 0)
                {
                    // Supported: strip the into-surface velocity (facet
                    // normals are index-aligned with the planes), so the
                    // next slice slides along the surface instead of
                    // re-curving into it; no gravity sag while grounded.
                    int pc = math.min(count, hits.Length);
                    for (int i = 0; i < pc; i++)
                    {
                        float3 nn = hits[i].Normal;
                        v -= nn * math.min(0f, math.dot(v, nn));
                    }
                    segTarget = v * dtSeg;
                }
                else
                {
                    // Airborne: gravity accelerates the carried velocity;
                    // displacement = inertia + half-g sag.
                    v += gravity * dtSeg;
                    segTarget = v * dtSeg + 0.5f * gravity * (dtSeg * dtSeg);
                }

                float3 solved = count > 0
                    ? Mover.SolvePlanes(segTarget, planes).Delta
                    : segTarget;
                float len = math.length(solved);
                if (len < 1e-6f) break;
                float frac = world.CastMover(vOrigin, cap, solved, filter);
                // Blocked: advance to contact PLUS a small penetration
                // probe - CollideMover reports overlap-only (no
                // speculative margin, proven by MoverLookaheadTests) and
                // near-touch re-sweeps are unreliable, so without probing
                // into the face the next collide pass finds nothing and
                // the path dead-ends at first contact. A free sweep just
                // ends the slice - the arc keeps bending, so unlike the
                // old straight-ray budget it does NOT end the trajectory.
                float advance = frac >= 1f
                    ? len
                    : frac * len + PenetrationProbe;
                vOrigin += solved * (advance / len);
                _lookPts[_lookCount++] = vOrigin;
            }

            // Final pose support: the last collide ran before the final
            // advance of the last slice - re-sample at the endpoint pose.
            if (_lookCount >= 2)
            {
                int endCount = world.CollideMover(_lookPts[_lookCount - 1], cap, filter, planes, hits);
                TrackNearestAhead(hits, endCount, origin, dir, _smoothUp,
                    upMinGroundDot, measuredAvg, dupFilterActive, ref result.BestNormal, ref result.BestDistance);
            }

            // Arc length of the predicted path: parameterizes
            // TryGetPredictedLead (time along path = arc / cast speed).
            _lookArc = 0f;
            for (int i = 1; i < _lookCount; i++)
                _lookArc += math.distance(_lookPts[i - 1], _lookPts[i]);

            // Mirror the most recent contacts for the gizmo pass.
            int srcCount = math.min(hits.Length, _hits.Length);
            for (int i = 0; i < srcCount; i++) _hits[i] = hits[i];
            _hitCount = srcCount;
            return result;
        }

        /// <summary>Up reference: -World.GravityVector (normalized). Never the
        /// hard-coded world up.</summary>
        private float3 GravityUp()
        {
            return math.normalizesafe(-GravityVector(), new float3(0f, 1f, 0f));
        }

        /// <summary>World gravity (acceleration, downward) for ballistic
        /// lookahead slices.</summary>
        private float3 GravityVector()
        {
            var w = World;
            return w.IsValid ? (float3)Box3DWorld.Instance.GravityVector
                             : new float3(0f, -9.81f, 0f);
        }

        /// <summary>Predicted up: frames without NEW surface evidence (standing
        /// still, airborne) HOLD the current support - resting under-foot
        /// contacts when present, else the last smoothed tilt. No gravity
        /// erosion: the old lerp(gravityUp, ...) fixed point was gravityUp for
        /// any blend < 1, which slowly straightened the posture on slopes.
        /// upPredictionBlend is the authority of the trajectory's nearest new
        /// support over that hold.</summary>
        private float3 PredictedUp(float3 baselineAvg, float3 bestN)
        {
            if (!math.any(bestN))
                return math.normalizesafe(baselineAvg, _smoothUp);

            return math.normalizesafe(
                math.lerp(baselineAvg, bestN, math.saturate(upPredictionBlend)), _smoothUp);
        }

        /// <summary>Nearest-in-approach support selection: among a collide
        /// pass's contacts, keep the up-facing ones (not walls) that differ
        /// from the current support (baseline duplicates), and track the one
        /// closest along the approach direction. Distance is the contact point
        /// projected on the trajectory's start direction, so earlier surfaces
        /// along the path win over later ones.</summary>
        private static void TrackNearestAhead(Span<MoverHit> hits, int count,
            float3 origin, float3 dir, float3 up, float upMinGroundDot,
            float3 measuredAvg, bool dupFilterActive, ref float3 bestN, ref float bestD)
        {
            int n = math.min(count, hits.Length);
            for (int i = 0; i < n; i++)
            {
                float3 nn = hits[i].Normal;
                if (math.dot(nn, up) < upMinGroundDot) continue;   // wall
                // "Duplicate" = repeats the MEASURED support - only while
                // measured contacts exist (curN > 0). Filtering against the
                // smoothed up instead would go circular: as the target
                // approaches a surface within the band the surface stops
                // counting as evidence and hold freezes the residual tilt.
                // Floating (no contacts) - every contact is new evidence.
                if (dupFilterActive && math.dot(nn, measuredAvg) > 0.995f) continue;
                float d = math.dot(hits[i].Point - origin, dir);
                if (d < bestD) { bestD = d; bestN = nn; }
            }
        }

        /// <summary>Low-passes the transform's up toward the predicted up,
        /// preserving forward (yaw); the collision capsule stays world-vertical.</summary>
        private void ApplyUpBlend(float dt)
        {
            float k = 1f - math.exp(-math.max(0f, upBlendRate) * dt);
            _smoothUp = math.normalizesafe(
                math.lerp(_smoothUp, _targetUp, k), GravityUp());

            float3 fwd = (float3)transform.forward;
            float3 f = fwd - _smoothUp * math.dot(fwd, _smoothUp);
            if (math.lengthsq(f) < 1e-6f)
            {
                // Forward parallel to up (extreme tilt): pick a perpendicular.
                f = math.cross(_smoothUp, new float3(1f, 0f, 0f));
                if (math.lengthsq(f) < 1e-6f) f = math.cross(_smoothUp, new float3(0f, 0f, 1f));
            }
            transform.rotation = quaternion.LookRotation(
                math.normalizesafe(f), _smoothUp);
        }

        /// <summary>Query capsule along the given up axis (world-space center
        /// offsets from the query origin, native mover convention). The mover
        /// represents the CHARACTER's footprint, not a vertical stick: with up
        /// = SmoothUp the support collide, ground probe and the trajectory all
        /// see the leaned shape. The mover API honors arbitrary segment
        /// orientation (proven by MoverLookaheadTests).
        /// MoverLookaheadTests.TiltedMoverCapsule_ContactsWhereVerticalCannot).</summary>
        private Capsule CapsuleFor(float3 up)
        {
            float halfSpan = math.max(0f, height - 2f * radius) * 0.5f;
            up = math.normalizesafe(up, new float3(0f, 1f, 0f));
            return new Capsule
            {
                Center1 = -up * halfSpan,
                Center2 = up * halfSpan,
                Radius = math.max(1e-3f, radius),
            };
        }

        private QueryFilter MoverFilter()
        {
            // Project convention (cf. FootStabilizer / Box3DBody.Awake): Unity
            // layer == category bit. Category claims our own layer (so shapes
            // masked to this layer accept the query); the mask is the exposed
            // LayerMask. A LayerMask value IS the 1<<layer bitmask, so it maps
            // straight onto MaskBits - untick the presence-capsule's layer to
            // keep the queries self-excluding.
            int layer = gameObject.layer;
            return new QueryFilter
            {
                CategoryBits = layer >= 0 ? 1UL << layer : ulong.MaxValue,
                MaskBits = (ulong)collisionMask.value,
            };
        }

        /// <summary>Grounded: sweep a hair below the sole; any blocking contact
        /// within the band counts (also used live by the IsGrounded property).
        /// Plane normals from MoveCapsule's collide pass are not reused - the
        /// property must answer outside moves too.</summary>
        private bool ProbeGround(float3 origin)
        {
            // Sweep along the CHARACTER's down, not world down: with the
            // tilted query capsule this measures true perpendicular clearance
            // to the surface underfoot (on a slope, a vertical probe shortens
            // by cos(tilt) and reads thin).
            float3 upQ = math.normalizesafe(_smoothUp, new float3(0f, 1f, 0f));
            Capsule cap = CapsuleFor(upQ);
            float probe = skinWidth + groundExtra;
            float frac = World.CastMover(origin, cap,
                -upQ * probe, MoverFilter());
            return frac < 1f;
        }

        // --- gizmos -----------------------------------------------------------

        /// <summary>Popcron GL overlay: the (tilted) mover capsule, the
        /// contacts of the last collide pass (point + normal) and the ground
        /// probe sweep (green = grounded, red = airborne).</summary>
        private void Update()
        {
            if (!drawGizmos) return;

            float3 origin = CenterWorld();
            float halfSpan = math.max(0f, height - 2f * radius) * 0.5f;
            float3 upQ = math.normalizesafe(_smoothUp, new float3(0f, 1f, 0f));
            float3 a = origin - upQ * halfSpan;
            float3 b = origin + upQ * halfSpan;
            PopcronGizmos.Sphere(a, radius, Color.cyan);
            PopcronGizmos.Sphere(b, radius, Color.cyan);
            PopcronGizmos.Line(a, b, Color.cyan);

            if (!Application.isPlaying) return;

            for (int i = 0; i < _hitCount; i++)
            {
                PopcronGizmos.Sphere(_hits[i].Point, 0.03f, new Color(1f, 0.5f, 0f));
                PopcronGizmos.Line(_hits[i].Point,
                    _hits[i].Point + _hits[i].Normal * 0.25f, Color.yellow);
            }

            float probe = height * 0.5f + skinWidth + groundExtra;
            bool grounded = ProbeGround(origin);
            PopcronGizmos.Line(origin, origin - upQ * probe,
                grounded ? Color.green : Color.red);
            PopcronGizmos.Line(origin, origin + _smoothUp * (height * 0.5f), Color.magenta);

            // Predicted trajectory polyline (lookahead waypoints), drawn at
            // ground-contact level: waypoints are capsule-center poses, so
            // shift by the offset to the current under-foot contact. Only
            // SAMPLED poses are drawn - the path never speculates past its
            // last verified free sweep.
            for (int i = 1; i < _lookCount; i++)
                PopcronGizmos.Line(_lookPts[i - 1] + _lookGroundOffset,
                    _lookPts[i] + _lookGroundOffset, Color.yellow);
        }
    }
}
