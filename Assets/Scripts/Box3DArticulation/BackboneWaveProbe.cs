using Unity.Mathematics;
using UnityEngine;
using Box3D.Hybrid;
using Box3D;
using PopcronGizmos = Popcron.Gizmos;

namespace Box3DArticulation
{
    /// <summary>Static (policy-invisible) parameters, drawn per limb per
    /// instance by the batch spawner. The policy never sees or commands
    /// any of these - each limb lives with its draw for the instance's
    /// lifetime, and reward differences between instances carrying
    /// different genomes are the selection signal for a later PBT loop.
    /// Survives pooled episode resets (fields on the persistent probe);
    /// re-applied by the agent's ResolveLimbs after a full rebuild.</summary>
    public readonly struct WaveGenome
    {
        public readonly float wavenumber, twistAmp, twistPhase,
            ffGateAngle, endpointGain, endpointMaxRate,
            kp, kd, zeta, lookahead, wCeiling;

        public WaveGenome(float wavenumber, float twistAmp, float twistPhase,
            float ffGateAngle, float endpointGain, float endpointMaxRate,
            float kp, float kd, float zeta, float lookahead, float wCeiling)
        {
            this.wavenumber = wavenumber;
            this.twistAmp = twistAmp;
            this.twistPhase = twistPhase;
            this.ffGateAngle = ffGateAngle;
            this.endpointGain = endpointGain;
            this.endpointMaxRate = endpointMaxRate;
            this.kp = kp;
            this.kd = kd;
            this.zeta = zeta;
            this.lookahead = lookahead;
            this.wCeiling = wCeiling;
        }
    }

    /// <summary>
    /// Backbone-wave probe: the chain is driven by a traveling curvature wave
    /// along the backbone, not by end-effector targets or per-joint keys.
    ///
    ///   kappa(s,t) = A(s) * sin(2*pi*f*t - k*s + phi)     bend wave (plane X-Y)
    ///   A(s)       = lerp(ampBase, ampTip, s)             amplitude envelope
    ///   s          = normalized arc length along the chain [0..1]
    ///
    /// Per FixedUpdate the sampled curvature integrates to per-segment bend
    /// angles; backbone directions are reconstructed by rotating the base
    /// direction about the (constant) bend axis — a Bishop frame degenerate to
    /// planar waves, which is what the elbow hinge (Z) constrains us to anyway.
    /// The existing inertia-aware servo tracks the sampled poses. Zero-g: the
    /// wave is the only motion source, so faithfulness is measurable.
    ///
    /// Later swaps, same interface: sine source -> CPG oscillator chain;
    /// kinematic sampler -> discrete elastic rod; hand-set params -> baked
    /// params (FFT along t, least-squares along s) from recorded animation.
    /// </summary>
    public class BackboneWaveProbe : MonoBehaviour
    {
        [Header("Bones")]
        [Header("Bones (human scale: Winter anthropometry, 70 kg adult)")]
        [SerializeField] private float boneLength = 0.33f;   // upper arm segment
        [SerializeField] private float boneThickness = 0.08f;
        [SerializeField, Tooltip("Attach a FootStabilizer to the distal body (legs).")]
        private bool footContact = false;

        /// <summary>Attach a passive foot-contact stabilizer to the distal body on build.
        /// Must be set before Start (the rig builder sets it right after AddComponent).</summary>
        public bool FootContact { get => footContact; set => footContact = value; }
        [SerializeField] private float rootHeight = 1.5f;    // shoulder height
        [Tooltip("Optional: attach the limb to an EXISTING body (e.g. a leg on a torso) instead of spawning a static root cube. Spawn pose and evaluation anchor follow that body; the shoulder joint connects at RootAnchorLocal (body-local).")]
        [SerializeField] private Box3DBody externalRoot;
        [Tooltip("Shoulder anchor in the external root's body-local frame (bottom face of the torso = -0.5*h). Ignored without externalRoot.")]
        [SerializeField] private float3 rootAnchorLocal = new float3(0f, -0.05f, 0f);
        [SerializeField] private float upperMassKg = 1.9f;
        [SerializeField] private float foreMassKg = 1.6f;    // forearm + hand

        [Header("Wave: kappa(s,t) = A(s)*sin(2pi f t - k s + phi)")]
        [SerializeField] private float freqHz = 1f;
        [SerializeField] private float wavenumber = 6.283185f; // rad per unit s (2pi = one wavelength)
        [SerializeField] private float phase = 0f;
        [SerializeField] private float bias = 0.8f;   // mean curvature: oscillate around a flexed pose (rad)
        [SerializeField] private float ampBase = 0.0f;  // envelope at s=0 (rad)
        [SerializeField] private float ampTip = 0.8f;   // envelope at s=1 (rad)
        [SerializeField] private Vector3 baseDirection = Vector3.down; // backbone direction at s=0
        [SerializeField] private Vector3 bendAxis = Vector3.forward;   // wave plane normal at s=0 (elbow hinge Z)
        [Header("Twist wave: bend-axis rotation along the chain (general 3D curve)")]
        [SerializeField, Range(0f, 3f)] private float twistAmp = 0.6f;      // rad
        [SerializeField] private float twistWavenumber = 6.283f;            // rad per unit s
        [SerializeField, Range(-3.14f, 3.14f)] private float twistPhase = 0f;
        [Header("Roll: constant rotation of the WHOLE limb frame about the base direction axis")]
        [SerializeField, Range(-3.14f, 3.14f), Tooltip("Rolls the entire bend envelope around the base axis (base dir is invariant, so the root stays put - only the bend plane orientation changes). Mirrored per side by the caller.")]
        private float roll = 0f;
        [Header("Pose roll: rotate the FINAL projected pose about the root->tip chord")]
        [SerializeField, Range(-3.14f, 3.14f), Tooltip("Post-transform on the finished wave pose: root and tip positions are preserved, the intermediate bend (elbow/knee direction) swings around the root->tip chord. Lateral pose without re-aiming any wave axis and without touching the twist envelopes. Rad. 0 = legacy pose (bitwise).")]
        private float poseRoll = 0f;

        [Header("[DEPRECATED] Target event: steer the tip through a target, then blend back")]
        [SerializeField] private Transform targetPoint;

        /// <summary>Public read for external adapters (DmpProbeStrike falls back
        /// to this when its own target is unset).</summary>
        public Transform TargetPoint => targetPoint;
        [SerializeField] private KeyCode steerKey = KeyCode.Space;

        /// <summary>Public read for external adapters (DmpProbeStrike reuses the key).</summary>
        public KeyCode SteerKey => steerKey;
        [SerializeField, Range(0.3f, 3f)] private float arrivalTime = 1f;   // pass-through moment
        [SerializeField, Range(0.05f, 1f)] private float blendInTime = 0.3f;
        [SerializeField, Range(0.1f, 2f)] private float blendOutTime = 0.5f;
        
        [Tooltip("0 = great-circle crest arc; larger = wider hooked swing (side of swing = sign)")]
        [SerializeField, Range(-90f, 90f)] private float arcTwistDeg = 30f;
        [Tooltip("Crest approach speed: <1 slows the wave (crest arrives gently), >1 accelerates")]
        [SerializeField, Range(0.3f, 3f)] private float crestSpeed = 1f;
        [Tooltip("While Space is held (aiming), the live wave eases to this fraction of its frequency")]
        [SerializeField, Range(0.05f, 1f)] private float holdFreqScale = 0.35f;
        [Tooltip("Ramp time for the hold slowdown in/out (seconds) - integrated, so no phase jump")]
        [SerializeField, Range(0.05f, 1f)] private float holdRampTime = 0.25f;

        [Header("Drive (human peak voluntary torque; extreme athletes +30%)")]
        [SerializeField] private float shoulderTorqueLimit = 150f;  // N*m (flexion/extension)
        [SerializeField] private float shoulderTwistTorqueLimit = 25f; // N*m, long-axis (muscle-realistic)
        [Tooltip("Twist-feasibility clamp: the demanded shoulder rotation about the bone's long axis is capped at this angle. The bridge/IK solve is position-only - its aims carry an under-determined twist component that can exceed what the weak twist motor (50x less inertia) can deliver; saturating it starves the bend axes and the whole articulation drifts. 0 = no clamp.")]
        [SerializeField, Range(0f, 120f)] private float shoulderTwistLimitDeg = 65f;
        [SerializeField] private float elbowTorqueLimit = 120f;     // N*m
        [Tooltip("Absolute |w| command ceiling — bounds torque kicks on tiny-inertia axes")]
        [SerializeField, Range(5f, 600f)] private float wCeiling = 90f;
        [SerializeField] private bool driveEnabled = true;
        [SerializeField] private bool torqueFeedforward = true;
        [Tooltip("Joint-frame error beyond which feedforward is gated out (rad) - prevents ff pumping when desynced")]
        [SerializeField, Range(0.05f, 15f)] private float ffGateAngle = 0.2f;
        [Tooltip("Horizon shaping: planned-deceleration clamp into wave crests")]
        [SerializeField] private bool crestClamp = false;
        [Tooltip("Closed-loop endpoint correction: measured hand error distributed over both bends (Jacobian least-squares)")]
        [SerializeField] private bool endpointCorrection = true;
        [SerializeField, Range(0.1f, 50f)] private float endpointGain = 0.2f;   // 1/s rate gain
        [SerializeField, Range(0.1f, 50f)] private float endpointMaxRate = 5f;  // rad/s clamp on correction
        [SerializeField] private bool torqueTelemetry = false;                 // log motor torque vs limit
        [SerializeField, Range(0.1f, 2f)] private float kp = 1f;
        [SerializeField, Range(0f, 1f)] private float kd = 0.05f;
        [SerializeField, Range(0.5f, 2f)] private float zeta = 1f;

        [Header("Joint springs (stability only)")]
        // Soft-constraint stability: hertz must be well under HALF the sim
        // frequency (Box2D-family rule). 200 Hz at 50 Hz sim = hertz*dt = 4,
        // far past the stability region -> the constraint overshoots each step
        // and periodically separates under wave loading. 25 Hz = critically
        // damped, stable, and still effectively rigid at wave timescales.
        [SerializeField, Range(5f, 200f)] private float shoulderHertz = 25f;
        [SerializeField] private float shoulderDamping = 0.5f;
        [SerializeField, Range(5f, 200f)] private float elbowHertz = 25f;
        [SerializeField] private float elbowDamping = 0.2f;
        [SerializeField] private float elbowLimitDeg = 150f;  // anatomical hinge limit
        [SerializeField] private float elbowMinDeg = -10f;
        [Tooltip("Wave sampled at t + lookahead to cancel pipeline delay (1 step = full step of the discrete loop)")]
        [SerializeField] private float lookahead = 0.02f;

        /// <summary>Writes the genome's static parameters. Safe any time:
        /// every value here is re-read from the serialized fields each
        /// FixedUpdate (the live single-source rule).</summary>
        public void ApplyGenome(in WaveGenome g)
        {
            wavenumber = g.wavenumber;
            twistAmp = g.twistAmp;
            twistPhase = g.twistPhase;
            ffGateAngle = g.ffGateAngle;
            endpointGain = g.endpointGain;
            endpointMaxRate = g.endpointMaxRate;
            kp = g.kp;
            kd = g.kd;
            zeta = g.zeta;
            lookahead = g.lookahead;
            wCeiling = g.wCeiling;
        }

        /// <summary>External knob override (DMP strike schedule): given the
        /// predicted command phase, returns the wave frame (aim) and bias to
        /// command instead of the idle/event values. Null return = no override
        /// this tick. Once per FixedUpdate, not per joint - cost is nil.
        /// Set by DmpProbeStrike (Assembly-CSharp); cleared when spent.</summary>
        public System.Func<float, (quaternion aim, float bias)?> DmpKnobOverride;
        /// <summary>External frequency override (Hz, signed). NaN = the
        /// inspector parameter rules. The phase is the integral of frequency,
        /// so even a step change is jump-free - reversing the sign simply
        /// stops the traveling wave and sends it back the other way.</summary>
        [HideInInspector] public float waveFreqOverride = float.NaN;

        /// <summary>Set by DmpProbeStrike: called with the target position when
        /// the steer key is RELEASED, instead of committing the crest-strike
        /// event. The crest machinery stays for aiming preview only.</summary>
        public System.Action<float3> DmpStrikeTrigger;

        /// <summary>True while a DMP strike schedule is driving the knobs -
        /// the key handler ignores input so the crest event can't fire mid-strike.</summary>
        public bool DmpStrikeActive;

        /// <summary>Knob trajectory derivative feedforward (set by DmpProbeStrike
        /// during its knob override): bias rate [rad/s] and aim-twist rate
        /// [rad/s about the bone]. The idle-wave analytic rates know nothing
        /// about the per-tick knob solve - without these the servo predicts a
        /// standing-still command during the whole strike.</summary>
        public float DmpKnobBiasRate;
        public float DmpKnobTwistRate;

        /// <summary>External TIP override (DMP strike schedule): given the
        /// predicted command phase, returns the exact want-point for the hand.
        /// When set, the wave's bend computation is REPLACED by closed-form
        /// 2-link IK onto this point (the wave remains only as the underlying
        /// frame) - the hand is commanded along the baked trajectory directly.</summary>
        public System.Func<float, float3?> DmpTipOverride;

        [SerializeField, Tooltip("Draw the idle-wave tip prediction gizmo (and compute it). Keep OFF in batch - it is per-probe-per-step cost.")]
        public bool drawTipPrediction = false;

        /// <summary>World-space tip (end of the distal bone) for external planners.</summary>
        public float3 TipWorld => m_fore != null
            ? BodyPoint(m_fore.Body, new float3(0f, -boneLength / 2f, 0f))
            : float3.zero;

        /// <summary>World-space mid-chain joint (elbow/knee): the upper segment's
        /// distal end. Read together with TipWorld by the pose-imitation reward.</summary>
        public float3 JointWorld => m_upper != null && m_upper.Body.IsValid
            ? BodyPoint(m_upper.Body, new float3(0f, -boneLength / 2f, 0f))
            : float3.zero;

        /// <summary>False while the rig is mid-rebuild: pose reads (JointWorld /
        /// TipWorld) are garbage, consumers must skip the step.</summary>
        public bool ChainValid => m_upper != null && m_fore != null
                               && m_upper.Body.IsValid && m_fore.Body.IsValid;

        /// <summary>Worst joint-anchor separation this step (m) across the
        /// shoulder and elbow. 0 = seated; growth = solver-driven dislocation.</summary>
        public float SepMax => m_sepMax;

        /// <summary>"LeftLeg (Biped7)" style tag for one-line diagnostics.</summary>
        private string ProbeTag()
        {
            var owner = transform.parent != null ? transform.parent.name : null;
            return owner != null && owner != name ? $"{name} ({owner})" : name;
        }
        private float m_sepMax;

        /// <summary>How contorted the limb is right now: angle (rad) between
        /// the actual chain direction (fore - upper) and the rest direction
        /// (BaseDirW). 0 = perfectly on-plane; grows with fold. Consumers use
        /// it as a per-second posture penalty. 0 if bodies aren't live.</summary>
        public float ExtremityRad()
        {
            if (m_upper == null || m_fore == null || !m_upper.Body.IsValid || !m_fore.Body.IsValid) return 0f;
            float3 dir = math.normalizesafe((float3)m_fore.Body.GetPosition() - (float3)m_upper.Body.GetPosition());
            if (math.lengthsq(dir) < 1e-8f) return 0f;
            float c = math.clamp(math.dot(dir, BaseDirW), -1f, 1f);
            return math.acos(c);
        }

        private class JointRef
        {
            public JointId id;
            public bool isHinge;
            public B3Transform frameA, frameB;
            public Body[] distal;
            public Box3DBody bodyA, bodyB; // joint endpoints - effort metric (motor power)
            public float torqueLimit;
            public float lastErrMag; // previous-step joint-frame error (ff gating)
        }

        private Box3DBody m_root, m_upper, m_fore;
        /// Distal (foot-side) body - the gait reward measures foot excursion relative to the torso COM.
        public Box3DBody DistalBody => m_fore;

        /// Summed joint motor power |tau| * |omega_rel| (W) over this limb's two
        /// servo joints - the effort metric for the reward layer. Static holds
        /// (omega ~ 0) are free by construction, so the gravity baseline of
        /// standing costs nothing; only joint MOTION under torque is taxed.
        public float JointPower()
        {
            return JointPower(m_shoulder) + JointPower(m_elbow);
        }

        private static float JointPower(JointRef j)
        {
            if (j == null || j.bodyA == null || j.bodyB == null
                || !j.bodyA.Body.IsValid || !j.bodyB.Body.IsValid) return 0f;
            float tau = j.isHinge
                ? math.abs(new RevoluteJoint { Id = j.id }.GetMotorTorque())
                : math.length(new SphericalJoint { Id = j.id }.GetMotorTorque());
            float3 wRel = j.bodyB.Body.GetAngularVelocity() - j.bodyA.Body.GetAngularVelocity();
            return tau * math.length(wRel);
        }
        private float3 m_anchorLocal; // shoulder anchor in the root body's local frame
        private JointRef m_shoulder, m_elbow;
        private float3 m_halfExtents;
        private float3[] m_targetDirs;   // sampled backbone directions per bone
        private float3[] m_targetPoints; // sampled backbone points (for gizmos)
        private float[] m_bendRates;     // analytic theta-dot per segment (feedforward)
        private float[] m_bendAccel;     // analytic theta-double-dot per segment (torque ff)
        private float[] m_crestDist;     // planned angle to next crest per segment (horizon clamp)
        [Tooltip("Time constant (s) of the evaluation-anchor follower on the root body's live pose. Small = tight tracking, jittery through drags; large = smooth but laggy. Continuous base motion is unaffected - this only shapes reaction to discontinuities.")] [Min(0.01f)]
        public float anchorLagTau = 0.12f;
        private float3 m_anchorP0;   // smoothed evaluation anchor (P0 + frame)
        private quaternion m_anchorRot;
        private bool m_anchorInit;
        private float3 m_framePos;    // root object pose captured at spawn
        private quaternion m_frameRot;
        private float3[] m_bendAxisW;    // transported bend axis per segment (Bishop frame)
        private float[] m_twistRates;    // analytic psi-dot per segment (twist ff)
        private float3 m_hingeAxisInUpper; // elbow hinge axis in upper-arm local space
        private float m_lastTelemetry;
        private float3[] m_predPath;      // predicted tip trajectory (gizmo)
        private float[] m_predPhase;      // sin(tip-segment wave phase) per sample
        private bool m_predValid;
        private float3[] m_predPathPv;    // hold-preview plan path (dashed)
        private float[] m_predPhasePv;
        private bool m_pvPathValid;
        // Hold-to-preview plan: the crest-steer solution computed while the
        // key is DOWN; firing the key UP promotes it to the live event.
        private bool m_pvActive;
        private float m_holdEnv;          // 0=normal speed, 1=hold slowdown
        private WaveParams m_pvBase, m_pvSteered;
        private quaternion m_qPv = quaternion.identity;
        private float m_pvStart;

        // ---- wave sequencer: parameters are TRAJECTORIES, phase is INTEGRATED ----
        // The wave phase is never evaluated as omega*t: it accumulates
        // d(phi)/dt = 2*pi*freq(t). Frequency (and every other parameter) can
        // then blend mid-flight with ZERO phase discontinuity, because the
        // phase is the integral of the parameter history.
        private struct WaveParams
        {
            public float freqHz, wavenumber, phase, ampBase, ampTip, bias;
            public float twistAmp, twistWavenumber, twistPhase;
            public float poseRoll;
        }

        private enum EvState { None, BlendIn, Hold, BlendOut }
        private EvState m_ev;
        private float m_evStart;
        private WaveParams m_evBase, m_evSteered;
        private float m_phiAccum;   // integrated wave phase (rad)

        // Pooled-reset spawn capture (ResetToSpawn), filled by SpawnCore.
        private Vector3 m_spawnUpperPos, m_spawnForePos, m_spawnRootPos;
        private Quaternion m_spawnUpperRot, m_spawnForeRot;
        private bool m_rootStatic;
        // Persistent spatial phase offset on the idle figure (rad). Lets an
        // external planner rebase the idle so it continues from where a strike
        // left the tip, WITHOUT the planner having to hold the override
        // forever. Ramped at <= one wave cycle per second: the whole figure
        // slides continuously, no reference jump.
        private float m_phaseOffset;
        private float m_phaseOffsetTarget;

        /// Rebase the idle wave: the figure shifts by dPhi (ramped). The knob
        /// override receives the OFFSET command phase, so planner-side phase
        /// math stays consistent automatically.
        public void SetWavePhaseOffset(float dPhi) => m_phaseOffsetTarget = dPhi;
        /// True once the offset ramp has settled onto its target.
        public bool PhaseOffsetSettled =>
            math.abs(m_phaseOffsetTarget - m_phaseOffset) < 1e-3f;

        // --- runtime control surface (agent / manual tester) ---
        // The idle wave is re-derived from the serialized fields every FixedUpdate
        // (SerializedParams -> EvalEvent), so writing them is live and single-source:
        // no parallel override fields. Freq/amp/bias/wavenumber take effect on the
        // next step (phase is integrated, so even step changes are jump-free);
        // bendAxis re-derives the world bend plane immediately (BendAxisW reads
        // the field per step). phaseOffset stays absolute-target via the ramp.

        /// Integrated wave phase including the live offset (rad). Unbounded -
        /// feed sin/cos of this to observers, never the raw value.
        public float Phase => m_phiAccum + m_phaseOffset;
        /// Current live phase offset (rad) - what SetWavePhaseOffset is ramped to.
        public float PhaseOffset => m_phaseOffset;

        /// <summary>Pooled episode reset: teleport both bones back to their
        /// spawn poses, zero velocities, clear per-episode state. Same native
        /// bodies/joints/components stay alive - none of the GameObject/native
        /// body/joint lifecycle cost of a rebuild. Teleports are separate
        /// native calls; the joint solver pulls any residual mismatch shut on
        /// the next step (identical to spawn). Returns false when the rig
        /// cannot be trusted (never spawned, bodies invalid): the caller must
        /// fall back to a full rebuild.</summary>
        public bool ResetToSpawn()
        {
            if (!m_spawned) return false;
            if (m_upper == null || m_fore == null ||
                !m_upper.Body.IsValid || !m_fore.Body.IsValid) return false;
            m_upper.Position = m_spawnUpperPos;
            m_upper.Rotation = m_spawnUpperRot;
            m_fore.Position = m_spawnForePos;
            m_fore.Rotation = m_spawnForeRot;
            var ub = m_upper.Body; ub.LinearVelocity = 0f; ub.AngularVelocity = 0f;
            var fb = m_fore.Body;  fb.LinearVelocity = 0f; fb.AngularVelocity = 0f;
            if (m_rootStatic && m_root != null && m_root.Body.IsValid)
            {
                m_root.Position = m_spawnRootPos;
                var rb = m_root.Body; rb.LinearVelocity = 0f; rb.AngularVelocity = 0f;
            }
            // Per-episode state, equivalent to a fresh Start():
            m_phiAccum = 0f;                    // wave phase restarts from zero
            m_phaseOffset = 0f; m_phaseOffsetTarget = 0f;
            m_ev = EvState.None; m_evStart = 0f;
            m_pvActive = false; m_pvStart = 0f;
            m_tiltX = 0f; SetTilt(0f);          // authored base direction restored
            m_dislocLogged = false; m_stepsSinceSpawn = 0; m_sepMax = 0f;
            m_anchorInit = false;               // anchor follower re-snaps to the teleported root
            return true;
        }

        /// Writes the shared wave parameters (idle/event BASE values). Event and
        /// knob overrides still shadow them while active, as designed.
        public void SetRuntimeParams(float freqHz_, float ampTip_, float bias_, float wavenumber_,
            float twistAmp_ = float.NaN, float twistPhase_ = float.NaN, float ampBase_ = float.NaN,
            float roll_ = float.NaN, float poseRoll_ = float.NaN)
        {
            freqHz = freqHz_;
            ampTip = ampTip_;
            bias = bias_;
            wavenumber = wavenumber_;
            if (!float.IsNaN(twistAmp_)) twistAmp = twistAmp_;
            if (!float.IsNaN(twistPhase_)) twistPhase = twistPhase_;
            if (!float.IsNaN(ampBase_)) ampBase = ampBase_;
            if (!float.IsNaN(roll_)) roll = roll_;
            if (!float.IsNaN(poseRoll_)) poseRoll = poseRoll_;
        }

        /// Current shared parameters (the values the idle wave will use).
        public (float freqHz, float ampTip, float bias, float wavenumber, float twistAmp, float twistPhase, float ampBase)
            GetRuntimeParams() => (freqHz, ampTip, bias, wavenumber, twistAmp, twistPhase, ampBase);

        /// Per-limb lean of the base direction (rad): a signed ROTATION of the
        /// spawned base direction about the root frame's Z axis. The demo sets
        /// base = (0,-1,0) (legs) or (+-1,0,0) (arms); 0 writes back the
        /// authored direction exactly, so any base direction keeps its full
        /// range. Sign convention: the caller applies the side sign - positive
        /// tilt rotates the base toward local +x on both sides (down limbs
        /// lean sideways; horizontal limbs elevate).
        public float BaseDirXTilt
        {
            get => m_tiltX;
            set { if (m_spawned) SetTilt(value); }
        }

        private float m_tiltX; // stored tilt angle: single source for the getter

        /// Canonical base write. Base = Rot(Z, x) * authored base, clamped so
        /// the base never approaches the bend axis (0.98 parallel refusal).
        private void SetTilt(float xIn)
        {
            float maxX = math.abs(bendAxis.z) > 0.7f ? 1.53f : 1.2f;
            m_tiltX = math.clamp(xIn, -maxX, maxX);
            float2 b0 = new float2(m_baseDir0.x, m_baseDir0.y);
            float l0 = math.length(b0);
            float3 b03 = l0 > 1e-6f
                ? new float3(b0.x / l0, b0.y / l0, m_baseDir0.z)
                : new float3(0f, -1f, 0f);
            float3 v = math.mul(quaternion.RotateZ(m_tiltX), b03);
            baseDirection = new Vector3(v.x, v.y, v.z);
        }
        private Vector3 m_baseDir0 = Vector3.down; // authored base, captured at spawn
        private bool m_spawned; // false until SpawnCore: gates stateful pushes (BaseDirXTilt)
        private quaternion m_qArc = quaternion.identity; // crest-arc frame rotation

        private WaveParams SerializedParams() => new WaveParams
        {
            freqHz = freqHz, wavenumber = wavenumber, phase = phase,
            ampBase = ampBase, ampTip = ampTip, bias = bias,
            twistAmp = twistAmp, twistWavenumber = twistWavenumber, twistPhase = twistPhase,
            poseRoll = poseRoll,
        };

        /// <summary>Wave configuration exposed for tooling (authoring/preview):
        /// everything a kinematic preview needs to realize this rig's wave.</summary>
        // Wave frame in WORLD space: the serialized directions are authored
        // in the root's LOCAL frame. Rotation source = the SMOOTHED anchor
        // pose (follower on the root body's live pose) - moving or rotating
        // the spawned root bone re-anchors evaluation continuously; a tilt
        // becomes a short ramp, not an instant target jump.
        quaternion RootRot => m_anchorInit ? m_anchorRot : m_frameRot;
        float3 BaseDirW => math.normalizesafe(math.mul(RootRot, (float3)baseDirection));
        // ROLL: whole-limb frame rotation about the base axis. Applied at the
        // frame SOURCE (BendAxisW) so the eval loop, WaveSetup export, tracking
        // and gizmos all see the identical rolled frame. Rotation about the
        // base axis leaves baseDirection fixed, so the parallel-refusal check
        // is unaffected.
        float3 BendAxisW
        {
            get
            {
                float3 ax = math.normalizesafe(math.mul(RootRot, (float3)bendAxis));
                return math.normalizesafe(math.mul(quaternion.AxisAngle(BaseDirW, roll), ax));
            }
        }

        public struct WaveSetup
        {
            public float freqHz, wavenumber, phase, ampBase, ampTip, bias;
            public float twistAmp, twistWavenumber, twistPhase;
            public float3 baseDirection, bendAxis;
            public float boneLength, elbowLimitDeg;
            public float3 shoulderPosition;
        }

        public WaveSetup GetWaveSetup() => new WaveSetup
        {
            freqHz = freqHz, wavenumber = wavenumber, phase = phase,
            ampBase = ampBase, ampTip = ampTip, bias = bias,
            twistAmp = twistAmp, twistWavenumber = twistWavenumber, twistPhase = twistPhase,
            baseDirection = BaseDirW, bendAxis = BendAxisW,
            boneLength = boneLength, elbowLimitDeg = elbowLimitDeg,
            // m_targetPoints is populated by the probe's runtime init; in edit
            // mode fall back to the transform position.
            shoulderPosition = m_targetPoints != null && m_targetPoints.Length > 0
                ? m_targetPoints[0] : (float3)transform.position,
        };

        private static WaveParams Blend(WaveParams a, WaveParams b, float u)
        {
            u = math.saturate(u);
            WaveParams r;
            r.freqHz = math.lerp(a.freqHz, b.freqHz, u);
            r.wavenumber = math.lerp(a.wavenumber, b.wavenumber, u);
            r.phase = math.lerp(a.phase, b.phase, u);
            r.ampBase = math.lerp(a.ampBase, b.ampBase, u);
            r.ampTip = math.lerp(a.ampTip, b.ampTip, u);
            r.bias = math.lerp(a.bias, b.bias, u);
            r.twistAmp = math.lerp(a.twistAmp, b.twistAmp, u);
            r.twistWavenumber = math.lerp(a.twistWavenumber, b.twistWavenumber, u);
            r.twistPhase = math.lerp(a.twistPhase, b.twistPhase, u);
            r.poseRoll = math.lerp(a.poseRoll, b.poseRoll, u);
            return r;
        }

        private static float3 kLocalBone = new float3(0f, -1f, 0f);

        private void Start()
        {
            try
            {
                SpawnCore();
                m_spawnHealthy = true;
            }
            catch (System.Exception e)
            {
                // A throw mid-Start leaves bones spawned but unjointed - and
                // Start never re-runs, so the limb stays broken the whole
                // episode. Never silent: this is the dislocated-spawn bug's
                // primary suspect path.
                m_spawnHealthy = false;
                Debug.LogError($"[Wave:{ProbeTag()}] spawn FAILED (limb unrecoverable this episode): {e}");
            }
        }

        /// <summary>False when the limb failed to build (exception, invalid
        /// root) or was born dislocated - consumers (agent, batch) must not
        /// punish the policy for a rig the policy did not break.</summary>
        public bool SpawnHealthy => m_spawnHealthy;
        private bool m_spawnHealthy;
        private bool m_dislocLogged;
        private int m_stepsSinceSpawn;

        private void SpawnCore()
        {
            // Capture the AUTHORED base direction BEFORE any runtime lean push:
            // BaseDirXTilt rotates relative to this (it must not inherit the
            // old down-based parameterization). Default stays down for the
            // failure path - a failed spawn leaves legs behaving as before.
            m_baseDir0 = baseDirection;
            m_tiltX = 0f; // authored pose: no residual tilt across resets
            m_spawned = true; // stateful setters (BaseDirXTilt) may write from here on

            // ROOT FRAME (captured ONCE at spawn): every spawn offset and
            // authored direction is interpreted in the root object's local
            // frame. The physical rig cannot follow a live-moving transform
            // (physics owns the bodies), so later edits to the root object's
            // transform are deliberately ignored - the rig is rooted at the
            // spawn pose and all evaluation stays relative to it.
            m_frameRot = transform.rotation;
            m_framePos = transform.position;

            // External root: take the host body's live pose as the root frame
            // (legs hang in the torso's frame, not world-down).
            bool ext = externalRoot != null && externalRoot.Body.IsValid;
            if (ext)
            {
                m_framePos = externalRoot.transform.position;
                m_frameRot = externalRoot.transform.rotation;
                // anchor = the given body-local point (p0Live below follows it)
                m_anchorLocal = rootAnchorLocal;
            }
            else
            {
                // ALWAYS loud: this branch silently mis-spawns the limb as a
                // free-hanging chain on a static cube while the torso floats
                // separately - visually identical to the dislocated-spawn bug.
                Debug.LogWarning($"[Wave:{ProbeTag()}] external root INVALID at Start " +
                    $"(externalRoot={externalRoot?.name ?? "null"}, bodyValid={externalRoot?.Body.IsValid.ToString() ?? "-"}) " +
                    "- falling back to static-root spawn; limb will NOT track the torso");
                m_anchorLocal = new float3(0f, -boneThickness * 0.5f, 0f);
            }

            float halfBone = boneLength / 2f;
            float halfThick = boneThickness / 2f;
            m_halfExtents = new float3(halfThick, halfBone, halfThick);

            float shoulderY = rootHeight - halfThick;

            Box3DBody root;
            if (ext)
            {
                root = externalRoot; // no static cube: the limb rides the host
            }
            else
            {
                root = MakeBone("Root", m_framePos + math.mul(m_frameRot,
                        new float3(0f, rootHeight, 0f)),
                    new Vector3(boneThickness, boneThickness, boneThickness), Box3DBodyType.Static, 1f);
            }
            // Spawn datum = the shoulder anchor for BOTH modes: static root
            // bottom face or the external body's anchor point. Bones hang
            // straight from it - no initial joint separation for the solver
            // to yank shut.
            float3 down0 = math.mul(m_frameRot, (float3)baseDirection.normalized);
            float3 shoulderWorld = BodyPoint(root.Body, m_anchorLocal);
            // Bone frame: root frame rotated so local Y (box long axis) points
            // along the spawn direction. Opposite vectors are degenerate for
            // FromToRotation - hanging down IS the root frame, keep it.
            float3 b = (float3)baseDirection.normalized;
            // Shortest-arc rotation kLocalBone (bone-local -Y = segment dir) -> b.
            // The DRIVE's datum is kLocalBone = (0,-1,0): aligning +Y instead
            // flips the bone frame 180 deg - the box looks identical but the
            // servo believes the segment points opposite and drives the limb
            // to the mirrored pose (arms up, prediction down).
            float3 seg = kLocalBone; // (0,-1,0)
            float3 arcAxis = math.cross(seg, b);
            float cosA = math.clamp(math.dot(seg, b), -1f, 1f);
            quaternion align = math.lengthsq(arcAxis) < 1e-10f
                ? (cosA > 0f ? quaternion.identity : quaternion.RotateZ(math.PI))
                : quaternion.AxisAngle(math.normalizesafe(arcAxis), math.acos(cosA));
            Quaternion boneRot = math.dot(b, seg) > -0.999f
                ? math.mul(m_frameRot, align)
                : m_frameRot;
            var upper = MakeBone("UpperArm", shoulderWorld + down0 * halfBone,
                new Vector3(boneThickness, boneLength, boneThickness), Box3DBodyType.Dynamic, upperMassKg, boneRot);
            var fore = MakeBone("Forearm", shoulderWorld + down0 * (boneLength + halfBone),
                new Vector3(boneThickness, boneLength, boneThickness), Box3DBodyType.Dynamic, foreMassKg, boneRot);

            // Zero-g: the wave is the only motion source.
            upper.Body.SetGravityScale(0f);
            fore.Body.SetGravityScale(0f);

            m_shoulder = CreateSpherical(root, upper, shoulderWorld, shoulderHertz, shoulderDamping);
            m_elbow = CreateHinge(upper, fore, BodyPoint(upper.Body, new float3(0f, -halfBone, 0f)),
                BendAxisW, elbowHertz, elbowDamping);
            m_root = root; m_upper = upper; m_fore = fore;
            // Spawn poses for the pooled episode reset (ResetToSpawn):
            // teleports are absolute, so capture ONCE here - later steps
            // overwrite the transforms via ApplyMoveEvent.
            m_spawnUpperPos = upper.transform.position; m_spawnUpperRot = upper.transform.rotation;
            m_spawnForePos = fore.transform.position;   m_spawnForeRot = fore.transform.rotation;
            m_rootStatic = root != externalRoot; // non-ext mode owns its own static root bone
            m_spawnRootPos = root.transform.position;
            m_shoulder.distal = new[] { upper.Body, fore.Body };
            m_elbow.distal = new[] { fore.Body };
            m_shoulder.torqueLimit = shoulderTorqueLimit;
            m_elbow.torqueLimit = elbowTorqueLimit;

            // Foot contact: passive stabilizer on the distal body (legs only - set via
            // FootContact by the rig builder). Sphere mover at the bone tip, analytic
            // support+friction from FootContactModel, applied pre-solve.
            if (footContact)
            {
                var foot = fore.gameObject.AddComponent<FootStabilizer>();
                foot.Configure(new float3(0f, -halfBone, 0f),
                               boneLength * 0.5f, boneThickness * 0.5f, boneThickness * 0.5f);
            }

            m_targetDirs = new float3[2];
            m_targetPoints = new float3[3];
            m_bendRates = new float[2];
            m_bendAccel = new float[2];
            m_crestDist = new float[2];
            m_bendAxisW = new float3[2];
            m_twistRates = new float[2];
            m_predPath = new float3[96];
            m_predPhase = new float[96];
            m_predPathPv = new float3[48];
            m_predPhasePv = new float[48];
            m_targetPoints[0] = shoulderWorld;
            m_hingeAxisInUpper = math.mul(math.inverse(upper.Body.GetRotation()),
                BendAxisW);
        }

        private void Update()
        {
            if (!driveEnabled || targetPoint == null || m_upper == null) return;
            // DMP strike running: the schedule owns the knobs, no crest input
            if (DmpStrikeActive)
            {
                m_pvActive = false;
                return;
            }
#if ENABLE_INPUT_SYSTEM
            var key = ToInputKey(steerKey);
            if (!key.HasValue || UnityEngine.InputSystem.Keyboard.current == null) return;
            var kc = UnityEngine.InputSystem.Keyboard.current[key.Value];
            bool down = kc.isPressed;
            bool released = kc.wasReleasedThisFrame;
#else
            bool down = Input.GetKey(steerKey);
            bool released = Input.GetKeyUp(steerKey);
#endif
            if (m_ev != EvState.None)
            {
                m_pvActive = false;
                return; // event running - ignore the key until it finishes
            }
            if (down)
            {
                // Recompute the plan every held frame: target may move, and
                // the crest phase alignment tracks the live accumulator.
                ComputeCrestPlan((float3)targetPoint.position,
                    out m_pvBase, out m_pvSteered, out m_qPv);
                m_pvStart = Time.time;
                m_pvActive = true;
            }
            else if (released && m_pvActive)
            {
                // DMP strike adapter present: hand the release over to it
                // (the crest event is the fallback, untouched)
                if (DmpStrikeTrigger != null)
                {
                    DmpStrikeTrigger((float3)targetPoint.position);
                    m_pvActive = false;
                    return;
                }
                // Promote the preview to the live event.
                m_evBase = m_pvBase;
                m_evSteered = m_pvSteered;
                m_qArc = m_qPv;
                m_evStart = Time.time;
                m_ev = EvState.BlendIn;
                // Kill the aim slowdown NOW: from this instant the accumulator
                // must advance exactly per PhiAdvance or the committed path
                // prediction slides and the tip drifts off the dashed plan.
                m_holdEnv = 0f;
                m_pvActive = false;
                Debug.Log($"[Wave] crest strike committed: radius {m_pvSteered.bias + m_pvSteered.ampTip:F2} rad bend");
            }
            else
            {
                m_pvActive = false;
            }
        }

        private static UnityEngine.InputSystem.Key? ToInputKey(KeyCode code) => code switch
        {
            KeyCode.Space => UnityEngine.InputSystem.Key.Space,
            KeyCode.Return or KeyCode.KeypadEnter => UnityEngine.InputSystem.Key.Enter,
            KeyCode.A => UnityEngine.InputSystem.Key.A,
            KeyCode.B => UnityEngine.InputSystem.Key.B,
            KeyCode.C => UnityEngine.InputSystem.Key.C,
            KeyCode.D => UnityEngine.InputSystem.Key.D,
            KeyCode.E => UnityEngine.InputSystem.Key.E,
            KeyCode.F => UnityEngine.InputSystem.Key.F,
            KeyCode.G => UnityEngine.InputSystem.Key.G,
            KeyCode.H => UnityEngine.InputSystem.Key.H,
            KeyCode.I => UnityEngine.InputSystem.Key.I,
            KeyCode.J => UnityEngine.InputSystem.Key.J,
            KeyCode.K => UnityEngine.InputSystem.Key.K,
            KeyCode.L => UnityEngine.InputSystem.Key.L,
            KeyCode.M => UnityEngine.InputSystem.Key.M,
            KeyCode.N => UnityEngine.InputSystem.Key.N,
            KeyCode.O => UnityEngine.InputSystem.Key.O,
            KeyCode.P => UnityEngine.InputSystem.Key.P,
            KeyCode.Q => UnityEngine.InputSystem.Key.Q,
            KeyCode.R => UnityEngine.InputSystem.Key.R,
            KeyCode.S => UnityEngine.InputSystem.Key.S,
            KeyCode.T => UnityEngine.InputSystem.Key.T,
            KeyCode.U => UnityEngine.InputSystem.Key.U,
            KeyCode.V => UnityEngine.InputSystem.Key.V,
            KeyCode.W => UnityEngine.InputSystem.Key.W,
            KeyCode.X => UnityEngine.InputSystem.Key.X,
            KeyCode.Y => UnityEngine.InputSystem.Key.Y,
            KeyCode.Z => UnityEngine.InputSystem.Key.Z,
            KeyCode.LeftArrow => UnityEngine.InputSystem.Key.LeftArrow,
            KeyCode.RightArrow => UnityEngine.InputSystem.Key.RightArrow,
            KeyCode.UpArrow => UnityEngine.InputSystem.Key.UpArrow,
            KeyCode.DownArrow => UnityEngine.InputSystem.Key.DownArrow,
            _ => null,
        };

        // Crest steering: point the whole wave frame at the target with a
        // SLERP (the wide arc), align the nearest future CREST of the tip
        // segment with the pass moment, and set the elbow extension so the
        // crest lands exactly at the target's radius. Closed form - no
        // Jacobian, no iteration, cannot diverge.
        private void StartSteerEvent()
        {
            ComputeCrestPlan((float3)targetPoint.position,
                out m_evBase, out m_evSteered, out m_qArc);
            m_evStart = Time.time;
            m_ev = EvState.BlendIn;
            m_holdEnv = 0f;
        }

        // Pure computation - no state mutation - so it can run every frame
        // for the hold-to-preview visualization.
        private void ComputeCrestPlan(float3 target,
            out WaveParams oBase, out WaveParams oSteered, out quaternion oArc)
        {
            oBase = SerializedParams();
            oSteered = oBase;
            // NOTE: the hold slowdown is an INPUT STABILIZER only - it eases
            // the live wave while aiming but NEVER enters the plan. The plan
            // always runs at the original frequency.
            float3 P0 = m_targetPoints[0];

            // --- current crest pose (identity frame, UPPER crest, fully
            // extended: theta_j = bias + amp(s)) ---
            float3 dir0 = BaseDirW;
            float3 axisW = BendAxisW;
            float theta0 = oBase.bias + math.lerp(oBase.ampBase, oBase.ampTip, 0.25f);
            float theta1 = math.clamp(oBase.bias + oBase.ampTip,
                math.radians(-10f), math.radians(elbowLimitDeg));
            // Segment directions under crest bends: d1 = rot(d0, theta1) about
            // the (bent) bend axis; the 2-link radius is r = 2L*cos(theta1/2),
            // and the tip direction is d0 rotated by theta1/2 in-plane.
            float3 d0 = math.mul(quaternion.AxisAngle(axisW, theta0), dir0);
            float3 d1 = math.mul(quaternion.AxisAngle(axisW, theta1), d0);
            float3 crestDir = math.normalizesafe(d0 + d1);
            float crestRadius = math.length(d0 + d1) * boneLength;

            // --- target: reach-clamped direction + radius ---
            float3 toT = target - P0;
            float tDist = math.length(toT);
            float rMax = 1.9f * boneLength;                 // leave servo margin
            float rMin = 2f * boneLength * math.cos(math.radians(elbowLimitDeg) * 0.5f);
            float rWant = math.clamp(tDist, rMin + 0.02f, rMax);
            float3 tDir = tDist > 1e-4f ? toT / tDist : crestDir;
            if (tDist > 2f * boneLength)
                Debug.LogWarning($"[Wave] target {tDist:F2} m from shoulder exceeds reach " +
                    $"({2f * boneLength:F2} m) - crest will strike at the closest reachable radius.");

            // --- arc frame: shortest-arc rotation, optionally wound around the
            // start direction to widen the swing into a hook ---
            quaternion qMin = math.normalizesafe(FromToRotation(crestDir, tDir));
            float twist = math.radians(arcTwistDeg);
            quaternion qArc = (math.abs(twist) > 1e-3f)
                ? math.mul(quaternion.AxisAngle(crestDir, twist), qMin) : qMin;
            // Degenerate case: start and target nearly opposite - pick a
            // perpendicular hinge for the shortest-arc rotation.
            if (math.dot(crestDir, tDir) < -0.999f)
            {
                float3 perp = math.normalizesafe(math.cross(crestDir,
                    math.abs(crestDir.y) < 0.9f ? new float3(0, 1, 0) : new float3(1, 0, 0)));
                qArc = math.mul(quaternion.AxisAngle(crestDir, twist), quaternion.AxisAngle(perp, math.PI));
            }
            oArc = qArc;

            // --- elbow extension at the crest: radius depends ONLY on the
            // relative elbow bend: r = 2L*cos(theta1/2) -> theta1* ---
            float theta1Star = 2f * math.acos(math.clamp(rWant / (2f * boneLength), 0f, 1f));
            // Steered ampTip makes the tip-segment UPPER crest equal theta1*.
            // Elbow stiffness lives here: the smaller the delta, the less the
            // elbow has to move - the frame (slerp) does the reaching instead.
            oSteered.ampTip = math.clamp(theta1Star - oBase.bias, 0f, 1.5f);

            // --- crest timing: nearest future UPPER crest of the tip segment
            // aligned with the pass moment via a phase offset (blended in -
            // continuous through the accumulator) ---
            float crestSpeedF = crestSpeed;
            oSteered.freqHz = oBase.freqHz * crestSpeedF; // slow-in / accelerate
            float phiAtPass = m_phiAccum + PhiAdvance(arrivalTime,
                oBase.freqHz, oSteered.freqHz);
            float s1 = 0.75f; // tip segment midpoint
            float xNow = phiAtPass + oBase.phase - oBase.wavenumber * s1;
            float dPhi = math.PI / 2f - xNow;               // to the next +pi/2 crest
            // wrap to nearest crest (2pi periodic)
            dPhi -= 2f * math.PI * math.round(dPhi / (2f * math.PI));
            oSteered.phase = oBase.phase + dPhi;
            float3 crestToT = math.mul(qArc, crestDir) * crestRadius + P0;
        }

        // Rotation taking vector a to vector b (shortest arc).
        private static quaternion FromToRotation(float3 a, float3 b)
        {
            float3 axis = math.cross(a, b);
            float la = math.length(axis);
            if (la < 1e-6f)
                return math.dot(a, b) > 0f
                    ? quaternion.identity
                    : quaternion.AxisAngle(math.normalizesafe(math.cross(a,
                        math.abs(a.y) < 0.9f ? new float3(0, 1, 0) : new float3(1, 0, 0))), math.PI);
            return quaternion.AxisAngle(axis / la, math.acos(math.clamp(math.dot(a, b), -1f, 1f)));
        }

        // Exact integral of the blended frequency over [0, te]: the phase
        // accumulator integrates omega(t), so any prediction of the future
        // phase must integrate the blend, not multiply by the future omega.
        // smoothstep(u) = 3u^2-2u^3 integrates to u^3 - u^4/2.
        private float PhiAdvance(float te, float f0, float f1)
        {
            const float K = 2f * math.PI;
            float ssInt(float u) => u * u * u - u * u * u * u * 0.5f;
            float total = 0f;
            float tIn = blendInTime, T = arrivalTime, tOut = blendOutTime;
            if (te <= tIn)
            {
                float u = te / math.max(tIn, 1e-3f);
                return K * (f0 * te + (f1 - f0) * tIn * ssInt(u));
            }
            total += K * (f0 * tIn + (f1 - f0) * tIn * 0.5f); // full blend-in
            if (te <= T)
                return total + K * f1 * (te - tIn);           // hold at f1
            total += K * f1 * (T - tIn);
            if (te <= T + tOut)
            {
                float u = (te - T) / math.max(tOut, 1e-3f);
                return total + K * (f1 * (te - T) - (f1 - f0) * tOut * ssInt(u));
            }
            total += K * (f1 * tOut - (f1 - f0) * tOut * 0.5f);
            return total + K * f0 * (te - T - tOut);          // back at f0
        }

        // Envelope value for elapsed time since a schedule start (pure).
        private float EnvFromSchedule(float tE)
        {
            if (tE < blendInTime)
            {
                float u = math.saturate(tE / math.max(blendInTime, 1e-3f));
                return u * u * (3f - 2f * u);
            }
            if (tE < arrivalTime) return 1f;
            float v = math.saturate((tE - arrivalTime) / math.max(blendOutTime, 1e-3f));
            return 1f - v * v * (3f - 2f * v);
        }

        // Envelope value at an ABSOLUTE time (side-effect-free) - used to
        // replay the schedule into the future for trajectory prediction.
        private float EnvAtTime(float t)
        {
            if (m_ev == EvState.None) return 0f;
            float tE = t - m_evStart;
            float u = (m_ev == EvState.BlendOut)
                ? math.saturate((tE - arrivalTime) / blendOutTime)
                : math.saturate(tE / blendInTime);
            float e = u * u * (3f - 2f * u);
            return (m_ev == EvState.BlendOut) ? 1f - e : e;
        }

        // Predicted tip path from NOW over the event window: replays the
        // blend schedule, aim slerp, and phase accumulation forward. Same
        // backbone composition as the sampler - this is the honest forecast
        // (drift = feedforward error the correction layer will absorb).
        // Fill one predicted-path buffer from a plan source. scheduled=true
        // replays the blend schedule from time 'schedOff' (elapsed event time
        // for a running event, 0 for a preview); false = plain idle wave.
        private void FillPath(float horizon, float3[] path, float[] phaseOut,
            WaveParams pBase, WaveParams pSteer, quaternion qArcS, float schedOff,
            bool scheduled)
        {
            int n = path.Length;
            float3 P0 = m_targetPoints[0];
            float K = 2f * math.PI;
            // External frequency override (reversed idle) must shape the
            // forecast too, or the drawn path diverges from the real wave.
            float fOverride = waveFreqOverride;
            for (int i = 0; i < n; i++)
            {
                float te = horizon * i / (n - 1);
                float env = scheduled ? EnvFromSchedule(schedOff + te) : 0f;
                WaveParams p = Blend(pBase, pSteer, env);
                quaternion aim = math.slerp(quaternion.identity, qArcS, env);
                // Remaining phase under the plan: integral over
                // [schedOff, schedOff + te] of the plan's frequency profile.
                // Plus the drive's sampling lead: the rig is commanded at
                // m_phiAccum + w*(dt+lookahead) (phiPred convention), so the
                // prediction must describe the COMMANDED pose at each instant
                // or the projection leads the plan by 2pi*f*(dt+lookahead) rad.
                float fSched = math.lerp(pBase.freqHz, pSteer.freqHz, env);
                float fEff = float.IsNaN(fOverride) ? fSched : fOverride;
                float lead = K * fEff * (Time.fixedDeltaTime + lookahead);
                float phi = m_phiAccum + lead + (scheduled
                    ? PhiAdvance(schedOff + te, pBase.freqHz, pSteer.freqHz)
                      - PhiAdvance(schedOff, pBase.freqHz, pSteer.freqHz)
                    : K * fEff * te);
                float3 dir = math.mul(aim, BaseDirW);
                float3 tA = math.mul(aim, BendAxisW);
                float3 pos = P0;
                for (int j = 0; j < 2; j++)
                {
                    float sg = (j + 0.5f) / 2f;
                    float psi = p.twistAmp * math.sin(phi + p.twistPhase - p.twistWavenumber * sg);
                    float3 bAxis = math.normalizesafe(math.mul(quaternion.AxisAngle(dir, psi), tA));
                    float amp = math.lerp(p.ampBase, p.ampTip, sg);
                    float theta = p.bias + amp * math.sin(phi + p.phase - p.wavenumber * sg);
                    if (j == 1)
                    {
                        // color-code: sin of the tip-segment phase (crest..trough)
                        phaseOut[i] = math.sin(phi + p.phase - p.wavenumber * sg);
                        theta = math.clamp(theta,
                            math.radians(-10f), math.radians(elbowLimitDeg));
                    }
                    // BEND FIRST, then advance - must match the sampler's
                    // order or the path runs one station behind the pose.
                    quaternion bend = quaternion.AxisAngle(bAxis, theta);
                    dir = math.mul(bend, dir);
                    tA = math.mul(bend, tA);
                    pos += dir * boneLength;
                }
                // Pose-roll offset (viz): rotate the sampled tip position about
                // the P0->tip chord so the preview matches the executed pose.
                if (p.poseRoll != 0f)
                {
                    float3 chord = math.normalizesafe(pos - P0);
                    quaternion poseRollQ = quaternion.AxisAngle(chord, p.poseRoll);
                    pos = P0 + math.mul(poseRollQ, pos - P0);
                }
                path[i] = pos;
            }
        }

        private void PredictTipPath(float idleHorizon, WaveParams idleP)
        {
            // Viz-only compute (2x FillPath x 96 samples per probe per step) -
            // skip entirely unless drawing is on. At batch scale (256 probes)
            // this was ~24k wasted path samples per physics step.
            if (!drawTipPrediction) { m_predValid = false; m_pvPathValid = false; return; }
            bool inEvent = m_ev != EvState.None;
            if (inEvent)
            {
                // Pin the schedule to the event clock: te=0 here is NOW, so
                // pass the elapsed time as the offset. Blend-out then stays at
                // its absolute moment instead of sliding forward every frame.
                float elapsed = Time.time - m_evStart;
                float horizon = math.max(arrivalTime + blendOutTime - elapsed, 0f) + 0.25f;
                FillPath(horizon, m_predPath, m_predPhase,
                    m_evBase, m_evSteered, m_qArc, elapsed, true);
                m_predValid = true;
                m_pvPathValid = false;
                return;
            }
            // Idle wave path - ALWAYS drawn, hold or not. Frequency comes
            // from idleP: the real frequency normally, or the 1 Hz nominal
            // sweep when the wave is frozen (envelope preview).
            FillPath(idleHorizon, m_predPath, m_predPhase,
                idleP, idleP, quaternion.identity, 0f, false);
            m_predValid = true;
            // Hold-preview plan overlay - dashed. Uses the ORIGINAL frequency
            // (the slowdown is input stabilization, not part of the plan).
            if (m_pvActive)
            {
                FillPath(arrivalTime + blendOutTime + 0.25f, m_predPathPv, m_predPhasePv,
                    m_pvBase, m_pvSteered, m_qPv, 0f, true);
                m_pvPathValid = true;
            }
            else
            {
                m_pvPathValid = false;
            }
        }

        // Blend envelope + current parameter set. env: 0=base, 1=steered.
        private WaveParams EvalEvent(float now, out float env)
        {
            if (m_ev == EvState.None)
            {
                env = 0f;
                return SerializedParams(); // live inspector edits when idle
            }
            float tE = now - m_evStart;
            switch (m_ev)
            {
                case EvState.BlendIn:
                {
                    float u = math.saturate(tE / blendInTime);
                    env = u * u * (3f - 2f * u);              // smoothstep in
                    m_envRate = 6f * u * (1f - u) / math.max(blendInTime, 1e-3f);
                    return Blend(m_evBase, m_evSteered, env);
                }
                case EvState.Hold:
                    env = 1f;
                    m_envRate = 0f;
                    return Blend(m_evBase, m_evSteered, 1f);
                default: // BlendOut
                {
                    float u = math.saturate((tE - arrivalTime) / blendOutTime);
                    env = 1f - u * u * (3f - 2f * u);         // smoothstep out
                    m_envRate = -6f * u * (1f - u) / math.max(blendOutTime, 1e-3f);
                    return Blend(m_evBase, m_evSteered, env);
                }
            }
        }

        private float m_envRate;


        private void FixedUpdate()
        {
            if (!driveEnabled || m_upper == null) return;
            float dt = Time.fixedDeltaTime;

            // Spawn dislocation watchdog: one permanent warning per limb if it
            // was BORN separated (solver never pulled the joint shut) - the
            // signature of a mis-created joint frame, not of agent-driven
            // motion. Only within the first second: later separation is real
            // dynamics and the agent's business.
            m_stepsSinceSpawn++;
            if (!m_dislocLogged && m_stepsSinceSpawn > 60 && m_sepMax > 0.05f)
            {
                m_dislocLogged = true;
                // Verbose only: born-dislocated limbs are excluded from the
                // reward penalty, so the trainer doesn't need the spam - the
                // SpawnHealthy gate handles the policy side silently.
                if (Box3DArticulation.WaveDiagnostics.Verbose)
                    Debug.LogWarning($"[Wave:{ProbeTag()}] BORN DISLOCATED: joint separation {m_sepMax:F3} m one second after spawn " +
                        "- joint frames likely mis-created at spawn; policy cannot fix this");
            }

            // LIVE ROOT ANCHOR: the shoulder point P0 (the base of the whole
            // forward-integrated spline and every servo target) follows the
            // root body's LIVE pose through a critically damped follower.
            // Continuous base motion passes through ~1:1; a drag/tilt of the
            // root bone becomes a short ramp instead of a step input - a
            // step saturates the rate-limited servo (wStop/wCrest caps) and
            // it bang-bangs around the pose (the wobble) while falling
            // behind the real-time phase accumulator (the extra delay).
            if (m_root != null && m_root.Body.IsValid)
            {
                float3 p0Live = BodyPoint(m_root.Body, m_anchorLocal);
                quaternion rotLive = m_root.Body.GetRotation();
                if (!m_anchorInit)
                {
                    m_anchorP0 = p0Live;
                    m_anchorRot = rotLive;
                    m_anchorInit = true;
                }
                else
                {
                    float k = 1f - math.exp(
                        -dt / math.max(1e-3f, anchorLagTau));
                    m_anchorP0 = math.lerp(m_anchorP0, p0Live, k);
                    m_anchorRot = math.normalizesafe(
                        math.slerp(m_anchorRot, rotLive, k));
                }
                m_targetPoints[0] = m_anchorP0;
            }

            // --- Event sequencer: state transitions + envelope ---
            if (m_ev != EvState.None)
            {
                float tE = Time.time - m_evStart;
                if (m_ev == EvState.BlendIn && tE >= blendInTime) m_ev = EvState.Hold;
                else if (m_ev == EvState.Hold && tE >= arrivalTime)
                {
                    // Pass-through moment: report how close the ACTUAL tip got.
                    m_ev = EvState.BlendOut;
                    float3 tipNow = (float3)m_fore.Body.GetPosition()
                        + math.mul(m_fore.Body.GetRotation(), new float3(0f, -boneLength / 2f, 0f));
                    if (targetPoint != null)
                        Debug.Log($"[Wave] pass-through error: {math.length(tipNow - (float3)targetPoint.position):F3} m");
                }
                else if (m_ev == EvState.BlendOut && tE >= arrivalTime + blendOutTime) m_ev = EvState.None;
            }

            // Blended parameter set + envelope rate (0 = idle, base = inspector).
            WaveParams par = EvalEvent(Time.time, out float env);
            if (!float.IsNaN(waveFreqOverride)) par.freqHz = waveFreqOverride;
            float envRate = m_envRate;
            float4 dPar = (m_ev == EvState.None)
                ? float4.zero
                : new float4(
                    m_evSteered.bias - m_evBase.bias,
                    m_evSteered.ampTip - m_evBase.ampTip,
                    m_evSteered.phase - m_evBase.phase,
                    m_evSteered.twistPhase - m_evBase.phase);

            // Hold-to-aim slowdown: while the key is held the live wave eases
            // toward a fraction of its frequency. Ramped and fed through the
            // accumulator, so the slowdown itself is phase-continuous.
            float holdStep = dt / math.max(holdRampTime, 1e-3f);
            float holdTarget = m_pvActive ? 1f : 0f;
            m_holdEnv = math.clamp(
                m_holdEnv + math.clamp(holdTarget - m_holdEnv, -holdStep, holdStep),
                0f, 1f);
            float holdScale = math.lerp(1f, holdFreqScale, m_holdEnv);

            // Advance the phase accumulator. Phase = integral of frequency:
            // blending freq (or anything else) cannot jump the wave.
            float w = 2f * math.PI * par.freqHz * holdScale;
            m_phiAccum += w * dt;

            // The lookahead is now a phase increment, not a time offset: the
            // command executes over [t, t+dt), so the wave is sampled one
            // step ahead in accumulated phase.
            float phiPred = m_phiAccum + w * (Time.fixedDeltaTime + lookahead);

            // phase-offset ramp: <= one wave cycle per second toward the target.
            // abs(): freqHz may be negative (reversed wave). Floor 0.5 Hz: at
            // frozen frequency the wave phase never advances, so the offset is
            // the ONLY way to move the pose - without a floor, sweeping the
            // phase trim at 0 Hz takes minutes instead of ~2 s per cycle.
            float dOff = m_phaseOffsetTarget - m_phaseOffset;
            float rate = 2f * math.PI * math.max(0.5f, math.abs(par.freqHz));
            m_phaseOffset += math.clamp(dOff, -rate * dt, rate * dt);
            phiPred += m_phaseOffset;

            // --- Sample the backbone wave: curvature -> per-segment bend angles ---
            // Stations at s = 0 (shoulder), 0.5 (elbow), 1 (tip). Station j's bend
            // angle feeds segment j. kappa integrated piecewise-constant.
            // The wave frame slerps under the crest-arc steering: the whole
            // envelope (crest disk included) swings continuously - this IS the
            // wide arc. Slerp has no phase, no crests, cannot jump.
            quaternion aim = (m_ev == EvState.None) ? quaternion.identity
                : math.slerp(quaternion.identity, m_qArc, env);
            // DMP strike schedule override: the planner commands the wave frame
            // and bias directly; the event envelope stays underneath but is
            // shadowed while the override answers.
            bool knobOv = false;
            if (DmpKnobOverride != null)
            {
                var ov = DmpKnobOverride(phiPred);
                if (ov.HasValue)
                {
                    aim = ov.Value.aim;
                    par.bias = ov.Value.bias;
                    knobOv = true;
                }
            }
            float3 axis = math.mul(aim, BendAxisW);
            float3 dir = math.mul(aim, BaseDirW);
            // A hinge bends only perpendicular to the bone. If the bend axis is
            // (nearly) parallel to the base direction, AxisAngle rotates the
            // backbone onto itself — no bend exists, targets degenerate to twist
            // noise. Refuse to run rather than chase an impossible manifold.
            float para = math.abs(math.dot(axis, dir));
            if (para > 0.98f)
            {
                Debug.LogError($"[Wave] bendAxis {bendAxis} is parallel to baseDirection {baseDirection} " +
                    $"(dot={para:F2}) — a hinge cannot bend about its own bone axis. " +
                    "Set bendAxis perpendicular to baseDirection (e.g. base=down, bend=Z).");
                return;
            }
            float3 n = axis;

            m_bendRates[0] = m_bendRates[1] = 0f;
            float bend0 = 0f, bend1 = 0f;
            float3 tAxis = axis;               // transported bend axis (Bishop frame) - already rolled (BendAxisW)
            float3 dir0 = dir;                 // bone dir at station 0 (twist ff axis)
            // Envelope derivatives of the parameters (analytic - the blend is
            // smoothstep, so dp/dt is known): they extend the feedforward
            // through parameter CHANGES, not just steady waves.
            float Omega = w + envRate * dPar.z;             // d(x)/dt incl. phase blend
            float OmegaPsi = w + envRate * dPar.w;
            for (int j = 0; j < m_targetDirs.Length; j++)
            {
                float s = (j + 0.5f) / m_targetDirs.Length; // segment midpoints
                float amp = math.lerp(par.ampBase, par.ampTip, s);
                float ampD = envRate * math.lerp(0f, dPar.y, s); // d(amp)/dt at s
                float x = phiPred + par.phase - par.wavenumber * s;
                float sinX = math.sin(x), cosX = math.cos(x);
                // theta = bias(u) + amp(u)*sin(x): full analytic chain rule.
                float theta = par.bias + amp * sinX;
                // During the knob override the bias itself moves (the DMP's
                // knob trajectory) - its rate comes from the planner's finite
                // difference, NOT from the event envelope (dPar.x = 0 then).
                float biasRate = knobOv ? DmpKnobBiasRate : envRate * dPar.x;
                m_bendRates[j] = biasRate + ampD * sinX + amp * cosX * Omega;
                m_bendAccel[j] = 2f * ampD * cosX * Omega - amp * sinX * Omega * Omega;
                // Twist wave: the bend plane itself rotates along the chain. A
                // general space curve needs this - the shoulder twist exists to
                // keep the elbow hinge aligned with the LOCAL bend direction.
                float xPsi = phiPred + par.twistPhase - par.twistWavenumber * s;
                float taD = envRate * (m_ev == EvState.None ? 0f
                    : m_evSteered.twistAmp - m_evBase.twistAmp);
                float psi = par.twistAmp * math.sin(xPsi);
                // Knob override: the aim rotation IS plane steering - its rate
                // about the bone is the shoulder long-axis feedforward.
                m_twistRates[j] = (knobOv ? DmpKnobTwistRate : 0f)
                    + taD * math.sin(xPsi) + par.twistAmp * math.cos(xPsi) * OmegaPsi;
                // Bend axis at this station: tAxis rotated about the bone by psi.
                float3 bAxis = math.normalizesafe(
                    math.mul(quaternion.AxisAngle(dir, psi), tAxis));
                m_bendAxisW[j] = bAxis;
                // Bend the frame (dir AND axis) about bAxis - rigid rotation keeps
                // the pair orthogonal, no Frenet flip degeneracy.
                quaternion bend = quaternion.AxisAngle(bAxis, theta);
                dir = math.mul(bend, dir);
                tAxis = math.mul(bend, tAxis);
                // Horizon: angular distance to the NEXT wave crest (theta-dot = 0).
                if (amp > 1e-4f)
                {
                    float halfPi = math.PI / 2f;
                    float xRev = math.ceil((x - halfPi) / math.PI) * math.PI + halfPi;
                    m_crestDist[j] = math.abs(amp * (math.sin(xRev) - math.sin(x)));
                }
                else
                {
                    m_crestDist[j] = float.PositiveInfinity; // no reversal to respect
                }
                if (j == 0)
                {
                    bend0 = theta;
                    dir0 = dir;
                }
                else
                {
                    // The target MUST be reachable: clamping to the hinge range.
                    float lowerRad = math.radians(-10f);
                    float upperRad = math.radians(elbowLimitDeg);
                    bend1 = math.clamp(theta, lowerRad, upperRad);
                }
            }

            // --- DMP tip override: closed-form 2-link IK onto the scheduled
            // want-point. Replaces the wave's bend computation entirely: the
            // bends/dirs below are the IK solution, the wave loop's output is
            // discarded. Endpoint correction + DriveTo run unchanged on top.
            bool tipOverridden = false;
            if (DmpTipOverride != null)
            {
                float3? wantOv = DmpTipOverride(phiPred);
                if (wantOv.HasValue)
                {
                    tipOverridden = ApplyTipOverride(wantOv.Value, aim, dt,
                        ref bend0, ref bend1);
                }
            }

            // Predicted tip trajectory: idle wave path ALWAYS (original
            // frequency); the hold-preview plan overlays on top when aiming.
            if (drawTipPrediction && !tipOverridden)
            {
                // ALWAYS 1.5 cycles at a FIXED 60 Hz sample rate: sample count
                // scales with the horizon (low freq = more samples), buffer
                // reallocs only when the size changes. Frozen wave (freq ~ 0)
                // sweeps the envelope at nominal 1 Hz - the orbit it would
                // trace once unfrozen.
                // Cap 256, not 2048: near-cancellation blends (mixed +/- freq
                // primitives) sit at |f| ~ 0.001..0.04 Hz, where 60 Hz
                // resolution would demand 2k-7k samples PER PROBE PER FRAME,
                // and the gizmo lines (drawn with duration) accumulate in the
                // editor renderer - the source of the editor-wide lag. 256
                // samples over 1.5 cycles is visually identical at any freq.
                var idleP = SerializedParams();
                float fDraw = math.abs(idleP.freqHz) < 1e-3f ? 1f : math.abs(idleP.freqHz);
                idleP.freqHz = fDraw;
                float horizon = 1.5f / fDraw;
                int sampleCount = math.clamp((int)math.ceil(horizon / math.max(Time.fixedDeltaTime, 1e-4f)), 16, 256);
                if (m_predPath == null || m_predPath.Length != sampleCount)
                {
                    m_predPath = new float3[sampleCount];
                    m_predPhase = new float[sampleCount];
                }
                PredictTipPath(horizon, idleP);
            }
            else
            {
                m_predValid = false;
            }

            // --- Endpoint correction: distribute MEASURED hand error over bends ---
            // The wave is feedforward-only; without this layer the shoulder never
            // uses its spare DOF to cover elbow lag, so hand error accumulates.
            // J = d(hand)/d(bend0, bend1), analytic for the planar 2-link chain.
            float3 handTarget = m_targetPoints[0]
                + math.mul(quaternion.AxisAngle(axis, bend0),
                    math.mul(aim, BaseDirW)) * boneLength
                + math.mul(quaternion.AxisAngle(axis, bend0 + bend1),
                    math.mul(aim, BaseDirW)) * boneLength;
            if (endpointCorrection && m_fore != null)
            {
                float3 handActual = (float3)m_fore.Body.GetPosition()
                    + math.mul(m_fore.Body.GetRotation(), new float3(0f, -boneLength / 2f, 0f));
                float3 e = handTarget - handActual;
                // Out-of-(local bend)-plane error is not the hinge's job: the
                // twist datum handles plane alignment, the correction bends only.
                e -= m_bendAxisW[1] * math.dot(e, m_bendAxisW[1]);

                float3 d0 = math.mul(quaternion.AxisAngle(axis, bend0),
                    math.mul(aim, BaseDirW));
                float3 d1 = math.mul(quaternion.AxisAngle(axis, bend0 + bend1),
                    math.mul(aim, BaseDirW));
                float3 c0 = math.cross(axis, d0), c1 = math.cross(axis, d1);
                float3 j0 = boneLength * c0 + boneLength * c1; // d(hand)/d(bend0)
                float3 j1 = boneLength * c1;                   // d(hand)/d(bend1)

                // 2x2 normal equations (J^T J) delta = J^T e, closed form.
                float a00 = math.dot(j0, j0), a01 = math.dot(j0, j1), a11 = math.dot(j1, j1);
                float b0 = math.dot(j0, e), b1 = math.dot(j1, e);
                float det = a00 * a11 - a01 * a01;
                if (math.abs(det) > 1e-8f)
                {
                    // Motion gate: the correction exists to cover wave-TRACKING
                    // lag, which is proportional to wave speed. On a frozen or
                    // near-frozen wave (|w| ~ 0) the residual error is gravity
                    // droop + actuation lag the servo can never zero - chasing
                    // it closes a delayed loop around a static target and pumps
                    // a limit cycle (limbs sway while the projected pose sits
                    // still). Authority scales with wave motion instead.
                    float motion = math.saturate(math.abs(w) / (2f * math.PI * 0.2f));
                    // RATE controller: error -> desired correction velocity
                    // (rad/s), clamped. A per-frame position jump here has ~30 Hz
                    // loop bandwidth with no damping -> target spins. Integrated
                    // rate is unconditionally stable and cannot wind up.
                    float w0 = (a11 * b0 - a01 * b1) / det * endpointGain * motion;
                    float w1 = (a00 * b1 - a01 * b0) / det * endpointGain * motion;
                    float maxW = endpointMaxRate;
                    w0 = math.clamp(w0, -maxW, maxW);
                    w1 = math.clamp(w1, -maxW, maxW);
                    float db0 = w0 * dt, db1 = w1 * dt;
                    // Respect the elbow's anatomical range in the corrected target
                    // (same bounds the wave clamp uses — one consistent wall).
                    float limitRad = math.radians(elbowLimitDeg);
                    bend1 = math.clamp(bend1 + db1, math.radians(-10f), limitRad);
                    bend0 = math.clamp(bend0 + db0, -math.PI, math.PI);
                    // Feedforward must track the CORRECTED trajectory: the
                    // correction rate IS the added target velocity.
                    m_bendRates[0] += w0;
                    m_bendRates[1] += w1;
                }
            }

            // Rebuild the backbone from the (corrected) bends on the transported axes.
            // MUST start from the AIMED direction: the axes were transported in
            // the aimed frame, so composing them onto the plain base direction
            // builds a different curve whenever aim != identity (any event).
            dir = math.mul(aim, BaseDirW);
            for (int j = 0; j < m_targetDirs.Length; j++)
            {
                float theta = (j == 0) ? bend0 : bend1;
                dir = math.mul(quaternion.AxisAngle(m_bendAxisW[j], theta), dir);
                m_targetDirs[j] = dir;
                m_targetPoints[j + 1] = m_targetPoints[j] + dir * boneLength;
            }

            // Pose-roll offset: rotate the FINAL projected pose about the
            // root->tip chord. Post-transform on the finished wave: root and
            // tip are preserved, the intermediate bend direction swings around
            // the chord. The bend-axis targets rotate with the pose, so the
            // twist datum (TwistCorrect) and the FF axes stay consistent. A
            // constant offset contributes no angular velocity; if poseRoll is
            // animated by the caller, its rate is a shoulder long-axis term
            // the servo absorbs (slow lerp on the agent side).
            if (par.poseRoll != 0f)
            {
                float3 chord = math.normalizesafe(
                    m_targetPoints[2] - m_targetPoints[0]);
                quaternion poseRollQ = quaternion.AxisAngle(chord, par.poseRoll);
                for (int j = 0; j < m_targetDirs.Length; j++)
                {
                    m_targetDirs[j] = math.mul(poseRollQ, m_targetDirs[j]);
                    m_bendAxisW[j] = math.mul(poseRollQ, m_bendAxisW[j]);
                    m_targetPoints[j + 1] = m_targetPoints[0]
                        + math.mul(poseRollQ, m_targetPoints[j + 1] - m_targetPoints[0]);
                }
            }

            // --- Torque feedforward: tau = I_eff * alpha, from the analytic wave ---
            // The velocity motor discovers deceleration at a wave crest only when
            // it happens; the wave's second derivative is known in advance. Apply
            // each body's composite-inertia torque for its ABSOLUTE target angular
            // acceleration now, so the motor only corrects residual error.
            // Gate: feedforward only helps ON the manifold; desynced, it pumps
            // (resonance). Fade ff out with error, let the P-servo recover.
            // During the knob override the rates are the COMMAND's true rates
            // (finite-diff of the knob trajectory) - they must not be error-
            // gated: the strike's errors are big by nature and the gate would
            // kill exactly the feedforward that removes the lag.
            float gate0 = knobOv ? 1f
                : math.saturate(1f - m_shoulder.lastErrMag / ffGateAngle);
            float gate1 = knobOv ? 1f
                : math.saturate(1f - m_elbow.lastErrMag / ffGateAngle);
            if (torqueFeedforward)
            {
                float alphaUpper = m_bendAccel[0];                    // abs ang accel of upper arm
                float alphaFore = m_bendAccel[0] + m_bendAccel[1];    // abs ang accel of forearm
                var anchor0 = (float3)m_upper.Body.GetPosition();
                float iUpper = SplineChainController.CompositeInertiaAboutAxis(
                    m_shoulder.distal, axis, anchor0);
                float iFore = SplineChainController.BodyInertiaAboutAxis(m_fore.Body, axis);
                m_upper.Body.ApplyTorque(axis * (iUpper * alphaUpper * gate0), true);
                m_fore.Body.ApplyTorque(axis * (iFore * alphaFore * gate1), true);
            }

            // --- Targets: bone alignment + twist correction (same datum logic) ---
            // Shoulder twist datum: the elbow hinge axis in UPPER-arm LOCAL space
            // (frameB's rotation is forearm-local — the wrong basis for qUpper;
            // at spawn both coincide, which is why the pose probe got away with it).
            quaternion qUpper = ShortestArc(kLocalBone, m_targetDirs[0]);
            // Shoulder twist datum: rotate the hinge axis (upper-local) onto the
            // STATION-1 transported bend axis - the twist that makes the hinge's
            // 1-DOF bend coincide with the wave's local bend. This is how a
            // hinge-only chain follows a general space curve.
            qUpper = TwistCorrect(qUpper, m_targetDirs[0], m_hingeAxisInUpper, m_bendAxisW[1]);

            quaternion qFore = ShortestArc(kLocalBone, m_targetDirs[1]);
            // Forearm's own datum stays forearm-local (frameB basis) — correct for qFore.
            qFore = TwistCorrect(qFore, m_targetDirs[1],
                math.mul(m_elbow.frameB.Rotation, math.forward()), m_bendAxisW[1]);

            // Feedforward: the wave is analytic, so the target velocity is known
            // in advance — sample-ahead removes tracking lag at the source.
            // Shoulder: bend-rate[0] rotates the whole chain about the bend axis.
            // Elbow: bend-rate[1] IS the hinge rate.
            // Shoulder angular velocity = bend about station-0 axis + twist
            // (psi-dot) about the bone direction. Both analytic.
            float3 ffWorld0 = m_bendAxisW[0] * m_bendRates[0] + dir0 * m_twistRates[0];
            DriveTo(m_shoulder, m_root.Body, m_upper.Body, m_root.Body.GetRotation(), qUpper, dt,
                ffWorld0, 0f, crestClamp ? m_crestDist[0] : float.PositiveInfinity, gate0);
            DriveTo(m_elbow, m_upper.Body, m_fore.Body, qUpper, qFore, dt,
                float3.zero, m_bendRates[1] * gate1,
                crestClamp ? m_crestDist[1] : float.PositiveInfinity, gate1);

            // Joint separation drift: each joint's two world anchors must
            // coincide. sep = (pA + Ra*la) - (pB + Rb*lb). Growth here = the
            // solver being driven past its correction budget (the "dislocation").
            // (Earlier version had the rotation terms swapped and measured
            // body-center-to-anchor offset — a constant — as 2 m of "drift".)
            // CreateSpherical(root, upper): frameA lives on the ROOT.
            float3 sepU = (float3)m_root.Body.GetPosition()
                + math.mul(m_root.Body.GetRotation(), (float3)m_shoulder.frameA.Position)
                - ((float3)m_upper.Body.GetPosition()
                + math.mul(m_upper.Body.GetRotation(), (float3)m_shoulder.frameB.Position));
            float3 sepE = (float3)m_upper.Body.GetPosition()
                + math.mul(m_upper.Body.GetRotation(), (float3)m_elbow.frameA.Position)
                - ((float3)m_fore.Body.GetPosition()
                + math.mul(m_fore.Body.GetRotation(), (float3)m_elbow.frameB.Position));

            // Telemetry: did the motor actually hit its torque ceiling? Distinguishes
            // "driver never commands enough" from "torque limit saturates".
            m_sepMax = math.max(math.length(sepU), math.length(sepE)); // live for reward/obs consumers
            if (torqueTelemetry && Time.time - m_lastTelemetry > 0.5f)
            {
                m_lastTelemetry = Time.time;
                float tS = math.length(new SphericalJoint { Id = m_shoulder.id }.GetMotorTorque());
                float tE = new RevoluteJoint { Id = m_elbow.id }.GetMotorTorque();
                float sU = math.length(sepU), sE = math.length(sepE);
                Debug.Log($"[Wave] shoulder tau={tS:F1}/{shoulderTorqueLimit} N*m | " +
                          $"elbow tau={tE:F1}/{elbowTorqueLimit} N*m | " +
                          $"err={m_shoulder.lastErrMag:F3},{m_elbow.lastErrMag:F3} rad | " +
                          $"sep={sU:F4},{sE:F4} m");
                if (sE > 0.02f)
                    Debug.LogWarning($"[Wave] ELBOW SEPARATION {sE:F3} m; " +
                        $"ff: {torqueFeedforward}, crest: {crestClamp}, ecorr: {endpointCorrection}, " +
                        $"gates: {gate0:F2}/{gate1:F2}, wCrest: {m_crestDist[0]:F2},{m_crestDist[1]:F2} | " +
                        $"|w| upper={math.length(m_upper.Body.GetAngularVelocity()):F2} " +
                        $"fore={math.length(m_fore.Body.GetAngularVelocity()):F2} rad/s | " +
                        $"sep dir={(math.normalizesafe(sepE)):F2}");
            }
        }

        // ---- drive (identical math to DriveSignProbe: inertia-aware servo) ----

        /// Closed-form planar 2-link IK onto 'want' in the aimed bend plane.
        /// Sets bend0/bend1 so the rebuilt backbone puts the hand on the point;
        /// rates/accel/crest-horizon come from finite differences (no wave
        /// analytic). Returns false if the point is degenerate.
        private bool ApplyTipOverride(float3 want, quaternion aim, float dt,
            ref float bend0, ref float bend1)
        {
            float3 n = math.mul(aim, BendAxisW);
            float3 u0 = math.mul(aim, BaseDirW);
            float3 rel = want - m_targetPoints[0];
            float d = math.length(rel);
            float rMin = 2f * boneLength * math.cos(math.radians(elbowLimitDeg) * 0.5f);
            if (d < 1e-4f) return false;
            d = math.clamp(d, rMin + 1e-3f, 2f * boneLength - 1e-3f);

            // in-plane target direction (the plane through p0 with normal n)
            float3 wt = rel - n * math.dot(rel, n);
            if (math.lengthsq(wt) < 1e-8f) return false;
            wt = math.normalizesafe(wt);
            float3 c = math.cross(u0, wt);
            float phi = math.atan2(math.dot(c, n), math.dot(u0, wt)); // signed, (-pi, pi]

            // law of cosines: interior elbow angle for the chord length d
            float cosGamma = (d * d - 2f * boneLength * boneLength)
                / (2f * boneLength * boneLength);
            float bendMag = math.PI - math.acos(math.clamp(cosGamma, -1f, 1f));
            // anatomical hinge range; shoulder turn takes the remainder so the
            // chord (bend0 + bend1/2) points at the target
            bend1 = math.clamp(bendMag, math.radians(-10f), math.radians(elbowLimitDeg));
            bend0 = math.clamp(phi - bend1 * 0.5f, -math.PI, math.PI);

            // feedforward from finite differences of the commanded bends
            if (m_tipOvValid && dt > 0f)
            {
                m_bendRates[0] = (bend0 - m_prevBend0) / dt;
                m_bendRates[1] = (bend1 - m_prevBend1) / dt;
            }
            m_prevBend0 = bend0;
            m_prevBend1 = bend1;
            m_tipOvValid = true;
            m_bendAccel[0] = m_bendAccel[1] = 0f;
            m_twistRates[0] = m_twistRates[1] = 0f;
            m_crestDist[0] = m_crestDist[1] = float.PositiveInfinity;
            return true;
        }

        private float m_prevBend0, m_prevBend1;
        private bool m_tipOvValid;

        private void DriveTo(JointRef j, Body parent, Body child,
            quaternion parentTarget, quaternion childTarget, float dt,
            float3 targetAngVelWorld, float hingeRateFf, float crestDist, float ffGate)
        {
            quaternion qRelT = SplineChainController.RelativeTarget(
                parentTarget, childTarget, j.frameA, j.frameB);
            quaternion qRelA = SplineChainController.RelativeTarget(
                parent.GetRotation(), child.GetRotation(), j.frameA, j.frameB);
            quaternion e = math.mul(qRelT, math.inverse(qRelA));
            float3 rv = SplineChainController.RotationVector(e);
            // Twist-feasibility: the long-axis component of the demanded error
            // is capped. Everything downstream (wStop, servo gain, torque
            // budget) then plans around an ACHIEVABLE twist instead of one the
            // motor saturates on - the bend axes keep their budget.
            if (!j.isHinge && shoulderTwistLimitDeg > 0f)
            {
                float3 fwdA = math.forward(math.mul(parent.GetRotation(), j.frameA.Rotation));
                float twistErr = math.dot(rv, fwdA);
                float twistMax = math.radians(shoulderTwistLimitDeg);
                if (math.abs(twistErr) > twistMax)
                    rv -= fwdA * (twistErr - math.sign(twistErr) * twistMax);
            }
            float theta = math.length(rv);

            float3 anchorW = (float3)parent.GetPosition()
                + math.mul(parent.GetRotation(), (float3)j.frameA.Position);

            float3 wRelWorld = child.GetAngularVelocity() - parent.GetAngularVelocity();
            float3 wRelA = math.mul(math.inverse(math.mul(parent.GetRotation(), j.frameA.Rotation)), wRelWorld);

            float3 axisW = (theta > 1e-6f)
                ? math.normalizesafe(math.mul(parent.GetRotation(), math.mul(j.frameA.Rotation, rv / theta)))
                : math.mul(parent.GetRotation(), math.mul(j.frameA.Rotation, math.forward()));
            float iEff = SplineChainController.CompositeInertiaAboutAxis(j.distal, axisW, anchorW);
            float iSelf = SplineChainController.BodyInertiaAboutAxis(child, axisW);
            float alphaMax = j.torqueLimit / math.max(iEff, 1e-4f);
            float wStop = math.sqrt(2f * alphaMax * theta);
            // Horizon shaping: never command more speed than the PLANNED
            // deceleration into the next wave crest can cancel. Bounds the servo
            // to the manifold -> no resonance pumping at mismatched parameters.
            float wCrest = crestDist < float.PositiveInfinity
                ? math.sqrt(2f * alphaMax * math.max(crestDist, 1e-4f))
                : float.PositiveInfinity;
            // First-order position->velocity map at an ACHIEVABLE bandwidth.
            // The old kp*theta/dt assumed instant velocity realization (50 rad/s
            // commands): at human-scale inertia the motor reverses full-torque
            // every half-cycle -> limit cycle + constraint-solver spikes (the
            // hinge "dislocation" along its axis). kp slider 0.1-2 -> 2-40 rad/s
            // per rad of error, i.e. 0.3-6 Hz bandwidth, discrete-stable at 50 Hz.
            float servoGain = kp * 20f; // 1/s
            float mag = math.min(math.min(servoGain * theta, wStop), wCrest);
            // Absolute ceiling: on the bone's long axis the correct inertia is
            // ~0.002 kg*m^2 -> wStop reaches ~90 rad/s and any noise becomes a
            // 150 N*m kick (the twist chatter). No velocity command may exceed
            // what one control step can meaningfully realize.
            mag = math.min(mag, wCeiling);
            float3 dirV = (theta > 1e-6f) ? rv / theta : float3.zero;
            // Damping-in-command, capped: sqrt(iEff/iSelf) can reach ~4 at human
            // scale and an uncapped kd*theta-dot term through a laggy velocity
            // motor is itself an oscillator. 3 1/s cap keeps it a brake, not a driver.
            float kdHere = math.min(kd * math.sqrt(iEff / math.max(iSelf, 1e-4f)) * zeta, 3f);

            // Feedforward target velocity -> joint-A space. The servo only has to
            // correct residual error; the lag no longer comes from the P term.
            float3 wFf = math.mul(math.inverse(math.mul(parent.GetRotation(), j.frameA.Rotation)),
                targetAngVelWorld - parent.GetAngularVelocity());

            float3 w = dirV * mag + wFf * ffGate - kdHere * wRelA;
            j.lastErrMag = theta;

            if (j.isHinge)
            {
                var rev = new RevoluteJoint { Id = j.id };
                rev.EnableMotor(true);
                rev.SetMaxMotorTorque(j.torqueLimit);
                rev.SetMotorSpeed(math.dot(w, math.forward()) + hingeRateFf);
            }
            else
            {
                float3 wWorld = math.mul(parent.GetRotation(), math.mul(j.frameA.Rotation, w));
                var sph = new SphericalJoint { Id = j.id };
                sph.EnableMotor(true);
                // Per-axis budget: the long axis (twist) has ~50x less inertia
                // than the transverse axes; one torque limit would put 150 N*m
                // into a 0.002 kg*m^2 axis -> noise amplification. Split the
                // spherical command: twist component gets the (smaller) twist
                // budget, bend components the main one.
                bool twistDominant = math.abs(math.dot(w, math.forward())) > math.length(w) * 0.7f;
                float budget = twistDominant ? shoulderTwistTorqueLimit : j.torqueLimit;
                sph.SetMaxMotorTorque(budget);
                sph.SetMotorVelocity(wWorld);
            }
        }

        // ---- setup helpers (official HybridSpawner pattern, same as probe) ----

        private static Mesh s_cubeMesh; // built-in cube, editor visual only

        private Box3DBody MakeBone(string name, Vector3 position, Vector3 scale, Box3DBodyType type, float massKg,
            Quaternion rotation = default)
        {
            // NOT CreatePrimitive: the primitive's MeshCollider cooks a mesh
            // per bone (8 per rig rebuild) that we immediately destroy, and
            // the renderer is dead weight in headless batch. Box3DBoxShape
            // owns collision. Editor-only visuals keep scene-view tuning
            // (ConventionTuner) readable - builds spawn invisible bones, fine
            // for a batch-trained rig.
            var go = new GameObject(name);
            go.SetActive(false); // defer Awakes: Box3DBody.Awake gathers shapes exactly once
#if UNITY_EDITOR
            var mf = go.AddComponent<MeshFilter>();
            if (s_cubeMesh == null) s_cubeMesh = Resources.GetBuiltinResource<Mesh>("Cube.fbx");
            mf.sharedMesh = s_cubeMesh;
            go.AddComponent<MeshRenderer>();
#endif
            go.name = name;
            // Parent under the probe: without this the bones are scene-root
            // orphans - the demo's ClearGenerated (episode reset) only sweeps
            // its own children, so every reset rained a new limb set into the
            // world while the old ones lived on.
            go.transform.SetParent(transform, false);
            go.transform.position = position;
            // Bones are boxes with the long axis on local Y. The root frame is
            // vertical only when the limb hangs DOWN - a horizontal base
            // direction (T-pose arms) must rotate the bone so local Y aligns
            // with the spawn direction, or the box stays upright and slices
            // through the torso at spawn (contact kick -> born dislocation).
            go.transform.rotation = rotation == default ? m_frameRot : rotation; // body-local frame = root frame
            go.transform.localScale = scale;
            go.AddComponent<Box3DBoxShape>();
            var body = go.AddComponent<Box3DBody>();
            body.BodyType = type;
            // DYNAMIC ONLY: SetMassData on a STATIC body
            // SetMassData must come AFTER activation: the GameObject is inactive,
            // so Box3DBody.Awake() (which creates the native body) hasn't run —
            // calling into native with an invalid handle crashes the editor.
            go.SetActive(true);
            // Explicit anthropometric mass: I = m/12*(b^2 + c^2) for a box about
            // its COM (long axis Y). Otherwise shape density decides the mass and
            // every torque/acceleration number loses its physical meaning.
            // DYNAMIC ONLY: SetMassData on a STATIC body gives it a finite
            // inverse mass - the joint solver then physically moves the root
            // bone (measured: ±0.3 m oscillation at idle), the wave anchor
            // dances and the servo limit-cycles forever.
            if (body.Body.IsValid && type == Box3DBodyType.Dynamic)
            {
                float a = scale.y, b = scale.x, c = scale.z;
                var md = new MassData
                {
                    Mass = massKg,
                    Center = float3.zero,
                    Inertia = new B3Matrix3
                    {
                         Cx = new float3(massKg / 12f * (a * a + c * c), 0f, 0f),
                         Cy = new float3(0f, massKg / 12f * (b * b + c * c), 0f),
                         Cz = new float3(0f, 0f, massKg / 12f * (a * a + b * b)),
                    },
                };
                body.Body.SetMassData(md);
            }
            return body;
        }

        private JointRef CreateSpherical(Box3DBody a, Box3DBody b, float3 anchor, float hertz, float damping)
        {
            quaternion frameRot = SplineChainController.FrameFromAxisWorld(
                math.mul(m_frameRot, new float3(0f, -1f, 0f)));
            var frames = LocalFrames(a.Body, b.Body, new B3Transform { Position = anchor, Rotation = frameRot });
            var def = SphericalJointDef.Default;
            def.Hertz = hertz; def.DampingRatio = damping;
            def.Base.BodyIdA = a.Body.Id; def.Base.BodyIdB = b.Body.Id; def.Base.CollideConnected = false;
            def.Base.LocalFrameA = frames.a; def.Base.LocalFrameB = frames.b;
            return new JointRef
            {
                id = Box3DWorld.Instance.World.CreateSphericalJoint(def).Id,
                isHinge = false, frameA = frames.a, frameB = frames.b,
                bodyA = a, bodyB = b,
            };
        }

        private JointRef CreateHinge(Box3DBody a, Box3DBody b, float3 anchor, float3 axis, float hertz, float damping)
        {
            quaternion frameRot = SplineChainController.FrameFromAxisWorld(axis);
            var frames = LocalFrames(a.Body, b.Body, new B3Transform { Position = anchor, Rotation = frameRot });
            var def = RevoluteJointDef.Default;
            def.Hertz = hertz; def.DampingRatio = damping;
            def.EnableLimit = true;
            def.LowerAngle = math.radians(elbowMinDeg);                 // slight hyperextension slack
            def.UpperAngle = math.radians(elbowLimitDeg);
            def.Base.BodyIdA = a.Body.Id; def.Base.BodyIdB = b.Body.Id; def.Base.CollideConnected = false;
            def.Base.LocalFrameA = frames.a; def.Base.LocalFrameB = frames.b;
            return new JointRef
            {
                id = Box3DWorld.Instance.World.CreateRevoluteJoint(def).Id,
                isHinge = true, frameA = frames.a, frameB = frames.b,
                bodyA = a, bodyB = b,
            };
        }

        private static (B3Transform a, B3Transform b) LocalFrames(Body a, Body b, B3Transform worldFrame)
        {
            quaternion qa = a.GetRotation(), qb = b.GetRotation();
            var la = new B3Transform
            {
                Position = math.mul(math.inverse(qa), (float3)(worldFrame.Position - a.GetPosition())),
                Rotation = math.mul(math.inverse(qa), worldFrame.Rotation)
            };
            var lb = new B3Transform
            {
                Position = math.mul(math.inverse(qb), (float3)(worldFrame.Position - b.GetPosition())),
                Rotation = math.mul(math.inverse(qb), worldFrame.Rotation)
            };
            return (la, lb);
        }

        private static float3 BodyPoint(Body body, float3 local)
            => body.GetPosition() + math.mul(body.GetRotation(), local);

        private static quaternion ShortestArc(float3 from, float3 to)
        {
            from = math.normalizesafe(from, kLocalBone);
            to = math.normalizesafe(to, kLocalBone);
            float3 axis = math.cross(from, to);
            float axisLen = math.length(axis);
            if (axisLen < 1e-6f)
                return math.dot(from, to) > 0f ? quaternion.identity
                     : new quaternion(0f, 1f, 0f, 0f);
            return quaternion.AxisAngle(axis / axisLen, math.acos(math.clamp(math.dot(from, to), -1f, 1f)));
        }

        private static quaternion TwistCorrect(quaternion qTarget, float3 boneDir,
            float3 hingeAxisLocal, float3 bendNormal)
        {
            float3 axisW = math.mul(qTarget, hingeAxisLocal);
            float3 axisP = axisW - boneDir * math.dot(axisW, boneDir);
            float3 hP = bendNormal - boneDir * math.dot(bendNormal, boneDir);
            if (math.lengthsq(axisP) < 1e-6f || math.lengthsq(hP) < 1e-6f)
                return qTarget;
            return math.mul(ShortestArc(math.normalizesafe(axisP), math.normalizesafe(hP)), qTarget);
        }

        // ---- gizmos: red = sampled backbone (target), green/white = actual ----

        private void OnDrawGizmos()
        {
            if (!drawTipPrediction) return; // batch mode: no per-limb viz
            if (targetPoint != null)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(targetPoint.position, 0.04f);
            }
            if (m_targetPoints != null && m_targetPoints.Length == 3)
            {
                Gizmos.color = Color.red;
                //Debug.DrawLine((Vector3)m_targetPoints[0], (Vector3)m_targetPoints[1], Color.red, 0.5f);
                //Debug.DrawLine((Vector3)m_targetPoints[1], (Vector3)m_targetPoints[2], Color.red, 0.5f);
                Gizmos.DrawWireSphere((Vector3)m_targetPoints[1], 0.06f);
                Gizmos.DrawWireSphere((Vector3)m_targetPoints[2], 0.06f);
            }

            if (m_predValid && m_predPath != null)
            {
                // color-code by tip-segment wave phase: RED = crest (max
                // extension, zero velocity), BLUE = trough, gray = crossing.
                var crest = new Color(1f, 0.25f, 0.1f);
                var trough = new Color(0.2f, 0.5f, 1f);
                var cross = new Color(0.55f, 0.55f, 0.55f);
                bool inEvent = m_ev != EvState.None;
                int n = m_predPath.Length;
                float elapsedG = inEvent ? (Time.time - m_evStart) : 0f;
                float horizonG = arrivalTime + blendOutTime + 0.25f;
                int passIdx = math.clamp(
                    (int)(n * math.saturate((arrivalTime - elapsedG) / horizonG)), 0, n - 1);
                for (int i = 1; i < n; i++)
                {
                    float m = m_predPhase[i]; // -1..1
                    Color c = m >= 0f
                        ? Color.Lerp(cross, crest, m)
                        : Color.Lerp(cross, trough, -m);
                    float alpha = (!inEvent || i <= passIdx) ? 1f : 0.35f;
                    c.a = alpha;
                    Debug.DrawLine((Vector3)m_predPath[i - 1], (Vector3)m_predPath[i], c);
                }
                if (inEvent)
                {
                    // crest strike point (the sample at the pass moment)
                    Gizmos.color = crest;
                    Gizmos.DrawWireSphere((Vector3)m_predPath[passIdx], 0.05f);
                }
                // Hold-preview plan: dashed overlay, same color coding.
                if (m_pvPathValid)
                {
                    int np = m_predPathPv.Length;
                    for (int i = 1; i < np; i++)
                    {
                        float m = m_predPhasePv[i];
                        Color c = m >= 0f
                            ? Color.Lerp(cross, crest, m)
                            : Color.Lerp(cross, trough, -m);
                        PopcronGizmos.Line((Vector3)m_predPathPv[i - 1],
                            (Vector3)m_predPathPv[i], c, true);
                    }
                }
            }

            if (m_upper != null) DrawLink(m_upper);
            if (m_fore != null) DrawLink(m_fore);
        }

        private void DrawLink(Box3DBody body)
        {
            float3 p = (float3)body.Body.GetPosition();
            quaternion q = body.Body.GetRotation();

            Vector3 c = new Vector3(p.x, p.y, p.z);
            Vector3 rx = math.mul(q, new float3(m_halfExtents.x, 0f, 0f));
            Vector3 ry = math.mul(q, new float3(0f, m_halfExtents.y, 0f));
            Vector3 rz = math.mul(q, new float3(0f, 0f, m_halfExtents.z));
            var p000 = c - rx - ry - rz; var p100 = c + rx - ry - rz;
            var p110 = c + rx + ry - rz; var p010 = c - rx + ry - rz;
            var p001 = c - rx - ry + rz; var p101 = c + rx - ry + rz;
            var p111 = c + rx + ry + rz; var p011 = c - rx + ry + rz;
            Gizmos.color = Color.white;
            Gizmos.DrawLine(p000, p100); Gizmos.DrawLine(p100, p110); Gizmos.DrawLine(p110, p010); Gizmos.DrawLine(p010, p000);
            Gizmos.DrawLine(p001, p101); Gizmos.DrawLine(p101, p111); Gizmos.DrawLine(p111, p011); Gizmos.DrawLine(p011, p001);
            Gizmos.DrawLine(p000, p001); Gizmos.DrawLine(p100, p101); Gizmos.DrawLine(p110, p111); Gizmos.DrawLine(p010, p011);
        }
    }
}
