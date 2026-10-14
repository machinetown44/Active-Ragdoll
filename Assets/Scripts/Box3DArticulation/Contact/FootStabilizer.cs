using System;
using Box3D;
using Box3D.Hybrid;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>Passive foot contact for a limb's distal body. A query-only sphere mover rides
    /// the bone tip; FootContactModel (the snowboard body-contact port) converts planes + hits
    /// into analytic support + friction applied to the distal body pre-solve. The engine's
    /// contact solver never sees the tip — no solver contact noise.
    ///
    /// Support: plastic foundation (graded capacity ramp + plateau), lift band = liftDepth
    /// above the terrain, damping ratio zeta. Grip (mu) falls off with the angle between the
    /// bone's long axis and the contact normal — within gripConeAngle of vertical the foot
    /// grips like a planted sole (muPlanted); past gripLossAngle it slides (muFallen). That is
    /// the passive stabilizer: planted feet hold, collapsed/rolled feet slide.
    ///
    /// Execution order -200: before Box3DWorld.FixedUpdate, forces land pre-solve.
    /// Terrain is selected by the Box3DTerrain Unity layer — the floor must be on it.</summary>
    [DefaultExecutionOrder(-200)]
    [RequireComponent(typeof(Box3DBody))]
    public sealed class FootStabilizer : MonoBehaviour
    {
        public const string TerrainLayerName = "Box3DTerrain";

        [Header("Tip mover")]
        [SerializeField, Tooltip("Bone tip in the body frame (query sphere center).")]
        private Vector3 tipLocal = new Vector3(0f, -0.25f, 0f);
        [SerializeField, Tooltip("Query sphere radius (m). Clamped >= liftDepth: the mover stays overlapped while the tip rides the lift band.")]
        private float probeRadius = 0.2f;

        [Header("Support (rigid-ground regime of the snow model)")]
        [SerializeField, Tooltip("Lift distance (m): support engages within this band above the terrain (snowDepth).")]
        private float liftDepth = 0.25f;
        [SerializeField, Tooltip("Bearing capacity at full burial, in g-loads of the loaded mass.")]
        private float bearingFactor = 5f;
        [SerializeField, Tooltip("Capacity ramp length (m): pressure ramps linearly over this depth, then plateaus — the softness.")]
        private float capacityRamp = 0.03f;
        [SerializeField, Tooltip("Damping ratio of the approach-velocity term.")]
        private float dampingRatio = 0.2f;
        [SerializeField, Tooltip("Force budget in g-loads of the loaded mass.")]
        private float maxGLoad = 5f;
        [SerializeField, Tooltip("Rotational damping multiplier at the contact (passive ankle).")]
        private float torqueDamp = 0.2f;
        [SerializeField, Tooltip("Anchor crossfade: torque arm blends to COM as burial shrinks.")]
        private float centroidFade = 0.5f;
        [SerializeField, Tooltip("Contact-normal slew limit (deg/s).")]
        private float normalSlewDegPerSec = 0f;

        [Header("Grip cone (foot articulation limits)")]
        [SerializeField, Tooltip("Friction mu at full grip (bone axis within the cone of the contact normal).")]
        private float muPlanted = 0.9f;
        [SerializeField, Tooltip("Friction mu once grip is lost (bone tilted past gripLossAngle).")]
        private float muFallen = 0.15f;
        [SerializeField, Tooltip("Bone-normal angle (deg) up to which grip stays full.")]
        private float gripConeAngle = 35f;
        [SerializeField, Tooltip("Bone-normal angle (deg) at which grip is fully lost.")]
        private float gripLossAngle = 90f;

        [Header("Cone spring (passive ankle limit)")]
        [SerializeField, Tooltip("Restorative torque toward the cone center, N*m per rad of axis-normal angle. Zero = damping only.")]
        private float coneStiffness = 300f;
        [SerializeField, Tooltip("Damping of the cone spring, N*m*s/rad (same rotation axis as the spring).")]
        private float coneDamping = 2f;

        [Header("Suspension bridge (real-contact foot surgery)")]
        [SerializeField, Tooltip("On the first play step, insert the foot suspension: calf -slider- bridge -spherical(ankle)- sole box. The sole then rides a real solved contact; the analytic support path below retires once inserted.")]
        private bool insertSuspension = true;

        [Header("Slider joint (calf-bridge suspension)")]
        [SerializeField, Min(0.01f), Tooltip("Bridge body mass (kg). Keep small - it rides in every distal-inertia computation.")]
        private float bridgeMassKg = 0.3f;
        [SerializeField, Min(0.005f), Tooltip("Slider travel half-range (m): suspension compression along the calf axis.")]
        private float sliderTravel = 0.1f;
        [SerializeField, Tooltip("Slider spring natural frequency (Hz) - the suspension softness under landing impact.")]
        private float sliderSpringHertz = 10f;
        [SerializeField, Range(0.05f, 2f), Tooltip("Slider spring damping ratio. ~0.7 = near-critical absorption without sag.")]
        private float sliderSpringDamping = 8f;
        [SerializeField, Tooltip("Slider anchor offset along the calf axis (m, distal from the tip): where the bridge hangs at rest. Negative = above the tip.")]
        private float sliderAnchorOffset = 0.05f;

        [Header("Ankle joint (bridge-sole hinge, passive)")]
        [SerializeField, Tooltip("Ankle spring natural frequency (Hz): returns the sole to neutral like the old cone spring.")]
        private float ankleSpringHertz = 10f;
        [SerializeField, Range(0.05f, 2f), Tooltip("Ankle spring damping ratio.")]
        private float ankleSpringDamping = 10f;
        [SerializeField, Tooltip("Ankle hinge limit, min angle (deg) about the lateral axis - negative = plantarflexion.")]
        private float ankleLimitMinDeg = -45f;
        [SerializeField, Tooltip("Ankle hinge limit, max angle (deg) about the lateral axis - positive = dorsiflexion.")]
        private float ankleLimitMaxDeg = 65f;

        [Header("Sole (ground contact body)")]
        [SerializeField, Min(0.01f), Tooltip("Sole body mass (kg) - the box that actually contacts the ground.")]
        private float soleMassKg = 0.5f;
        [SerializeField, Min(0f), Tooltip("Sole width (m, across the stride). 0 = auto from the builder footprint cross-section.")]
        private float soleWidth = 0f;
        [SerializeField, Min(0f), Tooltip("Sole length (m, heel to toe). 0 = auto from the builder footprint length.")]
        private float soleLength = 0f;
        [SerializeField, Min(0f), Tooltip("Sole thickness (m). 0 = auto from the builder footprint cross-section.")]
        private float soleThickness = 0f;
        [SerializeField, Tooltip("Ankle anchor offset (m, FORWARD along the foot): distance from the ankle to the sole box center. Positive = the foot extends toe-ward from the ankle (heel behind it).")]
        private float soleAnchorOffset = 0.01f;

        [Header("Sole visuals")]
        [SerializeField, Tooltip("Spawn a visual child cube sized/matched to the sole box (same pattern as the rig builder's limb renderers).")]
        private bool soleRenderer = true;
        [SerializeField, Tooltip("Material for the sole visual. Null = builder-passed material (limb material); if that is null too, Unity default material.")]
        private Material soleRendererMaterial;

        /// <summary>Builder passes its limb material so soles match limb visuals.
        /// The serialized field overrides it when set.</summary>
        public void SetRendererMaterial(Material mat)
        {
            if (soleRendererMaterial == null) soleRendererMaterial = mat;
        }

        [Header("Readouts")]
        public int LastHits;
        public float LastNormalForce;
        public float LastFrictionForce;
        public float LastBuriedFraction;
        public float LastMuEff;
        public float LastGrip;
        public float LastConeTorque;

        private Box3DWorld _world;
        private Box3DWorld _worldOverride; // passed by the rig builder, if any
        private Box3DBody _body;
        private bool _inserted;        // bridge surgery done
        private Capsule _mover;      // coincident centers = sphere at the tip
        private QueryFilter _filter;
        private float3 _smoothNormal = new float3(0f, 1f, 0f);
        private float _halfLength, _halfWidth; // footprint half-extents, set by Configure
        private float _loadedMass;   // resolved once from the owning rig
        private bool _massResolved;
        private readonly float3[] _polyScratch = new float3[8]; // gizmo: buried polygon (reused, no alloc)
        private int _polyCount;
        private int _stepsSinceContact = SalvageWindow; // arm salvage from the very first step

        // Submersion salvage tuning: probes are re-raised in 0.15 m steps (up to 0.45 m),
        // gated to a short window after the last real contact (swing phases pay nothing).
        private const int SalvageSteps = 0;
        private const int SalvageWindow = 0;

        /// <summary>Runtime geometry set by the rig builder (serialized fields are fallbacks).
        /// tip: bone tip in the body frame; halfLength/halfWidth: footprint half-extents
        /// (long axis = body Y).</summary>
        public void Configure(float3 tip, float halfLength, float halfWidth, float radius)
        {
            tipLocal = tip;
            _halfLength = math.max(1e-3f, halfLength);
            _halfWidth = math.max(1e-3f, halfWidth);
            _mover = new Capsule
            {
                Center1 = tipLocal,
                Center2 = tipLocal, // coincident centers: the capsule degenerates to a sphere
                Radius = math.max(math.max(1e-3f, radius), liftDepth + 0.005f),
            };
        }

        /// <summary>World passed by the rig builder (it resolves the same
        /// instance for joints). Null falls back to Box3DWorld.Instance.</summary>
        public void ConfigureWorld(Box3DWorld world) => _worldOverride = world;

        /// <summary>The mass actually loading this contact — the whole rig, set by the builder.
        /// 0 falls back to lazy rig resolution, then to the own body's mass.</summary>
        public void SetLoadedMass(float kg)
        {
            _loadedMass = math.max(0f, kg);
            _massResolved = _loadedMass > 0f;
        }

        private void Awake()
        {
            _body = GetComponent<Box3DBody>();
            _world = _worldOverride != null ? _worldOverride : Box3DWorld.Instance;
            if (_mover.Radius <= 0f)
            {
                // No Configure call: use the serialized defaults.
                Configure(tipLocal, 0.2f, 0.05f, probeRadius);
            }
            int terrainLayer = LayerMask.NameToLayer(TerrainLayerName);
            if (terrainLayer < 0)
                Debug.LogWarning($"[FootStabilizer:{name}] Layer '{TerrainLayerName}' missing - foot queries will match nothing. Re-import TagManager/project settings.");
            ulong bit = terrainLayer >= 0 ? 1UL << terrainLayer : ulong.MaxValue;
            _filter = new QueryFilter { CategoryBits = bit, MaskBits = bit };
        }

        private void FixedUpdate()
        {
            if (_world == null || !_body || !_body.Body.IsValid || _mover.Radius <= 0f) return;

            // Suspension surgery once, on the first step: shank -slider- bridge
            // -spherical(ankle)- foot with a real sole collider. After this the
            // analytic support path retires - the solver's contact owns support
            // and friction; the slider spring owns impact absorption.
            if (insertSuspension && !_inserted)
            {
                InsertSuspension();
                return;
            }
            if (_inserted) return; // physics-only: joints + solved contact, no analytic forces
            if (Time.frameCount % 300 == 0)
                Debug.LogWarning($"[FootStabilizer:{name}] suspension requested but insert failed - analytic path still running", this);

            float dt = Time.fixedDeltaTime;
            float3 origin = _body.Body.Position;   // box3d body position IS the COM
            quaternion rot = _body.Body.Rotation;
            float3 vel = _body.Body.LinearVelocity;
            float3 angVel = _body.Body.AngularVelocity;

            float mass = ResolveLoadedMass();
            if (mass <= 1e-4f) return;
            float g = math.max(0.01f, math.abs(_world.GravityVector.y));

            // Mover capsule centers are WORLD-space offsets from the origin param (native
            // mover convention). Rotate the tip offset per step so the sphere rides the
            // true bone tip at any attitude; origin stays the body COM (Evaluate's frame).
            var mover = _mover;
            float3 tipWorld = origin + math.mul(rot, (float3)tipLocal);
            mover.Center1 = tipWorld - origin;
            mover.Center2 = tipWorld - origin;
            Span<CollisionPlane> planes = stackalloc CollisionPlane[8];
            Span<MoverHit> hits = stackalloc MoverHit[8];
            int count = _world.World.CollideMover(origin, mover, _filter, planes, hits, float.MaxValue);

            // Submersion salvage: an overlap query dies once the sphere is fully inside a
            // thick terrain slab (fall faster than the overlap window per step). Re-probe
            // with the center raised in steps until it re-overlaps the surface - the
            // footprint math only needs plane normals + deepest points, which stay world-
            // correct from a raised sphere. Only shortly after a recent contact, so swing
            // phases don't pay for it.
            if (count <= 0 && _stepsSinceContact <= SalvageWindow)
            {
                var up = new float3(0f, 1f, 0f);
                for (int k = 1; k <= SalvageSteps && count <= 0; k++)
                {
                    mover.Center1 = mover.Center2 = up * (k * 0.15f);
                    count = _world.World.CollideMover(origin, mover, _filter, planes, hits, float.MaxValue);
                }
                mover.Center1 = mover.Center2 = tipWorld - origin; // restore for next step
            }
            _stepsSinceContact = count > 0 ? 0 : _stepsSinceContact + 1;
            LastHits = count;
            if (count <= 0)
            {
                // Contactless: decay the smoothed normal toward world-up (snow-model parity).
                _smoothNormal = math.normalizesafe(
                    math.lerp(_smoothNormal, new float3(0f, 1f, 0f), math.min(dt * 3f, 1f)),
                    new float3(0f, 1f, 0f));
                LastNormalForce = LastFrictionForce = LastBuriedFraction = 0f;
                _polyCount = 0;
                return;
            }

            // Material constants (snow-model formulas): damping is 2*zeta*sqrt(bearing*mass)
            // — force per velocity, dimensionally exact.
            float bearingForce = math.max(0.2f, bearingFactor) * mass * g;
            float damping = 2f * dampingRatio * math.sqrt(bearingForce * math.max(mass, 1e-3f));
            float maxForce = mass * g * math.max(maxGLoad, 1f);

            // Grip cone: angle between the bone's long axis and the current contact normal.
            // Within the cone the foot grips like a planted sole; past the loss angle it
            // slides. Isotropic mu: both cone axes get the same value (circle, not ellipse).
            float3 axis = math.mul(rot, new float3(0f, 1f, 0f));
            float thetaDeg = math.degrees(math.acos(math.clamp(
                math.abs(math.dot(axis, _smoothNormal)), -1f, 1f)));
            float grip = 1f - math.smoothstep(gripConeAngle, gripLossAngle, thetaDeg);
            float muEff = math.lerp(muFallen, muPlanted, grip);

            var result = FootContactModel.Evaluate(
                planes, hits, count,
                origin, vel, angVel,
                math.mul(rot, new float3(0f, 0f, 1f)),   // bodyUp
                math.mul(rot, new float3(0f, 1f, 0f)),   // bodyForward (bone long axis)
                math.mul(rot, new float3(1f, 0f, 0f)),   // bodyRight
                liftDepth, bearingForce, capacityRamp, damping,
                muEff, muEff,
                _halfLength, _halfWidth, maxForce, dt,
                torqueDamp, centroidFade, mass, (float3)_world.GravityVector,
                ref _smoothNormal, normalSlewDegPerSec, _polyScratch);
            _polyCount = result.PolyCount;

            // Cone-centering spring (spherical-joint limit analog): restorative torque that
            // rotates the bone's long axis toward the contact normal. tau_spring = k*(axis x n):
            // magnitude ~ k*sin(theta) ~ k*theta (small-angle), direction rotates axis -> n.
            // Scaled by grip, so past the loss angle the cone exerts nothing - that fade IS
            // the limit. Damped along the same axis to avoid ringing against the contact
            // damping. Readout in N*m (force units, unlike the per-step force readouts).
            float3 coneAxis = math.cross(axis, _smoothNormal);
            float sinTheta = math.length(coneAxis);
            float3 coneDir = sinTheta > 1e-5f ? coneAxis / sinTheta : new float3(0f, 1f, 0f);
            float3 coneTorque = grip * (coneStiffness * coneAxis
                - coneDamping * math.dot(angVel, coneDir) * coneDir);

            if (result.PlaneCount > 0)
            {
                _body.Body.ApplyLinearImpulse(result.Impulse, origin, wake: true);
                _body.Body.ApplyTorque(result.Torque + coneTorque, wake: true);
            }

            // Force readouts (per-step, not per-second — comparable with the reward terms).
            LastNormalForce = math.length(result.TotalNormalForce) * dt;
            LastFrictionForce = math.length(result.FrictionForce) * dt;
            LastBuriedFraction = result.BuriedFraction;
            LastMuEff = muEff;
            LastGrip = grip;
            LastConeTorque = math.length(coneTorque);
        }

        /// <summary>Visual child cube matching the sole box extents (copied
        /// pattern from the rig builder's limb renderer; collider destroyed -
        /// visual only).</summary>
        private void AddSoleRenderer(GameObject go, Vector3 size)
        {
            var child = GameObject.CreatePrimitive(PrimitiveType.Cube);
            UnityEngine.Object.Destroy(child.GetComponent<Collider>()); // visual only
            child.name = go.name + "_Renderer";
            child.transform.SetParent(go.transform, false);
            child.transform.localPosition = Vector3.zero;
            child.transform.localScale = size;
            var r = child.GetComponent<MeshRenderer>();
            if (soleRendererMaterial != null) r.sharedMaterial = soleRendererMaterial;
            r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.On;
        }

        // --- suspension surgery (first play step) ---------------------------

        /// <summary>Insert the suspension: full topology, no existing joint is
        /// touched. calf -slider(axis = calf Y, limits, spring)- bridge (small
        /// connector mass at the tip) -spherical(ankle spring to neutral)- sole
        /// capsule (the body that actually contacts the ground, sized from the
        /// builder-passed footprint). Landing impact compresses the slider;
        /// sole orientation articulates at the ankle spring.</summary>
        private void InsertSuspension()
        {
            float3 bodyPos = _body.Body.Position;
            quaternion bodyRot = _body.Body.Rotation;
            float3 down = -math.mul(bodyRot, math.up()); // segment long axis, distal direction
            float3 tipW = bodyPos + math.mul(bodyRot, (float3)tipLocal);
            float radius = math.max(0.02f, _halfWidth * 0.8f);
            float capHeight = math.max(_halfLength, radius); // capsule along Y (halved)
            float bridgeOffset = sliderAnchorOffset;

            // Sole dims: exposed values, footprint-derived fallbacks on 0.
            float soleW = soleWidth > 0f ? soleWidth : 2f * _halfWidth;
            float soleL = soleLength > 0f ? soleLength : _halfLength;
            float soleT = soleThickness > 0f ? soleThickness : 2f * radius;

            // Sole spawns and attaches RELATIVE TO THE BRIDGE: the ankle
            // (bridge-sole hinge) sits at the bridge's rest position, and the
            // sole extends forward from it, hanging just below that plane
            // (half thickness + clearance). sliderAnchorOffset shifts the
            // whole ankle assembly distally along the calf.
            float3 bridgePos = tipW + down * bridgeOffset;
            // Foot forward: the RIG ROOT's facing projected perpendicular to
            // the calf axis - NOT the body frame's local Z (bone frames are
            // built -Y-along-bone with arbitrary yaw from ShortestArcDownTo,
            // so local Z can point sideways).
            var rig = GetComponentInParent<Box3DRigBuilder>();
            if (rig == null) Debug.LogWarning($"[FootStabilizer:{name}] no Box3DRigBuilder in parents - shapes will NOT be matrix-registered", this);
            else Debug.Log($"[MatrixReg probe] '{name}' rig='{rig.name}' logToggle={rig.MatrixLogEnabled} rules={rig.CollisionRules.Count}", this); // unconditional one-shot: proves toggle + rules reach THIS component
            bool flatten = rig != null && rig.FlattenEnabled;
            float3 rootFwd = (float3)(rig != null ? rig.transform.forward : Vector3.forward);
            float3 fwd = math.normalizesafe(rootFwd - down * math.dot(rootFwd, down),
                math.mul(bodyRot, math.forward()));
            float3 solePos = bridgePos + fwd * soleAnchorOffset;
               // + down * (soleT * 0.5f + 0.005f);

            // Collision shapes: always the Rig* variants (flagged for the
            // custom-filter callback) and always registered - the builder's
            // RegisterMatrixShape no-ops when no matrix is authored. The
            // stabilizer owns no matrix decisions.
            var bridgeGo = new GameObject($"FootBridge_{name}");
            bridgeGo.SetActive(false);
            bridgeGo.transform.SetPositionAndRotation(bridgePos, quaternion.identity);
            bridgeGo.transform.SetParent(transform, true);
            var bridge = bridgeGo.AddComponent<Box3DBody>();
            bridge.BodyType = Box3DBodyType.Dynamic;
            var bridgeCap = bridgeGo.AddComponent<RigCapsuleShape>();
            SetCapsulePrivate(bridgeCap, radius, capHeight);
            bridgeGo.SetActive(true);
            if (!bridge.Body.IsValid) { Debug.LogError($"[FootStabilizer:{name}] bridge body failed to materialize - keeping analytic support", this); return; }
            rig?.RegisterMatrixShape(bridgeCap, _body);
            SetBoxMass(bridge.Body, bridgeMassKg, 2f * radius, capHeight, 2f * radius);

            // Sole: the contact body, hanging below the bridge at neutral. A
            // BOX oriented orthogonally to the bridge (long axis of the foot
            // runs along the bridge's local Z; bridge is axis-aligned so this
            // is world space at spawn). Size from the builder-passed footprint.
            var soleGo = new GameObject($"FootSole_{name}");
            soleGo.SetActive(false);
            soleGo.transform.SetPositionAndRotation(solePos,
                quaternion.LookRotationSafe(fwd, -down)); // long axis along rig-facing fwd, flat vs the calf axis
            soleGo.transform.SetParent(bridgeGo.transform, true);
            var sole = soleGo.AddComponent<Box3DBody>();
            sole.BodyType = Box3DBodyType.Dynamic;
            var soleBox = soleGo.AddComponent<RigBoxShape>();
            SetBoxShapePrivate(soleBox, new Vector3(soleW, soleT, soleL)); // width, thickness, length
            if (soleRenderer) AddSoleRenderer(soleGo, new Vector3(soleW, soleT, soleL));
            soleGo.SetActive(true);
            if (!sole.Body.IsValid) { Debug.LogError($"[FootStabilizer:{name}] sole body failed to materialize - keeping analytic support", this); return; }
            rig?.RegisterMatrixShape(soleBox, _body);
            SetBoxMass(sole.Body, soleMassKg, soleW, soleT, soleL);

            // Slider calf-bridge: slides along the segment's long axis (body Y),
            // spring-damped toward zero travel, hard limits. Frames coincide at
            // the BRIDGE'S REST POSITION (matches the spawn pose - not the tip).
            // NOTE: prismatic slides along frame X - so the frame maps X onto
            // the axis (unlike FrameFromAxisWorld, which builds Z=axis).
            quaternion slideRot = Quaternion.FromToRotation(Vector3.right, (Vector3)(-down));
            var slideFrames = JointLocalFrames(_body.Body, bridge.Body,
                new B3Transform { Position = bridgePos, Rotation = slideRot });
            var sdef = PrismaticJointDef.Default;
            sdef.EnableSpring = true;
            sdef.Hertz = sliderSpringHertz;
            sdef.DampingRatio = sliderSpringDamping;
            sdef.TargetTranslation = 0f;
            sdef.EnableLimit = true;
            sdef.LowerTranslation = -sliderTravel;
            sdef.UpperTranslation = sliderTravel;
            sdef.Base.BodyIdA = _body.Body.Id;
            sdef.Base.BodyIdB = bridge.Body.Id;
            sdef.Base.CollideConnected = false; // bridge rides off the tip, not inside the volume
            sdef.Base.LocalFrameA = slideFrames.a;
            sdef.Base.LocalFrameB = slideFrames.b;
            var slider = _world.World.CreatePrismaticJoint(sdef);
            if (!slider.IsValid) { Debug.LogError($"[FootStabilizer:{name}] slider creation failed - keeping analytic support", this); return; }

            // Ankle bridge-sole: revolute hinge about the lateral axis -
            // perpendicular to the rig-facing fwd and the calf axis (NOT body
            // local X, whose yaw is arbitrary) - pitch only, with the exposed
            // limits and a spring-damped return to neutral. Passive.
            float3 pitchAxisW = math.normalizesafe(math.cross(-down, fwd), math.mul(bodyRot, math.right()));
            quaternion ankleRot = SplineChainController.FrameFromAxisWorld(pitchAxisW);
            var ankleFrames = JointLocalFrames(bridge.Body, sole.Body,
                new B3Transform { Position = bridgePos, Rotation = ankleRot });
            var adef = RevoluteJointDef.Default;
            adef.EnableSpring = true;
            adef.Hertz = ankleSpringHertz;
            adef.DampingRatio = ankleSpringDamping; // TargetAngle defaults to 0 = spawn pose
            adef.EnableLimit = true;
            adef.LowerAngle = math.radians(ankleLimitMinDeg);
            adef.UpperAngle = math.radians(ankleLimitMaxDeg);
            adef.Base.BodyIdA = bridge.Body.Id;
            adef.Base.BodyIdB = sole.Body.Id;
            adef.Base.CollideConnected = false;
            adef.Base.LocalFrameA = ankleFrames.a;
            adef.Base.LocalFrameB = ankleFrames.b;
            var ankle = _world.World.CreateRevoluteJoint(adef);
            if (!ankle.IsValid) { Debug.LogError($"[FootStabilizer:{name}] ankle creation failed - sole is UNJOINTED", this); return; }

            // Sole-calf filter joint: the sole is two joints from the calf, so
            // no CollideConnected flag on the chain covers that pair - Box3D's
            // suppression is pairwise only. A FilterJoint with collision
            // disabled kills the pair directly, independent of the matrix.
            var fdef = FilterJointDef.Default;
            fdef.Base.BodyIdA = _body.Body.Id;
            fdef.Base.BodyIdB = sole.Body.Id;
            fdef.Base.CollideConnected = false;
            var footFilter = _world.World.CreateFilterJoint(fdef);
            if (!footFilter.IsValid) { Debug.LogError($"[FootStabilizer:{name}] sole-calf filter joint failed", this); return; }

            _inserted = true;
            if (flatten) rig.FlattenChildBodies(); // suspension bodies join the flattened hierarchy
            Debug.Log($"[FootStabilizer:{name}] suspension inserted: bridge {bridgeMassKg:F1} kg + sole {soleMassKg:F1} kg (box {soleW:F3}x{soleT:F3}x{soleL:F3}), slider \u00b1{sliderTravel * 100f:F0} cm @ {sliderSpringHertz:F0} Hz, ankle hinge [{ankleLimitMinDeg:F0}\u00b0, {ankleLimitMaxDeg:F0}\u00b0] @ {ankleSpringHertz:F0} Hz", this);
        }

        /// <summary>Box3DBoxShape's Size is private serialized (builder parity:
        /// reflection, setup-time only).</summary>
        private static void SetBoxShapePrivate(Box3DBoxShape shape, Vector3 size)
        {
            typeof(Box3DBoxShape)
                .GetField("Size", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                .SetValue(shape, size);
        }

        /// <summary>Box-enclosing inertia (builder parity), extents (x, y, z).
        /// Mass authority lives here, not in shape density.</summary>
        private static void SetBoxMass(Body body, float m, float x, float y, float z)
        {
            body.SetMassData(new MassData
            {
                Mass = m,
                Center = float3.zero,
                Inertia = new B3Matrix3
                {
                    Cx = new float3(m / 12f * (y * y + z * z), 0f, 0f),
                    Cy = new float3(0f, m / 12f * (x * x + z * z), 0f),
                    Cz = new float3(0f, 0f, m / 12f * (x * x + y * y)),
                },
            });
        }

        // Box3DCapsuleShape's Radius/Height are private serialized (builder
        // parity: reflection, build-time only - not in the hot loop).
        private static void SetCapsulePrivate(Box3DCapsuleShape shape, float radius, float height)
        {
            var t = typeof(Box3DCapsuleShape);
            t.GetField("Radius", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                .SetValue(shape, radius);
            t.GetField("Height", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                .SetValue(shape, height);
        }

        /// <summary>Builder-parity: one world frame expressed in both bodies'
        /// local frames (positions from COM, rotations conjugated).</summary>
        private static (B3Transform a, B3Transform b) JointLocalFrames(Body a, Body b, B3Transform worldFrame)
        {
            quaternion qa = a.GetRotation(), qb = b.GetRotation();
            var la = new B3Transform
            {
                Position = math.mul(math.inverse(qa), (float3)(worldFrame.Position - a.GetPosition())),
                Rotation = math.mul(math.inverse(qa), worldFrame.Rotation),
            };
            var lb = new B3Transform
            {
                Position = math.mul(math.inverse(qb), (float3)(worldFrame.Position - b.GetPosition())),
                Rotation = math.mul(math.inverse(qb), worldFrame.Rotation),
            };
            return (la, lb);
        }

        private void OnDrawGizmosSelected()
        {
            float radius = math.max(_mover.Radius, Application.isPlaying ? 0f : probeRadius);
            if (radius <= 0f) radius = probeRadius;
            // No TransformPoint: the bone GO is scaled (thickness, length, thickness) and
            // TransformPoint would apply the scale to the tip offset. Rotation only.
            float3 tip = transform.position + (Vector3)(math.mul(transform.rotation, (float3)tipLocal));

            if (!Application.isPlaying || _mover.Radius <= 0f)
            {
                // Edit-time: just the query sphere at the bone tip.
                Gizmos.color = new Color(1f, 0.6f, 0f);
                Gizmos.DrawWireSphere(tip, radius);
                return;
            }

            // Grip-colored sphere: green = planted (full mu), red = fallen (no grip).
            Gizmos.color = Color.Lerp(Color.red, Color.green, LastGrip);
            Gizmos.DrawSphere(tip, radius);

            // Contact normal (slew-limited) from the tip.
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(tip, _smoothNormal * 0.25f);

            // Buried footprint polygon (last evaluated): the shape the forces were
            // integrated over. Nothing drawn while airborne (poly empty).
            if (_polyCount >= 3)
            {
                Gizmos.color = Color.yellow;
                for (int i = 0; i < _polyCount; i++)
                    Gizmos.DrawLine(_polyScratch[i], _polyScratch[(i + 1) % _polyCount]);
            }
        }

        /// <summary>Loaded mass: explicit override, else one lazy walk of the owning rig
        /// (all dynamic bodies under the PinnedBipedDemo — the foot carries the whole rig
        /// through the joints). Falls back to the own body's mass.</summary>
        private float ResolveLoadedMass()
        {
            if (_massResolved) return _loadedMass;
            _massResolved = true;
            _loadedMass = 0f;
            var demo = GetComponentInParent<PinnedBipedDemo>();
            if (demo != null)
            {
                var bodies = demo.GetComponentsInChildren<Box3DBody>(false);
                for (int i = 0; i < bodies.Length; i++)
                {
                    if (bodies[i] && bodies[i].Body.IsValid && bodies[i].BodyType == Box3DBodyType.Dynamic)
                        _loadedMass += bodies[i].Body.GetMass();
                }
            }
            if (_loadedMass <= 1e-4f) _loadedMass = _body.Body.GetMass();
            return _loadedMass;
        }
    }
}
