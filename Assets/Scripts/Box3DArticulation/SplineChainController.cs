using System;
using System.Collections.Generic;
using Box3D;
using Box3D.Hybrid;
using Unity.Mathematics;
using UnityEngine;
using Joint = Box3D.Joint;
namespace Box3DArticulation
{
    [Serializable]
    public struct JointConfig
    {
        public bool isHinge;
        public Vector3 hingeAxisLocal;          // Zero = spherical
        [Min(0f)] public float hertz;
        [Min(0f)] public float dampingRatio;
        [Min(0f)] public float limitDegrees;    // 0 = no limit (hinge only)
        [Min(0f)] public float torqueLimit;
    }

    /// <summary>
    /// Multiple open spline rails control multiple bone chains. Each chain has its own root body
    /// (receives God force), bones, dedicated rail, and joint config. Chains may share bodies
    /// (e.g., upperBody is bone of spine chain AND root of arm chains).
    /// </summary>
    [DisallowMultipleComponent]
    [ExecuteAlways]
    public sealed class SplineChainController : MonoBehaviour
    {
        [Header("Chains")]
        [SerializeField] private List<Chain> chains = new();

        /// <summary>All chains (read-only for debug/tracking).</summary>
        public IReadOnlyList<Chain> Chains => chains;

        /// <summary>Desired torso rotation computed in ApplyGodForces (for tracking).</summary>
        public Quaternion DesiredTorsoRotation { get; private set; }
        [SerializeField, Min(2)] private int samplesPerSegment = 8;

        [Header("Drive probe (diagnostics)")]
        [SerializeField] private bool probeLogging = false;
        [SerializeField, Min(0.05f)] private float probeLogInterval = 0.5f;
        private float probeLogTimer;
        private System.IO.StreamWriter probeWriter;

        // Contract probe (audit D7): runs on chain[1]'s two joints. Phase 0
        // captures relZ at rest; phase 1 commands RAW +kMotorVel about frame Z
        // for 1s; the relZ delta reveals the engine's motor-sign convention.
        [SerializeField] private bool contractProbe = false;
        private int contractPhase;
        private float contractT;
        private float contractLastLog;
        private readonly float[] contractRelZ0 = new float[8];
        private const float ContractMotorVel = 3f;

        [Header("Torso-only god controller")]
        [SerializeField] private bool disableGravity = true;
        [SerializeField] private bool disableJointLimits = false;
        [SerializeField, Min(0f)] private float positionGain = 800f;
        [SerializeField, Min(0f)] private float velocityGain = 400f;
        [SerializeField, Min(0f)] private float maxForce = 5000f;
        [SerializeField, Min(0f)] private float orientationGain = 4000f;
        [SerializeField, Min(0f)] private float angularVelocityGain = 1600f;
        [SerializeField, Min(0f)] private float maxTorque = 5000f;
        [SerializeField, Min(0f)] private float jointMotorTorque = 5000f;

        [Header("VerticalWave Feedforward (auto-detected from SplineAnimator)")]
        [SerializeField, Min(0f)] private float waveAmplitude = 1.5f;
        [SerializeField, Min(0f)] private float waveFrequency = 0.5f;
        [SerializeField, Min(0f)] private float waveSpeed = 1f;
        private SplineAnimator splineAnimator;

        [Header("Debug")]
        [SerializeField] private bool drawRail = true;
        [SerializeField] private bool drawTargets = true;

        /// <summary>Sample point on the spline rail with arc-length parameter, position, and tangent.</summary>
        public struct RailSample { public float s; public float3 position; public float3 tangent; }
        
        /// <summary>
        /// Generic bone chain joint data for physics-driven IK.
        /// Supports arbitrary number of bones with spherical/hinge joints.
        /// </summary>
        internal struct BoneJoint
        {
            public JointId jointId;                // Box3D joint id (spherical or revolute)
            public B3Transform frameA, frameB;     // Joint local frames (body A, body B)
            public bool isHinge;                   // true = revolute (hinge), false = spherical
            public float3 hingeAxisLocal;          // Hinge axis in body A local space (if isHinge)
            public float3 boneAxisLocal;           // Bone direction in body A local space (at creation)
            public float hertz, damping;           // Motor tuning parameters
            public float lowerLimit, upperLimit;   // Hinge limits (radians)
            public quaternion previousTarget;      // Previous target relative pose
            public bool hasPreviousTarget;
            public float3 previousMotorVelocity;    // Previous motor velocity (for inter-frame smoothing)
        }
        
        /// <summary>
        /// Reusable damped rail coordinate tracker.
        /// Spring-damper system for smoothly tracking a target rail parameter s.
        /// </summary>
        public struct DampedRailTracker
        {
            public float currentS;          // Current rail coordinate
            public float velocityS;         // Current velocity along rail (s/sec)
            public float hertz;             // Spring frequency (Hz)
            public float dampingRatio;      // Damping ratio (1 = critical)
            public float maxSpeed;          // Maximum speed along rail (s/sec)
            public bool initialized;        // Whether tracker has been initialized

            /// <summary>Initialize with starting position and spring parameters.</summary>
            public void Initialize(float startS, float hertz, float dampingRatio, float maxSpeed)
            {
                currentS = startS;
                velocityS = 0f;
                this.hertz = math.max(0f, hertz);
                this.dampingRatio = math.max(0f, dampingRatio);
                this.maxSpeed = math.max(0f, maxSpeed);
                initialized = true;
            }

            /// <summary>Initialize with current values (preserve velocity).</summary>
            public void Reinitialize(float startS, float hertz, float dampingRatio, float maxSpeed)
            {
                currentS = startS;
                this.hertz = math.max(0f, hertz);
                this.dampingRatio = math.max(0f, dampingRatio);
                this.maxSpeed = math.max(0f, maxSpeed);
                initialized = true;
            }

            /// <summary>Step the tracker toward targetS with spring-damper dynamics.</summary>
            public float Step(float targetS, float dt, float railLength)
            {
                if (!initialized) { currentS = targetS; initialized = true; return currentS; }
                if (dt <= 0f) return currentS;
                if (hertz <= 0f) { currentS = targetS; velocityS = 0f; return currentS; }

                // Spring-damper: x'' + 2*zeta*omega*x' + omega^2*x = omega^2*target
                // Discretized with semi-implicit Euler (stable for stiff springs)
                float omega = 2f * math.PI * hertz;
                float omega2 = omega * omega;
                float twoZetaOmega = 2f * dampingRatio * omega;

                float error = targetS - currentS;
                // Wrap error for circular rails if needed (not needed for open spline)

                // Semi-implicit Euler integration
                float accel = omega2 * error - twoZetaOmega * velocityS;
                velocityS += accel * dt;

                // Clamp velocity
                if (maxSpeed > 0f)
                    velocityS = math.clamp(velocityS, -maxSpeed, maxSpeed);

                currentS += velocityS * dt;

                // Clamp to rail bounds
                currentS = math.clamp(currentS, 0f, railLength);

                // If very close to target and low velocity, snap to avoid oscillation
                if (math.abs(error) < 1e-4f && math.abs(velocityS) < 1e-3f)
                {
                    currentS = targetS;
                    velocityS = 0f;
                }

                return currentS;
            }

            /// <summary>Step without clamping to rail bounds (for internal use).</summary>
            public float StepUnclamped(float targetS, float dt)
            {
                if (!initialized) { currentS = targetS; initialized = true; return currentS; }
                if (dt <= 0f) return currentS;
                if (hertz <= 0f) { currentS = targetS; velocityS = 0f; return currentS; }

                float omega = 2f * math.PI * hertz;
                float omega2 = omega * omega;
                float twoZetaOmega = 2f * dampingRatio * omega;

                float error = targetS - currentS;
                float accel = omega2 * error - twoZetaOmega * velocityS;
                velocityS += accel * dt;

                if (maxSpeed > 0f)
                    velocityS = math.clamp(velocityS, -maxSpeed, maxSpeed);

                currentS += velocityS * dt;

                if (math.abs(error) < 1e-4f && math.abs(velocityS) < 1e-3f)
                {
                    currentS = targetS;
                    velocityS = 0f;
                }

                return currentS;
            }

            /// <summary>Reset velocity (e.g., on teleport or large correction).</summary>
            public void ResetVelocity() => velocityS = 0f;

            /// <summary>Hard set position (e.g., initialization or forced sync).</summary>
            public void SetPosition(float s) { currentS = s; velocityS = 0f; initialized = true; }
        }

        /// <summary>
        /// Reusable damped 3D vector tracker.
        /// Spring-damper system for smoothly tracking a target 3D position.
        /// </summary>
        public struct DampedVector3Tracker
        {
            public float3 current;          // Current position
            public float3 velocity;         // Current velocity
            public float hertz;             // Spring frequency (Hz)
            public float dampingRatio;      // Damping ratio (1 = critical)
            public float maxSpeed;          // Maximum speed (units/sec)
            public bool initialized;        // Whether tracker has been initialized

            /// <summary>Initialize with starting position and spring parameters.</summary>
            public void Initialize(float3 start, float hertz, float dampingRatio, float maxSpeed)
            {
                current = start;
                velocity = float3.zero;
                this.hertz = math.max(0f, hertz);
                this.dampingRatio = math.max(0f, dampingRatio);
                this.maxSpeed = math.max(0f, maxSpeed);
                initialized = true;
            }

            /// <summary>Step the tracker toward target with spring-damper dynamics.</summary>
            public float3 Step(float3 target, float dt)
            {
                if (!initialized) { current = target; initialized = true; return current; }
                if (dt <= 0f) return current;
                if (hertz <= 0f) { current = target; velocity = float3.zero; return current; }

                float omega = 2f * math.PI * hertz;
                float omega2 = omega * omega;
                float twoZetaOmega = 2f * dampingRatio * omega;

                float3 error = target - current;
                float3 accel = omega2 * error - twoZetaOmega * velocity;
                velocity += accel * dt;

                if (maxSpeed > 0f)
                {
                    float speed = math.length(velocity);
                    if (speed > maxSpeed) velocity *= maxSpeed / speed;
                }

                current += velocity * dt;

                if (math.lengthsq(error) < 1e-8f && math.lengthsq(velocity) < 1e-6f)
                {
                    current = target;
                    velocity = float3.zero;
                }

                return current;
            }

            /// <summary>Reset velocity.</summary>
            public void ResetVelocity() => velocity = float3.zero;

            /// <summary>Hard set position.</summary>
            public void SetPosition(float3 pos) { current = pos; velocity = float3.zero; initialized = true; }
        }

        /// <summary>
        /// Reusable damped quaternion tracker.
        /// Spring-damper system for smoothly tracking a target rotation.
        /// </summary>
        public struct DampedQuaternionTracker
        {
            public quaternion current;      // Current rotation
            public float3 angularVelocity;  // Current angular velocity (axis * speed)
            public float hertz;             // Spring frequency (Hz)
            public float dampingRatio;      // Damping ratio (1 = critical)
            public float maxAngularSpeed;   // Maximum angular speed (rad/sec)
            public bool initialized;        // Whether tracker has been initialized

            /// <summary>Initialize with starting rotation and spring parameters.</summary>
            public void Initialize(quaternion start, float hertz, float dampingRatio, float maxAngularSpeed)
            {
                current = start;
                angularVelocity = float3.zero;
                this.hertz = math.max(0f, hertz);
                this.dampingRatio = math.max(0f, dampingRatio);
                this.maxAngularSpeed = math.max(0f, maxAngularSpeed);
                initialized = true;
            }

            /// <summary>Step the tracker toward target rotation with spring-damper dynamics.</summary>
            public quaternion Step(quaternion target, float dt)
            {
                if (!initialized) { current = target; initialized = true; return current; }
                if (dt <= 0f) return current;
                if (hertz <= 0f) { current = target; angularVelocity = float3.zero; return current; }

                // Compute rotation error as axis-angle
                quaternion errorRot = math.mul(target, math.inverse(current));
                float3 errorAxis;
                float errorAngle;
                // Ensure shortest path
                if (errorRot.value.w < 0f) errorRot.value = -errorRot.value;
                errorAngle = 2f * math.acos(math.clamp(errorRot.value.w, -1f, 1f));
                if (errorAngle < 1e-6f)
                {
                    errorAxis = float3.zero;
                }
                else
                {
                    errorAxis = math.normalize(errorRot.value.xyz);
                }

                float omega = 2f * math.PI * hertz;
                float omega2 = omega * omega;
                float twoZetaOmega = 2f * dampingRatio * omega;

                // Angular acceleration = omega^2 * error - 2*zeta*omega * angularVelocity
                float3 errorVec = errorAxis * errorAngle;
                float3 accel = omega2 * errorVec - twoZetaOmega * angularVelocity;
                angularVelocity += accel * dt;

                if (maxAngularSpeed > 0f)
                {
                    float speed = math.length(angularVelocity);
                    if (speed > maxAngularSpeed) angularVelocity *= maxAngularSpeed / speed;
                }

                // Integrate rotation
                float3 deltaAngle = angularVelocity * dt;
                float deltaMag = math.length(deltaAngle);
                if (deltaMag > 1e-6f)
                {
                    quaternion deltaRot = quaternion.AxisAngle(math.normalize(deltaAngle), deltaMag);
                    current = math.mul(deltaRot, current);
                }

                if (errorAngle < 1e-4f && math.lengthsq(angularVelocity) < 1e-6f)
                {
                    current = target;
                    angularVelocity = float3.zero;
                }

                return current;
            }

            /// <summary>Reset angular velocity.</summary>
            public void ResetVelocity() => angularVelocity = float3.zero;

            /// <summary>Hard set rotation.</summary>
            public void SetRotation(quaternion rot) { current = rot; angularVelocity = float3.zero; initialized = true; }
        }

        /// <summary>
        /// Runtime state for a single chain (spine or limb).
        /// </summary>
        private struct ChainRuntime
        {
            public Box3DBody rootBody;
            public Box3DBody[] bones;
            public BoneJoint[] joints;             // All joints in chain (root to tip)
            public float3[] localAnchors;          // Local anchor of each joint on its parent body
            public float[] boneLengths;            // Length of each segment: joint-to-joint chords + final anchor-to-tip segment
            public float[] jointS;                 // Fixed rail-local s for root + each joint anchor (NO tip — it's extrapolated off-rail)
            public float rootStationS;             // Fixed rail-local station for the root (set at rebuild)
            public RailSample[] rail;              // Cached rail samples for this chain
            public float railLength;
            public float[] probeErrDeg;            // Per-joint pose error (deg), probe only
            public float3[] probeTargetRvDeg;      // Commanded target rotation vector (deg), probe only
            public float3[] probeActualRvDeg;      // Actual rotation vector (deg), probe only
            public float[] probePoseErrDeg;        // World-space bone-vs-pose-segment angle (deg), probe only
            public Body[][] distalBodies;          // Per joint: bodies beyond it (composite inertia for the drive)
        }
        
        /// <summary>
        /// Configuration for a chain (spine or limb).
        /// </summary>
        [Serializable]
        public struct Chain
        {
            public Box3DBody rootBody;              // God force applied here
            public Box3DBody[] bones;               // Distal bones in order (rootBody not included)
            public Transform[] railControlPoints;   // Control points for this chain's rail
            public JointConfig[] joints;            // Per-joint config (same length as bones)
            [Tooltip("Rest-pose root anchor in rootBody local space (e.g. the shoulder on the chest). Zero = root body center. Defines joint 0's anchor and the root rail station.")]
            public float3 rootAnchorLocal;          // Root anchor on rootBody (zero = body center)
            [Min(0f)] public float slideSearchDistance; // FABRIK temporal-coherence window
            [Min(0f)] public float maxSlideSpeed;       // Root projection rate limit (rail units/s)
        }

        private readonly List<Joint> nativeJoints = new List<Joint>();
        private readonly List<Body> bodies = new List<Body>();
        private ChainRuntime[] runtime;
        private Box3DWorld world;

        private float[] godDesiredS;

        // Torso dampened trackers
        private DampedVector3Tracker torsoPositionTracker;
        private DampedQuaternionTracker torsoRotationTracker;
        private bool built;
        // Feedforward: rail center velocity/acceleration tracking
        private float3 prevRailCenter;
        private float3 prevRailVelocity;
        private float torsoMass;

        // Debug data exposure
        [NonSerialized] private LimbDebugData[] debugData;

        /// <summary>
        /// Debug data for a single limb chain: actual vs target joint positions.
        /// </summary>
        public struct LimbDebugData
        {
            public int limbIndex;
            public int jointCount;
            public float3[] actualJointPositions;    // World positions of joint anchors on bodies
            public float3[] targetJointPositions;    // Rail-sampled target positions
            public float[] deviations;               // Distance between actual and target
            public float3[] deviationVectors;        // Target - actual vectors
            public float[] jointSValues;             // Current rail coordinates
        }

        /// <summary>
        /// Get read-only access to rail samples for debug rendering.
        /// Returns the first arm chain's rail (index 1) if present, else the first chain's rail.
        /// </summary>
        private static readonly Color[] ChainColorPalette =
        {
            new Color(0.2f, 1f, 0.3f),   // 0: spine
            new Color(1f, 0.45f, 0.1f),  // 1: left arm
            new Color(0.25f, 0.55f, 1f), // 2: right arm
            new Color(1f, 0.9f, 0.2f),   // 3+: wraps
        };

        /// <summary>
        /// Shared per-chain color (index-stable). Used by all debug/visualization drawing.
        /// </summary>
        public static Color GetChainColor(int index) => ChainColorPalette[index % ChainColorPalette.Length];

        public IReadOnlyList<RailSample> RailSamples
        {
            get
            {
                int idx = (runtime != null && runtime.Length > 1 && runtime[1].rail != null) ? 1 : 0;
                return (runtime != null && runtime.Length > idx && runtime[idx].rail != null) ? runtime[idx].rail : Array.Empty<RailSample>();
            }
        }

        /// <summary>Compatibility: shoulder-carrying body = last bone of the spine chain (chain 0).</summary>
        public Box3DBody TorsoBody
        {
            get
            {
                if (chains.Count == 0 || chains[0].bones == null || chains[0].bones.Length == 0) return null;
                return chains[0].bones[chains[0].bones.Length - 1];
            }
        }

        /// <summary>Compatibility shim for debug monitor.</summary>
        public Box3DBody Torso => TorsoBody;

        /// <summary>Consecutive bone pairs from arm chains (index >= 1) for debug renderers.</summary>
        public struct BonePair { public Box3DBody upperBody; public Box3DBody lowerBody; public float upperLength; public float lowerLength; }

        public List<BonePair> Limbs
        {
            get
            {
                var result = new List<BonePair>();
                for (int i = 1; i < chains.Count; i++)
                {
                    var bones = chains[i].bones;
                    if (bones == null) continue;
                    for (int b = 0; b + 1 < bones.Length; b++)
                    {
                        float upperLen = 0f, lowerLen = 0f;
                        if (bones[b] && bones[b + 1] && bones[b].Body.IsValid && bones[b + 1].Body.IsValid)
                            upperLen = math.distance(bones[b].Body.GetPosition(), bones[b + 1].Body.GetPosition());
                        result.Add(new BonePair { upperBody = bones[b], lowerBody = bones[b + 1], upperLength = upperLen, lowerLength = upperLen });
                    }
                }
                return result;
            }
        }

        /// <summary>
        /// Get read-only access to physics bodies for debug rendering.
        /// </summary>
        public IReadOnlyList<Body> PhysicsBodies => bodies;

        /// <summary>
        /// Get debug data for all chains (actual vs target joint positions).
        /// Call after FixedUpdate to get latest data.
        /// </summary>
        public LimbDebugData[] GetDebugData()
        {
            if (!built || runtime == null) return Array.Empty<LimbDebugData>();
            
            if (debugData == null || debugData.Length != runtime.Length)
                debugData = new LimbDebugData[runtime.Length];
            
            for (int i = 0; i < runtime.Length; i++)
            {
                var state = runtime[i];
                var chain = chains[i];
                int pointCount = state.jointS.Length + 1; // root + joint anchors + extrapolated tip
                
                debugData[i].limbIndex = i;
                debugData[i].jointCount = pointCount;
                debugData[i].actualJointPositions = new float3[pointCount];
                debugData[i].targetJointPositions = new float3[pointCount];
                debugData[i].deviations = new float[pointCount];
                debugData[i].deviationVectors = new float3[pointCount];
                debugData[i].jointSValues = new float[pointCount];
                
                // Root point: LIVE root anchor on the root body (shoulder for arms,
                // body center for the spine) — never the frozen rebuild-time position.
                debugData[i].actualJointPositions[0] = (float3)chain.rootBody.Body.GetPosition()
                    + math.mul(chain.rootBody.Body.GetRotation(), chain.rootAnchorLocal);
                debugData[i].targetJointPositions[0] = SampleOnRail(state.jointS[0], state.rail, state.railLength).position;
                debugData[i].jointSValues[0] = state.jointS[0];
                
                // Middle points: CURRENT joint anchor world positions from Box3D
                for (int j = 0; j < state.joints.Length; j++)
                {
                    var boneJoint = state.joints[j];
                    Box3DBody bodyA = (j == 0) ? chain.rootBody : chain.bones[j - 1];
                    float3 actualAnchor;
                    if (bodyA && bodyA.Body.IsValid)
                    {
                        actualAnchor = (float3)bodyA.Body.GetPosition() + math.mul(bodyA.Body.GetRotation(), boneJoint.frameA.Position);
                    }
                    else
                    {
                        actualAnchor = (float3)bodyA.transform.TransformPoint((Vector3)state.localAnchors[j]);
                    }
                    debugData[i].actualJointPositions[j + 1] = actualAnchor;
                    
                    // Target from rail — SampleOnRail clamps s into [0, railLength]
                    float targetS = state.jointS[j + 1];
                    debugData[i].targetJointPositions[j + 1] = SampleOnRail(targetS, state.rail, state.railLength).position;
                    debugData[i].jointSValues[j + 1] = targetS;
                }
                
                // Tip point: extrapolated far end of the last bone (e.g. the hand).
                // Off-rail by design — same straight continuation as the drive target.
                int last = pointCount - 1;
                float3 lastBodyPos = (float3)chain.bones[chain.bones.Length - 1].Body.GetPosition();
                debugData[i].actualJointPositions[last] = 2f * lastBodyPos - debugData[i].actualJointPositions[last - 1];
                debugData[i].targetJointPositions[last] = ComputeTipTarget(ref state);
                debugData[i].jointSValues[last] = state.jointS[last - 1]; // tip has no rail s; report the wrist's
                
                // Compute deviations
                for (int j = 0; j < pointCount; j++)
                {
                    float3 diff = debugData[i].targetJointPositions[j] - debugData[i].actualJointPositions[j];
                    debugData[i].deviationVectors[j] = diff;
                    debugData[i].deviations[j] = math.length(diff);
                }
            }
            return debugData;
        }

        private void OnEnable()
        {
            if (!built || runtime == null)
            {
                if (world && chains.Count > 0)
                    Rebuild();
            }
        }

        private void Start() => Rebuild();

        private void FixedUpdate()
        {
            if (!built) return;
            CacheRails(); // Build rail per chain
            bool anyValid = false;
            foreach (var state in runtime) {
                if (state.railLength > 1e-4f) { anyValid = true; break; }
            }
            if (!anyValid) return;
            //ApplyGodForces();
            DriveArticulation();
        }

        [ContextMenu("Rebuild")]
        public void Rebuild()
        {
            DestroyJoints();
            built = false;
            world = Box3DWorld.Instance;
            Debug.Log(
                $"World: {world}, Valid:{world.World.IsValid}\nChains:{chains.Count}");
            if (!world || !world.World.IsValid || chains.Count == 0)
                return;

            runtime = new ChainRuntime[chains.Count];
            bodies.Clear();
            
            // Validate all chains and collect bodies
            for (int i = 0; i < chains.Count; i++)
            {
                var chain = chains[i];
                if (chain.rootBody == null || !chain.rootBody.Body.IsValid)
                {
                    Debug.LogError($"{name}: Chain {i} rootBody is invalid.", this);
                    return;
                }
                if (chain.bones == null || chain.bones.Length == 0)
                {
                    Debug.LogError($"{name}: Chain {i} needs at least 1 bone.", this);
                    return;
                }
                if (chain.railControlPoints == null || chain.railControlPoints.Length < 2)
                {
                    Debug.LogError($"{name}: Chain {i} needs at least 2 rail control points.", this);
                    return;
                }
                if (chain.joints == null || chain.joints.Length != chain.bones.Length)
                {
                    Debug.LogError($"{name}: Chain {i} joints array length must match bones length.", this);
                    return;
                }
                
                bodies.Add(chain.rootBody.Body);
                for (int b = 0; b < chain.bones.Length; b++)
                {
                    if (!chain.bones[b] || !chain.bones[b].Body.IsValid)
                    {
                        Debug.LogError($"{name}: Chain {i} bone {b} is invalid.", this);
                        return;
                    }
                    bodies.Add(chain.bones[b].Body);
                }
            }

            if (disableGravity)
                foreach (Body body in bodies) body.SetGravityScale(0f);

            // Get SplineAnimator for VerticalWave parameters
            splineAnimator = GetComponent<SplineAnimator>();
            if (splineAnimator != null)
            {
                var field = typeof(SplineAnimator).GetField("waveAmplitude", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
                if (field != null) waveAmplitude = (float)field.GetValue(splineAnimator);
                field = typeof(SplineAnimator).GetField("waveFrequency", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
                if (field != null) waveFrequency = (float)field.GetValue(splineAnimator);
                field = typeof(SplineAnimator).GetField("waveSpeed", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
                if (field != null) waveSpeed = (float)field.GetValue(splineAnimator);
            }

            // Cache per-chain rails (shared, never flipped), then build joints.
            CacheRails();
            for (int i = 0; i < chains.Count; i++) CreateChainJoints(i);
            built = nativeJoints.Count > 0 && nativeJoints.TrueForAll(joint => joint.IsValid);
            if (!built) Debug.LogError($"{name}: Box3D joint creation failed.", this);
        }

        private void CacheRails()
        {
            // Build each chain's rail from its control points. Rails are shared
            // geometry in ONE coordinate system — never flipped. Limb chains that
            // share a spline get identical copies; sides are handled by per-chain
            // direction signs + branch bounds in CreateChainJoints.
            for (int i = 0; i < chains.Count; i++)
            {
                var railSamples = BuildRail(chains[i].railControlPoints);
                runtime[i].rail = railSamples;
                runtime[i].railLength = railSamples.Length > 0 ? railSamples[railSamples.Length - 1].s : 0f;
            }
        }


        private RailSample[] BuildRail(Transform[] controlPoints)
        {
            var rail = new List<RailSample>();
            float railLength = 0f;
            if (controlPoints == null || controlPoints.Length < 2) return rail.ToArray();

            Transform first = controlPoints[0];
            if (!first) return rail.ToArray();

            // For Catmull-Rom we need at least 4 points. If fewer, fall back to linear.
            bool useCatmullRom = controlPoints.Length >= 4;

            if (useCatmullRom)
            {
                // Precompute all control point positions
                int n = controlPoints.Length;
                float3[] points = new float3[n];
                for (int i = 0; i < n; i++)
                    points[i] = controlPoints[i] ? (float3)controlPoints[i].position : float3.zero;

                // Add first point
                rail.Add(new RailSample
                {
                    s = 0f, position = points[0], tangent = math.normalizesafe(points[1] - points[0], math.right())
                });

                // Sample each segment using Catmull-Rom
                for (int segment = 0; segment < n - 1; segment++)
                {
                    // Get 4 control points for this segment
                    float3 p0 = points[math.max(0, segment - 1)];
                    float3 p1 = points[segment];
                    float3 p2 = points[segment + 1];
                    float3 p3 = points[math.min(n - 1, segment + 2)];

                    float3 previous = p1;
                    for (int step = 1; step <= samplesPerSegment; step++)
                    {
                        float t = step / (float)samplesPerSegment;
                        float3 point = CatmullRom(p0, p1, p2, p3, t);
                        float distance = math.length(point - previous);
                        if (distance <= 1e-5f) continue;
                        railLength += distance;
                        float3 tangent = CatmullRomTangent(p0, p1, p2, p3, t);
                        rail.Add(new RailSample
                        {
                            s = railLength, position = point, tangent = math.normalizesafe(tangent, math.forward())
                        });
                        previous = point;
                    }
                }
            }
            else
            {
                // Linear fallback for < 4 points
                float3 previous = (float3)first.position;
                rail.Add(new RailSample { s = 0f, position = previous, tangent = math.right() });
                for (int segment = 0; segment < controlPoints.Length - 1; segment++)
                {
                    Transform a = controlPoints[segment];
                    Transform b = controlPoints[segment + 1];
                    if (!a || !b) continue;
                    for (int step = 1; step <= samplesPerSegment; step++)
                    {
                        float3 point = math.lerp((float3)a.position, (float3)b.position,
                            step / (float)samplesPerSegment);
                        float distance = math.length(point - previous);
                        if (distance <= 1e-5f) continue;
                        railLength += distance;
                        rail.Add(new RailSample
                        {
                            s = railLength, position = point,
                            tangent = math.normalizesafe(point - previous, math.forward())
                        });
                        previous = point;
                    }
                }
            }

            if (rail.Count > 1)
            {
                RailSample start = rail[0];
                start.tangent = rail[1].tangent;
                rail[0] = start;
            }

            return rail.ToArray();
        }

        // Catmull-Rom spline evaluation (uniform parameterization)
        private static float3 CatmullRom(float3 p0, float3 p1, float3 p2, float3 p3, float t)
        {
            float t2 = t * t;
            float t3 = t2 * t;
            return 0.5f * (
                2f * p1 +
                (-p0 + p2) * t +
                (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2 +
                (-p0 + 3f * p1 - 3f * p2 + p3) * t3
            );
        }

        private static float3 CatmullRomTangent(float3 p0, float3 p1, float3 p2, float3 p3, float t)
        {
            float t2 = t * t;
            return 0.5f * (
                (-p0 + p2) +
                2f * (2f * p0 - 5f * p1 + 4f * p2 - p3) * t +
                3f * (-p0 + 3f * p1 - 3f * p2 + p3) * t2
            );
        }

        private void CreateChainJoints(int index)
        {
            var chain = chains[index];
            var state = new ChainRuntime();
            
            // Build full body chain: rootBody + bones
            int numBones = chain.bones.Length;
            int numJoints = numBones;
            
            state.rootBody = chain.rootBody;
            state.bones = chain.bones;
            // Preserve the rail cached by CacheRails() — the final runtime[index] = state
            // assignment below would otherwise wipe it.
            state.rail = runtime[index].rail;
            state.railLength = runtime[index].railLength;
            state.joints = new BoneJoint[numJoints];
            state.localAnchors = new float3[numJoints];
            // Stations: root + one per joint anchor. NO tip station — the tip is a
            // free end extrapolated straight off the last anchor (never clamped to,
            // or searched along, the rail). Segments: root->J0, ..., Jlast->tip.
            state.boneLengths = new float[numBones + 1];
            state.jointS = new float[numBones + 1];
            state.probeErrDeg = probeLogging ? new float[numJoints] : null;
            state.probeTargetRvDeg = probeLogging ? new float3[numJoints] : null;
            state.probeActualRvDeg = probeLogging ? new float3[numJoints] : null;
            state.probePoseErrDeg = probeLogging ? new float[numJoints] : null;
            // Distal subtree per joint: bodies[j] and beyond - what the joint's
            // motor has to accelerate/brake. Cached once, reused every frame.
            state.distalBodies = new Body[numJoints][];
            for (int j = 0; j < numJoints; j++)
            {
                var distal = new Body[numBones - j];
                for (int i = j; i < numBones; i++) distal[i - j] = chain.bones[i].Body;
                state.distalBodies[j] = distal;
            }
            
            // Skeleton points: J0 at the chain's defined root anchor (e.g. the shoulder
            // on the chest — NOT the midpoint to the arm COM, which slides to the chest's
            // lower half when the arm hangs), Jj = midpoint between adjacent body centers
            // for the rest, tip = extrapolated far end of the last bone.
            float3[] jointPos = new float3[numJoints + 1];
            bool hasRootAnchor = math.lengthsq(chain.rootAnchorLocal) > 1e-8f;
            float3 rootAnchorWorld = hasRootAnchor
                ? (float3)chain.rootBody.Body.GetPosition() + math.mul(chain.rootBody.Body.GetRotation(), chain.rootAnchorLocal)
                : (float3)chain.rootBody.Body.GetPosition();
            {
                float3 prevPos = rootAnchorWorld;
                for (int j = 0; j < numJoints; j++)
                {
                    float3 curPos = (float3)chain.bones[j].Body.GetPosition();
                    jointPos[j] = (j == 0) ? rootAnchorWorld : (prevPos + curPos) * 0.5f;
                    prevPos = curPos;
                }
                // Tip position past the last joint, used for the final chord length.
                jointPos[numJoints] = prevPos + (prevPos - jointPos[numJoints - 1]);
            }
            
            // Create joints along the chain
            for (int j = 0; j < numJoints; j++)
            {
                Box3DBody bodyA = (j == 0) ? chain.rootBody : chain.bones[j - 1];
                Box3DBody bodyB = chain.bones[j];
                var jointConfig = chain.joints[j];
                
                // World position of this joint: midpoint between bodyA and bodyB centers
                float3 anchor = jointPos[j];
                
                state.localAnchors[j] = math.mul(math.inverse(bodyA.Body.GetRotation()), anchor - (float3)bodyA.Body.GetPosition());
                
                // Compute initial bone direction in bodyA local space
                float3 bodyBPos = bodyB.Body.GetPosition();
                float3 boneDirWorld = math.normalizesafe(bodyBPos - anchor, math.forward());
                float3 boneAxisLocal = math.mul(math.inverse(bodyA.Body.GetRotation()), boneDirWorld);
                // Solver chord = joint-to-joint spacing, matching the physical joint layout
                state.boneLengths[j] = math.length(jointPos[j + 1] - jointPos[j]);
                
                bool isHinge = jointConfig.isHinge;
                float3 hingeAxis = isHinge ? math.normalize(jointConfig.hingeAxisLocal) : float3.zero;
                
                if (isHinge)
                {
                    // Hinge joint: Z axis = hinge axis in WORLD space.
                    // FrameFromAxisWorld: LookRotationSafe(axis, up) is degenerate
                    // for near-vertical axes (audit D1).
                    float3 worldHingeAxis = math.mul(bodyA.Body.GetRotation(), hingeAxis);
                    quaternion hingeFrameRot = FrameFromAxisWorld(worldHingeAxis);
                    
                    RevoluteJointDef def = RevoluteJointDef.Default;
                    SetupFrames(ref def.Base, bodyA.Body, bodyB.Body,
                        new B3Transform { Position = anchor, Rotation = hingeFrameRot },
                        out state.joints[j].frameA, out state.joints[j].frameB);
                    
                    def.EnableSpring = false;
                    def.Hertz = jointConfig.hertz > 0f ? jointConfig.hertz : 30f;
                    def.DampingRatio = jointConfig.dampingRatio > 0f ? jointConfig.dampingRatio : 0.7f;
                    
                    if (!disableJointLimits && jointConfig.limitDegrees > 0f)
                    {
                        def.EnableLimit = true;
                        float limit = math.radians(jointConfig.limitDegrees);
                        def.LowerAngle = -limit;
                        def.UpperAngle = limit;
                    }
                    else
                    {
                        def.EnableLimit = false;
                    }
                    
                    state.joints[j].jointId = world.World.CreateRevoluteJoint(def).Id;
                    state.joints[j].isHinge = true;
                    state.joints[j].hingeAxisLocal = hingeAxis;
                    state.joints[j].boneAxisLocal = boneAxisLocal;
                    state.joints[j].hertz = def.Hertz;
                    state.joints[j].damping = def.DampingRatio;
                    state.joints[j].lowerLimit = def.LowerAngle;
                    state.joints[j].upperLimit = def.UpperAngle;
                }
                else
                {
                    // Spherical joint: align joint frame Z with initial bone direction
                    quaternion boneFrameRot = FrameFromAxisWorld(boneDirWorld);
                    
                    SphericalJointDef def = SphericalJointDef.Default;
                    SetupFrames(ref def.Base, bodyA.Body, bodyB.Body,
                        new B3Transform { Position = anchor, Rotation = boneFrameRot },
                        out state.joints[j].frameA, out state.joints[j].frameB);
                    
                    def.EnableSpring = false;
                    def.Hertz = jointConfig.hertz > 0f ? jointConfig.hertz : 200f;
                    def.DampingRatio = jointConfig.dampingRatio > 0f ? jointConfig.dampingRatio : 1.0f;
                    
                    state.joints[j].jointId = world.World.CreateSphericalJoint(def).Id;
                    state.joints[j].isHinge = false;
                    state.joints[j].boneAxisLocal = boneAxisLocal;
                    state.joints[j].hertz = def.Hertz;
                    state.joints[j].damping = def.DampingRatio;
                }
                
                nativeJoints.Add(new Joint { Id = state.joints[j].jointId });
            }
            // Final segment: last joint anchor -> extrapolated tip (e.g. the hand).
            state.boneLengths[numBones] = math.length(jointPos[numBones] - jointPos[numBones - 1]);
            
            // Initialize rail coordinates using FABRIK on this chain's rail.
            // Rails are canonical (s increases root -> tip), so downstream joints
            // always start at higher s than the root.
            var chainRail = runtime[index].rail;
            float railLen = runtime[index].railLength;

            // Initialize rail stations — SEARCHLESS layout. The rail is one shared,
            // never-flipped coordinate system. The split between the two branches is
            // the projection of the root body center, fixed at rebuild ("midpoint").
            // Each chain owns ONE branch: stations are laid out by ADDING chord
            // lengths in the chain's direction, clamped to the branch bounds, so no
            // station can ever cross the crown to the other side.
            float rootS = ProjectOntoRail(rootAnchorWorld, chainRail, railLen);
            float splitS = ProjectOntoRail((float3)chain.rootBody.Body.GetPosition(), chainRail, railLen);
            // Unanchored chains (spine: root = body center = split) march toward +s;
            // anchored chains (arms) march away from the split into their own branch.
            float dir = hasRootAnchor ? (rootS >= splitS ? 1f : -1f) : 1f;
            float minS = dir > 0f ? splitS : 0f;
            float maxS = dir > 0f ? railLen : splitS;

            // Root station is a fixed RAIL-LOCAL s: its world position follows the
            // animated rail (the rig rides the wave), but it can never drift along
            // the rail when the body is pushed.
            state.rootStationS = rootS;
            state.jointS[0] = rootS;

            // Lay out downstream anchor stations by addition — no search, no FABRIK.
            float prevS = rootS;
            for (int j = 0; j < numBones; j++)
            {
                prevS = math.clamp(prevS + dir * state.boneLengths[j], minS, maxS);
                state.jointS[j + 1] = prevS;
            }
            
            runtime[index] = state;
        }

        private float ProjectOntoRail(float3 worldPos, RailSample[] rail, float railLength)
        {
            if (rail == null || rail.Length < 2) return 0f;
            float bestS = 0f;
            float bestDistSq = float.MaxValue;
            for (int i = 0; i < rail.Length - 1; i++)
            {
                float3 a = rail[i].position;
                float3 b = rail[i + 1].position;
                float3 ab = b - a;
                float lenSq = math.lengthsq(ab);
                if (lenSq < 1e-8f) continue;
                float t = math.clamp(math.dot(worldPos - a, ab) / lenSq, 0f, 1f);
                float3 closest = a + ab * t;
                float distSq = math.lengthsq(worldPos - closest);
                if (distSq < bestDistSq)
                {
                    bestDistSq = distSq;
                    bestS = math.lerp(rail[i].s, rail[i + 1].s, t);
                }
            }
            return bestS;
        }

        private void ApplyGodForces()
        {
            int count = chains.Count;
            if (godDesiredS == null || godDesiredS.Length != count) godDesiredS = new float[count];

            // Pass 1: per-chain rail station for the god-force target.
            for (int i = 0; i < count; i++)
            {
                var state = runtime[i];
                if (state.rail == null || state.rail.Length < 2) { godDesiredS[i] = -1f; continue; }

                if (i == 0)
                {
                    // Spine chain: pelvis rides its fixed rail-local station.
                    godDesiredS[i] = state.rootStationS;
                }
                else
                {
                    // Arm chains: chest rides the shoulder station (fixed rail-local s,
                    // captured at rebuild from the rest-pose shoulder anchor projection).
                    godDesiredS[i] = state.rootStationS;
                }
            }

            // Pass 2: ONE PD per distinct root body. Chains sharing a root (both arms on
            // the chest) contribute averaged targets — two independent full-magnitude PDs
            // on the same body double the correction energy and fight each other.
            for (int i = 0; i < count; i++)
            {
                var chain = chains[i];
                var rootBody = chain.rootBody;
                var state = runtime[i];
                if (!rootBody || !rootBody.Body.IsValid || state.rail == null || state.rail.Length < 2) continue;

                // Apply once per group, at the first chain that owns this root body.
                bool isFirstOfGroup = true;
                for (int j = 0; j < i; j++)
                    if (chains[j].rootBody == rootBody) { isFirstOfGroup = false; break; }
                if (!isFirstOfGroup) continue;

                // Average position targets and rotation targets across the group.
                float3 desiredPosition = float3.zero;
                quaternion desiredRotation = quaternion.identity;
                int groupSize = 0;
                for (int j = i; j < count; j++)
                {
                    if (chains[j].rootBody != rootBody) continue;
                    var groupState = runtime[j];
                    if (groupState.rail == null || groupState.rail.Length < 2 || godDesiredS[j] < 0f) continue;
                    RailSample sample = SampleOnRail(godDesiredS[j], groupState.rail, groupState.railLength);
                    // Rails are shared and never flipped — tangents are already in
                    // the original rail direction for every chain.
                    float3 tangent = sample.tangent;
                    desiredPosition += sample.position;
                    // Spine: body up along the rail tangent (pelvis upright on a vertical rail).
                    // Arms: body forward along the tangent (shoulder station orientation).
                    // Degeneracy-safe: spine up along tangent, arms forward along
                    // tangent (audit D2 — LookRotationSafe variants flip when the
                    // vertical rail tangent parallels the hint axis).
                    quaternion rotation = (j == 0)
                        ? RotationUpAlong(tangent)
                        : RotationForwardAlong(tangent);
                    desiredRotation = groupSize == 0
                        ? rotation
                        : math.normalizesafe(math.slerp(desiredRotation, rotation, 1f / (groupSize + 1)));
                    groupSize++;
                }
                if (groupSize == 0) continue;
                desiredPosition /= groupSize;

                // Store for tracking: the chest group (shared root) drives torso orientation.
                if (groupSize > 1) DesiredTorsoRotation = desiredRotation;

                // PD control
                quaternion currentRotation = rootBody.Body.GetRotation();
                float3 currentCom = rootBody.Body.GetWorldCenterOfMass();
                float3 force = ClampMagnitude(positionGain * (desiredPosition - currentCom) - velocityGain * rootBody.Body.GetLinearVelocity(), maxForce);
                float3 torque = ClampMagnitude(orientationGain * RotationVector(math.mul(desiredRotation, math.inverse(currentRotation))) - angularVelocityGain * rootBody.Body.GetAngularVelocity(), maxTorque);

                rootBody.Body.ApplyForceToCenter(force, wake: true);
                rootBody.Body.ApplyTorque(torque, wake: true);
            }
        }

        private static RailSample SampleOnRail(float s, RailSample[] rail, float railLength)
        {
            if (rail == null || rail.Length < 2) return new RailSample();
            s = math.clamp(s, 0f, railLength);
            for (int i = 0; i < rail.Length - 1; i++)
            {
                if (rail[i].s <= s && rail[i + 1].s >= s)
                {
                    float t = (rail[i + 1].s - rail[i].s) > 1e-6f 
                        ? (s - rail[i].s) / (rail[i + 1].s - rail[i].s) 
                        : 0f;
                    float3 pos = math.lerp(rail[i].position, rail[i + 1].position, t);
                    float3 tan = math.normalizesafe(math.lerp(rail[i].tangent, rail[i + 1].tangent, t), math.forward());
                    return new RailSample { s = s, position = pos, tangent = tan };
                }
            }
            return rail[rail.Length - 1];
        }

        private float3[] driveSegDirs;
        private float3[] driveHingeAxes;

        /// <summary>
        /// Measures Box3D's motor-velocity sign convention on the LIVE chain[1]
        /// joints: relZ delta after 1s of raw +ContractMotorVel about frame Z.
        /// DIRECT => delta > 0; NEGATED => delta < 0. Writes
        /// [CONTRACT] verdict lines to console AND Logs/drive-probe.log.
        /// </summary>
        private void StepContractProbe(float dt)
        {
            var state = runtime[1];
            int n = math.min(state.joints.Length, contractRelZ0.Length);

            if (contractPhase == 0)
            {
                contractT += dt;
                if (contractT >= 0.5f)
                {
                    for (int j = 0; j < n; j++) contractRelZ0[j] = JointRelZ(state, j);
                    contractPhase = 1;
                    contractT = 0f;
                    Log($"[CONTRACT] chain[1] settled — commanding +{ContractMotorVel} rad/s about frame Z on {n} joints");
                }
                return;
            }

            if (contractPhase == 1)
            {
                contractT += dt;
                for (int j = 0; j < n; j++)
                {
                    var joint = state.joints[j];
                    float3 cmd = math.forward() * ContractMotorVel;
                    if (joint.isHinge)
                    {
                        var rj = new RevoluteJoint { Id = joint.jointId };
                        rj.EnableSpring(false);
                        rj.EnableMotor(true);
                        rj.SetMotorSpeed(math.dot(cmd, math.forward()));
                        rj.SetMaxMotorTorque(1e6f);
                    }
                    else
                    {
                        var sj = new SphericalJoint { Id = joint.jointId };
                        sj.EnableSpring(false);
                        sj.EnableMotor(true);
                        sj.SetMotorVelocity(cmd);
                        sj.SetMaxMotorTorque(1e6f);
                    }
                    state.joints[j] = joint;
                }
                runtime[1] = state;

                if (contractT >= 1f)
                {
                    for (int j = 0; j < n; j++)
                    {
                        float delta = JointRelZ(state, j) - contractRelZ0[j];
                        string verdict = delta > 0.05f ? "DIRECT (sign +1)" : delta < -0.05f ? "NEGATED (sign -1)" : "NO MOTION";
                        Log($"[CONTRACT] chain[1] j{j} {(state.joints[j].isHinge ? "HINGE" : "SPHERICAL")} relDeltaZ={delta:F4} => {verdict}");
                    }
                    contractPhase = 2;
                    contractT = 0f;
                    Log("[CONTRACT] phase 2: closed-loop — driving chain[1] toward 40deg-about-Z relative target");
                }
                return;
            }

            if (contractPhase == 2 || contractPhase == 3)
            {
                contractT += dt;
                // Closed loop: relative target = 40deg about joint-frame Z, held
                // fixed. childPose solved from RelativeTarget's definition:
                //   qRel = inv(parentPose*frameA) * (childPose*frameB)
                //   => childPose = parentPose * frameA * qRel * inv(frameB)
                quaternion qRelTarget = math.mul(
                    new quaternion(0f, 0f, math.sin(0.5f * math.radians(40f)), math.cos(0.5f * math.radians(40f))),
                    quaternion.identity);
                for (int j = 0; j < n; j++)
                {
                    var joint = state.joints[j];
                    Box3DBody bodyA = (j == 0) ? state.rootBody : state.bones[j - 1];
                    Box3DBody bodyB = state.bones[j];
                    if (!bodyA || !bodyB || !bodyA.Body.IsValid || !bodyB.Body.IsValid) continue;

                    quaternion parentPose = bodyA.Body.GetRotation();
                    quaternion childPose = math.mul(math.mul(math.mul(parentPose, joint.frameA.Rotation), qRelTarget),
                                                     math.inverse(joint.frameB.Rotation));

                    SplineChainController.JointDriveTarget(ref joint, bodyA.Body, bodyB.Body,
                        parentPose, childPose, joint.frameA, joint.frameB, dt, 5000f, null);
                    state.joints[j] = joint;
                }
                runtime[1] = state;

                if (contractT - contractLastLog >= 0.25f)
                {
                    contractLastLog = contractT;
                    for (int j = 0; j < n; j++)
                    {
                        var joint = state.joints[j];
                        Box3DBody bodyA = (j == 0) ? state.rootBody : state.bones[j - 1];
                        Box3DBody bodyB = state.bones[j];
                        if (!bodyA || !bodyB || !bodyA.Body.IsValid || !bodyB.Body.IsValid) continue;
                        quaternion qRel = RelativeTarget(bodyA.Body.GetRotation(), bodyB.Body.GetRotation(), joint.frameA, joint.frameB);
                        quaternion err = math.mul(qRelTarget, math.inverse(qRel));
                        if (err.value.w < 0f) err.value = -err.value;
                        float errDeg = math.degrees(2f * math.acos(math.clamp(err.value.w, -1f, 1f)));
                        Log($"[CLOSED-LOOP] {(joint.isHinge ? "HINGE" : "SPHERICAL")} j{j} t={contractT:F2} errDeg={errDeg:F2}");
                    }
                }
                return;
            }
        }

        /// <summary>Z component of the joint-frame relative rotation vector (rad).</summary>
        private float JointRelZ(ChainRuntime state, int j)
        {
            var joint = state.joints[j];
            Box3DBody bodyA = (j == 0) ? state.rootBody : state.bones[j - 1];
            Box3DBody bodyB = state.bones[j];
            quaternion q = RelativeTarget(bodyA.Body.GetRotation(), bodyB.Body.GetRotation(), joint.frameA, joint.frameB);
            q = math.normalizesafe(q, quaternion.identity);
            if (q.value.w < 0f) q.value = -q.value;
            float sin = math.length(q.value.xyz);
            float angle = 2f * math.atan2(sin, q.value.w);
            float3 axis = sin < 1e-5f ? new float3(0, 0, 1) : q.value.xyz / sin;
            return angle * axis.z;
        }

        private void Log(string message)
        {
            Debug.Log(message);
            if (probeWriter == null)
            {
                System.IO.Directory.CreateDirectory("Logs");
                probeWriter = new System.IO.StreamWriter("Logs/drive-probe.log", append: true) { AutoFlush = true };
            }
            probeWriter.WriteLine(message);
        }

        private void DriveArticulation()
        {
            float dt = Time.fixedDeltaTime;

            if (contractProbe && chains.Count > 1) { StepContractProbe(dt); return; }

            // Scratch buffers, reused per frame (no allocations in the hot path).
            int maxJoints = 0;
            for (int i = 0; i < chains.Count; i++) maxJoints = math.max(maxJoints, runtime[i].joints.Length);
            if (driveSegDirs == null || driveSegDirs.Length < maxJoints)
            {
                driveSegDirs = new float3[maxJoints];
                driveHingeAxes = new float3[maxJoints];
            }

            for (int i = 0; i < chains.Count; i++)
            {
                var chain = chains[i];
                var state = runtime[i];
                int n = state.joints.Length;

                // --- Projected pose: station polyline → segment directions ---
                for (int j = 0; j < n; j++)
                {
                    float3 start = SampleOnRail(state.jointS[j], state.rail, state.railLength).position;
                    float3 end = (j == n - 1)
                        ? ComputeTipTarget(ref state)
                        : SampleOnRail(state.jointS[j + 1], state.rail, state.railLength).position;
                    driveSegDirs[j] = math.normalizesafe(end - start, math.forward());
                }

                // --- Twist datum, inferred from the pose itself ---
                // The bend-plane normal between consecutive segments IS the hinge
                // axis the pose demands. Where segments are collinear (cross ~ 0),
                // parallel-transport the previous normal along the segment so twist
                // stays pose-derived, never externally pinned.
                float3 hPrev = float3.zero;
                for (int j = 0; j < n; j++)
                {
                    float3 h;
                    if (j + 1 < n)
                    {
                        float3 c = math.cross(driveSegDirs[j], driveSegDirs[j + 1]);
                        if (math.lengthsq(c) > 1e-6f)
                            h = math.normalizesafe(c);
                        else
                            h = TransportDatum(hPrev, driveSegDirs[math.max(j - 1, 0)], driveSegDirs[j]);
                    }
                    else
                    {
                        h = TransportDatum(hPrev, driveSegDirs[math.max(j - 1, 0)], driveSegDirs[j]);
                    }
                    driveHingeAxes[j] = h;
                    hPrev = h;
                }

                // --- Per joint: full target pose + hierarchical propagation ---
                quaternion prevChildTarget = quaternion.identity;
                bool prevChildTargetValid = false;
                for (int j = 0; j < n; j++)
                {
                    var joint = state.joints[j];
                    Box3DBody bodyA = (j == 0) ? chain.rootBody : chain.bones[j - 1];
                    Box3DBody bodyB = chain.bones[j];
                    bool validA = bodyA && bodyA.Body.IsValid;
                    bool validB = bodyB && bodyB.Body.IsValid;

                    float3 d = driveSegDirs[j];
                    float3 h = driveHingeAxes[j];
                    quaternion childTarget;
                    quaternion parentTarget;

                    if (validA && validB)
                    {
                        // Step 1: rotate the child's ACTUAL pose so its bone points
                        // along the pose segment (shortest arc — bone datum only).
                        float3 curBone = math.normalizesafe(
                            (float3)bodyB.Body.GetPosition() - (float3)bodyA.Body.GetPosition(), d);
                        quaternion rotB = bodyB.Body.GetRotation();
                        childTarget = math.mul(ShortestArcRotation(curBone, d), rotB);

                        // Step 2: twist correction — datum depends on joint type.
                        // Hinge: frameB Z IS the hinge axis; align it to the bend
                        // normal at THIS joint (between parent's segment d[j-1] and
                        // child's segment d[j]) = driveHingeAxes[j-1].
                        // Spherical: frameB Z is the BONE axis (already aligned in
                        // step 1 — never use it as a twist datum, that would curl
                        // the bone 90° off its target). The twist datum is the NEXT
                        // joint's hinge axis, expressed in this child's local space,
                        // aligned to the NEXT segment bend normal driveHingeAxes[j].
                        float3 uLocal, hTarget;
                        bool hasTwistDatum;
                        if (joint.isHinge)
                        {
                            hasTwistDatum = j > 0;
                            uLocal = math.mul(joint.frameB.Rotation, math.forward());
                            hTarget = (j > 0) ? driveHingeAxes[j - 1] : float3.zero;
                        }
                        else if (j + 1 < n && state.joints[j + 1].isHinge)
                        {
                            uLocal = math.mul(state.joints[j + 1].frameB.Rotation, math.forward());
                            hTarget = driveHingeAxes[j];
                            hasTwistDatum = true;
                        }
                        else
                        {
                            hasTwistDatum = false; uLocal = hTarget = float3.zero;
                        }

                        if (hasTwistDatum)
                        {
                            float3 axisW = math.mul(childTarget, uLocal);
                            float3 axisP = axisW - d * math.dot(axisW, d);
                            float3 hP = hTarget - d * math.dot(hTarget, d);
                            if (math.lengthsq(axisP) > 1e-6f && math.lengthsq(hP) > 1e-6f)
                            {
                                axisP = math.normalizesafe(axisP);
                                hP = math.normalizesafe(hP);
                                // Shortest arc between two vectors in the plane ⟂ d is
                                // exactly the twist about d — no AxisAngle needed.
                                childTarget = math.mul(ShortestArcRotation(axisP, hP), childTarget);
                            }
                        }

                        // Step 3: hierarchical datum. Joint j's parent target is the
                        // TARGET pose of its parent body (child target of joint j-1);
                        // joint 0's parent is the root body whose pose is owned by
                        // the god forces, so it uses the root's actual rotation.
                        parentTarget = (j == 0 || !prevChildTargetValid)
                            ? bodyA.Body.GetRotation()
                            : prevChildTarget;
                    }
                    else
                    {
                        childTarget = quaternion.identity;
                        parentTarget = validA ? bodyA.Body.GetRotation() : quaternion.identity;
                    }
                    prevChildTarget = childTarget;
                    prevChildTargetValid = true;

                    JointDriveTarget(ref joint, bodyA.Body, bodyB.Body,
                        parentTarget, childTarget,
                        joint.frameA, joint.frameB, dt, chain.joints[j].torqueLimit,
                        state.distalBodies != null ? state.distalBodies[j] : null);

                    state.joints[j] = joint;

                    // Probe: joint-frame angle between commanded and actual pose.
                    // Editor diagnostics only — zero cost when disabled.
                    if (probeLogging && state.probeErrDeg != null && validA && validB)
                    {
                        var qTarget = RelativeTarget(parentTarget, childTarget, joint.frameA, joint.frameB);
                        var qActual = RelativeTarget(bodyA.Body.GetRotation(), bodyB.Body.GetRotation(), joint.frameA, joint.frameB);
                        float dot = math.clamp(math.abs(math.dot(qTarget, qActual)), 0f, 1f);
                        state.probeErrDeg[j] = math.degrees(2f * math.acos(dot));

                        // Commanded target as axis-angle in joint frame: reveals WHAT
                        // the pose demands (e.g. mostly about X for a sane hinge vs.
                        // a large off-axis component when the twist datum is wrong).
                        float3 rvT = RotationVector(qTarget);
                        float3 rvA = RotationVector(qActual);
                        state.probeTargetRvDeg[j] = math.degrees(rvT);
                        state.probeActualRvDeg[j] = math.degrees(rvA);

                        // GROUND TRUTH: world-space angle between the actual bone
                        // direction and the projected pose's segment direction.
                        // This is what "replicating the pose" means — independent
                        // of any joint frame convention.
                        float3 actualBone = math.normalizesafe(
                            (float3)bodyB.Body.GetPosition() - (float3)bodyA.Body.GetPosition(), driveSegDirs[j]);
                        float boneDot = math.clamp(math.dot(actualBone, driveSegDirs[j]), -1f, 1f);
                        state.probePoseErrDeg[j] = math.degrees(math.acos(boneDot));
                    }
                    else if (probeLogging && state.probeErrDeg != null)
                    {
                        state.probeErrDeg[j] = -1f; // invalid bodies marker
                        state.probePoseErrDeg[j] = -1f;
                    }
                }
                runtime[i] = state;

                if (probeLogging && state.probeErrDeg != null)
                {
                    probeLogTimer += dt;
                    if (probeLogTimer >= probeLogInterval)
                    {
                        probeLogTimer = 0f;
                        var sb = new System.Text.StringBuilder(256);
                        sb.Append($"[DRIVE-PROBE] chain[{i}]");
                        for (int j = 0; j < state.probeErrDeg.Length; j++)
                        {
                            var t = state.probeTargetRvDeg[j];
                            var a = state.probeActualRvDeg[j];
                            sb.Append($" j{j}: poseErr={state.probePoseErrDeg[j]:F1} jfErr={state.probeErrDeg[j]:F1} tgt=({t.x:F0},{t.y:F0},{t.z:F0}) act=({a.x:F0},{a.y:F0},{a.z:F0})");
                        }
                        Log(sb.ToString());
                    }
                }
            }
        }

        /// <summary>
        /// Parallel transport: rotate a plane normal by the shortest arc that
        /// carries direction 'from' onto 'to'. Used where the station polyline
        /// is straight and the bend normal is degenerate.
        /// </summary>
        private static float3 TransportDatum(float3 datum, float3 from, float3 to)
        {
            if (math.lengthsq(datum) < 1e-6f || math.lengthsq(from) < 1e-6f) return datum;
            return math.mul(ShortestArcRotation(from, to), datum);
        }

        /// <summary>
        /// Tip target: straight continuation of the chain's last solved segment.
        /// The tip is a free end — wrist anchor position + the last segment's
        /// chord direction x the rest-pose anchor-to-tip length. Never clamped
        /// to or searched along the rail, so it can never cross to the other
        /// branch or pin at a rail end.
        /// </summary>
        private static float3 ComputeTipTarget(ref ChainRuntime state)
        {
            int lastAnchor = state.jointS.Length - 1;
            RailSample last = SampleOnRail(state.jointS[lastAnchor], state.rail, state.railLength);
            RailSample prev = SampleOnRail(state.jointS[lastAnchor - 1], state.rail, state.railLength);
            float3 dir = math.normalizesafe(last.position - prev.position, last.tangent);
            return last.position + dir * state.boneLengths[state.boneLengths.Length - 1];
        }

        private static quaternion ShortestArcRotation(float3 from, float3 to)
        {
            from = math.normalizesafe(from, math.forward());
            to = math.normalizesafe(to, math.forward());
            float dot = math.dot(from, to);
            if (dot > 1f - 1e-6f) return quaternion.identity;
            float3 axis = from;
            if (dot < -1f + 1e-6f)
            {
                // 180 degree rotation: pick a deterministic perpendicular axis.
                // Use cross(from, math.right()) first; fall back to cross(from, math.forward()).
                float3 axis1 = math.cross(from, math.right());
                axis = math.lengthsq(axis1) > 0.001f ? math.normalize(axis1) : math.normalize(math.cross(from, math.forward()));
                return quaternion.AxisAngle(axis, math.PI);
            }
            axis = math.normalize(math.cross(from, to));
            float angle = math.acos(dot);
            return quaternion.AxisAngle(axis, angle);
        }

        private static float3 SegmentDirection(RailSample start, RailSample end)
        {
            // Tangents are the primary pose target; chord direction is only a stable fallback for
            // opposing/degenerate tangents on a sharply folded rail.
            float3 tangentDirection = start.tangent + end.tangent;
            return math.normalizesafe(tangentDirection, math.normalizesafe(end.position - start.position, math.down()));
        }

        /// <summary>
        /// Orthonormal joint frame with Z along 'axis' that NEVER degenerates:
        /// the up hint is derived from a world axis least parallel to 'axis',
        /// unlike LookRotationSafe(axis, up) which returns an arbitrary
        /// perpendicular when axis ~ up (vertical hinge axes — pelvis, elbows
        /// on a vertical rail — got coin-flip frames per rebuild).
        /// </summary>
        internal static quaternion FrameFromAxisWorld(float3 axis)
        {
            axis = math.normalizesafe(axis, math.forward());
            float3 reference = math.abs(math.dot(axis, math.up())) > 0.9f ? math.forward() : math.up();
            float3 yAxis = math.normalizesafe(math.cross(reference, axis), math.right());
            float3 upHint = math.normalizesafe(math.cross(axis, yAxis), math.up());
            return quaternion.LookRotationSafe(axis, upHint); // hints are orthonormal by construction
        }

        /// <summary>Body rotation with its UP axis along 'up' (never degenerate).</summary>
        private static quaternion RotationUpAlong(float3 up)
        {
            up = math.normalizesafe(up, math.forward());
            // cross(right, up) is perpendicular to 'up' by construction; the
            // fallback only fires for up ~ world X where LookRotationSafe's own
            // projection is well-defined anyway.
            float3 fwd = math.normalizesafe(math.cross(math.right(), up), math.forward());
            return quaternion.LookRotationSafe(fwd, up);
        }

        /// <summary>Body rotation with its FORWARD axis along 'fwd' (never degenerate).</summary>
        private static quaternion RotationForwardAlong(float3 fwd)
        {
            fwd = math.normalizesafe(fwd, math.forward());
            // cross(fwd, right) is perpendicular to 'fwd' by construction.
            float3 up = math.normalizesafe(math.cross(fwd, math.right()), math.up());
            return quaternion.LookRotationSafe(fwd, up);
        }

        internal static quaternion RelativeTarget(quaternion parentPose, quaternion childPose, B3Transform localA, B3Transform localB) => math.mul(math.inverse(math.mul(parentPose, localA.Rotation)), math.mul(childPose, localB.Rotation));
        private static float SignedTwistZ(quaternion q) { q = math.normalizesafe(q, quaternion.identity); return 2f * math.atan2(q.value.z, q.value.w); }
        public static float3 RotationVector(quaternion q) { q = math.normalizesafe(q, quaternion.identity); if (q.value.w < 0f) q.value = -q.value; float sine = math.length(q.value.xyz); return sine < 1e-5f ? 2f * q.value.xyz : q.value.xyz * (2f * math.atan2(sine, q.value.w) / sine); }
        private static float3 ClampMagnitude(float3 value, float maximum) { float length = math.length(value); return length > maximum && length > 1e-5f ? value * (maximum / length) : value; }

        private static void SetupFrames(ref JointDefBase def, Body a, Body b, B3Transform worldFrame, out B3Transform localA, out B3Transform localB)
        {
            def.BodyIdA = a.Id; def.BodyIdB = b.Id; def.CollideConnected = false;
            localA = ToLocal(a, worldFrame); localB = ToLocal(b, worldFrame); def.LocalFrameA = localA; def.LocalFrameB = localB;
        }
        private static B3Transform ToLocal(Body body, B3Transform worldFrame) { quaternion q = body.GetRotation(); return new B3Transform { Position = math.mul(math.inverse(q), (float3)(worldFrame.Position - body.GetPosition())), Rotation = math.mul(math.inverse(q), worldFrame.Rotation) }; }

        private void DestroyJoints()
        {
            foreach (Joint joint in nativeJoints) if (joint.IsValid) joint.Destroy(wakeAttached: false);
            nativeJoints.Clear();
            bodies.Clear();
            runtime = null;
        }

        private void OnDisable()
        {
            DestroyJoints(); built = false;
            if (probeWriter != null) { probeWriter.Dispose(); probeWriter = null; }
        }

        private static quaternion ComputeBonePoseGeneric(float3 boneDir, float3 worldHingeAxis, quaternion jointFrameARotation, quaternion parentPose, float3 boneAxisLocal)
        {
            quaternion baseRotation = math.mul(parentPose, math.inverse(jointFrameARotation));
            quaternion invBase = math.inverse(baseRotation);

            float3 targetBoneAxis = math.mul(invBase, boneDir);

            if (math.lengthsq(worldHingeAxis) < 1e-6f)
            {
                // Spherical: align boneAxisLocal to targetBoneAxis
                quaternion align = ShortestArcRotation(boneAxisLocal, targetBoneAxis);
                return math.mul(baseRotation, align);
            }

            float3 targetHingeAxis = math.mul(invBase, worldHingeAxis);
            // In joint frame A, the hinge axis IS the Z axis (forward) by construction
            float3 localHingeAxis = math.forward();

            quaternion alignHinge = ShortestArcRotation(localHingeAxis, targetHingeAxis);
            float3 alignedBoneAxis = math.mul(alignHinge, boneAxisLocal);
            float3 planeBoneAxis = math.normalizesafe(targetBoneAxis - targetHingeAxis * math.dot(targetBoneAxis, targetHingeAxis), math.forward());
            float3 planeAligned = math.normalizesafe(alignedBoneAxis - targetHingeAxis * math.dot(alignedBoneAxis, targetHingeAxis), math.forward());
            quaternion twist = ShortestArcRotation(planeAligned, planeBoneAxis);

            return math.mul(baseRotation, math.mul(twist, alignHinge));
        }

        // Composite rigid-body inertia of the distal subtree about a world axis
        // through the joint anchor: rotated local tensors + parallel-axis terms.
        internal static float CompositeInertiaAboutAxis(Body[] bodies, float3 axisW, float3 anchor)
        {
            float3x3 I = float3x3.zero;
            for (int i = 0; i < bodies.Length; i++)
            {
                Body b = bodies[i];
                if (!b.IsValid) continue;
                float3x3 R = new float3x3(b.GetRotation());
                B3Matrix3 li = b.GetLocalRotationalInertia();
                I += math.mul(math.mul(R, new float3x3(li.Cx, li.Cy, li.Cz)), math.transpose(R));
                float m = b.GetMass();
                float3 r = (float3)b.GetWorldCenterOfMass() - anchor;
                I += m * (math.dot(r, r) * float3x3.identity - new float3x3(r * r.x, r * r.y, r * r.z));
            }
            return math.max(math.dot(axisW, math.mul(I, axisW)), 1e-4f);
        }

        // Single body's rotational inertia about a world axis (through its COM).
        internal static float BodyInertiaAboutAxis(Body b, float3 axisW)
        {
            if (!b.IsValid) return 1e-4f;
            float3x3 R = new float3x3(b.GetRotation());
            B3Matrix3 li = b.GetLocalRotationalInertia();
            float3x3 Iw = math.mul(math.mul(R, new float3x3(li.Cx, li.Cy, li.Cz)), math.transpose(R));
            return math.max(math.dot(axisW, math.mul(Iw, axisW)), 1e-4f);
        }

        internal static void JointDriveTarget(ref BoneJoint joint, Body parentBody, Body childBody, quaternion parentTarget, quaternion childTarget, B3Transform frameA, B3Transform frameB, float dt, float torqueLimit, Body[] distal = null)
        {
            quaternion relativeTarget = RelativeTarget(parentTarget, childTarget, frameA, frameB);
            quaternion relativeActual = RelativeTarget(parentBody.GetRotation(), childBody.GetRotation(), frameA, frameB);
            quaternion targetError = math.mul(relativeTarget, math.inverse(relativeActual));
            float3 errorVelocity = RotationVector(targetError) / math.max(dt, 1e-5f);

            quaternion targetDelta = joint.hasPreviousTarget
                ? math.mul(relativeTarget, math.inverse(joint.previousTarget))
                : quaternion.identity;
            float3 targetVelocity = RotationVector(targetDelta) / math.max(dt, 1e-5f);
            bool firstFrame = !joint.hasPreviousTarget;
            joint.previousTarget = relativeTarget;
            joint.hasPreviousTarget = true;
            if (firstFrame) joint.previousMotorVelocity = float3.zero;

            // Box3D motors command relative angular velocity in joint-frame A.
            // Limb-probe-proven drive (DriveSignProbe): inertia-aware command
            // shaping - the joint fights its whole distal subtree, so the command
            // magnitude is capped by the stopping-velocity profile of the achievable
            // angular decel (torqueLimit / I_eff), and damping acts on the MEASURED
            // relative spin (child - parent), not on the error velocity (which is
            // just a kp reduction and damps nothing).
            float kp = 0.5f * math.PI * math.max(1f, joint.hertz);  // Proportional gain

            // Parent coupling: measured relative angular velocity (child - parent),
            // world -> joint-A space. Absorbs the old 0.2 parent-velocity hack.
            float3 wRelWorld = childBody.GetAngularVelocity() - parentBody.GetAngularVelocity();
            float3 wRelA = math.mul(math.inverse(
                math.mul(parentBody.GetRotation(), frameA.Rotation)), wRelWorld);

            // Composite distal inertia about the error axis through the anchor.
            float3 rv = RotationVector(targetError);
            float theta = math.length(rv);
            float3 anchorW = (float3)parentBody.GetPosition()
                + math.mul(parentBody.GetRotation(), (float3)frameA.Position);
            float3 axisW = (theta > 1e-6f)
                ? math.normalizesafe(math.mul(parentBody.GetRotation(),
                    math.mul(frameA.Rotation, rv / theta)))
                : math.mul(parentBody.GetRotation(), math.mul(frameA.Rotation, math.forward()));
            float iEff = (distal != null && distal.Length > 0)
                ? CompositeInertiaAboutAxis(distal, axisW, anchorW)
                : BodyInertiaAboutAxis(childBody, axisW);
            float iSelf = BodyInertiaAboutAxis(childBody, axisW);

            // Stopping-velocity profile: never command more speed than the torque
            // limit can cancel over the remaining error angle - kills the
            // reach-overshoot-wobble cycle after fast motions.
            float alphaMax = torqueLimit / math.max(iEff, 1e-4f);
            float wStop = math.sqrt(2f * alphaMax * theta);
            float mag = math.min(kp * theta / math.max(dt, 1e-5f), wStop);
            float3 dir = (theta > 1e-6f) ? rv / theta : float3.zero;

            // Damping scaled by distal-load ratio (extended limb brakes harder);
            // baseline fraction validated by the limb probe (kd = 0.2 at ratio 1).
            float kdHere = 0.2f * joint.damping * math.sqrt(iEff / math.max(iSelf, 1e-4f));
            float3 motorVelocity = dir * mag + targetVelocity - kdHere * wRelA;
            // Inter-frame smoothing (kept near 1.0: heavier blending added phase lag).
            float3 smoothedMotorVelocity = math.lerp(joint.previousMotorVelocity, motorVelocity, 0.85f);
            joint.previousMotorVelocity = smoothedMotorVelocity;
            motorVelocity = ClampMagnitude(smoothedMotorVelocity, 20f);

            if (joint.isHinge)
            {
                // Hinge (revolute) motors use the frame-A velocity command directly —
                // probe evidence: negating here drove elbows PAST their target and
                // pinned them against the joint limit (act=-159 vs tgt=-99).
                // Spherical (spine) motors DO need the negated sign — see below.
                var revolute = new RevoluteJoint { Id = joint.jointId };
                float3 axis = math.forward();
                float speed = math.dot(motorVelocity, axis);
                revolute.EnableSpring(false);
                revolute.EnableMotor(true);
                revolute.SetMotorSpeed(speed);
                revolute.SetMaxMotorTorque(torqueLimit);
            }
            else
            {
                // Spherical motors interpret the velocity in WORLD space, not
                // joint-frame A (impulse-probe measured: cmd (0,0,1) moved qRel
                // about inv(frameA)*(0,0,1); cmd (0,1,0) -> inv(frameA)*(0,1,0);
                // sign DIRECT). motorVelocity here is joint-A space (RelativeTarget
                // basis), so convert: world = parentRot * frameA * v_A.
                float3 cmdWorld = math.mul(parentBody.GetRotation(),
                    math.mul(frameA.Rotation, motorVelocity));
                var spherical = new SphericalJoint { Id = joint.jointId };
                spherical.EnableSpring(false);
                spherical.EnableMotor(true);
                spherical.SetMotorVelocity(cmdWorld);
                spherical.SetMaxMotorTorque(torqueLimit);
            }
        }

        private void OnDrawGizmos()
        {
            if (!drawRail || runtime == null) return;

            foreach (var state in runtime)
            {
                if (state.rail == null || state.rail.Length < 2) continue;
                Gizmos.color = Color.cyan;
                var positions = new Vector3[state.rail.Length];
                for (int i = 0; i < state.rail.Length; i++)
                    positions[i] = (Vector3)state.rail[i].position;
                Gizmos.DrawLineStrip(positions, false);
            }
            
            foreach (var chain in chains)
            {
                if (chain.railControlPoints != null)
                {
                    Gizmos.color = Color.yellow;
                    for (int i = 0; i < chain.railControlPoints.Length; i++)
                        if (chain.railControlPoints[i])
                            Gizmos.DrawWireSphere(chain.railControlPoints[i].position, 0.05f);
                }
            }
        }
    }
}
