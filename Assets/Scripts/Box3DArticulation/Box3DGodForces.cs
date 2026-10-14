using System;
using System.Collections.Generic;
using Box3D;
using Box3D.Hybrid;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// "God mode" forces: drives selected Box3D bodies toward authored target
    /// transforms with world-space PD forces/torques (critically damped-ish
    /// spring, capped optionally). One component, one entry per body+target
    /// pair. Runs in FixedUpdate BEFORE Box3DWorld steps so forces accumulate
    /// into the next simulation step.
    ///
    /// Gains are per-unit-mass / per-unit-inertia (accelerations), so the
    /// same entry values work on a light hand and a heavy torso: force =
    /// m * (kp * posError + kd * velError), torque = I-scaled twist about
    /// the world axis. maxForce/maxTorque (Nm/N) clamp the result; 0 = off.
    /// </summary>
    [DefaultExecutionOrder(-110)] // before Box3DWorld (-100): forces land on the step we feed
    public sealed class Box3DGodForces : MonoBehaviour
    {
        [Serializable]
        public class Entry
        {
            [Tooltip("Body to drive. Static/kinematic bodies are skipped.")]
            public Box3DBody body;
            [Tooltip("World pose the body is pulled toward. Velocity is estimated from its per-step motion.")]
            public Transform target;

            [Header("Position")]
            [Tooltip("Position spring stiffness (1/s^2). ~w^2 of the tracking frequency: 400 = 3.2 Hz pull, 2500 = 8 Hz.")]
            public float positionGain = 900f;
            [Tooltip("Position damping relative to critical (kd = 2*sqrt(kp) * ratio). 1 = critically damped.")]
            [Range(0.05f, 2f)] public float positionDamping = 1f;
            [Tooltip("Force cap (N); 0 = uncapped.")]
            public float maxForce = 0f;

            [Header("Rotation")]
            [Tooltip("Axis remap: which TARGET-LOCAL axis the body's local +X/+Y/+Z must align to. Default identity ((1,0,0),(0,1,0),(0,0,1)) = body axes chase the target's axes directly. Example for a wrist: (0,0,1),(0,-1,0),(1,0,0) maps body X to target Z, body Y to -target Y, body Z to target X. Columns should be orthonormal; they are sanitized at use.")]
            public Vector3 axisRemapX = new Vector3(1f, 0f, 0f);
            public Vector3 axisRemapY = new Vector3(0f, 1f, 0f);
            public Vector3 axisRemapZ = new Vector3(0f, 0f, 1f);
            [HideInInspector] public Quaternion rotOffset; // driver-driven: Euler offset after the bind-relative alignment
            [HideInInspector] public float3 posOffset;     // driver-driven: bone-local position offset on top of the bind offset
            [Tooltip("Rotation stiffness (1/s^2) applied to the angle of the rotation error.")]
            public float rotationGain = 900f;
            [Tooltip("Rotation damping relative to critical.")]
            [Range(0.05f, 2f)] public float rotationDamping = 1f;
            [Tooltip("Torque cap (N.m); 0 = uncapped.")]
            public float maxTorque = 0f;
            [Tooltip("Per-axis torque limits (N.m) in the REFERENCE frame (bone local space): X/Y/Z torque components clamped independently. 0 = unlimited on that axis. Any non-zero value disables the scalar maxTorque cap.")]
            public float torqueLimitX = 0f;
            public float torqueLimitY = 0f;
            public float torqueLimitZ = 0f;
            [Tooltip("When >= 0 and the component's poseSource is valid: this entry's reference comes from the animated root motion (re-anchored to the live body) instead of a Transform. Spine stability entries use this; Transform target is then ignored.")]
            public int sourceEndpoint = -1;
            [HideInInspector] public bool driverDriven;    // set by HumanoidMotionDriver on bind
            [HideInInspector] public Vector3 driverRelPos; // bind pose relative to the rig root (world)
            [HideInInspector] public Quaternion driverRelRot;
            [HideInInspector] public Box3DBody rootBody;   // rig root body - position reference anchor
            [HideInInspector] public float3 driverBindPosLocal;  // bind COM, rig-root-local (stability blend target)
            [HideInInspector] public Quaternion driverBindRotLocal; // bind rotation, rig-root-local
        }

        [SerializeField, Tooltip("Body -> target entries. Evaluated top to bottom in FixedUpdate.")]
        private List<Entry> entries = new List<Entry>();

        /// <summary>Read-only entry access for decorators (e.g. lean controllers
        /// that rewrite axis remaps). Entries are live references - writes land
        /// in the drive loop on the next fixed step.</summary>
        public System.Collections.Generic.IReadOnlyList<Entry> Entries => entries;

        [Header("Physical Animator")]
        [SerializeField, Range(0f, 1f), Tooltip("Position bind bias: 1 = track the live animation fully (default); 0 = hold the bind-pose position (root-anchored: bind stance follows the rig root). Linear blend of the position target.")]
        private float bindBiasPosition = 1f;
        [SerializeField, Range(0f, 1f), Tooltip("Rotation bind bias: 1 = track the live animation fully (default); 0 = hold the bind-pose rotation (root-anchored). Linear blend of the rotation target.")]
        private float bindBiasRotation = 1f;
        [SerializeField, Range(0f, 1f), Tooltip("Root body position-gain scale for friction-propelled mode. 1 = pelvis hard-tracked to the animated pelvis (default); lower = the root mass is moved by the limb drives through ground friction instead of position-slaved to the animated pelvis. Friction-propulsion test knob.")]
        private float rootTrackingScale = 1f;
        [SerializeField, Tooltip("Root velocity anticipation (s): how far ahead of playback the ROOT entry's velocity feedforward looks, using MxM's pose-blend interpolation (GetRootVelocityAhead). Gives the drive the clip's imminent root velocity instead of a one-step-late finite difference - anticipates foot plants / clip transients at the pelvis. 0 = off (finite-difference feedforward only).")]
        private float rootAnticipationTime = 0f;
        [SerializeField, Range(0f, 1f), Tooltip("How much of the anticipated root velocity replaces the differenced feedforward (1 = full anticipation).")]
        private float rootAnticipationBlend = 1f;
        [SerializeField, Range(0f, 1f), Tooltip("Position anticipation lead (fraction of the anticipated velocity integral added to the root position target). Phase-lead on the P term - destabilizes if too high; start at 0.3 or keep 0 and rely on velocity anticipation alone.")]
        private float rootAnticipationLead = 0f;
        [SerializeField, Tooltip("Optional Box3DControllerWrapper driving the animated root. When set, the root velocity anticipation is scaled by the wrapper's MoveEfficiency (how much of the intended root motion survived collide-and-slide), so blocked or slid motion stops winding the feedforward/position-lead into walls.")]
        private Box3DControllerWrapper trajectorySource;
        [SerializeField, Tooltip("Optional MxM graph tap. Entries with sourceEndpoint >= 0 take their reference from the animated root motion (re-anchored to the live body) instead of a Transform.")]
        private Box3DPhysicalAnimator poseSource;
        [SerializeField, Range(-20f, 20f), Tooltip("Angular acceleration feedforward for pose-source entries (tau_ff = mass * rotationGain * gain * alpha_desired). 0 = off.")]
        private float angularAccelFeedforward = 0f;

        [Header("Target Motion")]
        [SerializeField, Tooltip("Derive target linear/angular velocity from the target transform's per-step motion (smoothed). Off = zero target velocity (pure damping to rest).")]
        private bool trackTargetVelocity = true;
        [SerializeField, Range(0f, 30f), Tooltip("Low-pass rate (Hz) for the estimated target velocities; 0 = unfiltered.")]
        private float targetVelocityFilter = 10f;
        [SerializeField, Tooltip("Low-pass rate (1/s) for the ANALYTIC (pose-source) target velocities. The raw per-step differencing of animated rotations is aliased; damping multiplies that noise (mass*kd). 0 = no filter (jittery damping above ~0.1).")]
        private float poseVelFilter = 15f;
        [SerializeField, Tooltip("Rotation error dead-band (deg). Below this angular error the drive torque ramps to zero - kills stick-slip limit cycles against ground friction (yaw jitter around rest). 0 = off.")]
        private float rotErrDeadband = 0.3f;

        [Header("Fall detection (drive saturation)")]
        [SerializeField, Tooltip("Detect falls from sustained drive saturation: bucket += (meanSaturation - threshold) * dt, leaked over time. Fall when the bucket fills.")]
        private bool fallDetect = false;
        [SerializeField, Range(0f, 3f), Tooltip("Saturation threshold: force magnitude / maxForce per body, averaged over bodies. Excess above this accumulates.")]
        private float fallSatThreshold = 0.85f;
        [SerializeField, Tooltip("Fall time (s): bucket fill needed to trigger. Spikes contribute proportionally, so sustained strain lowers the effective tolerance over time.")]
        private float fallTime = 0.5f;
        [SerializeField, Tooltip("Bucket leak rate (1/s): self-clears brief maneuver strain. 4 = bucket halves in ~0.17s of quiet.")]
        private float fallLeak = 4f;
        [SerializeField, Tooltip("Drives off for this long on fall, then re-enable (coroutine - survives the component being disabled).")]
        private float fallRecoverDelay = 3f;
        [SerializeField, Range(0f, 3f), Tooltip("Hard-spike tier: saturation at or above this for fallHardSteps consecutive steps triggers a fall immediately (hard collisions: impact velocity jumps spike the kd term even at small position error). 0 disables.")]
        private float fallHardThreshold = 0.98f;
        [SerializeField, Range(1, 60), Tooltip("Consecutive steps at/above fallHardThreshold required. 6 steps at 60Hz = 0.1s - immune to single-step solver noise.")]
        private int fallHardSteps = 6;
        [SerializeField, Range(0f, 1f), Tooltip("Fall response: fraction of the PD FORCES kept active while fallen (0 = drives fully off - classic behavior, 1 = forces keep their full influence). Ramped, not stepped.")]
        private float fallForceInfluence = 0f;
        [SerializeField, Range(0f, 1f), Tooltip("Fall response: fraction of the PD TORQUES kept active while fallen. Separate from the force fraction so posture can be held while position tracking lets go (or vice versa).")]
        private float fallTorqueInfluence = 0f;
        [SerializeField, Tooltip("Influence ramp rate (1/s): exponential blend speed between healthy (1) and fallen (fall*Influence) drive levels. High = snappy let-go, low = gradual collapse.")]
        private float fallInfluenceRampRate = 6f;

        // Re-anchored animated reference per entry (relative to the live body):
        // the target rides the physics body; only velocity/orientation come
        // from the animation - the reference never position-slaves the body.
        private readonly List<float3> m_refOffset = new List<float3>();     // target - body pos (world)
        private readonly List<quaternion> m_refRot = new List<quaternion>(); // target world rot
        private readonly List<bool> m_refValid = new List<bool>();
        private readonly List<float3> m_prevRefAngVel = new List<float3>(); // previous step's reference omega
        // Per-entry previous target pose for velocity estimation (world).
        private readonly List<float3> m_prevTargetPos = new List<float3>();
        private readonly List<quaternion> m_prevTargetRot = new List<quaternion>();
        private readonly List<float3> m_targetVel = new List<float3>();      // filtered linear velocity (m/s)
        private readonly List<float3> m_targetAngVel = new List<float3>();   // filtered angular velocity (rad/s)
        private readonly List<float3> m_filtVel = new List<float3>();        // filtered state, pose-source analytic velocities
        private readonly List<float3> m_filtAngVel = new List<float3>();
        private readonly List<bool> m_filtValid = new List<bool>();
        private bool m_prevValid;

        /// <summary>Entry count (for probes).</summary>
        public int EntryCount => entries.Count;
        /// <summary>Built body for entry i, null if unassigned.</summary>
        public Box3DBody GetBody(int i) => entries[i].body;

        private void OnEnable()
        {
            Resize();
            m_prevValid = false;
            m_fallen = false; m_fallBucket = 0f; m_fallHardCount = 0; // fresh detector each (re)enable
            m_forceInfluence = 1f; m_torqueInfluence = 1f;
            for (int i = 0; i < m_refValid.Count; i++) { m_refValid[i] = false; m_filtValid[i] = false; }
        }

        private void OnValidate()
        {
            if (entries.Count != m_prevTargetPos.Count) Resize();
        }

        private void Resize()
        {
            int n = entries.Count;
            while (m_prevTargetPos.Count < n)
            {
                m_prevTargetPos.Add(default); m_prevTargetRot.Add(quaternion.identity);
                m_targetVel.Add(default); m_targetAngVel.Add(default);
                m_filtVel.Add(default); m_filtAngVel.Add(default); m_filtValid.Add(false);
                m_refOffset.Add(default); m_refRot.Add(quaternion.identity);
                m_refValid.Add(false); m_prevRefAngVel.Add(default);
            }
            int m = m_prevTargetPos.Count;
            if (m > n)
            {
                m_prevTargetPos.RemoveRange(n, m - n); m_prevTargetRot.RemoveRange(n, m - n);
                m_targetVel.RemoveRange(n, m - n); m_targetAngVel.RemoveRange(n, m - n);
                m_filtVel.RemoveRange(n, m - n); m_filtAngVel.RemoveRange(n, m - n); m_filtValid.RemoveRange(n, m - n);
                m_refOffset.RemoveRange(n, m - n); m_refRot.RemoveRange(n, m - n);
                m_refValid.RemoveRange(n, m - n); m_prevRefAngVel.RemoveRange(n, m - n);
            }
        }

        /// <summary>Rebuild the entry list from a driver bind: each entry's
        /// reference is the animated root pose (re-anchored to the rig root
        /// body) offset by the node's bind-relative pose (relPos/relRot), so
        /// spine segments stay aligned with the animated spine while the root
        /// rides the physics body. Replaces all entries.</summary>
        public void SetPoseSource(Box3DPhysicalAnimator src) => poseSource = src;

        /// <summary>Wrapper whose collide-and-slide gates the anticipation
        /// (MoveEfficiency scaling). Null disables the gating.</summary>
        public void SetTrajectorySource(Box3DControllerWrapper src) => trajectorySource = src;

        public void SetupFromDriver(Box3DBody rootBody, List<Entry> newEntries,
            Vector3[] relPos, Quaternion[] relRot,
            float3[] bindPos = null, quaternion[] bindRot = null)
        {
            entries.Clear();
            // Bind-pose stability anchor: rig root body pose at bind. Per-entry
            // bind COM/rotation converted to root-local so the bias target
            // follows the (moving) rig root instead of freezing at spawn.
            if (rootBody != null && rootBody.Body.IsValid)
            {
                m_bindRootPos = rootBody.Body.Position;
                m_bindRootRot = rootBody.Body.Rotation;
            }
            quaternion invBindRoot = math.inverse(m_bindRootRot);
            for (int i = 0; i < newEntries.Count; i++)
            {
                var src = newEntries[i];
                entries.Add(new Entry
                {
                    body = src.body,
                    target = null,
                    positionGain = src.positionGain,
                    positionDamping = src.positionDamping,
                    maxForce = src.maxForce,
                    axisRemapX = src.axisRemapX,
                    axisRemapY = src.axisRemapY,
                    axisRemapZ = src.axisRemapZ,
                    rotOffset = src.rotOffset,
                    posOffset = src.posOffset,
                    rotationGain = src.rotationGain,
                    rotationDamping = src.rotationDamping,
                    maxTorque = src.maxTorque,
                    torqueLimitX = src.torqueLimitX,
                    torqueLimitY = src.torqueLimitY,
                    torqueLimitZ = src.torqueLimitZ,
                    sourceEndpoint = src.sourceEndpoint,
                    driverDriven = true,
                    driverRelPos = relPos != null && i < relPos.Length ? relPos[i] : Vector3.zero,
                    driverRelRot = relRot != null && i < relRot.Length ? relRot[i] : Quaternion.identity,
                    rootBody = rootBody,
                });
            }
            // Bind-pose stability targets come from the driver (bind-time body
            // pose), passed alongside the bone-relative offsets.
            if (bindPos != null && bindRot != null)
            {
                for (int i = 0; i < entries.Count; i++)
                {
                    if (i >= bindPos.Length || i >= bindRot.Length) break;
                    entries[i].driverBindPosLocal = math.mul(invBindRoot,
                        bindPos[i] - m_bindRootPos);
                    entries[i].driverBindRotLocal = math.mul(invBindRoot, bindRot[i]);
                }
            }
            Resize();
            for (int i = 0; i < m_refValid.Count; i++) { m_refValid[i] = false; m_filtValid[i] = false; }
            m_prevValid = false;
        }

        private float3 m_bindRootPos;   // rig root body pose at bind (stability-blend anchor)
        private quaternion m_bindRootRot;
        private float m_fallBucket;    // accumulated excess saturation (s)
        private int m_fallHardCount;   // consecutive steps at/above hard threshold
        private bool m_fallen;
        private float m_forceInfluence = 1f;   // ramped drive levels (1 healthy -> fall*Influence fallen)
        private float m_torqueInfluence = 1f;

        /// <summary>Pre-fall ramp progress (0..1) published every step while
        /// the fall detector runs. 0 = healthy; rises as the saturation bucket
        /// fills; 1 while fallen (drives off). Consumers (e.g. the driver's
        /// tracking-hertz ramp) subscribe to this.</summary>
        public event System.Action<float> PreFallProgress;

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            if (dt <= 0f) return;
            IWorldPoseSource src = poseSource != null && poseSource.IsValid ? poseSource : null;

            // 1) Resolve each entry's target world pose.
            //    Transform entries: read the transform. Pose-source entries:
            //    integrate the animated root delta into a reference relative
            //    to the live body (re-anchored - never position-slaving).
            Span<float3> tPos = stackalloc float3[Math.Max(1, entries.Count)];
            Span<quaternion> tRot = stackalloc quaternion[Math.Max(1, entries.Count)];
            for (int i = 0; i < entries.Count; i++)
            {
                var e = entries[i];
                if (e.driverDriven && e.sourceEndpoint >= 0 && src != null)
                {
                    // fromRoot: reference = the node's OWN animated bone frame
                    // (pivot + tip-bone rotation), offset by the bind-relative
                    // body pose. Absolute - re-derived each step, no lag/drift,
                    // and it tracks the animated lean, not just root motion.
                    if (e.body == null) continue;

                    src.GetNodeFrame(e.sourceEndpoint, out float3 bPos, out quaternion bRot, dt);

                    tPos[i] = bPos + math.mul(bRot, (float3)e.driverRelPos + e.posOffset);
                    tRot[i] = math.mul(math.mul(bRot, (quaternion)e.driverRelRot),
                        (quaternion)e.rotOffset);

                    // Bind-pose stability blend: pull the reference toward the
                    // bind stance, re-anchored to the LIVE rig root pose each
                    // step (turns/locomotes with the character), per channel.
                    // Bias 1 = pure live animation; 0 = pure bind pose.
                    if (bindBiasPosition < 1f || bindBiasRotation < 1f)
                    {
                        float3 liveRootPos = e.rootBody != null && e.rootBody.Body.IsValid
                            ? e.rootBody.Body.Position : m_bindRootPos;
                        quaternion liveRootRot = e.rootBody != null && e.rootBody.Body.IsValid
                            ? e.rootBody.Body.Rotation : m_bindRootRot;
                        float3 bPos2 = liveRootPos + math.mul(liveRootRot, e.driverBindPosLocal);
                        quaternion bRot2 = math.mul(liveRootRot, e.driverBindRotLocal);
                        tPos[i] = math.lerp(tPos[i], bPos2, 1f - bindBiasPosition);
                        tRot[i] = math.normalizesafe(math.slerp(tRot[i], bRot2, 1f - bindBiasRotation), tRot[i]);
                    }

                    // Reference velocities: analytic deltas of the (led) bone
                    // frame. NOTE: with extrapolation on, differencing bRot
                    // adds a d(lead)/dt ~ 0.5*dt*alpha term to the damping
                    // reference - an implicit acceleration feedforward that
                    // measurably helped spine tracking (kept deliberately;
                    // GetNodeAngVel exists as the lead-free alternative).
                    if (m_refValid[i])
                    {
                        quaternion dRot = math.mul(bRot, math.inverse(m_refRot[i]));
                        float3 rawAngVel = SplineChainController.RotationVector(dRot) / dt;
                        float3 rawVel = (bPos - m_refOffset[i]) / dt;
                        // Low-pass: raw differencing of per-step sampled bone
                        // frames is aliased (noise/dt); the damping terms
                        // multiply exactly that, so unfiltered jitter scales
                        // with positionDamping/rotationDamping.
                        if (poseVelFilter > 0f)
                        {
                            float follow = 1f - math.exp(-poseVelFilter * dt);
                            rawVel = m_filtValid[i]
                                ? math.lerp(m_filtVel[i], rawVel, follow) : rawVel;
                            rawAngVel = m_filtValid[i]
                                ? math.lerp(m_filtAngVel[i], rawAngVel, follow) : rawAngVel;
                            m_filtVel[i] = rawVel;
                            m_filtAngVel[i] = rawAngVel;
                            m_filtValid[i] = true;
                        }
                        m_targetAngVel[i] = rawAngVel;
                        m_targetVel[i] = rawVel;
                    }
                    else
                    {
                        m_targetAngVel[i] = default;
                        m_targetVel[i] = default;
                        m_filtValid[i] = false;
                    }
                    m_refRot[i] = bRot;
                    m_refOffset[i] = bPos;
                    m_refValid[i] = true;
                    m_prevValid = true; // estimator loop must not overwrite
                    continue;
                }
                if (e.target == null) continue;
                tPos[i] = e.target.position;
                tRot[i] = e.target.rotation;
            }

            // 2) Target velocities from target motion (world), low-passed.
            if (trackTargetVelocity)
            {
                for (int i = 0; i < entries.Count; i++)
                {
                    if (entries[i].sourceEndpoint >= 0 && src != null) continue; // analytic already
                    if (entries[i].target == null) continue;
                    float3 p = tPos[i];
                    quaternion r = tRot[i];
                    if (m_prevValid)
                    {
                        float3 v = (p - m_prevTargetPos[i]) / dt;
                        float3 w = SplineChainController.RotationVector(
                            math.mul(math.inverse(m_prevTargetRot[i]), r)) / dt;
                        float follow = targetVelocityFilter > 0f
                            ? 1f - math.exp(-targetVelocityFilter * dt) : 1f;
                        m_targetVel[i] = math.lerp(m_targetVel[i], v, follow);
                        m_targetAngVel[i] = math.lerp(m_targetAngVel[i], w, follow);
                    }
                    m_prevTargetPos[i] = p;
                    m_prevTargetRot[i] = r;
                }
                m_prevValid = true;
            }
            else
            {
                for (int i = 0; i < entries.Count; i++)
                { m_targetVel[i] = default; m_targetAngVel[i] = default; }
            }

            // 3) PD forces/torques, applied before Box3DWorld (-100) steps.
            float satSum = 0f; int satCount = 0; // fall detector: drive saturation

            // Fall influence ramp: exponential blend between healthy (1) and
            // the configured fallen drive levels. Scaled AFTER the caps so
            // the saturation detector measures the drive's true demand - a
            // weakened drive that keeps demanding full torque stays detected.
            float forceTgt = m_fallen ? fallForceInfluence : 1f;
            float torqueTgt = m_fallen ? fallTorqueInfluence : 1f;
            float a = 1f - math.exp(-math.max(fallInfluenceRampRate, 0.01f) * dt);
            m_forceInfluence = math.lerp(m_forceInfluence, forceTgt, a);
            m_torqueInfluence = math.lerp(m_torqueInfluence, torqueTgt, a);
            bool scaleForce = m_forceInfluence < 0.999f;
            bool scaleTorque = m_torqueInfluence < 0.999f;
            bool skipForce = m_forceInfluence < 0.001f;
            bool skipTorque = m_torqueInfluence < 0.001f;
            for (int i = 0; i < entries.Count; i++)
            {
                var e = entries[i];
                bool fromPose = e.driverDriven && e.sourceEndpoint >= 0 && src != null;
                if (e.body == null || !e.body.isActiveAndEnabled
                    || (!fromPose && e.target == null)) continue;
                Body b = e.body.Body;
                if (!b.IsValid || b.Type != Box3D.BodyType.Dynamic) continue;

                float mass = b.GetMass();
                float3 pos = b.Position;
                quaternion rot = b.Rotation;
                float3 vel = b.LinearVelocity;
                float3 angVel = b.AngularVelocity;

                // --- Position: critically-damped spring on the COM.
                // rootTrackingScale < 1 detunes the pelvis: the root mass is
                // then moved by the limb drives + ground friction instead of
                // being position-slaved to the animated pelvis.
                float rootScale = rootTrackingScale < 1f && e.body == e.rootBody
                    ? rootTrackingScale : 1f;
                float kp = e.positionGain * rootScale;
                float kd = 2f * math.sqrt(kp) * e.positionDamping;
                // Root anticipation: replace the differenced feedforward with
                // the clip's velocity a_dt ahead (partial blend). Damping then
                // pulls toward the imminent velocity, not the last step's -
                // the drive leads plant transients instead of chasing them.
                float3 ffVel = m_targetVel[i];
                float3 targetPos = tPos[i];
                if (fromPose && e.body == e.rootBody && rootAnticipationTime > 0f
                    && poseSource != null
                    && poseSource.GetRootVelocityAhead(rootAnticipationTime, out float3 vAhead))
                {
                    // Gate by the mover's trajectory: when collide-and-slide
                    // blocks or shortens the motion, the clip's anticipated
                    // velocity must not wind the feedforward (or the position
                    // lead) into the obstruction.
                    if (trajectorySource != null)
                        vAhead *= trajectorySource.MoveEfficiency;
                    ffVel = math.lerp(m_targetVel[i], vAhead, rootAnticipationBlend);
                    // Position lead over the anticipation window. Free-motion
                    // default: trapezoidal integral of the clip velocity.
                    // When the mover's predicted trajectory is available it
                    // replaces the integral: its polyline already embeds the
                    // collide-and-slide redirects (up ramps, along walls), so
                    // the lead follows the navigable path instead of aiming
                    // through the obstacle face. No extra MoveEfficiency
                    // scaling - the resolved path length already reflects it.
                    float3 lead = 0.5f * (m_targetVel[i] + vAhead) * rootAnticipationTime;
                    if (trajectorySource != null
                        && trajectorySource.TryGetPredictedLead(rootAnticipationTime, out float3 pLead))
                        lead = pLead;
                    // Local var - tPos[i] stays untouched for the fall
                    // detector / downstream consumers.
                    targetPos = math.lerp(targetPos, targetPos + lead, rootAnticipationLead);
                }
                float3 f = mass * (kp * (targetPos - pos)
                                 + kd * (ffVel - vel));
                if (fallDetect && e.maxForce > 0f
                    && !(rootTrackingScale < 1f && e.body == e.rootBody))
                {
                    satSum += math.min(math.length(f) / e.maxForce, 2f);
                    satCount++;
                }
                if (e.maxForce > 0f)
                {
                    float len = math.length(f);
                    if (len > e.maxForce) f *= e.maxForce / len;
                }

                if (m_fallen) f.y = 0;
                if (skipForce) f = float3.zero;
                else if (scaleForce) f *= m_forceInfluence;
                if (math.lengthsq(f) > 0f) b.ApplyForceToCenter(f, true);

                // --- Rotation: torque along the error axis. The target axes
                // are remapped first: desired orientation = targetRot * M,
                // where M's columns are the (sanitized) remap vectors - body
                // local X must point at target.rotation * M.column0, etc.
                float kpR = e.rotationGain;
                float kdR = 2f * math.sqrt(kpR) * e.rotationDamping;
                float3 colX = math.normalizesafe((float3)e.axisRemapX, new float3(1, 0, 0));
                float3 colY = math.normalizesafe((float3)e.axisRemapY, new float3(0, 1, 0));
                float3 colZ = math.normalizesafe((float3)e.axisRemapZ, new float3(0, 0, 1));
                // Sanitize to a proper rotation: re-orthogonalize Y against X,
                // then derive Z - avoids a skewed matrix feeding the solver.
                colY = math.normalizesafe(colY - math.dot(colY, colX) * colX, new float3(0, 1, 0));
                colZ = math.normalizesafe(math.cross(colX, colY), new float3(0, 0, 1));
                // Columns = images of the body basis vectors in target space:
                // desired world orientation = target.rotation * R(M).
                quaternion desired = math.mul(tRot[i],
                    new quaternion(new float3x3(colX, colY, colZ)));
                quaternion qErr = math.mul(desired, math.inverse(rot));
                float3 rvec = SplineChainController.RotationVector(qErr);
                // Dead-band ramp: inside the dead-band the error torque fades
                // to zero (smooth, no discontinuity). Without it, small yaw
                // errors cycle against ground friction: torque builds, static
                // friction breaks, body snaps - repeat (stick-slip jitter).
                float angErr = math.length(rvec);
                float db = math.radians(math.max(rotErrDeadband, 0f));
                if (db > 0f)
                {
                    float ramp = math.saturate((angErr - db) / (math.max(db, 1e-4f)));
                    rvec *= ramp;
                }
                float3 torque = rvec * (mass * kpR)
                              + (m_targetAngVel[i] - angVel) * (mass * kdR);
                if (fromPose) m_prevRefAngVel[i] = m_targetAngVel[i];
                bool perAxis = e.torqueLimitX > 0f || e.torqueLimitY > 0f || e.torqueLimitZ > 0f;
                if (perAxis)
                {
                    // Clamp torque per axis in the REFERENCE frame (bone local
                    // space for driver-driven entries): e.g. a hinge-like
                    // spine entry can forbid twist (X) while allowing swing.
                    quaternion invRef = math.inverse(tRot[i]);
                    float3 tl = math.mul(invRef, torque);
                    tl = math.clamp(tl,
                        new float3(-math.max(e.torqueLimitX, 0f), -math.max(e.torqueLimitY, 0f), -math.max(e.torqueLimitZ, 0f)),
                        new float3(math.max(e.torqueLimitX, 0f), math.max(e.torqueLimitY, 0f), math.max(e.torqueLimitZ, 0f)));
                    torque = math.mul(tRot[i], tl);
                }
                else if (e.maxTorque > 0f)
                {
                    float len = math.length(torque);
                    if (len > e.maxTorque) torque *= e.maxTorque / len;
                }
                if (skipTorque) torque = float3.zero;
                else if (scaleTorque) torque *= m_torqueInfluence;
                if (math.lengthsq(torque) > 0f) b.ApplyTorque(torque, true);
            }

            // 4) Fall detection, two tiers:
            //    tier 1: leaky bucket of excess saturation (sustained strain),
            //    tier 2: extreme saturation held for consecutive steps
            //            (hard collisions - impact velocity spikes the kd term).
            if (fallDetect && !m_fallen && satCount > 0)
            {
                float sat = satSum / satCount;

                // Tier 2: hard spike.
                if (fallHardThreshold > 0f && sat >= fallHardThreshold)
                {
                    if (++m_fallHardCount >= fallHardSteps) TriggerFall(sat);
                }
                else m_fallHardCount = 0;

                // Tier 1: sustained excess accumulation with leak.
                float excess = sat - fallSatThreshold;
                if (excess > 0f) m_fallBucket += excess * dt;
                else if (m_fallBucket > 0f)
                {
                    m_fallBucket -= m_fallBucket * fallLeak * dt;
                    if (m_fallBucket < 0.001f) m_fallBucket = 0f;
                }
                if (m_fallBucket >= fallTime) TriggerFall(sat);
            }
            if (fallDetect && PreFallProgress != null)
                PreFallProgress.Invoke(m_fallen ? 1f : math.saturate(m_fallBucket / math.max(fallTime, 1e-4f)));
        }

        private void TriggerFall(float sat)
        {
            m_fallBucket = 0f;
            m_fallHardCount = 0;
            m_fallen = true;
            Debug.Log($"[Box3DGodForces] FALL: saturation {sat:F2} vs threshold {fallSatThreshold:F2} - drive influence ramping to F:{fallForceInfluence:P0}/T:{fallTorqueInfluence:P0}, recovery {fallRecoverDelay:F1}s", this);
            // Component STAYS enabled (unlike the classic drives-off path): the
            // detector, target sampling and filter state keep running, and the
            // influence scalars below ramp the applied drive levels down.
            StartCoroutine(RecoverFromFall());
        }

        private System.Collections.IEnumerator RecoverFromFall()
        {
            yield return new WaitForSeconds(fallRecoverDelay);
            m_fallen = false; // influence ramps back up in the drive loop
            Debug.Log("[Box3DGodForces] fall recovery: drives ramping back in", this);
        }
    }
}