using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Animations;
using UnityEngine.Playables;
using Unity.Burst;
using MxM;

namespace Box3DArticulation
{
    /// <summary>Pose source for physics-driven characters: taps the MxM
    /// PlayableGraph output with an animation job that extracts world-space
    /// pivot/tip/roll points for a <see cref="HumanoidMotionDriver"/>'s
    /// endpoints (stream-side, no Transform reads) plus the animated root
    /// motion delta. The driver extrapolates the freshest sample to the
    /// upcoming physics step, removing the transform-pipeline latency.</summary>
    public interface IWorldPoseSource
    {
        /// <summary>World position of endpoint i's pivot, extrapolated dt ahead.</summary>
        float3 GetPivot(int i, float dt);
        /// <summary>World position of endpoint i's tip, extrapolated dt ahead.</summary>
        float3 GetTip(int i, float dt);
        /// <summary>World position of endpoint i's roll point, extrapolated dt ahead.</summary>
        float3 GetRoll(int i, float dt);
        /// <summary>Absolute animated root motion delta for the upcoming step (world space).</summary>
        void GetRootDelta(out float3 posDelta, out quaternion rotDelta, float dt);
        /// <summary>Absolute animated root pose in world space (current poll).</summary>
        void GetRootPose(out float3 pos, out quaternion rot, float dt);
        /// <summary>Per-node animated frame: pivot world position (extrapolated)
        /// and the tip bone's world rotation (current poll). God-force
        /// references track THIS - the actual animated bone lean - not the root.</summary>
        void GetNodeFrame(int i, out float3 pivotPos, out quaternion boneRot, float dt);
        /// <summary>Per-node tip-bone angular velocity (world, rad/s) from the
        /// tap ring's last two polls - first-derivative data, no lead.
        /// Zero when the ring has fewer than two samples.</summary>
        float3 GetNodeAngVel(int i);
        /// <summary>True after a successful graph tap.</summary>
        bool IsValid { get; }
    }

    /// <summary>
    /// Insert an <see cref="AnimationScriptPlayable"/> post-node into the MxM
    /// graph: the existing AnimationPlayableOutput flows through the tap, so
    /// the ghost Animator keeps rendering while the job copies endpoint world
    /// points into a shared buffer. The component polls the buffer stamp each
    /// FixedUpdate and keeps a two-sample ring (current + previous evaluation)
    /// for linear extrapolation (pose for t+dt) and analytic velocities.
    ///
    /// Endpoint layout mirrors the driver's endpoint list: all PointRef targets
    /// must be non-null ghost-bone transforms; PointRef offsets are applied
    /// in-job (worldPos = bonePos + boneRot * offset, matching
    /// Transform.TransformPoint).
    /// </summary>
    [DefaultExecutionOrder(-120)] // fill the ring before the driver (-110) consumes it
    public sealed class Box3DPhysicalAnimator : MonoBehaviour, IWorldPoseSource
    {
        [SerializeField, Tooltip("MxMAnimator whose graph is tapped. The graph output is re-wired through our extraction job on enable and restored on disable.")]
        private MxMAnimator mxmAnimator;
        [SerializeField, Tooltip("Driver defining the endpoint set (pivot/tip/roll PointRefs). Its targets must be the ghost's bones.")]
        private HumanoidMotionDriver driver;

        [SerializeField, Tooltip("Rig builder of the physical character. Non-null: physics -> MxM playback feedback is active (UserPlaybackSpeedMultiplier modulated by the ground-plane COM velocity error). Null: disabled.")]
        private Box3DRigBuilder rig;
        [SerializeField, Tooltip("Payload registry (gameplay-attached weights). Non-null: payload mass/velocity included in the COM the feedback tracks. Empty = resolved automatically.")]
        private Box3DPayloadRegistry payloadRegistry;
        [SerializeField, Tooltip("Absolute minimum playback speed multiplier (hard clamp). Feedback authority = this window; 0 = no floor.")]
        private float playbackSpeedMin = 0.85f;
        [SerializeField, Tooltip("Absolute maximum playback speed multiplier (hard clamp). Feedback authority = this window; 0 = no ceiling.")]
        private float playbackSpeedMax = 1.15f;
        [SerializeField, Tooltip("Phase error to speed-multiplier gain: multiplier change per m/s of ground-plane COM velocity error along the animated root's forward. 1 = 0.2 m/s trailing slows playback by 20% at max 0.15.")]
        private float speedFeedbackGain = 1f;
        [SerializeField, Tooltip("Phase error low-pass rate (1/s). Higher = snappier feedback, more gait noise.")]
        private float speedFeedbackSmoothing = 1.5f;
        [SerializeField, Tooltip("COM velocity dead-zone (m/s). Errors below this do not modulate playback (prevents idle speed drift).")]
        private float speedFeedbackDeadzone = 0.1f;
        [SerializeField, Tooltip("Yaw feedback gain: multiplier change per rad/s of yaw-rate error (animated root yaw rate minus physical rig yaw rate, both around world up). 0 = off. Slightly slows the clip when the body's rotation lags the animation's - the global counter to turn-drag (feet dragging behind on rapid turns).")]
        private float yawFeedbackGain = 0.5f;
        [SerializeField, Tooltip("Yaw-rate dead-zone (rad/s). Errors below this do not modulate playback (prevents idle yaw drift from gait noise).")]
        private float yawFeedbackDeadzone = 0.3f;
        [SerializeField, Range(0f, 1f), Tooltip("Root-COM coupling: how much of the XZ offset between the animated root (the character controller) and the physical COM the root follows. This is the CONTRACT link - it makes character movement derive from the rig's applied friction instead of animation alone, and it feeds the trajectory generator (which reads this same transform). 0 = kinematic root (CC moves independently, contract dead).")]
        private float rootComCoupling = 0f;
        [SerializeField, Range(0.5f, 6f), Tooltip("Coupling low-pass rate (1/s). MUST stay well below the drive's bandwidth (~2-4): the coupling's job is to extract the friction-derived NET drift, not to track drive ripple. Too high and the root follows intra-gait COM oscillation - the rig shake/jitter. This is the load-bearing tuning knob.")]
        private float rootComSpring = 3f;
        [SerializeField, Tooltip("Per-step displacement cap (m). Clamps the offset the root may move in one fixed step, so a transient COM spike (landing, stumble) cannot yank the character controller. 0 = uncapped.")]
        private float rootComMaxStep = 0.05f;

        [SerializeField, Tooltip("Warn once when the Animator update mode is not 'Animate Physics'. In that mode the reference is evaluated at RENDER rate but consumed at the FIXED rate - the aliasing that shows up as rig shake while stationary. Animate Physics is the supported configuration for physics-driven play.")]
        private bool warnOnNonFixedUpdateMode = true;

        // --- physics feedback state ------------------------------------------
        private float m_speedErrF;   // filtered phase error (m/s, root-forward)
        private bool m_speedFeedbackActive;

        // --- extraction job -------------------------------------------------
        [BurstCompile]
        private struct PoseExtractJob : IAnimationJob
        {
            public NativeArray<TransformStreamHandle> Handles; // 3 per endpoint: pivot, tip, roll
            public NativeArray<float3> Offsets;                // PointRef offsets (target-local)
            public NativeArray<float3> Points;                 // world points, same layout
            public NativeArray<float4> RootPose;               // [0]=eval root pos [1]=eval root rot
            public NativeArray<quaternion> BoneRots;           // tip bone world rot per endpoint
            public NativeArray<float> Stamp;                   // accumulated eval time
            public TransformStreamHandle RootHandle;           // animation root (MxM warps/turns it)

            public void ProcessRootMotion(AnimationStream stream)
            {
                // Eval-time animation root frame, read from a stream handle. Do
                // NOT use stream.rootMotionPosition/Rotation: MxM applies root
                // motion + trajectory WARP directly to the animation root
                // transform after the animator update - the stream deltas miss
                // all of it (turning comes only from angular warp when root
                // motion is off).
                RootPose[0] = new float4((float3)RootHandle.GetPosition(stream), 0f);
                RootPose[1] = new float4(math.normalizesafe(
                    (quaternion)RootHandle.GetRotation(stream)).value);
                Stamp[0] += stream.deltaTime > 0f ? stream.deltaTime : 0f;
            }

            public void ProcessAnimation(AnimationStream stream)
            {
                int n = BoneRots.Length;
                for (int i = 0; i < n; i++)
                    BoneRots[i] = math.normalizesafe(
                        (quaternion)Handles[i * 3 + 1].GetRotation(stream));
                for (int i = 0; i < Handles.Length; i++)
                {
                    float3 p = Handles[i].GetPosition(stream);
                    quaternion r = Handles[i].GetRotation(stream);
                    Points[i] = p + math.mul(r, Offsets[i]); // = TransformPoint(offset)
                }
            }
        }

        // --- runtime state --------------------------------------------------
        private AnimationScriptPlayable m_tap;
        private Playable m_originalSource;
        private AnimationPlayableOutput m_output;
        private bool m_tapInstalled;
        private bool m_valid;

        private NativeArray<TransformStreamHandle> m_handles;
        private NativeArray<float3> m_offsets;
        private NativeArray<float3> m_points;
        private NativeArray<float4> m_rootPose;
        private NativeArray<quaternion> m_boneRots;
        private NativeArray<float> m_stamp;

        [SerializeField, Tooltip("Root velocity reference from MxM's baked per-pose velocity (scaled by effective playback speed, plus event warp as a rate). Off = transform differencing.")]
        private bool useMmRootVelocity = true;
        [SerializeField, Tooltip("Extrapolate the tip-bone rotation one step ahead in GetNodeFrame (parity with the point extrapolation). Off = old behavior: current-poll rotation, no lead.")]
        private bool extrapolateBoneRot = true;

        // Two-sample ring (polled on the main thread from m_points/m_rootPose).
        private float3[] m_cur;        // 3n
        private float3[] m_prev;       // 3n
        private float4 m_curRootPos, m_curRootRot, m_prevRootPos, m_prevRootRot;
        private float4[] m_curRots;    // tip bone world rot per endpoint
        private float4[] m_prevRots;
        private Transform m_animRoot;
        private float m_lastStamp = float.MinValue;
        private float m_evalInterval = 1f / 60f;
        private bool m_hasPrev;
        private bool m_warnedUpdateMode;

        /// <summary>Ordering contract: the poll and the reference frame
        /// resolution happen in FixedUpdate (-120), i.e. after the previous
        /// physics step, before Box3DGodForces (-110) and Box3DWorld (-100).
        /// The COM coupling measures the solver-fresh COM in the SAME
        /// FixedUpdate (fixed cadence, no render-rate sampling - that was the
        /// aliasing bug) but APPLIES the resulting offset in LateUpdate, after
        /// MxM has applied its root motion. The post-step write is deliberate:
        /// it is the contract link that makes the character controller follow
        /// the physical rig's friction. Its stability comes from bandwidth
        /// separation (rootComSpring well below the drive's bandwidth), not
        /// from the write's position in the frame.</summary>

        // Fixed-step COM coupling state. The error is low-passed at the fixed
        // rate; the offset is applied once in LateUpdate.
        private float3 m_comErrF;
        private float3 m_comOffset;
        private bool m_comOffsetValid;
        private bool m_comErrValid;

        public bool IsValid => m_valid && m_tapInstalled;

        public Transform AnimRoot => mxmAnimator != null
            ? (mxmAnimator.AnimationRoot != null ? mxmAnimator.AnimationRoot : mxmAnimator.transform)
            : null;

        // ---------------------------------------------------------------- setup

        private void OnEnable()
        {
            m_valid = Validate();
            if (m_valid) TryInstallTap();
        }

        private void OnDisable()
        {
            RemoveTap();
            DisposeBuffers();
            m_valid = false;
            m_hasPrev = false;
            if (m_speedFeedbackActive && mxmAnimator != null)
                mxmAnimator.UserPlaybackSpeedMultiplier = 1f; // leave clean
            m_speedFeedbackActive = false;
            m_speedErrF = 0f;
            // Coupling state: a stale error/offset would apply a phantom
            // displacement on the first frame after re-enable.
            m_comErrValid = false;
            m_comOffsetValid = false;
            m_comErrF = default;
            m_comOffset = default;
        }

        private bool Validate()
        {
            if (mxmAnimator == null || driver == null)
            { Debug.LogWarning("[Box3DPhysicalAnimator] MxM animator and driver must be assigned.", this); return false; }
            int n = driver.EndpointCount;
            if (n == 0)
            { Debug.LogWarning("[Box3DPhysicalAnimator] driver has no endpoints.", this); return false; }
            for (int i = 0; i < n; i++)
            {
                var e = driver.GetEndpoint(i);
                if (e.tip.target == null || e.pivot.target == null)
                {
                    Debug.LogWarning($"[Box3DPhysicalAnimator] endpoint '{e.nodeName}' has a null tip/pivot target - the tap needs real ghost bones for all points.", this);
                    return false;
                }
                // Roll is only read for chain references; a placeholder is bound
                // otherwise (tip handle), so a missing roll there is fine.
                if (e.chainReference && e.roll.target == null)
                {
                    Debug.LogWarning($"[Box3DPhysicalAnimator] chain-reference endpoint '{e.nodeName}' has a null roll target.", this);
                    return false;
                }
            }
            return true;
        }

        private void AllocateBuffers()
        {
            int np = driver.EndpointCount * 3;
            m_handles = new NativeArray<TransformStreamHandle>(np, Allocator.Persistent);
            m_offsets = new NativeArray<float3>(np, Allocator.Persistent);
            m_points = new NativeArray<float3>(np, Allocator.Persistent);
            m_boneRots = new NativeArray<quaternion>(driver.EndpointCount, Allocator.Persistent);
            m_rootPose = new NativeArray<float4>(2, Allocator.Persistent);
            m_stamp = new NativeArray<float>(1, Allocator.Persistent);
            m_cur = new float3[np];
            m_prev = new float3[np];
            m_curRots = new float4[driver.EndpointCount];
            m_prevRots = new float4[driver.EndpointCount];
        }

        private void TryInstallTap()
        {
            var graph = mxmAnimator.MxMPlayableGraph;
            if (!graph.IsValid())
                return; // MxM not initialized yet; retried from Update

            // Find MxM's animation output (created by AnimationPlayableUtilities.Play).
            AnimationPlayableOutput output = default;
            bool found = false;
            for (int i = 0; i < graph.GetOutputCount(); i++)
            {
                PlayableOutput o = graph.GetOutput(i);
                // PlayableOutput -> AnimationPlayableOutput is a runtime-checked
                // explicit cast (throws when incompatible); try/catch guards the
                // non-animation outputs in the graph.
                try
                {
                    output = (AnimationPlayableOutput)o;
                    found = true;
                    break;
                }
                catch (InvalidCastException) { /* not an animation output */ }
            }
            if (!found)
            {
                Debug.LogWarning("[Box3DPhysicalAnimator] no AnimationPlayableOutput on the MxM graph yet.", this);
                return;
            }

            AllocateBuffers();

            // Bind stream handles for all endpoint points.
            Animator animator = mxmAnimator.UnityAnimator;
            int n = driver.EndpointCount;
            for (int i = 0; i < n; i++)
            {
                var e = driver.GetEndpoint(i);
                int b = i * 3;
                m_handles[b + 0] = animator.BindStreamTransform(e.pivot.target);
                m_handles[b + 1] = animator.BindStreamTransform(e.tip.target);
                // Non-chain refs: roll slot becomes a tip placeholder (never read).
                bool hasRoll = e.roll.target != null;
                m_handles[b + 2] = animator.BindStreamTransform(hasRoll ? e.roll.target : e.tip.target);
                m_offsets[b + 0] = e.pivot.offset;
                m_offsets[b + 1] = e.tip.offset;
                m_offsets[b + 2] = hasRoll ? e.roll.offset : e.tip.offset;
            }

            var job = new PoseExtractJob
            {
                Handles = m_handles, Offsets = m_offsets, Points = m_points,
                RootPose = m_rootPose, Stamp = m_stamp, BoneRots = m_boneRots,
                RootHandle = animator.BindStreamTransform(
                    mxmAnimator.AnimationRoot != null ? mxmAnimator.AnimationRoot : mxmAnimator.transform),
            };
            m_originalSource = output.GetSourcePlayable();
            m_output = output;
            m_tap = AnimationScriptPlayable.Create(graph, job, 1);
            // Inputs MUST be processed (default): the animation stream flows
            // clips -> tap job -> output. SetProcessInputs(false) would skip
            // evaluating the input chain entirely - blank pose.
            m_tap.ConnectInput(0, m_originalSource, 0, 1f);
            output.SetSourcePlayable(m_tap);
            m_tapInstalled = true;

            // Stamp baseline so the first poll takes a full snapshot.
            m_lastStamp = m_stamp[0];
        }

        private void RemoveTap()
        {
            if (!m_tapInstalled) return;
            if (m_output.IsOutputValid() && m_originalSource.IsValid())
                m_output.SetSourcePlayable(m_originalSource);
            if (m_tap.IsValid()) m_tap.Destroy();
            m_tapInstalled = false;
        }

        private void DisposeBuffers()
        {
            if (m_boneRots.IsCreated) m_boneRots.Dispose();
            if (m_handles.IsCreated) m_handles.Dispose();
            if (m_offsets.IsCreated) m_offsets.Dispose();
            if (m_points.IsCreated) m_points.Dispose();
            if (m_rootPose.IsCreated) m_rootPose.Dispose();
            if (m_stamp.IsCreated) m_stamp.Dispose();
            m_handles = default; m_offsets = default; m_points = default;
            m_boneRots = default;
            m_rootPose = default; m_stamp = default;
        }

        private void Update()
        {
            // MxM creates its graph asynchronously during its own Start; retry
            // the tap until the output exists.
            if (m_valid && !m_tapInstalled) TryInstallTap();

            if (warnOnNonFixedUpdateMode && !m_warnedUpdateMode && m_valid
                && mxmAnimator != null && mxmAnimator.UpdateMode != AnimatorUpdateMode.Fixed)
            {
                m_warnedUpdateMode = true;
                Debug.LogWarning("[Box3DPhysicalAnimator] Animator update mode is not 'Animate Physics': "
                    + "the animation reference is evaluated at render rate but polled at the fixed rate. "
                    + "The poll's stamp guard keeps the pose data consistent, but MxM's root motion and "
                    + "warping already ran at render cadence on the transform, so the coupling, the "
                    + "trajectory and the playback feedback all see a render-stepped root. Set the "
                    + "Animator to 'Animate Physics' for physics-driven play.", this);
            }
        }

        /// <summary>Coupling apply half: after MxM has applied its root motion
        /// (warp/turn included), move the animation root XZ by the fixed-step
        /// COM offset. The trajectory generator reads this same transform, so
        /// the friction-derived displacement propagates into matching. Vertical
        /// is untouched (jumps/crouches must not fight the coupling).
        /// Stability comes from the low-pass bandwidth separation in
        /// UpdateComCoupling, not from where the write lands in the frame.</summary>
        private void LateUpdate()
        {
            if (!m_comOffsetValid) return;
            var t = mxmAnimator.transform;
            Vector3 p = t.position;
            p.x += m_comOffset.x;
            p.z += m_comOffset.z;
            t.position = p;
        }

        private void FixedUpdate()
        {
            if (!m_tapInstalled) return;
            if (m_animRoot == null)
            {
                m_animRoot = mxmAnimator.AnimationRoot;
                if (m_animRoot == null) return;
            }

            float stamp = m_stamp[0];
            if (stamp == m_lastStamp) return; // no new evaluation

            if (m_lastStamp > float.MinValue && stamp > m_lastStamp)
                m_evalInterval = math.clamp(stamp - m_lastStamp, 1e-4f, 0.5f);

            // Root reference: the ACTUAL animation root transform. This IS the
            // character controller - MxM applies root motion + warp + turn
            // directly to it. It is the only absolute reference in the system
            // (a target cannot be expressed relative to the object it drives),
            // and the trajectory generator reads this same transform, so the
            // coupling write below reaches the matcher.
            float4 nowPos = new float4((float3)m_animRoot.position, 0f);
            float4 nowRot = new float4(math.normalizesafe(
                (quaternion)m_animRoot.rotation).value);

            if (m_hasPrev)
            {
                // cur becomes prev before taking the new snapshot.
                for (int i = 0; i < m_cur.Length; i++) m_prev[i] = m_cur[i];
                for (int i = 0; i < m_curRots.Length; i++) m_prevRots[i] = m_curRots[i];
                m_prevRootPos = m_curRootPos;
                m_prevRootRot = m_curRootRot;
            }

            // Correct the stream points into the CURRENT root frame: the tap
            // sampled them in the eval-time frame (the root handle), and MxM
            // may have moved the transform since (warp/turn happen after the
            // animator update).
            float3 evalPos = (float3)m_rootPose[0].xyz;
            quaternion evalRot = math.normalizesafe(
                (quaternion)m_rootPose[1], quaternion.identity);
            quaternion dNowEval = math.mul((quaternion)nowRot, math.inverse(evalRot));
            for (int i = 0; i < m_points.Length; i++)
                m_cur[i] = (float3)nowPos.xyz + math.mul(dNowEval, m_points[i] - evalPos);

            m_curRootPos = nowPos;
            m_curRootRot = nowRot;
            // Bone rots corrected into the CURRENT root frame too - same
            // treatment as the points: eval-frame world rots, and MxM may
            // have warped/turned the root since the evaluation.
            for (int i = 0; i < m_boneRots.Length; i++)
                m_curRots[i] = math.mul(dNowEval, (quaternion)m_boneRots[i]).value;
            m_hasPrev = true;
            m_lastStamp = stamp;

            UpdatePlaybackFeedback();
            UpdateComCoupling();
        }

        /// <summary>Root-COM coupling, measurement half (fixed cadence): the
        /// character controller follows the physical rig's COM, so movement is
        /// derived from applied friction rather than animation alone. Measures
        /// the solver-fresh COM and low-passes the XZ error at rootComSpring.
        /// The rate MUST stay well below the drive's bandwidth - the coupling
        /// extracts net drift, not intra-gait ripple. Applying a render-rate
        /// filter here (as the old version did) is what made the result depend
        /// on the animator update mode.</summary>
        private void UpdateComCoupling()
        {
            m_comOffsetValid = false;
            if (rootComCoupling <= 0f || rig == null || mxmAnimator == null) return;

            var bodies = rig.Bodies;
            float totalMass = 0f;
            float3 com = float3.zero;
            for (int i = 0; i < bodies.Count; i++)
            {
                if (bodies[i] == null) continue;
                var b = bodies[i].Body;
                if (!b.IsValid || b.Type != Box3D.BodyType.Dynamic) continue;
                float m = b.GetMass();
                com += (float3)b.GetWorldCenterOfMass() * m;
                totalMass += m;
            }
            if (totalMass <= 0f) return;
            com /= totalMass;

            float3 rootP = (float3)m_curRootPos.xyz;
            float3 err = new float3(com.x - rootP.x, 0f, com.z - rootP.z);

            // Low-pass the error at the FIXED rate. This is the bandwidth
            // separation: a slow filter passes the friction-derived drift and
            // rejects gait ripple / drive oscillation.
            float dt = Time.fixedDeltaTime;
            float a = 1f - math.exp(-rootComSpring * dt);
            if (!m_comErrValid) { m_comErrF = err; m_comErrValid = true; }
            else m_comErrF = math.lerp(m_comErrF, err, a);

            float3 offset = m_comErrF * rootComCoupling;
            // Per-step displacement clamp: a transient COM spike (landing,
            // stumble, collision) must not yank the controller.
            float len = math.length(offset);
            if (rootComMaxStep > 0f && len > rootComMaxStep)
                offset *= rootComMaxStep / len;
            m_comOffset = offset;
            m_comOffsetValid = true;
        }

        /// <summary>Physics -> MxM playback feedback: modulates
        /// UserPlaybackSpeedMultiplier by the ground-plane COM velocity error
        /// (animation vs physical rig), measured along the animated root's
        /// forward. Body trailing => slow playback slightly (let it catch up);
        /// body leading => speed up. Facing comes from the root rotation, not
        /// the COM; vertical axis is projected out so jumps/crouches don't
        /// read as phase error.</summary>
        private void UpdatePlaybackFeedback()
        {
            float dt = Time.fixedDeltaTime;
            bool active = rig != null && rig.Root != null && mxmAnimator != null
                          && mxmAnimator.CurrentAnimData != null && m_hasPrev;
            if (active && payloadRegistry == null)
                payloadRegistry = FindFirstObjectByType<Box3DPayloadRegistry>(); // lazy once
            if (active != m_speedFeedbackActive)
            {
                if (!active) mxmAnimator.UserPlaybackSpeedMultiplier = 1f; // leave clean
                m_speedFeedbackActive = active;
                if (!active) return;
            }
            if (!active) return;

            // Rig COM velocity: mass-weighted over dynamic bodies + registered
            // gameplay payloads (stride-averaged by construction - no gait
            // ripple). Payloads move with the rig, so their velocity tracks;
            // including them keeps the MASS weighting honest (the true
            // dynamics carry the payload).
            var bodies = rig.Bodies;
            float totalMass = 0f;
            float3 vCom = float3.zero;
            for (int i = 0; i < bodies.Count; i++)
            {
                if (bodies[i] == null) continue;
                var b = bodies[i].Body;
                if (!b.IsValid || b.Type != Box3D.BodyType.Dynamic) continue;
                float m = b.GetMass();
                vCom += (float3)b.GetLinearVelocity() * m;
                totalMass += m;
            }
            if (totalMass <= 0f) return;
            vCom /= totalMass;
            if (payloadRegistry != null)
            {
                var ps = payloadRegistry.Payloads;
                for (int i = 0; i < ps.Count; i++)
                {
                    var pb = ps[i].body;
                    if (pb == null) continue;
                    var b = pb.Body;
                    if (!b.IsValid || b.Type != Box3D.BodyType.Dynamic) continue;
                    float m = b.GetMass();
                    vCom += (float3)b.GetLinearVelocity() * m;
                    totalMass += m;
                }
                if (totalMass > 0f) vCom /= totalMass;
            }
            vCom.y = 0f; // ground-plane

            // Animation reference velocity: the same baked-MM source as
            // GetRootDelta (playback-speed scaled + warp rate), ground-plane.
            float eff = mxmAnimator.DesiredPlaybackSpeed
                      * mxmAnimator.UserPlaybackSpeedMultiplier;
            float3 vAnim = math.mul((quaternion)m_curRootRot,
                (float3)mxmAnimator.BodyVelocity * eff);
            float cdt = mxmAnimator.CurrentDeltaTime;
            if (cdt > 1e-4f)
                vAnim += (float3)mxmAnimator.PosWarpThisFrame / cdt;
            vAnim.y = 0f;

            // Signed phase error along the animated root's forward.
            float3 fwd = math.normalizesafe(
                math.mul((quaternion)m_curRootRot, math.forward()), float3.zero);
            fwd.y = 0f;
            float fwdLen = math.length(fwd);
            if (fwdLen < 1e-4f) return;
            float err = math.dot(vAnim - vCom, fwd / fwdLen);

            // Angular analog: yaw-rate error around world up. Animated rate
            // from the root rotation delta between fixed polls (includes MxM
            // warp/turn - the poll is one fixed step); physical rate from the
            // rig root body's angular velocity Y component (physics truth).
            // A body whose spin lags the clip gets slowed down globally, so
            // planted feet stop being swept out from under the turn.
            float yawErr = 0f;
            if (yawFeedbackGain > 0f)
            {
                float3 fwdPrev = math.normalizesafe(
                    math.mul(math.normalizesafe((quaternion)m_prevRootRot, quaternion.identity), math.forward()), float3.zero);
                float animYawRate = 0f;
                if (fwdLen > 1e-4f)
                {
                    fwdPrev.y = 0f;
                    float3 fpn = math.normalizesafe(fwdPrev, float3.zero);
                    if (math.lengthsq(fpn) > 0.5f)
                    {
                        float cy = fpn.x * fwd.x + fpn.z * fwd.z;
                        float sy = fpn.x * fwd.z - fpn.z * fwd.x;
                        animYawRate = math.atan2(sy, cy) / dt;
                    }
                }
                float physYawRate = 0f;
                if (rig != null && rig.Root != null && rig.Root.Body.IsValid)
                    physYawRate = rig.Root.Body.GetAngularVelocity().y;
                yawErr = animYawRate - physYawRate;
            }

            // Low-pass the error, then modulate the playback multiplier with a
            // dead-zone and a hard clamp. NOTE: eff above uses the PREVIOUS
            // multiplier; the loop gain is small (max +/- maxSpeed), so this
            // one-step lag is stable by design.
            m_speedErrF = math.lerp(m_speedErrF, err,
                1f - math.exp(-math.max(speedFeedbackSmoothing, 0.01f) * dt));
            float mult = 1f;
            float e = m_speedErrF;
            if (math.abs(e) > speedFeedbackDeadzone)
                mult -= speedFeedbackGain * (math.abs(e) - speedFeedbackDeadzone)
                      * math.sign(e);

            // Angular feedback, multiplicative with the linear term: the two
            // errors are independent phase metrics, so they compose as factors
            // rather than adding magnitudes.
            float eYaw = math.abs(yawErr) - math.max(yawFeedbackDeadzone, 0f);
            if (eYaw > 0f)
                mult *= 1f - math.clamp(yawFeedbackGain * eYaw * math.sign(yawErr), -0.5f, 0.5f);
            // Absolute limits are the only clamp: they ARE the feedback
            // authority window (0 = unbounded on that side).
            if (playbackSpeedMin > 0f) mult = math.max(mult, playbackSpeedMin);
            if (playbackSpeedMax > 0f) mult = math.min(mult, playbackSpeedMax);
            mxmAnimator.UserPlaybackSpeedMultiplier = mult;
        }

        // ---------------------------------------------------------- IWorldPoseSource

        private float3 Extrapolate(int idx, float dt)
        {
            float3 cur = m_cur[idx];
            if (!m_hasPrev) return cur;
            float3 prev = m_prev[idx];
            return cur + (cur - prev) * (dt / m_evalInterval);
        }

        /// <summary>Root velocity look-ahead: the clip's interpolated root
        /// velocity a_dt seconds ahead of playback, scaled by the effective
        /// playback speed, rotated into world, plus the warp rate. Same
        /// convention as GetRootDelta's position branch. False = no MxM
        /// animator bound.</summary>
        public bool GetRootVelocityAhead(float dt, out float3 vel)
        {
            vel = default;
            if (mxmAnimator == null) return false;

            float eff = mxmAnimator.DesiredPlaybackSpeed
                      * mxmAnimator.UserPlaybackSpeedMultiplier;
            float3 v = math.mul((quaternion)m_curRootRot,
                (float3)mxmAnimator.GetRootVelocityAhead(dt) * eff);
            float cdt = mxmAnimator.CurrentDeltaTime;
            if (cdt > 1e-4f)
                v += (float3)mxmAnimator.PosWarpThisFrame / cdt;
            vel = v;
            return true;
        }

        public float3 GetPivot(int i, float dt) => Extrapolate(i * 3 + 0, dt);
        public float3 GetTip(int i, float dt) => Extrapolate(i * 3 + 1, dt);
        public float3 GetRoll(int i, float dt) => Extrapolate(i * 3 + 2, dt);

        public void GetRootDelta(out float3 posDelta, out quaternion rotDelta, float dt)
        {
            posDelta = default;
            rotDelta = quaternion.identity;
            if (!m_hasPrev) return;

            // Rotation: exact one-fixed-step transform delta (includes MxM
            // root motion, trajectory warp and turning).
            rotDelta = math.normalizesafe(
                math.mul((quaternion)m_curRootRot, math.inverse((quaternion)m_prevRootRot)),
                quaternion.identity);

            // Position: MxM's baked per-pose velocity when available - clean
            // (single source of truth, no double differentiation), already
            // interpolated across the blend. It is clip-space root-local and
            // UNscaled by playback speed, so scale by the effective speed and
            // rotate into world; event warp is added as a rate.
            if (useMmRootVelocity && mxmAnimator != null)
            {
                float eff = mxmAnimator.DesiredPlaybackSpeed
                          * mxmAnimator.UserPlaybackSpeedMultiplier;
                float3 v = math.mul((quaternion)m_curRootRot,
                    (float3)mxmAnimator.BodyVelocity * eff);
                float cdt = mxmAnimator.CurrentDeltaTime;
                if (cdt > 1e-4f)
                    v += (float3)mxmAnimator.PosWarpThisFrame / cdt;
                posDelta = v * dt;
                return;
            }

            posDelta = new float3(m_curRootPos.xyz - m_prevRootPos.xyz);
        }

        public void GetRootPose(out float3 pos, out quaternion rot, float dt)
        {
            // Return the ANIMATED root (reference for remap axis / lean); the
            // drive targets are anchored to physics, but GetRootPose is read by
            // the lean controller for axis remapping against the authored root
            // frame. If a consumer needs the physics-root pose for comparison,
            // it reads rig.Root.Body directly; this stays truthful to the
            // animation reference.
            pos = (float3)m_curRootPos.xyz;
            rot = math.normalizesafe((quaternion)m_curRootRot, quaternion.identity);
        }

        public void GetNodeFrame(int i, out float3 pivotPos, out quaternion boneRot, float dt)
        {
            // Pivot: extrapolated (same as GetPivot). Bone rot: current poll
            // PLUS one-step ring angular velocity - parity with the point
            // extrapolation, removes the rotational reference lag.
            pivotPos = Extrapolate(i * 3 + 0, dt);
            boneRot = math.normalizesafe((quaternion)m_curRots[i], quaternion.identity);
            if (extrapolateBoneRot && m_hasPrev)
            {
                quaternion prev = math.normalizesafe((quaternion)m_prevRots[i], quaternion.identity);
                float3 w = RotationVector(
                    math.mul(boneRot, math.inverse(prev))) / m_evalInterval;
                boneRot = math.normalizesafe(math.mul(boneRot, new quaternion(
                    math.float4(w * (0.5f * dt), 1f))), boneRot);
            }
        }

        /// <summary>Tip-bone angular velocity from the ring (world, rad/s).
        /// First-derivative data at poll rate - no extrapolation lead.</summary>
        public float3 GetNodeAngVel(int i)
        {
            if (!m_hasPrev) return default;
            quaternion cur = math.normalizesafe((quaternion)m_curRots[i], quaternion.identity);
            quaternion prev = math.normalizesafe((quaternion)m_prevRots[i], quaternion.identity);
            return RotationVector(math.mul(cur, math.inverse(prev))) / m_evalInterval;
        }

        private static float3 RotationVector(quaternion q)
        {
            q = math.normalizesafe(q, quaternion.identity);
            if (q.value.w < 0f) q.value = -q.value;
            float s = math.length(q.value.xyz);
            return s < 1e-5f ? 2f * q.value.xyz
                             : q.value.xyz * (2f * math.atan2(s, q.value.w) / s);
        }
    }
}