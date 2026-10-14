using Box3D;
using Box3D.Hybrid;
using MxM;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Physics -> MxM pose-cost feedback. After MxMAnimator computes the
    /// runtime query pose (m_curInterpolatedPose, root-relative joint
    /// positions/velocities of the MatchBones subset), this component biases
    /// selected joints toward the physical rig's actual body poses - same
    /// trick as the trajectory bias, applied to the pose cost instead of the
    /// goal. blend 0 = stock MxM.
    ///
    /// JointData conventions (mirrored from the MxM pre-processor):
    ///   Position  = root.InverseTransformPoint(jointWorld)
    ///   Velocity  = root.InverseTransformVector(jointWorldVel) - LocalVelocity
    /// (velocity is "body velocity dependent": root motion excluded).
    ///
    /// The node->body map is captured at BIND time (offsets are constants of
    /// the bind) via the driver, and the animator's PoseBias hook is
    /// registered here. Call BuildFromDriver after the driver binds (the
    /// driver can do this itself when autoBuild is set).
    ///
    /// Root-frame bias (rootPositionBlend/rootRotationBlend/rootVelocityBlend):
    /// the query cloud can be re-expressed in a virtual frame blended toward
    /// the physical pelvis, and LocalVelocity replaced by the measured pelvis
    /// velocity - making fallen baked poses true nearest neighbors of a fallen
    /// body. The goal trajectory is NOT touched (it is regenerated after this
    /// hook against the animated root and consumed by the trajectory job in
    /// the ghost frame; the two cost channels frame independently).
    /// </summary>
    public sealed class Box3DPhysicsPoseBias : MonoBehaviour, IPoseBias
    {
        [SerializeField, Tooltip("How strongly the query pose is pulled toward the physical rig (0 = stock MxM, 1 = matching sees the physics pose outright).")]
        private float blend = 0f;
        [SerializeField, Tooltip("Hard clamp on the per-joint position bias (m, root-relative). Keeps the query reachable.")]
        private float posBiasMax = 0.5f;
        [SerializeField, Tooltip("Hard clamp on the per-joint velocity bias (m/s, root-relative, root motion excluded). 0 = don't bias velocities.")]
        private float velBiasMax = 1f;
        [SerializeField, Tooltip("Position-bias dead-band (m, root-relative): inside this per-joint error the bias injects NOTHING (animation owns the pose); beyond it only the EXCESS error is injected. Makes the bias conditional - animation wins unless it disagrees. Set above normal gait noise (animation vs physics jitter) but below the mismatch you want caught (gravity lean). 0 = unconditional injection.")]
        private float posBiasDeadband = 0f;
        [SerializeField, Tooltip("Velocity-bias dead-band (m/s, root-relative, root motion excluded). Same conditional semantics as posBiasDeadband, for the velocity channels.")]
        private float velBiasDeadband = 0f;
        [SerializeField, Tooltip("Node names (rig config names) mapped to JointsData indices. Index i = JointsData[i] = MxM's match bone i (MatchBones order). Empty string = that joint is not biased. Names must match the demo skeleton's match bones - rig nodes are matched by name via the driver.")]
        private string[] nodeByJoint = System.Array.Empty<string>();
        [SerializeField, Range(0f, 1f), Tooltip("Root-frame position bias: how far the query frame origin moves from the animated root to the physical pelvis (1 = fully pelvis-anchored). Makes the measured cloud correspond to how baked poses frame their data (the baked root falls WITH the body in source clips; the animated root does not). Near-neutral in normal locomotion.")]
        private float rootPositionBlend = 0f;
        [SerializeField, Range(0f, 1f), Tooltip("Root-frame ROTATION bias: slerp from animated root rotation to physical pelvis rotation. NOISY during falls (pelvis rolls on contacts) and tilts the pose cloud - keep 0 unless fallen-pose matching needs the roll/pitch correspondence.")]
        private float rootRotationBlend = 0f;
        [SerializeField, Range(0f, 1f), Tooltip("Root velocity bias: replaces the query's LocalVelocity (root velocity channel) with the measured pelvis velocity. Required for fallen-pose entry: baked fallen poses carry ~zero root velocity while the animated LocalVelocity keeps reporting clip motion.")]
        private float rootVelocityBlend = 0f;
        [SerializeField, Tooltip("Per-joint bias multipliers (parallel to nodeByJoint). Empty = all 1.")]
        private float[] jointWeights = System.Array.Empty<float>();
        [SerializeField, Tooltip("Per-joint endpoint flags (parallel to nodeByJoint). True = the bias point is the body's distal tip (body-local -Y * half-extent, matching the builder's bone-axis convention) instead of the node pivot. Use for terminal match bones with no rig body (hands, toes).")]
        private bool[] endpointByJoint = System.Array.Empty<bool>();
        [SerializeField, Tooltip("Driver supplying bind-time node frames. The map is built automatically right after the driver binds.")]
        private HumanoidMotionDriver driver;

        private Box3DBody m_rootBody;           // physical root (pelvis) - live frame source

        private struct BoneMap
        {
            public Box3DBody body;
            public float3 offset;   // node pivot, body-local at bind
            public float weight;
        }
        private BoneMap[] m_maps = System.Array.Empty<BoneMap>();
        private bool m_built;

        private void OnEnable()
        {
            var animator = GetComponent<MxMAnimator>();
            if (animator != null) animator.PoseBias = this;
        }

        private void OnDisable()
        {
            var animator = GetComponent<MxMAnimator>();
            if (animator != null && animator.PoseBias == this) animator.PoseBias = null;
        }

        /// <summary>Build the joint->body map from the driver's bind. Must run
        /// while the rig is bound (physics bodies at their spawn pose).</summary>
        public void BuildFromDriver(HumanoidMotionDriver a_driver)
        {
            m_built = false;
            m_maps = System.Array.Empty<BoneMap>();
            m_rootBody = a_driver != null && a_driver.Rig != null ? a_driver.Rig.Root : null;
            if (a_driver == null || nodeByJoint == null || nodeByJoint.Length == 0) return;

            m_maps = new BoneMap[nodeByJoint.Length];
            int built = 0;
            for (int i = 0; i < nodeByJoint.Length; i++)
            {
                string nodeName = nodeByJoint[i];
                if (string.IsNullOrEmpty(nodeName)) continue;
                if (!a_driver.TryGetBindNodeFrame(nodeName, out var body,
                    out float3 pivotW, out quaternion spawnRotW))
                {
                    Debug.LogWarning($"[Box3DPhysicsPoseBias] no rig node '{nodeName}' for joint {i} - joint left unbiased", this);
                    continue;
                }
                // Bias point in body-local space: node pivot by default, distal
                // tip when flagged. Tip = body frame -Y * half-extent - the same
                // convention the foot stabilizer uses for tipLocal. Extents come
                // from the rig node that owns the body.
                float3 localPoint = math.mul(math.inverse(spawnRotW),
                    pivotW - (float3)body.Body.Position);
                bool endpoint = endpointByJoint != null && i < endpointByJoint.Length && endpointByJoint[i];
                if (endpoint)
                {
                    if (a_driver.Rig != null && a_driver.Rig.TryGetNodeSize(body, out var sz))
                        localPoint = new float3(0f, -0.5f * sz.y, 0f);
                    else
                        Debug.LogWarning($"[Box3DPhysicsPoseBias] endpoint flag on joint {i} ('{nodeName}') but no rig node size found - using node pivot", this);
                }
                m_maps[built++] = new BoneMap
                {
                    body = body,
                    // Bias point, body-local at bind: node pivot by default, distal
                    // tip when flagged - rotated by the live body rotation at query
                    // time to recover the measured point.
                    offset = localPoint,
                    weight = jointWeights != null && i < jointWeights.Length
                        ? jointWeights[i] : 1f,
                };
            }
            System.Array.Resize(ref m_maps, built);
            m_built = built > 0;
            Debug.Log($"[Box3DPhysicsPoseBias] mapped {built}/{nodeByJoint.Length} joints", this);
        }

        // --- IPoseBias ------------------------------------------------------

        public void BiasPose(ref PoseData pose)
        {
            if (!m_built || blend <= 0f) return;
            var joints = pose.JointsData;
            if (joints == null) return;

            var t = transform;
            float3 rootPos = t.position;
            quaternion rootRot = t.rotation;

            // Virtual query frame: blend from the animated root frame toward
            // the physical pelvis frame. The baked set frames every pose in
            // the SOURCE clip's root frame - which falls WITH the body in
            // fallen clips. Expressing the measured cloud in the pelvis frame
            // makes fallen baked poses true nearest neighbors. The two cost
            // channels frame independently (pose job: this cloud; trajectory
            // job: the ghost-frame goal, generated after this hook), so the
            // goal trajectory needs no re-expression here.
            float3 framePos = rootPos;
            quaternion frameRot = rootRot;
            float3 pelvisVel = float3.zero;
            bool havePelvis = m_rootBody != null && m_rootBody.Body.IsValid;
            if (havePelvis)
            {
                var rb = m_rootBody.Body;
                if (rootPositionBlend > 0f) framePos = math.lerp(rootPos, (float3)rb.Position, rootPositionBlend);
                if (rootRotationBlend > 0f) frameRot = math.normalizesafe(
                    math.slerp(rootRot, (quaternion)rb.Rotation, rootRotationBlend), rootRot);
                pelvisVel = rb.GetLinearVelocity();
            }
            quaternion invFrame = math.inverse(frameRot);

            // Root velocity channel: baked LocalVelocity is the clip root's
            // velocity in root frame; the faithful analog is the measured
            // pelvis velocity in the (blended) frame. Computed FIRST so the
            // joint velocities below subtract the SAME value that lands in
            // the velocity lanes (bake convention: jointVel - LocalVelocity).
            float3 localVel = (float3)pose.LocalVelocity;
            float3 localVelUsed = havePelvis && rootVelocityBlend > 0f
                ? math.lerp(localVel, math.mul(invFrame, pelvisVel), rootVelocityBlend)
                : localVel;

            for (int i = 0; i < m_maps.Length; i++)
            {
                var map = m_maps[i];
                if (map.body == null) continue;
                var b = map.body.Body;
                if (!b.IsValid || b.Type != Box3D.BodyType.Dynamic) continue;

                // Measured bias point in world, recovered from the body pose.
                float3 nodeW = b.Position + math.mul(b.Rotation, map.offset);
                // Point velocity: body linear + angular contribution at the
                // offset r (inert for COM-pivots, matters for tip offsets).
                float3 pointVel = b.GetLinearVelocity()
                    + math.cross(b.GetAngularVelocity(), math.mul(b.Rotation, map.offset));
                float3 relPos = math.mul(invFrame, nodeW - framePos);

                float w = blend * map.weight;
                float3 dPos = relPos - (float3)joints[i].Position;
                float dLen = math.length(dPos);
                // Dead-band: inject only the error EXCESS beyond the band.
                // Unconditional injection can feed back positive (bias leans
                // the query -> pick leans further -> measured leans further);
                // the band makes the loop negative - a query that already
                // agrees with the measurement shrinks the residual to zero.
                if (dLen <= posBiasDeadband) dPos = float3.zero;
                else if (posBiasDeadband > 0f)
                    dPos *= (dLen - posBiasDeadband) / dLen;
                dLen = math.length(dPos);
                if (posBiasMax > 0f && dLen > posBiasMax)
                    dPos *= posBiasMax / dLen;
                joints[i].Position = (float3)joints[i].Position + dPos * w;

                if (velBiasMax > 0f)
                {
                    // Mirror the bake: InverseTransformVector(worldVel) - LocalVelocity.
                    float3 relVel = math.mul(invFrame, pointVel) - localVelUsed;
                    float3 dVel = relVel - (float3)joints[i].Velocity;
                    float vLen = math.length(dVel);
                    if (vLen <= velBiasDeadband) dVel = float3.zero;
                    else if (velBiasDeadband > 0f)
                        dVel *= (vLen - velBiasDeadband) / vLen;
                    vLen = math.length(dVel);
                    if (vLen > velBiasMax) dVel *= velBiasMax / vLen;
                    joints[i].Velocity = (float3)joints[i].Velocity + dVel * w;
                }
            }

            if (localVelUsed.x != localVel.x || localVelUsed.y != localVel.y || localVelUsed.z != localVel.z)
                pose.LocalVelocity = localVelUsed;
        }
    }
}
