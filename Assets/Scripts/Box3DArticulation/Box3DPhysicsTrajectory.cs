using MxM;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Physics -> MxM trajectory feedback. Decorator over the real trajectory
    /// generator: biases the DESIRED future goal points toward the physical
    /// rig's center of mass so clip selection anticipates the simulated body
    /// (dragged sideways / lagging / pushed => MxM picks recovery motion).
    ///
    /// How it works: MxMAnimator polls GetCurrentGoal() each update and treats
    /// the returned array as read-only, so this component never mutates the
    /// inner generator's state - it keeps an unbiased snapshot (refreshed on
    /// SetGoal/CopyGoalFromPose/ResetMotion) and returns a biased clone.
    /// Past points (prediction time <= 0) are never biased - the recorded
    /// history must stay truthful.
    ///
    /// Setup: place this component ABOVE the real trajectory generator on the
    /// MxM character. MxMAnimator uses the first enabled IMxMTrajectory.
    /// Facing is NOT biased (it stays input-driven); position only.
    /// </summary>
    [DefaultExecutionOrder(-115)] // inner generator updates its goal before MxM polls
    public sealed class Box3DPhysicsTrajectory : MonoBehaviour, IMxMTrajectory
    {
        [SerializeField, Tooltip("The real trajectory generator to decorate (its goal gets biased).")]
        private MxMTrajectoryGenerator generator;
        [SerializeField, Tooltip("Rig builder of the physical character - source of the COM.")]
        private Box3DRigBuilder rig;
        [SerializeField, Range(0f, 1f), Tooltip("How strongly the farthest future point is pulled toward the physical COM (0 = raw input trajectory, 1 = point fully re-anchored to the COM). Ramp: first future point 0 strength, farthest point full strength.")]
        private float biasStrength = 0.5f;
        [SerializeField, Range(0.25f, 4f), Tooltip("Falloff of the bias over the prediction horizon: s = (t/tMax)^power. < 1 front-loads the bias (trajectory bends early, near-term matching is affected sooner); > 1 concentrates it on the far points (input-faithful early trajectory). 1 = linear.")]
        private float biasFalloff = 1f;
        [SerializeField, Tooltip("Hard clamp on the positional bias (m). Keeps input authority - feedback can never fully own the desired path.")]
        private float biasMax = 1.5f;
        [SerializeField, Tooltip("COM spatial error low-pass rate (1/s). Higher = snappier bias, more gait jitter.")]
        private float errorSmoothing = 3f;

        private IMxMTrajectory m_inner;
        private float[] m_predictionTimes;
        private TrajectoryPoint[] m_biased;     // reused return buffer (copied fresh from the inner goal every call)
        private float3 m_errF;                  // filtered COM - character-root error (world, XZ)
        private bool m_hasErr;

        private void OnEnable()
        {
            m_inner = generator != null ? generator : GetComponent<MxMTrajectoryGenerator>();
            if (m_inner == null)
            {
                Debug.LogWarning("[Box3DPhysicsTrajectory] No MxMTrajectoryGenerator found to decorate.", this);
                enabled = false;
                return;
            }
            // Must be the first enabled IMxMTrajectory on this GameObject, or
            // MxMAnimator will pick the raw generator and no bias happens.
            var all = GetComponents<IMxMTrajectory>();
            if (all.Length == 0 || all[0] != this)
                Debug.LogWarning("[Box3DPhysicsTrajectory] Place this component ABOVE the trajectory generator in the component list, or MxM will use the un-biased goal.", this);
        }

        private void OnDisable()
        {
            // MxM falls back to the first enabled generator - the inner one.
            // If we were above it, MxM re-picks it only on start, so warn.
        }

        private void Update()
        {
            // Sample the COM error at render rate for the low-pass; the bias
            // itself is applied in GetCurrentGoal (polled by MxMAnimator).
            if (!m_hasErr && rig != null && m_inner != null)
                m_errF = float3.zero;
            float3 err = ComputeComError();
            if (m_hasErr)
                m_errF = math.lerp(m_errF, err, 1f - math.exp(-errorSmoothing * Time.deltaTime));
            else
            {
                m_errF = err;
                m_hasErr = true;
            }
        }

        private float3 ComputeComError()
        {
            if (rig == null || m_inner == null) return default;
            var t = m_inner.GetTransform();
            if (t == null) return default;
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
            if (totalMass <= 0f) return default;
            com /= totalMass;
            float3 root = (float3)t.position;
            return new float3(com.x - root.x, 0f, com.z - root.z); // world, XZ only
        }

        // --- IMxMTrajectory ------------------------------------------------

        public TrajectoryPoint[] GetCurrentGoal()
        {
            if (m_inner == null) return null;
            var raw = m_inner.GetCurrentGoal();
            if (raw == null || raw.Length == 0) return raw;

            // Copy fresh EVERY call: the inner generator rewrites its goal
            // every update (input-following). Caching a snapshot would freeze
            // the desired trajectory and dead-lock the character.
            int n = raw.Length;
            if (m_biased == null || m_biased.Length != n)
                m_biased = new TrajectoryPoint[n];
            System.Array.Copy(raw, m_biased, n);

            if (m_predictionTimes == null || m_predictionTimes.Length != n)
                return m_biased; // requirements not pushed yet - pass through

            // Per-point strength ramp: 0 at the first future point, full at
            // the farthest - near-term points stay input-faithful because
            // that is where matching is most sensitive.
            int firstFuture = -1, lastFuture = -1;
            for (int i = 0; i < n; i++)
            {
                if (m_predictionTimes[i] > 0f)
                {
                    if (firstFuture < 0) firstFuture = i;
                    lastFuture = i;
                }
            }
            if (firstFuture >= 0 && biasStrength > 0f && rig != null && m_hasErr)
            {
                float span = math.max(lastFuture - firstFuture, 1);
                for (int i = firstFuture; i <= lastFuture; i++)
                {
                    float s = biasStrength * math.pow(
                        math.saturate((float)(i - firstFuture) / span), biasFalloff);
                    float3 b = math.clamp(m_errF * s, -biasMax, biasMax);
                    m_biased[i].Position = m_biased[i].Position
                        + new Vector3(b.x, 0f, b.z);
                }
            }
            return m_biased;
        }

        public Transform GetTransform() => m_inner != null ? m_inner.GetTransform() : null;

        public void SetGoalRequirements(float[] a_predictionTimes)
        {
            m_predictionTimes = a_predictionTimes;
            m_inner?.SetGoalRequirements(a_predictionTimes);
        }

        public void SetGoal(TrajectoryPoint[] a_goal)
        {
            m_inner?.SetGoal(a_goal);
        }

        public void CopyGoalFromPose(ref PoseData a_poseData)
        {
            m_inner?.CopyGoalFromPose(ref a_poseData);
        }

        public void Pause() => m_inner?.Pause();
        public void UnPause() => m_inner?.UnPause();
        public void ResetMotion(float a_rotation = 0f)
        {
            m_hasErr = false;
            m_inner?.ResetMotion(a_rotation);
        }

        public bool HasMovementInput() => m_inner != null && m_inner.HasMovementInput();
        public bool IsEnabled() => m_inner != null && m_inner.IsEnabled() && enabled;
    }
}
