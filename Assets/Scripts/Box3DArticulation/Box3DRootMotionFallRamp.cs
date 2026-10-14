using MxM;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Fall ramp for MxM root motion. As the god forces fall detector's
    /// <see cref="Box3DGodForces.PreFallProgress"/> rises, scales the
    /// <see cref="MxMRootMotionApplicator"/>'s move scale (its public
    /// SpeedScale property) down toward a fallen fraction - the animated
    /// ghost stops striding forward while the body falls, instead of
    /// dragging its root (and the goal trajectory) through the fall.
    ///
    /// Decoration only: the plugin component is untouched; the user's
    /// inspector SpeedScale is treated as the healthy base and restored on
    /// disable. Rotation deltas from the applicator are NOT scaled (the
    /// plugin applies them unscaled) - turn-in-place warp keeps working
    /// during a fall, which is usually desirable.
    /// </summary>
    [DisallowMultipleComponent]
    public class Box3DRootMotionFallRamp : MonoBehaviour
    {
        [SerializeField, Tooltip("God forces component publishing PreFallProgress. Auto-found when null.")]
        private Box3DGodForces godForces;
        [SerializeField, Range(0f, 1f), Tooltip("Move-scale fraction at full fall progress (0 = root motion fully let go while fallen, 1 = no ramp). The healthy value is the applicator's own SpeedScale.")]
        private float fallenMoveScale = 0f;
        [SerializeField, Tooltip("Ramp smoothing rate (1/s): exponential blend between healthy and fallen scale, applied both ways. High = snappy let-go, low = gradual drift.")]
        private float rampRate = 6f;

        private MxMRootMotionApplicator m_applicator;
        private float m_baseScale = 1f;   // user's healthy SpeedScale, snapshotted on enable
        private float m_progress;         // last PreFallProgress value
        private float m_smoothed;         // low-passed ramp value
        private bool m_subscribed;

        private void OnEnable()
        {
            m_applicator = GetComponentInChildren<MxMRootMotionApplicator>();
            if (m_applicator == null)
            {
                Debug.LogWarning("[Box3DRootMotionFallRamp] no MxMRootMotionApplicator found - disabled", this);
                enabled = false;
                return;
            }
            if (godForces == null) godForces = FindFirstObjectByType<Box3DGodForces>();
            m_baseScale = m_applicator.SpeedScale;
            m_progress = 0f;
            m_smoothed = 0f;
            if (godForces != null)
            {
                godForces.PreFallProgress += OnPreFallProgress;
                m_subscribed = true;
            }
        }

        private void OnDisable()
        {
            if (m_subscribed && godForces != null)
                godForces.PreFallProgress -= OnPreFallProgress;
            m_subscribed = false;
            if (m_applicator != null) m_applicator.SpeedScale = m_baseScale;
        }

        private void OnPreFallProgress(float progress) => m_progress = progress;

        private void Update()
        {
            if (m_applicator == null) return;
            float a = 1f - math.exp(-math.max(rampRate, 0.01f) * Time.deltaTime);
            m_smoothed = math.lerp(m_smoothed, m_progress, a);
            m_applicator.SpeedScale = m_baseScale
                * math.lerp(1f, fallenMoveScale, math.saturate(m_smoothed));
        }
    }
}