using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>Tuning-mode helper: pushes roll / poseRoll / lean in the AGENT's
    /// output space onto all four limbs. One slider value per channel plus a
    /// per-channel mirror switch that negates the LEFT side on push - exactly
    /// the mapping the agent's PushLimb uses (sideSign: left -1, right +1).
    /// Editing the probes' serialized fields instead does NOT give symmetric
    /// actuation: serialized values are frame-local (no side mirroring) and
    /// poseRoll is overwritten by the agent every physics step anyway.
    /// Fields are re-read every LateUpdate, so mirror switches and values can
    /// be flipped live during play on the suspended rig. Owns ONLY the three
    /// channels it pushes: every other wave param passes through untouched
    /// (read via GetRuntimeParams, written back verbatim).</summary>
    public sealed class ConventionTuner : MonoBehaviour
    {
        [SerializeField] private PinnedBipedDemo demo;

        [Header("Channel values (rad) - one value, applied to all four limbs")]
        [SerializeField, Range(-3.14f, 3.14f)] private float lean = 0f;
        [SerializeField, Range(-3.14f, 3.14f)] private float roll = 0f;
        [SerializeField, Range(-3.14f, 3.14f)] private float poseRoll = 0f;

        [Header("Mirror switches (negate LEFT side on push)")]
        [SerializeField, Tooltip("Matches the agent: left negated, so +lean = outward tilt on BOTH legs.")]
        private bool mirrorLean = true;
        [SerializeField, Tooltip("Matches the agent: roll's axis (base direction) is shared L/R, so left = -value gives mirrored bend planes from one value - L/R-symmetric envelopes. OFF = same world spin on both sides.")]
        private bool mirrorRoll = true;
        [SerializeField, Tooltip("Matches the agent: poseRoll's axis (projected chord) is mirrored L/R, and mirroring flips handedness - so left = -value yields symmetric swing (both elbows the same world direction). OFF = antisymmetric, gait-like opposition.")]
        private bool mirrorPoseRoll = true;

        private BackboneWaveProbe m_legL, m_legR, m_armL, m_armR;

        private void Start()
        {
            if (demo == null) demo = FindAnyObjectByType<PinnedBipedDemo>();
            if (demo == null) { enabled = false; return; }
            m_legL = demo.LegL; m_legR = demo.LegR;
            m_armL = demo.ArmL; m_armR = demo.ArmR;
        }

        private void LateUpdate()
        {
            float leanL = mirrorLean ? -lean : lean;
            float rollL = mirrorRoll ? -roll : roll;
            float poseRollL = mirrorPoseRoll ? -poseRoll : poseRoll;
            Push(m_legL, leanL, rollL, poseRollL);
            Push(m_legR, lean, roll, poseRoll);
            Push(m_armL, leanL, rollL, poseRollL);
            Push(m_armR, lean, roll, poseRoll);
        }

        private static void Push(BackboneWaveProbe p, float lean, float roll, float poseRoll)
        {
            if (p == null) return;
            var cur = p.GetRuntimeParams(); // pass-through: only our 3 channels change
            p.SetRuntimeParams(cur.freqHz, cur.ampTip, cur.bias, cur.wavenumber,
                float.NaN, float.NaN, cur.ampBase, roll, poseRoll);
            p.BaseDirXTilt = lean;
        }
    }
}
