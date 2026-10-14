using System;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>One library entry: a full wave parameter set (the agent
    /// GroupState coordinates). Captured from manual tuning sessions; the
    /// blend preview shows what the agent's gate weights will produce.
    /// Structural signs (wavenumber/twist) must agree across entries
    /// intended to blend together - never blend across a sign flip.</summary>
    [Serializable]
    public struct WavePrimitive
    {
        public string name;
        [Header("shared")]
        public float bias, ampTip, wavenumber, twistAmp, twistPhase, ampBase;
        [Header("per limb (L/R)")]
        public float freqL, freqR, leanL, leanR, phaseL, phaseR;
        [Header("roll L/R - whole-limb rotation about the base axis (mirrored per side)")]
        public float rollL, rollR;
        [Header("pose roll L/R (rad) - rotate the FINAL pose about the root->tip chord; lateral pose, twist untouched")]
        public float poseRollL, poseRollR;
        [Header("arm-decoupled (fall back to the leg value when 0)")]
        public float armLeanL, armLeanR, armRollL, armRollR;
        public float armPoseRollL, armPoseRollR;

        public static float ArmVal(float arm, float leg) => arm != 0f ? arm : leg;
        public float ArmLeanL => ArmVal(armLeanL, leanL);
        public float ArmLeanR => ArmVal(armLeanR, leanR);
        public float ArmRollL => ArmVal(armRollL, rollL);
        public float ArmRollR => ArmVal(armRollR, rollR);
        public float ArmPoseRollL => ArmVal(armPoseRollL, poseRollL);
        public float ArmPoseRollR => ArmVal(armPoseRollR, poseRollR);
    }

    /// <summary>The blend semantics shared by the authoring tester and the
    /// training agent - ONE implementation, so a gait captured in the tester
    /// means exactly the same thing inside the agent's gate layer.</summary>
    public static class WaveBlend
    {
        /// <summary>Normalized linear blend in parameter space. Weights are
        /// normalized over ALL entries (negatives clamped to 0), sized to the
        /// library (missing weights count as 0).</summary>
        public static WavePrimitive Mix(WavePrimitive[] lib, float[] w)
        {
            var p = new WavePrimitive { name = "blend" };
            float s = 1e-5f;
            for (int i = 0; i < lib.Length; i++) s += math.max(i < w.Length ? w[i] : 0f, 0f);
            // Frequencies blend MAGNITUDE-only, sign resolved by dominance:
            // lerping signed freqs straight through zero desynchronizes the
            // pair (one limb decelerates while the other holds tempo) whenever
            // entries disagree on a limb's sign. speed = lerped |f|; dir = the
            // dominant side's sign; exact tie -> frozen (deterministic).
            float speedL = 0f, dirL = 0f, speedR = 0f, dirR = 0f;
            for (int i = 0; i < lib.Length; i++)
            {
                float wi = i < w.Length ? w[i] : 0f;
                float weight = math.max(wi, 0f) / s;
                var q = lib[i];
                p.bias += q.bias * weight; p.ampTip += q.ampTip * weight; p.ampBase += q.ampBase * weight;
                p.wavenumber += q.wavenumber * weight; p.twistAmp += q.twistAmp * weight; p.twistPhase += q.twistPhase * weight;
                p.leanL += q.leanL * weight; p.leanR += q.leanR * weight;
                p.armLeanL += q.armLeanL * weight; p.armLeanR += q.armLeanR * weight;
                p.rollL += q.rollL * weight; p.rollR += q.rollR * weight;
                p.armRollL += q.armRollL * weight; p.armRollR += q.armRollR * weight;
                p.poseRollL += q.poseRollL * weight; p.poseRollR += q.poseRollR * weight;
                p.armPoseRollL += q.armPoseRollL * weight; p.armPoseRollR += q.armPoseRollR * weight;
                p.phaseL += q.phaseL * weight; p.phaseR += q.phaseR * weight;
                speedL += math.abs(q.freqL) * weight; dirL += math.sign(q.freqL) * weight;
                speedR += math.abs(q.freqR) * weight; dirR += math.sign(q.freqR) * weight;
            }
            p.freqL = math.abs(dirL) > 1e-4f ? math.sign(dirL) * speedL : 0f;
            p.freqR = math.abs(dirR) > 1e-4f ? math.sign(dirR) * speedR : 0f;
            return p;
        }
    }

    /// <summary>Shared primitive library asset: authored in the tester's
    /// inspector, consumed by the training agent's gate layer. One asset = one
    /// source of truth; the tester's copy command overwrites the entries.</summary>
    [CreateAssetMenu(fileName = "WavePrimitiveLibrary", menuName = "Box3DArticulation/Wave Primitive Library")]
    public class WavePrimitiveLibrary : ScriptableObject
    {
        public WavePrimitive[] entries = new WavePrimitive[0];
    }
}