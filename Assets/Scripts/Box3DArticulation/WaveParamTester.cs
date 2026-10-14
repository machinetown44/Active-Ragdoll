using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>Authoring surface for the primitive-gate control scheme.
    /// Holds the wave primitive library and gate weights; blends in parameter
    /// space exactly as the agent's gate layer will, applies live modulation
    /// (tempo scale, lean/offset trims) and pushes to the probes. Attach to
    /// the PinnedBipedDemo object; probes auto-found by child name.</summary>
    public sealed class WaveParamTester : MonoBehaviour
    {

        [Header("Primitive library (blended in parameter space)")]
        [SerializeField] private WavePrimitive[] library = new WavePrimitive[]
        {
            new WavePrimitive { name = "idle",  bias = -0.03f, ampTip = 0.28f, wavenumber = 8.47f, twistAmp = 0.05f, twistPhase = 2.7f, ampBase = 0.28f },
            new WavePrimitive { name = "walkF", bias = -0.03f, ampTip = 0.6f,  wavenumber = 8.47f, twistAmp = 0.3f,  twistPhase = 2.7f, ampBase = 0.5f, freqL = 1.4f, freqR = 1.4f, phaseR = 1.57f },
            new WavePrimitive { name = "brace", bias = 0.1f,   ampTip = 0.15f, wavenumber = 10f,   twistAmp = 0f,    twistPhase = 0f,   ampBase = 0.15f },
        };

        [Header("Training handoff")]
        [SerializeField, Tooltip("Asset the agent's gate layer reads. 'Copy Library To Training Asset' overwrites its entries with the local library.")]
        private WavePrimitiveLibrary trainingLibrary;

        [Header("Gate weights (legs | arms) - the agent's blend surface")]
        [SerializeField, Tooltip("One softmax-ish weight per library entry; auto-sizes when the library grows. Values are normalized, so absolute size is irrelevant.")]
        private float[] wLegs = { 1f, 0f, 0f };
        [SerializeField]
        private float[] wArms = { 0.6f, 0.4f, 0f };

        [Header("Live modulation (the agent's scalar knobs)")]
        [SerializeField, Range(-3f, 3f), Tooltip("Global leg tempo scale on top of the blend. Negative reverses the wave.")]
        private float legFreqScale = 1f;
        [SerializeField, Range(-3f, 3f), Tooltip("Global arm tempo scale on top of the blend. Negative reverses the wave.")]
        private float armFreqScale = 1f;
        [SerializeField, Range(-1.2f, 1.2f), Tooltip("Added to the blended leg lean, per limb (before the demo's side-sign mirror).")]
        private float legLeanTrimL, legLeanTrimR;
        [SerializeField, Range(-1.53f, 1.53f), Tooltip("Added to the blended arm lean, per limb (before the demo's side-sign mirror). Near-full ROM: Z-bend arms never approach their bend axis, so elevation can swing to ~88 deg.")]
        private float armLeanTrimL, armLeanTrimR;
        [SerializeField, Range(-3.14159f, 3.14159f), Tooltip("Added to the blended leg phase offset, per limb.")]
        private float legPhaseTrimL, legPhaseTrimR;
        [SerializeField, Range(-3.14159f, 3.14159f), Tooltip("Added to the blended arm phase offset, per limb.")]
        private float armPhaseTrimL, armPhaseTrimR;

        // Blended result readout (debug view) - exactly what the probes receive.
        [Header("Readouts (debug view)")]
        [SerializeField] private float blendLegFreq, blendLegAmp, blendLegBias;
        [SerializeField] private float blendArmFreq, blendArmAmp, blendArmBias;
        [SerializeField] private float legPhiL, legPhiR, armPhiL, armPhiR;

        [Header("Auto-found (LeftLeg/RightLeg/LeftArm/RightArm children)")]
        [SerializeField] private BackboneWaveProbe leftLeg, rightLeg, leftArm, rightArm;

        [Header("Authoring: torso pin")]
        [SerializeField, Tooltip("Pin the torso in the air (kinematic, held at its spawn pose) so limbs swing freely while tuning. Release with the context-menu command.")]
        private bool pinTorso = true;
        [SerializeField, Min(0f), Tooltip("Pin height above the demo origin (m). Only read when pinning.")]
        private float pinHeight = 1.5f;
        private bool m_pinApplied;

        private void Start()
        {
            // Manual tuning session: full diagnostic stream (rig-created,
            // born-dislocated warnings) like heuristic batch.
            Box3DArticulation.WaveDiagnostics.Verbose = true;
            if (leftLeg == null) leftLeg = FindChild("LeftLeg");
            leftLeg.drawTipPrediction = true;
            if (rightLeg == null) rightLeg = FindChild("RightLeg");
            rightLeg.drawTipPrediction = true;
            if (leftArm == null) leftArm = FindChild("LeftArm");
            leftArm.drawTipPrediction = true;
            if (rightArm == null) rightArm = FindChild("RightArm");
            rightArm.drawTipPrediction = true;
        }

        private BackboneWaveProbe FindChild(string name)
        {
            var t = transform.Find(name);
            return t != null ? t.GetComponent<BackboneWaveProbe>() : null;
        }

        // --- torso pin (authoring) ---
        // Kinematic: infinite mass (limbs hang freely) but Transform-driven,
        // so the same mechanism can raise/lower the pin later. Static would
        // also hold the pose but is semantically 'never moves again'.
        private void ApplyPin()
        {
            var demo = GetComponent<PinnedBipedDemo>();
            var torso = demo != null ? demo.Torso : null;
            if (torso == null || !torso.Body.IsValid) { m_pinApplied = false; return; }
            if (pinTorso && !m_pinApplied)
            {
                torso.BodyType = Box3D.Hybrid.Box3DBodyType.Kinematic;
                // Reset the pose too: after a release the torso keeps its
                // last (possibly tilted) rotation. Rebuild from yaw only,
                // same projection the demo's yaw-lock uses, and kill spin.
                var b = torso.Body;
                float3 fwd = math.mul(b.Rotation, new float3(0f, 0f, 1f)); // RIG-legacy +Z, matches the demo's yaw-lock
                float yaw = math.atan2(fwd.x, fwd.z);
                // Use the hybrid wrapper's warp (Position/Rotation setters),
                // NOT native Body.SetTransform: a kinematic body follows the
                // GameObject Transform, so a native-only teleport is silently
                // discarded on the next transform sync - the torso never
                // moves. Warp writes both the transform and the native state.
                torso.Position = transform.position + Vector3.up * pinHeight;
                torso.Rotation = quaternion.RotateY(yaw);
                b.AngularVelocity = float3.zero;
                b.LinearVelocity = float3.zero;
                m_pinApplied = true;
            }
            else if (!pinTorso && m_pinApplied)
            {
                torso.BodyType = Box3D.Hybrid.Box3DBodyType.Dynamic;
                m_pinApplied = false;
            }
        }

        /// <summary>Release the pin and rebuild the rig from scratch: torso
        /// back to dynamic at its spawn drop, limbs hanging. Also re-binds the
        /// cached probe refs - the rebuild destroys the old probes.</summary>
        [ContextMenu("Release And Reset")]
        public void ReleaseAndReset()
        {
            pinTorso = false;
            m_pinApplied = false;
            var demo = GetComponent<PinnedBipedDemo>();
            if (demo == null) return;
            demo.ClearGeneratedPublic();
            demo.CreateDemo();
            leftLeg = FindChild("LeftLeg");
            leftLeg.drawTipPrediction = true;
            rightLeg = FindChild("RightLeg");
            rightLeg.drawTipPrediction = true;
            leftArm = FindChild("LeftArm");
            leftArm.drawTipPrediction = true;
            rightArm = FindChild("RightArm");
            rightArm.drawTipPrediction = true;
            
        }

        /// <summary>Copy the current blended result over library entry i -
        /// capture a tuned gait as a primitive. Also grows the weight arrays
        /// to match the library so the new entry is immediately blendable.</summary>
        [ContextMenu("Capture Blend As New Primitive")]
        public void CaptureBlendAsNewPrimitive()
        {
            var legs = Mix(library, wLegs);
            var arms = Mix(library, wArms);
            // A captured primitive keeps one tempo pair: legs drive the entry,
            // arms inherit the same base so tempo scales stay meaningful.
            var p = legs;
            p.freqL = legs.freqL; p.freqR = legs.freqR;
            // Arm decoupling at capture: arm fields inherit the blended leg
            // values, so the entry starts as it behaved pre-decoupling.
            p.armLeanL = legs.leanL; p.armLeanR = legs.leanR;
            p.armRollL = legs.rollL; p.armRollR = legs.rollR;
            p.leanL += legLeanTrimL; p.leanR += legLeanTrimR;
            p.phaseL += legPhaseTrimL; p.phaseR += legPhaseTrimR;
            var arr = new WavePrimitive[library.Length + 1];
            library.CopyTo(arr, 0);
            arr[library.Length] = p;
            library = arr;
            wLegs = AppendWeight(wLegs, 0f); // new entry starts at zero weight
            wArms = AppendWeight(wArms, 0f);
        }

        private static float[] AppendWeight(float[] w, float value)
        {
            var arr = new float[w.Length + 1];
            w.CopyTo(arr, 0);
            arr[w.Length] = value;
            return arr;
        }

        // Blending now lives in WaveBlend.Mix (shared with the agent's gate
        // layer) - one implementation, so captured gaits mean the same thing
        // in training.
        private static WavePrimitive Mix(WavePrimitive[] lib, float[] w) => WaveBlend.Mix(lib, w);

        /// <summary>Push the local library into the shared training asset the
        /// agent's gate layer consumes. Creates the asset on first use.</summary>
        [ContextMenu("Copy Library To Training Asset")]
        public void CopyLibraryToTrainingAsset()
        {
#if UNITY_EDITOR
            if (trainingLibrary == null)
            {
                trainingLibrary = UnityEditor.AssetDatabase.LoadAssetAtPath<WavePrimitiveLibrary>("Assets/WavePrimitiveLibrary.asset");
                if (trainingLibrary == null)
                {
                    trainingLibrary = ScriptableObject.CreateInstance<WavePrimitiveLibrary>();
                    UnityEditor.AssetDatabase.CreateAsset(trainingLibrary, "Assets/WavePrimitiveLibrary.asset");
                }
            }
            trainingLibrary.entries = library;
            UnityEditor.EditorUtility.SetDirty(trainingLibrary);
            UnityEditor.AssetDatabase.SaveAssets();
#endif
        }

        private void Update()
        {
            ApplyPin();

            // Blend in parameter space, then apply the agent's live modulation
            // knobs. This is the exact pipeline the gate layer will run.
            var legs = Mix(library, wLegs);
            float lfl = legs.freqL * legFreqScale, lfr = legs.freqR * legFreqScale;
            blendLegFreq = (lfl + lfr) * 0.5f; blendLegAmp = legs.ampTip; blendLegBias = legs.bias;
            Push(leftLeg, legs, lfl, -(legs.leanL + legLeanTrimL), legs.phaseL + legPhaseTrimL, -legs.rollL, legs.poseRollL);
            Push(rightLeg, legs, lfr, legs.leanR + legLeanTrimR, legs.phaseR + legPhaseTrimR, legs.rollR, legs.poseRollR);

            var arms = Mix(library, wArms);
            float afl = arms.freqL * armFreqScale, afr = arms.freqR * armFreqScale;
            blendArmFreq = (afl + afr) * 0.5f; blendArmAmp = arms.ampTip; blendArmBias = arms.bias;
            // Arm bend axis is NEGATED in the rig (elbows bend opposite to
            // knees), so the same numeric offset = negated world bend. Shift
            // arms by pi so one phase value means one world-space pose for
            // every limb - "arms and legs at phase X swing together".
            const float ArmPhaseShift = math.TAU * 0.5f;
            Push(leftArm, arms, afl, -(arms.ArmLeanL + armLeanTrimL), arms.phaseL + armPhaseTrimL + ArmPhaseShift, -arms.ArmRollL, arms.ArmPoseRollL);
            Push(rightArm, arms, afr, arms.ArmLeanR + armLeanTrimR, arms.phaseR + armPhaseTrimR + ArmPhaseShift, arms.ArmRollR, arms.ArmPoseRollR);

            if (leftLeg != null) legPhiL = leftLeg.Phase;
            if (rightLeg != null) legPhiR = rightLeg.Phase;
            if (leftArm != null) armPhiL = leftArm.Phase;
            if (rightArm != null) armPhiR = rightArm.Phase;
        }

        // One limb: blended params + tempo scale + per-limb lean/offset trims.
        // Sign convention (mirrored at push time, as the old ApplyGroup): the
        // LEFT limb gets the NEGATED lean, the RIGHT gets it as-is - positive
        // lean = outward tilt for both sides. If a limb visibly leans the
        // wrong way after capture, flip the sign in the primitive, not here.
        private static void Push(BackboneWaveProbe probe, in WavePrimitive p,
            float freq, float lean, float phase, float roll, float poseRoll)
        {
            if (probe == null) return;
            probe.SetRuntimeParams(freq, p.ampTip, p.bias, p.wavenumber, p.twistAmp, p.twistPhase, p.ampBase, roll, poseRoll);
            probe.BaseDirXTilt = lean;
            probe.SetWavePhaseOffset(phase);
        }
    }
}