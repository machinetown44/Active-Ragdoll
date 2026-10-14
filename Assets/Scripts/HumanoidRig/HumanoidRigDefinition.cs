using UnityEngine;
using Unity.Mathematics;
using System;

namespace HumanoidRig
{
    /// <summary>
    /// Serializable definition of a humanoid rig driven by spline rails.
    /// Create via Assets > Create > Humanoid Rig > Humanoid Rig Definition.
    /// </summary>
    [CreateAssetMenu(fileName = "HumanoidRigDefinition", menuName = "Humanoid Rig/Humanoid Rig Definition")]
    public class HumanoidRigDefinition : ScriptableObject
    {
        [Header("Bone Mapping (Unity Humanoid)")]
        public Transform pelvis;
        public Transform leftHip, leftKnee, leftAnkle, leftFoot;
        public Transform rightHip, rightKnee, rightAnkle, rightFoot;
        public Transform spine, chest, neck, head;
        public Transform leftShoulder, leftElbow, leftWrist, leftHand;
        public Transform rightShoulder, rightElbow, rightWrist, rightHand;

        [Header("Spline Rails")]
        public SplineRail torsoRail = new SplineRail();
        public SplineRail leftLegRail = new SplineRail();
        public SplineRail rightLegRail = new SplineRail();
        public SplineRail leftArmRail = new SplineRail();
        public SplineRail rightArmRail = new SplineRail();

        [Header("Rail Parameters")]
        [Range(2, 32)] public int samplesPerSegment = 8;
        [Range(0.01f, 1f)] public float chainPartitionGap = 0.05f;

        [Header("Gait / Timing")]
        public float cyclePeriod = 1f;           // seconds per full cycle
        [Range(0f, 1f)] public float legPhaseOffset = 0.5f;  // 0.5 = trot
        [Range(0f, 1f)] public float armPhaseOffset = 0.5f;
        public bool armsOppositeToLegs = true;

        [Header("Limb Lengths (for rail projection)")]
        public float thighLength = 0.4f;
        public float shinLength = 0.4f;
        public float footLength = 0.15f;
        public float upperArmLength = 0.35f;
        public float forearmLength = 0.3f;
        public float handLength = 0.1f;

        [Header("Hinge Axes (Parent Local Space)")]
        public Vector3 kneeHingeAxis = new Vector3(0, 0, 1);
        public Vector3 ankleHingeAxis = new Vector3(0, 0, 1);
        public Vector3 elbowHingeAxis = new Vector3(0, 0, 1);
        public Vector3 wristHingeAxis = Vector3.zero;  // zero = spherical

        [Header("Spine Joint Types")]
        public bool hipSpherical = true;
        public bool shoulderSpherical = true;
        public bool spineSpherical = true;

        [Header("Solver")]
        [Range(1, 10)] public int fabrikIterations = 4;
        [Range(0f, 1f)] public float damping = 0.1f;

        public bool Validate(out string error)
        {
            var bones = new (Transform, string)[]
            {
                (pelvis, "Pelvis"), (leftHip, "Left Hip"), (leftKnee, "Left Knee"),
                (leftAnkle, "Left Ankle"), (leftFoot, "Left Foot"),
                (rightHip, "Right Hip"), (rightKnee, "Right Knee"),
                (rightAnkle, "Right Ankle"), (rightFoot, "Right Foot"),
                (spine, "Spine"), (chest, "Chest"), (neck, "Neck"), (head, "Head"),
                (leftShoulder, "Left Shoulder"), (leftElbow, "Left Elbow"),
                (leftWrist, "Left Wrist"), (leftHand, "Left Hand"),
                (rightShoulder, "Right Shoulder"), (rightElbow, "Right Elbow"),
                (rightWrist, "Right Wrist"), (rightHand, "Right Hand")
            };
            foreach (var (t, name) in bones)
                if (t == null) { error = $"Missing bone: {name}"; return false; }

            if (torsoRail.controlPoints == null || torsoRail.controlPoints.Length < 2) { error = "Torso rail needs ≥2 control points"; return false; }
            if (leftLegRail.controlPoints == null || leftLegRail.controlPoints.Length < 2) { error = "Left leg rail needs ≥2 control points"; return false; }
            if (rightLegRail.controlPoints == null || rightLegRail.controlPoints.Length < 2) { error = "Right leg rail needs ≥2 control points"; return false; }
            if (leftArmRail.controlPoints == null || leftArmRail.controlPoints.Length < 2) { error = "Left arm rail needs ≥2 control points"; return false; }
            if (rightArmRail.controlPoints == null || rightArmRail.controlPoints.Length < 2) { error = "Right arm rail needs ≥2 control points"; return false; }

            error = null;
            return true;
        }

        [Serializable]
        public class SplineRail
        {
            public Transform[] controlPoints = new Transform[0];
            [NonSerialized] public RailSample[] samples;
            [NonSerialized] public float length;

            public bool IsValid => controlPoints != null && controlPoints.Length >= 2 && controlPoints[0] != null && controlPoints[1] != null;
        }

        [Serializable]
        public struct RailSample
        {
            public float s;          // arc-length parameter
            public Vector3 position; // world position
            public Vector3 tangent;  // unit tangent
        }
    }
}