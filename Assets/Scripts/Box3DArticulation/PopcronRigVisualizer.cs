using UnityEngine;
using System.Collections.Generic;
using Popcron;
using Box3D.Hybrid;
using Gizmos = Popcron.Gizmos;
using static Unity.Mathematics.math;

namespace Box3DArticulation
{
    [DisallowMultipleComponent]
    [ExecuteAlways]
    public sealed class PopcronRigVisualizer : MonoBehaviour
    {
        [Header("Visualization")]
        [SerializeField] private bool showColliders = true;
        [SerializeField] private bool showSpline = true;
        [SerializeField] private bool showLimbs = true;
        [SerializeField] private bool showJoints = true;
        [SerializeField] private bool showProjectedLimbs = true;
        [SerializeField] private bool showDeviations = true;
        [SerializeField] private bool showBodyTrajectory = true;
        [SerializeField] private bool showBodyOrientationHistory = true;
        [SerializeField, Min(1)] private int orientationSampleEveryFrames = 10;
        [SerializeField, Min(2)] private int bodyHistoryCapacity = 120;
        [SerializeField, Min(0.01f)] private float orientationAxisLength = 0.25f;
        [SerializeField, Min(0.005f)] private float trajectoryPointRadius = 0.045f;

        [Header("Colors")]
        [SerializeField] private Color colliderColor = new Color(0.2f, 0.8f, 1f, 0.8f);
        [SerializeField] private Color splineColor = new Color(1f, 0.8f, 0.2f, 0.8f);
        [SerializeField] private Color limbColor = new Color(0.8f, 1f, 0.2f, 0.8f);
        [SerializeField] private Color jointColor = new Color(1f, 0.2f, 0.8f, 1f);
        [SerializeField] private Color projectedLimbColor = new Color(1f, 0.5f, 0f, 0.9f);
        [SerializeField] private Color projectedJointColor = new Color(1f, 0.3f, 0f, 1f);
        [SerializeField] private Color deviationColor = new Color(1f, 0f, 0f, 0.8f);
        [SerializeField] private Color bodyTrajectoryColor = new Color(1f, 0.05f, 0.8f, 1f);
        [SerializeField] private Color bodyForwardColor = new Color(1f, 0.2f, 0.2f, 0.95f);
        [SerializeField] private Color bodyUpColor = new Color(0.2f, 1f, 0.3f, 0.95f);
        [SerializeField] private Color bodyRightColor = new Color(0.2f, 0.5f, 1f, 0.95f);

        [Header("Camera")]
        [SerializeField] private Camera targetCamera;

        private SplineChainController controller;
        private LimbDeviationTracker tracker;
        private bool cameraFilterRegistered;

        private struct BodyPoseSample
        {
            public Vector3 position;
            public Quaternion rotation;
        }

        private readonly List<BodyPoseSample> bodyHistory = new List<BodyPoseSample>();
        private int visualizerFrame;

        private void Awake()
        {
            controller = GetComponent<SplineChainController>();
            tracker = GetComponent<LimbDeviationTracker>();
        }

        private void OnEnable()
        {
            RegisterCameraFilter();
        }

        private void OnDisable()
        {
            UnregisterCameraFilter();
        }

        private void RegisterCameraFilter()
        {
            if (cameraFilterRegistered) return;
            Gizmos.CameraFilters.Add(CameraFilterCallback);
            cameraFilterRegistered = true;
        }

        private void UnregisterCameraFilter()
        {
            if (!cameraFilterRegistered) return;
            Gizmos.CameraFilters.Remove(CameraFilterCallback);
            cameraFilterRegistered = false;
        }

        private bool CameraFilterCallback(Camera cam)
        {
            return targetCamera != null && cam == targetCamera;
        }

        private void Update()
        {
            int callCount = 0;

            // Colliders draw standalone - used for side-by-side previews of a
            // Box3DRigBuilder rig against its animation source.
            if (showColliders)
            {
                callCount += DrawColliders();
            }

            if (controller == null) return;

            var world = Box3DWorld.Instance;
            if (world == null || !world.World.IsValid) return;

            if (showSpline)
            {
                callCount += DrawSpline();
            }

            if (showLimbs)
            {
                callCount += DrawLimbs();
            }

            if (showProjectedLimbs)
            {
                callCount += DrawProjectedLimbs();
            }

            RecordBodyPose();

            if (showBodyTrajectory)
            {
                callCount += DrawBodyTrajectory();
            }

            if (showBodyOrientationHistory)
            {
                callCount += DrawBodyOrientationHistory();
            }

            if (showDeviations)
            {
                callCount += DrawDeviations();
            }
            
        }

        private int DrawColliders()
        {
            var bodies = GetComponentsInChildren<Box3DBody>();
            int count = 0;
            foreach (var body in bodies)
            {
                if (!body.Body.IsValid) continue;

                var shapes = body.GetComponents<Box3DShape>();
                foreach (var shape in shapes)
                {
                    if (shape is Box3DBoxShape box)
                    {
                        var size = GetPrivateField<Vector3>(box, "Size");
                        Gizmos.Cube(body.transform.position, body.transform.rotation, size, colliderColor);
                        count++;
                    }
                    else if (shape is Box3DCapsuleShape capsule)
                    {
                        var radius = GetPrivateField<float>(capsule, "Radius");
                        var height = GetPrivateField<float>(capsule, "Height");
                        Gizmos.Cube(body.transform.position, body.transform.rotation, new Vector3(radius * 2, height, radius * 2), colliderColor);
                        count++;
                    }
                }
            }
            return count;
        }

        private int DrawSpline()
        {
            var railSamples = controller.RailSamples;
            
            if (railSamples == null || railSamples.Count < 2) return 0;

            int count = 0;
            for (int i = 0; i < railSamples.Count - 1; i++)
            {
                var a = railSamples[i].position;
                var b = railSamples[i + 1].position;
                Gizmos.Line(a, b, splineColor);
                count++;
            }
            return count;
        }

        private int DrawLimbs()
        {
            var limbs = controller.Limbs;
            if (limbs == null || limbs.Count == 0) return 0;

            int count = 0;
            foreach (var limb in limbs)
            {
                if (limb.upperBody != null && limb.upperBody.Body.IsValid)
                {
                    Gizmos.Cube(limb.upperBody.transform.position, limb.upperBody.transform.rotation, 
                        new Vector3(0.24f, limb.upperLength, 0.24f), limbColor);
                    count++;
                }
                if (limb.lowerBody != null && limb.lowerBody.Body.IsValid)
                {
                    Gizmos.Cube(limb.lowerBody.transform.position, limb.lowerBody.transform.rotation, 
                        new Vector3(0.24f, limb.lowerLength, 0.24f), limbColor);
                    count++;
                }
            }
            return count;
        }

        private int DrawProjectedLimbs()
        {
            var debugData = controller.GetDebugData();
            if (debugData == null || debugData.Length == 0) return 0;

            int count = 0;
            for (int i = 0; i < debugData.Length; i++)
            {
                // Draw target joint positions (projected limb skeleton)
                var targets = debugData[i].targetJointPositions;
                if (targets == null || targets.Length < 2) continue;

                Color baseColor = SplineChainController.GetChainColor(i);
                int last = targets.Length - 1;

                // Brightness gradient along the chain: root end is pulled toward white,
                // tip end keeps the base hue, and the root sphere is drawn larger —
                // makes root->tip order obvious even between mirrored chains.
                for (int j = 0; j < last; j++)
                {
                    float t = 1f - (j + 0.5f) / last;
                    Gizmos.Line(targets[j], targets[j + 1], Color.Lerp(baseColor, Color.white, t * 0.6f));
                    count++;
                }

                for (int j = 0; j <= last; j++)
                {
                    float t = 1f - j / (float)last;
                    Gizmos.Sphere(targets[j], j == 0 ? 0.06f : 0.035f, Color.Lerp(baseColor, Color.white, t * 0.6f));
                    count++;
                }
            }
            return count;
        }

        private void RecordBodyPose()
        {
            if (controller == null || controller.TorsoBody == null || !controller.TorsoBody.Body.IsValid)
                return;

            visualizerFrame++;
            if (visualizerFrame % Mathf.Max(1, orientationSampleEveryFrames) != 0)
                return;

            var body = controller.TorsoBody.Body;
            bodyHistory.Add(new BodyPoseSample
            {
                position = body.GetWorldCenterOfMass(),
                rotation = body.GetRotation()
            });

            int excess = bodyHistory.Count - Mathf.Max(2, bodyHistoryCapacity);
            if (excess > 0)
                bodyHistory.RemoveRange(0, excess);
        }

        private int DrawBodyTrajectory()
        {
            if (bodyHistory.Count < 2) return 0;

            int count = 0;
            for (int i = 0; i < bodyHistory.Count; i++)
            {
                Gizmos.Sphere(bodyHistory[i].position, trajectoryPointRadius, bodyTrajectoryColor);
                count++;
                if (i == 0) continue;

                Gizmos.Line(bodyHistory[i - 1].position, bodyHistory[i].position, bodyTrajectoryColor);
                count++;
            }
            return count;
        }

        private int DrawBodyOrientationHistory()
        {
            if (bodyHistory.Count == 0) return 0;

            int count = 0;
            float axisLength = orientationAxisLength;
            for (int i = 0; i < bodyHistory.Count; i++)
            {
                BodyPoseSample sample = bodyHistory[i];
                Vector3 origin = sample.position;
                Gizmos.Line(origin, origin + sample.rotation * Vector3.forward * axisLength, bodyForwardColor);
                Gizmos.Line(origin, origin + sample.rotation * Vector3.up * axisLength, bodyUpColor);
                Gizmos.Line(origin, origin + sample.rotation * Vector3.right * axisLength, bodyRightColor);
                count += 3;
            }
            return count;
        }

        private int DrawDeviations()
        {
            var debugData = controller.GetDebugData();
            if (debugData == null || debugData.Length == 0) return 0;

            int count = 0;
            foreach (var limbData in debugData)
            {
                var actual = limbData.actualJointPositions;
                var target = limbData.targetJointPositions;
                var deviations = limbData.deviationVectors;
                
                if (actual != null && target != null && actual.Length == target.Length)
                {
                    for (int j = 0; j < actual.Length; j++)
                    {
                        // Draw line from actual to target position
                        Gizmos.Line(actual[j], target[j], deviationColor);
                        count++;
                        
                        // Draw deviation magnitude as small sphere at target
                        if (deviations != null && j < deviations.Length)
                        {
                            float dev = length(deviations[j]);
                            if (dev > 0.01f)
                            {
                                Gizmos.Sphere(target[j], 0.02f + dev * 0.1f, deviationColor);
                                count++;
                            }
                        }
                    }
                }
            }
            return count;
        }

        private static T GetPrivateField<T>(object obj, string name)
        {
            var field = obj.GetType().GetField(name, 
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            return (T)field?.GetValue(obj);
        }
    }
}