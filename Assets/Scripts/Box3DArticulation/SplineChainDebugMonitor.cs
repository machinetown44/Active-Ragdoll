using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Tracks deviation of actual joint anchor positions from their spline-projected targets
    /// over a limited time horizon. Provides real-time debug rendering and historical data.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(SplineChainController))]
    public sealed class SplineChainDebugMonitor : MonoBehaviour
    {
        [Header("Tracking")]
        [SerializeField, Tooltip("Seconds of history to keep (ring buffer)")]
        private float historyDuration = 5f;
        
        [SerializeField, Tooltip("Maximum samples to store (caps memory)")]
        private int maxSamples = 600; // 5s @ 120Hz fixed update
        
        [Header("Debug Rendering")]
        [SerializeField] private bool drawActualTargets = true;
        [SerializeField] private bool drawDeviationVectors = true;
        [SerializeField] private bool drawRail = true;
        [SerializeField] private bool drawColliders = true;
        [SerializeField] private bool drawJoints = true;
        [SerializeField] private float deviationVectorScale = 10f;
        [SerializeField] private Color actualColor = Color.green;
        [SerializeField] private Color targetColor = Color.red;
        [SerializeField] private Color deviationColor = Color.yellow;
        [SerializeField] private Color railColor = Color.cyan;
        [SerializeField] private Color colliderColor = new Color(1f, 0.5f, 0f, 0.5f);
        [SerializeField] private Color jointColor = Color.magenta;

        [Header("Thresholds")]
        [SerializeField, Tooltip("Deviation magnitude (m) above which to highlight")]
        private float warningThreshold = 0.05f;
        [SerializeField, Tooltip("Deviation magnitude (m) above which to flag as error")]
        private float errorThreshold = 0.15f;

        // Ring buffer for historical deviation data
        private struct SampleFrame
        {
            public float time;
            public SplineChainController.LimbDebugData[] limbData;
        }

        private SampleFrame[] history;
        private int historyHead = 0;
        private int historyCount = 0;
        private float nextSampleTime = 0f;
        private float sampleInterval; // FixedUpdate interval

        // Runtime stats
        private float maxDeviationSeen = 0f;
        private float avgDeviation = 0f;
        private int framesWithWarnings = 0;
        private int framesWithErrors = 0;

        private SplineChainController controller;

        private void Awake()
        {
            controller = GetComponent<SplineChainController>();
            sampleInterval = Time.fixedDeltaTime;
            int capacity = Mathf.Min(maxSamples, Mathf.CeilToInt(historyDuration / sampleInterval) + 1);
            history = new SampleFrame[capacity];
        }

        private void OnValidate()
        {
            if (historyDuration < 0.1f) historyDuration = 0.1f;
            if (maxSamples < 10) maxSamples = 10;
            if (deviationVectorScale < 0f) deviationVectorScale = 0f;
            if (warningThreshold < 0f) warningThreshold = 0f;
            if (errorThreshold < warningThreshold) errorThreshold = warningThreshold;
        }

        private void LateUpdate()
        {
            if (!controller || !controller.enabled) return;

            float now = Time.time;
            if (now < nextSampleTime) return;
            nextSampleTime = now + sampleInterval;

            var debugData = controller.GetDebugData();
            if (debugData == null || debugData.Length == 0) return;

            // Store in ring buffer
            history[historyHead] = new SampleFrame
            {
                time = now,
                limbData = DeepCopy(debugData)
            };
            historyHead = (historyHead + 1) % history.Length;
            historyCount = Mathf.Min(historyCount + 1, history.Length);

            // Update stats
            UpdateStats(debugData);
        }

        private static SplineChainController.LimbDebugData[] DeepCopy(SplineChainController.LimbDebugData[] src)
        {
            var dst = new SplineChainController.LimbDebugData[src.Length];
            for (int i = 0; i < src.Length; i++)
            {
                int n = src[i].jointCount;
                dst[i].limbIndex = src[i].limbIndex;
                dst[i].jointCount = n;
                dst[i].actualJointPositions = (float3[])src[i].actualJointPositions.Clone();
                dst[i].targetJointPositions = (float3[])src[i].targetJointPositions.Clone();
                dst[i].deviations = (float[])src[i].deviations.Clone();
                dst[i].deviationVectors = (float3[])src[i].deviationVectors.Clone();
                dst[i].jointSValues = (float[])src[i].jointSValues.Clone();
            }
            return dst;
        }

        private void UpdateStats(SplineChainController.LimbDebugData[] data)
        {
            float frameMax = 0f;
            float frameSum = 0f;
            int frameCount = 0;
            bool hasWarning = false;
            bool hasError = false;

            for (int i = 0; i < data.Length; i++)
            {
                for (int j = 0; j < data[i].jointCount; j++)
                {
                    float dev = data[i].deviations[j];
                    frameMax = math.max(frameMax, dev);
                    frameSum += dev;
                    frameCount++;
                    if (dev > warningThreshold) hasWarning = true;
                    if (dev > errorThreshold) hasError = true;
                }
            }

            maxDeviationSeen = math.max(maxDeviationSeen, frameMax);
            avgDeviation = frameCount > 0 ? frameSum / frameCount : 0f;
            if (hasWarning) framesWithWarnings++;
            if (hasError) framesWithErrors++;
        }

        private void OnDrawGizmos()
        {
            if (!controller || !controller.enabled) return;

            var data = controller.GetDebugData();
            if (data == null || data.Length == 0) return;

            // Draw rail
            if (drawRail) DrawRail();

            // Draw colliders
            if (drawColliders) DrawColliders();

            // Draw joints
            if (drawJoints) DrawJoints();

            // Draw actual vs target positions + deviation vectors, color-coded per chain
            for (int i = 0; i < data.Length; i++)
            {
                var limb = data[i];
                Color chainColor = SplineChainController.GetChainColor(i);
                for (int j = 0; j < limb.jointCount; j++)
                {
                    float3 actual = limb.actualJointPositions[j];
                    float3 target = limb.targetJointPositions[j];
                    float3 deviation = limb.deviationVectors[j];
                    float devMag = limb.deviations[j];

                    // Actual position
                    if (drawActualTargets)
                    {
                        Gizmos.color = actualColor;
                        Gizmos.DrawWireSphere(actual, 0.025f);
                        Gizmos.DrawSphere(actual, 0.015f);
                    }

                    // Target position — chain color, pulled toward white at the root
                    // joint (j == 0) so root->tip order stays readable
                    if (drawActualTargets)
                    {
                        Gizmos.color = j == 0 ? Color.Lerp(chainColor, Color.white, 0.6f) : chainColor;
                        Gizmos.DrawWireSphere(target, j == 0 ? 0.04f : 0.025f);
                    }

                    // Deviation vector
                    if (drawDeviationVectors && devMag > 1e-4f)
                    {
                        Color c = devMag > errorThreshold ? Color.red : 
                                  devMag > warningThreshold ? Color.yellow : deviationColor;
                        Gizmos.color = c;
                        Gizmos.DrawLine(actual, actual + deviation * deviationVectorScale);
                        Gizmos.DrawWireSphere(target, 0.02f);
                    }
                }
            }
        }

        private void DrawRail()
        {
            var railSamples = controller.RailSamples;
            if (railSamples != null && railSamples.Count > 1)
            {
                Gizmos.color = railColor;
                var positions = new Vector3[railSamples.Count];
                for (int i = 0; i < railSamples.Count; i++)
                {
                    positions[i] = (Vector3)railSamples[i].position;
                }
                Gizmos.DrawLineStrip(positions, false);
            }
        }

        private void DrawColliders()
        {
            var data = controller.GetDebugData();
            if (data == null) return;

            Gizmos.color = colliderColor;
            for (int i = 0; i < data.Length; i++)
            {
                var limb = data[i];
                if (limb.jointCount < 2) continue;

                // Draw capsules between consecutive joint positions (approximates limb colliders)
                for (int j = 0; j < limb.jointCount - 1; j++)
                {
                    float3 a = limb.actualJointPositions[j];
                    float3 b = limb.actualJointPositions[j + 1];
                    DrawCapsuleGizmo(a, b, 0.05f); // approximate radius
                }
            }

            // Draw torso as box (using torso body's actual transform)
            if (controller.Torso != null && controller.Torso.Body.IsValid)
            {
                var torsoBody = controller.Torso.Body;
                var pos = torsoBody.GetPosition();
                var rot = torsoBody.GetRotation();
                // Approximate torso size from COM offset
                Gizmos.matrix = Matrix4x4.TRS((Vector3)pos, (Quaternion)rot, Vector3.one);
                Gizmos.DrawWireCube(Vector3.zero, new Vector3(0.4f, 0.5f, 0.2f)); // approximate
                Gizmos.matrix = Matrix4x4.identity;
            }
        }

        private void DrawCapsuleGizmo(float3 a, float3 b, float radius)
        {
            float3 dir = b - a;
            float length = math.length(dir);
            if (length < 1e-4f) return;
            dir = dir / length;

            float halfLen = math.max(0f, length * 0.5f - radius);
            float3 center = (a + b) * 0.5f;
            quaternion rot = quaternion.LookRotationSafe(dir, math.up());

            // Draw capsule using Gizmos.matrix for orientation
            var oldMatrix = Gizmos.matrix;
            Gizmos.matrix = Matrix4x4.TRS((Vector3)center, (Quaternion)rot, Vector3.one);
            // Draw cylinder part
            if (halfLen > 0f)
            {
                Gizmos.DrawWireCube(Vector3.up * 0f, new Vector3(radius * 2, halfLen * 2, radius * 2));
            }
            // Draw end spheres
            Gizmos.DrawWireSphere(Vector3.up * halfLen, radius);
            Gizmos.DrawWireSphere(Vector3.down * halfLen, radius);
            Gizmos.matrix = oldMatrix;
        }

        private void DrawJoints()
        {
            // Native Box3D joints are internal; draw limb connections instead
            var data = controller.GetDebugData();
            if (data == null) return;

            Gizmos.color = jointColor;
            for (int i = 0; i < data.Length; i++)
            {
                var limb = data[i];
                for (int j = 0; j < limb.jointCount - 1; j++)
                {
                    Gizmos.DrawLine(limb.actualJointPositions[j], limb.actualJointPositions[j + 1]);
                }
            }
        }

        /// <summary>
        /// Get deviation history for a specific limb and joint.
        /// Returns (times, deviations) arrays.
        /// </summary>
        public (float[] times, float[] deviations) GetJointDeviationHistory(int limbIndex, int jointIndex)
        {
            if (limbIndex < 0 || historyCount == 0) return (Array.Empty<float>(), Array.Empty<float>());

            var times = new List<float>();
            var deviations = new List<float>();

            int start = (historyHead - historyCount + history.Length) % history.Length;
            for (int k = 0; k < historyCount; k++)
            {
                int idx = (start + k) % history.Length;
                var frame = history[idx];
                if (frame.limbData != null && limbIndex < frame.limbData.Length)
                {
                    var limb = frame.limbData[limbIndex];
                    if (jointIndex < limb.jointCount)
                    {
                        times.Add(frame.time);
                        deviations.Add(limb.deviations[jointIndex]);
                    }
                }
            }

            return (times.ToArray(), deviations.ToArray());
        }

        /// <summary>
        /// Get aggregate stats over the history window.
        /// </summary>
        public (float maxDev, float avgDev, float warningRate, float errorRate) GetStats()
        {
            if (historyCount == 0) return (0f, 0f, 0f, 0f);

            float maxDev = 0f;
            float sumDev = 0f;
            int totalSamples = 0;
            int warningFrames = 0;
            int errorFrames = 0;

            int start = (historyHead - historyCount + history.Length) % history.Length;
            for (int k = 0; k < historyCount; k++)
            {
                int idx = (start + k) % history.Length;
                var frame = history[idx];
                if (frame.limbData == null) continue;

                for (int i = 0; i < frame.limbData.Length; i++)
                {
                    var limb = frame.limbData[i];
                    for (int j = 0; j < limb.jointCount; j++)
                    {
                        float dev = limb.deviations[j];
                        maxDev = math.max(maxDev, dev);
                        sumDev += dev;
                        totalSamples++;
                        if (dev > warningThreshold) warningFrames++;
                        if (dev > errorThreshold) errorFrames++;
                    }
                }
            }

            float avgDev = totalSamples > 0 ? sumDev / totalSamples : 0f;
            float warningRate = historyCount > 0 ? (float)warningFrames / historyCount : 0f;
            float errorRate = historyCount > 0 ? (float)errorFrames / historyCount : 0f;

            return (maxDev, avgDev, warningRate, errorRate);
        }

        /// <summary>
        /// Clear history buffer.
        /// </summary>
        public void ClearHistory()
        {
            historyHead = 0;
            historyCount = 0;
            maxDeviationSeen = 0f;
            avgDeviation = 0f;
            framesWithWarnings = 0;
            framesWithErrors = 0;
            Array.Clear(history, 0, history.Length);
        }

        /// <summary>
        /// Current frame's limb debug data (for EditorWindow polling).
        /// </summary>
        public SplineChainController.LimbDebugData[] CurrentData => controller?.GetDebugData();

        /// <summary>
        /// Number of valid history frames.
        /// </summary>
        public int HistoryFrameCount => historyCount;

        /// <summary>
        /// History duration in seconds.
        /// </summary>
        public float HistoryDuration => historyDuration;

        /// <summary>
        /// Maximum deviation seen in history.
        /// </summary>
        public float MaxDeviationSeen => maxDeviationSeen;
    }
}