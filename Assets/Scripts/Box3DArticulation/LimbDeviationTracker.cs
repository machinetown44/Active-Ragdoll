using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using System;
using System.Collections.Generic;

namespace Box3DArticulation
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(SplineChainController))]
    [ExecuteAlways]
    public sealed class LimbDeviationTracker : MonoBehaviour
    {
        [Header("Graph Settings")]
        [SerializeField] private int graphWidth = 512;
        [SerializeField] private int graphHeight = 256;
        [SerializeField] private float historyDuration = 10f; // seconds
        [SerializeField] private float maxPositionDeviation = 0.5f; // meters
        [SerializeField] private float maxOrientationDeviation = 45f; // degrees

        [Header("Colors")]
        [SerializeField] private Color backgroundColor = new Color(0.1f, 0.1f, 0.12f, 1f);
        [SerializeField] private Color gridColor = new Color(0.3f, 0.3f, 0.35f, 1f);
        [SerializeField] private Color positionColor = new Color(0.2f, 0.8f, 0.4f, 1f);
        [SerializeField] private Color orientationColor = new Color(0.9f, 0.5f, 0.2f, 1f);
        [SerializeField] private Color warningColor = new Color(1f, 0.8f, 0.2f, 1f);
        [SerializeField] private Color errorColor = new Color(1f, 0.3f, 0.3f, 1f);

        [Header("Thresholds")]
        [SerializeField] private float positionWarningThreshold = 0.1f;
        [SerializeField] private float positionErrorThreshold = 0.25f;
        [SerializeField] private float orientationWarningThreshold = 15f;
        [SerializeField] private float orientationErrorThreshold = 30f;

        private SplineChainController controller;
        private RenderTexture graphTexture;
        private Texture2D readbackTexture;
        private Material glMaterial;
        private LimbDeviationHistory[] history;
        private int historyCapacity;
        private int historyHead = 0;
        private int historyCount = 0;
        private float timeAccumulator = 0f;
        private float lastSampleTime = 0f;
        private int limbsTracked = 0;

        // Per-segment data structures
        public struct SegmentDeviation
        {
            public Vector3 worldPosition;
            public Vector3 targetPosition;
            public float positionDeviation;
            public Vector3 actualDirection;
            public Vector3 targetDirection;
            public float orientationDeviation;
        }

        public struct LimbDeviations
        {
            public SegmentDeviation shoulder;
            public SegmentDeviation upper;
            public SegmentDeviation lower;
            public float avgPositionDeviation;
            public float avgOrientationDeviation;
        }

        public struct BodyDeviations
        {
            public Vector3 worldPosition;
            public Vector3 targetPosition;
            public float positionDeviation;
            public Quaternion worldRotation;
            public Quaternion targetRotation;
            public float orientationDeviation;
        }

        // Simple struct for CPU history
        private struct LimbDeviationHistory
        {
            public float time;
            // Per limb: shoulder, upper, lower
            public float posDev0_shoulder;
            public float posDev0_upper;
            public float posDev0_lower;
            public float posDev1_shoulder;
            public float posDev1_upper;
            public float posDev1_lower;
            public float oriDev0_shoulder;
            public float oriDev0_upper;
            public float oriDev0_lower;
            public float oriDev1_shoulder;
            public float oriDev1_upper;
            public float oriDev1_lower;
            // Body (torso)
            public float bodyPosDev;
            public float bodyOriDev;
        }

        private void Awake()
        {
            controller = GetComponent<SplineChainController>();
            InitializeGraph();
        }

        private void OnDestroy()
        {
            ReleaseResources();
        }

        private void OnEnable()
        {
            if (graphTexture == null)
                InitializeGraph();
        }

        private void OnDisable()
        {
            ReleaseResources();
        }

        private void InitializeGraph()
        {
            int samplesPerSecond = 60;
            historyCapacity = Mathf.CeilToInt(historyDuration * samplesPerSecond);
            historyCapacity = Mathf.Max(historyCapacity, 2);
            history = new LimbDeviationHistory[historyCapacity];

            graphTexture = new RenderTexture(graphWidth, graphHeight, 0, RenderTextureFormat.ARGB32);
            graphTexture.filterMode = FilterMode.Bilinear;
            graphTexture.wrapMode = TextureWrapMode.Clamp;
            graphTexture.Create();

            readbackTexture = new Texture2D(graphWidth, graphHeight, TextureFormat.RGBA32, false);

            // Create unlit material for GL rendering - use built-in unlit shader
            glMaterial = new Material(Shader.Find("Unlit/Transparent"));
            if (glMaterial == null)
                glMaterial = new Material(Shader.Find("Sprites/Default"));
            if (glMaterial == null)
                glMaterial = new Material(Shader.Find("UI/Default"));
            glMaterial.hideFlags = HideFlags.HideAndDontSave;
        }

        private void ReleaseResources()
        {
            if (graphTexture != null)
            {
                graphTexture.Release();
                UnityEngine.Object.DestroyImmediate(graphTexture);
                graphTexture = null;
            }
            if (readbackTexture != null)
            {
                UnityEngine.Object.DestroyImmediate(readbackTexture);
                readbackTexture = null;
            }
            if (glMaterial != null)
            {
                UnityEngine.Object.DestroyImmediate(glMaterial);
                glMaterial = null;
            }
        }

        private void Update()
        {
            if (controller == null) return;

            // Compute deviations directly from transforms and rail
            var deviations = ComputeDeviations();
            if (deviations == null || deviations.Length == 0) return;

            limbsTracked = deviations.Length;
            timeAccumulator += Time.deltaTime;

            // Sample at ~60Hz
            float sampleInterval = 1f / 60f;
            if (timeAccumulator - lastSampleTime >= sampleInterval)
            {
                lastSampleTime = timeAccumulator;

                var frame = new LimbDeviationHistory
                {
                    time = timeAccumulator,
                    posDev0_shoulder = deviations.Length > 0 ? deviations[0].shoulder.positionDeviation : 0f,
                    posDev0_upper = deviations.Length > 0 ? deviations[0].upper.positionDeviation : 0f,
                    posDev0_lower = deviations.Length > 0 ? deviations[0].lower.positionDeviation : 0f,
                    posDev1_shoulder = deviations.Length > 1 ? deviations[1].shoulder.positionDeviation : 0f,
                    posDev1_upper = deviations.Length > 1 ? deviations[1].upper.positionDeviation : 0f,
                    posDev1_lower = deviations.Length > 1 ? deviations[1].lower.positionDeviation : 0f,
                    oriDev0_shoulder = deviations.Length > 0 ? deviations[0].shoulder.orientationDeviation : 0f,
                    oriDev0_upper = deviations.Length > 0 ? deviations[0].upper.orientationDeviation : 0f,
                    oriDev0_lower = deviations.Length > 0 ? deviations[0].lower.orientationDeviation : 0f,
                    oriDev1_shoulder = deviations.Length > 1 ? deviations[1].shoulder.orientationDeviation : 0f,
                    oriDev1_upper = deviations.Length > 1 ? deviations[1].upper.orientationDeviation : 0f,
                    oriDev1_lower = deviations.Length > 1 ? deviations[1].lower.orientationDeviation : 0f,
                };

                // Body deviations
                var bodyDev = ComputeBodyDeviations();
                if (bodyDev.HasValue)
                {
                    frame.bodyPosDev = bodyDev.Value.positionDeviation;
                    frame.bodyOriDev = bodyDev.Value.orientationDeviation;
                }

                history[historyHead] = frame;
                historyHead = (historyHead + 1) % historyCapacity;
                historyCount = Mathf.Min(historyCount + 1, historyCapacity);
            }

            // Render graph every frame
            RenderGraph();
        }

        private LimbDeviations[] ComputeDeviations()
        {
            // Consume the controller's per-chain debug data (actual vs rail-target joint
            // positions). Arm chains are indices >= 1; chain 0 is the spine.
            var data = controller.GetDebugData();
            if (data == null || data.Length < 2) return null;

            var results = new LimbDeviations[data.Length - 1];
            for (int c = 1; c < data.Length; c++)
            {
                var dd = data[c];
                int n = dd.actualJointPositions.Length; // root + joints
                if (n < 2) continue;

                var shoulder = new SegmentDeviation
                {
                    worldPosition = dd.actualJointPositions[0],
                    targetPosition = dd.targetJointPositions[0],
                    positionDeviation = dd.deviations[0],
                    actualDirection = math.normalizesafe(dd.actualJointPositions[1] - dd.actualJointPositions[0], math.up()),
                    targetDirection = math.normalizesafe(dd.targetJointPositions[1] - dd.targetJointPositions[0], math.up()),
                };
                shoulder.orientationDeviation = AngleBetween(shoulder.actualDirection, shoulder.targetDirection);

                // Upper segment: joint 1 -> joint 2 (elbow -> hand) when available
                float3 upperActualDir = math.normalizesafe(dd.actualJointPositions[math.min(2, n - 1)] - dd.actualJointPositions[1], math.up());
                float3 upperTargetDir = math.normalizesafe(dd.targetJointPositions[math.min(2, n - 1)] - dd.targetJointPositions[1], math.up());
                var upper = new SegmentDeviation
                {
                    worldPosition = dd.actualJointPositions[1],
                    targetPosition = dd.targetJointPositions[1],
                    positionDeviation = dd.deviations[1],
                    actualDirection = upperActualDir,
                    targetDirection = upperTargetDir,
                };
                upper.orientationDeviation = AngleBetween(upper.actualDirection, upper.targetDirection);

                // Lower segment: distal end, direction falls back to last bone
                var lower = new SegmentDeviation
                {
                    worldPosition = dd.actualJointPositions[n - 1],
                    targetPosition = dd.targetJointPositions[n - 1],
                    positionDeviation = dd.deviations[n - 1],
                    actualDirection = upperActualDir,
                    targetDirection = upperTargetDir,
                };
                lower.orientationDeviation = upper.orientationDeviation;

                results[c - 1] = new LimbDeviations
                {
                    shoulder = shoulder,
                    upper = upper,
                    lower = lower,
                    avgPositionDeviation = (shoulder.positionDeviation + upper.positionDeviation + lower.positionDeviation) / 3f,
                    avgOrientationDeviation = (shoulder.orientationDeviation + upper.orientationDeviation + lower.orientationDeviation) / 3f
                };
            }
            return results;
        }

        private BodyDeviations? ComputeBodyDeviations()
        {
            var torso = controller.TorsoBody;
            var rail = controller.RailSamples;
            if (torso == null || rail == null || rail.Count < 2) return null;

            float railLength = GetRailLength(rail);
            Vector3 torsoWorld = torso.transform.position;
            float torsoS = ProjectToRail(rail, railLength, torsoWorld);
            float3 targetTorso = SampleRail(rail, railLength, torsoS).position;

            float posDev = math.length((float3)torsoWorld - targetTorso);
            
            // Orientation: actual torso rotation vs controller's desired torso rotation
            Quaternion actualRotation = torso.transform.rotation;
            Quaternion targetRotation = controller.DesiredTorsoRotation;
            float angle = Quaternion.Angle(actualRotation, targetRotation);

            return new BodyDeviations
            {
                worldPosition = torsoWorld,
                targetPosition = targetTorso,
                positionDeviation = posDev,
                worldRotation = actualRotation,
                targetRotation = targetRotation,
                orientationDeviation = angle
            };
        }

        private static float GetRailLength(IReadOnlyList<SplineChainController.RailSample> rail)
        {
            return rail.Count > 0 ? rail[rail.Count - 1].s : 0f;
        }

        private static float ProjectToRail(IReadOnlyList<SplineChainController.RailSample> rail, float railLength, Vector3 point)
        {
            float3 p = (float3)point;
            float bestS = 0f;
            float bestDistSq = float.MaxValue;

            for (int i = 1; i < rail.Count; i++)
            {
                float3 a = rail[i - 1].position;
                float3 b = rail[i].position;
                float3 segment = b - a;
                float segLenSq = math.lengthsq(segment);
                if (segLenSq < 1e-6f) continue;

                float t = math.saturate(math.dot(p - a, segment) / segLenSq);
                float3 closest = math.lerp(a, b, t);
                float distSq = math.lengthsq(p - closest);
                if (distSq < bestDistSq)
                {
                    bestDistSq = distSq;
                    bestS = math.lerp(rail[i - 1].s, rail[i].s, t);
                }
            }
            return math.clamp(bestS, 0f, railLength);
        }

        private static SplineChainController.RailSample SampleRail(IReadOnlyList<SplineChainController.RailSample> rail, float railLength, float s)
        {
            s = math.clamp(s, 0f, railLength);
            if (rail.Count < 2) return rail[0];

            for (int i = 1; i < rail.Count; i++)
            {
                if (rail[i].s >= s)
                {
                    float t = (rail[i].s - rail[i - 1].s > 1e-6f)
                        ? (s - rail[i - 1].s) / (rail[i].s - rail[i - 1].s)
                        : 0f;
                    return new SplineChainController.RailSample
                    {
                        s = s,
                        position = math.lerp(rail[i - 1].position, rail[i].position, t),
                        tangent = math.normalizesafe(math.lerp(rail[i - 1].tangent, rail[i].tangent, t), math.forward())
                    };
                }
            }
            return rail[rail.Count - 1];
        }

        private static float AngleBetween(float3 a, float3 b)
        {
            float dot = math.clamp(math.dot(a, b), -1f, 1f);
            return math.acos(dot) * Mathf.Rad2Deg;
        }

        private void RenderGraph()
        {
            if (graphTexture == null || glMaterial == null) return;

            RenderTexture.active = graphTexture;
            GL.Clear(true, true, backgroundColor);

            // Set up material for GL rendering
            glMaterial.SetPass(0);

            GL.PushMatrix();
            GL.LoadPixelMatrix(0, graphWidth, graphHeight, 0);

            DrawGrid();
            DrawHistoryCurves();

            GL.PopMatrix();
            RenderTexture.active = null;
        }

        private void DrawGrid()
        {
            GL.Begin(GL.LINES);
            GL.Color(gridColor);

            int hLines = 5;
            for (int i = 0; i <= hLines; i++)
            {
                float y = (float)i / hLines * graphHeight;
                GL.Vertex3(0, y, 0);
                GL.Vertex3(graphWidth, y, 0);
            }

            int vLines = 10;
            for (int i = 0; i <= vLines; i++)
            {
                float x = (float)i / vLines * graphWidth;
                GL.Vertex3(x, 0, 0);
                GL.Vertex3(x, graphHeight, 0);
            }

            GL.End();
        }

        private void DrawHistoryCurves()
        {
            if (historyCount < 2) return;

            float timeRange = historyDuration;
            float oldestTime = timeAccumulator - timeRange;
            if (oldestTime < 0) oldestTime = 0;

            // Position curves - per segment per limb
            DrawCurve(h => h.posDev0_shoulder, maxPositionDeviation, positionColor, oldestTime, timeRange);
            DrawCurve(h => h.posDev0_upper, maxPositionDeviation, new Color(positionColor.r, positionColor.g, positionColor.b, 0.7f), oldestTime, timeRange);
            DrawCurve(h => h.posDev0_lower, maxPositionDeviation, new Color(positionColor.r, positionColor.g, positionColor.b, 0.4f), oldestTime, timeRange);
            DrawCurve(h => h.posDev1_shoulder, maxPositionDeviation, new Color(positionColor.r, positionColor.g, positionColor.b, 0.6f), oldestTime, timeRange);
            DrawCurve(h => h.posDev1_upper, maxPositionDeviation, new Color(positionColor.r, positionColor.g, positionColor.b, 0.45f), oldestTime, timeRange);
            DrawCurve(h => h.posDev1_lower, maxPositionDeviation, new Color(positionColor.r, positionColor.g, positionColor.b, 0.3f), oldestTime, timeRange);

            // Orientation curves
            DrawCurve(h => h.oriDev0_shoulder, maxOrientationDeviation, orientationColor, oldestTime, timeRange);
            DrawCurve(h => h.oriDev0_upper, maxOrientationDeviation, new Color(orientationColor.r, orientationColor.g, orientationColor.b, 0.7f), oldestTime, timeRange);
            DrawCurve(h => h.oriDev0_lower, maxOrientationDeviation, new Color(orientationColor.r, orientationColor.g, orientationColor.b, 0.4f), oldestTime, timeRange);
            DrawCurve(h => h.oriDev1_shoulder, maxOrientationDeviation, new Color(orientationColor.r, orientationColor.g, orientationColor.b, 0.6f), oldestTime, timeRange);
            DrawCurve(h => h.oriDev1_upper, maxOrientationDeviation, new Color(orientationColor.r, orientationColor.g, orientationColor.b, 0.45f), oldestTime, timeRange);
            DrawCurve(h => h.oriDev1_lower, maxOrientationDeviation, new Color(orientationColor.r, orientationColor.g, orientationColor.b, 0.3f), oldestTime, timeRange);

            // Body curves
            DrawCurve(h => h.bodyPosDev, maxPositionDeviation, Color.white, oldestTime, timeRange);
            DrawCurve(h => h.bodyOriDev, maxOrientationDeviation, Color.cyan, oldestTime, timeRange);

            // Threshold lines
            DrawThresholdLine(positionWarningThreshold / maxPositionDeviation, warningColor);
            DrawThresholdLine(positionErrorThreshold / maxPositionDeviation, errorColor);
            DrawThresholdLine(orientationWarningThreshold / maxOrientationDeviation, warningColor);
            DrawThresholdLine(orientationErrorThreshold / maxOrientationDeviation, errorColor);
        }

        private void DrawCurve(Func<LimbDeviationHistory, float> valueSelector, float maxValue, Color color, float oldestTime, float timeRange)
        {
            GL.Begin(GL.LINE_STRIP);
            GL.Color(color);

            for (int k = 0; k < historyCount; k++)
            {
                int idx = (historyHead - historyCount + k + historyCapacity) % historyCapacity;
                var frame = history[idx];
                if (frame.time < timeAccumulator - historyDuration) continue;

                float t = (frame.time - oldestTime) / timeRange;
                t = Mathf.Clamp01(t);
                float x = t * graphWidth;

                float value = valueSelector(frame);
                float normalized = Mathf.Clamp01(value / maxValue);
                float y = graphHeight - normalized * graphHeight;

                GL.Vertex3(x, y, 0);
            }

            GL.End();
        }

        private void DrawThresholdLine(float normalizedHeight, Color color)
        {
            float y = graphHeight - Mathf.Clamp01(normalizedHeight) * graphHeight;
            GL.Begin(GL.LINES);
            GL.Color(new Color(color.r, color.g, color.b, 0.5f));
            GL.Vertex3(0, y, 0);
            GL.Vertex3(graphWidth, y, 0);
            GL.End();
        }

        /// <summary>Gets the current graph texture for UI display.</summary>
        public RenderTexture GraphTexture => graphTexture;

        /// <summary>Current limb count being tracked.</summary>
        public int LimbsTracked => limbsTracked;

        /// <summary>Public entry point to force a tracker update (used by editor scripts).</summary>
        public void UpdateTracker() => Update();

        /// <summary>Latest detailed limb deviations per limb.</summary>
        public LimbDeviations[] GetLatestLimbDeviations()
        {
            if (historyCount == 0) return Array.Empty<LimbDeviations>();
            int idx = (historyHead - 1 + historyCapacity) % historyCapacity;
            var frame = history[idx];
            var deviations = ComputeDeviations();
            return deviations ?? Array.Empty<LimbDeviations>();
        }

        /// <summary>Latest body (torso) deviations.</summary>
        public BodyDeviations? GetLatestBodyDeviations()
        {
            if (historyCount == 0) return null;
            return ComputeBodyDeviations();
        }

        /// <summary>Latest position deviations per limb (average of 3 segments).</summary>
        public float[] GetLatestPositionDeviations()
        {
            if (historyCount == 0) return Array.Empty<float>();
            int idx = (historyHead - 1 + historyCapacity) % historyCapacity;
            var frame = history[idx];
            return new[] 
            { 
                (frame.posDev0_shoulder + frame.posDev0_upper + frame.posDev0_lower) / 3f,
                (frame.posDev1_shoulder + frame.posDev1_upper + frame.posDev1_lower) / 3f
            };
        }

        /// <summary>Latest orientation deviations per limb (average of 3 segments).</summary>
        public float[] GetLatestOrientationDeviations()
        {
            if (historyCount == 0) return Array.Empty<float>();
            int idx = (historyHead - 1 + historyCapacity) % historyCapacity;
            var frame = history[idx];
            return new[] 
            { 
                (frame.oriDev0_shoulder + frame.oriDev0_upper + frame.oriDev0_lower) / 3f,
                (frame.oriDev1_shoulder + frame.oriDev1_upper + frame.oriDev1_lower) / 3f
            };
        }

        /// <summary>Gets the full history data for export/visualization.</summary>
        public List<object> GetHistoryData()
        {
            var list = new List<object>();
            for (int k = 0; k < historyCount; k++)
            {
                int idx = (historyHead - historyCount + k + historyCapacity) % historyCapacity;
                var frame = history[idx];
                list.Add(new
                {
                    time = frame.time,
                    limb0_shoulder_pos = frame.posDev0_shoulder,
                    limb0_upper_pos = frame.posDev0_upper,
                    limb0_lower_pos = frame.posDev0_lower,
                    limb1_shoulder_pos = frame.posDev1_shoulder,
                    limb1_upper_pos = frame.posDev1_upper,
                    limb1_lower_pos = frame.posDev1_lower,
                    limb0_shoulder_ori = frame.oriDev0_shoulder,
                    limb0_upper_ori = frame.oriDev0_upper,
                    limb0_lower_ori = frame.oriDev0_lower,
                    limb1_shoulder_ori = frame.oriDev1_shoulder,
                    limb1_upper_ori = frame.oriDev1_upper,
                    limb1_lower_ori = frame.oriDev1_lower,
                    body_pos = frame.bodyPosDev,
                    body_ori = frame.bodyOriDev,
                });
            }
            return list;
        }

        private void OnValidate()
        {
            if (graphTexture != null && (graphTexture.width != graphWidth || graphTexture.height != graphHeight))
            {
                ReleaseResources();
                InitializeGraph();
            }
        }
    }
}