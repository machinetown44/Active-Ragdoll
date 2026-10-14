using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;
using Box3D.Hybrid;

namespace Box3DArticulation
{
    [DisallowMultipleComponent]
    [ExecuteAlways]
    public sealed class Box3DDebugVisualization : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private SplineChainController controller;

        [Header("Visualization Settings")]
        [SerializeField] private bool drawColliders = true;
        [SerializeField] private bool drawTargetSpline = true;
        [SerializeField] private bool drawProjectedPoses = true;
        [SerializeField] private bool drawRailTangents = true;
        [SerializeField] private bool drawBodyAxes = true;

        [Header("Colors")]
        [SerializeField] private Color colliderColor = new Color(0.2f, 0.8f, 0.4f, 0.5f);
        [SerializeField] private Color colliderWireColor = new Color(0.2f, 0.8f, 0.4f, 1f);
        [SerializeField] private Color targetSplineColor = new Color(0.9f, 0.5f, 0.2f, 1f);
        [SerializeField] private Color projectedPoseColor = new Color(0.3f, 0.7f, 1f, 0.5f);
        [SerializeField] private Color projectedWireColor = new Color(0.3f, 0.7f, 1f, 1f);
        [SerializeField] private Color railTangentColor = new Color(1f, 0.5f, 0f, 1f);
        [SerializeField] private Color bodyAxisColor = Color.white;

        [Header("Spline Settings")]
        [SerializeField] private int splineSegments = 100;
        [SerializeField] private float tangentLength = 0.2f;

        private void Awake()
        {
            if (controller == null)
                controller = GetComponent<SplineChainController>();
        }

        private void OnDrawGizmos()
        {
            if (controller == null) return;

            DrawColliders();
            DrawTargetSpline();
            DrawProjectedPoses();
            DrawRailTangents();
            DrawBodyAxes();
        }

        private void OnDrawGizmosSelected()
        {
            // Additional detail when selected
            OnDrawGizmos();
        }

        private void DrawColliders()
        {
            if (!drawColliders || controller.Limbs == null) return;

            var torso = controller.TorsoBody;
            if (torso != null)
            {
                DrawCapsuleCollider(torso, colliderColor, colliderWireColor);
            }

            foreach (var limb in controller.Limbs)
            {
                if (limb.upperBody != null)
                    DrawCapsuleCollider(limb.upperBody, colliderColor, colliderWireColor);
                if (limb.lowerBody != null)
                    DrawCapsuleCollider(limb.lowerBody, colliderColor, colliderWireColor);
            }
        }

        private void DrawCapsuleCollider(Box3DBody body, Color fillColor, Color wireColor)
        {
            var capsule = body.GetComponent<UnityEngine.CapsuleCollider>();
            if (capsule == null) return;

            Matrix4x4 matrix = body.transform.localToWorldMatrix;
            Gizmos.matrix = matrix;

            Vector3 center = capsule.center;
            float height = capsule.height;
            float radius = capsule.radius;
            int direction = capsule.direction;

            // Draw wire capsule approximation
            Gizmos.color = wireColor;

            Vector3 p1, p2;
            switch (direction)
            {
                case 0: // X-axis
                    p1 = center + Vector3.left * (height * 0.5f - radius);
                    p2 = center + Vector3.right * (height * 0.5f - radius);
                    break;
                case 1: // Y-axis
                    p1 = center + Vector3.down * (height * 0.5f - radius);
                    p2 = center + Vector3.up * (height * 0.5f - radius);
                    break;
                default: // Z-axis
                    p1 = center + Vector3.back * (height * 0.5f - radius);
                    p2 = center + Vector3.forward * (height * 0.5f - radius);
                    break;
            }

            // Draw wire capsule using lines
            DrawWireCapsule(p1, p2, radius, direction);

            // Fill
            Gizmos.color = fillColor;
            DrawSolidCapsule(p1, p2, radius, direction);

            Gizmos.matrix = Matrix4x4.identity;
        }

        private void DrawWireCapsule(Vector3 p1, Vector3 p2, float radius, int direction)
        {
            Vector3 up, right, forward;
            GetAxes(direction, out up, out right, out forward);

            int segments = 12;
            float halfHeight = Vector3.Distance(p1, p2) * 0.5f;
            Vector3 center = (p1 + p2) * 0.5f;

            // Draw rings at p1 and p2
            DrawCircle(p1, up, radius, segments);
            DrawCircle(p2, up, radius, segments);

            // Draw side lines
            for (int i = 0; i < segments; i += 3)
            {
                float angle = i * Mathf.PI * 2f / segments;
                Vector3 offset = (right * Mathf.Cos(angle) + forward * Mathf.Sin(angle)) * radius;
                Gizmos.DrawLine(p1 + offset, p2 + offset);
            }
        }

        private void DrawSolidCapsule(Vector3 p1, Vector3 p2, float radius, int direction)
        {
            // Simple filled representation - draw center sphere and cylinder
            Vector3 up, right, forward;
            GetAxes(direction, out up, out right, out forward);

            Gizmos.DrawSphere(p1, radius);
            Gizmos.DrawSphere(p2, radius);
        }

        private void DrawCircle(Vector3 center, Vector3 normal, float radius, int segments)
        {
            Vector3 up = normal.normalized;
            Vector3 right = Vector3.Cross(up, Vector3.forward);
            if (right.sqrMagnitude < 0.001f)
                right = Vector3.Cross(up, Vector3.up);
            right.Normalize();
            Vector3 forward = Vector3.Cross(up, right);

            Vector3 prev = center + right * radius;
            for (int i = 1; i <= segments; i++)
            {
                float angle = i * Mathf.PI * 2f / segments;
                Vector3 curr = center + (right * Mathf.Cos(angle) + forward * Mathf.Sin(angle)) * radius;
                Gizmos.DrawLine(prev, curr);
                prev = curr;
            }
        }

        private void GetAxes(int direction, out Vector3 up, out Vector3 right, out Vector3 forward)
        {
            switch (direction)
            {
                case 0: // X
                    up = Vector3.right;
                    right = Vector3.up;
                    forward = Vector3.forward;
                    break;
                case 1: // Y
                    up = Vector3.up;
                    right = Vector3.right;
                    forward = Vector3.forward;
                    break;
                default: // Z
                    up = Vector3.forward;
                    right = Vector3.right;
                    forward = Vector3.up;
                    break;
            }
        }

        private void DrawTargetSpline()
        {
            if (!drawTargetSpline) return;

            var rail = controller.RailSamples;
            if (rail == null || rail.Count < 2) return;

            Gizmos.color = targetSplineColor;

            for (int i = 1; i < rail.Count; i++)
            {
                Gizmos.DrawLine(rail[i - 1].position, rail[i].position);
            }
        }

        private void DrawProjectedPoses()
        {
            if (!drawProjectedPoses || controller.Limbs == null) return;

            var rail = controller.RailSamples;
            if (rail == null || rail.Count < 2) return;

            float railLength = GetRailLength(rail);
            var torso = controller.TorsoBody;

            // Project torso
            if (torso != null)
            {
                float torsoS = ProjectToRail(rail, railLength, torso.transform.position);
                var target = SampleRail(rail, railLength, torsoS);
                DrawProjectedCapsule(target.position, target.tangent, torso, projectedPoseColor, projectedWireColor);
            }

            // Project limbs
            foreach (var limb in controller.Limbs)
            {
                if (limb.upperBody != null)
                {
                    float s = ProjectToRail(rail, railLength, limb.upperBody.transform.position);
                    var target = SampleRail(rail, railLength, s);
                    DrawProjectedCapsule(target.position, target.tangent, limb.upperBody, projectedPoseColor, projectedWireColor);
                }
                if (limb.lowerBody != null)
                {
                    float s = ProjectToRail(rail, railLength, limb.lowerBody.transform.position);
                    var target = SampleRail(rail, railLength, s);
                    DrawProjectedCapsule(target.position, target.tangent, limb.lowerBody, projectedPoseColor, projectedWireColor);
                }
            }
        }

        private void DrawProjectedCapsule(Vector3 position, Vector3 tangent, Box3DBody body, Color fillColor, Color wireColor)
        {
            var capsule = body.GetComponent<UnityEngine.CapsuleCollider>();
            if (capsule == null) return;

            Gizmos.matrix = Matrix4x4.identity;

            Vector3 p1, p2;
            float height = capsule.height;
            float radius = capsule.radius;

            Vector3 halfHeightVec = tangent * (height * 0.5f - radius);
            p1 = position - halfHeightVec;
            p2 = position + halfHeightVec;

            Gizmos.color = wireColor;
            DrawWireCapsuleAligned(p1, p2, radius, tangent);

            Gizmos.color = fillColor;
            Gizmos.DrawSphere(p1, radius);
            Gizmos.DrawSphere(p2, radius);
        }

        private void DrawWireCapsuleAligned(Vector3 p1, Vector3 p2, float radius, Vector3 tangent)
        {
            Vector3 up = tangent.normalized;
            Vector3 right = Vector3.Cross(up, Vector3.forward);
            if (right.sqrMagnitude < 0.001f)
                right = Vector3.Cross(up, Vector3.up);
            right.Normalize();
            Vector3 forward = Vector3.Cross(up, right);

            int segments = 12;
            DrawCircle(p1, up, radius, segments);
            DrawCircle(p2, up, radius, segments);

            for (int i = 0; i < segments; i += 3)
            {
                float angle = i * Mathf.PI * 2f / segments;
                Vector3 offset = (right * Mathf.Cos(angle) + forward * Mathf.Sin(angle)) * radius;
                Gizmos.DrawLine(p1 + offset, p2 + offset);
            }
        }

        private void DrawRailTangents()
        {
            if (!drawRailTangents) return;

            var rail = controller.RailSamples;
            if (rail == null || rail.Count < 2) return;

            Gizmos.color = railTangentColor;

            int step = Mathf.Max(1, rail.Count / 20);
            for (int i = 0; i < rail.Count; i += step)
            {
                Vector3 pos = rail[i].position;
                Vector3 tangent = rail[i].tangent;
                Gizmos.DrawLine(pos, pos + tangent * tangentLength);
            }
        }

        private void DrawBodyAxes()
        {
            if (!drawBodyAxes || controller.Limbs == null) return;

            var torso = controller.TorsoBody;
            if (torso != null)
            {
                DrawAxes(torso.transform);
            }

            foreach (var limb in controller.Limbs)
            {
                if (limb.upperBody != null)
                    DrawAxes(limb.upperBody.transform);
                if (limb.lowerBody != null)
                    DrawAxes(limb.lowerBody.transform);
            }
        }

        private void DrawAxes(Transform t)
        {
            Gizmos.matrix = Matrix4x4.identity;
            float size = 0.15f;

            Gizmos.color = Color.red;
            Gizmos.DrawLine(t.position, t.position + t.right * size);

            Gizmos.color = Color.green;
            Gizmos.DrawLine(t.position, t.position + t.up * size);

            Gizmos.color = Color.blue;
            Gizmos.DrawLine(t.position, t.position + t.forward * size);
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
    }
}