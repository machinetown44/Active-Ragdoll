// ----------------------------------------------------------------------------
// -- MxM Wrapper — live trajectory math shared by the live view (drawing)   --
// -- and the search runner (goal generation). Pure static functions over   --
// -- LiveSessionModel trajectory points.                                   --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEngine;

namespace MxMWrapper
{
    public static class LiveTrajectory
    {
        /// <summary>Catmull-Rom position at segment parameter t ∈ [0, n−1].</summary>
        public static Vector2 CatmullRom(List<TrajectoryPoint> pts, float t)
        {
            int n = pts.Count;
            if (n == 0)
                return Vector2.zero;
            if (n == 1)
                return pts[0].Position;
            t = Mathf.Clamp(t, 0f, n - 1f);
            int i = Mathf.Min((int)t, n - 2);
            float u = t - i;
            Vector2 p0 = pts[Mathf.Max(0, i - 1)].Position;
            Vector2 p1 = pts[i].Position;
            Vector2 p2 = pts[i + 1].Position;
            Vector2 p3 = pts[Mathf.Min(n - 1, i + 2)].Position;
            float u2 = u * u, u3 = u2 * u;
            return 0.5f * ((2f * p1) + (-p0 + p2) * u +
                (2f * p0 - 5f * p1 + 4f * p2 - p3) * u2 +
                (-p0 + 3f * p1 - 3f * p2 + p3) * u3);
        }

        /// <summary>Polyline length of the spline (coarse but stable).</summary>
        public static float Length(List<TrajectoryPoint> pts)
        {
            if (pts.Count < 2)
                return 0f;
            float len = 0f;
            Vector2 prev = pts[0].Position;
            int samples = Mathf.Max(8, pts.Count * 12);
            for (int i = 1; i <= samples; ++i)
            {
                Vector2 w = CatmullRom(pts, i / (float)samples * (pts.Count - 1));
                len += (w - prev).magnitude;
                prev = w;
            }
            return len;
        }

        /// <summary>Spline parameter for a distance traveled at the given
        /// speed (clamped at the spline end).</summary>
        public static float DistToT(List<TrajectoryPoint> pts, float totalLen, float dist)
        {
            if (totalLen < 1e-4f || pts.Count < 2)
                return 0f;
            dist = Mathf.Min(dist, totalLen);
            // Invert Length() by walking the same sampling.
            int samples = Mathf.Max(8, pts.Count * 12);
            float walked = 0f;
            Vector2 prev = pts[0].Position;
            for (int i = 1; i <= samples; ++i)
            {
                Vector2 w = CatmullRom(pts, i / (float)samples * (pts.Count - 1));
                float seg = (w - prev).magnitude;
                if (walked + seg >= dist)
                {
                    float frac = seg > 1e-6f ? (dist - walked) / seg : 0f;
                    return (i - 1 + frac) / (float)samples * (pts.Count - 1);
                }
                walked += seg;
                prev = w;
            }
            return pts.Count - 1f;
        }

        /// <summary>World position at a time: distance = speed × time along
        /// the spline.</summary>
        public static Vector2 PosAtTime(List<TrajectoryPoint> pts, float time, float speed)
        {
            return CatmullRom(pts, DistToT(pts, Length(pts), speed * time));
        }

        /// <summary>Heading (yaw radians, 0 = +Z) at spline parameter t:
        /// tangent direction.</summary>
        public static float TangentAngleRad(List<TrajectoryPoint> pts, float t)
        {
            Vector2 a = CatmullRom(pts, Mathf.Max(0f, t - 0.05f));
            Vector2 b = CatmullRom(pts, Mathf.Min(pts.Count - 1f, t + 0.05f));
            Vector2 d = b - a;
            if (d.sqrMagnitude < 1e-8f)
                return 0f;
            return Mathf.Atan2(d.x, d.y);
        }

        /// <summary>Heading at a time: spline tangent, overridden by the
        /// nearest passed pinned look point (the last pinned point whose
        /// parameter is at-or-behind the current position governs until the
        /// next pinned point).</summary>
        public static float HeadingAtTime(List<TrajectoryPoint> pts, float time, float speed)
        {
            if (pts.Count < 2)
                return 0f;
            float totalLen = Length(pts);
            float t = DistToT(pts, totalLen, speed * time);

            int governing = -1;
            for (int k = 0; k < pts.Count; ++k)
            {
                if (pts[k].LookPinned && k <= t + 0.001f)
                    governing = k;
            }
            if (governing >= 0)
                return pts[governing].LookAngleRad;
            return TangentAngleRad(pts, t);
        }

        /// <summary>Wrap an angle difference in degrees to [-180, 180].</summary>
        public static float WrapAngleDeg(float diff)
        {
            diff -= Mathf.Floor(diff / 360f) * 360f;
            if (diff > 180f)
                diff -= 360f;
            return diff;
        }
    }
}
#endif