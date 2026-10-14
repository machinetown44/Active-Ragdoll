using System.Runtime.InteropServices;
using Unity.Mathematics;

namespace Box3D
{
    /// <summary>Mirrors native b3Plane (16 bytes). Separation = dot(normal, point) - offset.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct B3Plane
    {
        public float3 Normal;
        public float Offset;
    }

    /// <summary>Mirrors native b3PlaneResult (44 bytes): the separating plane between a mover and
    /// a shape, the closest point on the shape, the true surface facet normal (mesh triangle
    /// / height-field cell) oriented to agree with Plane.Normal (zero vector for analytic shapes
    /// — fall back to Plane.Normal), and the index of the triangle that produced the plane
    /// (height-field convention: cellIndex * 2 + corner; −1 for facet-less shapes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct PlaneResult
    {
        public B3Plane Plane;
        public float3 Point;
        public float3 FacetNormal;
        public int TriangleIndex;
    }

    /// <summary>Mirrors native b3CollisionPlane (28 bytes). Input/output of the plane solver:
    /// PushLimit = float.MaxValue for rigid planes (lower = soft); Push is written by the solver;
    /// ClipVelocity should be false for soft planes.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CollisionPlane
    {
        public B3Plane Plane;
        public float PushLimit;
        public float Push;
        public NativeBool ClipVelocity;
    }

    /// <summary>Mirrors native b3PlaneSolverResult (16 bytes).</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct PlaneSolverResult
    {
        public float3 Delta;
        public int IterationCount;
    }

    /// <summary>Per-plane collision details gathered by
    /// <see cref="World.CollideMover(float3, in Capsule, QueryFilter, System.Span{CollisionPlane}, System.Span{MoverHit}, float)"/>,
    /// index-aligned with the CollisionPlane buffer: the shape that produced the plane, the
    /// closest point on that shape in world space (may not be unique for deep overlaps), the
    /// true facet normal of the surface that produced the plane (falls back to Plane.Normal for
    /// facet-less shapes), and the index of the mesh/height-field triangle that produced the
    /// plane (−1 for facet-less shapes) — enables per-triangle material/attribute lookups
    /// (e.g. density or UV sampling on the Unity mesh). Use for debug drawing, pushing dynamic
    /// bodies, per-surface tuning, or facet-accurate force directions.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct MoverHit
    {
        public ShapeId ShapeId;
        public float3 Point;
        public float3 Normal;
        public int TriangleIndex;
    }
}
