using Unity.Mathematics;
using UnityEngine;

namespace Box3D.Hybrid
{
    /// <summary>A box shape, analogous to Unity's BoxCollider.</summary>
    [Icon("Packages/com.suvitruf.box3d/Box3D.Hybrid.Editor/Icons/Box3DBoxShape.png")]
    [AddComponentMenu("Box3D/Shapes/Box Shape")]
    public class Box3DBoxShape : Box3DShape
    {
        [SerializeField, Tooltip("Full box size in local units (like Unity's BoxCollider.size).")]
        private Vector3 Size = Vector3.one;

        protected override Shape CreateShape(Body body, float3 localPosition, quaternion localRotation, float3 scale)
        {
            float3 halfExtents = (float3)Size * 0.5f * math.abs(scale);
            var frame = new B3Transform
            {
                Position = ShapeCenter(localPosition, localRotation, scale),
                Rotation = localRotation,
            };
            BoxHull hull = BoxHull.CreateTransformed(halfExtents.x, halfExtents.y, halfExtents.z, frame);
            return body.CreateHullShape(BuildDef(), in hull);
        }

        private void OnDrawGizmosSelected()
        {
            SetGizmoFrame();
            float3 size = (float3)Size * math.abs((float3)transform.lossyScale);
            Gizmos.DrawWireCube((Vector3)ScaledCenter, (Vector3)size);
        }
    }
}
