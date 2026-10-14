using Box3D;
using Box3D.Hybrid;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Box shape used by Box3DRigBuilder when a node-pair collision matrix is
    /// authored: identical to Box3DBoxShape, but the shape def opts into the
    /// Box3D custom-filter callback so the builder's per-node matrix is
    /// consulted for this shape. Without the flag the callback is never
    /// called for the pair, so plain Box3DBoxShape is used when no rules exist.
    /// </summary>
    public sealed class RigBoxShape : Box3DBoxShape
    {
        protected override ShapeDef BuildDef()
        {
            ShapeDef def = base.BuildDef();
            def.EnableCustomFiltering = true;
            return def;
        }
    }

    /// <summary>
    /// Capsule shape used by Box3DRigBuilder when a node-pair collision matrix
    /// is authored: identical to Box3DCapsuleShape, but the shape def opts
    /// into the Box3D custom-filter callback (same pattern as RigBoxShape).
    /// </summary>
    public sealed class RigCapsuleShape : Box3DCapsuleShape
    {
        protected override ShapeDef BuildDef()
        {
            ShapeDef def = base.BuildDef();
            def.EnableCustomFiltering = true;
            return def;
        }
    }
}