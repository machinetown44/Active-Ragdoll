using System;
using System.Collections.Generic;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Serialized rest-pose bind data for a HumanoidMotionDriver. Captured
    /// once from a driver whose endpoint points sit on a source character in
    /// the desired rest pose (t-pose), then reused by any driver whose
    /// endpoint targets live on a DIFFERENT character instance or a source
    /// that cannot reach the rest pose at bind time.
    ///
    /// Storage is three points per endpoint (tip/pivot/roll) in the RIG
    /// ROOT's local space - the frame both capture and bind have in common.
    /// Any driver using the same rig config consumes the asset regardless
    /// of where its own component sits in the hierarchy. Everything else
    /// the bind derives - rest directions, chain arcs, hinge datums, bind
    /// frames - follows deterministically from these plus the rig config.
    /// </summary>
    [CreateAssetMenu(menuName = "Box3DArticulation/Humanoid Motion Bind Asset",
                     fileName = "HumanoidMotionBind")]
    public sealed class HumanoidMotionBindAsset : ScriptableObject
    {
        [Serializable]
        public class BindPoint
        {
            [Tooltip("Endpoint (rig node) name this entry supplies rest data for.")]
            public string nodeName = "";
            [Tooltip("Captured tip point, rig-root local, at the rest pose.")]
            public Vector3 tip;
            [Tooltip("Captured pivot point, rig-root local, at the rest pose.")]
            public Vector3 pivot;
            [Tooltip("Captured roll point, rig-root local, at the rest pose (chain references).")]
            public Vector3 roll;
        }

        [Tooltip("One entry per endpoint. Matched by node name at bind; entries the endpoint list lacks are ignored, missing entries fall back to live point resolution (with a warning).")]
        public List<BindPoint> points = new List<BindPoint>();

        [SerializeField, Tooltip("Storage frame version. 0 = legacy driver-local (ignored at bind), 1 = rig-root local.")]
        private int frameVersion;

        public const int CurrentFrameVersion = 1; // rig-root local

        /// <summary>False for legacy driver-local captures - consumers must
        /// ignore the asset rather than mis-bind.</summary>
        public bool IsRigRootFrame => frameVersion >= CurrentFrameVersion;

        /// <summary>Marks the capture as rig-root framed (call on capture).</summary>
        public void SetRigRootFrame() => frameVersion = CurrentFrameVersion;

        /// <summary>Linear scan - bind-time only, endpoint counts are tiny.</summary>
        public BindPoint Find(string nodeName)
        {
            for (int i = 0; i < points.Count; i++)
                if (points[i].nodeName == nodeName) return points[i];
            return null;
        }
    }
}
