using UnityEditor;

namespace Box3D.Hybrid.Editor
{
    /// <summary>Adds a scrub timeline to <see cref="Box3DReplayer"/>.</summary>
    [CustomEditor(typeof(Box3DReplayer))]
    public class Box3DReplayerEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            ReplayTimelineGUI.Draw(this, (Box3DReplayer)target);
        }
    }
}
