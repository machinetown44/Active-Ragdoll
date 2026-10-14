using UnityEditor;

namespace Box3D.Hybrid.Editor
{
    /// <summary>Adds a scrub timeline to <see cref="Box3DVisualReplayer"/>.</summary>
    [CustomEditor(typeof(Box3DVisualReplayer))]
    public class Box3DVisualReplayerEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            ReplayTimelineGUI.Draw(this, (Box3DVisualReplayer)target);
        }
    }
}
