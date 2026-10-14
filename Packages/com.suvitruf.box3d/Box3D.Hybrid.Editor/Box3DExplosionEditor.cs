using UnityEditor;
using UnityEngine;

namespace Box3D.Hybrid.Editor
{
    /// <summary>Explosion inspector: the normal fields plus an Explode button for testing blasts
    /// live (enabled in play mode only — exploding requires a stepping world).</summary>
    [CustomEditor(typeof(Box3DExplosion))]
    [CanEditMultipleObjects]
    public class Box3DExplosionEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            EditorGUILayout.Space();
            using (new EditorGUI.DisabledScope(!Application.isPlaying))
            {
                if (GUILayout.Button(Application.isPlaying ? "Explode" : "Explode (enter Play mode)"))
                {
                    foreach (Object t in targets)
                    {
                        ((Box3DExplosion)t).Explode();
                    }
                }
            }
        }
    }
}
