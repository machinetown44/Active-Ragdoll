// ----------------------------------------------------------------------------
// -- MxM Preprocess Wrapper — EditorWindow hosting the stock                --
// -- MxMPreProcessData inspector in a collapsible, draggable, scrollable    --
// -- side pane. Built on id.elemtk (EmbeddedInspector + CollapsibleSidePane).
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using id.elemtk.Framework;
using MxMEditor;
using UnityEditor;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace MxMWrapper
{
    /// <summary>
    /// Wraps the MxM pre-process asset: workflow panel on the left, the
    /// stock MxMPreProcessData inspector hosted in a collapsible side pane
    /// on the right (horizontal collapse animation, drag-resizable
    /// separator, scrollable content). The embedded inspector is the real
    /// custom editor (Editor.CreateEditor) — fully interactive.
    /// </summary>
    public class MxMPreprocessWrapperWindow : EditorWindow
    {
        private const string k_Menu = "Tools/MxM/Preprocess Wrapper";

        private MxMPreProcessData m_data;
        private EmbeddedInspector m_embedded;
        private CollapsibleSidePane m_pane;
        private PannableCanvas m_canvas;
        private ObjectField m_assetField;
        private CensusDataPanel m_censusData;
        private CensusPreviewPanel m_censusPreview;
        private CensusTimelinePanel m_censusTimeline;

        [MenuItem(k_Menu)]
        public static void ShowWindow()
        {
            var win = GetWindow<MxMPreprocessWrapperWindow>();
            win.titleContent = new GUIContent("MxM Preprocess");
            win.minSize = new Vector2(900f, 500f);
        }

        private void OnEnable()
        {
            BuildUI();
            if (m_data == null)
                AutoFindData();
        }

        private void OnDisable()
        {
            if (m_embedded != null)
            {
                m_embedded.Dispose();
                m_embedded = null;
            }

            if (m_censusPreview != null)
            {
                m_censusPreview.Dispose();
                m_censusPreview = null;
            }
        }

        private void AutoFindData()
        {
            // Convention: the demo pre-process asset by name. First hit wins;
            // the object field allows an explicit override at any time.
            string[] guids = AssetDatabase.FindAssets("MocapPreProcessData t:MxMPreProcessData");
            if (guids.Length == 0)
                return;

            string path = AssetDatabase.GUIDToAssetPath(guids[0]);
            SetData(AssetDatabase.LoadAssetAtPath<MxMPreProcessData>(path));
        }

        private void SetData(MxMPreProcessData data)
        {
            if (m_data == data)
                return;

            m_data = data;
            m_embedded?.SetTarget(data);
            m_censusData?.Refresh(data);
            m_censusPreview?.SetData(data);
        }

        private void BuildUI()
        {
            rootVisualElement.Clear();

            var header = new Toolbar();
            m_assetField = new ObjectField("Preprocess Data")
            {
                objectType = typeof(MxMPreProcessData),
                style = { flexGrow = 1f }
            };
            if (m_data != null) m_assetField.SetValueWithoutNotify(m_data);
            m_assetField.RegisterValueChangedCallback(evt => SetData(evt.newValue as MxMPreProcessData));
            header.Add(m_assetField);
            rootVisualElement.Add(header);

            // Pane is right-aligned: the absolute canvas takes no flex
            // space, so justifyContent does the pushing. Canvas paints
            // first (tree order) → visible through the pane's gutter.
            var split = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    flexGrow = 1f,
                    justifyContent = Justify.FlexEnd
                }
            };
            rootVisualElement.Add(split);

            // Left: pannable canvas — absolute backdrop spanning the full
            // split area, painted first; the pane's transparent gutter then
            // shows the grid through it (grip/divider paint over). Collapsing
            // the pane visually grows the canvas for free.
            m_canvas = new PannableCanvas();
            m_canvas.style.position = Position.Absolute;
            m_canvas.style.left = 0f; m_canvas.style.top = 0f;
            m_canvas.style.right = 0f; m_canvas.style.bottom = 0f;
            split.Add(m_canvas);

            // Census panels on the grid: data | preview, timeline below
            // spanning both. Self-contained blocks; selection flows
            // data → (preview, timeline); scrub flows timeline → preview.
            m_censusData = new CensusDataPanel();
            m_censusPreview = new CensusPreviewPanel();
            m_censusTimeline = new CensusTimelinePanel();

            m_censusData.EntrySelected += entry =>
            {
                m_censusPreview.SetEntry(entry);
                m_censusTimeline.Bind(entry,
                    () => m_censusPreview.CurrentTime,
                    () => (m_censusPreview.WindowStart, m_censusPreview.WindowEnd));
            };
            m_censusPreview.TimeChanged += _ => m_censusTimeline.MarkDirtyRepaint();
            m_censusTimeline.TimeRequested += t => m_censusPreview.SetTime(t);

            m_canvas.AddNode(m_censusData, 12f, 12f);
            m_canvas.AddNode(m_censusPreview, 12f + CensusDataPanel.k_Width + 16f, 12f);
            m_canvas.AddNode(m_censusTimeline, 12f,
                12f + 520f + 12f);

            // Right: stock inspector in a horizontal collapsible pane
            // (width tween, drag separator, scrollable content). Normal flex
            // sibling above the absolute canvas; pane background transparent
            // so the grid reads through the gutter below the grip.
            m_embedded = new EmbeddedInspector();
            if (m_data != null) m_embedded.SetTarget(m_data);
            if (m_data != null)
            {
                m_censusData.Refresh(m_data);
                m_censusPreview.SetData(m_data);
            }

            m_pane = new CollapsibleSidePane("Preprocess Inspector", 460f);
            m_pane.Content.Add(m_embedded);
            m_pane.style.backgroundColor = new Color(0f, 0f, 0f, 0f);
            split.Add(m_pane);
        }
    }
}
#endif