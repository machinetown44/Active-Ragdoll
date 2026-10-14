// ----------------------------------------------------------------------------
// -- MxM Studio — the workbench window.                                     --
// -- Work grid (pannable canvas, 5 panels): authoring tree | preview |      --
// -- timeline / baked tree (timeline spans tree columns).                   --
// -- Collapsible side panes host LEGACY EDITORS ONLY: left = pre-bake       --
// -- (authoring selection's stock MxM editor), right = post-bake            --
// -- (MxMAnimData inspector).                                               --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Text;
using id.elemtk.Framework;
using MxM;
using MxMEditor;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.UIElements;

namespace MxMWrapper
{
    public class MxMStudioWindow : EditorWindow
    {
        private const string k_Menu = "Tools/MxM/Studio";

        private MxMPreProcessData m_data;

        private readonly StudioSelection m_selection = new StudioSelection();
        private readonly StudioFilter m_filter = new StudioFilter();

        // Work grid
        private AuthoringTreePanel m_authoringTree;
        private BakedTreePanel m_bakedTree;
        private CensusPreviewPanel m_censusPreview;
        private CensusTimelinePanel m_censusTimeline;
        private LiveViewPanel m_liveView;
        private LiveSettingsPanel m_liveSettings;
        private LiveSearchRunner m_liveRunner;

        // Legacy editor panes
        private EmbeddedInspector m_authoringInspector;
        private EmbeddedInspector m_bakedInspector;
        private CollapsibleSidePane m_leftPane;
        private CollapsibleSidePane m_rightPane;

        private Label m_counts;
        private ObjectField m_assetField;

        [MenuItem(k_Menu)]
        public static void ShowWindow()
        {
            var win = GetWindow<MxMStudioWindow>();
            win.titleContent = new GUIContent("MxM Studio");
            win.minSize = new Vector2(1280f, 640f);
        }

        private void OnEnable()
        {
            m_filter.Query = EditorPrefs.GetString("MxMStudio:Filter:Query", string.Empty);
            BuildUI();
            if (m_data == null)
                AutoFindData();
            EditorApplication.delayCall += RefreshBakedBinding;
        }

        private void OnDisable()
        {
            if (m_censusPreview != null)
            {
                m_censusPreview.Dispose();
                m_censusPreview = null;
            }
            if (m_authoringInspector != null)
            {
                m_authoringInspector.Dispose();
                m_authoringInspector = null;
            }
            if (m_bakedInspector != null)
            {
                m_bakedInspector.Dispose();
                m_bakedInspector = null;
            }

            // Persist view state: tree expansion, query, pane collapse.
            if (m_authoringTree != null)
            {
                m_authoringTree.SaveExpandedState();
                m_authoringTree = null;
            }
            if (m_bakedTree != null)
            {
                m_bakedTree.SaveExpandedState();
                m_bakedTree = null;
            }
            if (m_leftPane != null)
                EditorPrefs.SetBool(k_PrefsLeftPane, m_leftPane.Expanded);
            if (m_rightPane != null)
                EditorPrefs.SetBool(k_PrefsRightPane, m_rightPane.Expanded);
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
            if (m_assetField != null)
                m_assetField.SetValueWithoutNotify(data);
            m_selection.Clear();
            m_selection.Dataset = data;
            m_selection.SetBakedData(data != null ? data.LastSavedAnimData : null);

            m_authoringTree?.SetDataset(data);
            m_censusPreview?.SetData(data);
            m_liveView?.SetRigPrefab(data != null ? data.Prefab : null);
            m_authoringInspector?.SetTarget(data);
            RefreshBakedBinding();
            RefreshCounts();
        }

        private void RefreshBakedBinding()
        {
            var baked = m_data != null ? m_data.LastSavedAnimData : null;
            m_selection.SetBakedData(baked);
            if (m_bakedTree != null)
                m_bakedTree.SetData(m_data, baked);
            m_bakedInspector?.SetTarget(baked);
            RefreshCounts();
        }

        /// <summary>Full teardown + rebuild from the toolbar Reload button.
        /// Equivalent to closing and reopening the window: disposes preview
        /// panels and the live runner, drops the cached dataset handle
        /// (re-polls LastSavedAnimData — the point after script changes),
        /// clears cached trees, then rebuilds the whole UI. The user's
        /// asset selection is preserved.</summary>
        private void ReloadAll()
        {
            var data = m_data; // preserve the user's selection
            OnDisable();
            m_selection.Clear();
            m_bakedClip = null;
            if (m_liveRunner != null)
            {
                m_liveRunner.Dispose();
                m_liveRunner = null;
            }
            m_censusTimeline?.Bind(null, null, null);
            rootVisualElement.Clear();

            // Force SetData to re-run end-to-end (it early-returns on the
            // same reference) — fresh asset loads + full tree rebuild.
            m_data = null;
            OnEnable();
            if (data != null)
                SetData(data);
        }

        // -- UI ------------------------------------------------------------------

        private void BuildUI()
        {
            rootVisualElement.Clear();

            // -- Header: dataset picker + typed counts ----------------------------
            var header = new Toolbar();

            m_assetField = new ObjectField("Preprocess Data")
            {
                objectType = typeof(MxMPreProcessData),
                style = { flexGrow = 1f }
            };
            if (m_data != null) m_assetField.SetValueWithoutNotify(m_data);
            m_assetField.RegisterValueChangedCallback(evt =>
                SetData(evt.newValue as MxMPreProcessData));
            header.Add(m_assetField);

            // Full re-init without the select-away dance after script
            // changes: dispose live state, clear caches, rebuild everything.
            var reloadBtn = new ToolbarButton(ReloadAll) { text = "Reload" };
            header.Add(reloadBtn);

            m_counts = new Label(string.Empty)
            {
                style = { paddingRight = 10f, unityTextAlign = TextAnchor.MiddleRight,
                          color = new Color(0.65f, 0.65f, 0.65f), fontSize = 11 }
            };
            header.Add(m_counts);

            // Shared filter: search + type toggles drive both trees.
            m_search = new ToolbarSearchField
            {
                style = { width = 240f, marginRight = 6f }
            };
            m_search.SetValueWithoutNotify(m_filter.Query);
            m_search.RegisterValueChangedCallback(evt => ApplyFilterQuery(evt.newValue));
            header.Add(m_search);

            // FullClip toggle: expands the time base past the composite
            // slice to the full primary clip (the composite is an authoring
            // slice; the clip plays whole in MM).
            m_fullClipToggle = new ToolbarToggle { text = "Full Clip" };
            m_fullClipToggle.SetValueWithoutNotify(m_selection.FullClip);
            m_fullClipToggle.RegisterValueChangedCallback(evt =>
            {
                m_selection.FullClip = evt.newValue;
                RebindCurrentUnit();
            });
            header.Add(m_fullClipToggle);

            var filterBtn = new ToolbarButton(FilterMenu) { text = "Filter" };
            header.Add(filterBtn);

            rootVisualElement.Add(header);

            var split = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    flexGrow = 1f,
                    justifyContent = Justify.SpaceBetween
                }
            };
            rootVisualElement.Add(split);

            // -- Work grid on a pannable canvas (absolute backdrop) ---------------
            var canvas = new PannableCanvas();
            canvas.style.position = Position.Absolute;
            canvas.style.left = 0f; canvas.style.top = 0f;
            canvas.style.right = 0f; canvas.style.bottom = 0f;
            split.Add(canvas);

            m_authoringTree = new AuthoringTreePanel(m_selection, m_filter);
            m_authoringTree.SelectionPicked += OnAuthoringPicked;

            m_bakedTree = new BakedTreePanel(m_selection, m_filter);
            m_bakedTree.SelectionPicked += OnBakedPicked;

            m_censusPreview = new CensusPreviewPanel();
            m_censusTimeline = new CensusTimelinePanel();
            m_liveView = new LiveViewPanel();
            if (m_data != null)
                m_liveView.SetRigPrefab(m_data.Prefab);

            m_censusPreview.TimeChanged += _ => m_censusTimeline.MarkDirtyRepaint();
            m_censusTimeline.TimeRequested += t => m_censusPreview.SetTime(t);

            // Census preview scrub → live start pose: while the live timeline
            // is not playing, the preview's seek position picks the live
            // start pose (nearest pose record of the selected baked clip).
            m_censusPreview.TimeChanged += t =>
            {
                if (m_bakedClip != null && m_liveRunner != null &&
                    !m_liveView.IsPlaying)
                    m_liveRunner.SeedStartPose(m_bakedClip, t);
            };

            // Grid layout: authoring | preview | baked | live on the top row;
            // timeline below spanning authoring+preview (widths sum exactly:
            // 320 + 16 + 400 = 736), baked tree extended down beside it, and
            // the live view (its own timeline inside) beside that.
            // The canvas pans, so fixed sizes are fine.
            float xTree = 12f;
            float xPreview = xTree + CensusDataPanel.k_Width + 16f;
            float xBaked = xPreview + CensusPreviewPanel.k_Width + 16f;
            float xLive = xBaked + CensusDataPanel.k_Width + 16f;
            float xSettings = xLive + LiveViewPanel.k_Width + 16f;
            float rowH = 12f + 520f + 12f; // top row bottom edge
            canvas.AddNode(WrapTree(m_authoringTree, CensusDataPanel.k_Width, 520f), xTree, 12f);
            canvas.AddNode(m_censusPreview, xPreview, 12f);
            canvas.AddNode(WrapTree(m_bakedTree, CensusDataPanel.k_Width,
                520f + 12f + CensusTimelinePanel.k_Height), xBaked, 12f);
            canvas.AddNode(m_censusTimeline, xTree, rowH);
            canvas.AddNode(m_liveView, xLive, 12f);
            m_liveSettings = new LiveSettingsPanel();
            m_liveSettings.SetModel(m_liveView.Model);
            canvas.AddNode(m_liveSettings, xSettings, 12f);

            // -- LEFT: legacy authoring editor in a collapsible pane --------------
            m_leftPane = new CollapsibleSidePane("Pre-Bake Editor", 420f,
                CollapsibleSidePane.Side.Left, fitContentHeight: true);
            m_leftPane.style.backgroundColor = new Color(0f, 0f, 0f, 0f);
            m_authoringInspector = new EmbeddedInspector();
            if (m_data != null)
                m_authoringInspector.SetTarget(m_data);
            m_leftPane.Content.Add(m_authoringInspector);
            if (EditorPrefs.HasKey(k_PrefsLeftPane))
                m_leftPane.SetExpanded(EditorPrefs.GetBool(k_PrefsLeftPane), animate: false);
            split.Add(m_leftPane);

            // -- RIGHT: legacy baked editor in a collapsible pane ------------------
            m_rightPane = new CollapsibleSidePane("Post-Bake Editor", 420f,
                fitContentHeight: true);
            m_rightPane.style.backgroundColor = new Color(0f, 0f, 0f, 0f);
            m_bakedInspector = new EmbeddedInspector();
            if (m_data != null && m_data.LastSavedAnimData != null)
                m_bakedInspector.SetTarget(m_data.LastSavedAnimData);
            m_rightPane.Content.Add(m_bakedInspector);
            if (EditorPrefs.HasKey(k_PrefsRightPane))
                m_rightPane.SetExpanded(EditorPrefs.GetBool(k_PrefsRightPane), animate: false);
            split.Add(m_rightPane);

            // Selection → legacy editors (stage decides which side).
            m_selection.Changed += OnSelectionChanged;
        }

        /// <summary>Host a TreeView in a titled panel shell sized for the
        /// grid. The tree IMGUI runs in an IMGUIContainer resized by layout
        /// callbacks (TreeView needs an explicit rect every frame).</summary>
        private static VisualElement WrapTree(TreeView<int> tree, float width, float height)
        {
            var panel = new CensusPanel(tree is AuthoringTreePanel ? "Authoring" : "Baked Data",
                width, height);
            IMGUIContainer host = null;
            host = new IMGUIContainer(() =>
                tree.OnGUI(new Rect(Vector2.zero, host.contentRect.size)))
            {
                style = { flexGrow = 1f }
            };
            panel.Content.Add(host);
            return panel;
        }

        // -- Filter UI + persistence ----------------------------------------

        private const string k_PrefsLeftPane = "MxMStudio:LeftPane:Expanded";
        private const string k_PrefsRightPane = "MxMStudio:RightPane:Expanded";
        private const string k_PrefsQuery = "MxMStudio:Filter:Query";

        private ToolbarSearchField m_search;
        private ToolbarToggle m_fullClipToggle;
        private CensusEntry m_boundEntry;

        private void ApplyFilterQuery(string query)
        {
            m_filter.Query = query ?? string.Empty;
            EditorPrefs.SetString(k_PrefsQuery, m_filter.Query);
            m_authoringTree?.Reload();
            m_bakedTree?.Reload();
        }

        private void FilterMenu()
        {
            var menu = new GenericMenu();
            AddToggle(menu, "Show Modules", m_filter.ShowModules,
                v => { m_filter.ShowModules = v; ReloadTrees(); });
            AddToggle(menu, "Show Categories", m_filter.ShowCategories,
                v => { m_filter.ShowCategories = v; ReloadTrees(); });
            AddToggle(menu, "Show Composites", m_filter.ShowComposites,
                v => { m_filter.ShowComposites = v; ReloadTrees(); });
            AddToggle(menu, "Show Blend Spaces", m_filter.ShowBlendSpaces,
                v => { m_filter.ShowBlendSpaces = v; ReloadTrees(); });
            AddToggle(menu, "Show Idle Sets", m_filter.ShowIdleSets,
                v => { m_filter.ShowIdleSets = v; ReloadTrees(); });
            menu.AddSeparator(string.Empty);
            AddToggle(menu, "Show Events", m_filter.ShowEvents,
                v => { m_filter.ShowEvents = v; ReloadTrees(); });
            AddToggle(menu, "Show Tracks", m_filter.ShowTracks,
                v => { m_filter.ShowTracks = v; ReloadTrees(); });
            AddToggle(menu, "Show Clips (baked)", m_filter.ShowClips,
                v => { m_filter.ShowClips = v; ReloadTrees(); });
            menu.ShowAsContext();
        }

        private static void AddToggle(GenericMenu menu, string label, bool value,
            System.Action<bool> set)
        {
            menu.AddItem(new GUIContent(label), value, () => set(!value));
        }

        private void ReloadTrees()
        {
            m_authoringTree?.Reload();
            m_bakedTree?.Reload();
        }

        private void OnAuthoringPicked(StudioSelection sel)
        {
            // Lowest authoring OBJECT: units are assets, modules are assets;
            // category-only picks fall back to the dataset.
            var target = (Object)sel.Unit ?? (Object)sel.Module ?? (Object)sel.Dataset;
            if (target != null)
                m_authoringInspector.SetTarget(target);

            // Preview/timeline: unit = time base. A marker pick re-frames
            // (seeks the event's fire point) without changing the base.
            RebindCurrentUnit();
            if (sel.Marker != null && m_boundEntry != null)
                m_censusPreview.SetTime(sel.Marker.EventTime); // seek to the fire point
        }

        /// <summary>Bind preview + timeline to the currently selected unit
        /// (composite / blend space / idle set). CollectForUnit builds the
        /// playable entry; the timeline derives marker/track lanes from the
        /// entry's OwnerComposite.</summary>
        private void RebindCurrentUnit()
        {
            var unit = m_selection.Unit;
            var entry = unit != null ? CensusModel.CollectForUnit(unit) : null;
            m_boundEntry = entry;

            m_censusPreview.SetEntry(entry);
            m_censusPreview.SetFullClip(m_selection.FullClip);
            m_censusTimeline.Bind(entry,
                () => m_censusPreview.CurrentTime,
                () => (m_censusPreview.WindowStart, m_censusPreview.WindowEnd),
                m_selection.FullClip);
        }

        private void OnBakedPicked(StudioSelection sel)
        {
            if (sel.Baked == null)
                return;

            m_bakedInspector.SetTarget(sel.Baked);

            // Live session binds to the baked dataset (the search source).
            // AttachRunner disposes + replaces the old runner (Dispose is
            // idempotent — no double-release of shared native data here).
            m_liveRunner = new LiveSearchRunner(sel.Baked, m_liveView.Model);
            m_liveView.AttachRunner(m_liveRunner);

            // Baked clip → old preview: bind a single-segment full-clip entry
            // so the existing event timeline can scrub it as usual.
            if (sel.ClipIndex >= 0 && sel.ClipIndex < sel.Baked.Clips.Length)
            {
                var clip = sel.Baked.Clips[sel.ClipIndex];
                if (clip != null)
                {
                    m_bakedClip = clip;
                    var entry = new CensusEntry
                    {
                        Clip = clip,
                        OwnerName = clip.name,
                        Category = "Baked Clips"
                    };
                    float offset = 0f;
                    AddSegment(entry, clip, ref offset);
                    m_censusPreview.SetEntry(entry);
                    m_censusPreview.SetFullClip(true);
                    float clipLen = clip.length;
                    m_censusTimeline.Bind(entry,
                        () => m_censusPreview.CurrentTime,
                        () => (0f, clipLen),
                        fullClip: true);
                }
            }
        }

        private AnimationClip m_bakedClip;

        private static void AddSegment(CensusEntry entry, AnimationClip clip,
            ref float offset)
        {
            if (clip == null)
                return;
            entry.Segments.Add(new CensusEntry.Segment
            {
                Clip = clip,
                Offset = offset,
                Length = clip.length
            });
            offset += clip.length;
        }

        private void OnSelectionChanged(StudioSelection sel)
        {
            // Reserved: cross-panel highlight sync (P2/P3).
        }

        private void RefreshCounts()
        {
            if (m_counts == null)
                return;

            if (m_data == null)
            {
                m_counts.text = "no dataset";
                return;
            }

            int modules = 0, categories = 0, composites = 0, blends = 0,
                idles = 0, markers = 0;

            if (m_data.CompositeCategories != null)
                categories += m_data.CompositeCategories.Count;
            if (m_data.BlendSpaces != null)
                blends += m_data.BlendSpaces.Count;
            if (m_data.AnimationIdleSets != null)
                idles += m_data.AnimationIdleSets.Count;
            if (m_data.AnimationModules != null)
                modules = m_data.AnimationModules.Count;

            if (m_data.CompositeCategories != null)
                for (int i = 0; i < m_data.CompositeCategories.Count; ++i)
                {
                    var cat = m_data.CompositeCategories[i];
                    if (cat?.Composites == null) continue;
                    composites += cat.Composites.Count;
                    for (int j = 0; j < cat.Composites.Count; ++j)
                        if (cat.Composites[j]?.EventMarkers != null)
                            markers += cat.Composites[j].EventMarkers.Count;
                }

            if (m_data.BlendSpaces != null)
                for (int i = 0; i < m_data.BlendSpaces.Count; ++i)
                    if (m_data.BlendSpaces[i]?.GeneratedEvents != null)
                        markers += m_data.BlendSpaces[i].GeneratedEvents.Count;

            if (m_data.AnimationModules != null)
                for (int i = 0; i < m_data.AnimationModules.Count; ++i)
                {
                    var mod = m_data.AnimationModules[i];
                    if (mod == null) continue;
                    if (mod.CompositeCategories != null)
                        categories += mod.CompositeCategories.Count;
                    if (mod.BlendSpaces != null)
                        blends += mod.BlendSpaces.Count;
                    if (mod.AnimationIdleSets != null)
                        idles += mod.AnimationIdleSets.Count;
                    if (mod.CompositeCategories != null)
                        for (int j = 0; j < mod.CompositeCategories.Count; ++j)
                        {
                            var cat = mod.CompositeCategories[j];
                            if (cat?.Composites == null) continue;
                            composites += cat.Composites.Count;
                            for (int k = 0; k < cat.Composites.Count; ++k)
                                if (cat.Composites[k]?.EventMarkers != null)
                                    markers += cat.Composites[k].EventMarkers.Count;
                        }
                    if (mod.BlendSpaces != null)
                        for (int j = 0; j < mod.BlendSpaces.Count; ++j)
                            if (mod.BlendSpaces[j]?.GeneratedEvents != null)
                                markers += mod.BlendSpaces[j].GeneratedEvents.Count;
                }

            var sb = new StringBuilder(96);
            sb.Append("Modules ").Append(modules);
            sb.Append(" · Categories ").Append(categories);
            sb.Append(" · Composites ").Append(composites);
            sb.Append(" · Blends ").Append(blends);
            sb.Append(" · Idles ").Append(idles);
            sb.Append(" · Markers ").Append(markers);

            var baked = m_data != null ? m_data.LastSavedAnimData : null;
            sb.Append(" · Poses ").Append(baked != null && baked.Poses != null
                ? baked.Poses.Length.ToString("N0") : "—");

            m_counts.text = sb.ToString();
        }
    }
}
#endif
