// ----------------------------------------------------------------------------
// -- MxM Studio — authoring tree panel: TreeView over StudioTreeModel.      --
// -- Selection raises StudioSelection (authoring stage); rows show typed    --
// -- icons and counts. Rebuilds only when the dataset changes.             --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Collections.Generic;
using MxM;
using MxMEditor;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace MxMWrapper
{
    /// <summary>Left panel: pre-bake hierarchy. Emits authoring selections.</summary>
    public sealed class AuthoringTreePanel : TreeView<int>
    {
        private readonly StudioTreeModel m_model = new StudioTreeModel();
        private readonly StudioSelection m_selection;
        private readonly StudioFilter m_filter;

        public event System.Action<StudioSelection> SelectionPicked;

        public AuthoringTreePanel(StudioSelection selection, StudioFilter filter,
            TreeViewState<int> state = null)
            : base(state ?? new TreeViewState<int>())
        {
            m_selection = selection;
            m_filter = filter;
            showBorder = true;
            Reload();
        }

        public void SetDataset(MxMPreProcessData data)
        {
            m_model.Rebuild(data);
            Reload();
            // TreeView starts with nothing expanded — the default view would
            // be a single collapsed dataset row. Restore the persisted
            // expansion when present; otherwise open the full hierarchy.
            if (data != null)
            {
                if (EditorPrefs.HasKey(k_PrefsExpanded))
                    RestoreExpandedState();
                else
                    ExpandAll();
            }
        }

        // -- Expansion persistence (P1): ids are model node ids, stable for
        // -- a given dataset; the state round-trips through EditorPrefs.
        private const string k_PrefsExpanded = "MxMStudio:AuthoringTree:Expanded";

        public void SaveExpandedState()
        {
            EditorPrefs.SetString(k_PrefsExpanded,
                string.Join(",", state.expandedIDs));
        }

        public void RestoreExpandedState()
        {
            var raw = EditorPrefs.GetString(k_PrefsExpanded, string.Empty);
            if (string.IsNullOrEmpty(raw))
                return;
            var parts = raw.Split(',');
            var ids = new List<int>(parts.Length);
            for (int i = 0; i < parts.Length; ++i)
                if (int.TryParse(parts[i], out int id))
                    ids.Add(id);
            state.expandedIDs = ids;
            Reload();
        }

        protected override TreeViewItem<int> BuildRoot()
        {
            var root = new TreeViewItem<int> { id = -1, depth = -1, displayName = "Root", children = new List<TreeViewItem<int>>() };
            var nodes = m_model.Nodes;

            // Visible set: node passes filter OR has a matching descendant.
            var visible = new List<TreeViewItem<int>>(nodes.Count);
            var visibleIds = new HashSet<int>();
            for (int i = 0; i < nodes.Count; ++i)
            {
                if (PassesFilter(nodes[i]) || HasMatchingDescendant(i))
                {
                    visible.Add(new TreeViewItem<int>
                    {
                        id = nodes[i].Id,
                        depth = DepthOf(nodes[i]),
                        displayName = RowLabel(nodes[i])
                    });
                    visibleIds.Add(nodes[i].Id);
                }
            }

            // Re-parent through hidden ancestors (collapsed filter gaps),
            // then build each parent's children list — TreeView derives
            // foldouts and lazy expansion from children, not parent refs.
            // visible is DFS-ordered, so appends keep sibling order.
            var byId = new Dictionary<int, TreeViewItem<int>>(visible.Count);
            for (int i = 0; i < visible.Count; ++i)
                byId[visible[i].id] = visible[i];
            for (int i = 0; i < visible.Count; ++i)
            {
                var n = nodes[visible[i].id];
                int p = n.Parent;
                while (p >= 0 && !visibleIds.Contains(p))
                    p = nodes[p].Parent;
                var item = visible[i];
                item.parent = p >= 0 ? byId[p] : root;
                var pc = item.parent.children;
                if (pc == null)
                    item.parent.children = pc = new List<TreeViewItem<int>>();
                pc.Add(item);
                item.depth = item.parent.depth + 1;
            }
            return root;
        }

        private int DepthOf(StudioNode n)
        {
            int depth = 0;
            int p = n.Parent;
            while (p >= 0 && p < m_model.Nodes.Count)
            {
                ++depth;
                p = m_model.Nodes[p].Parent;
            }
            return depth;
        }

        private bool PassesFilter(StudioNode n)
        {
            if (!m_filter.MatchesName(n.Name))
                return false;
            switch (n.Kind)
            {
                case StudioNodeKind.Module: return m_filter.ShowModules;
                case StudioNodeKind.Category: return m_filter.ShowCategories;
                case StudioNodeKind.Composite: return m_filter.ShowComposites;
                case StudioNodeKind.BlendSpace: return m_filter.ShowBlendSpaces;
                case StudioNodeKind.IdleSet: return m_filter.ShowIdleSets;
                case StudioNodeKind.Event: return m_filter.ShowEvents;
                case StudioNodeKind.Track: return m_filter.ShowTracks;
                default: return true;
            }
        }

        private bool HasMatchingDescendant(int idx)
        {
            // Small dataset (few thousand nodes max); plain DFS is fine.
            for (int i = 0; i < m_model.Nodes.Count; ++i)
            {
                if (i == idx)
                    continue;
                int p = m_model.Nodes[i].Parent;
                while (p >= 0)
                {
                    if (p == idx)
                        return PassesFilter(m_model.Nodes[i]);
                    p = m_model.Nodes[p].Parent;
                }
            }
            return false;
        }

        private static string RowLabel(StudioNode n)
        {
            switch (n.Kind)
            {
                case StudioNodeKind.Category:
                    return $"{n.Name} ({n.ChildCount})";
                case StudioNodeKind.Composite:
                case StudioNodeKind.BlendSpace:
                    return $"{n.Name}  [{n.LengthSeconds:0.0#}s]";
                default:
                    return n.Name;
            }
        }

        protected override void RowGUI(RowGUIArgs args)
        {
            var n = m_model.Nodes[args.item.id];
            var icon = CachedIcon(IconFor(n.Kind));

            var labelRect = args.rowRect;
            labelRect.x += GetContentIndent(args.item);
            labelRect.width -= labelRect.x - args.rowRect.x;

            var iconRect = labelRect;
            iconRect.width = 16f;
            if (icon != null)
                GUI.DrawTexture(iconRect, icon, ScaleMode.ScaleToFit);

            labelRect.x += 20f;
            labelRect.width -= 20f;
            GUI.Label(labelRect, args.label, EditorStyles.label);
        }

        private static readonly Dictionary<string, Texture> s_iconCache =
            new Dictionary<string, Texture>();

        private static Texture CachedIcon(string name)
        {
            if (string.IsNullOrEmpty(name))
                return null;
            Texture tex;
            if (!s_iconCache.TryGetValue(name, out tex))
            {
                tex = EditorGUIUtility.IconContent(name).image;
                s_iconCache[name] = tex;
            }
            return tex;
        }

        private static string IconFor(StudioNodeKind kind)
        {
            switch (kind)
            {
                case StudioNodeKind.Dataset: return "SceneAsset Icon";
                case StudioNodeKind.Module: return "GameManager Icon";
                case StudioNodeKind.Category: return "Folder Icon";
                case StudioNodeKind.Composite: return "AnimationClip Icon";
                case StudioNodeKind.BlendSpace: return "MeshRenderer Icon";
                case StudioNodeKind.IdleSet: return "CustomSorting";
                case StudioNodeKind.Event: return "Animation.EventMarker";
                case StudioNodeKind.Track: return "CustomTool";
                default: return "";
            }
        }

        protected override void SelectionChanged(IList<int> selectedIds)
        {
            if (selectedIds == null || selectedIds.Count == 0 || m_model.Nodes == null)
                return;

            var n = m_model.Nodes[selectedIds[0]];
            m_selection.SelectAuthoring(m_model.Dataset, n.Module, n.Category,
                n.Unit, n.TrackIndex, n.Marker);

            var handler = SelectionPicked;
            if (handler != null)
                handler(m_selection);
        }
    }
}
#endif
