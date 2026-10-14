// ----------------------------------------------------------------------------
// -- MxM Studio — baked tree panel: TreeView over the baked MxMAnimData.    --
// -- Hierarchy: Clips (AnimationClips, the real baked units) → typed pose   --
// -- ranges (CompositeData / BlendSpaceData / IdleSetData covering them) →  --
// -- events. Pose rows are P3; ranges carry pose counts for now.            --
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
    public enum BakedNodeKind
    {
        AnimData,
        Clip,
        Composite,
        BlendSpace,
        IdleSet,
        Event,
        PoseRange
    }

    public sealed class BakedNode
    {
        public BakedNodeKind Kind;
        public int Id;                 // TreeView id
        public int Parent;             // -1 = root
        public string Name;

        public AnimationClip Clip;     // for Clip rows
        public int ClipIndex = -1;     // index into AnimData.Clips
        public int RangeIndex = -1;    // index into the owning typed array
        public int StartPoseId;
        public int EndPoseId;          // inclusive
        public int PoseCount;

        public bool HasChildren
        {
            get { return false; }      // set at build time below
        }
    }

    /// <summary>Right panel: post-bake hierarchy over MxMAnimData. Emits baked
    /// selections (pose ranges in P1; individual poses in P3).</summary>
    public sealed class BakedTreePanel : TreeView<int>
    {
        private readonly List<BakedNode> m_nodes = new List<BakedNode>();
        private readonly StudioSelection m_selection;
        private readonly StudioFilter m_filter;
        private MxMPreProcessData m_dataset;

        public event System.Action<StudioSelection> SelectionPicked;

        public BakedTreePanel(StudioSelection selection, StudioFilter filter)
            : base(new TreeViewState<int>())
        {
            m_selection = selection;
            m_filter = filter;
            showBorder = true;
            Reload();
        }

        public void SetData(MxMPreProcessData dataset, MxMAnimData baked)
        {
            m_dataset = dataset;
            Rebuild(baked);
            Reload();
            if (baked != null)
            {
                if (EditorPrefs.HasKey(k_PrefsExpanded))
                    RestoreExpandedState();
                else
                    ExpandAll();
            }
        }

        // -- Expansion persistence (P1): ids are rebuild-deterministic for a
        // -- given baked asset, so the state round-trips through EditorPrefs.
        private const string k_PrefsExpanded = "MxMStudio:BakedTree:Expanded";

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

        private void Rebuild(MxMAnimData baked)
        {
            m_nodes.Clear();
            if (baked == null)
                return;

            int root = AddNode(new BakedNode
            {
                Kind = BakedNodeKind.AnimData,
                Parent = -1,
                Name = baked.name,
                PoseCount = baked.Poses != null ? baked.Poses.Length : 0
            });

            if (baked.Clips == null)
                return;

            // Each AnimationClip is the spine: its pose interval comes from
            // whichever typed range references it as ClipIdA/ClipIdB/member.
            for (int c = 0; c < baked.Clips.Length; ++c)
            {
                var clip = baked.Clips[c];
                if (clip == null)
                    continue;

                int clipNode = AddNode(new BakedNode
                {
                    Kind = BakedNodeKind.Clip,
                    Parent = root,
                    Name = clip.name,
                    Clip = clip,
                    ClipIndex = c
                });

                // Composites covering this clip (ClipIdA or ClipIdB).
                if (baked.Composites != null)
                {
                    for (int i = 0; i < baked.Composites.Length; ++i)
                    {
                        var comp = baked.Composites[i];
                        if (comp.ClipIdA != c && comp.ClipIdB != c)
                            continue;
                        AddNode(new BakedNode
                        {
                            Kind = BakedNodeKind.Composite,
                            Parent = clipNode,
                            Name = $"Composite [{i}]",
                            ClipIndex = c,
                            RangeIndex = i,
                            StartPoseId = comp.StartPoseId,
                            EndPoseId = comp.EndPoseId,
                            PoseCount = Mathf.Max(0, comp.EndPoseId - comp.StartPoseId + 1)
                        });
                    }
                }

                // Blend spaces containing this clip.
                if (baked.BlendSpaces != null)
                {
                    for (int i = 0; i < baked.BlendSpaces.Length; ++i)
                    {
                        var bs = baked.BlendSpaces[i];
                        if (bs.ClipIds == null || System.Array.IndexOf(bs.ClipIds, c) < 0)
                            continue;
                        AddNode(new BakedNode
                        {
                            Kind = BakedNodeKind.BlendSpace,
                            Parent = clipNode,
                            Name = $"BlendSpace [{i}]",
                            ClipIndex = c,
                            RangeIndex = i,
                            StartPoseId = bs.StartPoseId,
                            EndPoseId = bs.EndPoseId,
                            PoseCount = Mathf.Max(0, bs.EndPoseId - bs.StartPoseId + 1)
                        });
                    }
                }

                // Idle sets (primary or transitions or secondaries).
                if (baked.IdleSets != null)
                {
                    for (int i = 0; i < baked.IdleSets.Length; ++i)
                    {
                        var idle = baked.IdleSets[i];
                        bool primary = idle.PrimaryClipId == c;
                        bool transition = idle.TransitionClipIds != null
                            && System.Array.IndexOf(idle.TransitionClipIds, c) >= 0;
                        bool secondary = idle.SecondaryClipIds != null
                            && System.Array.IndexOf(idle.SecondaryClipIds, c) >= 0;
                        if (!primary && !transition && !secondary)
                            continue;
                        AddNode(new BakedNode
                        {
                            Kind = BakedNodeKind.IdleSet,
                            Parent = clipNode,
                            Name = $"IdleSet [{i}]{(primary ? " (primary)" : "")}",
                            ClipIndex = c,
                            RangeIndex = i,
                            StartPoseId = idle.PrimaryPoseIds != null && idle.PrimaryPoseIds.Length > 0
                                ? idle.PrimaryPoseIds[0] : -1,
                            EndPoseId = -1,
                            PoseCount = (idle.PrimaryPoseIds != null ? idle.PrimaryPoseIds.Length : 0)
                                + (idle.TransitionPoseIds != null ? idle.TransitionPoseIds.Length : 0)
                                + (idle.SecondaryPoseIds != null ? idle.SecondaryPoseIds.Length : 0)
                        });
                    }
                }
            }
        }

        private int AddNode(BakedNode node)
        {
            node.Id = m_nodes.Count;
            m_nodes.Add(node);
            return node.Id;
        }

        protected override TreeViewItem<int> BuildRoot()
        {
            var root = new TreeViewItem<int> { id = -1, depth = -1, displayName = "Root", children = new List<TreeViewItem<int>>() };
            var visible = new List<TreeViewItem<int>>(m_nodes.Count);

            // Filter: the AnimData row always shows; a clip shows when the
            // type toggle and query pass; children ride on their clip (a
            // hidden clip hides its subtree).
            var clipVisible = new bool[m_nodes.Count];
            for (int i = 0; i < m_nodes.Count; ++i)
            {
                var n = m_nodes[i];
                bool vis;
                if (n.Kind == BakedNodeKind.AnimData)
                    vis = true;
                else if (n.Kind == BakedNodeKind.Clip)
                    vis = m_filter.ShowClips && m_filter.MatchesName(n.Name);
                else
                    vis = n.Parent >= 0 && clipVisible[n.Parent];
                clipVisible[i] = vis;
                if (!vis)
                    continue;
                visible.Add(new TreeViewItem<int>
                {
                    id = n.Id,
                    depth = n.Parent >= 0 ? 1 : 0,
                    displayName = RowLabel(n)
                });
            }

            var byId = new Dictionary<int, TreeViewItem<int>>(m_nodes.Count);
            for (int i = 0; i < visible.Count; ++i)
                byId[visible[i].id] = visible[i];
            for (int i = 0; i < visible.Count; ++i)
            {
                var n = m_nodes[visible[i].id];
                var item = visible[i];
                item.parent = n.Parent >= 0 ? byId[n.Parent] : root;
                // TreeView derives foldouts/lazy expansion from children
                // lists, not parent refs — build them explicitly.
                var pc = item.parent.children;
                if (pc == null)
                    item.parent.children = pc = new List<TreeViewItem<int>>();
                pc.Add(item);
                item.depth = item.parent.depth + 1;
            }
            return root;
        }

        private static string RowLabel(BakedNode n)
        {
            switch (n.Kind)
            {
                case BakedNodeKind.AnimData:
                    return $"{n.Name}  [{n.PoseCount:N0} poses]";
                case BakedNodeKind.Clip:
                    return n.Name;
                default:
                    return $"{n.Name}  [{n.StartPoseId}..{n.EndPoseId}]";
            }
        }

        protected override void RowGUI(RowGUIArgs args)
        {
            var n = m_nodes[args.item.id];
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

        private static string IconFor(BakedNodeKind kind)
        {
            switch (kind)
            {
                case BakedNodeKind.AnimData: return "SceneAsset Icon";
                case BakedNodeKind.Clip: return "AnimationClip Icon";
                case BakedNodeKind.Composite: return "d_Animation.SequencerLink";
                case BakedNodeKind.BlendSpace: return "MeshRenderer Icon";
                case BakedNodeKind.IdleSet: return "CustomSorting";
                case BakedNodeKind.Event: return "Animation.EventMarker";
                default: return "CustomTool";
            }
        }

        protected override void SelectionChanged(IList<int> selectedIds)
        {
            if (selectedIds == null || selectedIds.Count == 0)
                return;

            var n = m_nodes[selectedIds[0]];
            var baked = m_selection.Baked;
            int poseId = n.Kind == BakedNodeKind.PoseRange ? n.StartPoseId : -1;
            int clipIndex = n.Kind == BakedNodeKind.Clip ? n.ClipIndex : -1;
            m_selection.SelectBaked(m_dataset, baked, poseId, clipIndex);

            var handler = SelectionPicked;
            if (handler != null)
                handler(m_selection);
        }
    }
}
#endif
