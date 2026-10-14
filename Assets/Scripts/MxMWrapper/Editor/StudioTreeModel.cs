// ----------------------------------------------------------------------------
// -- MxM Studio — authoring tree model: dataset → module → category → unit  --
// -- → tracks/events. Flat node list + id-indexed lookup; parents resolved  --
// -- by index. Built once per dataset/filter change; no per-frame work.     --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Collections.Generic;
using MxM;
using MxMEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace MxMWrapper
{
    public enum StudioNodeKind
    {
        Dataset,
        Module,
        Category,
        Composite,
        BlendSpace,
        IdleSet,
        Event,
        Track
    }

    /// <summary>One authoring-tree node. Wraps the source object plus its
    /// hierarchy context; TreeView ids are assigned sequentially at build.</summary>
    public sealed class StudioNode
    {
        public StudioNodeKind Kind;
        public int Id;                    // TreeView id
        public int Parent;                // parent node index, -1 = root
        public string Name;

        public AnimationModule Module;    // owning module (null = dataset root)
        public CompositeCategory Category;
        public UnityEngine.Object Unit;   // composite | blendspace | idleset
        public EventMarker Marker;
        public int TrackIndex = -1;

        // Display helpers
        public int ChildCount;
        public int EventCount;            // for category/composite rows
        public float LengthSeconds;       // for unit rows

        public bool HasChildren
        {
            get { return ChildCount > 0; }
        }
    }

    /// <summary>Builds and owns the authoring node list for a dataset.</summary>
    public sealed class StudioTreeModel
    {
        private readonly List<StudioNode> m_nodes = new List<StudioNode>();
        private MxMPreProcessData m_dataset;

        public IReadOnlyList<StudioNode> Nodes
        {
            get { return m_nodes; }
        }

        public MxMPreProcessData Dataset
        {
            get { return m_dataset; }
        }

        public void Rebuild(MxMPreProcessData data)
        {
            m_nodes.Clear();
            m_dataset = data;
            if (data == null)
                return;

            int root = AddNode(new StudioNode
            {
                Kind = StudioNodeKind.Dataset,
                Parent = -1,
                Name = data.name
            });

            // Root-level categories
            if (data.CompositeCategories != null)
            {
                for (int i = 0; i < data.CompositeCategories.Count; ++i)
                    AddCategory(data.CompositeCategories[i], null, root);
            }

            // Root-level blend spaces (own group node)
            if (data.BlendSpaces != null && data.BlendSpaces.Count > 0)
            {
                int group = AddNode(new StudioNode
                {
                    Kind = StudioNodeKind.Category,
                    Parent = root,
                    Name = "Blend Spaces"
                });
                for (int i = 0; i < data.BlendSpaces.Count; ++i)
                    AddBlendSpace(data.BlendSpaces[i], null, group);
            }

            // Root-level idle sets (own group node)
            if (data.AnimationIdleSets != null && data.AnimationIdleSets.Count > 0)
            {
                int group = AddNode(new StudioNode
                {
                    Kind = StudioNodeKind.Category,
                    Parent = root,
                    Name = "Idle Sets"
                });
                for (int i = 0; i < data.AnimationIdleSets.Count; ++i)
                    AddIdleSet(data.AnimationIdleSets[i], null, group);
            }

            // Modules: same shape, nested under their group node
            if (data.AnimationModules != null)
            {
                for (int i = 0; i < data.AnimationModules.Count; ++i)
                {
                    var module = data.AnimationModules[i];
                    if (module == null)
                        continue;

                    int modNode = AddNode(new StudioNode
                    {
                        Kind = StudioNodeKind.Module,
                        Parent = root,
                        Name = module.name,
                        Module = module
                    });

                    if (module.CompositeCategories != null)
                    {
                        for (int j = 0; j < module.CompositeCategories.Count; ++j)
                            AddCategory(module.CompositeCategories[j], module, modNode);
                    }

                    if (module.BlendSpaces != null && module.BlendSpaces.Count > 0)
                    {
                        int group = AddNode(new StudioNode
                        {
                            Kind = StudioNodeKind.Category,
                            Parent = modNode,
                            Name = "Blend Spaces",
                            Module = module
                        });
                        for (int k = 0; k < module.BlendSpaces.Count; ++k)
                            AddBlendSpace(module.BlendSpaces[k], module, group);
                    }

                    if (module.AnimationIdleSets != null && module.AnimationIdleSets.Count > 0)
                    {
                        int group = AddNode(new StudioNode
                        {
                            Kind = StudioNodeKind.Category,
                            Parent = modNode,
                            Name = "Idle Sets",
                            Module = module
                        });
                        for (int k = 0; k < module.AnimationIdleSets.Count; ++k)
                            AddIdleSet(module.AnimationIdleSets[k], module, group);
                    }
                }
            }

            ComputeChildCounts();
        }

        private void AddCategory(CompositeCategory category, AnimationModule module, int parent)
        {
            if (category == null)
                return;

            int idx = AddNode(new StudioNode
            {
                Kind = StudioNodeKind.Category,
                Parent = parent,
                Name = category.CatagoryName,
                Module = module,
                Category = category
            });

            if (category.Composites == null)
                return;

            for (int i = 0; i < category.Composites.Count; ++i)
                AddComposite(category.Composites[i], module, category, idx);
        }

        private void AddComposite(MxMAnimationClipComposite composite,
            AnimationModule module, CompositeCategory category, int parent)
        {
            if (composite == null)
                return;

            int idx = AddNode(new StudioNode
            {
                Kind = StudioNodeKind.Composite,
                Parent = parent,
                Name = string.IsNullOrEmpty(composite.CompositeName)
                    ? composite.name : composite.CompositeName,
                Module = module,
                Category = category,
                Unit = composite,
                LengthSeconds = CompositeLength(composite)
            });

            // Event children
            if (composite.EventMarkers != null)
            {
                for (int i = 0; i < composite.EventMarkers.Count; ++i)
                {
                    var marker = composite.EventMarkers[i];
                    AddNode(new StudioNode
                    {
                        Kind = StudioNodeKind.Event,
                        Parent = idx,
                        Name = marker != null && !string.IsNullOrEmpty(marker.EventName)
                            ? marker.EventName : "Event",
                        Module = module,
                        Category = category,
                        Unit = composite,
                        Marker = marker
                    });
                }
            }

            // Track children (tag tracks only in P1; warps/footsteps in P2)
            AddTracks(module, category, composite, idx,
                composite.TagTracks, "Tag");
            AddTracks(module, category, composite, idx,
                composite.FavourTagTracks, "Favour");
        }

        private void AddTracks(AnimationModule module, CompositeCategory category,
            MxMAnimationClipComposite composite, int parent, List<TagTrack> tracks, string label)
        {
            if (tracks == null)
                return;

            for (int i = 0; i < tracks.Count; ++i)
            {
                AddNode(new StudioNode
                {
                    Kind = StudioNodeKind.Track,
                    Parent = parent,
                    Name = $"{label} {i}",
                    Module = module,
                    Category = category,
                    Unit = composite,
                    TrackIndex = i
                });
            }
        }

        private void AddBlendSpace(MxMBlendSpace blendSpace, AnimationModule module, int parent)
        {
            if (blendSpace == null)
                return;

            int idx = AddNode(new StudioNode
            {
                Kind = StudioNodeKind.BlendSpace,
                Parent = parent,
                Name = blendSpace.BlendSpaceName ?? blendSpace.name,
                Module = module,
                Unit = blendSpace
            });

            if (blendSpace.GeneratedEvents != null)
            {
                for (int i = 0; i < blendSpace.GeneratedEvents.Count; ++i)
                {
                    var marker = blendSpace.GeneratedEvents[i];
                    AddNode(new StudioNode
                    {
                        Kind = StudioNodeKind.Event,
                        Parent = idx,
                        Name = marker != null && !string.IsNullOrEmpty(marker.EventName)
                            ? marker.EventName : "Event",
                        Module = module,
                        Unit = blendSpace,
                        Marker = marker
                    });
                }
            }
        }

        private void AddIdleSet(MxMAnimationIdleSet idleSet, AnimationModule module, int parent)
        {
            if (idleSet == null)
                return;

            AddNode(new StudioNode
            {
                Kind = StudioNodeKind.IdleSet,
                Parent = parent,
                Name = idleSet.name,
                Module = module,
                Unit = idleSet
            });
        }

        private int AddNode(StudioNode node)
        {
            node.Id = m_nodes.Count;
            m_nodes.Add(node);
            return node.Id;
        }

        private void ComputeChildCounts()
        {
            for (int i = 0; i < m_nodes.Count; ++i)
                m_nodes[i].ChildCount = 0;
            for (int i = 0; i < m_nodes.Count; ++i)
            {
                int p = m_nodes[i].Parent;
                if (p >= 0 && p < m_nodes.Count)
                    ++m_nodes[p].ChildCount;
            }
        }

        private static float CompositeLength(MxMAnimationClipComposite composite)
        {
            float len = 0f;
            if (composite.BeforeClips != null)
                for (int i = 0; i < composite.BeforeClips.Count; ++i)
                    if (composite.BeforeClips[i] != null) len += composite.BeforeClips[i].length;
            if (composite.TargetClip != null) len += composite.TargetClip.length;
            else if (composite.GeneratedClip != null) len += composite.GeneratedClip.length;
            if (composite.AfterClips != null)
                for (int i = 0; i < composite.AfterClips.Count; ++i)
                    if (composite.AfterClips[i] != null) len += composite.AfterClips[i].length;
            return len;
        }
    }
}
#endif
