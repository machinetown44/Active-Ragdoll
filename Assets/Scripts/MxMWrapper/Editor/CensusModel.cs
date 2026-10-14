// ----------------------------------------------------------------------------
// -- MxM Wrapper — event census model: flat enumeration of every event      --
// -- marker reachable from a MxMPreProcessData, for filtering + preview.    --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEngine;
using MxM;
using MxMEditor;

namespace MxMWrapper
{
    /// <summary>One row of the census: an event marker and the asset/clip
    /// that owns it. Phase layout per MxM's runtime event state machine:
    /// [EventTime - Windup .. EventTime] windup, cumulative Actions from
    /// EventTime, then FollowThrough, then Recovery.</summary>
    public class CensusEntry
    {
        /// <summary>One clip slice of the playable timeline: composites play
        /// Before + Primary + After in sequence, and markers are timed against
        /// that merged timeline — not the primary clip alone.</summary>
        public struct Segment
        {
            public AnimationClip Clip;
            public float Offset;
            public float Length;
        }

        public EventMarker Marker;
        public string OwnerName;   // composite / blend space asset name
        public string Category;    // grouping label for filters
        public AnimationClip Clip; // primary/preview clip (compat)

        /// <summary>Composite that owns the marker (when this entry is one
        /// marker of a composite). The timeline reads sibling markers and
        /// tag tracks from it to build the multi-lane view.</summary>
        public MxMAnimationClipComposite OwnerComposite;

        /// <summary>Playback plan: sequential segments covering the full
        /// composite timeline. Single segment for plain clips / blend spaces.</summary>
        public readonly List<Segment> Segments = new List<Segment>();

        public float CompositeLength
        {
            get
            {
                float sum = 0f;
                for (int i = 0; i < Segments.Count; ++i)
                    sum += Segments[i].Length;
                return sum;
            }
        }

        public string EventName
        {
            get { return Marker != null ? Marker.EventName : string.Empty; }
        }

        public float StartTime
        {
            get { return Marker.EventTime - Marker.Windup; }
        }

        public float ActionSum
        {
            get
            {
                float sum = 0f;
                var actions = Marker.Actions;
                if (actions != null)
                {
                    for (int i = 0; i < actions.Count; ++i)
                        sum += actions[i];
                }
                return sum;
            }
        }

        public float TotalLength
        {
            get
            {
                return Marker.Windup + ActionSum + Marker.FollowThrough + Marker.Recovery;
            }
        }

        public float EndTime
        {
            get { return StartTime + TotalLength; }
        }
    }

    /// <summary>Collects census entries from the serialized preprocess data.
    /// Marker carriers in MxM: composites (EventMarkers) and blend spaces
    /// (GeneratedEvents). Idle sets and blend clips carry none — skipped.</summary>
    public static class CensusModel
    {
        // Accumulator used by the private Add* helpers; copied into the
        // caller's list at the end of Collect.
        private static readonly List<CensusEntry> s_entries = new List<CensusEntry>();

        public static void Collect(MxMPreProcessData data, List<CensusEntry> entries)
        {
            entries.Clear();
            if (data == null)
                return;

            s_entries.Clear();

            if (data.CompositeCategories != null)
            {
                for (int i = 0; i < data.CompositeCategories.Count; ++i)
                    AddCategory(data.CompositeCategories[i]);
            }

            if (data.BlendSpaces != null)
            {
                for (int i = 0; i < data.BlendSpaces.Count; ++i)
                    AddBlendSpace(data.BlendSpaces[i], "Blend Spaces");
            }

            if (data.AnimationModules != null)
            {
                for (int i = 0; i < data.AnimationModules.Count; ++i)
                {
                    var module = data.AnimationModules[i];
                    if (module == null)
                        continue;

                    if (module.CompositeCategories != null)
                    {
                        for (int j = 0; j < module.CompositeCategories.Count; ++j)
                            AddCategory(module.CompositeCategories[j]);
                    }

                    if (module.BlendSpaces != null)
                    {
                        for (int j = 0; j < module.BlendSpaces.Count; ++j)
                            AddBlendSpace(module.BlendSpaces[j], module.name + " / Blends");
                    }
                }
            }

            entries.AddRange(s_entries);
        }

        private static void AddCategory(CompositeCategory category)
        {
            if (category?.Composites == null)
                return;

            for (int i = 0; i < category.Composites.Count; ++i)
                AddMarkers(category.Composites[i], category.CatagoryName);
        }

        private static void AddBlendSpace(MxMBlendSpace blendSpace, string category)
        {
            if (blendSpace?.GeneratedEvents == null)
                return;

            for (int i = 0; i < blendSpace.GeneratedEvents.Count; ++i)
            {
                var entry = new CensusEntry
                {
                    Marker = blendSpace.GeneratedEvents[i],
                    OwnerName = blendSpace.name,
                    Category = category,
                    Clip = blendSpace.TargetClip
                };
                float offset = 0f;
                AddSeg(entry, blendSpace.TargetClip, ref offset);
                s_entries.Add(entry);
            }
        }

        private static void AddMarkers(MxMAnimationClipComposite composite, string category)
        {
            if (composite?.EventMarkers == null)
                return;

            for (int i = 0; i < composite.EventMarkers.Count; ++i)
            {
                var entry = new CensusEntry
                {
                    Marker = composite.EventMarkers[i],
                    OwnerName = composite.name,
                    Category = category,
                    Clip = composite.TargetClip
                };
                AddSegments(entry, composite);
                s_entries.Add(entry);
            }
        }

        /// <summary>Build the composite playback plan: Before + Primary + After
        /// clips in sequence. Falls back to the primary clip alone.</summary>
        private static void AddSegments(CensusEntry entry, MxMAnimationClipComposite composite)
        {
            entry.OwnerComposite = composite;

            float offset = 0f;
            var before = composite.BeforeClips;
            if (before != null)
            {
                for (int i = 0; i < before.Count; ++i)
                    AddSeg(entry, before[i], ref offset);
            }

            var primary = composite.TargetClip != null
                ? composite.TargetClip
                : composite.GeneratedClip;
            AddSeg(entry, primary, ref offset);

            var after = composite.AfterClips;
            if (after != null)
            {
                for (int i = 0; i < after.Count; ++i)
                    AddSeg(entry, after[i], ref offset);
            }
        }

        private static void AddSeg(CensusEntry entry, AnimationClip clip, ref float offset)
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

        /// <summary>Distinct category labels present in the dataset (for the
        /// category filter popup). Includes module-level categories.</summary>
        public static void CollectCategories(MxMPreProcessData data, List<string> categories)
        {
            categories.Clear();
            if (data == null)
                return;

            var seen = new HashSet<string>();
            AddCategoryName(seen, categories, "Blend Spaces");

            if (data.CompositeCategories != null)
            {
                for (int i = 0; i < data.CompositeCategories.Count; ++i)
                    AddCategoryName(seen, categories, data.CompositeCategories[i].CatagoryName);
            }

            if (data.AnimationModules != null)
            {
                for (int i = 0; i < data.AnimationModules.Count; ++i)
                {
                    var module = data.AnimationModules[i];
                    if (module?.CompositeCategories == null)
                        continue;
                    for (int j = 0; j < module.CompositeCategories.Count; ++j)
                        AddCategoryName(seen, categories, module.CompositeCategories[j].CatagoryName);
                }
            }
        }

        private static void AddCategoryName(HashSet<string> seen, List<string> into, string name)
        {
            if (string.IsNullOrEmpty(name) || !seen.Add(name))
                return;
            into.Add(name);
        }

        /// <summary>Build a playable entry for a single unit (composite /
        /// blend space), for tree-driven preview scoping. Idle sets return a
        /// primary-clip entry (no events). Null when the unit has no clips.</summary>
        public static CensusEntry CollectForUnit(Object unit)
        {
            if (unit is MxMAnimationClipComposite composite)
            {
                var entry = new CensusEntry
                {
                    Marker = composite.EventMarkers != null && composite.EventMarkers.Count > 0
                        ? composite.EventMarkers[0] : null,
                    OwnerName = composite.name,
                    Category = string.Empty,
                    Clip = composite.TargetClip
                };
                AddSegments(entry, composite);
                if (entry.Segments.Count == 0)
                    return null;
                return entry;
            }

            if (unit is MxMBlendSpace blendSpace)
            {
                var entry = new CensusEntry
                {
                    Marker = blendSpace.GeneratedEvents != null && blendSpace.GeneratedEvents.Count > 0
                        ? blendSpace.GeneratedEvents[0] : null,
                    OwnerName = blendSpace.name,
                    Category = string.Empty,
                    Clip = blendSpace.TargetClip
                };
                float offset = 0f;
                AddSeg(entry, blendSpace.TargetClip, ref offset);
                if (entry.Segments.Count == 0)
                    return null;
                return entry;
            }

            if (unit is MxMAnimationIdleSet idleSet)
            {
                var entry = new CensusEntry
                {
                    Marker = null,
                    OwnerName = idleSet.name,
                    Category = string.Empty,
                    Clip = idleSet.PrimaryClip
                };
                float offset = 0f;
                AddSeg(entry, idleSet.PrimaryClip, ref offset);
                if (entry.Segments.Count == 0)
                    return null;
                return entry;
            }

            return null;
        }
    }

    /// <summary>Phase colors shared by the timeline bands and the preview
    /// trajectory, so both views read as one system.</summary>
    public static class CensusPhaseColors
    {
        public static readonly Color Windup = new Color(0.35f, 0.55f, 0.95f);
        public static readonly Color Action = new Color(0.95f, 0.6f, 0.2f);
        public static readonly Color FollowThrough = new Color(0.35f, 0.75f, 0.4f);
        public static readonly Color Recovery = new Color(0.5f, 0.5f, 0.5f);

        /// <summary>Phase color at composite time 't' for a marker. Null
        /// marker (idle sets, blend spaces) = neutral gray.</summary>
        public static Color At(EventMarker marker, float t)
        {
            if (marker == null)
                return new Color(0.65f, 0.65f, 0.65f, 0.8f);
            float cursor = marker.EventTime;
            if (t < cursor)
                return Windup;

            if (marker.Actions != null)
            {
                for (int i = 0; i < marker.Actions.Count; ++i)
                {
                    cursor += marker.Actions[i];
                    if (t < cursor)
                        return Action;
                }
            }

            // Follow-through then recovery tail (shared with the timeline
            // band math).
            return t < cursor + marker.FollowThrough ? FollowThrough : Recovery;
        }

        /// <summary>Deterministic per-tag-id hue (golden-ratio spread) —
        /// shared by the timeline track lanes and the preview tag bands.</summary>
        public static Color TagColor(float tagId)
        {
            float hue = Mathf.Repeat(tagId * 0.618034f, 1f);
            var c = Color.HSVToRGB(hue, 0.45f, 1f);
            c.a = 1f;
            return c;
        }
    }
}
#endif