// ----------------------------------------------------------------------------
// -- MxM Studio — selection spine: one evented scope object shared by all   --
// -- panels. Selecting on any panel updates the scope; every view re-frames --
// -- what it can show. Pure data + events, no Unity UI here.               --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System;
using MxM;
using MxMEditor;
using UnityEditor;
using UnityEngine;

namespace MxMWrapper
{
    /// <summary>Which bake stage the current selection belongs to. Decides
    /// which inspector drawer opens (left = pre-bake, right = post-bake).</summary>
    public enum StudioStage
    {
        None,
        Authoring,   // pre-bake: dataset/module/category/unit/tracks/events
        Baked        // post-bake: anim data rows, pose ranges, poses
    }

    /// <summary>The studio-wide selection scope. Fields stack downward:
    /// a composite selection implies its module/category context. Baked
    /// side is parallel — selecting a pose keeps the authoring context so
    /// the preview can cross-reference.</summary>
    public sealed class StudioSelection
    {
        // -- Authoring (pre-bake) -------------------------------------------------

        /// <summary>Root dataset. Never null once a dataset is loaded.</summary>
        public MxMPreProcessData Dataset;

        /// <summary>Module the selection lives under (null = root-level content).</summary>
        public AnimationModule Module;

        public CompositeCategory Category;

        /// <summary>The selected unit: MxMAnimationClipComposite,
        /// MxMBlendSpace or MxMAnimationIdleSet. Null at coarser levels.</summary>
        public UnityEngine.Object Unit;

        /// <summary>Index into the unit's track lists when a track row is
        /// selected (tag/favour/footstep/warp/curve) — -1 otherwise.</summary>
        public int TrackIndex = -1;

        /// <summary>Selected event marker (owned by Unit), if any.</summary>
        public EventMarker Marker;

        // -- Baked (post-bake) ----------------------------------------------------

        /// <summary>The bake output: Dataset.LastSavedAnimData.</summary>
        public MxMAnimData Baked;

        /// <summary>Selected pose index into Baked.Poses (-1 = none).</summary>
        public int PoseId = -1;

        /// <summary>Selected baked clip index into Baked.Clips (-1 = none).</summary>
        public int ClipIndex = -1;

        // -- View state --------------------------------------------------------------

        /// <summary>Expand preview/timeline scope past the authoring slice to
        /// the full primary clip (the clip that actually feeds MM).</summary>
        public bool FullClip;

        /// <summary>Stage of the most recent selection change — drives which
        /// inspector drawer is open.</summary>
        public StudioStage Stage = StudioStage.None;

        public event Action<StudioSelection> Changed;

        /// <summary>Select nothing (keep dataset context).</summary>
        public void Clear()
        {
            Module = null;
            Category = null;
            Unit = null;
            TrackIndex = -1;
            Marker = null;
            Baked = null;
            PoseId = -1;
            Stage = StudioStage.None;
            Raise();
        }

        /// <summary>Authoring-side selection. Component-level: pass what you
        /// know; earlier context fields are preserved when still valid.</summary>
        public void SelectAuthoring(MxMPreProcessData dataset, AnimationModule module,
            CompositeCategory category = null, UnityEngine.Object unit = null,
            int trackIndex = -1, EventMarker marker = null)
        {
            Dataset = dataset;
            Stage = StudioStage.Authoring;
            Module = module;
            Category = category;
            Unit = unit;
            TrackIndex = trackIndex;
            Marker = marker;
            PoseId = -1;
            Baked = null;
            Raise();
        }

        /// <summary>Baked-side selection (pose range row or pose row).</summary>
        public void SelectBaked(MxMPreProcessData dataset, MxMAnimData baked,
            int poseId, int clipIndex = -1)
        {
            Dataset = dataset;
            Stage = StudioStage.Baked;
            Baked = baked;
            PoseId = poseId;
            ClipIndex = clipIndex;
            Raise();
        }

        /// <summary>Re-frame only (marker/track click): does not reset the
        /// time base in views that distinguish framing from scoping.</summary>
        public void Reframe(EventMarker marker, int trackIndex)
        {
            Marker = marker;
            TrackIndex = trackIndex;
            Stage = StudioStage.Authoring;
            Raise();
        }

        public void SetFullClip(bool full)
        {
            if (FullClip == full)
                return;
            FullClip = full;
            Raise();
        }

        /// <summary>Update the baked-data reference (after a bake) without
        /// treating it as a selection change; views refresh on their own.</summary>
        public void SetBakedData(MxMAnimData baked)
        {
            Baked = baked;
        }

        private void Raise()
        {
            var handler = Changed;
            if (handler != null)
                handler(this);
        }
    }

    /// <summary>Filter shared by both trees: text query + per-type toggles.
    /// TreeView rows filter to matching nodes (ancestors revealed).</summary>
    public sealed class StudioFilter
    {
        public string Query = string.Empty;

        public bool ShowModules = true;
        public bool ShowCategories = true;
        public bool ShowComposites = true;
        public bool ShowBlendSpaces = true;
        public bool ShowIdleSets = true;
        public bool ShowEvents = true;
        public bool ShowTracks = true;

        public bool ShowClips = true;
        public bool ShowPoseRanges = true;
        public bool ShowPoses = true;

        /// <summary>Case-insensitive contains; empty query matches all.</summary>
        public bool MatchesName(string name)
        {
            if (string.IsNullOrEmpty(Query))
                return true;
            return name != null &&
                name.IndexOf(Query, StringComparison.OrdinalIgnoreCase) >= 0;
        }
    }
}
#endif
