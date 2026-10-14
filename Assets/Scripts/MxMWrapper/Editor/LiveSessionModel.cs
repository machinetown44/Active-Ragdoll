// ----------------------------------------------------------------------------
// -- MxM Wrapper — live search session data model:                          --
// -- trajectory points (with look-direction pins), trigger tag spans,      --
// -- runner settings. Pure data; no Unity UI. Invalidated-by-edit flags    --
// -- live here so the runner and the views share one invalidation source.  --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using UnityEngine;

namespace MxMWrapper
{
    /// <summary>One control point of the live trajectory. Position is in
    /// world XZ (top-down canvas). Look direction: auto = spline tangent;
    /// pinned = free angle, never re-derived when the spline updates.</summary>
    public class TrajectoryPoint
    {
        public Vector2 Position;      // world XZ
        public bool LookPinned;       // true = orientation is manual
        public float LookAngleRad;    // pinned yaw (radians, world)
    }

    /// <summary>A trigger span on the live timeline: toggles a tag for the
    /// search while the playhead is inside [Start, End).</summary>
    public class TriggerSpan
    {
        public float Start;
        public float End;
        public string TagName;
        public bool Required;         // true gates the search, false biases it
    }

    /// <summary>Runner settings for the live search session.</summary>
    [Serializable]
    public class LiveSessionSettings
    {
        /// <summary>Search decision cadence in frames (MxMAnimator-like).</summary>
        public int SearchIntervalFrames = 6;
        /// <summary>Blend-in duration (s) applied to a newly chosen pose.</summary>
        public float BlendInTime = 0.15f;
        /// <summary>Speed multiplier fed into trajectory velocity samples.</summary>
        public float Speed = 1f;
        /// <summary>Live timeline duration (s) — fixed-length for now.</summary>
        public float Duration = 8f;
    }

    /// <summary>Data + invalidation for one live search session. The runner
    /// consumes this; the live view + settings panel edit it.</summary>
    public class LiveSessionModel
    {
        public readonly List<TrajectoryPoint> Points = new List<TrajectoryPoint>();
        public readonly List<TriggerSpan> Triggers = new List<TriggerSpan>();
        public readonly LiveSessionSettings Settings = new LiveSessionSettings();

        /// <summary>Incremented on every edit to trajectory or triggers —
        /// the runner's decision-cache version stamp.</summary>
        public event Action Invalidated;

        private int m_version;

        /// <summary>Cache version stamp; runner compares against its cached
        /// build version to detect invalidation.</summary>
        public int Version { get { return m_version; } }

        public void MarkInvalidated()
        {
            m_version++;
            Invalidated?.Invoke();
        }

        /// <summary>Seed a default straight-line trajectory (3 points, 1 m
        /// apart along +Z) so the panel is never empty.</summary>
        public void SeedDefault()
        {
            Points.Clear();
            Triggers.Clear();
            for (int i = 0; i < 3; ++i)
                Points.Add(new TrajectoryPoint { Position = new Vector2(0f, i * 1f) });
        }
    }
}
#endif