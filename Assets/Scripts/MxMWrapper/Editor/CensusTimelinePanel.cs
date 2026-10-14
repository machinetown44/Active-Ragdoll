// ----------------------------------------------------------------------------
// -- MxM Wrapper — census timeline panel: unit-scoped multi-lane view.      --
// -- One lane per event marker (phase bands: windup / actions / follow-    --
// -- through / recovery), then one lane per tag/favour/footstep/warp       --
// -- track (colored spans). Ruler on top, draggable playhead synced with   --
// -- the preview, dimming outside the playable window (or, in FullClip     --
// -- mode, beyond the composite slice).                                    --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using UnityEditor;

namespace MxMWrapper
{
    public class CensusTimelinePanel : CensusPanel
    {
        public const float k_Width = 736f;
        public const float k_Height = 190f;

        private static readonly Color k_PlayheadColor = new Color(1f, 0.3f, 0.25f);
        private static readonly Color k_RulerColor = new Color(0.5f, 0.5f, 0.5f, 0.6f);
        private static readonly Color k_OutsideColor = new Color(0f, 0f, 0f, 0.45f);
        private static readonly Color k_FocusColor = new Color(1f, 1f, 1f, 0.55f);

        private CensusEntry m_entry;
        private System.Func<float> m_timeGetter;
        private System.Func<(float start, float end)> m_windowGetter;
        private bool m_fullClip;
        private bool m_dragging;
        private ScrollView m_scroll;
        private IMGUIContainer m_view;

        /// <summary>Tag/favour/footstep/warp overlay lanes, built at bind.
        /// Tags are (time, tagId) pairs; a tag spans until the next tag.</summary>
        private struct TrackLane
        {
            public string Name;
            public List<Vector2> Tags;
            public Color Base;
        }
        private readonly List<TrackLane> m_trackLanes = new List<TrackLane>();

        /// <summary>User scrubbed the playhead to a (clamped) time.</summary>
        public event System.Action<float> TimeRequested;

        public CensusTimelinePanel() : base("Event Timeline", k_Width, k_Height)
        {
            // Scrollable lane area: composites with many markers + tracks
            // exceed the fixed panel height, so the IMGUI view lives in a
            // vertical ScrollView sized to the lane stack (see Bind).
            m_scroll = new ScrollView { mode = ScrollViewMode.Vertical,
                style = { flexGrow = 1f } };
            m_view = new IMGUIContainer(DrawView)
            {
                style = { flexGrow = 1f }
            };
            m_scroll.Add(m_view);
            Content.Add(m_scroll);
        }

        /// <summary>Bind the timeline to a unit entry and a live-time source.
        /// The entry is the UNIT's time base (first marker or markerless);
        /// sibling marker lanes and tag tracks come from OwnerComposite.
        /// The window getter returns the preview's playable window.</summary>
        public void Bind(CensusEntry entry, System.Func<float> timeGetter,
            System.Func<(float, float)> windowGetter, bool fullClip = false)
        {
            m_entry = entry;
            m_timeGetter = timeGetter;
            m_windowGetter = windowGetter;
            m_fullClip = fullClip;

            m_trackLanes.Clear();
            var comp = entry?.OwnerComposite;
            if (comp != null)
            {
                AddTrackLanes(comp.TagTracks, "Tag", new Color(0.5f, 0.7f, 1f, 0.85f));
                AddTrackLanes(comp.FavourTagTracks, "Favour", new Color(1f, 0.7f, 0.3f, 0.85f));
                AddTrackLane(comp.LeftFootStepTrack, "Left Footstep", new Color(0.3f, 0.9f, 0.6f, 0.85f));
                AddTrackLane(comp.RightFootStepTrack, "Right Footstep", new Color(0.3f, 0.9f, 0.6f, 0.85f));
                AddTrackLane(comp.WarpPositionTrack, "Warp Position", new Color(0.85f, 0.5f, 1f, 0.85f));
                AddTrackLane(comp.WarpRotationTrack, "Warp Rotation", new Color(1f, 0.5f, 0.7f, 0.85f));
            }

            // View height from the lane stack (lanes clamp to 12–26 px, so
            // this guarantees no scroll for small stacks and scrolls for
            // large ones). Ruler + caption overhead ≈ 44 px.
            int laneCount = (comp?.Events != null ? comp.Events.Count : 0) + m_trackLanes.Count;
            m_view.style.height = Mathf.Max(110f,
                k_RulerH + 4f + laneCount * (26f + k_LaneGap) + 22f);
            MarkDirtyRepaint();
        }

        public void SetFullClip(bool on)
        {
            if (m_fullClip == on)
                return;
            m_fullClip = on;
            MarkDirtyRepaint();
        }

        private void AddTrackLanes(List<MxMEditor.TagTrack> tracks, string prefix, Color baseColor)
        {
            if (tracks == null)
                return;
            for (int i = 0; i < tracks.Count; ++i)
            {
                var t = tracks[i];
                if (t == null || t.Tags == null || t.Tags.Count == 0)
                    continue;
                // Unnamed tracks fall back to an index (matches the preview
                // dropdown's labels).
                string name = string.IsNullOrEmpty(t.TagName)
                    ? $"{prefix} {i + 1}"
                    : t.TagName;
                m_trackLanes.Add(new TrackLane
                {
                    Name = name,
                    Tags = t.Tags,
                    Base = baseColor
                });
            }
        }

        private void AddTrackLane(MxMEditor.TagTrackBase track, string fallbackName, Color baseColor)
        {
            if (track == null || track.TagCount == 0)
                return;
            m_trackLanes.Add(new TrackLane
            {
                Name = string.IsNullOrEmpty(track.Name) ? fallbackName : track.Name,
                Tags = track.TagPositions,
                Base = baseColor
            });
        }

        // -- Time base ------------------------------------------------------------

        /// <summary>Total timeline extent: in FullClip mode the primary clip
        /// may run past the composite slice; otherwise the last marker's
        /// recovery tail can exceed the composite length.</summary>
        private float Total()
        {
            var e = m_entry;
            if (e == null)
                return 1f;

            float total = e.CompositeLength;
            if (m_fullClip && e.Clip != null)
                total = Mathf.Max(total, e.Clip.length);

            var comp = e.OwnerComposite;
            if (comp?.Events != null)
            {
                for (int i = 0; i < comp.Events.Count; ++i)
                {
                    var mk = comp.Events[i];
                    if (mk == null)
                        continue;
                    float end = mk.EventTime + mk.FollowThrough + mk.Recovery;
                    var acts = mk.Actions;
                    if (acts != null)
                        for (int j = 0; j < acts.Count; ++j)
                            end += acts[j];
                    total = Mathf.Max(total, end);
                }
            }
            return Mathf.Max(0.01f, total);
        }

        // -- Scrubbing (IMGUI pass; no UIToolkit capture) -------------------------

        private void HandleScrubEvents(Rect r)
        {
            if (m_entry == null)
                return;

            var e = Event.current;
            switch (e.type)
            {
                case EventType.MouseDown:
                    if (e.button == 0 && r.Contains(e.mousePosition))
                    {
                        m_dragging = true;
                        Scrub(e.mousePosition.x, r);
                        e.Use();
                    }
                    break;

                case EventType.MouseDrag:
                    if (m_dragging && e.button == 0)
                    {
                        Scrub(e.mousePosition.x, r);
                        e.Use();
                    }
                    break;

                case EventType.MouseUp:
                    if (m_dragging && e.button == 0)
                    {
                        m_dragging = false;
                        e.Use();
                    }
                    break;
            }
        }

        private void Scrub(float mouseX, Rect r)
        {
            float frac = Mathf.Clamp01(
                (mouseX - k_PadX) / Mathf.Max(1f, r.width - k_PadX * 2f));
            TimeRequested?.Invoke(frac * Total());
        }

        private const float k_PadX = 12f;
        private const float k_RulerH = 14f;
        private const float k_LaneGap = 2f;

        // -- Drawing (IMGUI: text + band rects are trivial here) -------------------

        private void DrawView()
        {
            // GetRect must run on every GUI pass (layout AND repaint) —
            // skipping it on layout desyncs the group and throws.
            var r = GUILayoutUtility.GetRect(0f, 1e5f, 0f, 1e5f,
                GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(true));

            HandleScrubEvents(r);

            if (Event.current.type != EventType.Repaint)
                return;

            if (m_entry == null)
            {
                GUI.Label(r, "Select a unit in the authoring tree", EditorStyles.centeredGreyMiniLabel);
                return;
            }

            float total = Total();
            var win = m_windowGetter != null ? m_windowGetter() : (0f, total);
            float x0 = r.x + k_PadX;
            float x1 = r.x + r.width - k_PadX;
            float px = (x1 - x0) / total;
            float t = m_timeGetter != null ? m_timeGetter() : 0f;

            // Ruler: adaptive step — target ~50 px between ticks.
            float step = 0.05f;
            while (step * px < 50f) step *= 2f;
            GUIStyle tickStyle = new GUIStyle(EditorStyles.miniLabel);
            tickStyle.alignment = TextAnchor.UpperLeft;
            tickStyle.normal.textColor = k_RulerColor;

            for (float s = 0f; s <= total + 0.0001f; s += step)
            {
                float x = x0 + s * px;
                DrawVLine(x, r.y + 2f, r.y + 8f, 1f, k_RulerColor);
                GUI.Label(new Rect(x - 20f, r.y + 8f, 40f, 14f), s.ToString("0.0#"), tickStyle);
            }

            // Lane area: ruler to caption.
            float laneTop = r.y + k_RulerH + 4f;
            float laneBottom = r.yMax - 18f;

            var comp = m_entry.OwnerComposite;
            int markerCount = comp?.Events != null ? comp.Events.Count : 0;
            int laneCount = markerCount + m_trackLanes.Count;
            float laneH = laneCount > 0
                ? Mathf.Clamp((laneBottom - laneTop) / laneCount - k_LaneGap, 12f, 26f)
                : 0f;

            // Marker lanes: one per event; the focused marker (the bound
            // entry) draws at full alpha with a focus border.
            float y = laneTop;
            if (comp?.Events != null)
            {
                for (int i = 0; i < comp.Events.Count; ++i)
                {
                    var mk = comp.Events[i];
                    if (mk == null)
                        continue;
                    var lane = new Rect(x0, y, x1 - x0, laneH);
                    DrawMarkerLane(lane, mk, px, mk == m_entry.Marker);
                    y += laneH + k_LaneGap;
                }
            }

            // Track lanes: colored spans.
            for (int i = 0; i < m_trackLanes.Count; ++i)
            {
                DrawTrackLane(new Rect(x0, y, x1 - x0, laneH), m_trackLanes[i], px);
                y += laneH + k_LaneGap;
            }

            float lanesEnd = y;

            // Dimming. FullClip: everything past the composite slice is
            // authoring-only (the composite is a slice of the full clip).
            // Otherwise: outside the playable event window.
            if (m_fullClip && m_entry.Clip != null && m_entry.Clip.length > m_entry.CompositeLength)
            {
                float cx = x0 + m_entry.CompositeLength * px;
                EditorGUI.DrawRect(new Rect(cx, laneTop, x1 - cx, lanesEnd - laneTop), k_OutsideColor);
            }
            else
            {
                float ws = Mathf.Clamp(win.Item1, 0f, total);
                float we = Mathf.Clamp(win.Item2, 0f, total);
                if (ws > 0.001f)
                    EditorGUI.DrawRect(new Rect(x0, laneTop, ws * px, lanesEnd - laneTop), k_OutsideColor);
                if (we < total - 0.001f)
                    EditorGUI.DrawRect(new Rect(x0 + we * px, laneTop, (total - we) * px, lanesEnd - laneTop), k_OutsideColor);
            }

            // Playhead across ruler + all lanes.
            float hx = Mathf.Clamp(x0 + t * px, x0, x1);
            DrawVLine(hx, r.y + 2f, lanesEnd, 2f, k_PlayheadColor);

            // Caption: unit + mode + window + time.
            var cap = new GUIStyle(EditorStyles.miniLabel);
            cap.normal.textColor = new Color(0.7f, 0.7f, 0.7f);
            string focus = m_entry.Marker != null ? $"  ·  {m_entry.EventName}" : string.Empty;
            GUI.Label(new Rect(x0, r.yMax - 18f, r.width - k_PadX * 2f, 16f),
                $"{m_entry.OwnerName}{focus}  ·  " +
                (m_fullClip ? "FULL CLIP " : "") +
                $"window {Mathf.Min(win.Item1, total):0.00}–{Mathf.Min(win.Item2, total):0.00}s of {total:0.00}s  ·  t={t:0.00}s",
                cap);
        }

        private void DrawMarkerLane(Rect lane, MxMEditor.EventMarker mk, float px, bool focused)
        {
            // Phase bands, translucent; focused lane full alpha.
            float alpha = focused ? 0.9f : 0.45f;

            System.Func<float, float, Rect> band = (a, b) =>
            {
                float bx = lane.x + a * px;
                float bw = Mathf.Max(1f, (b - a) * px);
                return new Rect(bx, lane.y, bw, lane.height);
            };

            // Windup: [EventTime - Windup, EventTime] (may start < 0 — clamp).
            float windupStart = Mathf.Max(0f, mk.EventTime - mk.Windup);
            DrawBand(band(windupStart, mk.EventTime),
                Alpha(CensusPhaseColors.Windup, alpha), string.Empty);

            // Actions: cumulative segments from EventTime.
            float cursor = mk.EventTime;
            if (mk.Actions != null)
            {
                for (int i = 0; i < mk.Actions.Count; ++i)
                {
                    float a = cursor;
                    cursor += mk.Actions[i];
                    DrawBand(band(a, cursor), Alpha(CensusPhaseColors.Action, alpha), string.Empty);
                }
            }

            // Follow-through, then recovery (clamped to lane).
            float ftEnd = cursor + mk.FollowThrough;
            DrawBand(band(cursor, ftEnd), Alpha(CensusPhaseColors.FollowThrough, alpha), string.Empty);
            DrawBand(band(ftEnd, lane.xMax), Alpha(CensusPhaseColors.Recovery, alpha), string.Empty);

            // Event moment line.
            float ex = lane.x + mk.EventTime * px;
            DrawVLine(ex, lane.y - 1f, lane.yMax + 1f, 1f, new Color(1f, 1f, 1f, focused ? 0.8f : 0.4f));

            // Name: left of the lane, above the bands.
            GUIStyle nameStyle = new GUIStyle(EditorStyles.miniLabel);
            nameStyle.normal.textColor = focused
                ? new Color(1f, 1f, 1f, 1f)
                : new Color(0.85f, 0.85f, 0.85f, 0.8f);
            GUI.Label(new Rect(lane.x + 2f, lane.y - 1f, Mathf.Min(160f, lane.width), lane.height + 2f),
                mk.EventName, nameStyle);

            // Focus border.
            if (focused)
                DrawRectOutline(lane, k_FocusColor);
        }

        private void DrawTrackLane(Rect lane, TrackLane track, float px)
        {
            // Span per tag: [tag.x, next.x) (last tag spans to lane end).
            var tags = track.Tags;
            for (int i = 0; i < tags.Count; ++i)
            {
                float a = tags[i].x;
                float b = i + 1 < tags.Count ? tags[i + 1].x : lane.width / px;
                if (b <= a)
                    continue;
                var rect = new Rect(lane.x + a * px, lane.y, Mathf.Max(1f, (b - a) * px), lane.height);
                EditorGUI.DrawRect(rect, Alpha(TagColor(tags[i].y, track.Base), 0.55f));
            }

            GUIStyle nameStyle = new GUIStyle(EditorStyles.miniLabel);
            nameStyle.normal.textColor = new Color(0.9f, 0.9f, 0.9f, 0.85f);
            GUI.Label(new Rect(lane.x + 2f, lane.y - 1f, Mathf.Min(160f, lane.width), lane.height + 2f),
                track.Name, nameStyle);
        }

        /// <summary>Per-tag-id hue from the shared phase-color helper, over
        /// the lane's base alpha.</summary>
        private static Color TagColor(float tagId, Color baseColor)
        {
            var c = CensusPhaseColors.TagColor(tagId);
            c.a = baseColor.a;
            return c;
        }

        private static Color Alpha(Color c, float a)
        {
            c.a *= a;
            return c;
        }

        private static void DrawBand(Rect rect, Color color, string label)
        {
            if (rect.width < 1f)
                return;

            EditorGUI.DrawRect(rect, color);
            if (!string.IsNullOrEmpty(label) && rect.width > 14f)
            {
                GUIStyle st = new GUIStyle(EditorStyles.miniLabel);
                st.alignment = TextAnchor.MiddleCenter;
                st.normal.textColor = new Color(0.05f, 0.05f, 0.05f, 0.9f);
                GUI.Label(rect, label, st);
            }
        }

        private static void DrawRectOutline(Rect rect, Color color)
        {
            EditorGUI.DrawRect(new Rect(rect.x, rect.y, rect.width, 1f), color);
            EditorGUI.DrawRect(new Rect(rect.x, rect.yMax - 1f, rect.width, 1f), color);
            EditorGUI.DrawRect(new Rect(rect.x, rect.y, 1f, rect.height), color);
            EditorGUI.DrawRect(new Rect(rect.xMax - 1f, rect.y, 1f, rect.height), color);
        }

        private static void DrawVLine(float x, float y0, float y1, float width, Color color)
        {
            EditorGUI.DrawRect(new Rect(x, y0, width, Mathf.Max(1f, y1 - y0)), color);
        }
    }
}
#endif