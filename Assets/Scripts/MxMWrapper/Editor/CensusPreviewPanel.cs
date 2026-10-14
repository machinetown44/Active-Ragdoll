// ----------------------------------------------------------------------------
// -- MxM Wrapper — census preview panel: always-on clip preview driven by   --
// -- PreviewRenderUtility + AnimationClip.SampleAnimation (no Playables,    --
// -- no Play mode). Selecting a census entry seeks to the event window and  --
// -- loops it.                                                              --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using UnityEditor;

namespace MxMWrapper
{
    public class CensusPreviewPanel : CensusPanel, System.IDisposable
    {
        public const float k_Width = 400f;

        /// <summary>Playhead moved (playback tick or external SetTime).</summary>
        public event System.Action<float> TimeChanged;

        private PreviewRenderUtility m_preview;
        private IMGUIContainer m_view;
        private GameObject m_rig;
        private float m_time;
        private float m_lastTick;
        private bool m_playing;
        private CensusEntry m_entry;
        private bool m_fullClip;
        private ClipWindow m_window = new ClipWindow { start = 0f, end = 1f };

        // Viewport orbit/pan/zoom state (scene-view-style camera control).
        private Vector3 m_orbitTarget = Vector3.zero;
        private float m_orbitYaw;
        private float m_orbitPitch = 15f;
        private float m_orbitDist = 3f;
        private bool m_framed;

        // In-view overlay widgets + debug viz.
        private bool m_showGrid = true;
        private bool m_showRoot = true;
        private bool m_showTraj = true;
        private GameObject m_vizRoot;
        private GameObject m_grid;
        private GameObject m_rootMarker;
        private GameObject m_trajectory;
        private readonly List<GameObject> m_contacts = new List<GameObject>();

        /// <summary>Color sources for the trajectory segments: index 0 =
        /// event phase (default), then one entry per authored track
        /// (tags, favour tags, footsteps) exposed by the bound composite.</summary>
        private struct ColorTrack
        {
            public string Label;
            public List<Vector2> Tags; // (time, tagId), sorted by time
        }
        private readonly List<ColorTrack> m_colorTracks = new List<ColorTrack>();
        private DropdownField m_colorDropdown;
        private int m_colorMode;
        private Texture2D m_iconPlay, m_iconPause, m_iconGrid, m_iconRoot, m_iconTraj;
        private Texture2D m_iconBack, m_iconFwd;

        private struct ClipWindow
        {
            public float start;
            public float end;
            public float length
            {
                get { return Mathf.Max(0.01f, end - start); }
            }
        }

        public float CurrentTime
        {
            get { return m_time; }
        }

        public CensusPreviewPanel() : base("Preview", k_Width, 520f)
        {
            m_preview = new PreviewRenderUtility(true);
            m_preview.camera.backgroundColor = new Color(0.24f, 0.24f, 0.24f);
            m_preview.camera.clearFlags = CameraClearFlags.SolidColor;

            m_colorDropdown = new DropdownField { style = { marginLeft = 8f, marginRight = 6f } };
            m_colorDropdown.RegisterValueChangedCallback(evt =>
            {
                // String-indexed: this Unity DropdownField exposes the value,
                // not the selection index.
                m_colorMode = Mathf.Max(0, m_colorTracks.Count + 1 == m_colorDropdown.choices.Count
                    ? m_colorDropdown.choices.IndexOf(evt.newValue)
                    : 0);
                BuildTrajectory();
            });
            Header.Add(m_colorDropdown);

            m_view = new IMGUIContainer(DrawView)
            {
                style = { flexGrow = 1f }
            };
            Content.Add(m_view);

            // Playback tick ~30 Hz: advance time, resample, repaint.
            m_view.schedule.Execute(Tick).Every(33);
        }

        /// <summary>Point the preview at a census entry: swaps clip, seeks
        /// to the event start and loops the event window. Markerless units
        /// (idle sets, blend spaces) loop the full clip instead — there is
        /// no event window. Null = idle message.</summary>
        public void SetEntry(CensusEntry entry)
        {
            m_entry = entry;
            if (entry == null)
            {
                m_window = new ClipWindow { start = 0f, end = 1f };
                SetPlaying(false);
                return;
            }

            SetRig(entry.Clip != null ? FindPrefabForRig() : null);
            m_window = ComputeWindow();
            RebuildColorModes();
            m_time = m_window.start;
            m_lastTick = 0f; // selection keeps the current pause state
            m_view.MarkDirtyRepaint();
            RefreshViz();
        }

        /// <summary>FullClip mode: sample the whole primary clip, ignoring
        /// the composite slice. Toggling re-derives the window.</summary>
        public void SetFullClip(bool on)
        {
            if (m_fullClip == on)
                return;
            m_fullClip = on;
            if (m_entry != null)
            {
                m_window = ComputeWindow();
                m_time = Mathf.Clamp(m_time, m_window.start, m_window.end);
                Resample();
                BuildTrajectory();
            }
            m_view.MarkDirtyRepaint();
        }

        private ClipWindow ComputeWindow()
        {
            var e = m_entry;
            float clipLen = e.Clip != null ? e.Clip.length : 0f;

            // FullClip: the whole primary clip participates.
            if (m_fullClip && e.Clip != null)
                return new ClipWindow { start = 0f, end = clipLen };

            // Markerless units: loop the whole playable range (idle sets
            // loop the primary clip; blend spaces the target clip).
            if (e.Marker == null)
            {
                float total = Mathf.Max(e.CompositeLength, clipLen);
                return new ClipWindow { start = 0f, end = Mathf.Max(0.01f, total) };
            }

            return new ClipWindow
            {
                start = Mathf.Max(0f, e.StartTime),
                // Clamp to the full composite timeline, not the primary clip.
                end = Mathf.Min(e.CompositeLength, e.EndTime)
            };
        }

        /// <summary>Scrub: jump the playhead (from the timeline panel).</summary>
        public void SetTime(float t)
        {
            m_time = Mathf.Clamp(t, m_window.start, m_window.end);
            Resample();
            TimeChanged?.Invoke(m_time);
            m_view.MarkDirtyRepaint();
        }

        public float WindowStart
        {
            get { return m_window.start; }
        }

        public float WindowEnd
        {
            get { return m_window.end; }
        }

        /// <summary>Jump to the previous / next phase boundary (window start,
        /// event time, follow-through start, recovery start, window end).</summary>
        public void JumpPhase(int dir)
        {
            var mk = m_entry?.Marker;
            if (mk == null)
                return;

            float actions = 0f;
            var acts = mk.Actions;
            if (acts != null)
            {
                for (int i = 0; i < acts.Count; ++i)
                    actions += acts[i];
            }

            var bounds = new List<float>
            {
                m_window.start,
                mk.EventTime,
                mk.EventTime + actions,
                mk.EventTime + actions + mk.FollowThrough,
                m_window.end
            };

            if (dir > 0)
            {
                for (int i = 0; i < bounds.Count; ++i)
                {
                    if (bounds[i] > m_time + 0.001f)
                    { SetTime(bounds[i]); return; }
                }
            }
            else
            {
                for (int i = bounds.Count - 1; i >= 0; --i)
                {
                    if (bounds[i] < m_time - 0.001f)
                    { SetTime(bounds[i]); return; }
                }
            }
        }

        public void Dispose()
        {
            ClearRig();
            if (m_preview != null)
            {
                m_preview.Cleanup();
                m_preview = null;
            }
        }

        // -- Playback ------------------------------------------------------------

        private void Tick()
        {
            if (!m_playing || m_rig == null)
                return;

            float now = (float)EditorApplication.timeSinceStartup;
            float dt = m_lastTick > 0f ? Mathf.Min(0.1f, now - m_lastTick) : 0f;
            m_lastTick = now;

            m_time += dt;
            if (m_time > m_window.end)
                m_time = m_window.start; // loop the event window

            Resample();
            TimeChanged?.Invoke(m_time);
            m_view.MarkDirtyRepaint();
        }

        private void Resample()
        {
            if (m_rig == null)
                return;
            SampleComposite(m_time);
            UpdateRootMarker();
        }

        /// <summary>Samples the composite plan: find the segment that owns
        /// 't', sample that clip at the segment-local time. Segments are
        /// Before + Primary + After; markers are timed against the merged
        /// timeline, so this is the correct frame of reference.</summary>
        private void SampleComposite(float t)
        {
            var segs = m_entry?.Segments;
            if (segs == null || segs.Count == 0)
                return;

            // FullClip bypasses the composite plan: the primary clip plays
            // in full (the composite is only an authoring slice).
            if (m_fullClip && m_entry.Clip != null)
            {
                m_entry.Clip.SampleAnimation(m_rig,
                    Mathf.Clamp(t, 0f, Mathf.Max(0.01f, m_entry.Clip.length - 0.001f)));
                return;
            }

            for (int i = 0; i < segs.Count; ++i)
            {
                var s = segs[i];
                if (t <= s.Offset + s.Length || i == segs.Count - 1)
                {
                    float local = Mathf.Clamp(t - s.Offset, 0f,
                        Mathf.Max(0.01f, s.Length - 0.001f));
                    s.Clip.SampleAnimation(m_rig, local);
                    return;
                }
            }
        }

        private void SetPlaying(bool playing)
        {
            m_playing = playing;
            m_lastTick = 0f;
        }

        // -- Viewport input: orbit (LMB drag / MMB drag), pan (RMB drag),
        // -- zoom (wheel). Mirrors scene-view feel inside the IMGUI view.
        private void HandleViewportInput(Rect r)
        {
            if (!m_framed)
                return;

            var e = Event.current;
            if (e == null || !r.Contains(e.mousePosition))
                return;

            switch (e.type)
            {
                case EventType.MouseDrag:
                    if (e.button == 0 || e.button == 2)
                    {
                        m_orbitYaw += e.delta.x * 0.4f;
                        m_orbitPitch = Mathf.Clamp(m_orbitPitch + e.delta.y * 0.4f, -85f, 85f);
                    }
                    else if (e.button == 1)
                    {
                        // Pan in the camera's screen plane.
                        var cam = m_preview.camera;
                        float scale = m_orbitDist * 0.0015f;
                        Vector3 right = cam.transform.right;
                        Vector3 up = cam.transform.up;
                        m_orbitTarget -= (right * e.delta.x - up * e.delta.y) * scale;
                    }
                    else
                        return;

                    ApplyCameraTransform();
                    e.Use();
                    break;

                case EventType.ScrollWheel:
                    m_orbitDist = Mathf.Clamp(
                        m_orbitDist * (1f + e.delta.y * 0.06f), 0.2f, 100f);
                    ApplyCameraTransform();
                    e.Use();
                    break;
            }
        }

        private static float ClipLength(AnimationClip clip)
        {
            return clip != null ? clip.length : 1f;
        }

        // -- Preview scene -------------------------------------------------------

        private GameObject FindPrefabForRig()
        {
            // The preprocess data carries the rig prefab the data was built
            // against — that is the correct preview body.
            return m_currentData != null ? m_currentData.Prefab : null;
        }

        private MxMEditor.MxMPreProcessData m_currentData;

        /// <summary>Dataset context for rig lookup (called by the window).</summary>
        public void SetData(MxMEditor.MxMPreProcessData data)
        {
            m_currentData = data;
        }

        private void SetRig(GameObject prefab)
        {
            if (m_rig != null && prefab != null &&
                m_rig.name == prefab.name + "(PreviewRig)")
                return; // same rig, keep

            ClearRig();
            if (prefab == null)
                return;

            m_rig = (GameObject)m_preview.InstantiatePrefabInScene(prefab);
            m_rig.name = prefab.name + "(PreviewRig)";

            // Strip runtime drivers: preview sampling owns the pose.
            var animators = m_rig.GetComponentsInChildren<Animator>(true);
            for (int i = 0; i < animators.Length; ++i)
                animators[i].enabled = false;

            FrameCamera(m_rig);
            EnsureViz();
        }

        private void ClearRig()
        {
            ClearViz();
            if (m_rig != null)
            {
                Object.DestroyImmediate(m_rig);
                m_rig = null;
            }
        }

        private void FrameCamera(GameObject rig)
        {
            var bounds = new Bounds(rig.transform.position, Vector3.zero);
            bool hasBounds = false;
            var renderers = rig.GetComponentsInChildren<Renderer>(true);
            for (int i = 0; i < renderers.Length; ++i)
            {
                if (!hasBounds) { bounds = renderers[i].bounds; hasBounds = true; }
                else bounds.Encapsulate(renderers[i].bounds);
            }

            if (!hasBounds)
            {
                m_orbitTarget = Vector3.zero;
                m_orbitDist = 3f;
            }
            else
            {
                m_orbitTarget = bounds.center;
                m_orbitDist = Mathf.Max(0.6f, bounds.extents.magnitude) * 2.6f;
            }
            m_framed = true;
            ApplyCameraTransform();
        }

        private void ApplyCameraTransform()
        {
            var cam = m_preview.camera;
            Quaternion rot = Quaternion.Euler(m_orbitPitch, m_orbitYaw, 0f);
            cam.transform.position = m_orbitTarget + rot * new Vector3(0f, 0f, -m_orbitDist);
            cam.transform.LookAt(m_orbitTarget);
            cam.nearClipPlane = 0.01f;
            cam.farClipPlane = m_orbitDist * 20f + 10f;
        }

        // -- Drawing -------------------------------------------------------------

        private void DrawView()
        {
            var r = GUILayoutUtility.GetRect(0f, 1e5f, 0f, 1e5f,
                GUILayout.ExpandWidth(true), GUILayout.ExpandHeight(true));

            HandleViewportInput(r);

            if (Event.current.type == EventType.Repaint && m_rig != null)
            {
                // Begin/End pair allocates the target texture Render() draws
                // into; calling Render() outside the pair is a guaranteed NRE.
                m_preview.BeginPreview(r, GUIStyle.none);
                m_preview.Render(true, false);
                Texture tex = m_preview.EndPreview();
                GUI.DrawTexture(r, tex, ScaleMode.ScaleToFit, true);

                DrawContactLabels(r);
            }
            else if (Event.current.type == EventType.Repaint)
            {
                string msg = m_entry == null
                    ? "Select an event in the census"
                    : "No rig prefab on the preprocess data";
                GUI.Label(r, msg, EditorStyles.centeredGreyMiniLabel);
            }

            // Widgets AFTER the render: GUI.DrawTexture would paint over them.
            // They still allocate on the layout pass (no early return), so
            // control IDs match and clicks register.
            DrawOverlayWidgets(r);
        }

        // -- Overlay widgets: toggle icons top-right, play/pause bottom-center --
        private static Texture2D Icon(string name)
        {
            var dark = EditorGUIUtility.IconContent("d_" + name);
            if (dark?.image != null)
                return dark.image as Texture2D;
            return EditorGUIUtility.IconContent(name)?.image as Texture2D;
        }

        private void DrawOverlayWidgets(Rect r)
        {
            if (m_iconPlay == null)
            {
                m_iconPlay = Icon("Play");
                m_iconPause = Icon("Pause");
                m_iconGrid = Icon("Grid.Default");
                m_iconRoot = Icon("AvatarPivot");
                m_iconTraj = Icon("Animation.EventMarker");
                m_iconBack = Icon("back");
                m_iconFwd = Icon("forward");
            }

            const float size = 22f, gap = 4f;
            var style = EditorStyles.iconButton;
            float x = r.xMax - size - 8f;
            float y = r.y + 8f;

            if (ToggleIcon(new Rect(x, y, size, size), m_iconTraj, m_showTraj, style))
            { m_showTraj = !m_showTraj; ApplyToggles(); }
            x -= size + gap;
            if (ToggleIcon(new Rect(x, y, size, size), m_iconRoot, m_showRoot, style))
            { m_showRoot = !m_showRoot; ApplyToggles(); }
            x -= size + gap;
            if (ToggleIcon(new Rect(x, y, size, size), m_iconGrid, m_showGrid, style))
            { m_showGrid = !m_showGrid; ApplyToggles(); }

            // Playback cluster: prev phase | play/pause | next phase.
            float cy = r.yMax - size - 10f;
            float cx = r.center.x;
            if (GUI.Button(new Rect(cx - size * 1.5f - gap, cy, size, size),
                m_iconBack, style))
                JumpPhase(-1);
            if (GUI.Button(new Rect(cx - size * 0.5f, cy, size, size),
                m_playing ? m_iconPause : m_iconPlay, style))
                SetPlaying(!m_playing);
            if (GUI.Button(new Rect(cx + size * 0.5f + gap, cy, size, size),
                m_iconFwd, style))
                JumpPhase(1);
        }

        private bool ToggleIcon(Rect rect, Texture icon, bool active, GUIStyle style)
        {
            var prev = GUI.backgroundColor;
            GUI.backgroundColor = active
                ? new Color(1f, 1f, 1f, 1f)
                : new Color(1f, 1f, 1f, 0.35f);
            bool clicked = icon != null && GUI.Button(rect, icon, style);
            GUI.backgroundColor = prev;
            return clicked;
        }

        private void DrawContactLabels(Rect r)
        {
            if (m_rig == null || m_contacts.Count == 0)
                return;

            var cam = m_preview.camera;
            for (int i = 0; i < m_contacts.Count; ++i)
            {
                if (m_contacts[i] == null || !m_contacts[i].activeSelf)
                    continue;
                var vp = cam.WorldToViewportPoint(m_contacts[i].transform.position);
                if (vp.z <= 0f)
                    continue;
                var p = new Vector2(vp.x * r.width, (1f - vp.y) * r.height);
                if (!r.Contains(p))
                    continue;
                GUI.Label(new Rect(p.x + 6f, p.y - 8f, 60f, 14f), $"C{i + 1}",
                    EditorStyles.miniLabel);
            }
        }

        // -- Debug viz in the preview scene (grid, root, trajectory, contacts) --
        private void EnsureViz()
        {
            if (m_vizRoot != null)
                return;
            if (m_rig == null)
                return;

            m_vizRoot = new GameObject("CensusViz");
            m_preview.AddSingleGO(m_vizRoot);

            m_grid = MakeLineObject("Grid", GridPoints(), new Color(1f, 1f, 1f, 0.35f));
            m_grid.transform.SetParent(m_vizRoot.transform, false);

            m_rootMarker = MakeMarker("Root", new Color(0.2f, 0.85f, 1f, 1f));
            m_rootMarker.transform.localScale = Vector3.one * 0.05f;
            m_rootMarker.transform.SetParent(m_vizRoot.transform, false);
            UpdateRootMarker();

            m_trajectory = MakeLineObject("Trajectory", new Vector3[0],
                new Color(1f, 0.55f, 0.1f, 0.95f));
            m_trajectory.transform.SetParent(m_vizRoot.transform, false);

            ApplyToggles();
        }

        private void RefreshViz()
        {
            EnsureViz();
            BuildTrajectory();
            BuildContacts();
        }

        private static Vector3[] GridPoints()
        {
            const int half = 10;
            var pts = new Vector3[2 * 2 * (2 * half + 1)];
            int j = 0;
            for (int i = -half; i <= half; ++i)
            {
                pts[j++] = new Vector3(i, 0f, -half);
                pts[j++] = new Vector3(i, 0f, half);
                pts[j++] = new Vector3(-half, 0f, i);
                pts[j++] = new Vector3(half, 0f, i);
            }
            return pts;
        }

        private void BuildTrajectory()
        {
            if (m_trajectory == null || m_entry?.Clip == null || m_rig == null)
                return;

            // Sample the root path across the full timeline extent (matches
            // the timeline panel's Total(): FullClip extends past the
            // composite slice; markerless units use the clip length),
            // colored per phase to match the timeline bands, then restore
            // the pose.
            const int samples = 64;
            float total = TimelineExtent();
            var path = new Vector3[samples];
            var colors = new Color[samples];
            for (int i = 0; i < samples; ++i)
            {
                float t = total * i / (samples - 1);
                SampleComposite(t);
                path[i] = m_rig.transform.position;
                colors[i] = ColorForTime(t);
                // Mirror the timeline's dimming: path outside the playable
                // event window (before composite start / past the end) fades,
                // marking where the event window actually begins on the path.
                if (t < m_window.start || t > m_window.end)
                    colors[i].a *= 0.25f;
            }
            SampleComposite(m_time);

            Object.DestroyImmediate(m_trajectory);
            var segs = ToPolyline(path, colors, out var segColors);
            m_trajectory = MakeLineObject("Trajectory", segs, segColors);
            m_trajectory.transform.SetParent(m_vizRoot.transform, false);
            m_trajectory.SetActive(m_showTraj);
        }

        /// <summary>Trajectory segment color for time t: phase colors in the
        /// default mode; otherwise the active tag id's hue from the track
        /// chosen in the header dropdown (gaps render dim gray).</summary>
        private Color ColorForTime(float t)
        {
            if (m_colorMode > 0 && m_colorMode <= m_colorTracks.Count)
            {
                float id = ActiveTagId(m_colorTracks[m_colorMode - 1].Tags, t);
                if (id >= 0f)
                    return CensusPhaseColors.TagColor(id);
                return new Color(0.25f, 0.25f, 0.25f, 0.6f);
            }
            return CensusPhaseColors.At(m_entry.Marker, t);
        }

        /// <summary>Rebuild the header dropdown's choices from the bound
        /// composite: 'Phase' first, then every non-empty tag/favour/footstep
        /// track. Hidden when only the phase mode exists.</summary>
        private void RebuildColorModes()
        {
            m_colorTracks.Clear();

            var comp = m_entry?.OwnerComposite;
            if (comp != null)
            {
                AddColorTracks(comp.TagTracks, "Tag");
                AddColorTracks(comp.FavourTagTracks, "Favour");
                AddColorTrack(comp.LeftFootStepTrack, "Footstep Left");
                AddColorTrack(comp.RightFootStepTrack, "Footstep Right");
            }

            var labels = new List<string>(m_colorTracks.Count + 1) { "Phase" };
            for (int i = 0; i < m_colorTracks.Count; ++i)
                labels.Add(m_colorTracks[i].Label);

            m_colorDropdown.choices = labels;
            m_colorMode = Mathf.Clamp(m_colorMode, 0, labels.Count - 1);
            m_colorDropdown.SetValueWithoutNotify(labels[m_colorMode]);
            m_colorDropdown.style.display =
                m_colorTracks.Count > 0 ? DisplayStyle.Flex : DisplayStyle.None;
        }

        private void AddColorTracks(List<MxMEditor.TagTrack> tracks, string prefix)
        {
            if (tracks == null)
                return;
            for (int i = 0; i < tracks.Count; ++i)
            {
                var t = tracks[i];
                if (t?.Tags == null || t.Tags.Count == 0)
                    continue;
                // Unnamed tracks are common in MxM data (tags authored
                // without naming their track) — fall back to an index.
                string name = string.IsNullOrEmpty(t.TagName)
                    ? $"{prefix} {i + 1}"
                    : t.TagName;
                m_colorTracks.Add(new ColorTrack
                {
                    Label = $"{prefix}: {name}",
                    Tags = t.Tags
                });
            }
        }

        private void AddColorTrack(MxMEditor.TagTrackBase track, string label)
        {
            if (track?.TagPositions == null || track.TagPositions.Count == 0)
                return;
            m_colorTracks.Add(new ColorTrack
            {
                Label = label,
                Tags = track.TagPositions
            });
        }

        /// <summary>Timeline extent shared with the timeline panel's Total():
        /// FullClip includes the primary clip length past the composite
        /// slice; markerless units (idle sets / blend spaces) use the clip
        /// length — CensusEntry.EndTime would deref a null marker.</summary>
        private float TimelineExtent()
        {
            float clipLen = m_entry.Clip != null ? m_entry.Clip.length : 0f;
            float total = Mathf.Max(m_entry.CompositeLength, clipLen);
            if (m_fullClip)
                return Mathf.Max(0.01f, total);

            var comp = m_entry.OwnerComposite;
            if (comp?.Events != null)
            {
                for (int i = 0; i < comp.Events.Count; ++i)
                {
                    var mk = comp.Events[i];
                    if (mk == null) continue;
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

        /// <summary>Active tag id at time t: last tag with x &lt;= t. Tags are
        /// time-sorted; -1 when before the first tag (no active tag).</summary>
        private static float ActiveTagId(List<Vector2> tags, float t)
        {
            float id = -1f;
            for (int i = 0; i < tags.Count; ++i)
            {
                if (tags[i].x <= t)
                    id = tags[i].y;
                else
                    break;
            }
            return id;
        }

        private void BuildContacts()
        {
            ClearContacts();
            var contacts = m_entry?.Marker?.Contacts;
            if (contacts == null || m_vizRoot == null)
                return;

            for (int i = 0; i < contacts.Count; ++i)
            {
                var c = MakeMarker("Contact" + i, new Color(0.2f, 0.85f, 1f, 1f));
                c.transform.position = contacts[i].Position;
                c.transform.localScale = Vector3.one * 0.08f;
                c.transform.SetParent(m_vizRoot.transform, false);
                m_contacts.Add(c);
            }
            ApplyToggles();
        }

        private void UpdateRootMarker()
        {
            if (m_rootMarker != null && m_rig != null)
                m_rootMarker.transform.position = m_rig.transform.position;
        }

        private void ApplyToggles()
        {
            if (m_grid != null) m_grid.SetActive(m_showGrid);
            if (m_rootMarker != null) m_rootMarker.SetActive(m_showRoot);
            if (m_trajectory != null) m_trajectory.SetActive(m_showTraj);
            for (int i = 0; i < m_contacts.Count; ++i)
                if (m_contacts[i] != null)
                    m_contacts[i].SetActive(m_showTraj);
        }

        private void ClearViz()
        {
            ClearContacts();
            if (m_grid != null) { Object.DestroyImmediate(m_grid); m_grid = null; }
            if (m_rootMarker != null) { Object.DestroyImmediate(m_rootMarker); m_rootMarker = null; }
            if (m_trajectory != null) { Object.DestroyImmediate(m_trajectory); m_trajectory = null; }
            if (m_vizRoot != null) { Object.DestroyImmediate(m_vizRoot); m_vizRoot = null; }
        }

        private void ClearContacts()
        {
            for (int i = 0; i < m_contacts.Count; ++i)
                if (m_contacts[i] != null)
                    Object.DestroyImmediate(m_contacts[i]);
            m_contacts.Clear();
        }

        private static Vector3[] ToPolyline(Vector3[] pts, Color[] colors, out Color[] segColors)
        {
            if (pts.Length < 2)
            {
                segColors = new Color[0];
                return new Vector3[0];
            }
            var segs = new Vector3[(pts.Length - 1) * 2];
            segColors = new Color[segs.Length];
            for (int i = 0, j = 0; i < pts.Length - 1; ++i)
            {
                segs[j] = pts[i];
                segColors[j++] = colors[i];
                segs[j] = pts[i + 1];
                segColors[j++] = colors[i + 1];
            }
            return segs;
        }

        private static Vector3[] ToPolyline(Vector3[] pts)
        {
            return ToPolyline(pts,
                UniformColors(pts.Length, new Color(1f, 1f, 1f, 0.35f)), out _);
        }

        private static Color[] UniformColors(int count, Color color)
        {
            var colors = new Color[count];
            for (int i = 0; i < count; ++i)
                colors[i] = color;
            return colors;
        }

        private static Material s_lineMaterial;

        private static Material LineMaterial()
        {
            // Unlit vertex-color material: lines/markers read clearly against
            // the shaded rig.
            if (s_lineMaterial == null)
                s_lineMaterial = new Material(Shader.Find("Sprites/Default"));
            return s_lineMaterial;
        }

        private static GameObject MakeLineObject(string name, Vector3[] pts, Color color)
        {
            return MakeLineObject(name, pts, UniformColors(pts.Length, color));
        }

        private static GameObject MakeLineObject(string name, Vector3[] pts, Color[] colors)
        {
            var go = new GameObject(name);
            var mesh = new Mesh { vertices = pts };
            mesh.colors = colors;
            var idx = new int[pts.Length];
            for (int i = 0; i < pts.Length; i += 2)
            { idx[i] = i; idx[i + 1] = i + 1; }
            mesh.SetIndices(idx, MeshTopology.Lines, 0);

            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            go.AddComponent<MeshRenderer>().sharedMaterial = LineMaterial();
            return go;
        }

        private static GameObject MakeMarker(string name, Color color)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = name;
            Object.DestroyImmediate(go.GetComponent<Collider>());
            var mat = new Material(Shader.Find("Sprites/Default")) { color = color };
            go.GetComponent<MeshRenderer>().sharedMaterial = mat;
            return go;
        }
    }
}
#endif