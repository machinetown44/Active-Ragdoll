// ----------------------------------------------------------------------------
// -- MxM Wrapper — live pose-data view: top-down trajectory canvas +        --
// -- simulated-root playback + trigger timeline. Own timeline/seek,         --
// -- decoupled from the census preview. The search hookup arrives as a     --
// -- PoseProbe (LiveSearchRunner, next step); until then the view draws    --
// -- the trajectory/skeleton proxy from the session model.                 --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using UnityEditor;

namespace MxMWrapper
{
    public class LiveViewPanel : CensusPanel, System.IDisposable
    {
        public const float k_Width = 400f;
        public const float k_Height = 560f;

        private const float k_TimelineH = 64f; // seek + trigger lanes
        private const float k_PointPickPx = 14f;
        private const float k_PointSizeM = 0.1f;  // trajectory handle diamond
        private const float k_RayLenM = 0.5f;     // pinned look-direction ray

        /// <summary>Read-only access for the runner/settings panels.</summary>
        public LiveSessionModel Model { get { return m_model; } }

        /// <summary>Attach the search runner (owns the probe). Detaches any
        /// previous runner. Note: the runner owns NativeArrays — callers
        /// must Dispose the old one themselves.</summary>
        public void AttachRunner(LiveSearchRunner runner)
        {
            m_runner?.Dispose();
            m_runner = runner;
            SetProbe(runner);
            m_time = 0f;
        }

        /// <summary>Detached runner disposal (window teardown path).</summary>
        public void DetachRunner()
        {
            m_runner?.Dispose();
            m_runner = null;
        }

        /// <summary>Live playhead moved (tick or scrub).</summary>
        public event System.Action<float> LiveTimeChanged;

        /// <summary>Pluggable pose source for the skeleton proxy. The search
        /// runner implements this once the job hookup lands; null = proxy
        /// figure only (root + heading from the spline).</summary>
        public interface PoseProbe
        {
            /// <summary>Root-relative bone offsets (hip = root) of the pose
            /// to render at the simulated root, or null when unavailable.</summary>
            void GetSkeleton(IList<Vector3> offsetsOut);
            /// <summary>Current chosen-pose debug text, or null.</summary>
            string PoseInfo { get; }
        }

        private LiveSessionModel m_model;
        private PoseProbe m_probe;
        private LiveSearchRunner m_runner;

        // Playback state (own timeline — independent of the census preview).
        private float m_time;
        private bool m_playing;
        private float m_lastTick;
        private int m_lastVersion = -1;

        // 3D trajectory handles: last screen positions of the control points
        // (x,y GUI coords, z = valid flag). Refreshed each repaint; picked in
        // event passes against the previous frame's values.
        private readonly List<Vector3> m_handleScreen = new List<Vector3>(8);
        private int m_dragPointIndex = -1;
        private int m_hoverPoint = -1;

        // Viewport orbit/pan/zoom state (scene-view-style camera control,
        // same model as the census preview). While playing, the orbit target
        // tracks the figure (offset preserved); while paused, RMB pans it.
        private Vector3 m_orbitTarget = new Vector3(0f, 1f, 0f);
        private float m_orbitYaw;
        private float m_orbitPitch = 15f;
        private float m_orbitDist = 3f;
        private bool m_framed;
        private Vector3 m_lastFigurePos;

        private readonly IMGUIContainer m_view;
        private readonly Button m_playButton;
        private readonly Label m_timeLabel;

        // 3D rig viewport (PreviewRenderUtility — same mechanism as the
        // census preview). The rig plays the runner's chosen clip; its root
        // is pinned to the simulated spline root, decoupled from clip root
        // motion (see SampleRig). The trajectory is drawn over the same
        // viewport with GL lines and manipulated through screen-space handles.
        private PreviewRenderUtility m_preview;
        private GameObject m_rig;
        private Transform m_rigRootNode; // clip-root-motion carrier ("Root" child)
        private GameObject m_grid;       // ground grid (scene-object, like census)
        private GameObject m_trajViz;    // trajectory viz root (scene objects so
                                         // Render() draws it — GL over the
                                         // preview texture does not composite)
        private MeshFilter m_trajMesh;   // spline polyline (rebuilt on edit)
        private MeshFilter m_handleMesh; // per-point cross + look rays
        private int m_trajVizVersion = -1;
        private int m_lastHover = -1;
        private int m_lastDrag = -1;
        private const float k_SplineY = 0.02f;
        private static readonly Color k_SplineCol = new Color(0.4f, 0.8f, 1f, 0.9f);
        private static Material s_lineMat; // shared "Sprites/Default"

        public LiveViewPanel() : base("Live Data", k_Width, k_Height)
        {
            m_preview = new PreviewRenderUtility(true);
            m_preview.camera.backgroundColor = new Color(0.24f, 0.24f, 0.24f);
            m_preview.camera.clearFlags = CameraClearFlags.SolidColor;

            m_model = new LiveSessionModel();
            m_model.SeedDefault();
            m_lastVersion = m_model.Version;

            // Header: play/pause + time readout.
            m_playButton = new Button(TogglePlay) { text = "▶", style = { width = 24f } };
            Header.Add(m_playButton);
            m_timeLabel = new Label("0.00 s")
            {
                style = { marginLeft = 8f, color = new Color(0.7f, 0.7f, 0.7f) }
            };
            Header.Add(m_timeLabel);

            m_view = new IMGUIContainer(DrawView) { style = { flexGrow = 1f } };
            Content.Add(m_view);

            // ~30 Hz tick: advance the live playhead, run the search,
            // resample the rig, repaint.
            m_view.schedule.Execute(Tick).Every(33);
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

        /// <summary>Set the preview body (preprocess data Prefab). Same
        /// rig-reuse rule as the census preview.</summary>
        public void SetRigPrefab(GameObject prefab)
        {
            if (m_rig != null && prefab != null &&
                m_rig.name == prefab.name + "(LiveRig)")
                return;

            ClearRig();
            if (prefab == null)
                return;

            m_rig = (GameObject)m_preview.InstantiatePrefabInScene(prefab);
            m_rig.name = prefab.name + "(LiveRig)";
            var animators = m_rig.GetComponentsInChildren<Animator>(true);
            for (int i = 0; i < animators.Length; ++i)
                animators[i].enabled = false;

            EnsureGrid();
            FrameCamera();

            // The clip root-motion carrier: imported rigs put baked root
            // motion on a child (usually "Root"). Cancelling its local
            // offset pins the figure to the simulated spline root.
            m_rigRootNode = null;
            for (int i = 0; i < m_rig.transform.childCount; ++i)
            {
                var child = m_rig.transform.GetChild(i);
                if (string.Equals(child.name, "Root",
                    System.StringComparison.OrdinalIgnoreCase))
                {
                    m_rigRootNode = child;
                    break;
                }
            }
        }

        private void ClearRig()
        {
            ClearTrajViz();
            if (m_grid != null)
            {
                Object.DestroyImmediate(m_grid);
                m_grid = null;
            }
            if (m_rig != null)
            {
                Object.DestroyImmediate(m_rig);
                m_rig = null;
                m_rigRootNode = null;
            }
            m_framed = false;
        }

        /// <summary>Replace the session (settings panel reload, etc.).</summary>
        public void SetModel(LiveSessionModel model)
        {
            m_model = model ?? new LiveSessionModel();
            if (m_model.Points.Count == 0)
                m_model.SeedDefault();
            m_lastVersion = m_model.Version;
            m_time = 0f;
            m_playing = false;
            m_playButton.text = "▶";
        }

        public void SetProbe(PoseProbe probe)
        {
            m_probe = probe;
        }

        private void TogglePlay()
        {
            m_playing = !m_playing;
            m_lastTick = 0f;
            m_playButton.text = m_playing ? "⏸" : "▶";
            if (m_time >= m_model.Settings.Duration)
                m_time = 0f;
        }

        private void Tick()
        {
            if (m_model.Version != m_lastVersion)
            {
                // Model edited: the decision cache is stale; reset playback
                // (the runner rebuilds its sequence from t=0).
                m_lastVersion = m_model.Version;
                m_time = Mathf.Min(m_time, m_model.Settings.Duration);
            }

            if (m_playing)
            {
                float now = (float)EditorApplication.timeSinceStartup;
                float dt = m_lastTick > 0f ? Mathf.Min(0.1f, now - m_lastTick) : 0f;
                m_lastTick = now;

                m_time += dt;
                if (m_time >= m_model.Settings.Duration)
                {
                    m_time = m_model.Settings.Duration;
                    m_playing = false;
                    m_playButton.text = "▶";
                }
                if (m_runner != null)
                    m_runner.Step(dt, m_time);
                LiveTimeChanged?.Invoke(m_time);
            }

            SampleRig();

            m_timeLabel.text = $"{m_time:0.00} / {m_model.Settings.Duration:0.00} s" +
                (m_probe?.PoseInfo != null ? $"   ·   {m_probe.PoseInfo}" : "");
            m_view.MarkDirtyRepaint();
        }

        // -- Rig sampling -----------------------------------------------------

        /// <summary>Sample the runner's chosen clip onto the rig and pin the
        /// figure to the simulated spline root. Clip root motion is CANCELLED
        /// by compensating the rig placement with the root node's animated
        /// local offset — position comes from the trajectory only.</summary>
        private void SampleRig()
        {
            if (m_rig == null)
                return;

            var pts = m_model.Points;
            float speed = m_model.Settings.Speed;
            Vector2 rootPos = LiveTrajectory.PosAtTime(pts, m_time, speed);
            float yawRad = LiveTrajectory.HeadingAtTime(pts, m_time, speed);

            if (m_runner != null && m_runner.TryGetPlayback(out var clip, out float t))
                clip.SampleAnimation(m_rig, t);

            Vector3 rootOffset = Vector3.zero;
            if (m_rigRootNode != null)
                rootOffset = m_rigRootNode.localPosition;
            float c = Mathf.Cos(yawRad), s = Mathf.Sin(yawRad);
            Vector3 rot = new Vector3(
                rootOffset.x * c - rootOffset.z * s, rootOffset.y,
                rootOffset.x * s + rootOffset.z * c);

            m_rig.transform.position = new Vector3(rootPos.x, 0f, rootPos.y) - rot;
            m_rig.transform.rotation = Quaternion.Euler(0f, yawRad * Mathf.Rad2Deg, 0f);
        }

        private void UpdateCamera()
        {
            if (m_preview == null)
                return;

            // While playing the camera tracks the figure — the user's orbit
            // offset (yaw/pitch/dist + last pan offset) is preserved.
            if (m_playing && m_rig != null)
                m_orbitTarget += m_rig.transform.position - m_lastFigurePos;
            m_lastFigurePos = m_rig != null ? m_rig.transform.position : Vector3.zero;

            ApplyCameraTransform();
        }

        /// <summary>Frame the orbit target on the trajectory start point
        /// (covers the spline handles + the figure).</summary>
        private void FrameCamera()
        {
            var pts = m_model.Points;
            m_orbitTarget = pts.Count > 0
                ? To3D(pts[0].Position, 1f)
                : new Vector3(0f, 1f, 0f);
            m_orbitDist = 6f;
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

        // -- Preview-scene furniture (grid, like the census preview) -----------

        private void EnsureGrid()
        {
            if (m_grid != null)
                return;

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

            m_grid = new GameObject("LiveGrid");
            var mesh = new Mesh { vertices = pts };
            mesh.colors = UniformColors(pts.Length, new Color(1f, 1f, 1f, 0.35f));
            var idx = new int[pts.Length];
            for (int i = 0; i < pts.Length; i += 2)
            { idx[i] = i; idx[i + 1] = i + 1; }
            mesh.SetIndices(idx, MeshTopology.Lines, 0);
            m_grid.AddComponent<MeshFilter>().sharedMesh = mesh;
            m_grid.AddComponent<MeshRenderer>().sharedMaterial = LineMaterial();
            m_preview.AddSingleGO(m_grid);
        }

        private static Color[] UniformColors(int count, Color color)
        {
            var colors = new Color[count];
            for (int i = 0; i < count; ++i)
                colors[i] = color;
            return colors;
        }

        private static Material LineMaterial()
        {
            // Unlit vertex-color material: lines read clearly against the
            // shaded rig (same as the census preview's line objects).
            if (s_lineMat == null)
                s_lineMat = new Material(Shader.Find("Sprites/Default"));
            return s_lineMat;
        }

        // -- IMGUI: 3D viewport (rig + trajectory) + timeline -----------------

        private void DrawView()
        {
            var r = new Rect(0f, 0f, k_Width, Content.resolvedStyle.height);
            if (r.height < 10f)
                return;

            // Regions: 3D viewport (rig + trajectory handles) | timeline.
            var view = new Rect(0f, 0f, r.width, r.height - k_TimelineH - 4f);
            var timeline = new Rect(0f, view.yMax + 4f, r.width, k_TimelineH);

            if (Event.current.type == EventType.Repaint && m_rig != null)
            {
                UpdateCamera();
                // Begin/End pair allocates the target texture Render() draws
                // into; calling Render() outside the pair is a guaranteed NRE.
                UpdateTrajViz(view);
                m_preview.BeginPreview(view, GUIStyle.none);
                m_preview.Render(true, false);
                Texture tex = m_preview.EndPreview();
                GUI.DrawTexture(view, tex, ScaleMode.ScaleToFit, true);
            }
            else if (Event.current.type == EventType.Repaint)
            {
                GUI.Label(view, "Set a rig prefab / pick a baked clip",
                    EditorStyles.centeredGreyMiniLabel);
            }

            DrawTimeline(timeline);
            HandleInput(view, timeline);
        }

        // -- Trajectory in the 3D viewport ------------------------------------

        private static Vector3 To3D(Vector2 p, float y = 0.02f)
        {
            return new Vector3(p.x, y, p.y);
        }

        /// <summary>World point → GUI coords in the viewport rect. Returns
        /// false when the point is behind the camera.</summary>
        private bool WorldToGui(Rect view, Vector3 w, out Vector2 gui)
        {
            var cam = m_preview.camera;
            Vector3 sp = cam.WorldToScreenPoint(w);
            if (sp.z <= 0f)
            {
                gui = Vector2.zero;
                return false;
            }
            // BeginPreview sizes the texture to the rect; ratio mapping is
            // scale-safe (pixelWidth vs rect width may differ under DPI).
            gui = new Vector2(
                view.x + sp.x / cam.pixelWidth * view.width,
                view.y + (1f - sp.y / cam.pixelHeight) * view.height);
            return true;
        }

        /// <summary>Mouse GUI point → viewport pixel → camera ray.</summary>
        private Ray GuiToRay(Rect view, Vector2 gui)
        {
            var cam = m_preview.camera;
            var scr = new Vector3(
                (gui.x - view.x) / view.width * cam.pixelWidth,
                (1f - (gui.y - view.y) / view.height) * cam.pixelHeight, 0f);
            return cam.ScreenPointToRay(scr);
        }

        /// <summary>Intersect a camera ray with the horizontal plane at y.</summary>
        private static Vector3 RayPlaneHit(Ray ray, float y)
        {
            float dy = ray.direction.y;
            if (Mathf.Abs(dy) < 1e-5f)
                return ray.origin + ray.direction * 10f;
            float t = (y - ray.origin.y) / dy;
            return ray.origin + ray.direction * t;
        }

        /// <summary>Draw the trajectory spline + control-point handles over the
        /// rendered viewport (GL lines, world space). Also refreshes the
        /// screen-space handle positions used by input picking.</summary>
        // -- Trajectory viz: scene objects in the preview scene ----------------
        // (Census approach: line-mesh GOs rendered by Render(). GL immediate
        // over the preview texture does not composite — it draws into
        // whatever RT is active after EndPreview and ignores the rect.)

        /// <summary>Rebuild the trajectory line meshes when the model changed
        /// (drag, pin toggle, seed). Version-stamped — no per-frame work in
        /// steady state. Also refreshes the screen-space handle cache.</summary>
        private void UpdateTrajViz(Rect view)
        {
            if (m_rig == null)
                return;
            if (m_trajViz == null)
            {
                m_trajViz = new GameObject("LiveTrajViz");
                m_preview.AddSingleGO(m_trajViz);
                m_trajMesh = MakeLineObject("Spline", m_trajViz.transform);
                m_handleMesh = MakeLineObject("Handles", m_trajViz.transform);
            }

            EnsureHandleCache(view);

            // Rebuild on model edits and hover/drag changes (active tint).
            if (m_trajVizVersion == m_model.Version &&
                m_lastHover == m_hoverPoint && m_lastDrag == m_dragPointIndex)
                return;
            m_trajVizVersion = m_model.Version;
            m_lastHover = m_hoverPoint;
            m_lastDrag = m_dragPointIndex;

            var pts = m_model.Points;
            var verts = new List<Vector3>(pts.Count * 8 + 64);
            var cols = new List<Color>(pts.Count * 8 + 64);

            // Spline polyline (Catmull-Rom through the control points).
            int samples = Mathf.Max(24, pts.Count * 8);
            Vector3 prev = To3D(LiveTrajectory.CatmullRom(pts, 0f), k_SplineY);
            for (int i = 1; i < samples; ++i)
            {
                float t = i / (float)(samples - 1) * (pts.Count - 1);
                Vector3 cur = To3D(LiveTrajectory.CatmullRom(pts, t), k_SplineY);
                verts.Add(prev); cols.Add(k_SplineCol);
                verts.Add(cur); cols.Add(k_SplineCol);
                prev = cur;
            }
            SetMesh(m_trajMesh, verts, cols);

            // Per-point crosses + pinned look rays.
            verts.Clear(); cols.Clear();
            for (int i = 0; i < pts.Count; ++i)
            {
                Vector3 p = To3D(pts[i].Position, k_SplineY);
                bool active = i == m_hoverPoint || i == m_dragPointIndex;
                Color col = pts[i].LookPinned
                    ? new Color(1f, 0.6f, 0.25f, active ? 1f : 0.85f)
                    : new Color(0.4f, 0.8f, 1f, active ? 1f : 0.7f);
                AddCross(verts, cols, p, k_PointSizeM, col);
                if (pts[i].LookPinned)
                {
                    // Unity-yaw convention: forward = (sin a, 0, cos a).
                    float a = pts[i].LookAngleRad;
                    var dir = new Vector3(Mathf.Sin(a), 0f, Mathf.Cos(a));
                    Vector3 tip = p + dir * k_RayLenM;
                    verts.Add(p); cols.Add(col);
                    verts.Add(tip); cols.Add(col);
                    var n = new Vector3(dir.z, 0f, -dir.x); // right of the ray
                    verts.Add(tip); cols.Add(col);
                    verts.Add(tip - dir * 0.1f + n * 0.05f); cols.Add(col);
                    verts.Add(tip); cols.Add(col);
                    verts.Add(tip - dir * 0.1f - n * 0.05f); cols.Add(col);
                }
            }
            SetMesh(m_handleMesh, verts, cols);
        }

        private static void AddCross(List<Vector3> verts, List<Color> cols,
            Vector3 p, float d, Color col)
        {
            verts.Add(p + new Vector3(-d, 0f, 0f)); cols.Add(col);
            verts.Add(p + new Vector3(d, 0f, 0f)); cols.Add(col);
            verts.Add(p + new Vector3(0f, 0f, -d)); cols.Add(col);
            verts.Add(p + new Vector3(0f, 0f, d)); cols.Add(col);
            verts.Add(p + new Vector3(-d * 0.7f, 0f, -d * 0.7f)); cols.Add(col);
            verts.Add(p + new Vector3(d * 0.7f, 0f, d * 0.7f)); cols.Add(col);
            verts.Add(p + new Vector3(-d * 0.7f, 0f, d * 0.7f)); cols.Add(col);
            verts.Add(p + new Vector3(d * 0.7f, 0f, -d * 0.7f)); cols.Add(col);
        }

        private static MeshFilter MakeLineObject(string name, Transform parent)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            var mf = go.AddComponent<MeshFilter>();
            mf.sharedMesh = new Mesh();
            go.AddComponent<MeshRenderer>().sharedMaterial = LineMaterial();
            return mf;
        }

        private static void SetMesh(MeshFilter mf, List<Vector3> verts,
            List<Color> cols)
        {
            var mesh = mf.sharedMesh;
            mesh.Clear();
            mesh.SetVertices(verts);
            mesh.SetColors(cols);
            var idx = new int[verts.Count];
            for (int i = 0; i < verts.Count; i += 2)
            { idx[i] = i; idx[i + 1] = i + 1; }
            mesh.SetIndices(idx, MeshTopology.Lines, 0);
            mesh.RecalculateBounds(); // stale bounds would cull the lines
        }

        private void ClearTrajViz()
        {
            if (m_trajViz != null)
            {
                Object.DestroyImmediate(m_trajViz);
                m_trajViz = null;
                m_trajMesh = null;
                m_handleMesh = null;
            }
        }

        /// <summary>Refresh the screen-space handle positions used by the
        /// input pass (previous-frame picking — standard for editor
        /// handles). Runs every repaint; camera may have moved.</summary>
        private void EnsureHandleCache(Rect view)
        {
            m_handleScreen.Clear();
            var pts = m_model.Points;
            for (int i = 0; i < pts.Count; ++i)
            {
                if (WorldToGui(view, To3D(pts[i].Position), out var gui))
                    m_handleScreen.Add(new Vector3(gui.x, gui.y, 1f));
                else
                    m_handleScreen.Add(new Vector3(0f, 0f, 0f));
            }
        }

        private void DrawTimeline(Rect r)
        {
            EditorGUI.DrawRect(r, new Color(0.1f, 0.1f, 0.1f, 1f));
            float dur = Mathf.Max(0.01f, m_model.Settings.Duration);

            // Ruler.
            var ruler = new Rect(r.x, r.y, r.width, 14f);
            EditorGUI.DrawRect(ruler, new Color(0.16f, 0.16f, 0.16f, 1f));
            float secW = r.width / dur;
            var lbl = new GUIStyle(EditorStyles.miniLabel);
            lbl.normal.textColor = new Color(0.6f, 0.6f, 0.6f);
            for (int s = 0; s <= (int)dur; ++s)
            {
                float x = r.x + s * secW;
                EditorGUI.DrawRect(new Rect(x, ruler.yMax - 4f, 1f, 4f),
                    new Color(0.5f, 0.5f, 0.5f, 0.7f));
                GUI.Label(new Rect(x + 2f, ruler.y + 1f, 30f, 12f), $"{s}s", lbl);
            }

            // Trigger lanes.
            var lane = new Rect(r.x + 2f, ruler.yMax + 4f, r.width - 4f, 14f);
            EditorGUI.DrawRect(lane, new Color(0.14f, 0.14f, 0.14f, 1f));
            var trigCol = new Color(0.55f, 0.75f, 1f, 0.55f);
            var trigReqCol = new Color(0.95f, 0.55f, 0.35f, 0.6f);
            var trigs = m_model.Triggers;
            for (int i = 0; i < trigs.Count; ++i)
            {
                float x0 = r.x + trigs[i].Start * secW;
                float x1 = r.x + trigs[i].End * secW;
                EditorGUI.DrawRect(new Rect(x0, lane.y, x1 - x0, lane.height),
                    trigs[i].Required ? trigReqCol : trigCol);
            }

            // Playhead.
            float px = r.x + m_time / dur * r.width;
            EditorGUI.DrawRect(new Rect(px - 0.5f, r.y, 1.5f, r.height),
                new Color(1f, 1f, 1f, 0.95f));
        }

        // -- Input ------------------------------------------------------------

        private void HandleInput(Rect view, Rect timeline)
        {
            var e = Event.current;
            if (e == null)
                return;

            // Timeline: click/drag = seek; context-click on lane = drop a
            // trigger span stub (1 s); shift-context = delete under cursor.
            if (timeline.Contains(e.mousePosition))
            {
                float t = Mathf.Clamp(
                    (e.mousePosition.x - timeline.x) / timeline.width
                        * m_model.Settings.Duration,
                    0f, m_model.Settings.Duration);
                if (e.type == EventType.MouseDrag && e.button == 0)
                {
                    ScrubTo(t);
                    e.Use();
                }
                else if (e.type == EventType.MouseDown && e.button == 0)
                {
                    ScrubTo(t);
                    e.Use();
                }
                else if (e.type == EventType.ContextClick)
                {
                    var lane = new Rect(timeline.x + 2f, timeline.y + 18f,
                        timeline.width - 4f, 14f);
                    if (e.mousePosition.y >= lane.y && e.mousePosition.y <= lane.yMax)
                    {
                        if (e.shift)
                        {
                            for (int i = m_model.Triggers.Count - 1; i >= 0; --i)
                                if (t >= m_model.Triggers[i].Start && t <= m_model.Triggers[i].End)
                                    m_model.Triggers.RemoveAt(i);
                        }
                        else
                        {
                            m_model.Triggers.Add(new TriggerSpan
                            {
                                Start = t,
                                End = Mathf.Min(t + 1f, m_model.Settings.Duration),
                                TagName = "Trigger",
                                Required = true
                            });
                        }
                        m_model.MarkInvalidated();
                        e.Use();
                    }
                }
                return;
            }

            // Viewport input — census-style camera + live-data extras:
            // LMB on a handle = drag point; LMB empty = orbit; MMB = orbit;
            // RMB click on handle = pin toggle; RMB drag = pan;
            // wheel = zoom. While playing the camera tracks the figure.
            if (!view.Contains(e.mousePosition))
                return;

            switch (e.type)
            {
                case EventType.MouseDown:
                    if (e.button == 0)
                    {
                        m_dragPointIndex = PickHandle(e.mousePosition);
                        if (m_dragPointIndex >= 0)
                            e.Use();
                    }
                    else if (e.button == 1)
                    {
                        // Pin toggle only on a handle; drag pans the view.
                        m_rmbPanning = false;
                        if (PickHandle(e.mousePosition) >= 0)
                        {
                            ToggleLookPin(PickHandle(e.mousePosition));
                            e.Use();
                        }
                    }
                    break;

                case EventType.MouseDrag:
                    if (m_dragPointIndex >= 0 && e.button == 0)
                    {
                        // Drag the control point on the ground plane.
                        var ray = GuiToRay(view, e.mousePosition);
                        Vector3 hit = RayPlaneHit(ray, 0f);
                        m_model.Points[m_dragPointIndex].Position =
                            new Vector2(hit.x, hit.z);
                        m_model.MarkInvalidated();
                        e.Use();
                    }
                    else if (e.button == 2 || (e.button == 0 && m_dragPointIndex < 0))
                    {
                        // Orbit (MMB anywhere, LMB on empty space).
                        m_orbitYaw += e.delta.x * 0.4f;
                        m_orbitPitch = Mathf.Clamp(
                            m_orbitPitch + e.delta.y * 0.4f, -85f, 85f);
                        ApplyCameraTransform();
                        e.Use();
                    }
                    else if (e.button == 1)
                    {
                        // RMB drag: pan in the camera's screen plane.
                        m_rmbPanning = true;
                        var cam = m_preview.camera;
                        float scale = m_orbitDist * 0.0015f;
                        m_orbitTarget -= (cam.transform.right * e.delta.x -
                            cam.transform.up * e.delta.y) * scale;
                        ApplyCameraTransform();
                        e.Use();
                    }
                    break;

                case EventType.MouseUp:
                    m_dragPointIndex = -1;
                    m_rmbPanning = false;
                    break;

                case EventType.ScrollWheel:
                    m_orbitDist = Mathf.Clamp(
                        m_orbitDist * (1f + e.delta.y * 0.06f), 0.2f, 100f);
                    ApplyCameraTransform();
                    e.Use();
                    break;

                case EventType.MouseMove:
                    m_hoverPoint = PickHandle(e.mousePosition);
                    break;
            }
        }

        private bool m_rmbPanning;

        private void ToggleLookPin(int index)
        {
            var p = m_model.Points[index];
            p.LookPinned = !p.LookPinned;
            if (p.LookPinned)
            {
                // Seed the pin from the current tangent here.
                p.LookAngleRad = LiveTrajectory.TangentAngleRad(
                    m_model.Points, index);
            }
            m_model.MarkInvalidated();
        }

        private int PickHandle(Vector2 px)
        {
            for (int i = 0; i < m_handleScreen.Count; ++i)
            {
                var h = m_handleScreen[i];
                if (h.z < 1f)
                    continue;
                var d = new Vector2(h.x - px.x, h.y - px.y);
                if (d.sqrMagnitude <= k_PointPickPx * k_PointPickPx)
                    return i;
            }
            return -1;
        }

        /// <summary>Scrub: set time + replay the runner's decisions up to it
        /// (the cached decision sequence makes this cheap later).</summary>
        private void ScrubTo(float t)
        {
            m_time = t;
            if (m_runner != null)
                m_runner.ReplayTo(t);
            SampleRig();
            LiveTimeChanged?.Invoke(t);
        }

        /// <summary>Live timeline state for external seeding logic.</summary>
        public bool IsPlaying { get { return m_playing; } }
        public float CurrentLiveTime { get { return m_time; } }

    }
}
#endif
