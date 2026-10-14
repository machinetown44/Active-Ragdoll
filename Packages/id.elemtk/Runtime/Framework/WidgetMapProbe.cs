// ----------------------------------------------------------------------------
// -- Element Toolkit (id.elemtk) — experiment: IMGUI widget map via Harmony --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using HarmonyLib;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

namespace id.elemtk.Framework
{
    /// <summary>
    /// Experiment: live widget map of a stock IMGUI inspector.
    ///
    /// Design (no caller interception needed for fields):
    ///  - PropertyField postfix: per Repaint event, record {propertyPath -> rect}
    ///    while the draw is scoped to our EmbeddedInspector container.
    ///  - Press/focus events: mousePosition -> containing rect -> propertyPath.
    ///    No rect hit (buttons/actions): fallback = stack walk for the caller.
    ///  - GetControlID postfix: audit set of all live control ids; ids that
    ///    accepted input but have no rect are logged as coverage gaps.
    ///
    /// All patches no-op outside tracing. Scoped via an "inside our draw"
    /// static flag so the rest of the editor is not mapped.
    /// </summary>
    public static class WidgetMapProbe
    {
        private static HarmonyLib.Harmony m_harmony;
        private static bool m_patched;

        /// <summary>Master switch for the deep diagnostics ([Core] census,
        /// [Walk] frame dumps, [Map] field captures). Off = quiet operation;
        /// only the one-time patch-install line and consumer-facing events
        /// remain. Toggle any time; cores not yet fired will log on next use.</summary>
        public static bool VerboseLogging;

        // -- Capture state (valid while m_scopedDraw == true) ----------------

        internal static bool ScopedDraw;
        private static readonly Dictionary<string, Rect> s_FieldRects = new Dictionary<string, Rect>();
        private static readonly HashSet<int> s_LiveControlIds = new HashSet<int>();
        private static readonly HashSet<string> s_RectPaths = new HashSet<string>();

        // controlId -> calling method, recorded during MouseDown dispatch
        // (the whole OnInspectorGUI loop re-executes on event events, so the
        // widget's real author frames ARE on the stack at GetControlID time).
        private static readonly Dictionary<int, string> s_PressCallers = new Dictionary<int, string>();

        /// <summary>{propertyPath -> rect} captured during the last repaint
        /// of the traced inspector.</summary>
        public static IReadOnlyDictionary<string, Rect> FieldRects
        {
            get { return s_FieldRects; }
        }

        // -- Core discovery (experiment) ---------------------------------------
        // Below the public EditorGUI/GUILayout layer sits the internal
        // widget-core funnel (Do*/…Internal statics). Every widget — public
        // wrappers, custom drawers, MxM's hand-drawn rows — converges there,
        // and core calls carry id + rect + GUIContent. Discovered by
        // reflection, patched with one universal __args postfix.

        private static readonly HashSet<string> s_LoggedCores = new HashSet<string>();
        private static readonly Dictionary<string, Rect> s_CoreRects = new Dictionary<string, Rect>();

        private static void CorePostfix(object[] __args, MethodBase __originalMethod)
        {
            if (!ScopedDraw || Event.current == null)
                return;

            string name = __originalMethod.DeclaringType?.Name + "." + __originalMethod.Name;

            // Discovery census: first fire of each core, with its payload.
            if (VerboseLogging && s_LoggedCores.Add(name))
                Debug.Log($"[Core] {name}({DescribeArgs(__args)})");

            var type = Event.current.type;
            if (type != EventType.Repaint && type != EventType.MouseDown)
                return;

            int id = 0;
            bool hasId = false;
            Rect rect = default;
            bool hasRect = false;
            string label = null;

            for (int i = 0; i < __args.Length; i++)
            {
                var a = __args[i];
                if (a is int n)
                {
                    if (!hasId) { id = n; hasId = true; }
                }
                else if (a is Rect r)
                {
                    rect = r;
                    hasRect = true;
                }
                else if (a is GUIContent gc)
                {
                    if (label == null && !string.IsNullOrEmpty(gc.text))
                        label = gc.text;
                }
                else if (a is string s)
                {
                    if (label == null && s.Length > 0)
                        label = s;
                }
            }

            // Core calls carry the widget's own id: direct id → label join,
            // consumed by press diff via PressCaller.
            if (hasId && label != null)
                s_PressCallers[id] = "«" + label + "»";

            if (label == null)
                return;

            if (!hasRect)
            {
                rect = GUILayoutUtility.GetLastRect();
                hasRect = rect.width > 0f && rect.height > 0f;
            }

            if (hasRect)
                s_CoreRects["«" + label + "»"] = rect;
        }

        private static string DescribeArgs(object[] args)
        {
            var parts = new string[args.Length];
            for (int i = 0; i < args.Length; i++)
            {
                var a = args[i];
                if (a == null) parts[i] = "null";
                else if (a is GUIContent gc) parts[i] = $"GUIContent('{gc.text}')";
                else if (a is Rect r) parts[i] = r.ToString("F0");
                else if (a is string s) parts[i] = $"'{(s.Length > 24 ? s.Substring(0, 24) + "…" : s)}'";
                else parts[i] = a.GetType().Name + " " + a;
            }

            var joined = string.Join(", ", parts);
            return joined.Length > 300 ? joined.Substring(0, 300) + "…" : joined;
        }

        // -- Integration with EmbeddedInspector ------------------------------

        /// <summary>Call from DrawEmbedded BEFORE the inspector dispatch:
        /// begins scoped capture for this event. Returns captured data on
        /// request afterwards via FieldRects.</summary>
        // Verbose-walk diagnostics (experiment): log the full frame list of
        // the first stack walk and first field capture per dispatch event.
        private static bool s_LogNextWalk;
        private static bool s_LogNextField;

        internal static void BeginCapture()
        {
            s_LiveControlIds.Clear();
            s_PressCallers.Clear();
            s_LogNextWalk = true;
            s_LogNextField = true;
            ScopedDraw = true;
        }

        internal static void EndCapture()
        {
            ScopedDraw = false;
        }

        /// <summary>Map a panel-space point to the propertyPath whose rect
        /// contains it, or null. Rects are container-local; mousePosition in
        /// the embedded IMGUI event is container-local too, so direct hit.</summary>
        public static string HitTest(Vector2 localPoint)
        {
            foreach (var kv in s_CoreRects)
            {
                if (kv.Value.Contains(localPoint))
                    return kv.Key;
            }

            foreach (var kv in s_FieldRects)
            {
                if (kv.Value.Contains(localPoint))
                    return kv.Key;
            }
            return null;
        }

        /// <summary>Caller of the control that accepted the current MouseDown
        /// (mapped during dispatch via GetControlID stack walks), or null.</summary>
        public static string PressCaller(int controlId)
        {
            string caller;
            return s_PressCallers.TryGetValue(controlId, out caller) ? caller : null;
        }

        /// <summary>Walk the current stack for the widget's authoring method.
        /// Skips engine (UnityEngine AND UnityEditor), Harmony, BCL and our
        /// own frames; first managed frame outside those is the caller.</summary>
        private static string WalkCaller()
        {
            var st = new System.Diagnostics.StackTrace(2, false);
            string caller = null;
            var frames = VerboseLogging ? new List<string>(st.FrameCount) : null;

            for (int i = 0; i < st.FrameCount; i++)
            {
                var mb = st.GetFrame(i).GetMethod();
                var ns = mb.DeclaringType?.Namespace ?? string.Empty;
                bool skipped =
                    ns.StartsWith("UnityEngine") || ns.StartsWith("UnityEditor") ||
                    ns.StartsWith("HarmonyLib") || ns.StartsWith("System") ||
                    ns.StartsWith("Mono") || ns.StartsWith("id.elemtk");

                frames?.Add($"{(skipped ? "-" : "+")} {ns}.{mb.DeclaringType?.Name}.{mb.Name}");

                if (caller == null && !skipped)
                    caller = (mb.DeclaringType?.Name ?? "?") + "." + mb.Name;
            }

            // Verbose: full frame list with skip (+/-) marks — what the walk saw.
            if (s_LogNextWalk)
            {
                s_LogNextWalk = false;
                if (frames != null)
                    Debug.Log("[Walk] caller='" + caller + "' frames:\n  " + string.Join("\n  ", frames));
            }

            return caller;
        }

        // -- Patch installation ------------------------------------------------

        /// <summary>Install Harmony patches (idempotent). Editor-only.</summary>
        public static bool EnsurePatched()
        {
            if (m_patched)
                return true;

            try
            {
                m_harmony = new HarmonyLib.Harmony("id.elemtk.widgetmap");

                // 2) GetControlID audit — every interactive control passes
                //    through SOME overload; patch ALL of them by name (no
                //    exact-signature matching — Unity 6 overload sets differ
                //    from older versions and silent misses cause '?' press
                //    reports). Postfix uses __result only: no name binding,
                //    valid for every overload.
                var post = new HarmonyMethod(AccessTools.Method(
                    typeof(WidgetMapProbe), nameof(GetControlIdPostfix)));

                int patchCount = 0;
                foreach (var type in new[] { typeof(GUIUtility), typeof(EditorGUIUtility) })
                {
                    foreach (var m in type.GetMethods(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static)
                        .Where(mi => mi.Name == "GetControlID"))
                    {
                        try
                        {
                            m_harmony.Patch(m, postfix: post);
                            patchCount++;
                        }
                        catch (Exception e)
                        {
                            Debug.LogWarning($"[WidgetMapProbe] skipped {type.Name}.GetControlID({m.GetParameters().Length} args): {e.Message}");
                        }
                    }
                }

                // 1b) PropertyField — same enumerate-all strategy: every
                // overload starting (Rect, SerializedProperty, ...).
                var fieldPost = new HarmonyMethod(AccessTools.Method(
                    typeof(WidgetMapProbe), nameof(PropertyFieldPostfix)));

                foreach (var m in typeof(EditorGUI).GetMethods(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static)
                    .Where(mi => mi.Name == nameof(EditorGUI.PropertyField)))
                {
                    var ps = m.GetParameters();
                    if (ps.Length < 2 || ps[0].ParameterType != typeof(Rect) || ps[1].ParameterType != typeof(SerializedProperty))
                        continue;

                    try
                    {
                        m_harmony.Patch(m, postfix: fieldPost);
                        patchCount++;
                    }
                    catch (Exception e)
                    {
                        Debug.LogWarning($"[WidgetMapProbe] skipped PropertyField({ps.Length} args): {e.Message}");
                    }
                }

                m_patched = true;

                // Core discovery: reflect internal widget-core candidates
                // (Do*/…Internal statics on the IMGUI types), patch each
                // with the universal postfix. Fire-count log shows which
                // cores actually drive this inspector; payloads give
                // id → label joins without any per-widget code.
                int coreCount = 0;
                var corePost = new HarmonyMethod(AccessTools.Method(
                    typeof(WidgetMapProbe), nameof(CorePostfix)));

                var coreNameRx = new System.Text.RegularExpressions.Regex("^Do[A-Z]|Internal$");
                foreach (var type in new[] { typeof(GUI), typeof(EditorGUI), typeof(GUIUtility), typeof(EditorGUIUtility), typeof(GUILayoutUtility) })
                {
                    var cores = type.GetMethods(BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.Static)
                        .Where(m => coreNameRx.IsMatch(m.Name) && !m.IsGenericMethod && !m.IsAbstract);

                    foreach (var m in cores)
                    {
                        var ps = m.GetParameters();
                        if (ps.Length == 0 || ps.Length > 6)
                            continue;
                        if (ps.Any(p => p.ParameterType.IsByRef || p.ParameterType.IsPointer))
                            continue;

                        try
                        {
                            m_harmony.Patch(m, postfix: corePost);
                            coreCount++;
                        }
                        catch
                        {
                            // Cores with unsupported signatures (ref struct
                            // args, native-impl markers) — discovery skips.
                        }
                    }
                }

                Debug.Log($"[WidgetMapProbe] patches installed ({patchCount} public + {coreCount} internal cores, Harmony {typeof(HarmonyLib.Harmony).Assembly.GetName().Version})");
            }
            catch (Exception e)
            {
                Debug.LogError($"[WidgetMapProbe] patch failed: {e.Message}");
                m_patched = false;
            }

            return m_patched;
        }

        public static void Unpatch()
        {
            if (!m_patched)
                return;

            m_harmony.UnpatchAll(m_harmony.Id);
            m_patched = false;
        }

        // -- Postfix bodies (run for ALL callers; cheap no-ops when unscoped) --

        private static void PropertyFieldPostfix(Rect position, SerializedProperty property)
        {
            if (!ScopedDraw || Event.current == null || Event.current.type != EventType.Repaint)
                return;

            if (s_LogNextField)
            {
                s_LogNextField = false;
                if (VerboseLogging)
                    Debug.Log($"[Map] PropertyField postfix fired: path='{property?.propertyPath}' rect={position}");
            }

            if (property == null || string.IsNullOrEmpty(property.propertyPath))
                return;

            s_FieldRects[property.propertyPath] = position;
            s_RectPaths.Add(property.propertyPath);
        }

        private static void GetControlIdPostfix(ref int __result)
        {
            if (!ScopedDraw)
                return;

            s_LiveControlIds.Add(__result);

            // MouseDown dispatch: this call's stack contains the widget's
            // authoring code — record controlId -> caller for press reports.
            // Walks per interactive id per click (~dozens); clicks are rare.
            if (s_PressCallers != null && Event.current != null && Event.current.type == EventType.MouseDown
                && !s_PressCallers.ContainsKey(__result))
            {
                var caller = WalkCaller();
                if (caller != null)
                    s_PressCallers[__result] = caller;
            }
        }
    }
}
#endif