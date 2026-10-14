// ----------------------------------------------------------------------------
// -- Element Toolkit (id.elemtk) — generic editor wrapper primitives        --
// -- Fork lineage: instance.id/ElementAnimationToolkit (MIT)                --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System;
using UnityEditor;
using UnityEngine;
using UnityEngine.UIElements;

namespace id.elemtk.Framework
{
    /// <summary>
    /// A VisualElement that hosts the stock inspector of a target object
    /// (Editor.CreateEditor) as part of a larger window layout.
    ///
    /// Use for building workflow windows around assets whose authoring UI
    /// already exists as a CustomEditor (e.g. MxM PreProcessData). The hosted
    /// editor stays fully interactive (serialized property machinery works).
    ///
    /// Lifetime: the underlying Editor is created on target set and destroyed
    /// on detach/dispose. Call Dispose() from the owning window's OnDisable.
    /// Repaint coupling: the host repaints when the window repaints (same
    /// schedule), or immediately via RepaintEmbedded().
    /// </summary>
    public class EmbeddedInspector : VisualElement, IDisposable
    {
        private Editor m_editor;
        private UnityEngine.Object m_target;
        private IMGUIContainer m_container;

        /// <summary>The object whose stock inspector is hosted.</summary>
        public UnityEngine.Object Target
        {
            get { return m_target; }
        }

        /// <summary>True while a target is attached and the editor alive.</summary>
        public bool IsValid
        {
            get { return m_target != null && m_editor != null; }
        }

        public EmbeddedInspector()
        {
            style.flexGrow = 1;
            // Opaque backing: the IMGUI body itself paints transparent, so
            // whatever sits behind (grid canvas) would show through the
            // fields — unreadable overlap once the canvas has content.
            style.backgroundColor = new Color(0.2f, 0.2f, 0.2f);
            m_container = new IMGUIContainer(DrawEmbedded);
            Add(m_container);

            // Experiment controls live in the inspector's context menu, not
            // in host toolbars — diagnostics, off in normal workflow.
            this.AddManipulator(new ContextualMenuManipulator(evt =>
            {
                evt.menu.AppendAction("Trace interaction",
                    _ => TraceInteraction = !TraceInteraction,
                    TraceInteraction ? DropdownMenuAction.Status.Checked : DropdownMenuAction.Status.Normal);
                evt.menu.AppendAction("Widget map",
                    _ => WidgetMapEnabled = !WidgetMapEnabled,
                    WidgetMapEnabled ? DropdownMenuAction.Status.Checked : DropdownMenuAction.Status.Normal);
                evt.menu.AppendAction("Verbose diagnostics",
                    _ => VerboseLogging = !VerboseLogging,
                    VerboseLogging ? DropdownMenuAction.Status.Checked : DropdownMenuAction.Status.Normal);
            }));
        }

        // -- Interaction tracing (experiment) ---------------------------------
        // Captures every accepted mouse press and keyboard-focus change
        // inside the stock inspector by diffing IMGUI's published control
        // state (hotControl/keyboardControl) around the inspector's own
        // dispatch. No patching: hotControl is set by whichever control
        // accepted the event; 0 = nothing accepted. Scope is automatic —
        // DrawEmbedded runs only while THIS container dispatches.
        // Limitations: ids are opaque (no type/rect/propertyPath), hover is
        // invisible (IMGUI hover is not control-based), compound widgets
        // (property drawers) appear as one unit.

        private bool m_trace;
        private bool m_widgetMap;
        private int m_lastHotControl;
        private int m_lastKeyboardControl;

        /// <summary>Enable the press/focus tracer (console log per event).</summary>
        public bool TraceInteraction
        {
            get { return m_trace; }
            set
            {
                if (m_trace == value)
                    return;
                m_trace = value;
                m_lastHotControl = 0;
                m_lastKeyboardControl = 0;
            }
        }

        /// <summary>Enable the Harmony-backed widget map (per-repaint
        /// {propertyPath -> rect} capture + hit-testing).</summary>
        public bool WidgetMapEnabled
        {
            get { return m_widgetMap; }
            set { m_widgetMap = value; }
        }

        /// <summary>Route the deep diagnostics ([Trace]/[Map]/[Walk]/[Core]
        /// console logs) to the probe. Off = quiet; interaction events still
        /// fire when TraceInteraction is on.</summary>
        public bool VerboseLogging
        {
            get { return WidgetMapProbe.VerboseLogging; }
            set { WidgetMapProbe.VerboseLogging = value; }
        }

        /// <summary>Fired when a control inside the inspector accepts a mouse
        /// press/release or acquires/loses keyboard focus. Name is the probe's
        /// semantic identity: '«label»' (core-mapped widget), propertyPath
        /// (field), authoring method (caller walk), or null when unresolved.</summary>
        public struct InspectorInteraction
        {
            public int ControlId;
            public EventType Cause;   // MouseDown / MouseUp / ValidateCommand (focus marker)
            public string Name;       // semantic identity, null if unresolved
            public Vector2 Position;  // container-local
        }

        public event Action<InspectorInteraction> InteractionEvent;

        private void TraceInteractionDiff(EventType typeBefore, Vector2 pos, int hotBefore, int kbBefore)
        {
            int hot = GUIUtility.hotControl;
            int kb = GUIUtility.keyboardControl;

            if (WidgetMapProbe.VerboseLogging && typeBefore == EventType.MouseDown)
                Debug.Log($"[Trace] down pos={pos} hot {hotBefore}->{hot} kb {kbBefore}->{kb}");

            // Press accepted/released: hotControl changed during dispatch.
            if (hot != hotBefore && (typeBefore == EventType.MouseDown || typeBefore == EventType.MouseUp))
            {
                // Resolution order: core/field rect hit, then the
                // MouseDown-time caller walk, then opaque.
                string name = WidgetMapProbe.HitTest(pos)
                              ?? WidgetMapProbe.PressCaller(hot);
                Debug.Log($"[EmbeddedInspector] {(typeBefore == EventType.MouseDown ? "press  " : "release ")}id={hot} '{name ?? "?"}' at {pos}");
                InteractionEvent?.Invoke(new InspectorInteraction
                {
                    ControlId = hot,
                    Cause = typeBefore,
                    Name = name,
                    Position = pos
                });
                m_lastHotControl = hot;
            }

            // Focus moved: keyboardControl changed during dispatch.
            if (kb != kbBefore)
            {
                string semantic = WidgetMapProbe.HitTest(pos);
                InteractionEvent?.Invoke(new InspectorInteraction
                {
                    ControlId = kb,
                    Cause = EventType.ValidateCommand, // 'focus' marker
                    Name = semantic,
                    Position = pos
                });
                m_lastKeyboardControl = kb;
            }
        }

        /// <summary>Attach (or re-attach) the stock inspector for a target.
        /// Re-attaching the same target is a no-op; a different target
        /// destroys the previous editor first.</summary>
        public void SetTarget(UnityEngine.Object target)
        {
            if (target == m_target)
            {
                if (m_target == null || m_editor != null)
                    return;
            }

            Detach();

            if (target == null)
                return;

            m_target = target;
            m_editor = Editor.CreateEditor(target);
        }

        private void DrawEmbedded()
        {
            if (m_editor == null)
                return;

            try
            {
                bool scoped = m_trace || m_widgetMap;
                if (scoped)
                {
                    if (m_widgetMap)
                        WidgetMapProbe.EnsurePatched();
                    WidgetMapProbe.BeginCapture();
                }

                // Snapshot BEFORE dispatch: Event.current is a mutable
                // singleton — controls that Use() the event mutate its type
                // to Used, so only pre-copied values are trustworthy after.
                bool log = m_trace;
                EventType typeBefore = Event.current != null ? Event.current.type : EventType.Ignore;
                Vector2 posBefore = Event.current != null ? Event.current.mousePosition : Vector2.zero;
                int hotBefore = GUIUtility.hotControl;
                int kbBefore = GUIUtility.keyboardControl;

                m_editor.OnInspectorGUI();

                if (log)
                    TraceInteractionDiff(typeBefore, posBefore, hotBefore, kbBefore);

                if (scoped)
                    WidgetMapProbe.EndCapture();
            }
            catch (Exception e)
            {
                // A hosted inspector throwing must not wedge the host window.
                // Only end capture if we actually began it (Release-mode of
                // the probe: unscoped EndCapture would be a no-op anyway).
                WidgetMapProbe.EndCapture();
                GUILayout.Label($"Embedded inspector error: {e.Message}", EditorStyles.boldLabel);
            }
        }

        /// <summary>Force the embedded inspector to repaint now.</summary>
        public void RepaintEmbedded()
        {
            m_container.MarkDirtyRepaint();
        }

        private void Detach()
        {
            if (m_editor != null)
            {
                UnityEngine.Object.DestroyImmediate(m_editor);
                m_editor = null;
            }

            m_target = null;
        }

        public void Dispose()
        {
            Detach();
        }
    }
}
#endif