// ----------------------------------------------------------------------------
// -- Element Toolkit (id.elemtk) — generic editor wrapper primitives        --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using UnityEngine;
using UnityEngine.UIElements;

namespace id.elemtk.Framework
{
    /// <summary>
    /// A free-panning 2D canvas: children are placed at absolute world
    /// coordinates (see <see cref="AddNode"/>), the whole content layer pans
    /// by drag on empty background, and a reference grid is drawn behind
    /// everything, moving with the content (world-attached, not static).
    /// </summary>
    public class PannableCanvas : VisualElement
    {
        private const float k_MinorStep = 20f;
        private const float k_MajorStep = 100f; // must be a multiple of minor
        private static readonly Color k_MinorColor = new Color(0f, 0f, 0f, 0.18f);
        private static readonly Color k_MajorColor = new Color(0f, 0f, 0f, 0.38f);

        private Vector2 m_pan;

        private readonly VisualElement m_contentRoot;

        /// <summary>World→screen offset. Content and grid shift together.</summary>
        public Vector2 Pan
        {
            get { return m_pan; }
            set
            {
                m_pan = value;
                m_contentRoot.style.translate = new Translate(m_pan.x, m_pan.y);
                MarkDirtyRepaint();
            }
        }

        public PannableCanvas()
        {
            style.overflow = Overflow.Hidden;

            // Own background = grid (element mesh paints beneath children).
            generateVisualContent = DrawGrid;

            m_contentRoot = new VisualElement
            {
                pickingMode = PickingMode.Ignore,
                style =
                {
                    position = Position.Absolute,
                    left = 0f, top = 0f,
                    width = 0f, height = 0f // children overflow freely
                }
            };
            Add(m_contentRoot);

            RegisterCallback<MouseDownEvent>(OnMouseDown);
            RegisterCallback<MouseMoveEvent>(OnMouseMove);
            RegisterCallback<MouseUpEvent>(OnMouseUp);
            RegisterCallback<MouseCaptureOutEvent>(OnMouseUp);
        }

        /// <summary>Add a free-floating node at world coordinates.</summary>
        public void AddNode(VisualElement node, float x, float y)
        {
            node.style.position = Position.Absolute;
            node.style.left = x;
            node.style.top = y;
            m_contentRoot.Add(node);
        }

        // -- Pan by drag on empty background ---------------------------------

        private bool m_panning;
        private Vector2 m_dragStartPan;
        private Vector2 m_dragStartMouse;

        private void OnMouseDown(MouseDownEvent evt)
        {
            // Left button on the canvas surface itself only — child nodes
            // (buttons, labels with picking) stay interactive.
            if (evt.button != 0 || evt.target != this)
                return;

            m_panning = true;
            m_dragStartPan = m_pan;
            m_dragStartMouse = evt.mousePosition;
            this.CaptureMouse();
            evt.StopPropagation();
        }

        private void OnMouseMove(MouseMoveEvent evt)
        {
            if (!m_panning)
                return;

            Pan = m_dragStartPan + (evt.mousePosition - m_dragStartMouse);
            evt.StopPropagation();
        }

        private void OnMouseUp(EventBase evt)
        {
            if (!m_panning)
                return;

            m_panning = false;
            this.ReleaseMouse();
        }

        // -- World-attached grid ----------------------------------------------

        private void DrawGrid(MeshGenerationContext ctx)
        {
            float w = contentRect.width;
            float h = contentRect.height;
            if (w <= 0f || h <= 0f)
                return;

            DrawLineSet(ctx, k_MinorStep, k_MinorColor);
            DrawLineSet(ctx, k_MajorStep, k_MajorColor);
        }

        // World position k paints at screen k + pan; iterate the world range
        // visible at the current pan and stroke the aligned lines.
        private void DrawLineSet(MeshGenerationContext ctx, float step, Color color)
        {
            var painter = ctx.painter2D;
            painter.strokeColor = color;
            painter.lineWidth = 1f;

            float w = contentRect.width;
            float h = contentRect.height;

            painter.BeginPath();
            float worldStartX = Mathf.Floor(-m_pan.x / step) * step;
            for (float k = worldStartX; k <= -m_pan.x + w; k += step)
            {
                float sx = k + m_pan.x;
                painter.MoveTo(new Vector2(sx, 0f));
                painter.LineTo(new Vector2(sx, h));
            }

            float worldStartY = Mathf.Floor(-m_pan.y / step) * step;
            for (float k = worldStartY; k <= -m_pan.y + h; k += step)
            {
                float sy = k + m_pan.y;
                painter.MoveTo(new Vector2(0f, sy));
                painter.LineTo(new Vector2(w, sy));
            }
            painter.Stroke();
        }
    }
}
#endif