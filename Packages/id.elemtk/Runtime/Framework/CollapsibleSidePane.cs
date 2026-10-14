// ----------------------------------------------------------------------------
// -- Element Toolkit (id.elemtk) — generic editor wrapper primitives        --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System;
using UnityEngine;
using UnityEngine.UIElements;
using UnityEngine.UIElements.Experimental;

namespace id.elemtk.Framework
{
    /// <summary>
    /// A side pane that collapses horizontally (width tween — same axis as
    /// the collapse). Adapts to the side it is docked on: a Right pane lays
    /// out gutter | divider | content (grip faces the canvas), a Left pane
    /// mirrors it — content | divider | gutter, chevrons and drag direction
    /// flipped, title rotated to read top-down.
    ///
    /// Horizontal counterpart to EATK's AnimatedFoldout (which animates
    /// height). Content should be attached to <see cref="Content"/>.
    /// </summary>
    public class CollapsibleSidePane : VisualElement
    {
        /// <summary>Window edge the pane is docked against.</summary>
        public enum Side { Left, Right }

        /// <summary>When true the pane shrinks to its content height (pinned
        /// to the top of the dock area) instead of stretching full height —
        /// short inspectors stop wasting the space below them. Content still
        /// caps at the dock height and scrolls when taller.</summary>
        private readonly bool m_fitContent;

        private const float k_GutterWidth = 24f;
        private const float k_DividerWidth = 5f;
        private const float k_ToggleSize = 18f;
        private const int k_AnimationMs = 220;
        private const float k_MinWidth = 180f;
        private const float k_MaxWidth = 900f;
        // Single source of truth for the divider/grip plate color.
        private static readonly Color k_DividerColor = new Color(0.24f, 0.24f, 0.24f);
        // Hover states: grip = click affordance, divider = drag affordance.
        private static readonly Color k_HoverColor = new Color(0.35f, 0.35f, 0.35f);

        private float m_ExpandedWidth = 460f;
        private bool m_expanded = true;
        private bool m_hovered;
        // Docking side. Everything asymmetric (element order, grip anchor,
        // title rotation, chevron text, drag sign) derives from this.
        private readonly Side m_side;
        private bool IsLeft { get { return m_side == Side.Left; } }

        private readonly VisualElement m_gutter;
        private readonly VisualElement m_grip;
        private readonly Button m_toggleButton;
        private readonly Label m_titleLabel;
        private readonly VisualElement m_separator;
        private readonly ScrollView m_content;

        /// <summary>Scrollable content container — add children here.</summary>
        public VisualElement Content
        {
            get { return m_content; }
        }

        /// <summary>True when expanded.</summary>
        public bool Expanded
        {
            get { return m_expanded; }
        }

        /// <summary>Raised after an expand/collapse transition completes
        /// (not per-frame during the tween).</summary>
        public event Action ExpandedChanged;

        public CollapsibleSidePane(string title, float expandedWidth = 460f,
            Side side = Side.Right, bool fitContentHeight = false)
        {
            m_ExpandedWidth = expandedWidth;
            m_side = side;
            m_fitContent = fitContentHeight;

            style.flexDirection = FlexDirection.Row;
            style.flexShrink = 0f;
            style.width = expandedWidth;
            style.overflow = Overflow.Hidden;

            // Clickthrough: the pane's own transparent areas (e.g. the empty
            // strip below the fold button when collapsed) must not swallow
            // canvas pan-drag. Ignore skips THIS element's background only —
            // children (gutter children, divider, content) still receive
            // events normally.
            pickingMode = PickingMode.Ignore;

            if (m_fitContent)
            {
                // Opt out of the dock's cross-axis stretch: height becomes
                // the natural content height (capped to the dock via
                // OnFitGeometry below), pinned to the top.
                style.alignSelf = Align.FlexStart;
                RegisterCallback<GeometryChangedEvent>(OnFitGeometry);
            }

            // Gutter: always visible; holds the chevron toggle, and the
            // rotated title when collapsed. pickingMode.Ignore: the empty
            // gutter area must not intercept — fall through to the canvas
            // below (pan by drag). Children (grip, button, label) keep
            // their own picking.
            m_gutter = new VisualElement
            {
                pickingMode = PickingMode.Ignore,
                style =
                {
                    width = k_GutterWidth, flexGrow = 0f, flexShrink = 0f,
                    flexDirection = FlexDirection.Column,
                    alignItems = Align.Center,
                }
            };

            // Grip handle: plate spans the full gutter and sits flush
            // against the divider; the button is anchored to the plate's
            // right edge (0.5px inset) so it reads as mounted on the
            // separator, with symmetric side insets via the plate width.
            m_grip = new VisualElement
            {
                style =
                {
                    width = k_GutterWidth,
                    height = k_ToggleSize + 12f, // 6px plate margin above+below
                    flexGrow = 0f, flexShrink = 0f,
                    alignSelf = Align.FlexEnd,
                    marginBottom = 6f,
                    backgroundColor = k_DividerColor
                }
            };

            m_toggleButton = new Button()
            {
                text = "▶",
                // Purely an icon: the grip is the click target (see below),
                // so the button must not handle clicks (a bubbling click
                // would toggle twice otherwise).
                focusable = false,
                style = { width = k_ToggleSize, height = k_ToggleSize }
            };
            m_toggleButton.pickingMode = PickingMode.Ignore;
            // Strip the default button chrome entirely — the grip plate is
            // the visual background.
            var bs = m_toggleButton.style;
            bs.backgroundColor = new Color(0f, 0f, 0f, 0f);
            bs.borderTopWidth = 0f; bs.borderBottomWidth = 0f;
            bs.borderLeftWidth = 0f; bs.borderRightWidth = 0f;
            bs.borderTopLeftRadius = 0f; bs.borderTopRightRadius = 0f;
            bs.borderBottomLeftRadius = 0f; bs.borderBottomRightRadius = 0f;
            bs.marginTop = 6f; bs.marginBottom = 0f;
            bs.marginLeft = 0f; bs.marginRight = 0f;
            bs.paddingLeft = 0f; bs.paddingRight = 0f;
            // Normal flow inside the plate, horizontally centered (6px top
            // margin centers it in the 30px plate).
            bs.alignSelf = Align.Center;
            m_grip.Add(m_toggleButton);

            // The grip plate is the click target for expand/collapse — the
            // whole 24×30 plate, not just the icon. Clickable (not Button)
            // so hover/fade behavior stays unified; child events (icon)
            // bubble here and count as one click (icon is Ignore anyway).
            m_grip.AddManipulator(new Clickable(ToggleExpanded));
            m_grip.tooltip = "Collapse / expand pane";
            m_gutter.Add(m_grip);

            m_titleLabel = new Label(title);
            // Vertical title under the grip. Key fact: a rotated run always
            // paints centered on its box center — so the box must be
            // auto-sized to the text (NOT a tall top→bottom box, which put
            // the run mid-window no matter the TextAnchor). The rotated
            // strip then gets shifted into place in OnTitleGeometry once
            // the text is measured.
            var ls = m_titleLabel.style;
            ls.position = Position.Absolute;
            ls.top = (k_ToggleSize + 12f) + 6f; // grip plate + its margin + gap
            ls.left = 0f;
            ls.whiteSpace = WhiteSpace.NoWrap;
            // Right pane: −90° reads bottom-up (matches gutter-on-left);
            // Left pane: +90° reads top-down. Both keep the rotated strip
            // centered on the auto-sized box, so the translate fixup below
            // is identical.
            ls.rotate = new Rotate(new Angle(IsLeft ? 90f : -90f));
            ls.display = DisplayStyle.None;
            // Pass-through: never intercept (pan target under the text),
            // purely decorative.
            m_titleLabel.pickingMode = PickingMode.Ignore;
            m_titleLabel.RegisterCallback<GeometryChangedEvent>(OnTitleGeometry);
            m_gutter.Add(m_titleLabel);

            // Divider: full-height strip between gutter and content; drag to
            // resize the pane.
            m_separator = new VisualElement
            {
                style =
                {
                    width = k_DividerWidth, flexGrow = 0f, flexShrink = 0f,
                    alignSelf = Align.Stretch,
                    backgroundColor = k_DividerColor
                }
            };
            m_separator.RegisterCallback<MouseDownEvent>(OnSeparatorDown);
            m_separator.RegisterCallback<MouseMoveEvent>(OnSeparatorMove);
            m_separator.RegisterCallback<MouseUpEvent>(OnSeparatorUp);
            m_separator.RegisterCallback<MouseCaptureOutEvent>(OnSeparatorUp);

            // Hover affordance: grip plate + divider lighten as ONE unit
            // (shared fade, hierarchy-aware events). Registered on the
            // actual shapes only — plate (incl. chevron via bubbling) and
            // strip — so the empty gutter neither highlights nor blocks.
            m_grip.RegisterCallback<MouseEnterEvent>(OnHandleEnter);
            m_grip.RegisterCallback<MouseLeaveEvent>(OnHandleLeave);
            m_separator.RegisterCallback<MouseEnterEvent>(OnHandleEnter);
            m_separator.RegisterCallback<MouseLeaveEvent>(OnHandleLeave);

            // Content. flexGrow=1 fills the pane width (row main axis is
            // horizontal). Vertical fit-content comes from the root's
            // auto-height + maxHeight cap, NOT from here.
            m_content = new ScrollView(ScrollViewMode.Vertical)
            {
                style = { flexGrow = 1f }
            };

            // Scroller tracks: the default theme leaves them transparent,
            // which reads as a hole over the canvas. Give both tracks a
            // quiet background (dragger keeps its theme style).
            var trackColor = new Color(0.12f, 0.12f, 0.12f, 1f);
            m_content.verticalScroller.style.backgroundColor = trackColor;
            m_content.horizontalScroller.style.backgroundColor = trackColor;

            // Row order mirrors the docking side: the gutter always faces the
            // canvas, the content always faces the window edge. Set last —
            // every child must exist first.
            if (IsLeft)
            {
                Add(m_content);
                Add(m_separator);
                Add(m_gutter);
            }
            else
            {
                Add(m_gutter);
                Add(m_separator);
                Add(m_content);
            }
        }

        // -- Fit-content height ---------------------------------------------

        // Cap pane + content to the dock height so tall content scrolls
        // instead of overflowing; short content keeps its natural height.
        // Re-evaluated on every layout pass (dock can resize).
        private void OnFitGeometry(GeometryChangedEvent evt)
        {
            float dockH = parent != null ? parent.layout.height : float.MaxValue;
            if (dockH <= 0f)
                return;
            style.maxHeight = dockH;
            m_content.style.maxHeight = dockH;
        }

        // -- Hover: gutter + divider fade as one unit -----------------------

        private void OnHandleEnter(MouseEnterEvent evt)
        {
            if (m_hovered)
                return;
            m_hovered = true;
            FadeHandleBackground(k_HoverColor);
            evt.StopPropagation();
        }

        private void OnHandleLeave(MouseLeaveEvent evt)
        {
            // MouseLeaveEvent carries no relatedTarget; pick whatever is now
            // under the pointer. Crossing gutter ↔ divider stays inside the
            // handle unit — only fade out when leaving both entirely.
            var next = panel != null ? panel.Pick(evt.mousePosition) : null;
            if (next != null && (m_gutter.Contains(next) || m_separator.Contains(next)))
                return;

            m_hovered = false;
            FadeHandleBackground(k_DividerColor);
            evt.StopPropagation();
        }

        // Animate both surfaces from their CURRENT color (resolvedStyle is
        // live mid-fade) so rapid retargeting stays smooth. Called only
        // from pointer events, i.e. always panel-attached.
        private void FadeHandleBackground(Color target)
        {
            m_grip.experimental.animation
                .Start(new StyleValues { backgroundColor = m_grip.resolvedStyle.backgroundColor },
                    new StyleValues { backgroundColor = target }, k_AnimationMs)
                .Ease(EasyInOut());
            m_separator.experimental.animation
                .Start(new StyleValues { backgroundColor = m_separator.resolvedStyle.backgroundColor },
                    new StyleValues { backgroundColor = target }, k_AnimationMs)
                .Ease(EasyInOut());
        }

        // Auto-sized box (textLen × lineHeight) sits at top=42. Rotating it
        // −90° turns the run into a vertical strip centered on the box
        // center — half of it above the grip. Translate (applied after
        // rotation, in parent space) shifts the strip so its TOP lands at
        // the box top (just below the grip): dx centers it in the gutter,
        // dy = (textLen − lineHeight) / 2 drops it by half the run length.
        // Translate doesn't affect layout, so no geometry-event loop.
        private void OnTitleGeometry(GeometryChangedEvent evt)
        {
            Rect box = m_titleLabel.layout;
            if (box.width <= 0f)
                return;

            float dx = k_GutterWidth * 0.5f - box.width * 0.5f;
            float dy = (box.width - box.height) * 0.5f;
            m_titleLabel.style.translate = new Translate(dx, dy);
        }

        /// <summary>Expand or collapse (animated). Collapsed leaves the
        /// gutter (chevron + title) visible; expanded restores the last
        /// dragged width.</summary>
        public void SetExpanded(bool expanded, bool animate = true)
        {
            if (m_expanded == expanded)
                return;

            m_expanded = expanded;
            float target = expanded ? m_ExpandedWidth : k_GutterWidth;

            // Swap visuals at animation START, not completion: the width
            // tween then reveals/hides real content (overflow Hidden clips
            // it mid-slide). Completion only finalizes the collapsed state.
            m_separator.style.display = DisplayStyle.Flex;
            m_content.style.display = DisplayStyle.Flex;
            m_titleLabel.style.display = expanded ? DisplayStyle.None : DisplayStyle.Flex;
            m_toggleButton.text = expanded
                ? (IsLeft ? "◀" : "▶")
                : (IsLeft ? "▶" : "◀");

            if (animate)
            {
                this.experimental.animation
                    .Start(new StyleValues { width = style.width.value.value },
                        new StyleValues { width = target }, k_AnimationMs)
                    .Ease(EasyInOut())
                    .OnCompleted(OnTransitionDone);
            }
            else
            {
                style.width = target;
                OnTransitionDone();
            }
        }

        public void ToggleExpanded()
        {
            SetExpanded(!m_expanded);
        }

        private void OnTransitionDone()
        {
            // Collapsed final state: drop the content row so nothing paints
            // inside the 24px gutter strip.
            if (!m_expanded)
            {
                m_separator.style.display = DisplayStyle.None;
                m_content.style.display = DisplayStyle.None;
            }

            if (m_expanded)
                style.width = m_ExpandedWidth;

            ExpandedChanged?.Invoke();
        }

        // -- Drag-to-resize ------------------------------------------------

        private bool m_dragging;
        private float m_dragStartWidth;
        private float m_dragStartMouseX;

        private void OnSeparatorDown(MouseDownEvent evt)
        {
            m_dragging = true;
            m_dragStartWidth = m_ExpandedWidth;
            m_dragStartMouseX = evt.mousePosition.x;
            m_separator.CaptureMouse();
            evt.StopPropagation();
        }

        /// <summary>Animation duration for expand/collapse, in ms.</summary>
        public int AnimationMs
        {
            get { return k_AnimationMs; }
        }

        private void OnSeparatorMove(MouseMoveEvent evt)
        {
            if (!m_dragging)
                return;

            // Relative delta from the grab point: the pane widens when the
            // divider drags AWAY from the window edge — left on a Right pane
            // (mouseX decreases), right on a Left pane (mouseX increases).
            // Using the width at MouseDown as the base avoids the jump an
            // absolute reconstruction would produce — the grab point sits
            // mid-divider, not at the pane's outer edge. Max adapts to the
            // parent's current width so the pane can never overflow.
            float maxWidth = k_MaxWidth;
            if (parent != null)
                maxWidth = Mathf.Min(maxWidth, parent.layout.width - k_DividerWidth);
            maxWidth = Mathf.Max(maxWidth, k_MinWidth);

            float delta = evt.mousePosition.x - m_dragStartMouseX;
            if (IsLeft)
                delta = -delta;
            float width = Mathf.Clamp(
                m_dragStartWidth - delta,
                k_MinWidth, maxWidth);

            m_ExpandedWidth = width;
            style.width = width;
            evt.StopPropagation();
        }

        private void OnSeparatorUp(EventBase evt)
        {
            if (!m_dragging)
                return;

            m_dragging = false;
            m_separator.ReleaseMouse();
        }

        // Quadratic ease-in-out, matching EATK's feel without its dependency.
        private static Func<float, float> EasyInOut()
        {
            return t => t < 0.5f ? 2f * t * t : 1f - Mathf.Pow(-2f * t + 2f, 2f) / 2f;
        }
    }
}
#endif