// ----------------------------------------------------------------------------
// -- MxM Wrapper — shared panel shell for the canvas census panels:         --
// -- titled, fixed-size self-contained blocks placed on the PannableCanvas. --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using UnityEngine;
using UnityEngine.UIElements;

namespace MxMWrapper
{
    /// <summary>A titled panel for the workflow canvas: colored header bar
    /// with a caption and room for header controls, content area below.
    /// Panels are absolutely positioned on the canvas (see
    /// PannableCanvas.AddNode) and fully self-contained.</summary>
    public class CensusPanel : VisualElement
    {
        /// <summary>Header row: hosts the title label; subclasses may append
        /// controls (dropdowns, toggles) here.</summary>
        public readonly VisualElement Header;

        /// <summary>Scrollable/flexible content area under the header.</summary>
        public readonly VisualElement Content;

        /// <summary>Protected for panel subclasses; WrapTree uses the factory
        /// below, so external code never constructs the shell directly.</summary>
        public CensusPanel(string title, float width, float height)
        {
            style.position = Position.Absolute;
            style.width = width;
            style.height = height;
            style.backgroundColor = new Color(0.16f, 0.16f, 0.16f, 0.96f);
            style.borderTopLeftRadius = 6f; style.borderTopRightRadius = 6f;
            style.borderBottomLeftRadius = 6f; style.borderBottomRightRadius = 6f;
            style.overflow = Overflow.Hidden;

            Header = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    alignItems = Align.Center,
                    height = 24f,
                    backgroundColor = new Color(0.09f, 0.09f, 0.09f),
                    marginBottom = 2f
                }
            };
            var label = new Label(title)
            {
                style =
                {
                    unityTextAlign = TextAnchor.MiddleLeft,
                    paddingLeft = 10f,
                    color = new Color(0.85f, 0.85f, 0.85f)
                }
            };
            label.style.unityFontStyleAndWeight = FontStyle.Bold;
            Header.Add(label);
            Add(Header);

            Content = new VisualElement
            {
                style = { flexGrow = 1f }
            };
            Add(Content);
        }
    }
}
#endif