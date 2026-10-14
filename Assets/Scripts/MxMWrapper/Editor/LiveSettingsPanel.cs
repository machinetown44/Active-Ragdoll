// ----------------------------------------------------------------------------
// -- MxM Wrapper — live search session settings: search cadence, blend-in,  --
// -- trajectory speed, timeline duration. Edits write into the shared       --
// -- LiveSessionModel and invalidate it (runner + views react per tick).    --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using UnityEngine;
using UnityEngine.UIElements;

namespace MxMWrapper
{
    public class LiveSettingsPanel : CensusPanel
    {
        public const float k_Width = 200f;
        public const float k_Height = 150f;

        private LiveSessionModel m_model;

        private readonly SliderInt m_interval;
        private readonly Slider m_blend;
        private readonly Slider m_speed;
        private readonly FloatField m_duration;

        public LiveSettingsPanel() : base("Live Settings", k_Width, k_Height)
        {
            var body = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Column,
                    paddingTop = 6f, paddingLeft = 8f, paddingRight = 8f,
                    justifyContent = Justify.SpaceBetween,
                    flexGrow = 1f
                }
            };
            Content.Add(body);

            m_interval = AddSliderInt(body, "Search interval (f)", 1, 30);
            m_blend = AddSlider(body, "Blend-in (s)", 0f, 0.5f);
            m_speed = AddSlider(body, "Trajectory speed", 0.1f, 10f);
            m_duration = AddFloatField(body, "Duration (s)", 1f, 60f);

            m_interval.RegisterValueChangedCallback(evt =>
                Apply(s => s.SearchIntervalFrames = evt.newValue));
            m_blend.RegisterValueChangedCallback(evt =>
                Apply(s => s.BlendInTime = evt.newValue));
            m_speed.RegisterValueChangedCallback(evt =>
                Apply(s => s.Speed = evt.newValue));
            m_duration.RegisterValueChangedCallback(evt =>
                Apply(s => s.Duration = Mathf.Max(1f, evt.newValue)));
        }

        /// <summary>Bind the session (new runner / reload). Values are pushed
        /// without events — edits flow through Apply only.</summary>
        public void SetModel(LiveSessionModel model)
        {
            m_model = model;
            if (model == null)
                return;
            var s = model.Settings;
            m_interval.SetValueWithoutNotify(s.SearchIntervalFrames);
            m_blend.SetValueWithoutNotify(s.BlendInTime);
            m_speed.SetValueWithoutNotify(s.Speed);
            m_duration.SetValueWithoutNotify(s.Duration);
        }

        private void Apply(System.Action<LiveSessionSettings> mutator)
        {
            if (m_model == null)
                return;
            mutator(m_model.Settings);
            m_model.MarkInvalidated();
        }

        private static Slider AddSlider(VisualElement body, string label,
            float min, float max)
        {
            var s = new Slider(label, min, max) { showInputField = true };
            body.Add(s);
            return s;
        }

        private static SliderInt AddSliderInt(VisualElement body, string label,
            int min, int max)
        {
            var s = new SliderInt(label, min, max) { showInputField = true };
            body.Add(s);
            return s;
        }

        private static FloatField AddFloatField(VisualElement body, string label,
            float min, float max)
        {
            var row = new VisualElement
            {
                style = { flexDirection = FlexDirection.Row, alignItems = Align.Center }
            };
            row.Add(new Label(label) { style = { flexGrow = 1f } });
            var f = new FloatField { style = { width = 60f } };
            f.SetValueWithoutNotify(min);
            row.Add(f);
            body.Add(row);
            return f;
        }
    }
}
#endif
