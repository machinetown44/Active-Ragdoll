// ----------------------------------------------------------------------------
// -- MxM Wrapper — census data panel: filterable list of all event markers  --
// -- in the selected preprocess dataset. Fires EntrySelected on click.      --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

namespace MxMWrapper
{
    public class CensusDataPanel : CensusPanel
    {
        public const float k_Width = 320f;

        /// <summary>User picked an event row.</summary>
        public event System.Action<CensusEntry> EntrySelected;

        private readonly List<CensusEntry> m_entries = new List<CensusEntry>();
        private readonly List<CensusEntry> m_visible = new List<CensusEntry>();
        private readonly List<string> m_categories = new List<string>();

        private TextField m_search;
        private DropdownField m_category;
        private ListView m_list;
        private Label m_countLabel;

        public CensusDataPanel() : base("Event Census", k_Width, 520f)
        {
            var toolbar = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    paddingLeft = 6f, paddingRight = 6f, paddingTop = 4f, paddingBottom = 4f
                }
            };

            m_search = new TextField
            {
                style = { flexGrow = 1f, marginRight = 4f }
            };
            m_search.RegisterValueChangedCallback(_ => RebuildVisible());
            toolbar.Add(m_search);

            m_category = new DropdownField("All") { style = { width = 110f } };
            m_category.RegisterValueChangedCallback(_ => RebuildVisible());
            toolbar.Add(m_category);

            Content.Add(toolbar);

            m_countLabel = new Label("0 events")
            {
                style =
                {
                    paddingLeft = 8f, paddingBottom = 2f,
                    color = new Color(0.55f, 0.55f, 0.55f),
                    fontSize = 10f
                }
            };
            Content.Add(m_countLabel);

            m_list = new ListView
            {
                style = { flexGrow = 1f },
                virtualizationMethod = CollectionVirtualizationMethod.DynamicHeight
            };
            m_list.makeItem = MakeRow;
            m_list.bindItem = BindRow;
            m_list.selectionChanged += items =>
            {
                foreach (var e in items)
                {
                    EntrySelected?.Invoke(e as CensusEntry);
                    break;
                }
            };
            Content.Add(m_list);
        }

        /// <summary>Rebuild the census from a preprocess dataset. Call on
        /// dataset change or after edits that touch markers.</summary>
        public void Refresh(MxMEditor.MxMPreProcessData data)
        {
            CensusModel.Collect(data, m_entries);
            CensusModel.CollectCategories(data, m_categories);

            // Source counts make an empty list diagnosable: are there no
            // markers in the data, or no marker carriers at all?
            int comps = 0, blends = 0, modules = data?.AnimationModules?.Count ?? 0;
            if (data?.CompositeCategories != null)
                foreach (var c in data.CompositeCategories)
                    comps += c?.Composites?.Count ?? 0;
            blends = data?.BlendSpaces?.Count ?? 0;

            var options = new List<string> { "All" };
            options.AddRange(m_categories);
            m_category.choices = options;
            if (!options.Contains(m_category.value))
                m_category.SetValueWithoutNotify("All");

            RebuildVisible();
            m_countLabel.text =
                $"{m_visible.Count} / {m_entries.Count} events  ·  {comps} composites, {blends} blend spaces, {modules} modules";
        }

        private void RebuildVisible()
        {
            m_visible.Clear();

            string filter = string.IsNullOrEmpty(m_search.value)
                ? null : m_search.value.ToLowerInvariant();
            string cat = m_category.value;
            bool filterCategory = !string.IsNullOrEmpty(cat) && cat != "All";

            for (int i = 0; i < m_entries.Count; ++i)
            {
                var e = m_entries[i];
                if (filter != null &&
                    !e.EventName.ToLowerInvariant().Contains(filter) &&
                    !e.OwnerName.ToLowerInvariant().Contains(filter))
                    continue;
                if (filterCategory && e.Category != cat)
                    continue;
                m_visible.Add(e);
            }

            m_list.itemsSource = m_visible;
            m_list.Rebuild();
            m_countLabel.text = $"{m_visible.Count} / {m_entries.Count} events";
        }

        // -- Rows ---------------------------------------------------------------

        private static VisualElement MakeRow()
        {
            var row = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Column,
                    paddingLeft = 8f, paddingRight = 8f, paddingTop = 3f, paddingBottom = 3f
                }
            };

            var name = new Label { style = { unityFontStyleAndWeight = FontStyle.Bold } };
            var meta = new Label
            {
                style =
                {
                    color = new Color(0.6f, 0.6f, 0.6f),
                    fontSize = 10f
                }
            };
            row.Add(name);
            row.Add(meta);

            row.userData = (name, meta);
            return row;
        }

        private void BindRow(VisualElement row, int index)
        {
            // ListView does NOT set dataSource on row elements — read the
            // item from itemsSource by index.
            var e = m_list.itemsSource[index] as CensusEntry;
            if (e == null)
                return;

            var (name, meta) = ((Label, Label))row.userData;
            name.text = e.EventName;
            meta.text = $"{e.Marker.EventTime:0.00}s · {e.OwnerName}";
        }
    }
}
#endif