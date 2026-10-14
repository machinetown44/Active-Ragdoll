// ----------------------------------------------------------------------------
// -- Element Toolkit (id.elemtk) — generic editor wrapper primitives        --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using UnityEditor;

namespace id.elemtk.Framework
{
    /// <summary>
    /// Generic change detection for editor assets, for workflow windows that
    /// react to data mutated by other views (stock inspectors, our own
    /// controls, undo/redo).
    ///
    /// Two signals, both observation-only (no target-framework modification):
    /// 1. Polling: a cheap version hash of the watched object(s) compared
    ///    every interval via EditorApplication.update.
    /// 2. Undo: postprocessModifications / undoRedoPerformed fire on
    ///    serialized edits made through inspectors.
    ///
    /// Register with Add(target, hashGetter, onChanged); Unregister in
    /// OnDisable. One static instance per window is typical; or use the
    /// instance-based API.
    /// </summary>
    public class ChangeMonitor : IDisposable
    {
        private struct Watch
        {
            public UnityEngine.Object Target;
            public Func<int> HashGetter;
            public Action OnChanged;
            public int LastHash;
        }

        private readonly List<Watch> m_watches = new List<Watch>();
        private double m_nextPollTime;
        private double m_pollIntervalSeconds = 0.25;
        private bool m_registered;

        /// <summary>Seconds between hash polls (default 0.25).</summary>
        public double PollIntervalSeconds
        {
            get { return m_pollIntervalSeconds; }
            set { m_pollIntervalSeconds = value; }
        }

        /// <summary>Watch a target. hashGetter must be cheap (counts, sizes,
        /// serialized hashes) — it runs on the editor update loop. Fires
        /// onChanged whenever the hash changes.</summary>
        public void Add(UnityEngine.Object target, Func<int> hashGetter, Action onChanged)
        {
            if (target == null || hashGetter == null)
                return;

            var watch = new Watch
            {
                Target = target,
                HashGetter = hashGetter,
                OnChanged = onChanged,
                LastHash = hashGetter()
            };

            for (int i = 0; i < m_watches.Count; i++)
            {
                if (m_watches[i].Target == target)
                {
                    m_watches[i] = watch;
                    return;
                }
            }

            m_watches.Add(watch);
            Register();
        }

        public void Remove(UnityEngine.Object target)
        {
            for (int i = 0; i < m_watches.Count; i++)
            {
                if (m_watches[i].Target == target)
                {
                    m_watches.RemoveAt(i);
                    break;
                }
            }

            if (m_watches.Count == 0)
                Unregister();
        }

        private void Register()
        {
            if (m_registered)
                return;

            EditorApplication.update += Poll;
            Undo.postprocessModifications += OnPostprocessModifications;
            Undo.undoRedoPerformed += OnUndoRedo;
            m_registered = true;
        }

        private void Unregister()
        {
            if (!m_registered)
                return;

            EditorApplication.update -= Poll;
            Undo.postprocessModifications -= OnPostprocessModifications;
            Undo.undoRedoPerformed -= OnUndoRedo;
            m_registered = false;
        }

        private void Poll()
        {
            double now = EditorApplication.timeSinceStartup;
            if (now < m_nextPollTime)
                return;

            m_nextPollTime = now + m_pollIntervalSeconds;

            for (int i = m_watches.Count - 1; i >= 0; i--)
            {
                var w = m_watches[i];

                // Target destroyed (asset deleted): report change, drop watch.
                if (w.Target == null)
                {
                    m_watches.RemoveAt(i);
                    w.OnChanged?.Invoke();
                    if (m_watches.Count == 0) Unregister();
                    continue;
                }

                int hash = w.HashGetter();
                if (hash != w.LastHash)
                {
                    w.LastHash = hash;
                    w.OnChanged?.Invoke();
                }
            }
        }

        private UndoPropertyModification[] OnPostprocessModifications(UndoPropertyModification[] modifications)
        {
            // Serialized edits through any inspector — poll will pick up the
            // hash change on the next tick; no extra work needed here. Hook
            // exists so subclasses can intercept immediately.
            return modifications;
        }

        private void OnUndoRedo()
        {
            // Force an immediate re-hash after undo/redo (hashes change
            // asynchronously; this makes undo snappy).
            Poll();
            m_nextPollTime = EditorApplication.timeSinceStartup + m_pollIntervalSeconds;
        }

        public void Dispose()
        {
            m_watches.Clear();
            Unregister();
        }
    }
}
#endif