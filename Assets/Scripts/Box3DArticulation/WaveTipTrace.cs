using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>Lightweight authoring overlay: draws each limb's tip path over
    /// one wave period by sampling TipWorld into a per-limb ring buffer. The
    /// ring covers exactly the last 1/freq seconds, so the visible polyline is
    /// the current CPG's full closed tip path (plus contact/gravity effects -
    /// it traces the real tip, not the analytic servo target). Gizmo-only:
    /// no LineRenderer/material, zero per-frame allocation after init, and it
    /// does not touch the probe. Attach to the PinnedBipedDemo (probes
    /// auto-found); toggles per limb pair in the Inspector.</summary>
    public sealed class WaveTipTrace : MonoBehaviour
    {
        [Header("Probes (auto-found by name if empty)")]
        [SerializeField] private BackboneWaveProbe leftLeg, rightLeg, leftArm, rightArm;

        [Header("Draw toggles")]
        [SerializeField] private bool drawLegs = true;
        [SerializeField] private bool drawArms = true;

        [Header("Look")]
        [SerializeField] private Color legColor = new Color(0.2f, 0.9f, 1f);
        [SerializeField] private Color armColor = new Color(1f, 0.6f, 0.2f);
        [SerializeField, Min(0.002f), Tooltip("Sampling interval (s). 1/60 = every physics step.")]
        private float sampleInterval = 1f / 60f;

        // Ring buffers, one per limb, allocated on demand and re-allocated
        // only when the period (from live freq) changes size class.
        private class Trace
        {
            public BackboneWaveProbe probe;
            public Vector3[] pts;
            public int count, head;
            public float acc, interval, period;
        }
        private Trace m_legL, m_legR, m_armL, m_armR;

        private void Start()
        {
            if (leftLeg == null) leftLeg = FindChild("LeftLeg");
            if (rightLeg == null) rightLeg = FindChild("RightLeg");
            if (leftArm == null) leftArm = FindChild("LeftArm");
            if (rightArm == null) rightArm = FindChild("RightArm");
        }

        private BackboneWaveProbe FindChild(string name)
        {
            var t = transform.Find(name);
            return t != null ? t.GetComponent<BackboneWaveProbe>() : null;
        }

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            m_legL = Sample(m_legL, dt);
            m_legR = Sample(m_legR, dt);
            m_armL = Sample(m_armL, dt);
            m_armR = Sample(m_armR, dt);
        }

        // Realloc only when the required sample count changes by >25% - the
        // freq knob moves during authoring, but rarely by small fractions.
        // Instance method: reads the sampleInterval setting.
        private Trace Sample(Trace tr, float dt)
        {
            if (tr == null || tr.probe == null || tr.probe.TipWorld.Equals(float3.zero)) return tr;
            float freq = tr.probe.GetRuntimeParams().freqHz;
            float period = 1f / math.max(freq, 0.2f);
            int need = Mathf.Clamp(Mathf.CeilToInt(period / dt), 16, 2048);

            if (tr.pts == null || Mathf.Abs(tr.interval - sampleInterval) > 1e-4f ||
                need > tr.pts.Length * 1.25f || need < tr.pts.Length * 0.75f)
            {
                tr.pts = new Vector3[need];
                tr.count = 0; tr.head = 0;
                tr.interval = sampleInterval; tr.period = period;
            }

            tr.acc += dt;
            if (tr.acc < tr.interval) return tr;
            tr.acc -= tr.interval;

            float3 tip = tr.probe.TipWorld;
            tr.pts[tr.head] = new Vector3(tip.x, tip.y, tip.z);
            tr.head = (tr.head + 1) % tr.pts.Length;
            tr.count = Mathf.Min(tr.count + 1, tr.pts.Length);
            return tr;
        }

        private void OnDrawGizmos()
        {
            if (!Application.isPlaying) return;
            // Right-side limbs drawn dimmer so L/R is distinguishable without a legend.
            Color legsDim = new Color(legColor.r * 0.55f, legColor.g * 0.55f, legColor.b);
            Color armsDim = new Color(armColor.r * 0.55f, armColor.g * 0.55f, armColor.b);
            if (drawLegs)
            {
                Draw(m_legL, legColor);
                Draw(m_legR, legsDim);
            }
            if (drawArms)
            {
                Draw(m_armL, armColor);
                Draw(m_armR, armsDim);
            }
        }

        private static void Draw(Trace tr, Color c)
        {
            if (tr == null || tr.count < 3) return;
            Gizmos.color = c;
            Vector3 prev = tr.pts[(tr.head - 1 + tr.pts.Length) % tr.pts.Length];
            int idx = tr.head;
            for (int i = 0; i < tr.count; i++)
            {
                Vector3 cur = tr.pts[idx];
                if (i > 0) Gizmos.DrawLine(prev, cur);
                prev = cur;
                idx = (idx + 1) % tr.pts.Length;
            }
            // close the loop: the path over one period is cyclic
            if (tr.count == tr.pts.Length) Gizmos.DrawLine(prev, tr.pts[tr.head]);
        }
    }
}