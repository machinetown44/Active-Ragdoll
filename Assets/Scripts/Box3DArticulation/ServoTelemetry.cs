using System.Reflection;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Joint-space telemetry for tuning the velocity-motor servo. Samples
    /// relT / relA / |wRel| per physics step into ring buffers via reflection
    /// (diagnostic tool: keeps the driver's public surface clean). Dump
    /// returns per-joint stats plus raw waveforms for oscillation analysis.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class ServoTelemetry : MonoBehaviour
    {
        private const float SampleInterval = 0.01f; // 100 Hz
        private const int Capacity = 2400;          // 24 s window

        private struct JointRecord
        {
            public string name;
            public bool hasJoint;
            public float[] relT;   // deg, unsigned angle of target
            public float[] relA;   // deg, unsigned angle of measured
            public float[] wRel;   // rad/s, |relative angular velocity|
            public int head;
            public int count;
        }

        private HumanoidMotionDriver m_driver;
        private object[] m_nodes;
        private object[] m_chain;
        private JointRecord[] m_joints;
        private FieldInfo m_fFrameA, m_fFrameB, m_fParentBody, m_fBody;
        private float m_nextSample;
        private int m_sampleCount;

        private void OnEnable() => Resolve();

        private void Update() => Resolve();

        private void Resolve()
        {
            if (m_driver != null && m_nodes != null) return;
            m_driver = FindAnyObjectByType<HumanoidMotionDriver>();
            if (m_driver == null) return;
            var t = m_driver.GetType();
            const BindingFlags BF = BindingFlags.NonPublic | BindingFlags.Instance;
            var nodesF = t.GetField("m_nodes", BF);
            var chainF = t.GetField("m_chainSim", BF);
            if (nodesF == null || chainF == null) return;
            var nodesArr = (System.Array)nodesF.GetValue(m_driver);
            if (nodesArr == null || nodesArr.Length == 0) return;
            m_nodes = new object[nodesArr.Length];
            m_chain = new object[nodesArr.Length];
            var nodes = nodesF.GetValue(m_driver) as System.Array;
            var chain = chainF.GetValue(m_driver) as System.Array;
            if (nodes == null || chain == null) return;
            for (int i = 0; i < nodes.Length; i++) { m_nodes[i] = nodes.GetValue(i); m_chain[i] = chain.GetValue(i); }
            var nt = m_nodes[0].GetType();
            m_fFrameA = nt.GetField("frameARot", BF);
            m_fFrameB = nt.GetField("frameBRot", BF);
            m_fParentBody = nt.GetField("parentBody", BF);
            m_fBody = nt.GetField("body", BF);
            var fHasJoint = nt.GetField("hasJoint", BF);
            var fName = nt.GetField("name", BF);
            m_joints = new JointRecord[m_nodes.Length];
            for (int i = 0; i < m_nodes.Length; i++)
            {
                m_joints[i].hasJoint = fHasJoint != null && (bool)fHasJoint.GetValue(m_nodes[i]);
                m_joints[i].name = fName != null ? (string)fName.GetValue(m_nodes[i]) : i.ToString();
                m_joints[i].relT = new float[Capacity];
                m_joints[i].relA = new float[Capacity];
                m_joints[i].wRel = new float[Capacity];
            }
        }

        private void FixedUpdate()
        {
            if (m_joints == null || m_driver == null) return;
            if (Time.fixedTime < m_nextSample) return;
            m_nextSample = Time.fixedTime + SampleInterval;

            for (int i = 0; i < m_nodes.Length; i++)
            {
                if (!m_joints[i].hasJoint) continue;
                var n = m_nodes[i];
                var c = m_chain[i];
                var relT = (quaternion)(RelTField(c.GetType()) ?? m_relTCache).GetValue(c);
                var fA = (quaternion)m_fFrameA.GetValue(n);
                var fB = (quaternion)m_fFrameB.GetValue(n);
                var pb = (Box3D.Hybrid.Box3DBody)m_fParentBody.GetValue(n);
                var b = (Box3D.Hybrid.Box3DBody)m_fBody.GetValue(n);
                if (pb == null || b == null || b.Body == null || pb.Body == null) continue;

                MotionTransform.JointRelative(pb.Body.GetRotation(), b.Body.GetRotation(), fA, fB, out var relA);
                float3 wRelW = b.Body.GetAngularVelocity() - pb.Body.GetAngularVelocity();

                int h = m_joints[i].head;
                m_joints[i].relT[h] = QuaternionAngleDeg(relT);
                m_joints[i].relA[h] = QuaternionAngleDeg(relA);
                m_joints[i].wRel[h] = math.length(wRelW);
                m_joints[i].head = (h + 1) % Capacity;
                if (m_joints[i].count < Capacity) m_joints[i].count++;
            }
            m_sampleCount++;
        }

        private static FieldInfo m_relTCache;
        private FieldInfo RelTField(System.Type t)
            => m_relTCache ??= t.GetField("relT", BindingFlags.NonPublic | BindingFlags.Instance);

        private static float QuaternionAngleDeg(quaternion q)
        {
            float d = math.abs(math.dot(q.value, new float4(0f, 0f, 0f, 1f)));
            return 2f * Mathf.Acos(Mathf.Clamp(d, 0f, 1f)) * Mathf.Rad2Deg;
        }

        /// <summary>Stats over the window + raw waveform of one joint.</summary>
        public string Dump(int joint, float lastSeconds)
        {
            if (m_joints == null) return "no data (not bound)";
            var sb = new System.Text.StringBuilder();
            sb.Append("samples=").Append(m_sampleCount).Append('\n');
            for (int i = 0; i < m_joints.Length; i++)
            {
                if (!m_joints[i].hasJoint) continue;
                int n = m_joints[i].count;
                if (n < 2) continue;
                Stats(m_joints[i].relA, n, out float aMin, out float aMax, out float aMean, out float aStd);
                sb.Append(i).Append(' ').Append(m_joints[i].name)
                  .Append(" relA min=").Append(aMin.ToString("F1"))
                  .Append(" max=").Append(aMax.ToString("F1"))
                  .Append(" mean=").Append(aMean.ToString("F1"))
                  .Append(" std=").Append(aStd.ToString("F2"))
                  .Append('\n');
            }
            if (joint < 0 || joint >= m_joints.Length || !m_joints[joint].hasJoint) return sb.ToString();
            int cnt = m_joints[joint].count;
            int take = Mathf.Min(cnt, Mathf.CeilToInt(lastSeconds / SampleInterval));
            sb.Append("wave ").Append(m_joints[joint].name).Append(" (relT|relA|wRel)\n");
            for (int k = take; k >= 1; k--)
            {
                int idx = (m_joints[joint].head - k + Capacity * 2) % Capacity;
                sb.Append(m_joints[joint].relT[idx].ToString("F1")).Append('|')
                  .Append(m_joints[joint].relA[idx].ToString("F1")).Append('|')
                  .Append(m_joints[joint].wRel[idx].ToString("F2")).Append(' ');
            }
            return sb.ToString();
        }

        private static void Stats(float[] buf, int n, out float min, out float max, out float mean, out float std)
        {
            min = float.MaxValue; max = float.MinValue; mean = 0f;
            for (int i = 0; i < n; i++) { float v = buf[i]; if (v < min) min = v; if (v > max) max = v; mean += v; }
            mean /= n;
            float var = 0f;
            for (int i = 0; i < n; i++) { float d = buf[i] - mean; var += d * d; }
            std = Mathf.Sqrt(var / n);
        }

        [ContextMenu("Dump 10s of joint 10")]
        private void DumpJoint10() => Debug.Log(Dump(10, 10f));
    }
}
