// ----------------------------------------------------------------------------
// -- MxM Wrapper — live search probe: runs the real search pipeline for a   --
// -- straight and a bent trajectory and logs chosen pose transitions.      --
// -- Diagnostic for "search ignores turns". Temporary tool.               --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Text;
using MxM;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;

namespace MxMWrapper
{
    public class LiveSearchProbeTests
    {
        private const string k_DataPath =
            "Assets/Plugins/MotionMatching/Demo/Data/MotionMatching/MocapAnimData.asset";

        [Test]
        public void Probe_SearchReactsToTrajectoryBend()
        {
            var data = AssetDatabase.LoadAssetAtPath<MxMAnimData>(k_DataPath);
            Assert.NotNull(data, k_DataPath);
            Assert.Greater(data.PosePredictionTimes != null
                ? data.PosePredictionTimes.Length : 0, 0, "PosePredictionTimes empty");

            // Control: straight +Z walk.
            var model = new LiveSessionModel();
            model.SeedDefault();
            string straight = Run(data, model, 4f, "STRAIGHT");

            // Probe: hard left bend over the first two metres.
            model = new LiveSessionModel();
            model.SeedDefault();
            model.Points[1].Position = new Vector2(0.4f, 1.2f);
            model.Points[2].Position = new Vector2(-2.0f, 2.2f);
            model.MarkInvalidated();
            string bent = Run(data, model, 4f, "BENT-LEFT");

            Debug.Log($"PROBE STRAIGHT:\n{straight}\n\nPROBE BENT-LEFT:\n{bent}");
        }

        [Test]
        public void Probe_DecisionDump()
        {
            var data = AssetDatabase.LoadAssetAtPath<MxMAnimData>(k_DataPath);
            Assert.NotNull(data, k_DataPath);

            var model = new LiveSessionModel();
            model.SeedDefault();
            model.Points[1].Position = new Vector2(0.4f, 1.2f);
            model.Points[2].Position = new Vector2(-2.0f, 2.2f);
            model.MarkInvalidated();

            var runner = new LiveSearchRunner(data, model) { DebugEnabled = true };
            const float dt = 1f / 30f;
            string dump1 = null, dump2 = null;
            for (float t = 0f; t <= 2.5f; t += dt)
            {
                runner.Step(dt, t);
                if (dump1 == null && runner.DebugDump != null)
                    dump1 = $"t={t:0.00}\n{runner.DebugDump}";
                if (t >= 2.0f && dump2 == null && runner.DebugDump != null)
                    dump2 = $"t={t:0.00}\n{runner.DebugDump}";
            }
            runner.Dispose();
            Debug.Log($"DUMP FIRST DECISION:\n{dump1}\n\nDUMP LATER:\n{dump2}");
        }

        private static string Run(MxMAnimData data, LiveSessionModel model,
            float duration, string label)
        {
            var runner = new LiveSearchRunner(data, model);
            var sb = new StringBuilder();
            string last = null;
            const float dt = 1f / 30f;
            for (float t = 0f; t <= duration; t += dt)
            {
                runner.Step(dt, t);
                if (runner.PoseInfo != last)
                {
                    sb.AppendLine($"  t={t:0.00}  -> {runner.PoseInfo}");
                    last = runner.PoseInfo;
                }
            }
            runner.Dispose();
            return sb.ToString();
        }
    }
}
#endif
