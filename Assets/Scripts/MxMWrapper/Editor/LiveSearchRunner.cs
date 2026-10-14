// ----------------------------------------------------------------------------
// -- MxM Wrapper — live search runner: drives MxM's real Burst search jobs  --
// -- (ComputeTrajectoryXXCosts / ComputePoseXXCosts / FindMinima) directly  --
// -- over the baked MxMAnimData with a SYNTHETIC goal built from the live   --
// -- trajectory spline. Mirrors only MxMAnimator's private job-scheduling/  --
// -- packing code (GeneratePoseXJob etc.); the cost jobs are the plugin's,  --
// -- unmodified. Editor-only.                                              --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Collections.Generic;
using MxM;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace MxMWrapper
{
    public class LiveSearchRunner : LiveViewPanel.PoseProbe, System.IDisposable
    {
        private const float k_FavourMultiplier = 2f; // v1: exclusive favour-tag bias
        /// <summary>Same-location tolerance (s), mirrors MxMAnimator's
        /// FinalizePoseSearch winnerAtSameLocation test.</summary>
        private const float k_SameLocationTime = 0.2f;
        /// <summary>Playback speed warp limits (longitudinal error warp,
        /// mirrors MxMAnimator's speedWarpLimits defaults).</summary>
        private const float k_WarpMin = 0.6f;
        private const float k_WarpMax = 1.6f;

        private readonly MxMAnimData m_animData;
        private readonly LiveSessionModel m_model;
        private readonly int m_numJoints;      // MatchBones count (job variant)
        private readonly int m_numPredTimes;   // PosePredictionTimes count (job variant)

        private CalibrationData m_calib;

        // Scratch cost buffers — sized to the ACTIVE tag set's used-pose count.
        private NativeArray<float> m_poseCosts;
        private NativeArray<float> m_trajCosts;
        private NativeArray<int> m_chosenPoseId;
        private ETags m_scratchTags = (ETags)(-1); // sentinel: never matches

        // Playback state.
        private int m_curPoseId = -1;     // database pose record index
        private float m_poseTime;         // clip-local playback time
        private PoseData m_interpolated;  // job-input pose (joints lerp toward next record)
        private int m_frameCount;
        private string m_poseInfo;

        // Pose crossfade (BlendInTime): snapshot of the pose on display when
        // a new decision lands; blends into it over the window.
        private PoseData? m_blendFrom;   // nullable: class is a struct
        private float m_blendElapsed;

        // Decision cache (invalidated by model edits).
        private struct Decision
        {
            public float Time;
            public int PoseId;
        }
        private readonly List<Decision> m_decisions = new List<Decision>();
        private int m_cacheVersion = -1;

        public string PoseInfo { get { return m_poseInfo; } }

        /// <summary>When set, each search records a top-candidates dump in
        /// DebugDump (probe diagnostics; costs nothing when off).</summary>
        public bool DebugEnabled
        {
            get { return m_debugDump; }
            set { m_debugDump = value; }
        }
        private bool m_debugDump;
        public string DebugDump { get; private set; }

        private void BuildDebugDump(MxMNativeAnimData native,
            MxM.TrajectoryPoint[] goal, int chosenPoseId)
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("goal:");
            for (int i = 0; i < goal.Length; ++i)
                sb.AppendFormat("  [{0}] pos ({1:0.00},{2:0.00}) ang {3:0.0}\n",
                    i, goal[i].Position.x, goal[i].Position.z, goal[i].FacingAngle);

            int n = native.UsedPoseIds.Length;
            var idx = new int[n];
            var total = new float[n];
            for (int i = 0; i < n; ++i)
            { idx[i] = i; total[i] = m_poseCosts[i] + m_trajCosts[i]; }
            System.Array.Sort(total, idx);

            int chosenUsed = -1;
            for (int i = 0; i < n; ++i)
                if (native.UsedPoseIds[i] == chosenPoseId) { chosenUsed = i; break; }
            sb.AppendFormat("chosen pose {0} (usedIdx {1}): favour {2:0.00}, " +
                "poseCost {3:0.000} + trajCost {4:0.000}\n",
                chosenPoseId, chosenUsed,
                chosenUsed >= 0 ? native.FavourPacked[chosenUsed] : -1f,
                chosenUsed >= 0 ? m_poseCosts[chosenUsed] : -1f,
                chosenUsed >= 0 ? m_trajCosts[chosenUsed] : -1f);

            sb.AppendLine("top5 (poseCost + trajCost):");
            for (int k = 0; k < Mathf.Min(5, n); ++k)
            {
                int pid = native.UsedPoseIds[idx[k]];
                var p = m_animData.Poses[pid];
                sb.AppendFormat("  {0}. pose {1} clip {2} @ {3:0.00}s  " +
                    "{4:0.000} = {5:0.000} + {6:0.000}\n",
                    k + 1, pid, p.AnimId, p.Time,
                    total[k], m_poseCosts[idx[k]], m_trajCosts[idx[k]]);
            }
            DebugDump = sb.ToString();
        }

        /// <summary>Clip-local time of the currently played pose.</summary>
        public float CurrentPoseTime { get { return m_poseTime; } }

        /// <summary>Playback source for the live view's rig sampling: the
        /// chosen pose's clip + current clip-local time.</summary>
        public bool TryGetPlayback(out AnimationClip clip, out float time)
        {
            clip = null;
            time = 0f;
            if (m_curPoseId < 0)
                return false;
            var pd = m_animData.Poses[m_curPoseId];
            clip = m_animData.Clips[pd.PrimaryClipId];
            time = m_poseTime;
            return clip != null;
        }

        /// <summary>Seed the live session's start pose from a clip + time
        /// (the census preview's scrub position): pick the pose record of
        /// that clip nearest the time. Clears the decision cache — the next
        /// play-through continues from here.</summary>
        public void SeedStartPose(AnimationClip clip, float time)
        {
            if (m_animData == null || clip == null)
                return;

            // Pose records of clips are laid out ClipsData[i] range in clip
            // order; find the clip whose AnimationClip matches, then the
            // nearest record inside its pose range.
            int bestPoseId = -1;
            float bestDist = float.MaxValue;
            var clipsData = m_animData.ClipsData;
            for (int i = 0; i < clipsData.Length; ++i)
            {
                if (clipsData[i].ClipId < 0 || clipsData[i].ClipId >= m_animData.Clips.Length)
                    continue;
                if (m_animData.Clips[clipsData[i].ClipId] != clip)
                    continue;
                for (int p = clipsData[i].StartPoseId; p <= clipsData[i].EndPoseId; ++p)
                {
                    float d = Mathf.Abs(m_animData.Poses[p].Time - time);
                    if (d < bestDist)
                    {
                        bestDist = d;
                        bestPoseId = p;
                    }
                }
            }
            if (bestPoseId < 0)
                return;

            m_curPoseId = bestPoseId;
            m_poseTime = m_animData.Poses[bestPoseId].Time;
            BuildInterpolated();
            m_decisions.Clear();
            m_frameCount = 0;
        }

        public LiveSearchRunner(MxMAnimData animData, LiveSessionModel model)
        {
            m_animData = animData;
            m_model = model;
            m_numJoints = animData != null && animData.MatchBones != null
                ? Mathf.Min(animData.MatchBones.Length, 5) : 0;
            m_numPredTimes = animData != null && animData.PosePredictionTimes != null
                ? Mathf.Min(animData.PosePredictionTimes.Length, 6) : 0;

            if (animData != null && animData.CalibrationSets != null &&
                animData.CalibrationSets.Length > 0)
                m_calib = animData.CalibrationSets[0];

            if (animData != null)
            {
                m_interpolated = new PoseData(animData.Poses[0]);
                // Native packed data is runtime-built (MxMAnimator setup does
                // this normally); the baked asset has nothing in-editor until
                // we ask. Ref-counted — balanced in Dispose().
            m_animData.InitializeNativeData();
            m_warned = false;
            }
        }

        public void Reset()
        {
            m_curPoseId = -1;
            m_poseTime = 0f;
            m_frameCount = 0;
            m_blendFrom = null;
            m_blendElapsed = 0f;
            m_decisions.Clear();
            m_cacheVersion = m_model.Version;
        }

        /// <summary>Advance one frame: playback + periodic search decision.
        /// seekTime drives the synthetic goal (root + trajectory).</summary>
        public void Step(float dt, float seekTime)
        {
            if (m_animData == null)
                return;
            if (m_numJoints == 0 || m_numPredTimes == 0 || m_calib == null)
            {
                WarnOnce($"live search disabled: joints={m_numJoints} " +
                    $"predTimes={m_numPredTimes} calib={(m_calib != null)}");
                return;
            }
            if (m_cacheVersion != m_model.Version)
                Reset();

            m_frameCount++;
            m_blendElapsed += dt;

            // Search on the first frame and every SearchIntervalFrames after.
            if (m_curPoseId < 0 ||
                m_frameCount % Mathf.Max(1, m_model.Settings.SearchIntervalFrames) == 0)
                Search(seekTime);

            // Advance clip-local playback of the chosen pose (the clip's
            // own playback speed applies — ClipData, not AnimationClip).
            // Wrap/hold at clip end: looping clips wrap; others hold and let
            // the next search decision pick a continuation.
            if (m_curPoseId >= 0)
            {
                var pd = m_animData.Poses[m_curPoseId];
                var clipData = m_animData.ClipsData[pd.AnimId];

                // Speed warp: scale playback so the clip's root velocity
                // matches the trajectory speed. Without this the chosen clip
                // skates over the spline at whatever pace it was authored
                // (mirrors the animator's longitudinal error warping).
                float rootSpeed = new Vector2(m_interpolated.LocalVelocity.x,
                    m_interpolated.LocalVelocity.z).magnitude;
                float speed = Mathf.Max(0.01f, m_model.Settings.Speed);
                float warp = rootSpeed > 0.05f
                    ? Mathf.Clamp(speed / rootSpeed, k_WarpMin, k_WarpMax)
                    : 1f;

                m_poseTime += dt * clipData.PlaybackSpeed * warp;
                var clip = m_animData.Clips[pd.PrimaryClipId];
                float len = clip != null ? clip.length : float.MaxValue;
                if (m_poseTime > len)
                {
                    if (clipData.IsLooping)
                    {
                        m_poseTime = Mathf.Repeat(m_poseTime, len);
                        m_curPoseId = clipData.StartPoseId; // realign after wrap
                    }
                    else
                    {
                        m_poseTime = len;
                    }
                }

                // Anchor the record index to playback: the animator compares
                // search winners against the CURRENT interpolated pose, not
                // the last switched record — anchoring here keeps the
                // same-location gate honest and stops blend churn.
                while (m_curPoseId < clipData.EndPoseId &&
                    m_animData.Poses[m_curPoseId + 1].Time <= m_poseTime)
                    ++m_curPoseId;

                BuildInterpolated();
            }
        }

        /// <summary>Rebuild the decision sequence from t=0 up to seekTime
        /// (scrub path; v1 brute replay at 60 Hz steps).</summary>
        public void ReplayTo(float seekTime)
        {
            if (m_animData == null)
                return;
            float dt = 1f / 60f;
            Reset();
            // Step without playback drift: search decisions only depend on
            // seekTime; run the loop to land the cache before seekTime.
            float t = 0f;
            while (t < seekTime - 1e-4f)
            {
                Step(dt, Mathf.Min(t, seekTime));
                t += dt;
            }
            Step(dt, seekTime);
        }

        // -- The search decision ------------------------------------------

        private void Search(float seekTime)
        {
            var pts = m_model.Points;
            if (pts.Count < 2)
                return;

            // 1. Active tag sets from the trigger timeline.
            ETags required = ETags.None;
            ETags favour = ETags.None;
            var trigs = m_model.Triggers;
            for (int i = 0; i < trigs.Count; ++i)
            {
                if (seekTime < trigs[i].Start || seekTime >= trigs[i].End)
                    continue;
                if (trigs[i].Required)
                    required |= m_animData.TagFromName(trigs[i].TagName);
                else
                    favour |= m_animData.FavourTagFromName(trigs[i].TagName);
            }

            // 2. Native data for the required set (fallback: untagged set).
            MxMNativeAnimData native = GetNative(required);
            if (native == null || native.UsedPoseIds.Length == 0)
            {
                if (required != ETags.None)
                {
                    native = GetNative(ETags.None);
                    required = ETags.None;
                }
            }
            if (native == null || native.UsedPoseIds.Length == 0)
            {
                WarnOnce($"no native pose set for required tags '{required}' " +
                    $"(available: {NativeSetSummary()})");
                return;
            }

            EnsureScratch(native.UsedPoseIds.Length, required);

            // 3. Synthetic goal from the spline (root-relative to the
            // simulated root AT seek time — the classic frame bug lives here).
            float speed = Mathf.Max(0.01f, m_model.Settings.Speed);
            Vector2 rootPos = LiveTrajectory.PosAtTime(pts, seekTime, speed);
            float yawRad = LiveTrajectory.HeadingAtTime(pts, seekTime, speed);

            var goalWorld = new MxM.TrajectoryPoint[m_numPredTimes];
            for (int i = 0; i < m_numPredTimes; ++i)
            {
                float p = m_animData.PosePredictionTimes[i];
                Vector2 w = LiveTrajectory.PosAtTime(pts, seekTime + p, speed);
                float facingDeg = LiveTrajectory.HeadingAtTime(pts, seekTime + p, speed)
                    * Mathf.Rad2Deg;

                // Rotate into the root frame (position −rootPos, yaw −yaw).
                float c = Mathf.Cos(-yawRad), s = Mathf.Sin(-yawRad);
                float dx = w.x - rootPos.x, dz = w.y - rootPos.y;
                goalWorld[i] = new MxM.TrajectoryPoint(
                    new Vector3(dx * c - dz * s, 0f, dx * s + dz * c),
                    Mathf.Repeat(facingDeg - yawRad * Mathf.Rad2Deg, 360f));
            }

            // 4. Schedule: trajectory → pose → minima, exactly like the
            // animator's UpdateJobs pipeline (synchronous here — editor tool).
            JobHandle trajHandle = ScheduleTrajectoryJob(native, goalWorld);
            trajHandle = SchedulePoseJob(native, trajHandle);
            JobHandle.ScheduleBatchedJobs();
            trajHandle.Complete();

            ScheduleMinimaJob(native, favour);

            int chosenUsed = m_chosenPoseId[0];
            int poseId = native.UsedPoseIds[
                Mathf.Clamp(chosenUsed, 0, native.UsedPoseIds.Length - 1)];
            var bestPose = m_animData.Poses[poseId];

            if (m_debugDump)
                BuildDebugDump(native, goalWorld, poseId);

            // Same-location continuity gate (mirrors MxMAnimator's
            // FinalizePoseSearch 'winnerAtSameLocation'): when the winner is
            // the clip already playing at (nearly) the same database time —
            // or anywhere within a looping clip — playback just continues.
            // No time reset, no re-blend. Without this, every interval
            // re-picks the same neighbourhood and drags m_poseTime back by up
            // to one pose interval: fragment repeats + pose jitter.
            if (m_curPoseId >= 0)
            {
                var cur = m_animData.Poses[m_curPoseId];
                bool sameLocation = bestPose.AnimId == cur.AnimId &&
                    (Mathf.Abs(bestPose.Time - cur.Time) < k_SameLocationTime ||
                     m_animData.ClipsData[cur.AnimId].IsLooping);
                if (sameLocation)
                {
                    m_decisions.Add(new Decision { Time = seekTime,
                        PoseId = m_curPoseId });
                    return; // keep playing; Step() advances m_poseTime
                }
            }

            if (m_curPoseId >= 0 && poseId != m_curPoseId)
            {
                // Crossfade source: the pose currently on display.
                m_blendFrom = m_interpolated.JointsData != null &&
                    m_interpolated.JointsData.Length > 0
                        ? new PoseData(m_interpolated)
                        : (PoseData?)null;
                m_blendElapsed = 0f;
            }

            m_curPoseId = poseId;
            m_poseTime = m_animData.Poses[poseId].Time;
            BuildInterpolated();
            m_decisions.Add(new Decision { Time = seekTime, PoseId = poseId });

            var pd = m_animData.Poses[poseId];
            var cd = m_animData.ClipsData[pd.AnimId];
            m_poseInfo = $"pose {poseId} · clip {pd.PrimaryClipId} @ {m_poseTime:0.00}s" +
                $" ({cd.ComplexAnimType})";
        }

        // -- Job scheduling (packing mirrors MxMAnimator's private           --
        // -- GeneratePoseXJob / GenerateTrajectoryXJob; supports 1..5 for     --
        // -- both counts — the demo dataset uses 5/5)                          --

        private JobHandle ScheduleTrajectoryJob(MxMNativeAnimData native,
            MxM.TrajectoryPoint[] goal)
        {
            int numPoses = native.UsedPoseIds.Length;
            float posMult = m_calib.TrajPosMultiplier * m_calib.PoseTrajectoryRatio * 2f;
            float angMult = m_calib.TrajFAngleMultiplier * m_calib.PoseTrajectoryRatio * 2f;

            switch (m_numPredTimes)
            {
                case 1:
                {
                    var goal01 = new float4(goal[0].Position.x, goal[0].Position.y,
                        goal[0].Position.z, goal[0].FacingAngle);
                    return new ComputeTrajectory01Costs()
                    {
                        InputTrajectories = native.TrajectoriesPacked_01,
                        DesiredTrajectory = goal01,
                        TrajPosMultiplier = posMult,
                        TrajFAngleMultiplier = angMult,
                        GoalCosts = m_trajCosts
                    }.Schedule(numPoses, 64);
                }
                case 2:
                {
                    var goal02 = new float2x4();
                    goal02.c0 = new float2(goal[0].Position.x, goal[1].Position.x);
                    goal02.c1 = new float2(goal[0].Position.y, goal[1].Position.y);
                    goal02.c2 = new float2(goal[0].Position.z, goal[1].Position.z);
                    goal02.c3 = new float2(goal[0].FacingAngle, goal[1].FacingAngle);
                    return new ComputeTrajectory02Costs()
                    {
                        InputTrajectories = native.TrajectoriesPacked_02,
                        DesiredTrajectory = goal02,
                        TrajPosMultiplier = posMult,
                        TrajFAngleMultiplier = angMult,
                        GoalCosts = m_trajCosts
                    }.Schedule(numPoses, 64);
                }
                case 3:
                {
                    var goal03 = new float3x4();
                    goal03.c0 = new float3(goal[0].Position.x, goal[1].Position.x, goal[2].Position.x);
                    goal03.c1 = new float3(goal[0].Position.y, goal[1].Position.y, goal[2].Position.y);
                    goal03.c2 = new float3(goal[0].Position.z, goal[1].Position.z, goal[2].Position.z);
                    goal03.c3 = new float3(goal[0].FacingAngle, goal[1].FacingAngle, goal[2].FacingAngle);
                    return new ComputeTrajectory03Costs()
                    {
                        InputTrajectories = native.TrajectoriesPacked_03,
                        DesiredTrajectory = goal03,
                        TrajPosMultiplier = posMult,
                        TrajFAngleMultiplier = angMult,
                        GoalCosts = m_trajCosts
                    }.Schedule(numPoses, 64);
                }
                case 4:
                {
                    var goal04 = new float4x4();
                    goal04.c0 = new float4(goal[0].Position.x, goal[1].Position.x, goal[2].Position.x, goal[3].Position.x);
                    goal04.c1 = new float4(goal[0].Position.y, goal[1].Position.y, goal[2].Position.y, goal[3].Position.y);
                    goal04.c2 = new float4(goal[0].Position.z, goal[1].Position.z, goal[2].Position.z, goal[3].Position.z);
                    goal04.c3 = new float4(goal[0].FacingAngle, goal[1].FacingAngle, goal[2].FacingAngle, goal[3].FacingAngle);
                    return new ComputeTrajectory04Costs()
                    {
                        InputTrajectories = native.TrajectoriesPacked_04,
                        DesiredTrajectory = goal04,
                        TrajPosMultiplier = posMult,
                        TrajFAngleMultiplier = angMult,
                        GoalCosts = m_trajCosts
                    }.Schedule(numPoses, 64);
                }
                case 5:
                {
                    var goal05 = new Trajectory6();
                    goal05.A.c0 = new float3(goal[0].Position.x, goal[1].Position.x, goal[2].Position.x);
                    goal05.A.c1 = new float3(goal[0].Position.y, goal[1].Position.y, goal[2].Position.y);
                    goal05.A.c2 = new float3(goal[0].Position.z, goal[1].Position.z, goal[2].Position.z);
                    goal05.A.c3 = new float3(goal[0].FacingAngle, goal[1].FacingAngle, goal[2].FacingAngle);
                    goal05.B.c0 = new float3(goal[3].Position.x, goal[4].Position.x, 0f);
                    goal05.B.c1 = new float3(goal[3].Position.y, goal[4].Position.y, 0f);
                    goal05.B.c2 = new float3(goal[3].Position.z, goal[4].Position.z, 0f);
                    goal05.B.c3 = new float3(goal[3].FacingAngle, goal[4].FacingAngle, 0f);
                    return new ComputeTrajectory05Costs()
                    {
                        InputTrajectories = native.TrajectoriesPacked_06,
                        DesiredTrajectory = goal05,
                        TrajPosMultiplier = posMult,
                        TrajFAngleMultiplier = angMult,
                        GoalCosts = m_trajCosts
                    }.Schedule(numPoses, 64);
                }
                default:
                {
                    Debug.LogError($"LiveSearchRunner: unsupported trajectory " +
                        $"point count {m_numPredTimes} (runner supports 1–5; " +
                        $"dataset has {m_animData.PosePredictionTimes.Length})");
                    return default;
                }
            }
        }

        private JobHandle SchedulePoseJob(MxMNativeAnimData native, JobHandle dependsOn)
        {
            int numPoses = native.UsedPoseIds.Length;
            float poseJointMult = m_calib.PoseAspectMultiplier * (1f - m_calib.PoseTrajectoryRatio) * 2f;
            float poseVelMult = m_calib.PoseVelocityWeight * (1f - m_calib.PoseTrajectoryRatio) * 2f;
            var jd = m_interpolated.JointsData;
            var locVel = m_interpolated.LocalVelocity;

            switch (m_numJoints)
            {
                case 1:
                {
                    var desired = new float3x3();
                    desired.c0 = new float3(jd[0].Position.x, jd[0].Velocity.x, locVel.x);
                    desired.c1 = new float3(jd[0].Position.y, jd[0].Velocity.y, locVel.y);
                    desired.c2 = new float3(jd[0].Position.z, jd[0].Velocity.z, locVel.z);
                    return new ComputePose01Costs()
                    {
                        InputPoses = native.PosesPacked_01,
                        DesiredPose = desired,
                        Weights = new float3(
                            m_calib.JointPositionWeights[0] * poseJointMult,
                            m_calib.JointVelocityWeights[0] * poseVelMult,
                            poseVelMult),
                        GoalCosts = m_poseCosts
                    }.Schedule(numPoses, 64, dependsOn);
                }
                case 2:
                {
                    var desired = new Pose2();
                    desired.JointPositions.c0 = new float2(jd[0].Position.x, jd[1].Position.x);
                    desired.JointPositions.c1 = new float2(jd[0].Position.y, jd[1].Position.y);
                    desired.JointPositions.c2 = new float2(jd[0].Position.z, jd[1].Position.z);
                    desired.JointVelocities.c0 = new float3(locVel.x, jd[0].Velocity.x, jd[1].Velocity.x);
                    desired.JointVelocities.c1 = new float3(locVel.y, jd[0].Velocity.y, jd[1].Velocity.y);
                    desired.JointVelocities.c2 = new float3(locVel.z, jd[0].Velocity.z, jd[1].Velocity.z);
                    return new ComputePose02Costs()
                    {
                        InputPoses = native.PosesPacked_02,
                        DesiredPose = desired,
                        JointVelocityWeights = new float3(poseVelMult,
                            m_calib.JointVelocityWeights[0] * poseVelMult,
                            m_calib.JointVelocityWeights[1] * poseVelMult),
                        JointPositionWeights = new float2(
                            m_calib.JointPositionWeights[0] * poseJointMult,
                            m_calib.JointPositionWeights[1] * poseJointMult),
                        GoalCosts = m_poseCosts
                    }.Schedule(numPoses, 64, dependsOn);
                }
                case 3:
                {
                    var desired = new Pose3();
                    desired.JointPositions.c0 = new float3(jd[0].Position.x, jd[1].Position.x, jd[2].Position.x);
                    desired.JointPositions.c1 = new float3(jd[0].Position.y, jd[1].Position.y, jd[2].Position.y);
                    desired.JointPositions.c2 = new float3(jd[0].Position.z, jd[1].Position.z, jd[2].Position.z);
                    desired.JointVelocities.c0 = new float4(locVel.x, jd[0].Velocity.x, jd[1].Velocity.x, jd[2].Velocity.x);
                    desired.JointVelocities.c1 = new float4(locVel.y, jd[0].Velocity.y, jd[1].Velocity.y, jd[2].Velocity.y);
                    desired.JointVelocities.c2 = new float4(locVel.z, jd[0].Velocity.z, jd[1].Velocity.z, jd[2].Velocity.z);
                    return new ComputePose03Costs()
                    {
                        InputPoses = native.PosesPacked_03,
                        DesiredPose = desired,
                        JointVelocityWeights = new float4(poseVelMult,
                            m_calib.JointVelocityWeights[0] * poseVelMult,
                            m_calib.JointVelocityWeights[1] * poseVelMult,
                            m_calib.JointVelocityWeights[2] * poseVelMult),
                        JointPositionWeights = new float3(
                            m_calib.JointPositionWeights[0] * poseJointMult,
                            m_calib.JointPositionWeights[1] * poseJointMult,
                            m_calib.JointPositionWeights[2] * poseJointMult),
                        GoalCosts = m_poseCosts
                    }.Schedule(numPoses, 64, dependsOn);
                }
                case 4:
                {
                    var desired = new Pose4();
                    desired.JointPositions.c0 = new float4(jd[0].Position.x, jd[1].Position.x, jd[2].Position.x, jd[3].Position.x);
                    desired.JointPositions.c1 = new float4(jd[0].Position.y, jd[1].Position.y, jd[2].Position.y, jd[3].Position.y);
                    desired.JointPositions.c2 = new float4(jd[0].Position.z, jd[1].Position.z, jd[2].Position.z, jd[3].Position.z);
                    desired.JointVelocities.c0 = new float4(jd[0].Velocity.x, jd[1].Velocity.x, jd[2].Velocity.x, jd[3].Velocity.x);
                    desired.JointVelocities.c1 = new float4(jd[0].Velocity.y, jd[1].Velocity.y, jd[2].Velocity.y, jd[3].Velocity.y);
                    desired.JointVelocities.c2 = new float4(jd[0].Velocity.z, jd[1].Velocity.z, jd[2].Velocity.z, jd[3].Velocity.z);
                    desired.BodyVelocity = locVel;
                    return new ComputePose04Costs()
                    {
                        InputPoses = native.PosesPacked_04,
                        DesiredPose = desired,
                        JointVelocityWeights = new float4(
                            m_calib.JointVelocityWeights[0] * poseVelMult,
                            m_calib.JointVelocityWeights[1] * poseVelMult,
                            m_calib.JointVelocityWeights[2] * poseVelMult,
                            m_calib.JointVelocityWeights[3] * poseVelMult),
                        JointPositionWeights = new float4(
                            m_calib.JointPositionWeights[0] * poseJointMult,
                            m_calib.JointPositionWeights[1] * poseJointMult,
                            m_calib.JointPositionWeights[2] * poseJointMult,
                            m_calib.JointPositionWeights[3] * poseJointMult),
                        BodyVelocityWeight = poseVelMult,
                        GoalCosts = m_poseCosts
                    }.Schedule(numPoses, 64, dependsOn);
                }
                case 5:
                {
                    var desired = new Pose5();
                    desired.JointPositionsA.c0 = new float3(jd[0].Position.x, jd[1].Position.x, jd[2].Position.x);
                    desired.JointPositionsA.c1 = new float3(jd[0].Position.y, jd[1].Position.y, jd[2].Position.y);
                    desired.JointPositionsA.c2 = new float3(jd[0].Position.z, jd[1].Position.z, jd[2].Position.z);
                    desired.JointPositionsB.c0 = new float2(jd[3].Position.x, jd[4].Position.x);
                    desired.JointPositionsB.c1 = new float2(jd[3].Position.y, jd[4].Position.y);
                    desired.JointPositionsB.c2 = new float2(jd[3].Position.z, jd[4].Position.z);
                    desired.JointVelocitiesA.c0 = new float3(locVel.x, jd[0].Velocity.x, jd[1].Velocity.x);
                    desired.JointVelocitiesA.c1 = new float3(locVel.y, jd[0].Velocity.y, jd[1].Velocity.y);
                    desired.JointVelocitiesA.c2 = new float3(locVel.z, jd[0].Velocity.z, jd[1].Velocity.z);
                    desired.JointVelocitiesB.c0 = new float3(jd[2].Velocity.x, jd[3].Velocity.x, jd[4].Velocity.x);
                    desired.JointVelocitiesB.c1 = new float3(jd[2].Velocity.y, jd[3].Velocity.y, jd[4].Velocity.y);
                    desired.JointVelocitiesB.c2 = new float3(jd[2].Velocity.z, jd[3].Velocity.z, jd[4].Velocity.z);
                    return new ComputePose05Costs()
                    {
                        InputPoses = native.PosesPacked_05,
                        DesiredPose = desired,
                        JointVelocityWeightsA = new float3(poseVelMult,
                            m_calib.JointVelocityWeights[0] * poseVelMult,
                            m_calib.JointVelocityWeights[1] * poseVelMult),
                        JointVelocityWeightsB = new float3(
                            m_calib.JointVelocityWeights[2] * poseVelMult,
                            m_calib.JointVelocityWeights[3] * poseVelMult,
                            m_calib.JointVelocityWeights[4] * poseVelMult),
                        JointPositionWeightsA = new float3(
                            m_calib.JointPositionWeights[0] * poseJointMult,
                            m_calib.JointPositionWeights[1] * poseJointMult,
                            m_calib.JointPositionWeights[2] * poseJointMult),
                        JointPositionWeightsB = new float2(
                            m_calib.JointPositionWeights[3] * poseJointMult,
                            m_calib.JointPositionWeights[4] * poseJointMult),
                        GoalCosts = m_poseCosts
                    }.Schedule(numPoses, 64, dependsOn);
                }
                default:
                {
                    Debug.LogError($"LiveSearchRunner: unsupported match joint " +
                        $"count {m_numJoints} (runner supports 1–5; dataset has " +
                        $"{m_animData.MatchBones.Length})");
                    return dependsOn;
                }
            }
        }

        private void ScheduleMinimaJob(MxMNativeAnimData native, ETags favourTags)
        {
            if (favourTags == ETags.None)
            {
                new FindMinima()
                {
                    PoseCosts = m_poseCosts,
                    TrajCosts = m_trajCosts,
                    PoseFavour = native.FavourPacked,
                    ChosenPoseId = m_chosenPoseId
                }.Run();
            }
            else
            {
                new FindMinima_FavourExclusive()
                {
                    PoseCosts = m_poseCosts,
                    TrajCosts = m_trajCosts,
                    PoseFavour = native.FavourPacked,
                    PoseFavourTags = native.FavourTagsPacked,
                    FavourTags = favourTags,
                    FavourMultiplier = k_FavourMultiplier,
                    ChosenPoseId = m_chosenPoseId
                }.Run();
            }
        }

        private bool m_warned;

        private void WarnOnce(string msg)
        {
            if (m_warned)
                return;
            m_warned = true;
            Debug.LogWarning("LiveSearchRunner: " + msg);
        }

        private string NativeSetSummary()
        {
            if (m_animData.NativeAnimData == null)
                return "none (native data not initialized)";
            var sb = new System.Text.StringBuilder();
            foreach (var kv in m_animData.NativeAnimData)
                sb.Append(kv.Key).Append($"({kv.Value.UsedPoseIds.Length}) ");
            return sb.ToString();
        }

        private MxMNativeAnimData GetNative(ETags tags)
        {
            if (m_animData.NativeAnimData != null &&
                m_animData.NativeAnimData.TryGetValue(tags, out var native))
                return native;
            return null;
        }

        private void EnsureScratch(int usedCount, ETags tags)
        {
            if (m_scratchTags == tags && m_poseCosts.IsCreated &&
                m_poseCosts.Length == usedCount)
                return;
            if (m_poseCosts.IsCreated)
                m_poseCosts.Dispose();
            if (m_trajCosts.IsCreated)
                m_trajCosts.Dispose();
            if (m_chosenPoseId.IsCreated)
                m_chosenPoseId.Dispose();
            m_poseCosts = new NativeArray<float>(usedCount, Allocator.Persistent,
                NativeArrayOptions.ClearMemory);
            m_trajCosts = new NativeArray<float>(usedCount, Allocator.Persistent,
                NativeArrayOptions.ClearMemory);
            m_chosenPoseId = new NativeArray<int>(1, Allocator.Persistent);
            m_scratchTags = tags;
        }

        // -- Interpolated pose (job input): lerp toward the next pose record --
        // -- of the same clip. Approximates what the animated skeleton would --
        // -- give the animator; database-only (no clip sampling needed).      --

        private void BuildInterpolated()
        {
            if (m_curPoseId < 0)
                return;

            var pose = m_animData.Poses[m_curPoseId];
            m_interpolated = new PoseData(pose);

            var clipData = m_animData.ClipsData[pose.AnimId];
            int nextId = pose.PoseId + 1;
            if (nextId > clipData.EndPoseId)
                return; // hold at clip end (re-search picks a new clip)

            var next = m_animData.Poses[nextId];
            float interval = Mathf.Max(0.001f, m_animData.PoseInterval);
            float frac = Mathf.Clamp01((m_poseTime - pose.Time) / interval);

            int joints = pose.JointsData != null ? pose.JointsData.Length : 0;
            if (joints == 0 || next.JointsData == null || next.JointsData.Length != joints)
                return;

            if (m_interpolated.JointsData == null || m_interpolated.JointsData.Length != joints)
                m_interpolated.JointsData = new JointData[joints];

            for (int j = 0; j < joints; ++j)
            {
                m_interpolated.JointsData[j] = new JointData(
                    Vector3.Lerp(pose.JointsData[j].Position, next.JointsData[j].Position, frac),
                    Vector3.Lerp(pose.JointsData[j].Velocity, next.JointsData[j].Velocity, frac));
            }
            m_interpolated.LocalVelocity = Vector3.Lerp(
                pose.LocalVelocity, next.LocalVelocity, frac);

            ApplyBlend(m_interpolated);
        }

        /// <summary>Crossfade toward the displayed pose over BlendInTime:
        /// while the blend window is open, joints still carry a share of the
        /// pre-decision pose — a cheap stand-in for the animator's clip
        /// crossfading, enough to kill pose pops on re-decisions.</summary>
        private void ApplyBlend(PoseData target)
        {
            float blend = m_model.Settings.BlendInTime;
            if (!m_blendFrom.HasValue || m_blendFrom.Value.JointsData == null ||
                m_blendElapsed >= blend || blend <= 0f)
            {
                m_blendFrom = null;
                return;
            }

            var from = m_blendFrom.Value;
            float f = 1f - Mathf.Clamp01(m_blendElapsed / blend); // share of old pose
            int joints = Mathf.Min(target.JointsData.Length, from.JointsData.Length);
            for (int j = 0; j < joints; ++j)
            {
                target.JointsData[j] = new JointData(
                    Vector3.Lerp(target.JointsData[j].Position,
                        from.JointsData[j].Position, f),
                    Vector3.Lerp(target.JointsData[j].Velocity,
                        from.JointsData[j].Velocity, f));
            }
            target.LocalVelocity = Vector3.Lerp(
                target.LocalVelocity, from.LocalVelocity, f);
        }

        // -- PoseProbe (skeleton proxy for the live view) ----------------------

        public void GetSkeleton(IList<Vector3> offsetsOut)
        {
            offsetsOut.Clear();
            if (m_interpolated.JointsData == null)
                return;
            for (int i = 0; i < m_interpolated.JointsData.Length; ++i)
                offsetsOut.Add(m_interpolated.JointsData[i].Position);
        }

        private bool m_disposed;

        public void Dispose()
        {
            // Idempotent: the window and AttachRunner may both drop us.
            // ReleaseNativeData is ref-counted on the SHARED asset — double-
            // releasing here would free another runner's live buffers.
            if (m_disposed)
                return;
            m_disposed = true;
            if (m_animData != null)
                m_animData.ReleaseNativeData();
            if (m_poseCosts.IsCreated)
                m_poseCosts.Dispose();
            if (m_trajCosts.IsCreated)
                m_trajCosts.Dispose();
            if (m_chosenPoseId.IsCreated)
                m_chosenPoseId.Dispose();
        }
    }
}
#endif