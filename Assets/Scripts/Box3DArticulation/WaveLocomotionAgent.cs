using Unity.Mathematics;
using Unity.MLAgents;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>Wave-parameter locomotion agent. Controls the four limb probes
    /// through their runtime control surface: per-limb minimal wave params
    /// (ampBase/ampTip/bias/phase/freq + lean + poseRoll + roll). Actions are
    /// absolute targets in [-1,1], mapped onto the probe clamp ranges (phase
    /// offsets stay raw multiples of kPhaseLim - the probe's ramp smooths
    /// them). Repeats between decisions are idempotent by construction. Rewards pipe from RewardProbe per physics step. Attach
    /// to the same object as PinnedBipedDemo + RewardProbe (one instance per
    /// biped; refs are local, no scene lookups).</summary>
    [RequireComponent(typeof(RewardProbe))]
    public sealed class WaveLocomotionAgent : Agent
    {
        [SerializeField] private PinnedBipedDemo demo;
        [SerializeField] private RewardProbe rewardProbe;
        [SerializeField] private LocomotionTarget target;
        // Wired reflectively by BipedBatch when a reference prefab is assigned.
        // Null-safe everywhere: no ghost, no imitation behavior at all.
        private ImitationReference imitationReference;

        [SerializeField, Tooltip("Debug: force Heuristic-Only behavior (random-noise actions). Batch sets this from its toggle.")]
        private bool forceHeuristicOnly = false;

        [Header("Decision")]
        [SerializeField, Min(1), Tooltip("Physics steps per decision (60 Hz step: 2 = 30 Hz).")]
        private int decisionPeriod = 3;
        [SerializeField, Min(1), Tooltip("Physics steps per decision (60 Hz step: 2 = 30 Hz).")]
        private int decisionStep = 0;
        [SerializeField, Min(0.5f), Tooltip("Mean tracking distance (m) over an episode below which it counts as a curriculum success. Standing still near the target passes; drifting around does not.")]
        private float successMaxMeanDist = 0.5f;

        [SerializeField, Min(0f), Tooltip("Abort grace (s) after episode start: the rig spawns slightly off the ground and settles - transient height dips and pose errors during this window are not failures.")]
        private float abortGrace = 1.5f;
        [SerializeField, Tooltip("Per-second dislocation penalty beyond SepMax threshold (summed over 4 limbs). Puns solver-driven joint separation the posture term can't see.")]
        private float dislocationK = 0.05f;
        [SerializeField, Tooltip("COM-to-target distance (m) that aborts the episode - the ONLY early exit besides MaxStep. A fallen rig drifts past this as the target roams; straying also aborts.")]
        private float deviateAbortDist = 2f;
        [SerializeField, Tooltip("Separation (m) tolerated before the penalty starts - servo jitter band, per joint.")]
        private float dislocationTolerance = 0.02f;

        [SerializeField, Tooltip("Per-second posture penalty per rad of limb contortion (summed over 4 limbs). 0.1: fully-folded rig = ~-1.2/s.")]
        private float limbExtremityK = 0.01f;

        [Header("Torso yaw assist (soft cheat)")]
        [SerializeField, Tooltip("Apply a soft torque yaw-aligning the torso with the direction to the target. The wave gait has no yaw authority of its own - this is the assist. Torque scales with uprightness: a toppling or fallen torso gets no help, so falls still end episodes.")]
        private bool torsoYawAssist = true;
        [SerializeField, Min(0f), Tooltip("Yaw-align torque per rad of heading error (N*m/rad). Torso yaw inertia is ~0.5 kg*m^2 - start ~5.")]
        private float torsoYawTorque = -300f;
        [SerializeField, Min(0f), Tooltip("Yaw-rate damping of the assist torque (N*m per rad/s), keeps the assist from ringing around the heading.")]
        private float torsoYawDamp = 0.5f;

        // Absolute action mapping: a in [-1,1] -> param clamp range. One-sided
        // params (amps) use (a+1)/2; symmetric params use a * limit.

        // Hard clamp ranges (must mirror WaveParamTester sliders).
        private const float kFreqMax = 2f;
        private const float kLeanLim = 0.6f;
        private const float kLeanLimArm = 1.2f;
        private const float kBiasLim = 1.2f;
        private const float kAmpMax = 2.6f;
        private const float kPhaseLim = 3.14159f;

        // Runtime action state, PER LIMB (indices 0..3 = legL, legR, armL, armR).
        // Minimal wave-shape set: the 5 wave params + the 3 frame DOFs (lean,
        // pose roll, roll) = 8 per limb. Wavenumber is FIXED per limb (cached at
        // resolve): on a 2-link chain it is redundant with phase (Ijspeert
        // phase-lag form). Twist is dropped entirely: 3 points are always
        // coplanar.
        private struct LimbState
        {
            public float ampBase, ampTip, bias, phase, freq, lean, poseRoll, roll;
        }
        private const int kLimbParams = 8;
        private const float kRollLim = 3.14159f;
        private const int kActRaw = 4 * kLimbParams + 1; // + 1 smoothing-rate channel
        private readonly LimbState[] m_limbs = new LimbState[4];
        private readonly LimbState[] m_limbsT = new LimbState[4]; // smoothing targets
        private readonly float[] m_limbWaven = new float[4];      // fixed wavenumber per limb
        private static readonly LimbState kLimbDefault = new LimbState
        {
            bias = 0f, ampTip = 0.58f, ampBase = 0.58f,
            phase = 0f, freq = 0.5f, lean = 0f, poseRoll = 0f, roll = 0f
        };

        [Header("Initial wave randomization")]
        [SerializeField, Tooltip("Per-limb wave params randomized at every episode reset: phase uniform [-pi,pi], freq uniform [-0.25,0.25] Hz, amps uniform [initAmpMulMin,initAmpMulMax] x the construction default. lean/bias/poseRoll/roll stay at the default. The policy always sees its own params (action echoes), so this is pure domain randomization of the reset state - it breaks the identical symmetric spawn and the reset transient. Off = legacy identical reset.")]
        private bool randomizeInitWave = true;
        [SerializeField, Min(0f), Tooltip("Amp randomization range, multiples of the construction default (0.28 m).")]
        private float initAmpMulMin = 0.5f;
        [SerializeField, Min(0f)]
        private float initAmpMulMax = 2f;

        [Header("Action smoothing")]
        [SerializeField, Tooltip("Per-step lerp bounds for blending the live wave state toward the latest decision's targets. The policy's rate action [-1,1] maps to [min,max] alpha. 1 = instant (old behavior); min = slowest glide.")]
        private float smoothAlphaMin = 0.1f, smoothAlphaMax = 1f;

        private BackboneWaveProbe m_legL, m_legR, m_armL, m_armR;

        // Component wiring must happen in Awake: the Academy builds the policy
        // from BrainParameters when the agent registers (before Initialize),
        // so action/observation sizes set any later arrive too late - the
        // heuristic then runs against a zero-sized action buffer.
        private void Awake()
        {
            if (demo == null) demo = GetComponentInParent<PinnedBipedDemo>();
            if (rewardProbe == null) rewardProbe = GetComponent<RewardProbe>();
            // Seed the live wave state with the demo's construction defaults
            // (the policy then owns it from the first decision on).
            for (int i = 0; i < 4; i++) { m_limbs[i] = kLimbDefault; m_limbsT[i] = kLimbDefault; }

            var req = GetComponent<Unity.MLAgents.DecisionRequester>();
            if (req == null) req = gameObject.AddComponent<Unity.MLAgents.DecisionRequester>();
            req.DecisionPeriod = decisionPeriod;
            req.DecisionStep = decisionStep;
            // Absolute action targets are idempotent: re-applying the same
            // buffer between decisions just pushes identical params.
            req.TakeActionsBetweenDecisions = true;

            // Keep Behavior Parameters consistent with the code - the numbers
            // must match exactly (obs 88 = 16 base + 8 phase + 32 action
            // echoes + 32 ghost, actions 33) and hand-editing them is how
            // silent size bugs get in.
            var bp = GetComponent<BehaviorParameters>();
            if (bp == null) bp = gameObject.AddComponent<BehaviorParameters>();
            bp.BehaviorName = "WaveLoco";
            bp.BrainParameters.VectorObservationSize = 88;
            // Per-limb minimal wave params: 8 channels x 4 limbs + smoothing rate.
            bp.BrainParameters.ActionSpec = new ActionSpec(kActRaw, System.Array.Empty<int>());
            if (forceHeuristicOnly) bp.BehaviorType = BehaviorType.HeuristicOnly;
        }

        // Limb refs come straight from the demo's CreateDemo captures (demo
        // LegL/LegR/ArmL/ArmR) - name lookup is banned here: Transform.Find
        // matches INACTIVE children, so post-reset it bound the deactivated
        // pending-destroy old rig and pushes went into dead probes.
        private static BackboneWaveProbe RefindLimb(PinnedBipedDemo demo, string side, string kind)
        {
            if (demo == null) return null;
            return kind == "leg" ? (side == "L" ? demo.LegL : demo.LegR)
                                 : (side == "L" ? demo.ArmL : demo.ArmR);
        }

        // --- Genome: static policy-invisible parameters, unique per limb ---
        // Set once by the batch spawner (BipedBatch) before activation; every
        // ResolveLimbs re-applies them, so pooled resets AND full rebuilds
        // both land back on the instance's genome. Drawn independently per
        // limb - no L/R mirroring, that asymmetry is part of the identity.
        private readonly WaveGenome[] m_genomes = new WaveGenome[4];
        private bool m_genomesSet;
        /// <summary>Assign one limb's genome. Store-only here: application
        /// happens in ResolveLimbs (before the wavenumber push cache is
        /// built), which runs at Initialize and every episode begin.</summary>
        public void SetGenome(int limb, in WaveGenome g)
        {
            if (limb < 0 || limb > 3) return;
            m_genomes[limb] = g;
            m_genomesSet = true;
        }
        /// <summary>Read one limb's genome (selection loops clone/mutate it).</summary>
        public WaveGenome GetGenome(int limb) =>
            limb >= 0 && limb <= 3 ? m_genomes[limb] : default;

        // --- Genome fitness: mean episode reward over a trailing window ---
        // Selection signal for the batch's PBT-lite loop. The constant LR
        // means the policy NEVER settles, so single-episode rewards are
        // dominated by policy-update noise - the window is what lifts the
        // genome signal above it.
        [Header("Genome fitness (PBT selection signal, read by BipedBatch)")]
        [SerializeField, Min(3), Tooltip("Fitness = mean episode reward over the last N ended episodes. Keep long: the constant LR never settles the policy, so the window averages out update noise.")]
        private int fitnessWindow = 20;
        private float[] m_fitnessBuf;
        private int m_fitnessIdx, m_fitnessFilled, m_fitnessTotal;

        private void PushFitness(float episodeReward)
        {
            int size = Mathf.Max(3, fitnessWindow);
            if (m_fitnessBuf == null || m_fitnessBuf.Length != size)
                m_fitnessBuf = new float[size];
            m_fitnessBuf[m_fitnessIdx] = episodeReward;
            m_fitnessIdx = (m_fitnessIdx + 1) % size;
            m_fitnessFilled = Mathf.Min(m_fitnessFilled + 1, size);
            m_fitnessTotal++;
        }
        /// <summary>Mean episode reward over the trailing window. NaN until
        /// the first episode ends (selection must gate on FitnessSamples).</summary>
        public float GenomeFitness()
        {
            if (m_fitnessBuf == null || m_fitnessFilled == 0) return float.NaN;
            float sum = 0f;
            for (int i = 0; i < m_fitnessFilled; i++) sum += m_fitnessBuf[i];
            return sum / m_fitnessFilled;
        }
        /// <summary>Total ended episodes sampled (monotonic) - the gate for
        /// "this instance has enough history to rank".</summary>
        public int FitnessSamples => m_fitnessTotal;

        private void ResolveLimbs()
        {
            m_legL = RefindLimb(demo, "L", "leg");
            m_legR = RefindLimb(demo, "R", "leg");
            m_armL = RefindLimb(demo, "L", "arm");
            m_armR = RefindLimb(demo, "R", "arm");
            var probes = new[] { m_legL, m_legR, m_armL, m_armR };
            // Genome first: static policy-invisible params (wavenumber,
            // twist, servo shaping). Set by the batch spawner once, re-applied
            // after every full rebuild - a pooled reset keeps the probe
            // components and their genome. Applied BEFORE the wavenumber
            // cache read so pushes always use the genome's value.
            if (m_genomesSet)
                for (int i = 0; i < 4; i++)
                    if (probes[i] != null) probes[i].ApplyGenome(m_genomes[i]);
            // Cache each limb's fixed wavenumber (the genome's value)
            // - not policy-facing, but SetRuntimeParams requires it on push.
            for (int i = 0; i < 4; i++)
                if (probes[i] != null)
                    m_limbWaven[i] = probes[i].GetRuntimeParams().wavenumber;
        }

        public override void Initialize() => ResolveLimbs();

        public override void OnEpisodeBegin()
        {
            // 4.1.0 has no OnEpisodeEnd hook: a begin reached without an
            // intervening fall means the previous episode survived to MaxStep.
            // Success ALSO requires tracking progress: survival alone (standing
            // beside a frozen d=0 target) must not promote the curriculum.
            if (m_episodeSurvived &&
                m_distSteps > 0 && m_distSum / m_distSteps <= successMaxMeanDist)
                ReportSuccess();
            // Fitness sample for genome selection (BipedBatch reads it):
            // the PREVIOUS episode's total reward, pushed before the counters
            // and gauges reset. First-ever begin pushes nothing.
            if (m_episodeSteps > 0 && rewardProbe != null)
                PushFitness(rewardProbe.EpisodeReward);
            m_episodeSurvived = true;
            m_distSum = 0f;
            m_distSteps = 0;
            m_episodeSteps = 0;

            // Curriculum: one env parameter maps to everything (target motion,
            // debt forgiveness). Trainer overrides win; otherwise the local
            // lesson stepper's value applies.
            EnsureTrainerCallback();
            float d = kDifficulty;
            var envParams = Academy.Instance != null ? Academy.Instance.EnvironmentParameters : null;
            if (envParams != null) d = envParams.GetWithDefault("task_difficulty", kDifficulty);
            kDifficulty = math.clamp(d, 0f, 1f);
            // Rebuild the rig FIRST so the torso pose exists, then reset the
            // target facing the new rig's yaw.
            // Pooled reset first: same native bodies/joints teleported back
            // to the spawn pose - none of the GameObject/native lifecycle
            // cost of a rebuild (the dominant per-episode init expense).
            // Full rebuild only when the pool can't be trusted: first
            // episode, a limb failed to spawn, or bodies invalid.
            if (demo != null)
            {
                if (!demo.ResetToSpawn())
                {
                    demo.ClearGeneratedPublic();
                    demo.CreateDemo();
                }
            }
            ResolveLimbs();

            if (target != null)
            {
                target.ApplyDifficulty(kDifficulty);
                // Torso yaw at spawn: the target starts facing the same
                // direction (headingRandomizeAt can re-enable variety). Uses
                // the RIG-legacy +Z convention - this yaw feeds the target's
                // transform heading, which must match the rig's authored facing,
                // not the reward layer's -Z forward read.
                float yaw = 0f;
                if (demo != null && demo.Torso != null && demo.Torso.Body.IsValid)
                {
                    float3 fwd = math.mul(demo.Torso.Body.Rotation, new float3(0f, 0f, 1f));
                    yaw = math.atan2(fwd.x, fwd.z);
                }
                target.ResetToSpawn(kDifficulty, yaw);
                // Ghost reset: the imitation reference opens every episode at
                // its spawn point facing the rig's yaw (same convention as the
                // target), so ghost and rig start aligned. Explicit null check
                // (not ?.): destroyed Unity objects are fake-null and ?. would
                // still call into them.
                if (imitationReference != null)
                    imitationReference.EpisodeReset(yaw);
            }
            rewardProbe.ApplyDifficulty(kDifficulty);

            for (int i = 0; i < 4; i++)
            {
                var s = kLimbDefault;
                if (randomizeInitWave)
                {
                    // Phase per limb: breaks the symmetric spawn. freq/amps:
                    // uniform subranges with the construction default as mean.
                    // All draws inside the clamp ranges - the policy must be
                    // able to reach every state it is handed.
                    s.phase = (UnityEngine.Random.value * 2f - 1f) * kPhaseLim;
                    s.freq = (UnityEngine.Random.value * 2f - 1f) * (0.5f * kFreqMax);
                    float ampMul = UnityEngine.Random.Range(initAmpMulMin, initAmpMulMax);
                    s.ampBase = math.clamp(kLimbDefault.ampBase * ampMul, 0f, kAmpMax);
                    s.ampTip = math.clamp(kLimbDefault.ampTip * ampMul, 0f, kAmpMax);
                }
                m_limbs[i] = s;
                m_limbsT[i] = s; // targets = live state: no glide at spawn
            }
            PushLimbs();
            // Obs ghost tracker: m_gLastSample <= 0 makes the first post-reset
            // collection emit zero tip velocity (otherwise the stale pre-reset
            // positions would turn the teleport into one garbage obs sample).
            m_gLastSample = -1f;
            rewardProbe.ResetGaugesPublic();
        }

        private static readonly float[] s_zeroObs88 = new float[88];  // fallback: no torso
        private static readonly float[] s_zeroGhost8 = new float[8];  // fallback: no ghost

        // 88 floats, torso-local everywhere.
        public override void CollectObservations(VectorSensor sensor)
        {
            var torso = rewardProbe.TorsoBody;
            if (torso == null || !torso.Body.IsValid || rewardProbe.Target == null)
            {
                sensor.AddObservation(s_zeroObs88);
                return;
            }
            var b = torso.Body;
            float3 pos = b.Position;
            quaternion rot = b.Rotation;

            // Torso yaw frame (forward = local +Z, projected on XZ - matches the
            // rig's authored facing; no yaw-lock).
            float3 fwd = math.mul(rot, new float3(0f, 0f, 1f));
            float yaw = math.atan2(fwd.x, fwd.z);
            m_yawRot = quaternion.RotateY(-yaw); // world -> torso frame
            quaternion yawRot = m_yawRot;        // section 1-7 keep the local alias
            float2 ToLocal(float3 vW) => math.mul(yawRot, vW).xz;

            // 1. target direction + distance (torso-local)
            float3 dW = rewardProbe.Target.Position - pos;
            float2 dL = ToLocal(dW);
            float dist = math.length(dL);
            float2 dN = dist > 1e-4f ? dL / dist : new float2(1f, 0f);
            sensor.AddObservation(dN.x);
            sensor.AddObservation(dN.y);
            sensor.AddObservation(math.clamp(dist / 10f, 0f, 1f));

            // 2. target velocity (torso-local, m/s / 3)
            float3 tvW = rewardProbe.Target.MoveDirW * rewardProbe.Target.Speed;
            float2 tvL = ToLocal(tvW);
            sensor.AddObservation(tvL.x / 3f);
            sensor.AddObservation(tvL.y / 3f);

            // 3. torso linear velocity (torso-local, m/s / 3)
            float2 vL = ToLocal((float3)b.LinearVelocity);
            sensor.AddObservation(vL.x / 3f);
            sensor.AddObservation(vL.y / 3f);

            // 4. torso angular velocity (world frame is fine - tilt axes matter, /3)
            float3 w = (float3)b.AngularVelocity;
            sensor.AddObservation(w.x / 3f);
            sensor.AddObservation(w.y / 3f);
            sensor.AddObservation(w.z / 3f);

            // 5. upright dot + tilt debt + COM height (m / 2, clamped)
            float3 up = math.mul(rot, new float3(0f, 1f, 0f));
            sensor.AddObservation(up.y);
            sensor.AddObservation(rewardProbe.Debt);
            float3 com = b.Position + math.mul(rot, rewardProbe.ComLocal);
            sensor.AddObservation(math.clamp(com.y / 2f, 0f, 1f));

            // 5b. whole-rig COM offset from the torso COM (torso yaw frame):
            // limb swing momentum shifting the balance point - the signal the
            // torso proxy cannot see. xz / 2, y / 1 (rig COM rides ~at torso
            // height; deviations of +/-1 m are already extreme).
            float3 rigCom = rewardProbe.RigComW();
            float3 rigRel = rigCom - com;
            float2 rigL = ToLocal(rigRel);
            sensor.AddObservation(math.clamp(rigL.x / 2f, -1f, 1f));
            sensor.AddObservation(math.clamp(rigL.y / 2f, -1f, 1f));
            sensor.AddObservation(math.clamp(rigRel.y / 1f, -1f, 1f));

            // 6. per-limb phase (sin/cos) x4 limbs
            AddPhase(sensor, m_legL); AddPhase(sensor, m_legR);
            AddPhase(sensor, m_armL); AddPhase(sensor, m_armR);

            // 7. action echoes (normalized to [-1,1] over the clamp ranges)
            EchoLimb(sensor, 0); EchoLimb(sensor, 1);
            EchoLimb(sensor, 2); EchoLimb(sensor, 3);

            // 8. ghost reference (DeepMimic "goal" block): what the imitation
            // reward pays for, in the same torso-yaw frame as sections 1-5b.
            // Per limb: joint dir (2, XZ) + tip pos rel. anchor (3) + tip vel
            // rel. anchor (3) = 8 x 4 = 32. Zeros when no ghost is wired.
            float gDt = m_gLastSample > 0f ? Time.time - m_gLastSample : 0f;
            m_gLastSample = Time.time; // once per collection, not per limb
            GhostLimb(sensor, 0, gDt); GhostLimb(sensor, 1, gDt);
            GhostLimb(sensor, 2, gDt); GhostLimb(sensor, 3, gDt);
        }

        // Ghost finite-difference state, owned by the OBSERVATION path (the
        // reward path's caches are entangled with its early-returns, and obs
        // must work even with the imitation reward off).
        private readonly float3[] m_gPrevTip = new float3[4];
        private readonly float3[] m_gPrevAnchor = new float3[4];
        private float m_gLastSample = -1f;
        private quaternion m_yawRot; // torso yaw frame, set each CollectObservations

        private void GhostLimb(VectorSensor sensor, int i, float dt)
        {
            var p = rewardProbe.GetGhostLimbPose(i);
            if (!p.Valid)
            {
                sensor.AddObservation(s_zeroGhost8); // 2 + 3 + 3 zeros
                return;
            }

            // joint dir (torso-local XZ, meters - limb-scale magnitudes)
            float2 jL = math.mul(m_yawRot, p.JointW - p.AnchorW).xz;
            sensor.AddObservation(jL.x);
            sensor.AddObservation(jL.y);

            // tip pos rel. anchor (torso-local, full 3D - foot HEIGHT matters)
            float3 tL = math.mul(m_yawRot, p.TipW - p.AnchorW);
            sensor.AddObservation(tL.x);
            sensor.AddObservation(tL.y);
            sensor.AddObservation(tL.z);

            // tip vel rel. anchor (both sides anchor-relative: root motion
            // cancels, matching the reward's velocity term; m/s / 3). dt spans
            // the decision gap - the honest sampling rate of the obs stream.
            float3 vG = dt > 1e-5f
                ? ((p.TipW - m_gPrevTip[i]) - (p.AnchorW - m_gPrevAnchor[i])) / dt
                : float3.zero;
            m_gPrevTip[i] = p.TipW;
            m_gPrevAnchor[i] = p.AnchorW;
            float3 vL = math.mul(m_yawRot, vG);
            sensor.AddObservation(vL.x / 3f);
            sensor.AddObservation(vL.y / 3f);
            sensor.AddObservation(vL.z / 3f);
        }

        private static void AddPhase(VectorSensor sensor, BackboneWaveProbe p)
        {
            if (p == null) { sensor.AddObservation(0f); sensor.AddObservation(0f); return; }
            float phi = p.Phase;
            sensor.AddObservation(math.sin(phi));
            sensor.AddObservation(math.cos(phi));
        }

        // 8 normalized floats per limb, same order as the action channels.
        private void EchoLimb(VectorSensor sensor, int i)
        {
            var g = m_limbs[i];
            float leanLim = i >= 2 ? kLeanLimArm : kLeanLim;
            sensor.AddObservation(g.bias / kBiasLim);
            sensor.AddObservation(g.ampTip / kAmpMax - 0.5f);
            sensor.AddObservation(g.ampBase / kAmpMax - 0.5f);
            sensor.AddObservation(g.freq / kFreqMax);
            sensor.AddObservation(g.lean / leanLim);
            sensor.AddObservation(g.phase / kPhaseLim);
            sensor.AddObservation(g.poseRoll / kRollLim);
            sensor.AddObservation(g.roll / kRollLim);
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            var a = actions.ContinuousActions;
            // Per-step interpolation rate from the policy: last channel maps
            // [-1,1] -> [alphaMin, alphaMax]. Because TakeActionsBetweenDecisions
            // re-delivers the buffer every physics step, blending here turns the
            // decision-step param jumps into a continuous glide at a rate the
            // policy itself chooses (cruising gait -> small alpha = smooth;
            // evasive burst -> alpha near 1 = instant, old behavior).
            float alpha = math.lerp(smoothAlphaMin, smoothAlphaMax, (a[a.Length - 1] + 1f) * 0.5f);
            if (a.Length < kActRaw) return;
            // Per-limb minimal wave parameters, 8 channels each:
            // [8i+0] ampBase [8i+1] ampTip (one-sided [0, kAmpMax])
            // [8i+2] bias (+/-1.2) | [8i+3] phase (+/-pi) | [8i+4] freq (+/-0.5 Hz)
            // [8i+5] lean (+/-0.6 legs, +/-1.2 arms) | [8i+6] poseRoll (+/-pi)
            // [8i+7] roll (+/-pi)
            for (int i = 0; i < 4; i++)
            {
                int b = i * kLimbParams;
                float leanLim = i >= 2 ? kLeanLimArm : kLeanLim;
                m_limbsT[i].ampBase = (a[b + 0] + 1f) * 0.5f * kAmpMax;
                m_limbsT[i].ampTip = (a[b + 1] + 1f) * 0.5f * kAmpMax;
                m_limbsT[i].bias = a[b + 2] * kBiasLim;
                m_limbsT[i].phase = a[b + 3] * kPhaseLim;
                m_limbsT[i].freq = a[b + 4] * kFreqMax;
                m_limbsT[i].lean = a[b + 5] * leanLim;
                m_limbsT[i].poseRoll = a[b + 6] * kRollLim;
                m_limbsT[i].roll = a[b + 7] * kRollLim;
            }
            BlendLimbs(alpha);
            PushLimbs();
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            // Calm noise: this is the manual-test path, not exploration - the
            // trainer's own exploration noise handles exploration. Small
            // enough to watch gaits without seizure, large enough to prove
            // every channel is live.
            var c = actionsOut.ContinuousActions;
            for (int i = 0; i < c.Length; i++) c[i] = UnityEngine.Random.Range(-0.05f, 0.05f);
            // Rate channel at max: manual tests keep instant response.
            c[c.Length - 1] = 1f;
        }

        // Soft external torque yaw-aligning the torso with the target bearing,
        // scaled by uprightness: upright torso gets steering help, a toppling
        // or fallen one gets none - so the assist can never hold a fallen rig
        // up or mask a fall. Runs every physics step (the wave gait itself has
        // no yaw DOF; without this the torso yaws only from foot friction).
        private void ApplyTorsoYawAssist()
        {
            if (!torsoYawAssist || rewardProbe == null) return;
            var torso = rewardProbe.TorsoBody;
            if (torso == null || !torso.Body.IsValid || rewardProbe.Target == null) return;
            var b = torso.Body;

            // Uprightness gate: torso up-axis vs world up. 0 below ~14 deg from
            // horizontal (fallen), full above ~64 deg.
            float3 upW = math.mul(b.Rotation, new float3(0f, 1f, 0f));
            float upright = math.saturate((upW.y - 0.25f) / 0.5f);
            if (upright <= 0f) return;

            // Signed heading error: torso forward -> the target's LOOK direction
            // (what the align reward pays for), both projected on XZ (yaw only;
            // pitch/roll are the fall, not the aim). Deliberately NOT the
            // bearing to the target - the move and look channels are independent,
            // and the assist must pull toward the rewarded heading, not fight it.
            // Aim: the target's LOOK direction in BOTH modes, matching the
            // align reward - reward and assist must agree or the rig is paid
            // to face away from where it is pushed to face. The look/move
            // decoupling is gone; if the rig must face where it walks instead,
            // flip BOTH this and AlignError to vTarget + pursuit * (target - COM).
            float3 aim = rewardProbe.Target.LookDirW;
            aim.y = 0f;
            float aLen = math.length(aim);
            if (aLen < 1e-4f) return;
            aim /= aLen;
            float3 fwd = math.mul(b.Rotation, new float3(0f, 0f, 1f)); // rig forward = local +Z (authored facing; matches AlignError)
            fwd.y = 0f;
            float fLen = math.length(fwd);
            float err = fLen > 1e-4f
                ? math.atan2(fwd.x * aim.z - fwd.z * aim.x, fwd.x * aim.x + fwd.z * aim.z)
                : 0f;
            // Deadband: tiny heading error should not leak torque.
            if (math.abs(err) < 0.02f) return;

            // Sign: VERIFIED in play - positive Y-torque here DECREASES the
            // yaw read (b3's torque convention is not Unity's), so the correct
            // corrective torque is +torsoYawTorque * err with err = fwdYaw -
            // aimYaw. Flipping the fwd vector (Z sign) shifts err by pi but
            // leaves dyaw/dtau unchanged, so the tau formula survives; only
            // WHICH heading is rewarded changed. fwd is the torso's +Z (the
            // rig forward convention - same as AlignError, flipped together).
            float wy = b.GetAngularVelocity().y;
            float tau = upright * (torsoYawTorque * err - torsoYawDamp * wy);
            b.ApplyTorque(new float3(0f, -tau, 0f), true);
        }

        private void FixedUpdate()
        {
            // Property read, not RewardInto(AddReward): an instance method
            // group passed as Action<float> allocates a fresh delegate EVERY
            // call - 64 instances x 60 Hz of pure gen0 GC churn.
            AddReward(rewardProbe.LastStepReward);
            ApplyTorsoYawAssist();
            m_episodeSteps++; // episode clock for the remaining-time fall penalty
            // Posture: contortion punished per second, graded - extreme folds
            // bleed fast, mild swing costs nothing.
            float extremity = (m_legL != null ? m_legL.ExtremityRad() : 0f)
                            + (m_legR != null ? m_legR.ExtremityRad() : 0f)
                            + (m_armL != null ? m_armL.ExtremityRad() : 0f)
                            + (m_armR != null ? m_armR.ExtremityRad() : 0f);
            AddReward(-limbExtremityK * extremity * Time.fixedDeltaTime);
            // Dislocation: joint anchor separation beyond the servo jitter band
            // bleeds hard per second - a dislocated limb is not a pose, it's
            // solver failure, and the policy must never profit from it.
            float dislocation = (m_legL != null ? m_legL.SepMax : 0f)
                              + (m_legR != null ? m_legR.SepMax : 0f)
                              + (m_armL != null ? m_armL.SepMax : 0f)
                              + (m_armR != null ? m_armR.SepMax : 0f);
            dislocation = math.max(0f, dislocation - 4f * dislocationTolerance); // one limb's worth of jitter band
            AddReward(-dislocationK * dislocation * Time.fixedDeltaTime);
            m_distSum += rewardProbe.TargetDist;
            m_distSteps++;
            if (rewardProbe.ConsumeFall() || rewardProbe.PoseFail)
            {
                if (rewardProbe.EpisodeTime < abortGrace)
                {
                    // Spawn grace: the rig spawns slightly off the ground and
                    // settles during the first moments - transient height dips
                    // and pose errors then are not failures.
                    ResetStreak();
                }
                else if (rewardProbe.MinimalMode)
                {
                    // DeepMimic fail: fall or sustained pose error ENDS the
                    // episode, small terminal reward. The negative-stream
                    // misery logic is a non-minimal-mode feature; here the
                    // crisp boundary is the signal (value bootstrapping does
                    // the rest), and a small terminal avoids re-creating a
                    // nosedive incentive that the old big penalties carried.
                    AddReward(-0.5f);
                    m_episodeSurvived = false;
                    ResetStreak();
                    EndEpisode();
                    return;
                }
                else
                {
                    // Fall stamp only - falls do NOT end the episode. The rig lies
                    // there accruing the negative stream; since the target roams,
                    // the deviation abort (below) ends the episode once the lying
                    // rig is > abortDist from the target. No nosedive incentive:
                    // the ONLY early exit is a state reached by persistently
                    // failing to follow, not by a quick destructive act.
                    float remaining = math.saturate(1f - (float)m_episodeSteps / math.max(MaxStep, 1));
                    AddReward(math.max(-3f, -20f * remaining));
                    m_episodeSurvived = false;
                    ResetStreak(); // "consecutive" - a fall breaks the streak
                }
            }

            // Deviation abort: the single episode-ending condition besides
            // MaxStep. ONE 3D distance (COM vs the expected standing COM point
            // above the target) covers planar straying, airborne escape and
            // collapse - no separate axes to tune.
            if (rewardProbe.TargetDist > deviateAbortDist)
            {
                AddReward(-3f);
                m_episodeSurvived = false;
                ResetStreak();
                EndEpisode();
                return;
            }
            // MaxStep (inspector) handles the episode time limit.
        }

        // --- curriculum lesson stepper (static: shared across batch agents) ---
        // 4.1.0 EnvironmentParameters is runtime-read-only: the trainer pushes
        // reset parameters, we cannot. So kDifficulty IS the default source;
        // GetWithDefault returns it unless a trainer sent an explicit override.
        private const int kSuccessesPerLesson = 5;
        private const float kDifficultyStep = 0.1f;
        private static float kDifficulty;      // local lesson state
        private static int kSuccessStreak;
        private static bool s_callbackRegistered;

        // Canonical no-domain-reload static reset: runs on every play session
        // start (SubsystemRegistration fires even when domain reload is off).
        // Without it, difficulty promoted in one training run leaks into the
        // next session at stage 0.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void ResetSessionStatics()
        {
            kDifficulty = 0f;
            kSuccessStreak = 0;
            s_callbackRegistered = false;
        }

        public static float Difficulty => kDifficulty;

        private static void ReportSuccess()
        {
            kSuccessStreak++;
            if (kSuccessStreak >= kSuccessesPerLesson && kDifficulty < 1f)
            {
                kSuccessStreak = 0;
                kDifficulty = math.clamp(kDifficulty + kDifficultyStep, 0f, 1f);
                // No push needed: the next OnEpisodeBegin reads kDifficulty as
                // the GetWithDefault fallback. A connected trainer overrides it.
            }
        }

        // Static across all batch agents: kSuccessesPerLesson is the TOTAL
        // clean episodes required to promote (any agent's fall breaks it),
        // not per-agent - otherwise N instances promote N times faster.
        private static void ResetStreak() => kSuccessStreak = 0;

        // Trainer-side overrides arrive asynchronously; register once to adopt them.
        private static void EnsureTrainerCallback()
        {
            if (s_callbackRegistered) return;
            var academy = Academy.Instance;
            if (academy == null) return;
            academy.EnvironmentParameters.RegisterCallback("task_difficulty", v => kDifficulty = math.clamp(v, 0f, 1f));
            s_callbackRegistered = true;
        }
        private bool m_episodeSurvived = true;
        private int m_episodeSteps;
        private float m_distSum;   // tracking-distance integral over the episode
        private int m_distSteps;

        private void BlendLimbs(float alpha)
        {
            for (int i = 0; i < 4; i++)
            {
                m_limbs[i].ampBase = math.lerp(m_limbs[i].ampBase, m_limbsT[i].ampBase, alpha);
                m_limbs[i].ampTip = math.lerp(m_limbs[i].ampTip, m_limbsT[i].ampTip, alpha);
                m_limbs[i].bias = math.lerp(m_limbs[i].bias, m_limbsT[i].bias, alpha);
                m_limbs[i].phase = math.lerp(m_limbs[i].phase, m_limbsT[i].phase, alpha);
                m_limbs[i].freq = math.lerp(m_limbs[i].freq, m_limbsT[i].freq, alpha);
                m_limbs[i].lean = math.lerp(m_limbs[i].lean, m_limbsT[i].lean, alpha);
                m_limbs[i].poseRoll = math.lerp(m_limbs[i].poseRoll, m_limbsT[i].poseRoll, alpha);
                m_limbs[i].roll = math.lerp(m_limbs[i].roll, m_limbsT[i].roll, alpha);
            }
        }

        private void PushLimbs()
        {
            PushLimb(m_legL, 0, -1f);
            PushLimb(m_legR, 1, +1f);
            PushLimb(m_armL, 2, -1f);
            PushLimb(m_armR, 3, +1f);
        }

        // sideSign mirrors the lean convention: left negated, so positive lean
        // = outward tilt on BOTH sides (matches the tester). The frame-rotation
        // channels roll and poseRoll use the SAME mirror: both axes are
        // shared/mirrored between sides (roll's axis = base direction, shared;
        // poseRoll's axis = projected chord, mirrored), and under a mirror the
        // rotation sense flips handedness - so left = -value yields true
        // L/R-symmetric envelopes from one channel. Twist is GENOME-owned:
        // NaN leaves the probe's drawn twistAmp/twistPhase untouched (the
        // old behavior forced twistAmp to 0 - incompatible with a genome).
        private void PushLimb(BackboneWaveProbe p, int i, float sideSign)
        {
            if (p == null) return;
            var g = m_limbs[i];
            p.SetRuntimeParams(g.freq, g.ampTip, g.bias, m_limbWaven[i],
                float.NaN, float.NaN, g.ampBase, sideSign * g.roll, sideSign * g.poseRoll);
            p.BaseDirXTilt = sideSign * g.lean;
            p.SetWavePhaseOffset(g.phase);
        }
    }
}