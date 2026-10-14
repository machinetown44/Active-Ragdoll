using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>Grid batch spawner: N bipeds, each with its own RewardProbe +
    /// agent + LocomotionTarget, spaced so instances never interact physically.
    /// One shared Box3D world (auto-created by the first demo) and one shared
    /// floor. Drop on an empty GameObject, set instanceCount, play.
    /// All refs are wired explicitly - FindAnyObjectByType is ambiguous with
    /// N instances, so auto-find is bypassed entirely.</summary>
    public sealed class BipedBatch : MonoBehaviour
    {
        [SerializeField, Min(1), Tooltip("Total instances (grid is ceil(sqrt(n)) x ceil(n/x)).")]
        private int instanceCount = 64;
        [SerializeField, Tooltip("Grid cell size (m). 20 m >= 2x limb reach: instances never collide.")]
        private float cellSize = 20f;
        [SerializeField, Tooltip("Force Heuristic-Only behavior (random-noise actions). Play-mode debugging without a trainer.")]
        private bool heuristicOnly = false;
        [SerializeField, Tooltip("Optional animated humanoid reference prefab (ghost). Spawned per instance as a kinematic imitation reference that chases the same target - no physics, no reward, no termination impact.")]
        private GameObject referencePrefab;

        [SerializeField, Tooltip("Gravity scale applied to the shared Box3D world (1 = -9.81 m/s^2).")]
        private float gravityScale = 1f / 3f;

        [Header("Genome: static policy-invisible params (per limb, fixed for the instance's life)")]
        [SerializeField, Tooltip("Draw independent per-limb genomes (wavenumber, twist, servo shaping) and assign them to each instance's agent. The policy never sees these; reward differences between instances are the selection signal. Off = authored construction defaults.")]
        private bool genomeRandomize = true;
        [SerializeField, Tooltip("Wavenumber, multiples of the authored 8.47.")]
        private Vector2 genomeWavenumberMul = new(0.85f, 1.15f);
        [SerializeField, Tooltip("Twist amplitude (rad) - fixed roll perturbation the policy cannot command.")]
        private Vector2 genomeTwistAmp = new(0f, 0.15f);
        [SerializeField, Tooltip("Twist phase (deg); only shapes the limb when twistAmp > 0.")]
        private Vector2 genomeTwistPhaseDeg = new(-180f, 180f);
        [SerializeField, Tooltip("Feedforward gate angle (rad).")]
        private Vector2 genomeFfGate = new(0.1f, 0.4f);
        [SerializeField, Tooltip("Endpoint correction gain (1/s).")]
        private Vector2 genomeEndpointGain = new(0.1f, 0.4f);
        [SerializeField, Tooltip("Endpoint correction rate clamp (rad/s).")]
        private Vector2 genomeEndpointMaxRate = new(2.5f, 10f);
        [SerializeField] private Vector2 genomeKp = new(0.7f, 1.4f);
        [SerializeField] private Vector2 genomeKd = new(0.02f, 0.1f);
        [SerializeField] private Vector2 genomeZeta = new(0.85f, 1.2f);
        [SerializeField, Tooltip("Lookahead (s) - pipeline-delay compensation horizon.")]
        private Vector2 genomeLookahead = new(0.01f, 0.028f);
        [SerializeField, Tooltip("|w| command ceiling (rad/s).")]
        private Vector2 genomeWCeiling = new(60f, 150f);

        [Header("Genome evolution (PBT-lite selection over the population)")]
        [SerializeField, Tooltip("Periodically clone+mutate the top performers' genomes over the worst. Selection unit = instance (all 4 limbs move together, fitness is per instance); fitness = mean episode reward over the agent's trailing window. All instances share one curriculum difficulty, so rewards are comparable.")]
        private bool genomeEvolve = true;
        [SerializeField, Min(10f), Tooltip("Seconds between selection rounds (wall clock). Must exceed the time an instance needs to finish ~evolveMinEpisodes episodes.")]
        private float evolveIntervalSec = 300f;
        [SerializeField, Min(3), Tooltip("Minimum ended episodes per instance before ANY selection round runs - below this, reward differences are fall-luck, not genome quality.")]
        private int evolveMinEpisodes = 20;
        [SerializeField, Range(0.05f, 0.5f), Tooltip("Fraction of the population replaced each round (cloned from the top, mutated). 0.25 on 64 instances = worst 16 cloned from best 16.")]
        private float exploitFraction = 0.25f;
        [SerializeField, Range(0f, 0.5f), Tooltip("Multiplicative mutation sigma: each param scales by 1 + gaussian*sigma (clamped to its draw range). Twist phase mutates additively.")]
        private float genomeMutateSigma = 0.08f;

        private readonly System.Collections.Generic.List<WaveLocomotionAgent> m_agents =
            new System.Collections.Generic.List<WaveLocomotionAgent>();
        private float m_nextEvolve;

        [Header("Tuning mode")]
        [SerializeField, Tooltip("Sign-convention tuning: ONE rig only, no reward/agent/target/ghost - just the demo + tip traces. The torso is suspended by a world-anchored weld so servo reactions cannot translate it. Set instanceCount back manually when turning this off (the field is not modified).")]
        private bool tuningMode = false;
        [SerializeField, Min(0f), Tooltip("Suspension spring (Hz). 0 = perfectly rigid weld (torso frozen in place - cleanest read of the wave geometry); >0 = soft hang (torso can yield a little under servo reaction).")]
        private float suspensionHertz = 0f;

        private bool m_debugLogsOn;

        private void Awake()
        {
            // Reduced gravity: balance is the bottleneck skill; 1/3 g widens
            // the recoverable tilt window so the policy can learn gait first,
            // then gravity scales back up. Runs before the first body spawns.
            var world = Box3D.Hybrid.Box3DWorld.Instance;
            if (world != null)
                world.World.SetGravity(new Vector3(0f, -9.81f * gravityScale, 0f));

            int cols = Mathf.CeilToInt(Mathf.Sqrt(instanceCount));
            int rows = Mathf.CeilToInt((float)instanceCount / cols);
            Vector3 origin = transform.position
                - new Vector3(cols - 1, 0f, rows - 1) * (cellSize * 0.5f); // centered grid

            if (tuningMode)
            {
                // Single suspended rig for sign-convention tuning: no training
                // components at all, so nothing can push the limbs except the
                // wave servos themselves.
                CreateSharedFloor(cellSize, cellSize);
                Box3DArticulation.WaveDiagnostics.Verbose = true;
                SpawnTuningRig(transform.position);
                return;
            }

            CreateSharedFloor(cols * cellSize, rows * cellSize);

            // Heuristic-only runs are manual debugging sessions: enable the
            // full diagnostic stream BEFORE spawning (rig-created logs fire
            // inside SpawnInstance). Training runs stay quiet.
            if (heuristicOnly) Box3DArticulation.WaveDiagnostics.Verbose = true;

            for (int i = 0; i < instanceCount; i++)
            {
                int col = i % cols, row = i / cols;
                SpawnInstance(origin + new Vector3(col * cellSize, 0f, row * cellSize), i);
            }
            m_nextEvolve = Time.time + evolveIntervalSec; // first selection round

            if (m_debugLogsOn)
                Debug.Log($"[BipedBatch] HEURISTIC-ONLY diagnostic mode: {instanceCount} instances spawned; " +
                          "spawn + dislocation diagnostics are ON (training runs stay quiet)");
        }

        // One floor for the whole grid; per-demo floors are disabled (they would
        // all overlap at the origin).
        private void CreateSharedFloor(float sx, float sz)
        {
            var go = new GameObject("BatchFloor");
            go.transform.SetParent(transform, false);
            go.transform.position = transform.position + Vector3.down * 0.5f; // top face at y=0
            go.SetActive(false); // runtime-safe body creation: defer Awakes
            var shape = go.AddComponent<Box3D.Hybrid.Box3DBoxShape>();
            SetPrivate(shape, "Size", new Vector3(sx, 1f, sz));
            var body = go.AddComponent<Box3D.Hybrid.Box3DBody>();
            body.BodyType = Box3D.Hybrid.Box3DBodyType.Static;
            // Terrain layer BEFORE activation: Box3DBody.Awake bakes the shape's filter
            // category from the layer (see PinnedBipedDemo.CreateFloor).
            int terrain = LayerMask.NameToLayer(FootStabilizer.TerrainLayerName);
            if (terrain >= 0) go.layer = terrain;
            go.SetActive(true);
        }

        // Tuning rig: demo only + tip traces, torso welded to the world.
        // The weld is added AFTER the demo's torso exists (CreateDemo runs in
        // Awake); Box3DJoint creates the native joint in its own Start, so the
        // weld freezes whatever pose the torso has by the next frame.
        private void SpawnTuningRig(Vector3 pos)
        {
            var go = new GameObject("TuningRig");
            go.transform.SetParent(transform, false);
            go.transform.position = pos;
            go.SetActive(false);

            var demo = go.AddComponent<PinnedBipedDemo>();
            SetPrivate(demo, "floorEnabled", false); // shared tuning floor

            go.SetActive(true); // demo.Awake -> CreateDemo (torso body exists now)

            // Analytic tip prediction on every limb: the dashed idle-wave tip
            // path the probe already owns. Compare against the traced real tip
            // to spot sign-convention drift per limb.
            demo.LegL.drawTipPrediction = true;
            demo.LegR.drawTipPrediction = true;
            demo.ArmL.drawTipPrediction = true;
            demo.ArmR.drawTipPrediction = true;

            // Output-space convention tuner: one slider per channel, mirrored
            // onto L/R exactly like the agent's PushLimb. Runs with no agent,
            // so nothing else writes these channels.
            var tuner = go.AddComponent<ConventionTuner>();
            SetPrivate(tuner, "demo", demo);

            var torso = demo.Torso;
            if (torso == null) { Debug.LogError("[BipedBatch] tuning rig: torso missing after CreateDemo"); return; }
            // World-anchored weld (ConnectedBody null = fixed point): linear
            // + angular spring. Hertz 0 = rigid, else a damped hang.
            var weld = torso.gameObject.AddComponent<Box3D.Hybrid.Box3DFixedJoint>();
            SetPrivate(weld, "LinearHertz", suspensionHertz);
            SetPrivate(weld, "AngularHertz", suspensionHertz);
        }

        private void SpawnInstance(Vector3 pos, int index)
        {
            // Inactive until all private refs are wired: component OnEnable
            // auto-find would be ambiguous (any demo/target matches any probe).
            var go = new GameObject($"Biped{index}");
            go.transform.SetParent(transform, false);
            go.transform.position = pos;
            go.SetActive(false);

            var demo = go.AddComponent<PinnedBipedDemo>();
            SetPrivate(demo, "floorEnabled", false); // shared batch floor instead
            var probe = go.AddComponent<RewardProbe>();
            var agent = go.AddComponent<WaveLocomotionAgent>();
            SetPrivate(agent, "forceHeuristicOnly", heuristicOnly);
            if (heuristicOnly) m_debugLogsOn = true;

            var targetGo = new GameObject($"Target{index}");
            targetGo.transform.SetParent(transform, false);
            targetGo.transform.position = pos;
            var target = targetGo.AddComponent<LocomotionTarget>();
            SetPrivate(target, "floorSize", new Vector2(cellSize, cellSize)); // wander stays inside its own cell

            // Imitation reference ghost: parented under the BATCH root, not
            // the instance root - ClearGenerated sweeps the instance root's
            // children on every rig rebuild and would destroy the ghost. The
            // persistent Biped{i} root serves as the spawn anchor instead.
            if (referencePrefab != null)
            {
                var ghost = Instantiate(referencePrefab, pos, Quaternion.identity, transform);
                ghost.name = $"Reference{index}";
                var imitation = ghost.AddComponent<ImitationReference>();
                SetPrivate(imitation, "target", target);
                SetPrivate(imitation, "spawnAnchor", go.transform);
                SetPrivate(agent, "imitationReference", imitation);
                SetPrivate(probe, "imitationRef", imitation);
            }

            SetPrivate(probe, "demo", demo);
            SetPrivate(probe, "target", target);
            SetPrivate(agent, "demo", demo);
            SetPrivate(agent, "rewardProbe", probe);
            SetPrivate(agent, "target", target);
            agent.MaxStep = 1800; // 30 s at 60 Hz physics

            // Genome: independent draws per limb, stored on the agent and
            // applied by its ResolveLimbs (survives pooled resets, re-applied
            // after rebuilds). Drawn while inactive so Initialize lands on it.
            if (genomeRandomize)
                for (int limb = 0; limb < 4; limb++)
                    agent.SetGenome(limb, DrawGenome());
            m_agents.Add(agent); // registry for the selection loop

            go.SetActive(true); // OnEnable/Acquire: refs already wired, no ambiguity
        }

        private void Update()
        {
            if (!genomeEvolve || tuningMode || m_agents.Count == 0) return;
            if (Time.time < m_nextEvolve) return;
            m_nextEvolve = Time.time + evolveIntervalSec;
            EvolveGenomes();
        }

        /// <summary>PBT-lite round: worst exploitFraction cloned from the top
        /// exploitFraction (random pick within the top), each limb genome
        /// mutated independently. Async by design: SetGenome only stores -
        /// the loser's in-flight episode finishes on its old genome and the
        /// mutant lands at its next episode begin (ResolveLimbs applies).
        /// Guards: every instance must have evolveMinEpisodes ended episodes
        /// (below that, ranking measures fall luck), and top/bottom sets must
        /// stay disjoint so a genome never replaces itself.</summary>
        private void EvolveGenomes()
        {
            int n = m_agents.Count;
            int k = Mathf.Clamp(Mathf.RoundToInt(n * exploitFraction), 1, n / 2);
            if (n < 2 * k) return;
            for (int i = 0; i < n; i++)
                if (m_agents[i].FitnessSamples < evolveMinEpisodes) return;

            var order = new System.Collections.Generic.List<(float fit, int idx)>(n);
            for (int i = 0; i < n; i++) order.Add((m_agents[i].GenomeFitness(), i));
            order.Sort((a, b) => b.fit.CompareTo(a.fit)); // descending

            int replaced = 0;
            for (int b = 0; b < k; b++)
            {
                var loser = m_agents[order[n - 1 - b].idx];
                var winner = m_agents[order[UnityEngine.Random.Range(0, k)].idx];
                for (int limb = 0; limb < 4; limb++)
                    loser.SetGenome(limb, MutateGenome(winner.GetGenome(limb)));
                replaced++;
            }
            Debug.Log($"[BipedBatch] genome evolve: {replaced} instances cloned+mutated from top {k} " +
                      $"| best fitness {order[0].fit:F1}, median {order[n / 2].fit:F1}, worst {order[n - 1].fit:F1}");
        }

        private WaveGenome MutateGenome(in WaveGenome g)
        {
            float sigma = genomeMutateSigma;
            float Scale(float v) => v * (1f + Gaussian() * sigma);   // multiplicative: zeros stay zero
            float Clamp(float v, Vector2 r) => Mathf.Clamp(v, r.x, r.y);
            float phase = g.twistPhase + Gaussian() * sigma;         // additive: it's an angle
            phase = Mathf.Repeat(phase + Mathf.PI, 2f * Mathf.PI) - Mathf.PI;
            return new WaveGenome(
                Clamp(Scale(g.wavenumber), new Vector2(8.47f * genomeWavenumberMul.x, 8.47f * genomeWavenumberMul.y)),
                Clamp(Scale(g.twistAmp), genomeTwistAmp),
                phase,
                Clamp(Scale(g.ffGateAngle), genomeFfGate),
                Clamp(Scale(g.endpointGain), genomeEndpointGain),
                Clamp(Scale(g.endpointMaxRate), genomeEndpointMaxRate),
                Clamp(Scale(g.kp), genomeKp),
                Clamp(Scale(g.kd), genomeKd),
                Clamp(Scale(g.zeta), genomeZeta),
                Clamp(Scale(g.lookahead), genomeLookahead),
                Clamp(Scale(g.wCeiling), genomeWCeiling));
        }

        private static float Gaussian()
        {
            float u = 1f - UnityEngine.Random.value; // (0,1]: log safe
            float v = UnityEngine.Random.value;
            return Mathf.Sqrt(-2f * Mathf.Log(u)) * Mathf.Sin(2f * Mathf.PI * v);
        }

        private WaveGenome DrawGenome()
        {
            return new WaveGenome(
                wavenumber: 8.47f * UnityEngine.Random.Range(genomeWavenumberMul.x, genomeWavenumberMul.y),
                twistAmp: UnityEngine.Random.Range(genomeTwistAmp.x, genomeTwistAmp.y),
                twistPhase: UnityEngine.Random.Range(genomeTwistPhaseDeg.x, genomeTwistPhaseDeg.y) * Mathf.Deg2Rad,
                ffGateAngle: UnityEngine.Random.Range(genomeFfGate.x, genomeFfGate.y),
                endpointGain: UnityEngine.Random.Range(genomeEndpointGain.x, genomeEndpointGain.y),
                endpointMaxRate: UnityEngine.Random.Range(genomeEndpointMaxRate.x, genomeEndpointMaxRate.y),
                kp: UnityEngine.Random.Range(genomeKp.x, genomeKp.y),
                kd: UnityEngine.Random.Range(genomeKd.x, genomeKd.y),
                zeta: UnityEngine.Random.Range(genomeZeta.x, genomeZeta.y),
                lookahead: UnityEngine.Random.Range(genomeLookahead.x, genomeLookahead.y),
                wCeiling: UnityEngine.Random.Range(genomeWCeiling.x, genomeWCeiling.y));
        }

        private static void SetPrivate(object instance, string name, object value)
        {
            var f = instance.GetType().GetField(name,
                System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Public);
            f?.SetValue(instance, value);
        }
    }
}
