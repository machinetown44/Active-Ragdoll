using Box3D.Hybrid;
using System;
using Box3D;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>Two BackboneWaveProbe limbs attached as LEGS to a shared
    /// floating torso. Repurposed from the broken pinned biped: instead of
    /// spline chains, each leg is a full wave probe (external-root mode) -
    /// the wave drives the leg, the servo reactions push the torso around.</summary>
    [DisallowMultipleComponent]
    public sealed class PinnedBipedDemo : MonoBehaviour
    {
        [SerializeField] private bool createOnAwake = true;

        [Header("Torso")]
        [SerializeField] private Vector3 torsoSize = new Vector3(0.36f, 0.7f, 0.3f);
        [SerializeField] private float torsoLift = 1.1f;

        [Header("Legs")]
        [SerializeField] private float hipOffsetX = 0.16f;
        [SerializeField] private float boneLength = 0.4f;
        [SerializeField, Tooltip("Leg bone cross-section (m). Legs thicker than arms.")]
        private float legThickness = 0.11f;
        [SerializeField, Tooltip("Leg bone masses (kg): upper, fore.")]
        private Vector2 legMasses = new Vector2(3.2f, 2.6f);
        [SerializeField, Tooltip("Leg joint torque limits (N*m): hip, knee. ~2-3x arm - human proportions.")]
        private Vector2 legTorques = new Vector2(250f, 200f);
        [Header("Arms")]
        [SerializeField, Tooltip("Shoulder X offset from torso center (half torso width keeps the arm at the box edge).")] 
        private float shoulderOffsetX = 0.25f; // default matches old torsoSize.x * 0.5f
        [SerializeField] private float armBoneLength = 0.35f;
        [SerializeField, Tooltip("Arm bone cross-section (m).")]
        private float armThickness = 0.07f;
        [SerializeField, Tooltip("Arm bone masses (kg): upper, fore.")]
        private Vector2 armMasses = new Vector2(1.4f, 1.1f);
        [SerializeField, Tooltip("Arm joint torque limits (N*m): shoulder, elbow. Human-scale; arms are reaction mass, not legs.")]
        private Vector2 armTorques = new Vector2(90f, 60f);
        [SerializeField] private float armFreqHz = 0;
        [SerializeField] private float armBias = -0.03f;
        [SerializeField] private float armAmpTip = 1.42f;
        [SerializeField] private float freqHz = 0f;
        [SerializeField] private float bias = -0.03f; // flexed mean pose (knee bend)
        [SerializeField] private float ampTip = 1.42f;

        [Header("Ground contact")]
        [SerializeField] private bool floorEnabled = true;
        [SerializeField, Tooltip("Torso rotation locked to yaw (X/Z tilt zeroed every step). Debug aid - OFF for agent training: it falsifies the physics and fights the policy.")]
        private bool lockTorsoToYaw = false;

        private Box3DBody m_torso;

        /// <summary>The torso body (null until the demo is created). Reward
        /// probe / future agent read the COM from this.</summary>
        public Box3DBody Torso => m_torso;

        // Runs AFTER Box3DWorld's step (world is execution order -100), so the
        // yaw enforcement below corrects this step's solver output.
        private void FixedUpdate()
        {
            if (!lockTorsoToYaw || m_torso == null || !m_torso.Body.IsValid) return;

            var b = m_torso.Body;
            // Rebuild the orientation from its yaw only: project the body's
            // forward onto XZ. Position untouched (free), X/Z tilt removed.
            float3 fwd = math.mul(b.Rotation, new float3(0f, 0f, 1f)); // RIG-legacy: yaw-lock preserves the authored +Z frame - flipping this would rotate the rig 180 deg and reorient every limb
            float yaw = math.atan2(fwd.x, fwd.z);
            b.SetTransform(b.Position, quaternion.RotateY(yaw));

            // Kill tilt spin so the solver doesn't keep feeding it back.
            float3 w = b.AngularVelocity;
            b.AngularVelocity = new float3(0f, w.y, 0f);
        }

        private void Awake()
        {
            if (createOnAwake) CreateDemo();
        }

        [ContextMenu("Create Demo")]
        public void CreateDemo()
        {
            ClearGenerated();
            if (!FindAnyObjectByType<Box3DWorld>()) new GameObject("Box3D World").AddComponent<Box3DWorld>();

            Vector3 root = transform.position + Vector3.up * torsoLift;
            m_spawnRoot = root; m_spawnCaptured = true; // pooled-reset datum (ResetToSpawn)
            // Per-episode noise: verbose (heuristic/tester) sessions only.
            if (Box3DArticulation.WaveDiagnostics.Verbose)
                Debug.Log($"[PinnedBipedDemo:{name}] rig created @ {root:F2} (torso {torsoSize}, legs {boneLength} m / arms {armBoneLength} m)");

            // Runtime body creation pattern (per Box3D package mechanics):
            // add components while the GameObject is INACTIVE - Awakes are
            // deferred, so Box3DBody.Awake's one-shot shape gather sees the
            // shape added first. Body-then-shape on an active GO leaves the
            // shape unattached (body gathers nothing, shape defers to it) -
            // a shapeless dynamic body is never integrated => static torso.
            Transform torso = NewBodyGo("Torso", root).transform;
            Box3DBoxShape shape = torso.gameObject.AddComponent<Box3DBoxShape>();
            SetPrivateField(shape, "Size", torsoSize); // COM frame + collider come from the shape, NOT localScale
            torso.gameObject.AddComponent<Box3DBody>();
            var torsoBody = FinishBody(torso.gameObject, Box3DBodyType.Dynamic);
            
            m_torso = torsoBody;
            m_torso.Body.SetAngularDamping(500);

            if (floorEnabled) CreateFloor();

            // Two wave limbs as legs, one per hip. SAME bend axis on both:
            // knees bend the same world direction (mirroring the axis flips
            // the bend plane and the waves point opposite ways in Z).
            // Left leg: NEGATIVE bend axis and NEGATIVE wavenumber.
            CreateLeg("LeftLeg", torsoBody, new float3(-hipOffsetX, -torsoSize.y * 0.5f, 0f), -1f);
            CreateLeg("RightLeg", torsoBody, new float3(hipOffsetX, -torsoSize.y * 0.5f, 0f), +1f);            // Arms in T-POSE: base direction horizontal (+-X), so the chain
            // spawns straight out to the side. Same wave family as the legs;
            // left arm is the mirrored twin (negative twistWavenumber).
            CreateArm("LeftArm", torsoBody, new float3(-shoulderOffsetX, torsoSize.y * 0.4f, 0f), -1f);
            CreateArm("RightArm", torsoBody, new float3(shoulderOffsetX, torsoSize.y * 0.4f, 0f), +1f);
        }

        private void CreateFloor()
        {
            GameObject floor = NewBodyGo("Floor", new Vector3(0f, -0.5f, 0f)); // top face at y=0
            // Dedicated layer: the foot movers' CollideMover queries mask to this layer only,
            // so the tip spheres never hit the rig's own bodies. Default-layer collision matrix
            // is untouched - rigs still collide with the floor normally.
            // MUST be set BEFORE FinishBody: Box3DBody.Awake bakes the shape's filter category
            // from the layer at activation - assigning after would leave CategoryBits on the
            // old layer and the foot query would never match.
            int terrain = LayerMask.NameToLayer(FootStabilizer.TerrainLayerName);
            if (terrain >= 0) floor.layer = terrain;
            var shape = floor.AddComponent<Box3DBoxShape>();
            SetPrivateField(shape, "Size", new Vector3(40f, 1f, 40f));
            floor.AddComponent<Box3DBody>();
            FinishBody(floor, Box3DBodyType.Static);
        }

        /// Live limb probes, captured during CreateDemo. Consumers (the agent)
        /// must use these instead of name lookup: Transform.Find matches
        /// INACTIVE children too, so after an episode reset it binds the
        /// deactivated (pending-destroy) old rig and pushes into corpses.
        public BackboneWaveProbe LegL, LegR, ArmL, ArmR;

        private void CreateLeg(string name, Box3DBody torsoBody, float3 anchorLocal, float sign)
        {
            var go = new GameObject(name);            go.transform.SetParent(transform, true);
            go.transform.position = torsoBody.transform.position; // pose comes from the torso at Start

            var probe = go.AddComponent<BackboneWaveProbe>();
            probe.FootContact = true; // legs get the passive foot-contact stabilizer
            SetPrivateField(probe, "externalRoot", torsoBody);
            SetPrivateField(probe, "rootAnchorLocal", anchorLocal);
            SetPrivateField(probe, "boneLength", boneLength);
            SetPrivateField(probe, "boneThickness", legThickness);
            SetPrivateField(probe, "upperMassKg", legMasses.x);
            SetPrivateField(probe, "foreMassKg", legMasses.y);
            SetPrivateField(probe, "shoulderTorqueLimit", legTorques.x);
            SetPrivateField(probe, "elbowTorqueLimit", legTorques.y);
            SetPrivateField(probe, "elbowMinDeg", 2f);
            SetPrivateField(probe, "rootHeight", 0f);
            SetPrivateField(probe, "baseDirection", Vector3.down);
            SetPrivateField(probe, "bendAxis", new Vector3(1, 0f, 0f));
            SetPrivateField(probe, "freqHz", freqHz);
            SetPrivateField(probe, "wavenumber", 8.47f);
            SetPrivateField(probe, "phase", 0f);
            SetPrivateField(probe, "ampBase", 0.28f);
            // Minimal wave set: no twist (see CreateArm).
            SetPrivateField(probe, "twistAmp", 0f);
            SetPrivateField(probe, "bias", bias);
            SetPrivateField(probe, "ampTip", ampTip);
            if (sign < 0f) LegL = probe; else LegR = probe;
        }

        private void CreateArm(string name, Box3DBody torsoBody, float3 anchorLocal, float sign)
        {
            var go = new GameObject(name);
            go.transform.SetParent(transform, true);
            go.transform.position = torsoBody.transform.position;

            var probe = go.AddComponent<BackboneWaveProbe>();
            SetPrivateField(probe, "externalRoot", torsoBody);
            SetPrivateField(probe, "rootAnchorLocal", anchorLocal);
            SetPrivateField(probe, "boneLength", armBoneLength);
            SetPrivateField(probe, "boneThickness", armThickness);
            SetPrivateField(probe, "upperMassKg", armMasses.x);
            SetPrivateField(probe, "foreMassKg", armMasses.y);
            SetPrivateField(probe, "shoulderTorqueLimit", armTorques.x);
            SetPrivateField(probe, "elbowTorqueLimit", armTorques.y);
            SetPrivateField(probe, "rootHeight", 0f);
            SetPrivateField(probe, "baseDirection", Vector3.right*sign);              // same as legs
            SetPrivateField(probe, "bendAxis", new Vector3(0, 0f, -1f*sign));      // elbow bends opposite to the old convention (axis flip, not limits: the plan commands positive bend in the hinge frame, so limits [−10,150] must stay put and the frame mirrors)
            SetPrivateField(probe, "freqHz", armFreqHz);
            SetPrivateField(probe, "wavenumber", 2.47f);
            SetPrivateField(probe, "phase", 0f);
            SetPrivateField(probe, "ampBase", 0.28f);
            // Minimal wave set: no twist - on a 2-link chain lean + pose roll
            // span the full plane-orientation space, twist is redundant.
            SetPrivateField(probe, "twistAmp", 0.3f);
            SetPrivateField(probe, "bias", armBias);
            SetPrivateField(probe, "ampTip", armAmpTip);
            if (sign < 0f) ArmL = probe; else ArmR = probe;
        }

        // Inactive-GameObject helpers: the runtime-safe body creation pattern.
        private GameObject NewBodyGo(string name, Vector3 position)
        {
            GameObject go = new GameObject(name);
            go.transform.SetParent(transform, true);
            go.transform.position = position;
            go.transform.localScale = Vector3.one;
            go.SetActive(false); // defer Awakes: Box3DBody.Awake gathers shapes exactly once
            return go;
        }

        private Box3DBody FinishBody(GameObject go, Box3DBodyType type)
        {
            var body = go.GetComponent<Box3DBody>();
            body.BodyType = type; // field only until activation; Awake creates the native body with it
            go.SetActive(true);   // Awakes run: shape defers to the body, body gathers + attaches it
            return body;
        }

        private Transform CreateBody(string name, Vector3 position)
        {
            GameObject body = new GameObject(name); body.transform.SetParent(transform, true); body.transform.position = position;
            body.transform.localScale = Vector3.one;
            body.AddComponent<Box3DBody>().BodyType = Box3DBodyType.Dynamic; return body.transform;
        }
        /// Public hook for the agent's episode reset (rebuilds the rig).
        public void ClearGeneratedPublic() => ClearGenerated();

        /// <summary>Pooled episode reset: teleport the existing rig back to
        /// its spawn pose instead of destroying and rebuilding it. The torso
        /// returns to (transform.position + up*torsoLift, identity rotation)
        /// - exactly where CreateDemo put it - and each limb teleports back
        /// to its captured spawn pose. Returns false when the rig cannot be
        /// trusted (not built yet, a limb failed to spawn, bodies invalid):
        /// the caller must fall back to ClearGeneratedPublic + CreateDemo.</summary>
        public bool ResetToSpawn()
        {
            if (!m_spawnCaptured || m_torso == null || !m_torso.Body.IsValid) return false;
            // Probes first (their spawn poses assume the torso back at spawn),
            // then the torso. Each teleport is absolute, order is not load-bearing.
            bool ok = ProbeReset(LegL) && ProbeReset(LegR) && ProbeReset(ArmL) && ProbeReset(ArmR);
            if (!ok) return false;
            m_torso.Position = m_spawnRoot;
            m_torso.Rotation = Quaternion.identity;
            var tb = m_torso.Body;
            tb.LinearVelocity = 0f;
            tb.AngularVelocity = 0f;
            return true;
        }
        private static bool ProbeReset(BackboneWaveProbe p) => p != null && p.ResetToSpawn();
        private Vector3 m_spawnRoot;
        private bool m_spawnCaptured;

        private void ClearGenerated()
        {
            for (int i = transform.childCount - 1; i >= 0; i--)
            {
                GameObject go = transform.GetChild(i).gameObject;
                if (Application.isPlaying)
                {
                    // Destroy() is deferred to end of frame - with an immediate
                    // rebuild (episode reset) the new rig would spawn inside the
                    // still-live old bodies for one step and the solver would
                    // explode. SetActive(false) pulls Box3D bodies out of the
                    // simulation instantly; the deferred Destroy cleans up.
                    go.SetActive(false);
                    Destroy(go);
                }
                else DestroyImmediate(go);
            }
        }

        private static void SetPrivateField<T>(object instance, string name, T value)
        {
            var field = instance.GetType().GetField(name, System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic);
            if (field == null) throw new MissingFieldException(instance.GetType().Name, name); field.SetValue(instance, value);
        }
    }
}
