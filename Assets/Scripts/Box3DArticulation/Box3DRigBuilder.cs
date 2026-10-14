using System;
using System.Collections.Generic;
using Box3D;
using Box3D.Hybrid;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Data-driven Box3D rig builder: an inspector-editable segment tree
    /// (bodies + joints) with NO wave/agent/backbone logic attached. The tree
    /// is authored as a flat list of nodes (parent referenced by name, built
    /// topologically), so any chain/tree topology is tweakable without code
    /// changes. Spawn convention: root pelvis at transform.position +
    /// rotation*(up*rootLift) and inheriting the builder component's rotation;
    /// rig faces the component's +Z, legs hanging down, arms out along its
    /// +-X (T-pose).
    ///
    /// Conventions per node (authored in the PARENT's spawn frame; root nodes
    /// use the builder component's frame):
    ///  - "attachLocal": joint position as an offset from the PARENT box's
    ///    CENTER, in the parent's spawn rotation. Root node ignores it
    ///    (position comes from rootLift).
    ///  - "direction": spawn direction in the parent's frame; the box's long
    ///    axis (size.y) lies along it. 'down' = along the parent's bone axis
    ///    (the parent's local -Y points distally).
    ///  - "hingeAxisLocal": hinge axis in the CHILD's local frame at spawn.
    ///    Ignored for spherical joints.
    ///
    /// Bodies follow the package's deferred-Awake pattern: create INACTIVE,
    /// shape first, then Box3DBody, then activate (shape gathers exactly
    /// once); explicit box inertia is set after activation. Joints are built
    /// AFTER all bodies are active (anchors need native poses).
    /// </summary>
    public sealed class Box3DRigBuilder : MonoBehaviour
    {
        public enum JointKind { Spherical, Hinge }
        public enum LimbShape { Box, Capsule }

        [Serializable]
        public class RigNode
        {
            [Tooltip("Unique segment name. Parents reference this.")]
            public string name = "Segment";
            [Tooltip("Parent segment name. Empty = tree root (exactly one).")]
            public string parentName = "";
            [Tooltip("Joint position as an offset from the PARENT box's CENTER, in the PARENT's spawn frame (the parent's local -Y points distally, so along-bone offsets are negative y). Root: ignored.")]
            public Vector3 attachLocal = new Vector3(0f, -0.1f, 0f);
            [Tooltip("Spawn direction in the PARENT's spawn frame (root: builder component frame); 'down' = along the parent's bone. The box extends from the attach point along it (size.y = length).")]
            public Vector3 direction = Vector3.down;
            [Tooltip("Box extents (m). y runs along 'direction'.")]
            public Vector3 size = new Vector3(0.14f, 0.35f, 0.14f);
            [Tooltip("Collider shape. Capsule: axis = this box's LONGEST side (authored extents define length/radius - radius = half the smaller cross side).")]
            public LimbShape shape = LimbShape.Box;
            [Tooltip("Explicit mass (kg). Box inertia about COM is derived, not density-guessed.")]
            public float massKg = 5f;
            public JointKind joint = JointKind.Spherical;
            [Tooltip("Joint softness (Hz) and damping ratio. Dual role, mirroring the Box3D def: with the spring ON this is the pose-spring natural frequency (2-5 = soft hold, 10+ = stiff); with the spring OFF it is the constraint softness - keep it high (30+) or the joint is floppy.")]
            public float jointHertz = 30f;
            public float jointDamping = 1f;
            [Tooltip("Hinge axis in the CHILD's local frame at spawn (spherical: ignored).")]
            public Vector3 hingeAxisLocal = new Vector3(1f, 0f, 0f);
            [Tooltip("Hinge limits (deg) in the child's joint frame.")]
            public float limitMinDeg = -10f, limitMaxDeg = 150f;
            [Tooltip("Enable the pose spring: the joint applies corrective torque toward the spawn pose (TargetRotation/TargetAngle default to the spawn-relative identity). Damped by jointDamping. When disabling, raise jointHertz to 30+ (it reverts to constraint softness).")]
            public bool poseSpring = true;
            [Tooltip("Motor torque cap for this joint (Nm); 0 = use the motion driver's global maxMotorTorque. The motor tracks the commanded pose - raise for loaded joints (spine, hips), lower for distal ones.")]
            public float maxMotorTorque = 0f;
            [Tooltip("Attach a FootStabilizer to this segment's distal tip (feet).")]
            public bool footContact = false;
            [Tooltip("Pin this segment to the world (static body). Works on ANY node, not just the root - debugging aid for chains; keep OFF for a free-standing rig.")]
            public bool pinned = false;

            [NonSerialized] public Box3DBody body;
            [NonSerialized] public Box3D.Joint jointRef; // created in Build pass 2 (hinge or spherical)
            [NonSerialized] public float3 dirW;      // normalized spawn direction (world)
            [NonSerialized] public float3 attachW;   // joint anchor at spawn (world)
        }

        [Header("Tree")]
        [SerializeField, Tooltip("Segment list. Order free - built parent-first automatically. One node must have an empty parentName (the root).")]
        private List<RigNode> nodes = new List<RigNode>();

        [Header("Placement")]
        [SerializeField, Tooltip("Root segment center height above the component position (m).")]
        private float rootLift = 1.05f;
        [SerializeField, Tooltip("Root segment is Static (rig welded in the air - debugging aid; OFF for a free-standing rig).")]
        private bool rootStatic = false;
        [SerializeField, Tooltip("Reparent all body GameObjects to this component after Build, breaking body-parent links. Physics owns every world pose (dynamic bodies via move events, static ones never move), but a STATIC body that stays parented under its dynamic parent inherits the parent's transform - a pinned segment would visually follow its moving parent. Flatten to make pins and any non-simulated pose hold visually too.")]
        private bool flattenHierarchy = false;

        [Serializable]
        public class CollisionRule
        {
            [Tooltip("First segment name.")] public string nodeA;
            [Tooltip("Second segment name.")] public string nodeB;
            [Tooltip("False = the two segments never collide (pair is passed to the custom filter, which rejects it). True = default, collides.")]
            public bool collide = false;
        }

        [Header("Collision Matrix")]
        [SerializeField, Tooltip("Node-pair collision rules. Order-independent; unmatched names warn and are ignored. When any rule exists, rig shapes enable Box3D custom filtering and a world-level callback evaluates the matrix (layer masks and CollideConnected are unaffected).")]
        private List<CollisionRule> collisionRules = new List<CollisionRule>();

        [Header("Environment")]

        [SerializeField, Tooltip("Spawn a floor at y=0 on the FootStabilizer terrain layer (required for foot contacts).")]
        private bool floorEnabled = true;
        [SerializeField] private Vector2 floorSize = new Vector2(12f, 12f);

        [Header("Preview (Popcron GL)")]
        [SerializeField, Tooltip("Render actual mesh renderers on generated limb bodies: a child GameObject per body with a Unity cube scaled to the limb shape. Transform localScale stays 1 - the child carries the visual size.")]
        private bool limbRenderers = false;
        [SerializeField, Tooltip("Material for the limb renderer children; null = default diffuse.")]
        private Material limbRendererMaterial;
        [SerializeField, Tooltip("Render authored bone boxes and joint anchors through Popcron.Gizmos (GL overlay, shown while selected).")]
        private bool popcronBones = true;
        [SerializeField] private Color boneColor = new Color(0.3f, 0.8f, 1f, 0.85f);
        [SerializeField] private Color anchorColor = new Color(1f, 0.2f, 0.8f, 1f);
        [SerializeField, Tooltip("Short line along each hinge joint's axis at its anchor.")]
        private bool hingeAxes = true;
        [SerializeField, Tooltip("Arc showing a hinge joint's limit sweep: distal-tip path from limitMin to limitMax about the hinge axis.")]
        private bool limitArcs = true;
        [SerializeField, Range(2, 64)] private int limitArcSegments = 16;
        [SerializeField] private Color limitColor = new Color(1f, 0.6f, 0f, 0.9f);

        // --- Built state ---------------------------------------------------
        private readonly List<Box3DBody> m_bodies = new List<Box3DBody>();
        private readonly List<Box3D.Joint> m_joints = new List<Box3D.Joint>();
        private Box3DBody m_root;
        private GameObject m_floor;

        // Node collision matrix: the callback is per-world and only one can be
        // installed, so all rigs share a static shapeId -> (rig, node) registry.
        // Consulted on the stepping thread; builds/clears are main-thread (edit
        // mode or the same stepping frame) - dictionary ops are individually safe.
        private struct ShapeNodeRef
        {
            public Box3DRigBuilder Rig;
            public int NodeIndex;
        }
        private static readonly Dictionary<ulong, ShapeNodeRef> s_shapeNodes = new Dictionary<ulong, ShapeNodeRef>(32);
        private static readonly Dictionary<ulong, string> s_shapeNames = new Dictionary<ulong, string>(32); // id -> GameObject name, debug only
        [NonSerialized] private readonly Dictionary<Box3DBody, int> m_bodyOrder = new Dictionary<Box3DBody, int>(32); // body -> ordered (matrix) index
        private static bool s_logMatrixPairs; // static mirror: CustomFilterPair is static, has no instance
        private static readonly CustomFilterCallback s_customFilter = CustomFilterPair;
        private readonly List<ulong> m_shapeKeys = new List<ulong>(16);
        private bool[,] m_matrix;               // [i,j] true = collide; built at Build
        private string[] m_orderedNodeNames;    // ordered-build node names, same indexing as m_matrix

        /// <summary>Node-pair rules as authored (not built state).</summary>
        public IReadOnlyList<CollisionRule> CollisionRules => collisionRules;

        [SerializeField, Tooltip("Log every body pair evaluated by the collision matrix callback (debug - noisy).")]
        private bool logMatrixPairs = false; // serialize-reflected: probe prints it
        /// <summary>Inspector toggle state (read-only probe surface).</summary>
        public bool MatrixLogEnabled => logMatrixPairs;
        /// <summary>Static probe access for other components (synced at Build / registration).</summary>
        public static bool LogProbe => s_logMatrixPairs;

        private static bool CustomFilterPair(Shape shapeA, Shape shapeB)
        {
            // Shapes without custom filtering (floor, other rigs' plain shapes) never reach
            // this callback, so an unregistered id means a foreign filtering shape: allow.
            if (!s_shapeNodes.TryGetValue(shapeA.Id.ToUInt64(), out var a))
            { if (s_logMatrixPairs) Debug.Log($"[MatrixPair] UNREGISTERED shape {shapeA.Id.ToUInt64():X} ('{ShapeName(shapeA)}') in callback (other={shapeB.Id.ToUInt64():X} '{ShapeName(shapeB)}') - id mismatch or flag probe"); return true; }
            if (!s_shapeNodes.TryGetValue(shapeB.Id.ToUInt64(), out var b))
            { if (s_logMatrixPairs) Debug.Log($"[MatrixPair] UNREGISTERED shape {shapeB.Id.ToUInt64():X} ('{ShapeName(shapeB)}') in callback (other={shapeA.Id.ToUInt64():X} '{ShapeName(shapeA)}') - id mismatch or flag probe"); return true; }
            if (a.Rig == null || b.Rig == null || a.Rig.m_matrix == null) return true;
            bool result;
            if (a.Rig != b.Rig) result = true; // matrix is per-rig; cross-rig uses layer masks
            else if (a.NodeIndex == b.NodeIndex) result = false; // same node = nested/adjacent shapes (e.g. calf vs its sole): never self-collide
            else result = a.Rig.m_matrix[a.NodeIndex, b.NodeIndex];
            if (a.Rig.logMatrixPairs)
                Debug.Log($"[MatrixPair] {a.Rig.name}:{a.Rig.NodeName(a.NodeIndex)} <-> {(b.Rig != a.Rig ? b.Rig.name + ":" : "")}{b.Rig.NodeName(b.NodeIndex)} -> {(result ? "COLLIDE" : "skip")}");
            return result;
        }

        /// <summary>Register an extra shape spawned at runtime under a node's
        /// hierarchy (e.g. the foot suspension bodies) so the node-pair matrix
        /// governs it: it must (a) opt into custom filtering in its def — use
        /// RigBoxShape/RigCapsuleShape — and (b) map to a node index here. The
        /// owning body is matched against the rig nodes. No-op without an
        /// authored matrix (plain filtering applies).</summary>
        /// <summary>Shared registration core: id -> (rig, node index), teardown key,
        /// debug name map. Called for builder node shapes and stabilizer shapes alike.</summary>
        private void RegisterShapeRef(ulong key, int nodeIndex, string shapeName)
        {
            s_shapeNodes[key] = new ShapeNodeRef { Rig = this, NodeIndex = nodeIndex };
            m_shapeKeys.Add(key);
            s_shapeNames[key] = shapeName; // debug id->name map
        }

        public bool RegisterMatrixShape(Box3DShape shape, Box3DBody body)
        {
            if (shape == null || body == null) return false;
            s_logMatrixPairs = logMatrixPairs; // sync here too: mid-play toggle works without re-Build
            if (m_matrix == null)
            {
                if (logMatrixPairs) Debug.Log($"[MatrixReg] '{shape.name}' under '{body.name}': no matrix authored (no collision rules) - not registered", this);
                return false;
            }
            int ni = -1;
            if (!m_bodyOrder.TryGetValue(body, out ni))
            {
                if (logMatrixPairs) Debug.Log($"[MatrixReg] '{shape.name}' under '{body.name}': body is not a rig node - not registered", this);
                return false;
            }
            ulong key = shape.LiveShape.Id.ToUInt64();
            s_shapeNodes[key] = new ShapeNodeRef { Rig = this, NodeIndex = ni };
            m_shapeKeys.Add(key);
            s_shapeNames[key] = shape.name; // debug id->name map
            if (logMatrixPairs)
            {
                var shapeWorld = shape.LiveShape.GetWorld();
                var builderWorld = Box3DWorld.Instance != null && Box3DWorld.Instance.World.IsValid
                    ? Box3DWorld.Instance.World.Id : default;
                bool same = shapeWorld.ToUInt32() == builderWorld.ToUInt32();
                Debug.Log($"[MatrixReg] '{shape.name}' id {key:X} under '{nodes[ni].name}' (idx {ni}) | shape world {shapeWorld.ToUInt32()} vs builder world {builderWorld.ToUInt32()} {(same ? "MATCH" : "** WORLD MISMATCH **")}", this);
            }
            return true;
        }

        private static string ShapeName(Shape shape) =>
            s_shapeNames.TryGetValue(shape.Id.ToUInt64(), out var n) ? n : "unknown";

        /// <summary>Root (first parentless) body, null until built.</summary>
        public Box3DBody Root => m_root;
        /// <summary>All segment bodies in node order (null until built).</summary>
        public IReadOnlyList<Box3DBody> Bodies => m_bodies;
        /// <summary>Editable node list (null until built: dirW/attachW/jointRef).</summary>
        public IReadOnlyList<RigNode> Nodes => nodes;

        /// <summary>Node by segment name, null if absent.</summary>
        public RigNode FindNode(string name)
        {
            foreach (var n in nodes) if (n.name == name) return n;
            return null;
        }

        /// <summary>Built body for a segment name, null if absent.</summary>
        public Box3DBody FindBody(string name) => FindNode(name)?.body;
        public bool IsBuilt => m_root != null && m_root.Body.IsValid;

        private void Awake()
        {
            if (floorEnabled || nodes.Count > 0) Build();
        }

        [ContextMenu("Build Rig")]
        public void Build()
        {
            Clear();
            if (!FindAnyObjectByType<Box3DWorld>())
                new GameObject("Box3D World").AddComponent<Box3DWorld>();

            if (floorEnabled) CreateFloor();
            if (nodes.Count == 0) { Debug.LogWarning("[Box3DRigBuilder] no nodes", this); return; }

            // Validate + topological order (parents before children).
            var byName = new Dictionary<string, RigNode>(nodes.Count);
            foreach (var n in nodes)
            {
                if (string.IsNullOrWhiteSpace(n.name))
                { Debug.LogError("[Box3DRigBuilder] node with empty name", this); return; }
                if (!byName.TryAdd(n.name, n))
                { Debug.LogError($"[Box3DRigBuilder] duplicate node name '{n.name}'", this); return; }
            }
            int roots = 0;
            foreach (var n in nodes)
                if (string.IsNullOrEmpty(n.parentName)) roots++;
                else if (!byName.ContainsKey(n.parentName))
                { Debug.LogError($"[Box3DRigBuilder] '{n.name}' parent '{n.parentName}' not found", this); return; }
            if (roots != 1)
            { Debug.LogError($"[Box3DRigBuilder] need exactly 1 root (empty parentName), found {roots}", this); return; }

            var ordered = new List<RigNode>(nodes.Count);
            var done = new HashSet<string>();
            // Iterative topo pass; a cycle would never satisfy 'done' - bounded by count.
            var pending = new List<RigNode>(nodes);
            while (pending.Count > 0)
            {
                int before = pending.Count;
                for (int i = pending.Count - 1; i >= 0; i--)
                {
                    var n = pending[i];
                    if (string.IsNullOrEmpty(n.parentName) || done.Contains(n.parentName))
                    {
                        ordered.Add(n); done.Add(n.name); pending.RemoveAt(i);
                    }
                }
                if (pending.Count == before)
                { Debug.LogError("[Box3DRigBuilder] cycle in parent references", this); return; }
            }

            // Bodies: root at the datum in the BUILDER COMPONENT's frame; each
            // child's anchor and direction are authored in the PARENT's spawn
            // frame: anchor = parent CENTER + parentRot*attachLocal, dirW =
            // parentRot*direction, child rot = parentRot*ShortestArcDownTo(
            // direction). Authoring in the parent frame keeps bone orientations
            // consistent along chains (direction down = "along the parent's
            // bone") and works for a non-identity builder/root rotation.
            // Deferred-Awake pattern, built parent-first.
            quaternion builderRot = transform.rotation;
            float3 basePos = (float3)transform.position + math.mul(builderRot, new float3(0f, rootLift, 0f));
            var transforms = new Dictionary<string, Transform>(ordered.Count);
            s_logMatrixPairs = logMatrixPairs; // sync BEFORE anything logs (mid-play toggle works)
            BuildCollisionMatrix(ordered, byName);
            bool matrixActive = collisionRules.Count > 0;
            if (logMatrixPairs && !matrixActive)
                Debug.Log($"[MatrixReg] no collision rules authored - plain shapes, callback not armed, nothing registered", this);
            if (matrixActive)
            {
                var world = FindAnyObjectByType<Box3DWorld>();
                if (world != null && world.World != null) world.World.SetCustomFilterCallback(s_customFilter);
            }
            for (int ni = 0; ni < ordered.Count; ni++)
            {
                var n = ordered[ni];
                Transform parentT = null;
                float3 anchorW;
                quaternion parentRot;
                if (string.IsNullOrEmpty(n.parentName))
                {
                    parentRot = builderRot;
                    anchorW = basePos;
                }
                else
                {
                    parentT = transforms[n.parentName];
                    parentRot = parentT.rotation;
                    // Parent-frame offset from the parent box CENTER.
                    anchorW = (float3)parentT.position + math.mul(parentRot, (float3)n.attachLocal);
                }
                float3 dirLocal = math.normalizesafe((float3)n.direction, new float3(0f, -1f, 0f));
                n.dirW = math.mul(parentRot, dirLocal);
                n.attachW = anchorW;
                // Child frame = parent frame turned so its local -Y (bone
                // datum) points along the child's authored direction.
                quaternion rotW = math.mul(parentRot, ShortestArcDownTo(dirLocal));
                float3 centerW = anchorW + n.dirW * (n.size.y * 0.5f);

                var go = NewBodyGo(n.name, centerW, parentT);
                go.transform.rotation = rotW;
                if (n.shape == LimbShape.Capsule)
                {
                    // Capsule: axis = the box's longest side. RigCapsuleShape
                    // opts into the custom-filter callback just like the box,
                    // so the node-pair collision matrix covers capsules too.
                    var cap = (Box3DCapsuleShape)(matrixActive ? go.AddComponent<RigCapsuleShape>() : go.AddComponent<Box3DCapsuleShape>());
                    SetCapsuleFromExtents(cap, n.size, out int axis);
                    if (limbRenderers) AddLimbRenderer(go, n.size, axis);
                    go.AddComponent<Box3DBody>();
                    var typeC = n.pinned || (string.IsNullOrEmpty(n.parentName) && rootStatic)
                        ? Box3DBodyType.Static : Box3DBodyType.Dynamic;
                    var bodyC = FinishBody(go, typeC, n.massKg, n.size);
                    n.body = bodyC;
                    m_bodies.Add(bodyC);
                    m_bodyOrder[bodyC] = ni; // ordered (matrix) index space
                    if (string.IsNullOrEmpty(n.parentName)) m_root = bodyC;
                    transforms[n.name] = go.transform;
                    if (matrixActive) RegisterShapeRef(((RigCapsuleShape)cap).LiveShape.Id.ToUInt64(), ni, cap.name);
                    continue;
                }
                // RigBoxShape (subclass) opts into the custom-filter callback; plain
                // Box3DBoxShape when no rules are authored so no callback overhead.
                var shape = (Box3DBoxShape)(matrixActive ? go.AddComponent<RigBoxShape>() : go.AddComponent<Box3DBoxShape>());
                SetShapeSize(shape, n.size); // COM/collider from the shape, not localScale
                if (limbRenderers) AddLimbRenderer(go, n.size, 1);
                go.AddComponent<Box3DBody>();
                var type = n.pinned || (string.IsNullOrEmpty(n.parentName) && rootStatic)
                    ? Box3DBodyType.Static : Box3DBodyType.Dynamic;
                var body = FinishBody(go, type, n.massKg, n.size);
                n.body = body;
                m_bodies.Add(body);
                m_bodyOrder[body] = ni; // ordered (matrix) index space
                if (matrixActive) RegisterShapeRef(((RigBoxShape)shape).LiveShape.Id.ToUInt64(), ni, shape.name);
                if (string.IsNullOrEmpty(n.parentName)) m_root = body;
                transforms[n.name] = go.transform;
            }

            // Pass 2: joints (all bodies active -> native poses valid). Anchor
            // = child's attach point; frame axis = child's spawn direction
            // (spherical) or the child-local hinge axis lifted to world.
            foreach (var n in ordered)
            {
                if (string.IsNullOrEmpty(n.parentName)) continue;
                var p = byName[n.parentName];
                // Frame reference along the chain: the PARENT's spawn direction
                // (its body was already rotated by it; anchors are sibling-parallel).
                float3 axisW = p.dirW;
                if (n.joint == JointKind.Hinge)
                    axisW = math.mul(n.body.transform.rotation, math.normalizesafe((float3)n.hingeAxisLocal, new float3(1f, 0f, 0f)));
                var joint = n.joint == JointKind.Hinge
                    ? CreateHinge(p.body, n.body, n.attachW, axisW, n.jointHertz, n.jointDamping,
                        math.radians(n.limitMinDeg), math.radians(n.limitMaxDeg), n.poseSpring, n.maxMotorTorque)
                    : CreateSpherical(p.body, n.body, n.attachW, axisW, n.jointHertz, n.jointDamping, n.poseSpring, n.maxMotorTorque);
                m_joints.Add(joint);
                n.jointRef = joint;

                if (n.footContact)
                {
                    var foot = n.body.gameObject.AddComponent<FootStabilizer>();
                    // Tip at the box's distal face; support rect from the cross-section.
                    foot.Configure(new float3(0f, -n.size.y * 0.5f, 0f),
                        n.size.y * 0.5f, n.size.x * 0.5f, n.size.z * 0.5f);
                    foot.ConfigureWorld(Box3DWorld.Instance);
                    foot.SetRendererMaterial(limbRendererMaterial); // sole visual matches limb visuals
                }
            }

            Debug.Log($"[Box3DRigBuilder] built '{name}': {m_bodies.Count} bodies, {m_joints.Count} joints", this);

            if (flattenHierarchy)
            {
                // Break body-parent links (world pose preserved). Physics
                // drives dynamic bodies' world transforms anyway; this makes
                // static (pinned) bodies hold their spawn pose visually too
                // instead of inheriting their parent's motion. Covers ALL
                // bodies under this transform, including runtime-spawned
                // suspension children present at build time.
                FlattenChildBodies();
            }
        }

        /// <summary>Whether Build flattens the body hierarchy (also gates the
        /// stabilizer's post-insertion flatten call).</summary>
        public bool FlattenEnabled => flattenHierarchy;

        /// <summary>Node name by MATRIX index (ordered-build order; 'nodes'
        /// is authored order and can differ).</summary>
        private string NodeName(int matrixIndex) =>
            (m_orderedNodeNames != null && matrixIndex >= 0 && matrixIndex < m_orderedNodeNames.Length)
                ? m_orderedNodeNames[matrixIndex] : $"#{matrixIndex}";

        /// <summary>Extents of the rig node owning a body (endpoint math -
        /// e.g. physics pose bias tips). False when the body is not a node body.</summary>
        public bool TryGetNodeSize(Box3DBody body, out Vector3 size)
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i].body == body) { size = nodes[i].size; return true; }
            }
            size = default;
            return false;
        }

        [SerializeField, Tooltip("Debug view of the BUILT collision matrix (ordered-build indexing, rows in build order). Populated at Build; read-only.")]
        private List<MatrixRowDebug> builtMatrix = new List<MatrixRowDebug>();

        /// <summary>Debug row: node name + collide flags to all nodes (same order).</summary>
        [Serializable]
        public struct MatrixRowDebug
        {
            public string node;
            public bool[] collide;
        }

        // Inspector access to the BUILT matrix (read-only view).
        public bool HasMatrix => m_matrix != null;
        /// <summary>Built-matrix dimension (ordered-build node count), 0 without an authored matrix.</summary>
        public int MatrixSize => m_matrix?.GetLength(0) ?? 0;
        /// <summary>Node name by ordered-build index (same indexing as MatrixEntry).</summary>
        public string OrderedNodeName(int i) => NodeName(i);
        /// <summary>Built entry [i,j]: true = collide. False when no matrix is built.</summary>
        public bool MatrixEntry(int i, int j) => m_matrix != null && m_matrix[i, j];

        /// <summary>Reparent every Box3DBody under this transform (any depth,
        /// excluding this GameObject itself) as a direct child, world pose
        /// preserved. Idempotent; also picks up bodies spawned after Build
        /// (call again, e.g. after the foot stabilizer's insertion).</summary>
        public void FlattenChildBodies()
        {
            var bodies = GetComponentsInChildren<Box3DBody>(false);
            for (int i = 0; i < bodies.Length; i++)
            {
                var t = bodies[i].transform;
                if (t == transform || t.parent == transform) continue;
                t.SetParent(transform, true);
            }
        }

        [ContextMenu("Clear Rig")]
        public void Clear()
        {
            // Joints first, then bodies (wake attached so bodies notice).
            if (Box3DWorld.Instance != null && Box3DWorld.Instance.World != null)
                foreach (var j in m_joints) j.Destroy();
            m_joints.Clear();
            foreach (var b in m_bodies) if (b != null) Destroy(b.gameObject);
            m_bodies.Clear();
            if (m_floor != null) { Destroy(m_floor); m_floor = null; }
            m_root = null;
            foreach (var n in nodes) { n.body = null; n.jointRef = default; }

            // Unregister this rig's shapes; drop the world callback when none remain.
            foreach (ulong k in m_shapeKeys) s_shapeNodes.Remove(k);
            m_shapeKeys.Clear();
            m_bodyOrder.Clear(); // index map follows the built bodies
            m_matrix = null;
            builtMatrix.Clear(); // debug mirror follows the live matrix
            if (s_shapeNodes.Count == 0 && Box3DWorld.Instance != null && Box3DWorld.Instance.World != null)
                Box3DWorld.Instance.World.SetCustomFilterCallback(null);
        }

        private void BuildCollisionMatrix(List<RigNode> ordered, Dictionary<string, RigNode> byName)
        {
            int count = ordered.Count;
            m_matrix = new bool[count, count];
            for (int i = 0; i < count; i++)
                for (int j = 0; j < count; j++)
                    m_matrix[i, j] = true;
            var index = new Dictionary<string, int>(count);
            for (int i = 0; i < count; i++) index[ordered[i].name] = i;
            m_orderedNodeNames = new string[count];
            for (int i = 0; i < count; i++) m_orderedNodeNames[i] = ordered[i].name;
            foreach (var rule in collisionRules)
            {
                if (string.IsNullOrEmpty(rule.nodeA) || string.IsNullOrEmpty(rule.nodeB) ||
                    !index.TryGetValue(rule.nodeA, out int a) || !index.TryGetValue(rule.nodeB, out int b))
                { Debug.LogWarning($"[Box3DRigBuilder] collision rule '{rule.nodeA}'/'{rule.nodeB}' does not name two nodes", this); continue; }
                m_matrix[a, b] = m_matrix[b, a] = rule.collide;
            }

            // Debug mirror for the default inspector (names + collide flags,
            // ordered-build indexing).
            builtMatrix.Clear();
            for (int i = 0; i < count; i++)
            {
                var row = new bool[count];
                for (int j = 0; j < count; j++) row[j] = m_matrix[i, j];
                builtMatrix.Add(new MatrixRowDebug { node = ordered[i].name, collide = row });
            }
        }

        [ContextMenu("Reset Nodes To Default Biped")]
        private void ResetToDefaultBiped()
        {
            nodes.Clear();
            // Spine chain: pelvis -> waist -> torso -> neck -> head.
            // Authoring convention: direction down = continue along the
            // parent's bone (child inherits the parent's frame, datum -Y
            // distal); direction up = reverse (only the waist, growing off
            // the pelvis's proximal side). attachLocal y<0 = parent's distal
            // side, y>0 = proximal side (signs flip inside 180°-flipped
            // parents, e.g. the torso).
            nodes.Add(new RigNode { name = "Pelvis", parentName = "", attachLocal = Vector3.zero,
                direction = Vector3.down, size = new Vector3(0.24f, 0.18f, 0.14f), massKg = 10f,
                joint = JointKind.Spherical, jointHertz = 40f });
            nodes.Add(new RigNode { name = "Waist", parentName = "Pelvis", attachLocal = new Vector3(0f, 0.09f, 0f),
                direction = Vector3.up, size = new Vector3(0.22f, 0.15f, 0.13f), massKg = 6f,
                jointHertz = 35f });
            nodes.Add(new RigNode { name = "Torso", parentName = "Waist", attachLocal = new Vector3(0f, -0.075f, 0f),
                direction = Vector3.down, size = new Vector3(0.28f, 0.35f, 0.16f), massKg = 16f,
                jointHertz = 35f });            nodes.Add(new RigNode { name = "Neck", parentName = "Torso", attachLocal = new Vector3(0f, -0.175f, 0f),
                direction = Vector3.down, size = new Vector3(0.08f, 0.08f, 0.08f), massKg = 1.2f,
                jointHertz = 30f });
            nodes.Add(new RigNode { name = "Head", parentName = "Neck", attachLocal = new Vector3(0f, -0.04f, 0f),
                direction = Vector3.down, size = new Vector3(0.16f, 0.20f, 0.18f), massKg = 4f,
                jointHertz = 30f });
            // Legs from the pelvis (thigh spherical hip, calf hinge knee, foot on calf).
            foreach (var (side, x) in new[] { ("Left", -0.12f), ("Right", 0.12f) })
            {
                nodes.Add(new RigNode { name = $"{side}Thigh", parentName = "Pelvis",
                    attachLocal = new Vector3(x, -0.09f, 0f), direction = Vector3.down,
                    size = new Vector3(0.14f, 0.42f, 0.14f), massKg = 7f,
                    joint = JointKind.Spherical, jointHertz = 40f });
                nodes.Add(new RigNode { name = $"{side}Calf", parentName = $"{side}Thigh",
                    attachLocal = new Vector3(0f, -0.21f, 0f), direction = Vector3.down,
                    size = new Vector3(0.12f, 0.42f, 0.12f), massKg = 4f,
                    joint = JointKind.Hinge, hingeAxisLocal = new Vector3(1f, 0f, 0f),
                    limitMinDeg = -10f, limitMaxDeg = 150f,
                    footContact = true, jointHertz = 40f });
            }
            // Arms from the torso, T-pose along +-X (box long axis follows direction).
            foreach (var (side, x) in new[] { ("Left", -1f), ("Right", 1f) })
            {
                // Torso frame is 180°-flipped (local +y = world down), so the
                // authored signs are the world values mirrored.
                nodes.Add(new RigNode { name = $"{side}UpperArm", parentName = "Torso",
                    attachLocal = new Vector3(-0.14f * x, -0.12f, 0f), direction = new Vector3(-x, 0f, 0f),
                    size = new Vector3(0.09f, 0.30f, 0.09f), massKg = 1.6f,
                    joint = JointKind.Spherical, jointHertz = 30f });
                nodes.Add(new RigNode { name = $"{side}Forearm", parentName = $"{side}UpperArm",
                    attachLocal = new Vector3(0f, -0.15f, 0f), direction = Vector3.down,
                    size = new Vector3(0.08f, 0.28f, 0.08f), massKg = 1.1f,
                    joint = JointKind.Hinge,
                    // Elbow: 'down' direction = along the parent bone, so the
                    // forearm inherits the upper arm's frame (world ∓X for a
                    // T-pose arm). Paired-limb convention: joint params are
                    // IDENTICAL left/right - the mirror lives in the child's
                    // spawn rotation. Hinge axis local -X maps to mirrored
                    // world axes (left +Y, right -Y), so positive flexion
                    // sweeps BOTH hands forward (+Z) with the same limits.
                    hingeAxisLocal = new Vector3(-1f, 0f, 0f),
                    limitMinDeg = -10f, limitMaxDeg = 150f, jointHertz = 30f });
            }
        }

        // --- Body creation (PinnedBipedDemo pattern) -------------------------

        private GameObject NewBodyGo(string name, Vector3 position, Transform parent)
        {
            var go = new GameObject(name);
            go.layer = gameObject.layer; // Box3DBody.Awake bakes the shape's filter category from the layer - spawned bodies must share the builder's layer to be seen/accepted by the same queries
            go.transform.SetParent(parent != null ? parent : transform, true);
            go.transform.position = position;
            go.transform.localScale = Vector3.one;
            go.SetActive(false); // defer Awakes: Box3DBody.Awake gathers shapes exactly once
            return go;
        }

        private static void SetShapeSize(Box3DBoxShape shape, Vector3 size)
        {
            // Box3DBoxShape.Size is private serialized; COM frame + collider
            // come from the shape, NOT localScale (same trick as the demo).
            var f = typeof(Box3DBoxShape).GetField("Size",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
            f.SetValue(shape, size);
        }

        // Capsule from authored box extents: total height = the extent along
        // the longest side; radius = half the smaller cross side. Radius/
        // Height/Direction are private serialized (same reflection pattern as
        // SetShapeSize).
        private static void SetCapsuleFromExtents(Box3DCapsuleShape shape, Vector3 size, out int axis)
        {
            axis = 0;
            if (size.y >= size.x && size.y >= size.z) axis = 1;
            else if (size.z >= size.x) axis = 2;
            float radius = 0.5f * (axis == 0 ? math.min(size.y, size.z)
                          : axis == 1 ? math.min(size.x, size.z)
                                      : math.min(size.x, size.y));
            float height = axis == 0 ? size.x : axis == 1 ? size.y : size.z;
            var t = typeof(Box3DCapsuleShape);
            t.GetField("Radius", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                .SetValue(shape, radius);
            t.GetField("Height", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                .SetValue(shape, height);
            t.GetField("Direction", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                .SetValue(shape, (Box3DAxis)axis);
        }

        // Debug visual: a child GameObject with a cube mesh renderer scaled to
        // the limb's collider size. The body transform keeps localScale 1 (the
        // physics pipeline relies on unscaled transforms); the child carries
        // the visual size instead.
        // axis: 0/1/2 = the box's long side (X/Y/Z) - cube renders as-is,
        // capsule primitive (radius 0.5, height 2) is scaled and rotated so
        // its Y axis lands on the capsule axis.
        private void AddLimbRenderer(GameObject go, Vector3 size, int axis = 1)
        {
            var child = GameObject.CreatePrimitive(axis == 1 ? PrimitiveType.Cube : PrimitiveType.Capsule);
            UnityEngine.Object.Destroy(child.GetComponent<Collider>()); // visual only
            child.name = go.name + "_Renderer";
            child.transform.SetParent(go.transform, false);
            child.transform.localPosition = Vector3.zero;
            if (axis == 1) child.transform.localScale = size; // cube
            else
            {
                // Unity capsule: radius 0.5 (x,z), total height 2 (y).
                float radius = 0.5f * (axis == 0 ? math.min(size.y, size.z)
                              : axis == 1 ? math.min(size.x, size.z)
                                          : math.min(size.x, size.y));
                float length = axis == 0 ? size.x : axis == 1 ? size.y : size.z;
                child.transform.localScale = new Vector3(2f * radius, 0.5f * length, 2f * radius);
                // Rotate the capsule's Y axis onto the long side.
                child.transform.localRotation = axis == 0
                    ? Quaternion.Euler(0f, 0f, 90f)
                    : Quaternion.Euler(90f, 0f, 0f); // Z
            }
            var r = child.GetComponent<MeshRenderer>();
            if (limbRendererMaterial != null) r.sharedMaterial = limbRendererMaterial;
            r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.On;
        }

        private Box3DBody FinishBody(GameObject go, Box3DBodyType type, float massKg, Vector3 size)
        {
            var body = go.GetComponent<Box3DBody>();
            body.BodyType = type;
            go.SetActive(true); // Awakes run: shape defers to the body, body gathers + attaches it
            // Explicit anthropometric mass: box-enclosing inertia, I = m/12*(b^2+c^2)
            // per axis about the COM (general form - already axis-agnostic, the
            // extents carry which side is long). Approximation for capsules:
            // a capsule's true inertia is smaller than its enclosing box's -
            // conservative. Dynamic only - SetMassData on a static body
            // gives it finite inverse mass and the joint solver moves it.
            if (body.Body.IsValid && type == Box3DBodyType.Dynamic)
            {
                // Box-enclosing inertia about the COM, general form (correct for
                // any long axis - extents are symmetric inputs). For capsules this
                // is an approximation: a capsule's true inertia is smaller than
                // its enclosing box's; conservative.
                float x = size.x, y = size.y, z = size.z;
                body.Body.SetMassData(new MassData
                {
                    Mass = massKg,
                    Center = float3.zero,
                    Inertia = new B3Matrix3
                    {
                        Cx = new float3(massKg / 12f * (y * y + z * z), 0f, 0f),
                        Cy = new float3(0f, massKg / 12f * (x * x + z * z), 0f),
                        Cz = new float3(0f, 0f, massKg / 12f * (x * x + y * y)),
                    },
                });
            }
            return body;
        }

        private void CreateFloor()
        {
            m_floor = NewBodyGo("Floor", new Vector3(transform.position.x, -0.5f, transform.position.z), null);
            int terrain = LayerMask.NameToLayer(FootStabilizer.TerrainLayerName);
            if (terrain >= 0) m_floor.layer = terrain; // BEFORE FinishBody: Awake bakes the filter category
            var shape = m_floor.AddComponent<Box3DBoxShape>();
            SetShapeSize(shape, new Vector3(floorSize.x, 1f, floorSize.y));
            m_floor.AddComponent<Box3DBody>();
            var body = m_floor.GetComponent<Box3DBody>();
            body.BodyType = Box3DBodyType.Static;
            m_floor.SetActive(true);
        }

        // --- Joints (copied from the wave probe, self-contained) -------------

        private Box3D.Joint CreateSpherical(Box3DBody a, Box3DBody b, float3 anchor, float3 axisW,
            float hertz, float damping, bool spring, float maxMotorTorque)
        {
            quaternion frameRot = FrameFromAxisWorld(axisW);
            var frames = LocalFrames(a.Body, b.Body, new B3Transform { Position = anchor, Rotation = frameRot });
            var def = SphericalJointDef.Default;
            def.Hertz = hertz; def.DampingRatio = damping;
            def.EnableSpring = spring; // TargetRotation defaults to identity = spawn pose
            def.EnableMotor = true;
            def.MaxMotorTorque = maxMotorTorque;
            def.Base.BodyIdA = a.Body.Id; def.Base.BodyIdB = b.Body.Id; def.Base.CollideConnected = false;
            def.Base.LocalFrameA = frames.a; def.Base.LocalFrameB = frames.b;
            return Box3DWorld.Instance.World.CreateSphericalJoint(def);
        }

        private Box3D.Joint CreateHinge(Box3DBody a, Box3DBody b, float3 anchor, float3 axisW,
            float hertz, float damping, float minRad, float maxRad, bool spring, float maxMotorTorque)
        {
            quaternion frameRot = FrameFromAxisWorld(axisW);
            var frames = LocalFrames(a.Body, b.Body, new B3Transform { Position = anchor, Rotation = frameRot });
            var def = RevoluteJointDef.Default;
            def.Hertz = hertz; def.DampingRatio = damping;
            def.EnableSpring = spring; // TargetAngle defaults to 0 = spawn pose
            def.EnableMotor = true;
            def.MaxMotorTorque = maxMotorTorque;
            def.EnableLimit = true;
            def.LowerAngle = minRad;
            def.UpperAngle = maxRad;
            def.Base.BodyIdA = a.Body.Id; def.Base.BodyIdB = b.Body.Id; def.Base.CollideConnected = false;
            def.Base.LocalFrameA = frames.a; def.Base.LocalFrameB = frames.b;
            return Box3DWorld.Instance.World.CreateRevoluteJoint(def);
        }

        private static (B3Transform a, B3Transform b) LocalFrames(Body a, Body b, B3Transform worldFrame)
        {
            quaternion qa = a.GetRotation(), qb = b.GetRotation();
            var la = new B3Transform
            {
                Position = math.mul(math.inverse(qa), (float3)(worldFrame.Position - a.GetPosition())),
                Rotation = math.mul(math.inverse(qa), worldFrame.Rotation)
            };
            var lb = new B3Transform
            {
                Position = math.mul(math.inverse(qb), (float3)(worldFrame.Position - b.GetPosition())),
                Rotation = math.mul(math.inverse(qb), worldFrame.Rotation)
            };
            return (la, lb);
        }

        /// <summary>Stable frame whose X axis is 'axis' (copied from
        /// SplineChainController: never degenerate for vertical axes - joint
        /// frames must be identical across rebuilds or hinge limits wander).</summary>
        internal static quaternion FrameFromAxisWorld(float3 axis)
        {
            axis = math.normalizesafe(axis, math.forward());
            float3 reference = math.abs(math.dot(axis, math.up())) > 0.9f ? math.forward() : math.up();
            float3 yAxis = math.normalizesafe(math.cross(reference, axis), math.right());
            float3 upHint = math.normalizesafe(math.cross(axis, yAxis), math.up());
            return quaternion.LookRotationSafe(axis, upHint);
        }

        // --- Pose helpers -----------------------------------------------------

        /// <summary>Rotation whose local -Y (bone datum, same as the wave
        /// probes) points along 'dir'. Identity for straight-down segments.</summary>
        private static quaternion ShortestArcDownTo(float3 dir)
        {
            float3 seg = new float3(0f, -1f, 0f);
            float cosA = math.clamp(math.dot(seg, dir), -1f, 1f);
            float3 arcAxis = math.cross(seg, dir);
            if (math.lengthsq(arcAxis) < 1e-10f)
                return cosA > 0f ? quaternion.identity : quaternion.RotateZ(math.PI);
            return quaternion.AxisAngle(math.normalizesafe(arcAxis), math.acos(cosA));
        }

        private static quaternion Mul(quaternion a, quaternion b) => math.mul(a, b);

        private void OnDrawGizmosSelected()
        {
            // Tree overlay: joint anchors + segment rays at authored size.
            Gizmos.color = new Color(0.3f, 0.8f, 1f);
            var frames = EvaluatePoses();
            foreach (var n in nodes)
            {
                var pose = frames[n.name];
                Gizmos.DrawWireSphere(pose.Anchor, 0.03f);
                // rot maps the child's local -Y onto its authored direction.
                Gizmos.DrawRay(pose.Anchor, pose.DirW * n.size.y);

                // Popcron GL overlay: the bone's actual box shape + anchor +
                // hinge axis, in the authored pose.
                if (!popcronBones) continue;
                Popcron.Gizmos.Cube(pose.Center, pose.Rot, n.size, boneColor);
                Popcron.Gizmos.Sphere(pose.Anchor, 0.025f, anchorColor);
                if (n.joint == JointKind.Hinge)
                {
                    float3 axisW = math.mul(pose.Rot, math.normalizesafe((float3)n.hingeAxisLocal, new float3(1f, 0f, 0f)));
                    if (hingeAxes)
                        Popcron.Gizmos.Line(pose.Anchor - axisW * 0.1f, pose.Anchor + axisW * 0.1f, anchorColor);
                    if (limitArcs)
                        DrawLimitArc(n, pose, axisW);
                }
            }
        }

        /// Authored-pose frame of one node, evaluated from the config alone
        /// (no build required) - the same math as the gizmo preview.
        public readonly struct AuthoredFrame
        {
            public readonly float3 Anchor;  // joint anchor (world)
            public readonly float3 DirW;    // spawn direction (world)
            public readonly float Length;   // size.y
            public readonly quaternion Rot; // spawn rotation (world)

            public AuthoredFrame(float3 anchor, float3 dirW, float length, quaternion rot)
            { Anchor = anchor; DirW = dirW; Length = length; Rot = rot; }

            public float3 Tip => Anchor + DirW * Length;
        }

        /// Authored-pose frame per node, from the serialized config only.
        /// Independent of build state - used by editor tooling.
        public void GetAuthoredFrames(Dictionary<string, AuthoredFrame> frames)
        {
            frames.Clear();
            foreach (var kv in EvaluatePoses())
            {
                var p = kv.Value;
                frames[kv.Key] = new AuthoredFrame(p.Anchor, p.DirW, p.BoneLength, p.Rot);
            }
        }

        /// Evaluates the authored pose of every node from the config.
        private Dictionary<string, GizmoPose> EvaluatePoses()
        {
            var byName = new Dictionary<string, RigNode>(nodes.Count);
            foreach (var n in nodes) byName[n.name] = n;
            var frames = new Dictionary<string, GizmoPose>(nodes.Count);
            quaternion builderRot = transform.rotation;
            float3 basePos = (float3)transform.position + math.mul(builderRot, new float3(0f, rootLift, 0f));
            foreach (var n in nodes) GizmoFrame(n, byName, frames, builderRot, basePos);
            return frames;
        }

        /// Tip-sweep arc of a hinge's limits: the joint angle is the child's
        /// world rotation about the hinge axis, so rotating the spawn
        /// direction by [min, max] traces the reachable distal tip.
        private void DrawLimitArc(RigNode n, GizmoPose pose, float3 axisW)
        {
            float radius = n.size.y;
            int segs = Mathf.Max(2, limitArcSegments);
            float3 prev = LimitArcPoint(pose, axisW, n.limitMinDeg);
            for (int i = 1; i <= segs; i++)
            {
                float deg = math.lerp(n.limitMinDeg, n.limitMaxDeg, i / (float)segs);
                float3 cur = LimitArcPoint(pose, axisW, deg);
                Popcron.Gizmos.Line(prev, cur, limitColor);
                prev = cur;
            }
        }

        private static float3 LimitArcPoint(in GizmoPose pose, float3 axisW, float deg)
        {
            // Rotate the spawn direction about the hinge axis by the limit angle.
            float3 tipDir = math.mul(quaternion.AxisAngle(axisW, math.radians(deg)), pose.DirW);
            return pose.Anchor + tipDir * pose.BoneLength;
        }

        private struct GizmoPose
        {
            public float3 Anchor;   // joint anchor (parent-frame attach point)
            public float3 Center;   // box center = anchor + dir*half-length
            public float3 DirW;     // world spawn direction
            public float BoneLength; // size.y, the box extends this far from the anchor
            public quaternion Rot;  // spawn rotation (parent frame for children)
        }

        /// Authored-pose frame of a node (editor preview; physics poses come
        /// from the built bodies). Same parent-relative math as Build(): the
        /// anchor hangs off the parent box CENTER, not the parent's own
        /// anchor. Memoized, so the tree can be walked in any order.
        private static GizmoPose GizmoFrame(
            RigNode n, Dictionary<string, RigNode> byName,
            Dictionary<string, GizmoPose> frames,
            quaternion builderRot, float3 basePos)
        {
            if (frames.TryGetValue(n.name, out var cached)) return cached;
            float3 dirLocal = math.normalizesafe((float3)n.direction, new float3(0f, -1f, 0f));
            GizmoPose pose;
            if (string.IsNullOrEmpty(n.parentName) || !byName.TryGetValue(n.parentName, out var p))
            {
                // Root (or orphan - preview fallback): builder component frame.
                pose.Anchor = basePos;
                pose.Rot = math.mul(builderRot, ShortestArcDownTo(dirLocal));
                pose.DirW = math.mul(builderRot, dirLocal);
            }
            else
            {
                GizmoPose parent = GizmoFrame(p, byName, frames, builderRot, basePos);
                pose.Anchor = parent.Center + math.mul(parent.Rot, (float3)n.attachLocal);
                pose.Rot = math.mul(parent.Rot, ShortestArcDownTo(dirLocal));
                pose.DirW = math.mul(parent.Rot, dirLocal);
            }
            pose.BoneLength = n.size.y;
            pose.Center = pose.Anchor + pose.DirW * (n.size.y * 0.5f);
            frames[n.name] = pose;
            return pose;
        }
    }
}
