using System;
using System.Collections.Generic;
using Box3D;
using Box3D.Hybrid;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Retargets an arbitrary animated character onto a built Box3DRigBuilder
    /// through manually placed endpoint transforms, two per rig node:
    ///  - pivot: where the node's joint sits on the source character (socket);
    ///  - tip:   the node's distal end.
    ///
    /// All inference is DISPLACEMENT FROM REST: at bind, each segment's rest
    /// direction is captured from the placed points (source rest pose == rig
    /// spawn pose), so the source's attachment offsets and proportions never
    /// contaminate the angles. The rig's own attachLocal stays authoritative
    /// for the physical pivot location.
    ///
    /// Twist handling (twist about the bone axis is unobservable from two
    /// points per segment):
    ///  - Nodes marked "Chain Reference" (e.g. pelvis, torso) get a full
    ///    orientation frame: primary = pivot->tip, roll = pivot->roll point
    ///    (a lateral point such as the opposite-side limb socket). Their
    ///    measured frame delta yields BOTH swing and twist.
    ///  - Segments are grouped into CHAINS along one authored rig axis
    ///    (config directions, not endpoint positions). Spherical members
    ///    between references get their twist interpolated by arc-length
    ///    fraction between the surrounding reference observations.
    ///  - A hinge child is a derived twist OBSERVATION for its parent: the
    ///    bend direction of the child about the parent's axis reveals the
    ///    parent's roll. Compared as a bend PLANE (mod 180 deg, so hinge
    ///    hyperextension reads as zero twist, not 180), weighted by bend
    ///    magnitude (a straight hinge has no signal). The hinge joint itself
    ///    never receives a twist target.
    ///  - No reference at all: swing-only, twist stays spring-zeroed.
    ///
    /// Workflow: on rig assignment the mapping list is auto-populated (one
    /// entry per node, pivot/tip inferred from the rig layout, drawn as
    /// gizmos at the driver's position). Each entry gets optional target
    /// Transforms on the source character; assigning one captures the
    /// current world point into the local offset so the visualization
    /// doesn't jump. Entries can be hand-tuned afterwards. The rig builder's
    /// own definitions are never modified. No root motion. Runs in
    /// FixedUpdate; zero per-frame allocations.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class HumanoidMotionDriver : MonoBehaviour
    {
        [Serializable]
        public class PointRef
        {
            [Tooltip("Transform on the source character carrying this point. Null = offset relative to this component.")]
            public Transform target;
            [Tooltip("Point offset in the target's local space (driver's space while target is null).")]
            public Vector3 offset;
        }

        [Serializable]
        public class Endpoint
        {
            [Tooltip("Rig node this endpoint describes.")]
            public string nodeName = "";
            [Tooltip("Distal end of the node's segment on the source character.")]
            public PointRef tip = new PointRef();
            [Tooltip("Joint pivot (socket) of this node's joint on the source character.")]
            public PointRef pivot = new PointRef();
            [Tooltip("Full reference node: its orientation is fully measured (swing + twist) from its own points and feeds the chain twist blend. Mark chain anchors such as pelvis and torso.")]
            public bool chainReference = false;
            [Tooltip("Lateral reference point defining the roll axis (e.g. the opposite limb's socket). Required for chain references.")]
            public PointRef roll = new PointRef();
        }

        [Header("Source / Target")]
        [SerializeField] private Box3DRigBuilder rig;
        [SerializeField, Tooltip("Optional MxM graph tap (Box3DPhysicalAnimator). When valid, per-step endpoint points are sampled stream-side from the tapped pose and extrapolated to the upcoming step - no transform reads, no pipeline latency. Null/invalid = live transform sampling (legacy path).")]
        private Box3DPhysicalAnimator physicalPoseSource;
        private IWorldPoseSource m_poseSrc;

        [Header("Bind Asset")]
        [SerializeField, Tooltip("Serialized rest-pose bind data. When set, bind uses the captured tip/pivot/roll points instead of resolving them from the live source pose - so the source does NOT need to be at rest at bind time. Per-step observation still reads the live points.")]
        private HumanoidMotionBindAsset bindAsset;

        [Header("Endpoints")]
        [SerializeField, Tooltip("One entry per rig node; each joint reads pivot/tip of its own segment.")]
        private List<Endpoint> endpoints = new List<Endpoint>();

        [Serializable]
        public class GodForceSpec
        {
            [Tooltip("Rig node whose body is driven. The reference pose is derived from this bind: animated root motion (re-anchored to the rig root) offset by the node's bind-relative pose.")]
            public string nodeName = "";
            [Tooltip("Position spring (1/s^2). 0 = rotation-only (recommended for stability entries; feet/limbs should be position-free for locomotion).")]
            public float positionGain = 0f;
            [Range(0.05f, 2f)] public float positionDamping = 1f;
            [Tooltip("Force cap (N); 0 = uncapped.")]
            public float maxForce = 0f;
            [Tooltip("Rotation stiffness (1/s^2) toward the animated spine alignment.")]
            public float rotationGain = 900f;
            [Range(0.05f, 2f)] public float rotationDamping = 1f;
            [Tooltip("Torque cap (N.m); 0 = uncapped. Ignored when per-axis limits are set.")]
            public float maxTorque = 0f;
            [Tooltip("Per-axis torque limits (N.m) in the REFERENCE frame (bone local space): X/Y/Z components of the torque are clamped independently. 0 = unlimited on that axis. Any non-zero value disables the scalar maxTorque cap.")]
            public float torqueLimitX = 0f;
            public float torqueLimitY = 0f;
            public float torqueLimitZ = 0f;
            [Tooltip("Axis remap: which TARGET-LOCAL axis the body's local +X/+Y/+Z must align to. Default identity - body axes chase the ghost bone's axes directly (the bind offset already encodes the body-bone relationship). Set only for deliberate remaps, e.g. a wrist: (0,0,1),(0,-1,0),(1,0,0) maps body X to target Z, body Y to -target Y, body Z to target X. Columns must be orthonormal; sanitized at use.")]
            public Vector3 axisRemapX = new Vector3(1f, 0f, 0f);
            public Vector3 axisRemapY = new Vector3(0f, 1f, 0f);
            public Vector3 axisRemapZ = new Vector3(0f, 0f, 1f);
            [Tooltip("Rotation offset (degrees, Euler XYZ) applied AFTER the bind-relative alignment: desiredRot = boneRot * relRot * offset. Identity by default - the bind offset already encodes the correct body-bone relationship. Use for deliberate axis twists.")]
            public Vector3 rotOffsetEulerDeg = Vector3.zero;
            [Tooltip("Position offset (meters, BONE-LOCAL space) added to the bind-relative pivot: desiredPos = bonePos + boneRot * (relPos + offset). Zero by default - the bind offset already places the body at its capture pose. Use to nudge a body along the bone's axes (e.g. push a chest segment forward/back along the bone's Z).")]
            public Vector3 posOffset = Vector3.zero;
        }
        [Header("God Forces (auto-setup on bind)")]
        [SerializeField, Tooltip("Stability entries applied to the god-forces component on bind (created on this GameObject if absent/none assigned). Gains only - the reference pose comes from the bind.")]
        private List<GodForceSpec> godForceEntries = new List<GodForceSpec>();
        [SerializeField, Tooltip("Explicit god-forces component; empty = find or create one on this GameObject.")]
        private Box3DGodForces godForces;
        [SerializeField, Tooltip("Optional physics pose bias: built from this driver's bind (node->body map) right after the god-forces setup.")]
        private Box3DPhysicsPoseBias poseBias;

        [Header("Pre-fall tracking ramp")]
        [SerializeField, Tooltip("When the god-forces fall detector nears a fall, ramp trackingHertz down (soft-constraint hertz on all bound joints). Progress comes from the god-forces PreFallProgress event.")]
        private bool fallRampEnabled = false;
        [SerializeField, Range(0f, 1f), Tooltip("Tracking hertz floor at full fall progress, as a fraction of the hertz captured at bind.")]
        private float fallRampMinPct = 0.25f;
        [SerializeField, Tooltip("Absolute tracking hertz applied once the fall has actually triggered (progress = 1). Near-zero hertz = near-free joints for the ragdoll duration.")]
        private float fallHertzFloor = 0.1f;

        [Header("Tracking")]
        [SerializeField, Tooltip("Drive springs ON: keep the pose spring enabled ALONGSIDE the servo motors (compound controller - the torque-uncapped spring dominates the capped motor). Debug combo only; OFF for motor-only drive.")]
        private bool driveSprings = false;
        [SerializeField, Tooltip("Apply tracking spring values to every bound joint every step (live-tunable in the inspector during play).")]
        private bool retuneSprings = true;
        [SerializeField, Range(1f, 120f)] private float trackingHertz = 20f;
        [SerializeField, Range(0.1f, 2f)] private float trackingDamping = 1f;
        [SerializeField, Tooltip("Actuator choice. ON (default): spring-driven - the joint pose spring tracks relT (torque-uncapped, rock-stable, motors disabled at bind). OFF: velocity-motor servo (spring disabled at bind, ServoCommand commands relative velocity; needs the clamped ServoCommand to avoid limit-cycling).")]
        private bool springDriven = true;
        [SerializeField, Tooltip("Hinge twist observation pre-pass (pass A of retargeting). Only feeds parent twist via hinge bend-plane measurement. OFF when twist comes from chain references (thigh/upper-arm refs): skips an entire hierarchy pass per step.")]
        private bool hingeTwistObservation = true;
        [SerializeField, Range(1f, 60f), Tooltip("Position->velocity gain (1/s): commanded speed = gain * angle error. NOT per-step - per-step gain assumes instant velocity realization and limit-cycles (BackboneWaveProbe finding).")]
        private float servoGain = 10f;
        [SerializeField, Tooltip("Absolute ceiling on commanded joint speed (rad/s).")]
        private float maxCommandSpeed = 25f;
        [SerializeField, Range(0f, 2f), Tooltip("Damping on the measured relative angular velocity, in the joint frame (DriveSignProbe's kd). Without it the P-only velocity servo limit-cycles.")]
        private float motorDamping = 0.2f;
        [SerializeField, Range(0f, 2f), Tooltip("Reference feedforward on the servo command (motor mode): tracks the animated target's own velocity + acceleration in joint space (lead time = one step). 0 = pure PD servo; 1 = full velocity+accel feedforward.")]
        private float servoFeedforward = 0f;
        [SerializeField, Range(0f, 2f), Tooltip("Scale on the ACCELERATION term of the servo feedforward (alphaRef*dt - a second difference of the retargeted pose, so it injects per-step velocity spikes during fast motion; the at-rest behavior is unaffected since the term is ~0). 1 = full accel lead, 0 = velocity-only FF (recommended if the servo jitters during limb motion).")]
        private float servoFeedforwardAccel = 1f;
        [SerializeField, Range(0f, 2f), Tooltip("Reference feedforward on the spring target (spring mode): advances the commanded angle/rotation by the animated target's own velocity + acceleration (lead time = one step). Separate from the servo FF so the two actuator modes tune independently. 0 = plain target tracking.")]
        private float springFeedforward = 0f;
        [SerializeField, Range(0f, 1f), Tooltip("Gravity load feedforward (motor mode): the distal weight's angular acceleration about the command axis is added to each command as a per-step velocity lead - the standing-bias term a velocity servo otherwise settles into (no integral action). 1 = full compensation, 0 = off.")]
        private float gravityFeedforward = 1f;
        [SerializeField, Tooltip("Motor torque cap per joint (Nm) - authority for gravity compensation and tracking.")]
        private float maxMotorTorque = 300f;
        [SerializeField, Range(-45f, 45f), Tooltip("Post-bind leg splay (deg): all observed directions of each leg chain are rotated rigidly about the hip pivot, outward. Fixes foot-overfoot collision mid walk-cycle without disabling collisions. Applies live (no rebind needed); 0 = off.")]
        private float legSplitDeg = 0f;
        [SerializeField, Tooltip("Splay rotation axis in the root's bind frame. The character's forward (both legs rotate about it in opposite senses).")]
        private Vector3 legSplitAxisLocal = Vector3.forward;
        [SerializeField, Tooltip("How fast the commanded pose follows the rig root's measured rotation (Hz). The root is dynamic, so its fast reaction to limb torques must be filtered out of the feedback path or the rig oscillates.")]
        private float baseFollowHertz = 3f;
        [SerializeField] private bool drawExpectedPose = true;
        [SerializeField] private Vector3 expectedPoseOffset = Vector3.zero; // world offset for the ghost skeleton

        [Header("Settings Transfer")]
        [SerializeField, Tooltip("Source driver to copy from. Used by the 'Copy Settings From Source' context menu.")]
        private HumanoidMotionDriver settingsSource;
        [SerializeField, Tooltip("Root of the OLD source-character hierarchy. PointRef targets are remapped by relative path from here. Null = targets copied as-is (same source character).")]
        private Transform copyRemapFromRoot;
        [SerializeField, Tooltip("Root of the NEW source-character hierarchy. Null = targets copied as-is.")]
        private Transform copyRemapToRoot;

        /// <summary>Target rig (read access for the custom inspector).</summary>
        public Box3DRigBuilder Rig => rig;

        /// <summary>Endpoint count (for Box3DPhysicalAnimator setup).</summary>
        public int EndpointCount => endpoints != null ? endpoints.Count : 0;
        /// <summary>Endpoint i (for Box3DPhysicalAnimator setup).</summary>
        public Endpoint GetEndpoint(int i) => endpoints[i];

        /// <summary>Capture accessor for the custom editor.</summary>
        public HumanoidMotionBindAsset BindAsset => bindAsset;

        /// <summary>Copies retarget settings from 'source' onto this driver.
        /// Scalars + endpoints (deep clone). PointRef targets are remapped by
        /// hierarchy path from copyRemapFromRoot to copyRemapToRoot when both
        /// are set; target-less offsets are driver-relative and copied as-is
        /// (valid when the driver sits at the same pose relative to the
        /// character). The rig field is NOT copied - this driver keeps its
        /// own rig reference.</summary>
        [ContextMenu("Copy Settings From Source")]
        public void CopySettingsFromSource()
        {
            if (settingsSource == null) { Debug.LogWarning("[HumanoidMotionDriver] settingsSource not set.", this); return; }
            if (settingsSource == this) return;
            var src = settingsSource;

            driveSprings = src.driveSprings;
            retuneSprings = src.retuneSprings;
            trackingHertz = src.trackingHertz;
            trackingDamping = src.trackingDamping;
            springDriven = src.springDriven;
            hingeTwistObservation = src.hingeTwistObservation;
            servoGain = src.servoGain;
            maxCommandSpeed = src.maxCommandSpeed;
            motorDamping = src.motorDamping;
            servoFeedforward = src.servoFeedforward;
            servoFeedforwardAccel = src.servoFeedforwardAccel;
            springFeedforward = src.springFeedforward;
            gravityFeedforward = src.gravityFeedforward;
            maxMotorTorque = src.maxMotorTorque;
            legSplitDeg = src.legSplitDeg;
            legSplitAxisLocal = src.legSplitAxisLocal;
            baseFollowHertz = src.baseFollowHertz;
            drawExpectedPose = src.drawExpectedPose;
            expectedPoseOffset = src.expectedPoseOffset;
            bindAsset = src.bindAsset; // shared asset - driver-local data, no remap needed

            endpoints.Clear();
            foreach (var e in src.endpoints)
                endpoints.Add(new Endpoint
                {
                    nodeName = e.nodeName,
                    chainReference = e.chainReference,
                    tip = RemapPoint(e.tip),
                    pivot = RemapPoint(e.pivot),
                    roll = RemapPoint(e.roll),
                });

#if UNITY_EDITOR
            UnityEditor.EditorUtility.SetDirty(this);
#endif
            Debug.Log($"[HumanoidMotionDriver] Copied {endpoints.Count} endpoints + tracking settings from '{src.name}'.", this);
        }

        /// <summary>Writes the current resolved endpoint points into the
        /// assigned bind asset, in RIG-ROOT-LOCAL space. Run this while the
        /// endpoint targets sit on the source character AT THE REST POSE
        /// (t-pose). Does not require a built rig or a bound driver, but
        /// requires the rig reference (the storage frame is the rig root).</summary>
        [ContextMenu("Capture Bind To Asset")]
        public void CaptureBindToAsset()
        {
            if (bindAsset == null) { Debug.LogWarning("[HumanoidMotionDriver] No bind asset assigned.", this); return; }
            if (rig == null) { Debug.LogWarning("[HumanoidMotionDriver] No rig assigned - the capture frame is the rig root.", this); return; }
            var root = rig.transform;
            bindAsset.points.Clear();
            bindAsset.SetRigRootFrame();
            foreach (var e in endpoints)
            {
                if (string.IsNullOrEmpty(e.nodeName)) continue;
                bindAsset.points.Add(new HumanoidMotionBindAsset.BindPoint
                {
                    nodeName = e.nodeName,
                    tip = root.InverseTransformPoint((Vector3)ResolvePoint(e.tip)),
                    pivot = root.InverseTransformPoint((Vector3)ResolvePoint(e.pivot)),
                    roll = root.InverseTransformPoint((Vector3)ResolvePoint(e.roll)),
                });
            }
            Debug.Log($"[HumanoidMotionDriver] Captured {bindAsset.points.Count} rest points (rig-root local) into '{bindAsset.name}'.", this);
        }

        private PointRef RemapPoint(PointRef p)
        {
            if (p.target == null || copyRemapFromRoot == null || copyRemapToRoot == null)
                return new PointRef { target = p.target, offset = p.offset };
            var t = p.target;
            var path = new List<string>();
            while (t != null && t != copyRemapFromRoot)
            {
                path.Add(t.name);
                t = t.parent;
            }
            if (t == null)
            {
                Debug.LogWarning($"[HumanoidMotionDriver] Remap: '{p.target.name}' is not under '{copyRemapFromRoot.name}' - target kept as-is.", this);
                return new PointRef { target = p.target, offset = p.offset };
            }
            var cur = copyRemapToRoot;
            for (int i = path.Count - 1; i >= 0 && cur != null; i--)
            {
                cur = cur.Find(path[i]);
                if (cur == null)
                    Debug.LogWarning($"[HumanoidMotionDriver] Remap: '{path[i]}' not found under '{copyRemapToRoot.name}'.", this);
            }
            return new PointRef { target = cur, offset = p.offset };
        }

        // Per-node state, indexed by endpoint index. Pool allocated at bind,
        // zero per-frame allocation afterwards.
        private struct NodeState
        {
            // --- topology (bind) ---
            public int parent;              // node index, -1 for the root
            public bool hasJoint;
            public bool isHinge;
            public bool isChainRef;
            public int chain;               // chain id (-1 = none)
            public float s;                 // arc length from chain start (rest pivots)
            public float nextRefS;          // arc length of nearest chain ref above (>= s)
            public int nextRefIdx;          // that ref's node index (-1 = none)
            public int baseRef;             // nearest chain ref on the ancestor path (-1 = none): twist base at the chain start
            // --- bind-time geometry ---
            public float3 restDirW;         // pivot -> tip at rest (world)
            public float3 restBendW;        // hinge children: bend dir of the child at rest (world, on the PARENT)
            public float3 hingeAxisChild;   // hinge axis in the child's spawn frame
            public quaternion spawnChildRot;
            public quaternion frameARot;    // joint frame in parent's spawn frame
            public quaternion frameBRot;    // joint frame in child's spawn frame
            public quaternion restFrame;    // chain refs: rest orientation frame (world)
            public quaternion bindFrame;    // estimated source segment frame at bind (world)
            public int dirChild;            // child index providing a synthetic bone dir (degenerate tip), -1 = none
            // --- physics handles ---
            public Box3DBody body;
            public Box3DBody parentBody;
            public Box3D.Joint joint;
            public float3 frameAPos;        // joint anchor in the parent's spawn frame
            public float torqueCap;         // motor torque cap (per-node override or global)
            public Body[] distal;           // distal subtree bodies (composite inertia for drive shaping)
            // --- runtime ---
            public float3 curDirW;
            public float twist;             // blended twist angle (rad) about curDirW
            public float obsTwist;          // hinge-derived observation (rad)
            public float obsConf;           // observation confidence (0..1)
        }

        private float3[] m_tips = Array.Empty<float3>();
        private float3[] m_pivots = Array.Empty<float3>();
        private float3[] m_rolls = Array.Empty<float3>();
        private float3[] m_gCom = Array.Empty<float3>();            // gravity FF scratch: distal COMs (grown lazily)
        private float[] m_gMass = Array.Empty<float>();             // gravity FF scratch: distal masses
        private Body[][] m_distalBase = Array.Empty<Body[]>();      // bind-time distal lists (payload-free base for rebuilds)
        private List<Body>[] m_payloadsByNode = Array.Empty<List<Body>>(); // attached payloads per node index
        private NodeState[] m_nodes = Array.Empty<NodeState>();
        private int[] m_order = Array.Empty<int>();     // parent-first evaluation order
        [NonSerialized] private bool m_bound;           // runtime state - never persist to the scene
        [NonSerialized] private float m_fallRampFullHz = -1f; // trackingHertz captured at bind (pre-fall ramp base)
        [NonSerialized] private bool m_fallRampActive;
        [NonSerialized] private quaternion m_baseDelta; // runtime: filtered root rotation vs spawn (commanded-pose frame)

        // Expected-pose visualization buffers (filled in FixedUpdate, drawn
        // by the editor gizmo): FK of the commanded joint targets.
        private float3[] m_expPos = Array.Empty<float3>();      // expected world joint positions
        private quaternion[] m_expRot;                          // expected world rotations (root: measured frame)
        private MotionTransform.ChainSimNode[] m_chainSim = Array.Empty<MotionTransform.ChainSimNode>();
        private quaternion[] m_invCmdScratch = Array.Empty<quaternion>(); // per-pass inv(cmdRot) cache
        private float3[] m_tick;                                // expected roll-tick offsets (0 = none)
        private float[] m_edgeLen;                              // bind: |pivot - parentPivot|
        private float3[] m_restOffsetLocal;                     // bind: pivot offset from parent, parent spawn frame
        private float3[] m_bindPivots;                          // bind: world pivots at bind time
        private float3[] m_tipLocal;                            // bind: tip offset from pivot, node spawn frame
        private bool m_poseValid;

        private const float ChainAxisDot = 0.9f;        // |dot| threshold for same-chain (approx 25 deg)
        private const float ObsConfidenceMin = 0.15f;   // min bend |sin| for a hinge observation to win

        private void Start() => Bind();

        private void OnEnable() => Bind();

        private void OnDisable()
        {
            if (godForces != null) godForces.PreFallProgress -= OnPreFallProgress;
        }

        private void OnDestroy()
        {
            if (godForces != null) godForces.PreFallProgress -= OnPreFallProgress;
        }

        // Live actuator-mode switch: the motor/spring ownership is applied at
        // bind, so a checkbox flip must rebind to take effect. Only fires on
        // an actual mode change, not on every inspector nudge.
        private bool m_lastSpringDriven = true;
        private void OnValidate()
        {
            if (Application.isPlaying && m_bound && springDriven != m_lastSpringDriven)
                Rebind();
        }

        // ------------------------------------------------------------------
        // Bind

        private void Bind()
        {
            m_lastSpringDriven = springDriven;
            if (m_bound || rig == null || !rig.IsBuilt || endpoints.Count == 0) return;

            var indexByName = new Dictionary<string, int>(endpoints.Count);
            for (int i = 0; i < endpoints.Count; i++)
                if (!string.IsNullOrEmpty(endpoints[i].nodeName)) indexByName[endpoints[i].nodeName] = i;

            int count = endpoints.Count;
            m_tips = new float3[count];
            m_pivots = new float3[count];
            m_rolls = new float3[count];
            m_nodes = new NodeState[count];
            m_expPos = new float3[count];
            m_expRot = new quaternion[count];
            m_chainSim = new MotionTransform.ChainSimNode[count];
            m_invCmdScratch = new quaternion[count];
            m_tick = new float3[count];
            m_edgeLen = new float[count];
            m_restOffsetLocal = new float3[count];
            m_bindPivots = new float3[count];
            m_tipLocal = new float3[count];
            m_poseValid = false;
            var warnings = new List<string>();

            // Pass 1: points + per-node joint bookkeeping. With a rig-root-
            // framed bind asset, rest points come from the capture
            // (pose-independent); missing entries or a legacy driver-local
            // asset fall back to the live source pose.
            for (int i = 0; i < count; i++)
            {
                var e = endpoints[i];
                var bp = bindAsset != null && bindAsset.IsRigRootFrame ? bindAsset.Find(e.nodeName) : null;
                if (bp != null)
                {
                    m_tips[i] = rig.transform.TransformPoint(bp.tip);
                    m_pivots[i] = rig.transform.TransformPoint(bp.pivot);
                    m_rolls[i] = rig.transform.TransformPoint(bp.roll);
                }
                else
                {
                    if (bindAsset != null && !bindAsset.IsRigRootFrame)
                        warnings.Add($"'{e.nodeName}': bind asset is legacy driver-local - live pose used (recapture)");
                    else if (bindAsset != null)
                        warnings.Add($"'{e.nodeName}': no bind-asset entry - live pose used");
                    m_tips[i] = ResolvePoint(e.tip);
                    m_pivots[i] = ResolvePoint(e.pivot);
                    m_rolls[i] = ResolvePoint(e.roll);
                }

                var n = new NodeState { parent = -1, chain = -1, nextRefIdx = -1, baseRef = -1, nextRefS = float.PositiveInfinity, dirChild = -1 };
                var node = rig.FindNode(e.nodeName);
                if (node == null) { warnings.Add($"'{e.nodeName}': node missing from rig config"); m_nodes[i] = n; continue; }

                n.isChainRef = e.chainReference;
                if (node.parentName == "")
                {
                    // Root has no joint but may be a chain ref (pelvis): keep
                    // its body/spawn rotation so the gizmo can tick its twist.
                    if (node.body != null)
                    {
                        n.body = node.body;
                        n.spawnChildRot = node.body.transform.rotation;
                    }
                    // Root endpoints measure full frames too. Without a rest
                    // direction the bind frame falls back to WORLD-UP while
                    // the per-step frame uses the measured bone direction -
                    // anti-parallel for a downward pelvis bone -> a 180 deg
                    // relT injected at every chain base.
                    n.restDirW = math.normalizesafe(m_tips[i] - m_pivots[i], node.dirW);
                    m_tipLocal[i] = math.mul(math.inverse(n.spawnChildRot), m_tips[i] - m_pivots[i]);
                    m_nodes[i] = n;
                    continue;
                }
                var parentNode = rig.FindNode(node.parentName);
                var parentBody = rig.FindBody(node.parentName);
                if (parentNode == null || parentBody == null)
                {
                    warnings.Add($"'{e.nodeName}': parent '{node.parentName}' missing");
                    m_nodes[i] = n;
                    continue;
                }
                if (node.body == null || node.jointRef.Equals(default(Box3D.Joint)))
                {
                    warnings.Add($"'{e.nodeName}': not built");
                    m_nodes[i] = n;
                    continue;
                }
                if (indexByName.TryGetValue(node.parentName, out int pIdx)) n.parent = pIdx;

                bool isHinge = node.joint == Box3DRigBuilder.JointKind.Hinge;
                quaternion spawnChildRot = node.body.transform.rotation;
                quaternion spawnParentRot = parentBody.transform.rotation;
                // Joint frame MUST be parity with the builder (CreateHinge/
                // CreateSpherical): hinge Z = the child's authored axis
                // rotated to world by the SPAWN rotation; spherical Z = the
                // parent's config dir. A mismatch here makes relT carry a
                // constant offset vs the physical joint's frames -> command
                // at rest (oscillation) and bend targets the joint cannot
                // realize (no adaptation).
                quaternion jointFrameW = isHinge
                    ? Box3DRigBuilder.FrameFromAxisWorld(math.mul(spawnChildRot,
                        math.normalizesafe((float3)node.hingeAxisLocal, new float3(1f, 0f, 0f))))
                    : Box3DRigBuilder.FrameFromAxisWorld(parentNode.dirW);

                n.hasJoint = true;
                n.isHinge = isHinge;
                n.body = node.body;
                n.parentBody = parentBody;
                n.restDirW = math.normalizesafe(m_tips[i] - m_pivots[i], node.dirW);
                n.hingeAxisChild = isHinge ? math.normalizesafe((float3)node.hingeAxisLocal) : float3.zero;
                n.spawnChildRot = spawnChildRot;
                n.frameARot = math.mul(math.inverse(spawnParentRot), jointFrameW);
                n.frameBRot = math.mul(math.inverse(spawnChildRot), jointFrameW);
                n.joint = node.jointRef;
                // Actuator ownership. Spring-driven: motor off (it fights the
                // pose spring at solver rate and shakes the chain - verified
                // in-editor). Motor-driven: spring off (the stiff identity
                // spring overpowers the capped motor; thighs held relA=0
                // against relT=98 deg).
                if (springDriven)
                {
                    if (isHinge) new RevoluteJoint { Id = node.jointRef.Id }.EnableMotor(false);
                    else new SphericalJoint { Id = node.jointRef.Id }.EnableMotor(false);
                }
                else
                {
                    // Springs stay enabled in the driveSprings debug combo
                    // (spring + motor compound); RetuneSprings re-asserts it
                    // every step. Motor setup pass (below) enables motors
                    // whenever !springDriven.
                    if (!driveSprings)
                    {
                        if (isHinge) new RevoluteJoint { Id = node.jointRef.Id }.EnableSpring(false);
                        else new SphericalJoint { Id = node.jointRef.Id }.EnableSpring(false);
                    }
                }
                n.torqueCap = node.maxMotorTorque > 0f ? node.maxMotorTorque : maxMotorTorque;
                n.frameAPos = math.mul(math.inverse(spawnParentRot), m_pivots[i]);
                m_tipLocal[i] = math.mul(math.inverse(spawnChildRot), m_tips[i] - m_pivots[i]);
                m_nodes[i] = n;
            }

            // Pass 2: chain grouping along authored axes (config dirs, not
            // endpoint positions), parent-first. s accumulates rest pivot
            // distances; the chain length is only used as a sanity bound.
            m_bindPivots = (float3[])m_pivots.Clone();
            BuildOrder();
            for (int oi = 0; oi < m_order.Length; oi++)
            {
                int i = m_order[oi];
                ref var n = ref m_nodes[i];
                if (n.parent < 0) continue;
                ref var p = ref m_nodes[n.parent];

                if (n.hasJoint && n.parent >= 0)
                {
                    // Bind offset of this pivot from the parent's pivot, in
                    // the parent body's spawn frame - rotates with the
                    // expected parent rotation, so lateral attachments
                    // (shoulders, hips) reconstruct correctly.
                    m_restOffsetLocal[i] = math.mul(math.inverse(m_nodes[n.parent].spawnChildRot),
                        m_pivots[i] - m_pivots[n.parent]);
                    m_edgeLen[i] = math.length(m_restOffsetLocal[i]);
                }
            }

            // Pass 2+3: chain growth (aligned rest bones), arc lengths,
            // nearest chain ref above / on the ancestor path - pure
            // (MotionTransform.BuildChainTopology), mirrored through the
            // sim-node scratch array.
            for (int i = 0; i < m_nodes.Length; i++)
            {
                ref var c = ref m_chainSim[i];
                c.restPivotW = m_pivots[i];
                c.restDirW = m_nodes[i].restDirW;
                c.hasJoint = m_nodes[i].hasJoint;
                c.isChainRef = m_nodes[i].isChainRef;
                c.parent = m_nodes[i].parent;
            }
            MotionTransform.BuildChainTopology(m_chainSim, m_order, ChainAxisDot);
            for (int i = 0; i < m_nodes.Length; i++)
            {
                m_nodes[i].chain = m_chainSim[i].chain;
                m_nodes[i].s = m_chainSim[i].s;
                m_nodes[i].nextRefIdx = m_chainSim[i].nextRefIdx;
                m_nodes[i].nextRefS = m_chainSim[i].nextRefS;
                m_nodes[i].baseRef = m_chainSim[i].baseRef;
            }

            // Pass 3b: degenerate bone directions. A node whose tip == pivot
            // (pelvis, head, hands) has no measurable bone axis; the pipeline
            // would fall back to a WORLD-CONSTANT primary, which is not
            // covariant under rigid source rotation (root-rotation leak -
            // spurious per-chain swings). Synthesize the direction from the
            // first child's pivot instead: measured, rotates with the source.
            for (int i = 0; i < m_nodes.Length; i++)
            {
                if (math.lengthsq(m_bindPivots[i] - m_tips[i]) > 1e-10f) continue;
                int child = -1;
                for (int j = 0; j < m_nodes.Length; j++)
                    if (m_nodes[j].parent == i) { child = j; break; }
                if (child < 0) continue;
                m_nodes[i].dirChild = child;
                m_nodes[i].restDirW = math.normalizesafe(m_bindPivots[child] - m_bindPivots[i],
                    math.normalizesafe(m_nodes[i].restDirW, new float3(0, 1, 0)));
            }

            // Pass 4: rest frames for chain refs; rest bend dirs for hinge
            // observations; spring retune.
            for (int oi = 0; oi < m_order.Length; oi++)
            {
                int i = m_order[oi];
                ref var n = ref m_nodes[i];
                if (n.isChainRef)
                {
                    if (endpoints[i].roll.target == null && endpoints[i].roll.offset == Vector3.zero)
                    {
                        warnings.Add($"'{endpoints[i].nodeName}': chain reference without a roll point - treated as swing-only");
                        n.isChainRef = false;
                    }
                    else
                    {
                        n.restFrame = FrameFromPrimaryRoll(n.restDirW,
                            math.normalizesafe(m_rolls[i] - m_pivots[i], OrthoTo(n.restDirW)));
                    }
                }
                if (n.parent < 0 && !n.isChainRef)
                    warnings.Add("no measured root chain reference (pelvis with roll point) - root-relative invariance is NOT guaranteed; the projected pose will distort under source root rotation");
                if (n.isHinge && n.parent >= 0)
                {
                    // Rest bend direction of the child about the parent axis:
                    // the parent's roll reference datum.
                    ref var p = ref m_nodes[n.parent];
                    float3 childRest = n.restDirW;
                    float3 onPlane = childRest - p.restDirW * math.dot(childRest, p.restDirW);
                    if (math.lengthsq(onPlane) > 1e-8f)
                        p.restBendW = math.normalizesafe(onPlane);
                }
                if (n.hasJoint)
                {
                    // Motor setup only; spring tuning is continuous (see
                    // RetuneSprings in FixedUpdate). Mode-gated: spring mode
                    // must NOT have motors enabled - the velocity motor
                    // fighting the pose spring at solver rate is the shake.
                    if (springDriven) continue;
                    if (n.isHinge)
                    {
                        var j = new RevoluteJoint { Id = n.joint.Id };
                        j.EnableMotor(true);
                        j.SetMaxMotorTorque(n.torqueCap);
                    }
                    else
                    {
                        var j = new SphericalJoint { Id = n.joint.Id };
                        j.EnableMotor(true);
                        j.SetMaxMotorTorque(n.torqueCap);
                    }
                }
            }

            // Pass 4b: bind segment frames + joint-target preconjugations
            // (MotionTransform.ComposeBindFrames - same recursion as the
            // per-step pipeline, so bind == pose identity). Bind roll
            // directions: chain refs get their captured roll point.
            for (int i = 0; i < m_nodes.Length; i++)
            {
                ref var c = ref m_chainSim[i];
                c.restDirW = m_nodes[i].restDirW;
                c.restFrame = m_nodes[i].restFrame;
                c.spawnChildRot = m_nodes[i].spawnChildRot;
                c.frameARot = m_nodes[i].frameARot;
                c.frameBRot = m_nodes[i].frameBRot;
                if (m_nodes[i].isChainRef)
                    c.curRollDirW = math.normalizesafe(m_rolls[i] - m_pivots[i], OrthoTo(m_nodes[i].restDirW));
            }
            MotionTransform.ComposeBindFrames(m_chainSim, m_order);
            for (int i = 0; i < m_nodes.Length; i++)
                m_nodes[i].bindFrame = m_chainSim[i].bindFrame;

            // Pass 5: distal body lists per joint - the composite inertia the
            // drive's stopping-velocity clamp needs (BackboneWaveProbe recipe).
            // Base snapshots kept for payload rebuilds (runtime attachments).
            var distalTmp = new List<Body>(count);
            m_distalBase = new Body[count][];
            m_payloadsByNode = new List<Body>[count];
            for (int i = 0; i < count; i++)
            {
                ref var n = ref m_nodes[i];
                if (!n.hasJoint) continue;
                distalTmp.Clear();
                for (int jdx = 0; jdx < count; jdx++)
                {
                    if (jdx == i) continue;
                    for (int p = m_nodes[jdx].parent; p >= 0; p = m_nodes[p].parent)
                    {
                        if (p != i) continue;
                        if (m_nodes[jdx].body != null) distalTmp.Add(m_nodes[jdx].body.Body);
                        break;
                    }
                }
                n.distal = distalTmp.ToArray();
                m_distalBase[i] = n.distal;
            }

            if (warnings.Count > 0)
                Debug.LogWarning($"[HumanoidMotionDriver] {warnings.Count} issue(s):\n  " + string.Join("\n  ", warnings), this);

            m_baseDelta = quaternion.identity;
            m_bound = true;
            m_poseValid = false;
            // Re-push payloads registered while unbound (registry may have
            // enabled before the driver bound).
            FindFirstObjectByType<Box3DPayloadRegistry>()?.PushAll();
            RetuneSprings(); // apply actuator mode immediately, don't wait for an inspector nudge
            BuildLegSplay();

            // FF state reset: first tick after bind starts with zero velocity
            // reference (relT history is empty).
            m_prevRelT = new quaternion[m_nodes.Length];
            m_prevWRef = new float3[m_nodes.Length];
            m_wRefInit = new bool[m_nodes.Length];
            BindGodForces();
            Debug.Log($"[HumanoidMotionDriver] bound {CountJoints()} joints, {CountRefs()} chain reference(s) on '{rig.name}'", this);
        }

        // --- god-forces auto-setup (post-bind) ------------------------------

        /// <summary>Push the god-force specs as driver-driven entries: per node
        /// the body pose RELATIVE TO ITS OWN GHOST BONE (tip PointRef target,
        /// captured at bind) becomes the reference offset. At runtime the god
        /// forces re-apply that offset on the ANIMATED bone frame - so both
        /// position and rotation track the actual animated lean of that bone,
        /// not just the root.</summary>
        private void BindGodForces()
        {
            if (godForceEntries.Count == 0) return;
            if (godForces == null)
                godForces = GetComponent<Box3DGodForces>();
            if (godForces == null)
                godForces = gameObject.AddComponent<Box3DGodForces>();
            // Driver-driven entries take their reference from the animated root
            // pose - they need the same graph tap the driver's retarget uses.
            godForces.SetPoseSource(physicalPoseSource);

            Transform animRoot = physicalPoseSource != null ? physicalPoseSource.AnimRoot : null;
            Box3DBody rootBody = rig.Root;
            if (animRoot == null || rootBody == null)
            { Debug.LogWarning("[HumanoidMotionDriver] god-force setup: no animation root / rig root body", this); return; }

            var list = new List<Box3DGodForces.Entry>(godForceEntries.Count);
            var relPos = new Vector3[godForceEntries.Count];
            var relRot = new Quaternion[godForceEntries.Count];
            var bindPos = new float3[godForceEntries.Count];
            var bindRot = new quaternion[godForceEntries.Count];
            int entryCount = 0;
            for (int i = 0; i < godForceEntries.Count; i++)
            {
                var spec = godForceEntries[i];
                int idx = -1;
                for (int j = 0; j < endpoints.Count; j++)
                    if (endpoints[j].nodeName == spec.nodeName) { idx = j; break; }
                if (idx < 0)
                { Debug.LogWarning($"[HumanoidMotionDriver] god-force spec: no node named '{spec.nodeName}' - skipped", this); continue; }

                // Bind-relative body pose vs THIS node's own ghost bone (tip
                // PointRef target). At runtime god forces re-applies it on the
                // animated bone frame: tPos = bonePos + boneRot*relPos,
                // tRot = boneRot * relRot. Tracks animated lean directly.
                Transform bone = endpoints[idx].tip.target;
                if (bone == null)
                { Debug.LogWarning($"[HumanoidMotionDriver] god-force spec '{spec.nodeName}': null tip target", this); continue; }
                quaternion invBoneRot = math.inverse((quaternion)bone.rotation);
                relPos[entryCount] = math.mul(invBoneRot,
                    (float3)(rig.FindBody(spec.nodeName).Body.Position) - m_pivots[idx]);
                relRot[entryCount] = math.mul(invBoneRot, m_nodes[idx].spawnChildRot);
                bindPos[entryCount] = rig.FindBody(spec.nodeName).Body.Position;
                bindRot[entryCount] = m_nodes[idx].spawnChildRot;

                list.Add(new Box3DGodForces.Entry
                {
                    body = rig.FindBody(spec.nodeName),
                    sourceEndpoint = idx,
                    positionGain = spec.positionGain,
                    positionDamping = spec.positionDamping,
                    maxForce = spec.maxForce,
                    rotationGain = spec.rotationGain,
                    rotationDamping = spec.rotationDamping,
                    maxTorque = spec.maxTorque,
                    axisRemapX = spec.axisRemapX,
                    axisRemapY = spec.axisRemapY,
                    axisRemapZ = spec.axisRemapZ,
                    torqueLimitX = spec.torqueLimitX,
                    torqueLimitY = spec.torqueLimitY,
                    torqueLimitZ = spec.torqueLimitZ,
                    rotOffset = Quaternion.Euler(spec.rotOffsetEulerDeg),
                    posOffset = spec.posOffset,
                });
                entryCount++;
            }
            if (list.Count == 0) return;
            godForces.SetupFromDriver(rootBody, list, relPos, relRot,
                SliceArr(bindPos, entryCount), SliceArr(bindRot, entryCount));
            Debug.Log($"[HumanoidMotionDriver] god-forces configured: {list.Count} driver-driven entr(ies)", this);
            if (poseBias != null) poseBias.BuildFromDriver(this); // bind-time: bodies still at spawn pose
            if (godForces != null)
            {
                godForces.PreFallProgress -= OnPreFallProgress; // idempotent
                godForces.PreFallProgress += OnPreFallProgress;
                m_fallRampFullHz = trackingHertz; // ramp base captured at bind
            }
        }

        /// <summary>Truncate to the first n elements (entry arrays are
        /// spec-sized; skipped specs leave trailing slots).</summary>
        private static T[] SliceArr<T>(T[] arr, int n)
        {
            if (n == arr.Length) return arr;
            var r = new T[n];
            System.Array.Copy(arr, r, n);
            return r;
        }

        /// <summary>Bind-time node frame for external consumers (pose bias):
        /// the rig body, bind pivot (world) and spawn rotation (world) of the
        /// node named nodeName. Valid only while bound.</summary>
        public bool TryGetBindNodeFrame(string nodeName, out Box3DBody body,
            out float3 pivotW, out quaternion spawnRotW)
        {
            body = null; pivotW = default; spawnRotW = quaternion.identity;
            if (!m_bound) return false;
            for (int j = 0; j < endpoints.Count; j++)
            {
                if (endpoints[j].nodeName != nodeName) continue;
                body = m_nodes[j].body;
                if (body == null) return false;
                pivotW = m_pivots[j];
                spawnRotW = m_nodes[j].spawnChildRot;
                return true;
            }
            return false;
        }

        // --- runtime payloads (gameplay-attached weights) -------------------
        // Payloads are appended to the HOST node's distal list: the joint
        // connecting the node to its parent carries the payload's gravity
        // load, so both the composite-inertia shaping and the gravity
        // feedforward see it. Base lists are snapshotted at bind so rebuilds
        // stay deterministic across attach/detach cycles.

        /// <summary>Attach a payload body to a rig node's load path.
        /// Called by the Box3DPayloadRegistry (also at bind refresh).</summary>
        public void AttachPayload(Box3DBody payloadBody, string hostNodeName)
        {
            if (!m_bound || payloadBody == null || payloadBody.Body == null
                || string.IsNullOrEmpty(hostNodeName))
            {
                // Unbound is normal for edit-time entries (registry may enable
                // first): Bind() re-pushes the registry, no warning warranted.
                if (m_bound && payloadBody != null)
                    Debug.LogWarning($"[HumanoidMotionDriver] payload '{payloadBody.name}' not attachable (no body handle or no host node)", this);
                return;
            }
            int idx = IndexOfNodeName(hostNodeName);
            if (idx < 0)
            {
                Debug.LogWarning($"[HumanoidMotionDriver] payload '{payloadBody.name}' host node '{hostNodeName}' not found", this);
                return;
            }
            var list = m_payloadsByNode[idx] ??= new List<Body>();
            if (list.Contains(payloadBody.Body)) return; // idempotent
            list.Add(payloadBody.Body);
            RebuildDistal(idx);
        }

        /// <summary>Detach a payload body from wherever it was attached.</summary>
        public void DetachPayload(Box3DBody payloadBody)
        {
            if (payloadBody == null) return;
            var handle = payloadBody.Body;
            for (int i = 0; i < m_payloadsByNode.Length; i++)
            {
                var list = m_payloadsByNode[i];
                if (list == null || !list.Remove(handle)) continue;
                if (m_bound) RebuildDistal(i);
            }
        }

        private void RebuildDistal(int idx)
        {
            var baseList = m_distalBase[idx];
            var payloads = m_payloadsByNode[idx];
            int extra = payloads?.Count ?? 0;
            var merged = new Body[baseList.Length + extra];
            for (int i = 0; i < baseList.Length; i++) merged[i] = baseList[i];
            if (payloads != null)
                for (int i = 0; i < payloads.Count; i++) merged[baseList.Length + i] = payloads[i];
            m_nodes[idx].distal = merged;
        }

        private int IndexOfNodeName(string nodeName)
        {
            for (int i = 0; i < endpoints.Count; i++)
                if (endpoints[i].nodeName == nodeName) return i;
            return -1;
        }

        // --- leg splay bias (post-bind, per-step) ---------------------------
        // Rig bound -> each leg's observed directions rotate rigidly about
        // the hip pivot, outward. Only DIRECTIONS are biased (curDirW /
        // curRollDirW - the only inputs RetargetChains reads); pivots and
        // bind anchors stay put, so the hard point constraints are untouched.
        // Because the bias is a rigid rotation of the whole chain, each
        // node's PARENT-RELATIVE direction is unchanged -> relT is identity
        // below the hip: perfect continuity, one joint target per leg.

        private int[][] m_legNodes = System.Array.Empty<int[]>();   // node indices per leg

        // --- joint feedforward state (per node) -----------------------------
        private quaternion[] m_prevRelT = System.Array.Empty<quaternion>();
        private float3[] m_prevWRef = System.Array.Empty<float3>();
        private bool[] m_wRefInit = System.Array.Empty<bool>();
        private float[] m_legSigns = System.Array.Empty<float>();   // +1 left / -1 right (name match)
        private int[] m_legOf = System.Array.Empty<int>();          // node -> leg index (-1 = none)
        private float3 m_legAxisW;

        private void BuildLegSplay()
        {
            var legs = new List<int[]>();
            var signs = new List<float>();
            // Chain ids of nodes whose endpoint name looks like a thigh.
            // Group membership by chain id -> whole chain splays as one body.
            var legChains = new Dictionary<int, float>();
            for (int i = 0; i < endpoints.Count && i < m_nodes.Length; i++)
            {
                var name = endpoints[i].nodeName;
                if (string.IsNullOrEmpty(name)) continue;
                bool left = name.IndexOf("left", System.StringComparison.OrdinalIgnoreCase) >= 0;
                bool right = name.IndexOf("right", System.StringComparison.OrdinalIgnoreCase) >= 0;
                bool thigh = name.IndexOf("thigh", System.StringComparison.OrdinalIgnoreCase) >= 0
                          || name.IndexOf("upper_leg", System.StringComparison.OrdinalIgnoreCase) >= 0;
                if (!thigh || (!left && !right)) continue;
                int chain = m_nodes[i].chain;
                if (chain >= 0 && !legChains.ContainsKey(chain))
                    legChains[chain] = left ? 1f : -1f;
            }
            foreach (var kv in legChains)
            {
                var members = new List<int>();
                for (int i = 0; i < m_nodes.Length; i++)
                    if (m_nodes[i].chain == kv.Key) members.Add(i);
                if (members.Count > 0) { legs.Add(members.ToArray()); signs.Add(kv.Value); }
            }
            m_legNodes = legs.ToArray();
            m_legSigns = signs.ToArray();
            m_legOf = new int[m_nodes.Length];
            for (int i = 0; i < m_legOf.Length; i++) m_legOf[i] = -1;
            for (int li = 0; li < m_legNodes.Length; li++)
                for (int k = 0; k < m_legNodes[li].Length; k++)
                    m_legOf[m_legNodes[li][k]] = li;
            // Constant world axis from the RIG ROOT BODY's frame (the physics
            // rig root, e.g. pelvis) - NOT the bound source node's bind frame:
            // the bias is authored relative to the physical rig.
            m_legAxisW = rig != null && rig.Root != null && rig.Root.Body.IsValid
                ? math.normalizesafe(math.mul(rig.Root.Body.Rotation, (float3)legSplitAxisLocal))
                : math.normalizesafe((float3)legSplitAxisLocal);
        }

        /// <summary>Rotate the observed directions of each leg chain rigidly
        /// about the hip. Called per step BEFORE RetargetChains, so the
        /// retargeting sees a pre-splayed source: the thigh's observed frame
        /// carries the splay and every descendant's parent-relative
        /// direction cancels it (both sides rotate identically).</summary>
        private void ApplyLegSplay()
        {
            if (legSplitDeg == 0f || m_legNodes.Length == 0) return;
            // Axis from the LIVE rig root body rotation, re-derived per step:
            // a world-baked bind axis is not covariant under body rotation -
            // the character turns, the frozen axis doesn't, and the splay
            // becomes twist/collapse instead of a symmetric split.
            float3 axisW = rig != null && rig.Root != null && rig.Root.Body.IsValid
                ? math.normalizesafe(math.mul(rig.Root.Body.Rotation, (float3)legSplitAxisLocal))
                : math.normalizesafe((float3)legSplitAxisLocal);
            float angle = legSplitDeg * Mathf.Deg2Rad;
            for (int i = 0; i < m_nodes.Length; i++)
            {
                int li = i < m_legOf.Length ? m_legOf[i] : -1;
                if (li < 0) continue;
                var q = quaternion.AxisAngle(axisW, m_legSigns[li] * angle);
                ref var n = ref m_nodes[i];
                n.curDirW = math.normalizesafe(math.mul(q, n.curDirW), n.restDirW);
            }
        }

        /// <summary>Rotate one leg's chain-reference roll datum by the same
        /// splay (called from the mirror loop that fills curRollDirW, so the
        /// full-frame measurement stays consistent with the biased dirs).</summary>
        private quaternion LegRollBias(int nodeIdx)
        {
            int li = nodeIdx < m_legOf.Length ? m_legOf[nodeIdx] : -1;
            if (li < 0 || legSplitDeg == 0f) return quaternion.identity;
            return quaternion.AxisAngle(m_legAxisW, m_legSigns[li] * legSplitDeg * Mathf.Deg2Rad);
        }

        // Push trackingHertz/trackingDamping to every bound joint. The solver
        // rebuilds each substep's soft constraint from the live values, so
        // retuning mid-simulation is seamless - no rebind, no re-create.
        // No-op entirely when no spring is enabled anywhere (servo mode
        // without driveSprings): pushing spring values to disabled springs
        // is 2 wasted native calls per joint per step.
        private void RetuneSprings(float? hzOverride = null)
        {
            bool springActive = springDriven || driveSprings;
            if (!springActive) return;
            for (int i = 0; i < m_nodes.Length; i++)
            {
                ref var n = ref m_nodes[i];
                if (!n.hasJoint) continue;
                float hz = hzOverride ?? trackingHertz;
                if (n.isHinge)
                {
                    var j = new RevoluteJoint { Id = n.joint.Id };
                    j.EnableSpring(springActive);
                    j.SetSpringHertz(hz);
                    j.SetSpringDampingRatio(trackingDamping);
                }
                else
                {
                    var j = new SphericalJoint { Id = n.joint.Id };
                    j.EnableSpring(springActive);
                    j.SetSpringHertz(hz);
                    j.SetSpringDampingRatio(trackingDamping);
                }
            }
        }

        /// <summary>Pre-fall ramp: soften the joint springs as the fall
        /// detector's bucket fills (progress 0..1 from the god-forces event).
        /// Base hertz captured at bind; restores instantly at progress 0.</summary>
        private void OnPreFallProgress(float progress)
        {
            if (!fallRampEnabled || !m_bound) return;
            if (progress <= 0f)
            {
                if (m_fallRampActive)
                {
                    m_fallRampActive = false;
                    RetuneSprings(); // back to the live trackingHertz
                }
                return;
            }
            m_fallRampActive = true;
            if (progress >= 1f)
            {
                RetuneSprings(fallHertzFloor); // fallen: near-free joints
                return;
            }
            float hz = math.max(math.lerp(m_fallRampFullHz,
                m_fallRampFullHz * fallRampMinPct, progress), fallHertzFloor);
            RetuneSprings(hz);
        }

        private int CountJoints()
        {
            int c = 0;
            for (int i = 0; i < m_nodes.Length; i++) if (m_nodes[i].hasJoint) c++;
            return c;
        }

        private int CountRefs()
        {
            int c = 0;
            for (int i = 0; i < m_nodes.Length; i++) if (m_nodes[i].isChainRef) c++;
            return c;
        }

        /// <summary>Depth-sort the node indices so parents always evaluate
        /// before children (twist propagates down the chains).</summary>
        private void BuildOrder()
        {
            int count = m_nodes.Length;
            m_order = new int[count];
            for (int i = 0; i < count; i++) m_order[i] = i;
            var depth = new int[count];
            for (int i = 0; i < count; i++)
            {
                // Walk up to the root; iteratively memoized on the way back.
                // Rig trees are shallow - fine at bind time.
                int j = i, d = 0;
                while (j >= 0 && depth[j] == 0) { j = m_nodes[j].parent; d++; }
                // j < 0: virtual null root; j >= 0: first labeled ancestor.
                // depth = ancestor depth + nodes walked past it (dropping the
                // ancestor depth collapsed every node to 1 and silently
                // degraded the sort to serialized order).
                int baseDepth = j >= 0 ? depth[j] : 0;
                j = i;
                while (j >= 0 && depth[j] == 0) { depth[j] = baseDepth + d--; j = m_nodes[j].parent; }
            }
            Array.Sort(m_order, (a, b) => depth[a].CompareTo(depth[b]));
        }

        private void Unbind()
        {
            m_nodes = Array.Empty<NodeState>();
            m_tips = Array.Empty<float3>();
            m_pivots = Array.Empty<float3>();
            m_rolls = Array.Empty<float3>();
            m_order = Array.Empty<int>();
            m_expPos = Array.Empty<float3>();
            m_expRot = Array.Empty<quaternion>();
            m_chainSim = Array.Empty<MotionTransform.ChainSimNode>();
            m_invCmdScratch = Array.Empty<quaternion>();
            m_tick = Array.Empty<float3>();
            m_edgeLen = Array.Empty<float>();
            m_restOffsetLocal = Array.Empty<float3>();
            m_bindPivots = Array.Empty<float3>();
            m_tipLocal = Array.Empty<float3>();
            m_baseDelta = quaternion.identity;
            m_bound = false;
            m_poseValid = false;
        }

        /// <summary>Re-resolve after endpoint or rig changes.</summary>
        [ContextMenu("Rebind")]
        private void Rebind()
        {
            Unbind();
            Bind();
        }

        // ------------------------------------------------------------------
        // Runtime

        private void FixedUpdate()
        {
            if (!m_bound || rig == null || !rig.IsBuilt || endpoints == null
                || m_nodes.Length == 0 || endpoints.Count != m_nodes.Length
                || m_tips.Length != m_nodes.Length || m_pivots.Length != m_nodes.Length
                || m_restOffsetLocal.Length != m_nodes.Length || m_expRot == null
                || m_expPos.Length != m_nodes.Length || m_expRot.Length != m_nodes.Length
                || m_chainSim.Length != m_nodes.Length
                || m_order.Length != m_nodes.Length)
            {
                if (m_bound) Unbind(); // stale serialized state
                return;
            }

            // 1. Points and segment directions.
            float dt = Time.fixedDeltaTime; // step time - also the FF lead time
            for (int i = 0; i < m_tips.Length; i++)
            {
                m_poseSrc = physicalPoseSource != null && physicalPoseSource.IsValid
                    ? physicalPoseSource : null;
                if (m_poseSrc != null)
                {
                    // Stream-side tap: extrapolated points for the upcoming step.
                    float sdt = Time.fixedDeltaTime;
                    m_tips[i] = m_poseSrc.GetTip(i, sdt);
                    m_pivots[i] = m_poseSrc.GetPivot(i, sdt);
                    m_rolls[i] = m_poseSrc.GetRoll(i, sdt);
                }
                else
                {
                    m_tips[i] = ResolvePoint(endpoints[i].tip);
                    m_pivots[i] = ResolvePoint(endpoints[i].pivot);
                    m_rolls[i] = ResolvePoint(endpoints[i].roll);
                }
                ref var n = ref m_nodes[i];
                float3 tip = m_tips[i];
                if (n.dirChild >= 0 && math.lengthsq(tip - m_pivots[i]) <= 1e-10f)
                    tip = m_pivots[n.dirChild]; // synthetic bone dir (covariant)
                n.curDirW = math.normalizesafe(tip - m_pivots[i], n.restDirW);
                n.obsConf = 0f;
            }

            // 1a. Leg splay bias: rigid per-leg rotation of the observed
            // directions about the hip, applied BEFORE retargeting. Zero
            // allocations, skipped entirely at 0 deg.
            ApplyLegSplay();

            // 1b. Continuous spring tuning: inspector edits apply live, every
            // step. Two native calls per joint; no allocations.
            if (retuneSprings) RetuneSprings();

            // 2-4. Retargeting: hinge observations -> chain twist blend ->
            // root-anchored setpoints. The whole composition is pure
            // (MotionTransform.RetargetChains) and mirrors the bind state
            // into a sim-node scratch array (no allocations).
            quaternion rootSpawnRot = quaternion.identity;
            for (int i = 0; i < m_nodes.Length; i++)
            {
                if (m_nodes[i].parent < 0 && m_nodes[i].body != null)
                {
                    rootSpawnRot = m_nodes[i].spawnChildRot;
                    quaternion measured = math.mul(m_nodes[i].body.Body.GetRotation(),
                        math.inverse(rootSpawnRot));
                    if (math.dot(measured.value, m_baseDelta.value) < 0f) measured.value = -measured.value;
                    float follow = 1f - math.exp(-baseFollowHertz * Time.fixedDeltaTime);
                    m_baseDelta = math.normalizesafe(math.slerp(m_baseDelta, measured, follow));
                    break;
                }
            }
            // Ghost FK reads m_baseDelta (filtered this step above).

            // Fill the sim-node mirror (bind state + step-1 observations).
            // All observations are root-relative INSIDE RetargetChains: each
            // node's direction is expressed in its parent segment's estimated
            // frame, so rigid rotation of the source or the rig cancels
            // exactly. Targets are pure joint-relative rotations (identity =
            // spawn); the rig root pose is a free DOF.
            for (int i = 0; i < m_nodes.Length; i++)
            {
                ref var n = ref m_nodes[i];
                ref var c = ref m_chainSim[i];
                c.parent = n.parent;
                c.chain = n.chain;
                c.s = n.s;
                c.nextRefIdx = n.nextRefIdx;
                c.nextRefS = n.nextRefS;
                c.baseRef = n.baseRef;
                c.isChainRef = n.isChainRef;
                c.hasJoint = n.hasJoint;
                c.isHinge = n.isHinge;
                c.restDirW = n.restDirW;
                c.curDirW = n.curDirW;
                c.restBendW = n.restBendW;
                c.restFrame = n.restFrame;
                if (n.isChainRef)
                {
                    // Roll direction: the chain refs' full-frame measurement.
                    // Splay-biased with the same rigid leg rotation so the
                    // measured frame matches the biased bone directions.
                    var roll = math.normalizesafe(m_rolls[i] - m_pivots[i],
                        MotionTransform.OrthoTo(n.curDirW));
                    var qb = LegRollBias(i);
                    c.curRollDirW = math.normalizesafe(math.mul(qb, roll));
                }
            }
            MotionTransform.RetargetChains(m_chainSim, m_order, ObsConfidenceMin,
                hingeTwistObservation, m_invCmdScratch);

            // Copy back + FK of the commanded rotations for the gizmo.
            for (int oi = 0; oi < m_order.Length; oi++)
            {
                int i = m_order[oi];
                ref var n = ref m_nodes[i];
                ref var c = ref m_chainSim[i];

                // FK ghost + roll tick are computed by ComputeGhostPose()
                // after the loop - presentation-only, skipped when the gizmo
                // is off.

                n.twist = c.twist;
                n.obsTwist = c.obsTwist;
                n.obsConf = c.obsConf;

                if (n.hasJoint)
                {
                    // relT is the retargeting pipeline's joint-relative
                    // rotation (identity = spawn) - same frame convention as
                    // the measured relA (verified against the joint's stored
                    // LocalFrameA/B).
                    quaternion relT = c.relT;

                    // Joint-space reference feedforward: the animated target's
                    // own velocity + acceleration in the joint frame, from
                    // consecutive per-step relT. Computed UNSCALED once and
                    // scaled per actuator mode (servo vs spring FF strength).
                    float3 wRefA = default, alphaRefA = default;
                    if ((servoFeedforward > 0f || springFeedforward > 0f) && m_wRefInit.Length > 0)
                    {
                        if (m_wRefInit[i])
                        {
                            wRefA = SplineChainController.RotationVector(
                                math.mul(relT, math.inverse(m_prevRelT[i]))) / dt;
                            alphaRefA = (wRefA - m_prevWRef[i]) / dt;
                        }
                        m_prevRelT[i] = relT;
                        m_prevWRef[i] = wRefA;
                        m_wRefInit[i] = true;
                    }

                    // Both writers can fire in the driveSprings debug combo:
                    // the spring target tracks relT (same as pure spring
                    // mode) AND the servo commands velocity - they act on
                    // the SAME pose target, so the compound stays
                    // consistent. If the spring target write were skipped
                    // here, the enabled spring would hold its stale bind
                    // target and fight the servo toward the T-pose.
                    if (springDriven || driveSprings)
                    {
                        // Spring-driven: the joint pose spring is the
                        // actuator - torque-uncapped, rock-stable (verified:
                        // relA holds within 1 deg of relT). Wake on large
                        // errors so sleeping chains still track a moving
                        // source.
                        if (n.isHinge)
                        {
                            var rj = new RevoluteJoint { Id = n.joint.Id };
                            float targetAngle = MotionTransform.TwistAngle(relT, math.forward());
                            // Spring target lead: advance the commanded angle by
                            // the reference's own motion (velocity + accel).
                            targetAngle += math.dot(
                                wRefA * (springFeedforward * dt)
                                + alphaRefA * (0.5f * springFeedforward * dt * dt), math.forward());
                            rj.SetTargetAngle(targetAngle);
                            if (math.abs(targetAngle) > 0.005f)
                                new Box3D.Joint { Id = n.joint.Id }.WakeBodies();
                        }
                        else
                        {
                            var sj = new SphericalJoint { Id = n.joint.Id };
                            // Spring target lead: relT advanced by ff*(wRef*dt
                            // + 0.5*alpha*dt^2) (small-angle quaternion).
                            float3 lead = (wRefA * dt + alphaRefA * (0.5f * dt * dt)) * springFeedforward;
                            quaternion leadRot = math.normalizesafe(
                                new quaternion(math.float4(lead * 0.5f, 1f)), quaternion.identity);
                            sj.SetTargetRotation(math.mul(relT, leadRot));
                            if (math.length(relT.value.xyz) > 0.005f)
                                new Box3D.Joint { Id = n.joint.Id }.WakeBodies();
                        }
                    }
                    if (!springDriven)
                    {
                        // Velocity-motor servo. relA is measured with the
                        // same builder-parity frames the joint stores.
                        quaternion relA;
                        MotionTransform.JointRelative(n.parentBody.Body.GetRotation(), n.body.Body.GetRotation(),
                            n.frameARot, n.frameBRot, out relA);
                        float3 rv = SplineChainController.RotationVector(MotionTransform.ServoError(relT, relA));
                        float thetaErr = math.length(rv);

                        quaternion jointAW = math.mul(n.parentBody.Body.GetRotation(), n.frameARot);
                        float3 wRelWorld = n.body.Body.GetAngularVelocity()
                                         - n.parentBody.Body.GetAngularVelocity();
                        float3 wRelA = math.mul(math.inverse(jointAW), wRelWorld);

                        float3 dirA = thetaErr > 1e-6f ? rv / thetaErr : float3.zero;
                        float3 axisW = math.normalizesafe(math.mul(jointAW, dirA), math.forward(jointAW));
                        float3 anchorW = (float3)n.parentBody.Body.GetPosition()
                            + math.mul(n.parentBody.Body.GetRotation(), (float3)n.frameAPos);
                        float iEff = SplineChainController.CompositeInertiaAboutAxis(n.distal, axisW, anchorW);
                        float iSelf = SplineChainController.BodyInertiaAboutAxis(n.body.Body, axisW);
                        float3 w = servoFeedforward > 0f
                            ? MotionTransform.ServoCommandFF(
                                rv, wRelA, servoGain, maxCommandSpeed, motorDamping,
                                n.torqueCap, iEff, iSelf,
                                wRefA * servoFeedforward,
                                alphaRefA * (servoFeedforward * servoFeedforwardAccel), dt)
                            : MotionTransform.ServoCommand(
                                rv, wRelA, servoGain, maxCommandSpeed, motorDamping,
                                n.torqueCap, iEff, iSelf);

                        // Gravity load feedforward: the distal weight's angular
                        // acceleration about the command axis, as a per-step
                        // velocity lead. Added AFTER the caps: it cancels the
                        // standing load the P servo otherwise needs error to
                        // fight (the slow settle from a standing deviation).
                        // Scratch grown lazily once - distal lists are bind
                        // constants; no per-step allocation.
                        if (n.distal.Length > 0)
                        {
                            if (m_gCom.Length < n.distal.Length)
                            {
                                m_gCom = new float3[n.distal.Length];
                                m_gMass = new float[n.distal.Length];
                            }
                            for (int d = 0; d < n.distal.Length; d++)
                            {
                                Body b = n.distal[d];
                                m_gCom[d] = (float3)b.GetWorldCenterOfMass();
                                m_gMass[d] = b.GetMass();
                            }
                            float alphaG = MotionTransform.GravityFeedforwardAccel(
                                m_gCom.AsSpan(0, n.distal.Length),
                                m_gMass.AsSpan(0, n.distal.Length),
                                anchorW, axisW, iEff);
                            w += dirA * (alphaG * dt * gravityFeedforward);
                        }

                        if (n.isHinge)
                        {
                            // Scalar speed about frame Z (the hinge axis).
                            var rj = new RevoluteJoint { Id = n.joint.Id };
                            float speed = math.dot(w, math.forward());
                            rj.SetMotorSpeed(speed);
                            if (math.abs(speed) > 0.05f) new Box3D.Joint { Id = n.joint.Id }.WakeBodies();
                        }
                        else
                        {
                            float3 wWorld = math.mul(jointAW, w);
                            var sj = new SphericalJoint { Id = n.joint.Id };
                            sj.SetMotorVelocity(wWorld);
                            if (math.length(wWorld) > 0.05f) new Box3D.Joint { Id = n.joint.Id }.WakeBodies();
                        }
                    }
                }

            }
            // Ghost pose (gizmos + telemetry) - presentation-only. Skip in the
            // hot loop when nothing draws it; telemetry computes on demand.
            if (drawExpectedPose) ComputeGhostPose();
            else m_poseValid = false;
        }

        /// <summary>Ghost FK from the last commanded relT: expected world
        /// joint positions/rotations plus the roll tick. Presentation-only;
        /// never feeds the articulation (joints are commanded from c.relT
        /// directly). Uses m_baseDelta, which the tick loop already filtered
        /// this step - same value the inline pass used.</summary>
        private void ComputeGhostPose()
        {
            if (m_nodes == null || m_expPos.Length != m_nodes.Length) return;
            // Root spawn rotation (same derivation as the tick loop's
            // rootSpawnRot local).
            quaternion rootSpawnRot = quaternion.identity;
            for (int i = 0; i < m_nodes.Length; i++)
                if (m_nodes[i].parent < 0) { rootSpawnRot = m_nodes[i].spawnChildRot; break; }
            for (int oi = 0; oi < m_order.Length; oi++)
            {
                int i = m_order[oi];
                ref var n = ref m_nodes[i];
                ref var c = ref m_chainSim[i];

                // Expected joint position: parent's expected position plus
                // the bind offset rotated by the parent's expected rotation
                // (carries lateral attachment offsets correctly).
                if (n.parent < 0) m_expPos[i] = m_pivots[i];
                else m_expPos[i] = m_expPos[n.parent]
                    + math.mul(m_expRot[n.parent], m_restOffsetLocal[i]);

                if (!n.hasJoint)
                {
                    // Root: no joint. Ghost root = the root body's WORLD
                    // rotation (filtered spawn-relative reaction x spawn).
                    m_expRot[i] = math.mul(m_baseDelta, rootSpawnRot);
                }
                else
                {
                    // Ghost FK from relT (rig-relative). From relA parity:
                    //   childRot = parentRot * fA * relT * inv(fB)
                    // fA*inv(fB) = inv(sp)*sc carries the spawn differential,
                    // so at rest (relT = identity) the ghost is exactly the
                    // rig's spawn pose carried on the filtered base reaction.
                    m_expRot[i] = math.mul(m_expRot[n.parent],
                        math.mul(n.frameARot, math.mul(c.relT, math.inverse(n.frameBRot))));
                }

                // Roll tick: where the rest roll reference would sit under
                // the expected rotation (chain refs have a datum).
                m_tick[i] = n.isChainRef && n.body != null
                    ? math.mul(m_expRot[i], math.mul(math.inverse(n.spawnChildRot),
                          math.mul(n.restFrame, math.float3(0, 1, 0)))) * 0.15f
                    : float3.zero;
            }
            m_poseValid = true;
        }

        private float3 ResolvePoint(PointRef p)
        {
            Transform t = p.target != null ? p.target : transform;
            return (float3)t.TransformPoint(p.offset);
        }

        // --- math helpers (delegating to the tested MotionTransform) -------

        private static quaternion FrameFromPrimaryRoll(float3 primary, float3 rollRaw)
            => MotionTransform.FrameFromPrimaryRoll(primary, rollRaw);

        private static float TwistAngle(quaternion rot, float3 axis)
            => MotionTransform.TwistAngle(rot, axis);

        private static float SignedAngle(float3 a, float3 b, float3 axis)
            => MotionTransform.SignedAngle(a, b, axis);

        private static float WrapHalf(float ang) => MotionTransform.WrapHalf(ang);

        private static quaternion ShortestArc(float3 from, float3 to)
            => MotionTransform.ShortestArc(from, to);

        private static float3 OrthoTo(float3 v) => MotionTransform.OrthoTo(v);

        // ------------------------------------------------------------------
        // Edit-time mapping scaffolding (read-only wrt the rig builder).

        /// <summary>Creates one endpoint per rig node, pivot at the authored
        /// joint anchor (attachW), tip at the authored distal tip. Uses the
        /// serialized config only (GetAuthoredFrames) - building the rig is
        /// NOT required. Nodes whose name looks like a chain anchor
        /// (pelvis/torso/chest/hips) are pre-marked as chain references.</summary>
        public void InitializeMappingsFromRig()
        {
            if (rig == null) return;
            var frames = new Dictionary<string, Box3DRigBuilder.AuthoredFrame>();
            rig.GetAuthoredFrames(frames);
            endpoints.Clear();
            Quaternion driverRotInv = Quaternion.Inverse(transform.rotation);
            // Iterate in node (config) order - dictionary order is unstable.
            foreach (var n in rig.Nodes)
            {
                if (!frames.TryGetValue(n.name, out var f)) continue;
                string lower = n.name.ToLowerInvariant();
                bool looksLikeRef = lower.Contains("pelvis") || lower.Contains("torso")
                                 || lower.Contains("chest") || lower.Contains("hips");
                endpoints.Add(new Endpoint
                {
                    nodeName = n.name,
                    chainReference = looksLikeRef,
                    tip = new PointRef { offset = driverRotInv * ((Vector3)f.Tip - transform.position) },
                    pivot = new PointRef { offset = driverRotInv * ((Vector3)f.Anchor - transform.position) },
                });
            }
        }

        /// <summary>Endpoint gizmos: pivot->tip bone lines (cyan = hinge,
        /// orange = spherical; green = chain reference) with a red dot on
        /// each pivot and a green roll vector for references. Drawn via
        /// HumanoidMotionDriverEditor when selected. Works from the rig
        /// config, built or not.</summary>
        public void DrawEndpointsGizmo()
        {
            if (rig == null || endpoints.Count == 0) return;

            for (int i = 0; i < endpoints.Count; i++)
            {
                var e = endpoints[i];
                var node = rig.FindNode(e.nodeName);
                if (node == null) continue;
                // Joint kind is config data - draw even when not built.
                bool isHinge = node.joint == Box3DRigBuilder.JointKind.Hinge;

                Vector3 pivot = ResolvePoint(e.pivot);
                Vector3 tip = ResolvePoint(e.tip);
                Gizmos.color = e.chainReference ? new Color(0.3f, 1f, 0.4f)
                             : isHinge ? new Color(0.2f, 0.9f, 1f)
                             : new Color(1f, 0.75f, 0.2f);
                Gizmos.DrawLine(pivot, tip);
                Gizmos.DrawWireSphere(tip, 0.03f);
                Gizmos.color = new Color(1f, 0.2f, 0.2f, 0.6f);
                Gizmos.DrawSphere(pivot, 0.02f);
                if (e.chainReference)
                {
                    Gizmos.color = new Color(0.3f, 1f, 0.4f, 0.7f);
                    Gizmos.DrawLine(pivot, ResolvePoint(e.roll));
                }
            }
        }

        /// <summary>Expected articulated skeleton: FK of the joint targets
        // commanded in FixedUpdate - white bones between expected joint
        // positions, yellow joint dots, magenta roll ticks on chain
        // references. Only meaningful while playing (targets exist then).</summary>
        public void DrawExpectedPoseGizmo()
        {
            if (!drawExpectedPose || !m_poseValid || !m_bound) return;
            if (m_tipLocal == null || m_tipLocal.Length != m_nodes.Length) return; // stale bind data

            for (int i = 0; i < m_nodes.Length; i++)
            {
                ref var n = ref m_nodes[i];
                if (n.parent < 0 && !n.hasJoint && !n.isChainRef) continue;

                if (n.parent >= 0)
                {
                    Gizmos.color = new Color(0.95f, 0.95f, 0.95f, 0.9f);
                    Gizmos.DrawLine((Vector3)m_expPos[n.parent] + expectedPoseOffset,
                                    (Vector3)m_expPos[i] + expectedPoseOffset);
                }
                // True joint->tip bone, matching the authored endpoint gizmo:
                // bind tip offset carried by the node's expected rotation.
                if (n.hasJoint || n.parent < 0)
                {
                    Gizmos.DrawLine((Vector3)m_expPos[i] + expectedPoseOffset,
                        (Vector3)(m_expPos[i] + math.mul(m_expRot[i], m_tipLocal[i])) + expectedPoseOffset);
                }
                Gizmos.color = new Color(1f, 0.9f, 0.2f, 0.8f);
                Gizmos.DrawSphere((Vector3)m_expPos[i] + expectedPoseOffset, 0.015f);

                if (math.lengthsq(m_tick[i]) > 1e-10f)
                {
                    Gizmos.color = new Color(1f, 0.2f, 1f, 0.9f);
                    Gizmos.DrawLine((Vector3)m_expPos[i] + expectedPoseOffset,
                                    (Vector3)(m_expPos[i] + m_tick[i]) + expectedPoseOffset);
                }
            }
        }

        /// <summary>Dim gray skeleton of the pivots as captured at BIND time.
        /// Overlays the source endpoints iff bind ran in the current pose.</summary>
        public void DrawBindPoseGizmo()
        {
            if (!drawExpectedPose || !m_bound || m_bindPivots.Length == 0) return;
            Gizmos.color = new Color(0.6f, 0.6f, 0.6f, 0.5f);
            for (int i = 0; i < m_nodes.Length; i++)
            {
                if (m_nodes[i].parent < 0) continue;
                Gizmos.DrawLine((Vector3)m_bindPivots[m_nodes[i].parent] + expectedPoseOffset,
                                (Vector3)m_bindPivots[i] + expectedPoseOffset);
                Gizmos.DrawSphere((Vector3)m_bindPivots[i] + expectedPoseOffset, 0.012f);
            }
        }

        /// <summary>Per-node bind consistency report: rest-vs-current point
        /// drift (source moved since bind?), rest-vs-authored divergence
        /// (points placed off the body frames?), live twist, and which
        /// transform each tip actually reads.</summary>
        [ContextMenu("Dump Bind Diagnostics")]
        private void DumpDiagnostics()
        {
            if (!m_bound) { Debug.LogWarning("[HumanoidMotionDriver] not bound", this); return; }
            // One full consistency check instead of per-line null guards:
            // stale serialized bind state (m_bound persisted in the scene) or
            // a changed endpoint list force a clean unbind and say so.
            bool valid = rig != null && endpoints != null && m_nodes != null && m_nodes.Length > 0
                && endpoints.Count == m_nodes.Length
                && m_tips != null && m_tips.Length == m_nodes.Length
                && m_pivots != null && m_pivots.Length == m_nodes.Length
                && m_bindPivots != null && m_bindPivots.Length == m_nodes.Length
                && m_tipLocal != null && m_tipLocal.Length == m_nodes.Length
                && m_expPos != null && m_expRot != null && m_tick != null
                && m_expPos.Length == m_nodes.Length && m_expRot.Length == m_nodes.Length
                && m_tick.Length == m_nodes.Length;
            if (!valid)
            {
                Debug.LogWarning("[HumanoidMotionDriver] inconsistent bind state "
                    + $"(nodes={m_nodes?.Length ?? -1}, endpoints={endpoints?.Count ?? -1}, rig={(rig != null ? "ok" : "null")}) - unbinding; Rebind to fix", this);
                Unbind();
                return;
            }
            ComputeGhostPose(); // dump reads the ghost pose; recompute on demand
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[HumanoidMotionDriver] bind diagnostics on '{rig.name}':");
            sb.AppendLine($"  bind: nodes={m_nodes.Length} endpoints={endpoints.Count} "
                + $"tips={m_tips.Length} pivots={m_pivots.Length} bindPivots={m_bindPivots.Length} tipLocal={m_tipLocal.Length}");
            for (int i = 0; i < m_nodes.Length; i++)
            {
                ref var n = ref m_nodes[i];
                if (i >= endpoints.Count || i >= m_tips.Length || i >= m_pivots.Length
                    || i >= m_bindPivots.Length || i >= m_tipLocal.Length)
                {
                    sb.AppendLine($"  [{i}] BUFFER MISMATCH - node '{(i < endpoints.Count ? endpoints[i].nodeName : "?")}' has no bind data; endpoint list changed after bind");
                    continue;
                }
                var e = endpoints[i];
                var node = rig.FindNode(e.nodeName);
                float restVsCur = 0f, restVsAuth = 0f;
                if (node != null && math.lengthsq(node.dirW) > 1e-8f)
                {
                    float d = math.clamp(math.dot(math.normalizesafe(n.restDirW),
                        math.normalizesafe((float3)node.dirW)), -1f, 1f);
                    restVsAuth = math.degrees(math.acos(d));
                }
                if (math.lengthsq(n.restDirW) > 1e-8f && math.lengthsq(n.curDirW) > 1e-8f)
                {
                    float d = math.clamp(math.dot(math.normalizesafe(n.restDirW),
                        math.normalizesafe(n.curDirW)), -1f, 1f);
                    restVsCur = math.degrees(math.acos(d));
                }
                string tgt = e.tip.target != null ? e.tip.target.name : "<driver>";
                string ptgt = e.pivot.target != null ? e.pivot.target.name : "<driver>";
                float pivotDrift = math.length(m_pivots[i] - m_bindPivots[i]);
                float segLenCur = math.length(m_tips[i] - m_pivots[i]);
                float segLenBind = math.length(m_tipLocal[i]);
                float posErr = n.parent >= 0 || n.hasJoint || n.isChainRef
                    ? math.length(m_expPos[i] - m_pivots[i]) : 0f;
                // How far the commanded rotation is from the spawn rotation.
                float cmdDelta = 0f;
                if (n.hasJoint || n.parent < 0)
                {
                    quaternion d = math.mul(m_expRot[i], math.inverse(n.spawnChildRot));
                    float dn = math.length(d.value);
                    if (dn > 1e-6f)
                    {
                        float ang = 2f * math.acos(math.clamp(math.abs(d.value.w) / dn, -1f, 1f));
                        cmdDelta = math.degrees(ang);
                    }
                }
                sb.AppendLine($"  {e.nodeName}: parent={(n.parent >= 0 ? endpoints[n.parent].nodeName : "<none>")} "
                    + $"restVsCur={restVsCur:F1} restVsAuthored={restVsAuth:F1} "
                    + $"twist={math.degrees(n.twist):F1} obsTwist={math.degrees(n.obsTwist):F1} conf={n.obsConf:F2} cmdDelta={cmdDelta:F1} "
                    + $"pivotDrift={pivotDrift:F3} posErr={posErr:F3} "
                    + $"segLen cur={segLenCur:F3} bind={segLenBind:F3} "
                    + $"tip='{tgt}' pivot='{ptgt}' "
                    + $"hinge={n.isHinge} ref={n.isChainRef}");
            }
            Debug.Log(sb.ToString(), this);
        }

        /// <summary>Live motor diagnostics (play mode, rig bound): per joint,
        /// remaining pose error and applied-vs-max motor torque. Saturated
        /// joints (torque == max) mean the cap is too low for gravity load;
        /// growing error with low torque means a control problem.</summary>
        [ContextMenu("Dump Motor State")]
        private void DumpMotorState()
        {
            if (!m_bound || m_nodes.Length == 0) { Debug.LogWarning("[HumanoidMotionDriver] not bound", this); return; }
            var sb = new System.Text.StringBuilder();
            sb.AppendLine($"[HumanoidMotionDriver] motor state on '{rig.name}':");
            for (int oi = 0; oi < m_order.Length; oi++)
            {
                int i = m_order[oi];
                ref var n = ref m_nodes[i];
                if (!n.hasJoint || n.parent < 0 || n.body == null || n.parentBody?.Body == null) continue;
                float torque;
                if (n.isHinge)
                {
                    var j = new RevoluteJoint { Id = n.joint.Id };
                    torque = math.abs(j.GetMotorTorque());
                    // Commanded setpoint vs measured, both through the same
                    // frame pipeline as the servo: non-zero at rest = frame
                    // mismatch, zero but not tracking = gain/torque issue.
                    quaternion relT = m_chainSim[i].relT;
                    quaternion relA = math.mul(
                        math.inverse(math.mul(n.parentBody.Body.GetRotation(), n.frameARot)),
                        math.mul(n.body.Body.GetRotation(), n.frameBRot));
                    float3 rv = SplineChainController.RotationVector(math.mul(relT, math.inverse(relA)));
                    sb.AppendLine($"  {endpoints[i].nodeName}: hinge angle={math.degrees(j.GetAngle()):F1} motorSpeed={j.GetMotorSpeed():F2} "
                        + $"errVsCmd={math.degrees(math.length(rv)):F1}deg torque={torque:F1}/{n.torqueCap:F1}");
                }
                else
                {
                    var j = new SphericalJoint { Id = n.joint.Id };
                    float3 t = j.GetMotorTorque();
                    torque = math.length(t);
                    // Same pipeline as the servo: relT (setpoint) vs relA
                    // (measured) in joint space. errVsCmd ~0 at rest and
                    // shrinking during tracking; constant non-zero at rest
                    // = frame mismatch, growing = instability.
                    quaternion relT = m_chainSim[i].relT;
                    quaternion relA;
                    MotionTransform.JointRelative(n.parentBody.Body.GetRotation(), n.body.Body.GetRotation(),
                        n.frameARot, n.frameBRot, out relA);
                    float3 rv = SplineChainController.RotationVector(MotionTransform.ServoError(relT, relA));
                    sb.AppendLine($"  {endpoints[i].nodeName}: errVsCmd={math.degrees(math.length(rv)):F1}deg "
                        + $"torque={torque:F1}/{n.torqueCap:F1} dir={t / math.max(torque, 1e-6f)}");
                }
            }
            Debug.Log(sb.ToString(), this);
        }
    }
}