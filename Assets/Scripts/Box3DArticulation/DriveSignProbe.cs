using Unity.Mathematics;
using UnityEngine;
using Box3D.Hybrid;
using Box3D;

using Joint = Box3D.Joint;

namespace Box3DArticulation
{
    /// <summary>
    /// Limb chain + target-pose reaching.
    ///
    ///   Root (static) — shoulder (spherical) — UpperArm — elbow (hinge) — Forearm
    ///
    /// Target pose comes from control points (rig-style, no spline math):
    ///   cp0 = shoulder anchor, cp1 = target elbow, cp2 = target hand tip.
    /// Each bone's target direction = direction from its start point to its end
    /// point; the joint reaches for it with the minimal drive logic:
    ///   qRel error -> RotVec/dt -> SetMotorVelocity (world for spherical,
    ///   scalar about frame Z for hinge). No gains, no damping, no clamping.
    /// </summary>
    public class DriveSignProbe : MonoBehaviour
    {
        [Header("Bones")]
        [SerializeField] private float boneLength = 1f;
        [SerializeField] private float boneThickness = 0.2f;

        [Header("Layout")]
        [SerializeField] private float rootHeight = 2f;

        [Header("Target pose (control points)")]
        [SerializeField] private Vector3 elbowTarget = new Vector3(0.7f, 1.3f, 0f); // root-relative-ish
        [SerializeField] private Vector3 handTarget = new Vector3(1.7f, 1.3f, 0f);

        [Header("Drive")]
        [SerializeField] private float torqueLimit = 500f;
        [SerializeField] private bool driveEnabled = true;
        [SerializeField, Range(0.1f, 2f)] private float kp = 1f;   // error -> velocity gain
        [SerializeField, Range(0f, 1f)] private float kd = 0.2f;   // damping on measured spin
        [SerializeField, Range(0.5f, 2f)] private float zeta = 1f; // damping scale vs inertia ratio

        [Header("Joint springs (only stability, not pose tracking)")]
        [SerializeField] private float shoulderHertz = 200f;
        [SerializeField] private float shoulderDamping = 1f;
        [SerializeField] private float elbowHertz = 200f;
        [SerializeField] private float elbowDamping = 1f;

        private class JointRef
        {
            public JointId id;
            public bool isHinge;
            public B3Transform frameA, frameB;
            public Body[] distal; // bodies beyond this joint — the inertia it fights
        }

        private Box3DBody m_root, m_upper, m_fore;
        private JointRef m_shoulder, m_elbow;
        private float3[] m_segDirs, m_hingeAxes; // reused per frame (no allocs)
        private float3 m_halfExtents;
        private Transform m_cp0, m_cp1, m_cp2;

        private static readonly float3 kLocalBone = new float3(0f, -1f, 0f); // bone axis in body space

        private System.IO.StreamWriter m_log;

        private void Start()
        {
            m_log = new System.IO.StreamWriter("Logs/chain-probe.log", append: false) { AutoFlush = true };
            Log($"[CHAIN] scene={gameObject.scene.name} world={Box3DWorld.Instance.World.IsValid}");

            float halfBone = boneLength / 2f;
            float halfThick = boneThickness / 2f;
            m_halfExtents = new float3(halfThick, halfBone, halfThick);

            float shoulderY = rootHeight - halfThick;

            var root = MakeBone("Root", new Vector3(0f, rootHeight, 0f),
                new Vector3(boneThickness, boneThickness, boneThickness), Box3DBodyType.Static);
            var upper = MakeBone("UpperArm", new Vector3(0f, shoulderY - halfBone, 0f),
                new Vector3(boneThickness, boneLength, boneThickness), Box3DBodyType.Dynamic);
            var fore = MakeBone("Forearm", new Vector3(0f, shoulderY - boneLength - halfBone, 0f),
                new Vector3(boneThickness, boneLength, boneThickness), Box3DBodyType.Dynamic);

            // Owner-local anchors: root owns the shoulder, upper arm owns the elbow.
            var shoulderWorld = BodyPoint(root.Body, new float3(0f, -halfThick, 0f));
            var elbowWorld = BodyPoint(upper.Body, new float3(0f, -halfBone, 0f));
            m_shoulder = CreateSpherical(root, upper, shoulderWorld, shoulderHertz, shoulderDamping);
            m_elbow = CreateHinge(upper, fore, elbowWorld, new float3(0f, 0f, 1f), elbowHertz, elbowDamping);
            m_root = root; m_upper = upper; m_fore = fore;
            m_shoulder.distal = new[] { upper.Body, fore.Body }; // whole limb
            m_elbow.distal = new[] { fore.Body };                // forearm only

            // Target pose control points (rig-style rail markers).
            m_cp0 = MakeControlPoint("CP0_Shoulder", shoulderWorld);
            m_cp1 = MakeControlPoint("CP1_Elbow", elbowTarget + Vector3.up * 0f);
            m_cp2 = MakeControlPoint("CP2_Hand", handTarget);

            var counters = Box3DWorld.Instance.World.GetCounters();
            Log($"[CHAIN] bodies={counters.BodyCount} shapes={counters.ShapeCount} joints={counters.JointCount} " +
                $"cp1={elbowTarget} cp2={handTarget}");
        }

        private void FixedUpdate()
        {
            if (!driveEnabled || m_upper == null) return;
            float dt = Time.fixedDeltaTime;

            // Target direction per bone from the control points.
            float3 cp0 = (float3)m_cp0.position;
            float3 cp1 = (float3)m_cp1.position;
            float3 cp2 = (float3)m_cp2.position;
            float3 dirUpper = math.normalizesafe(cp1 - cp0, kLocalBone);
            float3 dirFore = math.normalizesafe(cp2 - cp1, kLocalBone);

            // --- Datum derivation (rig logic, generalized) ---
            // Segment directions from the control-point polyline; per joint, the
            // hinge axis the pose demands = bend-plane normal between consecutive
            // segments. Collinear segments have no normal -> parallel-transport
            // the previous datum along the segment (pose-derived twist everywhere,
            // never externally pinned). Scratch arrays, no per-frame allocs.
            if (m_segDirs == null)
            {
                m_segDirs = new float3[2];
                m_hingeAxes = new float3[2];
            }
            m_segDirs[0] = dirUpper;
            m_segDirs[1] = dirFore;

            float3 hPrev = float3.zero;
            for (int j = 0; j < m_segDirs.Length; j++)
            {
                float3 h;
                if (j + 1 < m_segDirs.Length)
                {
                    float3 c = math.cross(m_segDirs[j], m_segDirs[j + 1]);
                    h = math.lengthsq(c) > 1e-6f
                        ? math.normalizesafe(c)
                        : TransportDatum(hPrev, m_segDirs[math.max(j - 1, 0)], m_segDirs[j]);
                }
                else
                {
                    h = TransportDatum(hPrev, m_segDirs[math.max(j - 1, 0)], m_segDirs[j]);
                }
                m_hingeAxes[j] = h;
                hPrev = h;
            }

            // --- Per joint: bone alignment + twist correction (datum by joint type) ---
            // Spherical before a hinge: datum = next joint's hinge axis, aligned to
            // the bend normal at this joint. Hinge: datum = its own frameB Z,
            // aligned to the bend normal at the previous joint.
            float3 elbowHingeLocal = math.mul(m_elbow.frameB.Rotation, math.forward());

            quaternion qUpper = ShortestArc(kLocalBone, m_segDirs[0]);
            qUpper = TwistCorrect(qUpper, m_segDirs[0], elbowHingeLocal, m_hingeAxes[0]);

            quaternion qFore = ShortestArc(kLocalBone, m_segDirs[1]);
            qFore = TwistCorrect(qFore, m_segDirs[1], elbowHingeLocal, m_hingeAxes[0]);

            // Hierarchical datum: elbow drives against the upper arm's TARGET pose;
            // shoulder drives against the root's actual rotation (root owns its pose).
            DriveTo(m_shoulder, m_root.Body, m_upper.Body, m_root.Body.GetRotation(), qUpper, dt);
            DriveTo(m_elbow, m_upper.Body, m_fore.Body, qUpper, qFore, dt);
        }

        // The essential drive: relative-target error -> rotation vector -> motor
        // velocity, plus a kd damping term on the MEASURED relative angular
        // velocity. Without kd the P-only velocity servo releases at the target
        // (command -> 0) and gravity restarts the swing: reach, relax, re-engage.
        private void DriveTo(JointRef j, Body parent, Body child,
            quaternion parentTarget, quaternion childTarget, float dt)
        {
            quaternion qRelT = SplineChainController.RelativeTarget(
                parentTarget, childTarget, j.frameA, j.frameB);
            quaternion qRelA = SplineChainController.RelativeTarget(
                parent.GetRotation(), child.GetRotation(), j.frameA, j.frameB);
            quaternion e = math.mul(qRelT, math.inverse(qRelA));
            float3 rv = SplineChainController.RotationVector(e);
            float theta = math.length(rv);

            // Joint anchor in world (parent pose + frameA offset) for inertia queries.
            float3 anchorW = (float3)parent.GetPosition()
                + math.mul(parent.GetRotation(), (float3)j.frameA.Position);

            // Measured relative angular velocity (child - parent), world -> joint-A space.
            float3 wRelWorld = child.GetAngularVelocity() - parent.GetAngularVelocity();
            float3 wRelA = math.mul(math.inverse(math.mul(parent.GetRotation(), j.frameA.Rotation)), wRelWorld);

            // --- Inertia-aware command shaping ---
            // The joint fights its whole distal subtree fused into one rigid body.
            // I_eff (about the error axis through the anchor) + torqueLimit give the
            // achievable angular decel; the stopping-velocity profile sqrt(2*a*theta)
            // makes the arrival critically damped instead of a full-speed lunge that
            // the motor cannot cancel in time (the post-fast-move wobble).
            float3 axisW = (theta > 1e-6f)
                ? math.normalizesafe(math.mul(parent.GetRotation(), math.mul(j.frameA.Rotation, rv / theta)))
                : math.mul(parent.GetRotation(), math.mul(j.frameA.Rotation, math.forward()));
            float iEff = CompositeInertiaAboutAxis(j.distal, axisW, anchorW);
            float iSelf = BodyInertiaAboutAxis(child, axisW); // child alone = reference
            float alphaMax = torqueLimit / math.max(iEff, 1e-4f);
            float wStop = math.sqrt(2f * alphaMax * theta);
            float mag = math.min(kp * theta / math.max(dt, 1e-5f), wStop);
            float3 dir = (theta > 1e-6f) ? rv / theta : float3.zero;

            // Damping scaled by how much heavier the distal load is than the child
            // alone: extended limb -> stronger brake, folded -> unchanged elbow feel.
            float kdHere = kd * math.sqrt(iEff / math.max(iSelf, 1e-4f)) * zeta;
            float3 w = dir * mag - kdHere * wRelA;

            if (j.isHinge)
            {
                // Hinge: scalar speed about frame Z (the hinge axis), direct sign.
                var rev = new RevoluteJoint { Id = j.id };
                // A settled body sleeps and ignores new motor commands — wake it
                // whenever the command is non-trivial (target changed / error grew).
                if (math.length(w) > 0.05f) new Joint { Id = j.id }.WakeBodies();
                rev.EnableMotor(true);
                rev.SetMaxMotorTorque(torqueLimit);
                rev.SetMotorSpeed(math.dot(w, math.forward()));
            }
            else
            {
                // Spherical: velocity in world frame (measured convention), direct sign.
                float3 wWorld = math.mul(parent.GetRotation(), math.mul(j.frameA.Rotation, w));
                var sph = new SphericalJoint { Id = j.id };
                if (math.length(w) > 0.05f) new Joint { Id = j.id }.WakeBodies();
                sph.EnableMotor(true);
                sph.SetMaxMotorTorque(torqueLimit);
                sph.SetMotorVelocity(wWorld);
            }
        }

        // Composite rigid-body inertia of the distal subtree about a world axis
        // through the joint anchor: sum of rotated local inertia tensors plus
        // parallel-axis terms (m*(|r|^2 E - r r^T), r = com - anchor).
        private static float CompositeInertiaAboutAxis(
            System.Collections.Generic.IReadOnlyList<Body> bodies, float3 axisW, float3 anchor)
        {
            float3x3 I = float3x3.zero;
            for (int i = 0; i < bodies.Count; i++)
            {
                Body b = bodies[i];
                if (!b.IsValid) continue;
                float3x3 R = new float3x3(b.GetRotation());
                B3Matrix3 li = b.GetLocalRotationalInertia();
                float3x3 Il = new float3x3(li.Cx, li.Cy, li.Cz);
                I += math.mul(math.mul(R, Il), math.transpose(R));
                float m = b.GetMass();
                float3 r = (float3)b.GetWorldCenterOfMass() - anchor;
                I += m * (math.dot(r, r) * float3x3.identity - new float3x3(r * r.x, r * r.y, r * r.z));
            }
            return math.max(math.dot(axisW, math.mul(I, axisW)), 1e-4f);
        }

        // Single body's rotational inertia about a world axis (through its COM).
        private static float BodyInertiaAboutAxis(Body b, float3 axisW)
        {
            if (!b.IsValid) return 1e-4f;
            float3x3 R = new float3x3(b.GetRotation());
            B3Matrix3 li = b.GetLocalRotationalInertia();
            float3x3 Iw = math.mul(math.mul(R, new float3x3(li.Cx, li.Cy, li.Cz)), math.transpose(R));
            return math.max(math.dot(axisW, math.mul(Iw, axisW)), 1e-4f);
        }

        // Parallel transport: carry a plane normal by the shortest arc that takes
        // 'from' onto 'to' — used where segments are collinear and the bend
        // normal is degenerate, so twist flows continuously through straight runs.
        private static float3 TransportDatum(float3 datum, float3 from, float3 to)
        {
            if (math.lengthsq(datum) < 1e-6f || math.lengthsq(from) < 1e-6f) return datum;
            return math.mul(ShortestArc(from, to), datum);
        }

        // Twist correction (rig step 2): after bone alignment, rotate the target
        // about the bone axis so the hinge axis (hinge-frame Z in body-local
        // space) lands on the pose-demanded bend normal. Both vectors are
        // projected off the bone axis first, so this is a pure roll.
        private static quaternion TwistCorrect(quaternion qTarget, float3 boneDir,
            float3 hingeAxisLocal, float3 bendNormal)
        {
            float3 axisW = math.mul(qTarget, hingeAxisLocal);
            float3 axisP = axisW - boneDir * math.dot(axisW, boneDir);
            float3 hP = bendNormal - boneDir * math.dot(bendNormal, boneDir);
            if (math.lengthsq(axisP) < 1e-6f || math.lengthsq(hP) < 1e-6f)
                return qTarget;
            return math.mul(ShortestArc(math.normalizesafe(axisP), math.normalizesafe(hP)), qTarget);
        }

        // ---- setup helpers (official HybridSpawner pattern) ----

        private Box3DBody MakeBone(string name, Vector3 position, Vector3 scale, Box3DBodyType type)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.SetActive(false);
            Object.Destroy(go.GetComponent<Collider>());
            go.name = name;
            go.transform.position = position;
            go.transform.localScale = scale;
            go.AddComponent<Box3DBoxShape>();
            var body = go.AddComponent<Box3DBody>();
            body.BodyType = type;
            go.SetActive(true);
            return body;
        }

        private Transform MakeControlPoint(string name, Vector3 position)
        {
            var go = new GameObject(name);
            go.transform.position = position;
            return go.transform;
        }

        private JointRef CreateSpherical(Box3DBody a, Box3DBody b, float3 anchor, float hertz, float damping)
        {
            quaternion frameRot = SplineChainController.FrameFromAxisWorld(new float3(0f, -1f, 0f));
            var frames = LocalFrames(a.Body, b.Body, new B3Transform { Position = anchor, Rotation = frameRot });
            var def = SphericalJointDef.Default;
            def.Hertz = hertz; def.DampingRatio = damping;
            def.Base.BodyIdA = a.Body.Id; def.Base.BodyIdB = b.Body.Id; def.Base.CollideConnected = false;
            def.Base.LocalFrameA = frames.a; def.Base.LocalFrameB = frames.b;
            return new JointRef
            {
                id = Box3DWorld.Instance.World.CreateSphericalJoint(def).Id,
                isHinge = false, frameA = frames.a, frameB = frames.b,
            };
        }

        private JointRef CreateHinge(Box3DBody a, Box3DBody b, float3 anchor, float3 axis, float hertz, float damping)
        {
            quaternion frameRot = SplineChainController.FrameFromAxisWorld(axis);
            var frames = LocalFrames(a.Body, b.Body, new B3Transform { Position = anchor, Rotation = frameRot });
            var def = RevoluteJointDef.Default;
            def.Hertz = hertz; def.DampingRatio = damping; def.EnableLimit = false;
            def.Base.BodyIdA = a.Body.Id; def.Base.BodyIdB = b.Body.Id; def.Base.CollideConnected = false;
            def.Base.LocalFrameA = frames.a; def.Base.LocalFrameB = frames.b;
            return new JointRef
            {
                id = Box3DWorld.Instance.World.CreateRevoluteJoint(def).Id,
                isHinge = true, frameA = frames.a, frameB = frames.b,
            };
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

        private static float3 BodyPoint(Body body, float3 local)
            => body.GetPosition() + math.mul(body.GetRotation(), local);

        private static quaternion ShortestArc(float3 from, float3 to)
        {
            from = math.normalizesafe(from, kLocalBone);
            to = math.normalizesafe(to, kLocalBone);
            float3 axis = math.cross(from, to);
            float axisLen = math.length(axis);
            if (axisLen < 1e-6f)
                return math.dot(from, to) > 0f ? quaternion.identity
                     : new quaternion(0f, 1f, 0f, 0f);
            return quaternion.AxisAngle(axis / axisLen, math.acos(math.clamp(math.dot(from, to), -1f, 1f)));
        }

        // ---- gizmos ----
        // White OBB  = live body pose.  Yellow sphere = joint anchor (owner-local).
        // Green line = actual bone (anchor -> body center).
        // Red polyline cp0->cp1->cp2 = TARGET pose of the chain (elbow, hand).
        private void OnDrawGizmos()
        {
            if (m_upper != null)
                DrawLink(BodyPoint(m_root.Body, new float3(0f, -m_halfExtents.y * 0.4f, 0f)), m_upper);
            if (m_fore != null)
                DrawLink(BodyPoint(m_upper.Body, new float3(0f, -m_halfExtents.y, 0f)), m_fore);

            // Target pose: red polyline through the control points.
            if (m_cp0 != null && m_cp1 != null && m_cp2 != null)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(m_cp0.position, m_cp1.position);
                Gizmos.DrawLine(m_cp1.position, m_cp2.position);
                Gizmos.DrawWireSphere(m_cp1.position, 0.07f);
                Gizmos.DrawWireSphere(m_cp2.position, 0.07f);
            }
        }

        private void DrawLink(float3 anchor, Box3DBody body)
        {
            float3 p = (float3)body.Body.GetPosition();
            quaternion q = body.Body.GetRotation();

            DrawOBB(p, q, m_halfExtents);
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(new Vector3(anchor.x, anchor.y, anchor.z), 0.05f);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(new Vector3(anchor.x, anchor.y, anchor.z), new Vector3(p.x, p.y, p.z));
        }

        private static void DrawOBB(float3 center, quaternion rot, float3 halfExtents)
        {
            Vector3 c = new Vector3(center.x, center.y, center.z);
            Vector3 rx = math.mul(rot, new float3(halfExtents.x, 0f, 0f));
            Vector3 ry = math.mul(rot, new float3(0f, halfExtents.y, 0f));
            Vector3 rz = math.mul(rot, new float3(0f, 0f, halfExtents.z));
            var p000 = c - rx - ry - rz; var p100 = c + rx - ry - rz;
            var p110 = c + rx + ry - rz; var p010 = c - rx + ry - rz;
            var p001 = c - rx - ry + rz; var p101 = c + rx - ry + rz;
            var p111 = c + rx + ry + rz; var p011 = c - rx + ry + rz;
            Gizmos.color = Color.white;
            Gizmos.DrawLine(p000, p100); Gizmos.DrawLine(p100, p110); Gizmos.DrawLine(p110, p010); Gizmos.DrawLine(p010, p000);
            Gizmos.DrawLine(p001, p101); Gizmos.DrawLine(p101, p111); Gizmos.DrawLine(p111, p011); Gizmos.DrawLine(p011, p001);
            Gizmos.DrawLine(p000, p001); Gizmos.DrawLine(p100, p101); Gizmos.DrawLine(p110, p111); Gizmos.DrawLine(p010, p011);
        }

        private void Log(string message)
        {
            Debug.Log(message);
            m_log?.WriteLine(message);
        }

        private void OnDestroy()
        {
            m_log?.Dispose();
        }
    }
}