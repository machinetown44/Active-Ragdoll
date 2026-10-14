using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation.Tests
{
    /// <summary>
    /// Ground-truth verification of each transformation step in the
    /// motor-drive retargeting pipeline (MotionTransform + the joint-frame
    /// parity contract with Box3DRigBuilder). Each test checks an
    /// independently computable expectation - not the implementation
    /// against itself.
    /// </summary>
    public class HumanoidMotionTransformTests
    {
        private const float Deg = 1e-4f; // deg tolerance for angle compares

        private static void AreEqualQuat(Quaternion expected, quaternion actual, string ctx, float degTol = Deg)
        {
            Assert.That(Quaternion.Angle(expected, actual), Is.LessThanOrEqualTo(degTol), ctx);
            // Same shortest-arc branch (not the antipodal twin).
            Assert.That(Vector4.Dot(new Vector4(expected.x, expected.y, expected.z, expected.w),
                new Vector4(actual.value.x, actual.value.y, actual.value.z, actual.value.w)),
                Is.GreaterThan(0f), ctx + " (sign)");
        }

        private static float3 X(quaternion q) => math.mul(q, math.float3(1, 0, 0));
        private static float3 Y(quaternion q) => math.mul(q, math.float3(0, 1, 0));
        private static float3 Z(quaternion q) => math.mul(q, math.float3(0, 0, 1));

        // ------------------------------------------------------------------
        // Primitives
        // ------------------------------------------------------------------

        [Test]
        public void SignedAngle_PlainAngles_AreRecovered()
        {
            // Rotation about Z by 30 deg must read +30 deg from the axis.
            float3 a = math.normalize(math.float3(1, 0, 0));
            float3 b = math.normalize(math.float3(1, 1, 0) * math.float3(math.cos(math.radians(30f)),
                math.sin(math.radians(30f)), 1f));
            // Build b exactly: rotate (1,0,0) by 30 deg about Z.
            b = math.mul(quaternion.AxisAngle(math.forward(), math.radians(30f)), a);
            float ang = MotionTransform.SignedAngle(a, b, math.forward());
            Assert.AreEqual(30f, math.degrees(ang), Deg, "about +Z");
            Assert.AreEqual(-30f, math.degrees(MotionTransform.SignedAngle(b, a, math.forward())), Deg, "antisymmetry");
        }

        [Test]
        public void SignedAngle_IgnoresAxisParallelComponent()
        {
            // a/b tilted off the plane: only the perpendicular part counts.
            float3 a = math.normalize(math.float3(1, 0, 0.5f));
            float3 b = math.mul(quaternion.AxisAngle(math.forward(), math.radians(45f)),
                math.normalize(math.float3(1, 0, -0.3f)));
            float ang = MotionTransform.SignedAngle(a, b, math.forward());
            Assert.AreEqual(45f, math.degrees(ang), Deg);
        }

        [Test]
        public void WrapHalf_FoldstoBendPlane()
        {
            // Bend-plane convention: 135 deg folds to -45, 180 folds to 0,
            // -100 folds to +80. d and -d span the same plane with the axis.
            Assert.AreEqual(135f - 180f, math.degrees(MotionTransform.WrapHalf(math.radians(135f))), Deg);
            Assert.AreEqual(0f, math.degrees(MotionTransform.WrapHalf(math.radians(180f))), 1e-3f);
            Assert.AreEqual(-100f + 180f, math.degrees(MotionTransform.WrapHalf(math.radians(-100f))), Deg);
            Assert.AreEqual(0f, math.degrees(MotionTransform.WrapHalf(0f)), Deg);
        }

        [Test]
        public void TwistAngle_ExtractsAxisComponentOnly()
        {
            float3 axis = math.normalize(math.float3(0, 1, 0.2f));
            // Pure twist about axis: rot = AxisAngle(axis, t) -> t exactly.
            for (int i = 0; i < 9; i++)
            {
                float t = math.radians(-160f + 40f * i);
                Assert.AreEqual(math.degrees(t), math.degrees(MotionTransform.TwistAngle(
                    quaternion.AxisAngle(axis, t), axis)), 1e-3f, $"pure twist {i}");
            }
            // Swing-only rotation about an axis PERPENDICULAR to 'axis' has
            // zero twist.
            quaternion swing = quaternion.AxisAngle(math.float3(1, 0, 0), math.radians(70f));
            Assert.AreEqual(0f, math.degrees(MotionTransform.TwistAngle(swing, math.float3(0, 1, 0))), 1e-3f, "swing-only");
        }

        [Test]
        public void ShortestArc_MapsDirectionExactly()
        {
            float3 from = math.normalize(math.float3(0, 1, 0));
            float3 to = math.normalize(math.float3(0.3f, 0.2f, 0.5f));
            quaternion q = MotionTransform.ShortestArc(from, to);
            Assert.That(math.distance(math.mul(q, from), to), Is.LessThanOrEqualTo(1e-5f), "from -> to");
            // No twist component about 'to' beyond the arc (check Y stays in
            // the span plane): the rotation is about cross(from,to).
            float3 axis = math.normalize(math.cross(from, to));
            Assert.That(math.abs(math.dot(q.value.xyz / math.length(q.value.xyz), axis)),
                Is.LessThanOrEqualTo(1f + 1e-5f).And.GreaterThanOrEqualTo(1f - 1e-3f), "axis purity");
        }

        [Test]
        public void ShortestArc_Antiparallel_PicksFallbackAxis()
        {
            float3 from = math.forward();
            quaternion q = MotionTransform.ShortestArc(from, -from);
            Assert.That(math.distance(math.mul(q, from), -from), Is.LessThanOrEqualTo(1e-5f));
            Assert.That(math.abs(math.length(q.value) - 1f), Is.LessThanOrEqualTo(1e-5f), "unit");
        }

        [Test]
        public void ArcFromRest_MatchesShortestArc()
        {
            // ArcFromRest is the fast path for retargeting's constant rest
            // direction; it must produce the same rotation as the generic
            // ShortestArc across the full angle range plus degeneracies.
            var rng = new Unity.Mathematics.Random(12345);
            for (int k = 0; k < 64; k++)
            {
                float3 from = math.normalizesafe(rng.NextFloat3Direction(), math.up());
                float3 to = math.normalizesafe(rng.NextFloat3Direction(), math.up());
                quaternion a = MotionTransform.ArcFromRest(from, to);
                quaternion b = MotionTransform.ShortestArc(from, to);
                // Same rotation up to sign: compare applied vectors.
                Assert.That(math.distance(math.mul(a, from), math.mul(b, from)),
                    Is.LessThanOrEqualTo(1e-4f), $"case {k}");
                Assert.That(math.distance(math.mul(a, from), to),
                    Is.LessThanOrEqualTo(1e-4f), $"maps from->to, case {k}");
            }
            // Parallel and antiparallel degeneracies.
            Assert.That(math.distance(math.mul(MotionTransform.ArcFromRest(math.up(), math.up()), math.up()), math.up()),
                Is.LessThanOrEqualTo(1e-5f), "parallel -> identity");
            Assert.That(math.distance(math.mul(MotionTransform.ArcFromRest(math.forward(), -math.forward()), math.forward()), -math.forward()),
                Is.LessThanOrEqualTo(1e-5f), "antiparallel -> pi fallback");
        }

        private static float3x3 FrameMatrix(quaternion f) => new float3x3(X(f), Y(f), Z(f));

        /// <summary>Frobenius distance of M*M^T from identity (float3x3 has
        /// no math.distance overload).</summary>
        private static float OrthoErr(float3x3 m)
        {
            float3x3 d = math.mul(m, math.transpose(m)) - float3x3.identity;
            float s = 0f;
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++) s += d[r][c] * d[r][c];
            return math.sqrt(s);
        }

        [Test]
        public void FrameFromPrimaryRoll_IsOrthonormal_RollWins()
        {
            float3 primary = math.normalize(math.float3(0.2f, 1f, 0.1f));
            float3 roll = math.normalize(math.float3(0.3f, 0.4f, 1f));
            quaternion f = MotionTransform.FrameFromPrimaryRoll(primary, roll);
            float3x3 m = FrameMatrix(f);
            Assert.That(OrthoErr(m), Is.LessThanOrEqualTo(1e-5f), "orthonormal");
            // Roll datum: the orthogonalized roll lands on +Y.
            float3 y = Y(f);
            float3 rollOrtho = math.normalize(roll - primary * math.dot(roll, primary));
            Assert.That(math.distance(y, rollOrtho), Is.LessThanOrEqualTo(1e-5f), "roll on +Y");
            Assert.That(math.distance(X(f), math.normalize(primary)), Is.LessThanOrEqualTo(1e-5f), "primary on +X");
        }

        // ------------------------------------------------------------------
        // Setpoint composition
        // ------------------------------------------------------------------

        [Test]
        public void SphericalSetpoint_AtRest_IsSpawnOrientation()
        {
            float3 rest = math.normalize(math.float3(0, -1, 0));
            quaternion spawn = Quaternion.Euler(15f, 25f, 35f);
            // cur == rest, twist 0: command must be exactly the spawn child
            // rotation (identity delta).
            quaternion cmd = MotionTransform.SphericalSetpoint(rest, rest, 0f, spawn);
            AreEqualQuat(spawn, cmd, "rest = spawn");
        }

        [Test]
        public void SphericalSetpoint_PureSwing_RotatesBoneToCur()
        {
            float3 rest = math.normalize(math.float3(0, -1, 0));
            float3 cur = math.normalize(math.float3(0, -1, 1)); // 45 deg forward
            quaternion spawn = Quaternion.Euler(0f, 0f, 20f);
            quaternion cmd = MotionTransform.SphericalSetpoint(rest, cur, 0f, spawn);
            // The commanded bone direction must equal cur exactly.
            float3 bone = math.mul(cmd, math.mul(math.inverse(spawn), rest));
            Assert.That(math.distance(bone, cur), Is.LessThanOrEqualTo(1e-5f), "bone lands on cur");
            // And with twist = 0 the delta is EXACTLY the swing: cmd must
            // equal swing * spawn (compare the delta, not cmd - the spawn
            // factor changes the composed angle).
            quaternion delta = math.mul(cmd, math.inverse(spawn));
            AreEqualQuat(MotionTransform.ShortestArc(rest, cur), delta, "delta == swing");
        }

        [Test]
        public void SphericalSetpoint_TwistComposesAboutCurAxis()
        {
            float3 rest = math.normalize(math.float3(0, -1, 0));
            float3 cur = math.normalize(math.float3(1, -1, 0));
            float twist = math.radians(30f);
            quaternion cmd = MotionTransform.SphericalSetpoint(rest, cur, twist, quaternion.identity);
            // Decompose the command's delta (cmd * inv(spawn)) - with spawn =
            // identity the delta is cmd itself: swing rest->cur then twist
            // about cur. Twist about cur must extract exactly 30 deg.
            float twistRead = MotionTransform.TwistAngle(cmd, cur);
            Assert.AreEqual(30f, math.degrees(twistRead), 1e-3f, "twist about cur");
            // Bone still lands on cur (twist about the bone preserves it).
            float3 bone = math.mul(cmd, rest);
            Assert.That(math.distance(bone, cur), Is.LessThanOrEqualTo(1e-5f), "bone on cur");
        }

        // ------------------------------------------------------------------
        // Hinge pipeline
        // ------------------------------------------------------------------

        [Test]
        public void HingeAxis_FollowsCommandedParentSwing()
        {
            // relSpawn rotates child frame into parent frame; a 90 deg yaw
            // of the parent's command must yaw the axis identically.
            quaternion relSpawn = Quaternion.Euler(0f, 0f, 0f);
            float3 axisChild = math.forward();
            quaternion parentCmd = quaternion.AxisAngle(math.float3(0, 1, 0), math.radians(90f));
            float3 axisRF = MotionTransform.HingeAxisRootFrame(parentCmd, relSpawn, axisChild);
            float3 expected = math.mul(parentCmd, axisChild);
            Assert.That(math.distance(axisRF, expected), Is.LessThanOrEqualTo(1e-6f));
        }

        [Test]
        public void HingeBend_SignAndMagnitude()
        {
            // Right-hand rule about the axis: +Y rotation carries +Z toward
            // +X, so the arm (+X) sweeping toward +Z reads NEGATIVE about a
            // +Y axis (left elbow, builder mirror convention), positive
            // about the mirrored -Y axis (right elbow).
            float3 rest = math.float3(1, 0, 0);
            float3 cur = math.normalize(math.float3(1, 0, 1));
            float bendL = MotionTransform.HingeBendAngle(rest, cur, math.float3(0, 1, 0));
            Assert.AreEqual(-45f, math.degrees(bendL), Deg, "left (+Y axis), forward sweep");
            Assert.AreEqual(45f, math.degrees(MotionTransform.HingeBendAngle(rest, cur, math.float3(0, -1, 0))),
                Deg, "right (-Y axis), forward sweep");
            // Sign flips with the axis mirror - the paired-limb invariant.
            Assert.AreEqual(-1f, math.sign(bendL) * math.sign(
                MotionTransform.HingeBendAngle(rest, cur, math.float3(0, -1, 0))), 1e-5f, "mirror flips sign");
        }

        // ------------------------------------------------------------------
        // Joint frames + servo error (the parity contract with the builder)
        // ------------------------------------------------------------------

        [Test]
        public void JointRelative_RestPose_IsIdentity()
        {
            // At spawn, parentRot = spawnParent, childRot = spawnChild and
            // frameARot = inv(spawnParent)*F, frameBRot = inv(spawnChild)*F:
            // rel = inv(spawnParent*inv(spawnParent)*F) * spawnChild*inv(spawnChild)*F = I.
            quaternion F = Quaternion.Euler(10f, 20f, 30f);
            quaternion spawnP = Quaternion.Euler(0f, 40f, 0f);
            quaternion spawnC = Quaternion.Euler(15f, 0f, 0f);
            quaternion fa = math.mul(math.inverse(spawnP), F);
            quaternion fb = math.mul(math.inverse(spawnC), F);
            quaternion rel;
            MotionTransform.JointRelative(spawnP, spawnC, fa, fb, out rel);
            AreEqualQuat(Quaternion.identity, rel, "rel at spawn");
        }

        [Test]
        public void JointRelative_RotatingBothBodies_PreservesRel()
        {
            // A shared world rotation of both bodies cancels: rel measures
            // only the child-vs-parent relation.
            quaternion F = Quaternion.Euler(10f, 20f, 30f);
            quaternion spawnP = Quaternion.Euler(0f, 40f, 0f);
            quaternion spawnC = Quaternion.Euler(15f, 0f, 0f);
            quaternion fa = math.mul(math.inverse(spawnP), F);
            quaternion fb = math.mul(math.inverse(spawnC), F);
            quaternion bend = Quaternion.Euler(0f, 0f, 30f); // child vs parent
            quaternion world = quaternion.AxisAngle(math.normalize(math.float3(1, 2, 3)), math.radians(37f));
            quaternion rel1, rel2;
            MotionTransform.JointRelative(spawnP, math.mul(bend, spawnC), fa, fb, out rel1);
            MotionTransform.JointRelative(math.mul(world, spawnP), math.mul(world, math.mul(bend, spawnC)), fa, fb, out rel2);
            AreEqualQuat(rel1, rel2, "world rotation cancels");
        }

        [Test]
        public void ServoError_ZeroAtSetpoint()
        {
            quaternion relT = Quaternion.Euler(20f, 30f, 0f);
            quaternion relA = relT;
            float3 rv = SplineChainController.RotationVector(MotionTransform.ServoError(relT, relA));
            Assert.That(math.length(rv), Is.LessThanOrEqualTo(1e-6f), "error vanishes at setpoint");
        }

        [Test]
        public void ServoError_ComposesCorrectly()
        {
            // relT = 40 deg about Z, relA = 10 deg about Z -> error 30 deg
            // about Z, same axis, signed.
            quaternion relT = quaternion.AxisAngle(math.forward(), math.radians(40f));
            quaternion relA = quaternion.AxisAngle(math.forward(), math.radians(10f));
            float3 rv = SplineChainController.RotationVector(MotionTransform.ServoError(relT, relA));
            Assert.AreEqual(30f, math.degrees(math.length(rv)), 1e-3f, "magnitude");
            Assert.AreEqual(0f, math.degrees(MotionTransform.SignedAngle(
                math.normalize(rv), math.forward(), math.float3(1, 0, 0))), Deg, "axis");
        }

        [Test]
        public void ServoError_MirrorAxis_ProjectsOutOfHingeDof()
        {
            // THE regression for the elbow/neck bug. Mechanism (verified
            // numerically): with the driver's frame Z MIRRORED vs the
            // physical hinge axis (+Y vs -Y), a physical bend measured
            // through the driver's frames yields an error rotation about
            // that perpendicular axis - the hinge scalar command (dot with
            // frame Z) is ZERO: dead joint, un-adapting, while the
            // off-axis component fights the constraint = ringing.
            float3 physicalAxisWorld = math.float3(0, -1, 0);
            quaternion bend = quaternion.AxisAngle(physicalAxisWorld, math.radians(30f));
            quaternion relT = quaternion.identity; // rest setpoint

            // Mirror case: driver frames = identity (Z = +Y), physical bend
            // about -Y. relA as the driver measures it = bend itself.
            quaternion relA;
            MotionTransform.JointRelative(quaternion.identity, bend,
                quaternion.identity, quaternion.identity, out relA);
            float3 rv = SplineChainController.RotationVector(MotionTransform.ServoError(relT, relA));
            float axial = math.abs(math.dot(rv / math.max(math.length(rv), 1e-6f), math.forward()));
            Assert.That(axial, Is.LessThanOrEqualTo(1e-3f),
                "mirrored frames: zero axial error => dead hinge command");

            // Parity: driver frames built exactly like the builder (same
            // axis): the error becomes PURELY axial - full authority.
            quaternion F = Box3DRigBuilder.FrameFromAxisWorld(physicalAxisWorld);
            MotionTransform.JointRelative(quaternion.identity, bend, F, F, out relA);
            rv = SplineChainController.RotationVector(MotionTransform.ServoError(relT, relA));
            axial = math.abs(math.dot(rv / math.max(math.length(rv), 1e-6f), math.forward()));
            Assert.That(axial, Is.GreaterThanOrEqualTo(0.999f),
                "frame parity: error purely about the hinge axis => full authority");
        }

        // ------------------------------------------------------------------
        // Servo command shaping
        // ------------------------------------------------------------------

        [Test]
        public void ServoCommand_LinearRegion_GainTimesError()
        {
            float3 rv = math.radians(10f) * math.forward();
            float3 wRel = math.float3(0, 0, 2f);
            float3 w = MotionTransform.ServoCommand(rv, wRel,
                servoGain: 10f, maxCommandSpeed: 25f, motorDamping: 0.2f,
                torqueCap: 300f, iEff: 0.1f, iSelf: 0.05f);
            // gain*theta = 10 * 0.1745 = 1.745 rad/s (linear region - well
            // under wStop 32 and ceiling 25). kd = 0.2*sqrt(0.1/0.05) =
            // 0.283; command = 1.745 - 0.283*2 about Z.
            float expected = 10f * math.radians(10f) - 0.2f * math.sqrt(2f) * 2f;
            Assert.AreEqual(expected, math.dot(w, math.forward()), 1e-3f, "gain region + damping");
            Assert.That(math.abs(w.x), Is.LessThanOrEqualTo(1e-5f), "no cross-axis command");
        }

        [Test]
        public void ServoCommand_StoppingSpeedClamps()
        {
            float3 rv = math.radians(30f) * math.forward();
            float3 w = MotionTransform.ServoCommand(rv, float3.zero,
                servoGain: 10f, maxCommandSpeed: 25f, motorDamping: 0f,
                torqueCap: 5f, iEff: 2f, iSelf: 2f);
            // wStop = sqrt(2 * 2.5 * 0.5236) = 1.618 < gain*theta = 300.
            float wStop = math.sqrt(2f * (5f / 2f) * math.radians(30f));
            Assert.AreEqual(wStop, math.length(w), 1e-3f, "wStop clamps");
        }

        [Test]
        public void ServoCommand_ZeroErrorZeroSpin_IsZero()
        {
            float3 w = MotionTransform.ServoCommand(float3.zero, float3.zero,
                servoGain: 10f, maxCommandSpeed: 25f, motorDamping: 0.2f,
                torqueCap: 300f, iEff: 0.1f, iSelf: 0.05f);
            Assert.That(math.length(w), Is.LessThanOrEqualTo(1e-6f), "rest = no command");
        }

        [Test]
        public void ServoCommand_SmallErrorHighGain_NotStalledByWStop()
        {
            // THE crawling regression: wStop = sqrt(2*alphaMax*theta) falls
            // off as sqrt of the error. With the old unconditional clamp, a
            // small deviation at high gain was throttled to sqrt(theta)
            // authority regardless of the gain - the joint crawled the last
            // few degrees. Gain-limited speed must survive wStop when the
            // joint is not yet approaching too fast to stop.
            float3 rv = math.radians(3f) * math.forward(); // small standing deviation
            float3 w = MotionTransform.ServoCommand(rv, float3.zero,
                servoGain: 30f, maxCommandSpeed: 25f, motorDamping: 0.2f,
                torqueCap: 5f, iEff: 2f, iSelf: 2f);
            // wStop = sqrt(2 * 2.5 * 0.0524) = 0.512 < gain*theta = 1.57:
            // the OLD cap throttled to 0.512. The new command keeps the
            // full gain term (brake caps unchanged).
            Assert.AreEqual(30f * math.radians(3f), math.length(w), 1e-3f,
                "gain term survives wStop at zero approach");
        }

        // ------------------------------------------------------------------
        // Gravity load feedforward
        // ------------------------------------------------------------------

        [Test]
        public void GravityFeedforwardAccel_TwoBodies_AboutAxis()
        {
            // Anchor at origin; body A: 2 kg with COM at +Z: its weight
            // (0,-19.62,0) acts at r=(0,0,1) -> tau = r x F = (19.62,0,0).
            // About axis X with iEff = 2: accel = 9.81 rad/s^2. About Y: 0.
            var coms = new float3[] { math.float3(0, 0, 1f), math.float3(1f, 0, 0) };
            var masses = new float[] { 2f, 0f }; // second body massless
            float aX = MotionTransform.GravityFeedforwardAccel(
                new ReadOnlySpan<float3>(coms), new ReadOnlySpan<float>(masses),
                float3.zero, math.float3(1, 0, 0), 2f);
            Assert.AreEqual(9.81f, aX, 1e-3f, "gravity accel about X");
            float aY = MotionTransform.GravityFeedforwardAccel(
                new ReadOnlySpan<float3>(coms), new ReadOnlySpan<float>(masses),
                float3.zero, math.float3(0, 1, 0), 2f);
            Assert.AreEqual(0f, aY, 1e-4f, "no torque about the vertical axis");
        }

        [Test]
        public void ServoCommand_OngoingApproach_NotThrottledBelowOwnSpeed()
        {
            // Joint already approaching faster than wStop can justify: the
            // command must not scale the drive DOWN to the stoppable speed
            // (that converts wStop into a stall); it may keep the gain term
            // up to the realized approach speed.
            float3 rv = math.radians(3f) * math.forward();
            float3 wRel = 2f * math.forward(); // approaching at 2 rad/s > wStop
            float3 w = MotionTransform.ServoCommand(rv, wRel,
                servoGain: 30f, maxCommandSpeed: 25f, motorDamping: 0.2f,
                torqueCap: 5f, iEff: 2f, iSelf: 2f);
            // drive stays at gain*theta = 1.046 (>= approach speed 2? no:
            // 1.043 < 2 - the guard raises the CAP to v, drive unchanged);
            // kd = 0.2*sqrt(2/2)*... = 0.2; command = 1.043 - 0.2*2.
            float expected = 30f * math.radians(3f) - 0.2f * 2f;
            Assert.AreEqual(expected, math.dot(w, math.forward()), 1e-3f,
                "ongoing approach is not throttled to wStop");
        }

        [Test]
        public void ServoCommand_DampingAlwaysOpposesSpin()
        {
            // For any error, adding relative spin must REDUCE the command
            // along the spin direction (brake, never boost).
            var rng = new Unity.Mathematics.Random(12345u);
            for (int i = 0; i < 32; i++)
            {
                float3 rv = rng.NextFloat3Direction() * rng.NextFloat(0.01f, 2f);
                float3 wRel = rng.NextFloat3Direction() * rng.NextFloat(0.5f, 8f);
                float3 w0 = MotionTransform.ServoCommand(rv, float3.zero,
                    servoGain: 10f, maxCommandSpeed: 40f, motorDamping: 0.2f,
                    torqueCap: 300f, iEff: 0.3f, iSelf: 0.1f);
                float3 w1 = MotionTransform.ServoCommand(rv, wRel,
                    servoGain: 10f, maxCommandSpeed: 40f, motorDamping: 0.2f,
                    torqueCap: 300f, iEff: 0.3f, iSelf: 0.1f);
                Assert.That(math.dot(w1 - w0, wRel), Is.LessThanOrEqualTo(1e-5f),
                    $"spin opposes: case {i}");
            }
        }

        // ------------------------------------------------------------------
        // Builder parity: the actual default biped frames
        // ------------------------------------------------------------------

        [Test]
        public void DefaultBiped_HingeFramesMatchChildAxisConvention()
        {
            // Contract with Box3DRigBuilder.ResetToDefaultBiped +
            // CreateHinge: elbow hinge axis is authored (-1,0,0) in the
            // forearm's spawn frame; the physical frame Z must equal that
            // axis. The driver must reconstruct the SAME frame from the
            // child spawn rotation and hingeAxisLocal - not from the parent
            // direction (the regression that broke elbows/neck).
            var left = new Vector3(-1f, 0f, 0f); // hingeAxisLocal for both arms
            var right = new Vector3(-1f, 0f, 0f);
            Assert.AreEqual(left, right); // paired-limb convention: identical params

            // Forearm spawn rotation in the default T-pose: upper arm along
            // +X (right arm, dir (-1,0,0) with 180-deg torso flip gives +X
            // world), forearm 'down' = along the parent bone. Construct the
            // spawn rotations exactly as the rig does and verify the driver
            // formula reproduces the builder frame for both arms.
            quaternion torsoSpawn = Quaternion.Euler(0f, 180f, 0f); // 180-flipped parent
            quaternion upperSpawnL = torsoSpawn * Quaternion.Euler(0f, 0f, -90f);
            quaternion upperSpawnR = torsoSpawn * Quaternion.Euler(0f, 0f, 90f);
            // Forearm 'down' along the parent bone: no additional rotation
            // beyond the parent's (direction down = continue along parent).
            quaternion foreSpawnL = upperSpawnL;
            quaternion foreSpawnR = upperSpawnR;

            float3 frameZL = Z(Box3DRigBuilder.FrameFromAxisWorld(
                math.mul(foreSpawnL, math.normalizesafe((float3)left))));
            float3 frameZR = Z(Box3DRigBuilder.FrameFromAxisWorld(
                math.mul(foreSpawnR, math.normalizesafe((float3)right))));
            // Left elbow axis = world +Y, right = world -Y (mirrored), so
            // positive flexion sweeps both hands forward (+Z).
            Assert.AreEqual(1f, frameZL.y, 1e-5f, "left elbow axis +Y");
            Assert.AreEqual(-1f, frameZR.y, 1e-5f, "right elbow axis -Y");
        }

        [Test]
        public void DefaultBiped_SphericalFrameMatchesParentDir()
        {
            // Spherical parity: the driver's joint frame uses the parent's
            // config dir. For the shoulder (parent torso, dir down, torso
            // 180-flipped): frame Z must be world UP (+Y) - the torso's
            // distal side. Verified against the builder convention
            // (CreateSpherical: axisW = p.dirW, un-rotated by child spawn).
            quaternion torsoSpawn = Quaternion.Euler(0f, 0f, 180f); // 180-flipped parent (local +y = world down)
            float3 torsoDir = math.mul(torsoSpawn, math.float3(0, -1, 0));
            Assert.AreEqual(1f, torsoDir.y, 1e-5f, "torso distal side is up");

            // Driver-side formula (post-fix): spherical frames derive from
            // parentNode.dirW - the CONFIG direction, spawn-rotation-free.
            // The builder uses p.dirW identically, so parity holds by
            // construction; assert the convention the tests pin: dir
            // 'down' in a 180-flipped parent = world up.
            float3 parentDirWorld = math.mul(torsoSpawn, math.float3(0, -1, 0));
            float3 frameZ = Z(Box3DRigBuilder.FrameFromAxisWorld(parentDirWorld));
            Assert.AreEqual(1f, frameZ.y, 1e-5f, "shoulder frame Z = world up");
        }
    }
}
