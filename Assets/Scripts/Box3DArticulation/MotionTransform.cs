using System;
using Unity.Mathematics;

namespace Box3DArticulation
{
    /// <summary>
    /// Pure (no physics, no Unity objects) transformation steps of the
    /// motion-driver retargeting pipeline. Every function here is verified
    /// by HumanoidMotionTransformTests so the runtime servo consumes
    /// setpoints from verified math.
    ///
    /// ROOT-RELATIVE INVARIANCE (design invariant): the world-space
    /// orientation of the source root or the rig root must not affect any
    /// relative-joint command. The pipeline contains NO world-to-rig
    /// conversions: every observation is expressed in its parent segment's
    /// estimated frame, and both the observation and the frame rotate
    /// together under a rigid rotation, so the contamination cancels exactly
    /// at every level. Source-root rotation is never commanded into a joint -
    /// the rig root pose is a free DOF owned by locomotion.
    /// Prerequisite: the tree ROOT must be a fully measured chain reference
    /// (pivot/tip/roll) so its frame is covariant; ComposeBindFrames falls
    /// back to a swing-only root frame otherwise (NOT covariant - warn).
    /// </summary>
    public static class MotionTransform
    {
        /// <summary>Orthonormal frame quaternion from a primary axis and a
        /// (roughly perpendicular) roll direction: x = primary, y = roll
        /// orthogonalized, z = x cross y.</summary>
        public static quaternion FrameFromPrimaryRoll(float3 primary, float3 rollRaw)
        {
            float3 x = math.normalizesafe(primary, math.float3(0, 1, 0));
            float3 y = math.normalizesafe(rollRaw - x * math.dot(rollRaw, x), OrthoTo(x));
            return new quaternion(new float3x3(x, y, math.cross(x, y)));
        }

        /// <summary>Signed twist component of 'rot' about 'axis', in
        /// (-pi, pi] (swing-twist decomposition).</summary>
        public static float TwistAngle(quaternion rot, float3 axis)
        {
            float4 v = rot.value;
            float proj = math.dot(v.xyz, axis);
            float ang = 2f * math.atan2(proj, v.w);
            if (ang > math.PI) ang -= 2f * math.PI;
            if (ang <= -math.PI) ang += 2f * math.PI;
            return ang;
        }

        /// <summary>Signed angle from 'a' to 'b' about 'axis' (all unit,
        /// a/b not necessarily perpendicular to axis).</summary>
        public static float SignedAngle(float3 a, float3 b, float3 axis)
        {
            float3 ap = a - axis * math.dot(a, axis);
            float3 bp = b - axis * math.dot(b, axis);
            if (math.lengthsq(ap) < 1e-8f || math.lengthsq(bp) < 1e-8f) return 0f;
            ap = math.normalizesafe(ap);
            bp = math.normalizesafe(bp);
            return math.atan2(math.dot(math.cross(ap, bp), axis), math.dot(ap, bp));
        }

        /// <summary>Wrap 'ang' into (-pi/2, pi/2] - bend-plane comparison:
        /// directions d and -d span the same plane with the axis, so a
        /// hinge flipping bend side must read as zero twist, not pi.</summary>
        public static float WrapHalf(float ang)
        {
            const float halfPi = math.PI * 0.5f;
            while (ang > halfPi) ang -= math.PI;
            while (ang <= -halfPi) ang += math.PI;
            return ang;
        }

        /// <summary>Any unit vector perpendicular to 'v'.</summary>
        public static float3 OrthoTo(float3 v)
            => math.abs(v.x) < 0.9f ? math.normalizesafe(math.cross(v, math.float3(1, 0, 0)))
                                    : math.normalizesafe(math.cross(v, math.float3(0, 1, 0)));

        /// <summary>Rotation carrying 'from' onto 'to' by the shortest arc
        /// (undefined twist). Antiparallel case picks a fallback axis.</summary>
        public static quaternion ShortestArc(float3 from, float3 to)
        {
            float3 axis = math.cross(from, to);
            float d = math.dot(from, to);
            if (d < -0.999999f)
            {
                axis = OrthoTo(from);
                return quaternion.AxisAngle(axis, math.PI);
            }
            float w = 1f + d;
            float l = math.sqrt(w * w + math.lengthsq(axis));
            return l < 1e-8f ? quaternion.identity : new quaternion(math.float4(axis / l, w / l));
        }

        // --- setpoint composition helpers (verified by tests) ---------------

        /// <summary>Commanded orientation from a rest->current bone-direction
        /// delta (shortest arc, swing only) plus a twist about the CURRENT
        /// bone axis, composed onto the spawn orientation. Convention-free
        /// helper: callers conjugate the directions into their comparison
        /// frame first.</summary>
        public static quaternion SphericalSetpoint(float3 rest, float3 cur,
            float twist, quaternion spawn)
        {
            quaternion deltaW = math.mul(quaternion.AxisAngle(cur, twist), ShortestArc(rest, cur));
            return math.mul(deltaW, spawn);
        }

        /// <summary>Root-frame hinge axis: the child's authored hinge axis
        /// (child spawn frame) mapped through relSpawn (the spawn-relative
        /// joint rotation, frameA*inv(frameB)) and the parent's commanded
        /// rotation, so the bend axis follows the commanded swing.</summary>
        public static float3 HingeAxisRootFrame(quaternion parentCmdRot, quaternion relSpawn, float3 hingeAxisChild)
        {
            return math.mul(math.mul(parentCmdRot, relSpawn), hingeAxisChild);
        }

        /// <summary>Commanded hinge bend angle: signed angle from the rest
        /// to the observed direction about the commanded hinge axis.</summary>
        public static float HingeBendAngle(float3 restDirRF, float3 curDirRF, float3 axisRF)
        {
            return SignedAngle(restDirRF, curDirRF, axisRF);
        }

        /// <summary>Joint-frame-relative rotations, target and measured.
        /// relT is the servo setpoint; relA is what the physical joint
        /// currently realizes. Both use the SAME frameARot/frameBRot
        /// (builder parity is what makes this identity meaningful).</summary>
        public static void JointRelative(quaternion parentRot, quaternion childRot,
            quaternion frameARot, quaternion frameBRot,
            out quaternion rel)
        {
            rel = math.mul(math.inverse(math.mul(parentRot, frameARot)),
                           math.mul(childRot, frameBRot));
        }

        /// <summary>Servo error in joint-A space: e = relT * inv(relA).
        /// Identity (zero rotation vector) exactly when the joint is on its
        /// setpoint. This is the quantity the motor command is built from.</summary>
        public static quaternion ServoError(quaternion relT, quaternion relA)
            => math.mul(relT, math.inverse(relA));

        /// <summary>Gravity load feedforward: the angular ACCELERATION to add
        /// to the joint command so the servo pre-compensates the distal
        /// subtree's weight. The gravity torque about the command axis
        /// (tau = sum of cross(comOffset, m*g) over the distal bodies) points
        /// AWAY from the target when the joint sags - its projection is
        /// already negative along the error direction - so the compensation
        /// is the NEGATION: accel = -dot(tau, axis) / iEff. Positive
        /// multipliers on this value then reduce sag; negative ones amplify
        /// it (verified empirically before the fix - the inversion was real).
        /// Callers add accel*dt to the command AFTER the caps - load
        /// compensation is not error-driven and must not be scaled by the
        /// stop-law. Caller fills the parallel scratch arrays (no delegates,
        /// no allocations in the hot loop).</summary>
        public static float GravityFeedforwardAccel(
            ReadOnlySpan<float3> comW, ReadOnlySpan<float> mass,
            float3 anchorW, float3 axisW, float iEff)
        {
            float3 tau = float3.zero;
            for (int i = 0; i < comW.Length; i++)
            {
                float m = mass[i];
                if (m <= 0f) continue;
                float3 r = comW[i] - anchorW;
                tau += math.cross(r, m * Gravity);
            }
            return math.dot(tau, axisW) / math.max(iEff, 1e-4f);
        }

        /// <summary>World gravity (matches Physics.gravity default).</summary>
        public static readonly float3 Gravity = math.float3(0f, -9.81f, 0f);

        /// <summary>Relative-rotation error to angular velocity command in
        /// joint-A space, bandwidth-limited (BackboneWaveProbe recipe):
        /// speed = min(servoGain*theta, maxCommandSpeed), throttled by wStop
        /// (from the torque cap and the composite distal inertia) ONLY as an
        /// overshoot guard - an ongoing approach beyond wStop is never
        /// throttled below its own speed, because a sqrt(theta) drive cap
        /// withdraws authority exactly at small errors (the standing-pose
        /// crawl). The stop itself is the brake's job. kd damps the
        /// measured relative spin, capped so it stays a brake.</summary>
        public static float3 ServoCommand(float3 rv, float3 wRelA,
            float servoGain, float maxCommandSpeed, float motorDamping,
            float torqueCap, float iEff, float iSelf)
        {
            float theta = math.length(rv);
            if (theta < 1e-6f && math.lengthsq(wRelA) < 1e-12f) return float3.zero;
            float alphaMax = torqueCap / math.max(iEff, 1e-4f);
            float wStop = math.sqrt(2f * alphaMax * math.max(theta, 1e-6f));
            float mag = math.min(servoGain * theta, maxCommandSpeed);
            float kdHere = math.min(motorDamping * math.sqrt(iEff / math.max(iSelf, 1e-4f)), 3f);
            float3 dirV = theta > 1e-6f ? rv / theta : float3.zero;
            // Overshoot guard on the DRIVE only: cap the commanded approach
            // at wStop, but never below the approach speed the joint is
            // ALREADY realizing (dot of measured spin with the error
            // direction) - throttling an ongoing approach converts the
            // wStop cap into a sqrt(theta) stall (small errors crawl).
            // Stopping is the brake's job; see brakeCap below.
            float vApproach = math.dot(wRelA, dirV);
            if (vApproach > 0f)
                mag = math.min(mag, math.max(wStop, vApproach));
            else
                mag = math.min(mag, wStop); // zero/receding approach: stoppable speeds only
            // Clamp the terms INDEPENDENTLY. The brake must keep authority as
            // theta -> 0: a total-command clamp scales kd*wRel down with mag
            // (= gain*theta), so the joint arrives unbraked and overshoots
            // (the at-target oscillation). Reversal is still bounded - the
            // brake can never exceed wStop, so no 60 rad/s bang-bang.
            float3 drive = dirV * mag;
            float3 brake = -kdHere * wRelA;
            float brakeCap = math.max(wStop, mag);
            float bLen = math.length(brake);
            if (bLen > brakeCap) brake *= brakeCap / bLen;
            return drive + brake;
        }

        /// <summary>Reference feedforward variant: adds the animated target's
        /// own joint-relative velocity + acceleration (lead time = dt) to the
        /// PD command. FF is added AFTER the independent caps - it tracks the
        /// reference, it is not error-driven, so it must not be scaled down by
        /// the stopping-speed logic (small errors during fast motion must
        /// still carry the reference velocity).</summary>
        public static float3 ServoCommandFF(float3 rv, float3 wRelA,
            float servoGain, float maxCommandSpeed, float motorDamping,
            float torqueCap, float iEff, float iSelf,
            float3 wRefA, float3 alphaRefA, float dt)
        {
            float3 w = ServoCommand(rv, wRelA, servoGain, maxCommandSpeed,
                motorDamping, torqueCap, iEff, iSelf);
            return w + wRefA + alphaRefA * dt;
        }

        // --- chain retargeting (root-relative) ------------------------------

        /// <summary>Per-node state for the root-relative retargeting pipeline.
        /// Bind-time fields are filled once (BuildChainTopology +
        /// ComposeBindFrames); the caller refreshes the per-step observations
        /// (curDirW everywhere, curRollDirW for chain references) each step,
        /// then runs RetargetChains. Tests build synthetic chains directly.</summary>
        public struct ChainSimNode
        {
            // --- topology (bind) ---
            public int parent;            // -1 = root
            public int chain;             // chain id, -1 = none
            public float s;               // arc length from chain start (rest pivots)
            public int nextRefIdx;        // nearest chain ref ABOVE in the same chain (-1 = none)
            public float nextRefS;
            public int baseRef;           // nearest chain ref on the ancestor path (-1 = none)
            public bool isChainRef;       // full-frame reference (measures swing + twist)
            public bool hasJoint;
            public bool isHinge;          // this node's JOINT is a hinge (child side)
            // --- bind geometry (world) ---
            public float3 restPivotW;     // pivot at rest - arc-length input
            public float3 restDirW;       // pivot->tip at rest
            public float3 restBendW;      // rest bend of the hinge children about this node's axis
            public quaternion restFrame;  // chain refs: measured bind frame (world)
            public quaternion bindFrame;  // F0: estimated bind segment frame (ALL nodes)
            public quaternion restRel;    // inv(F0_parent) * F0_self
            public quaternion restRelInv; // inv(restRel) - chain-ref twist datum
            public quaternion invBindFrame; // inv(bindFrame) - bind constant
            public float3 restDirRel;     // inv(F0_parent) * restDirW - bind constant
            public bool hingeObsValid;    // hinge children: parent axis/bend datums usable
            public float3 hingeAxisRel;   // hinge children: parent bone axis (normalized, parent bind frame)
            public float3 hingeBendRel;   // hinge children: rest bend dir (projected, parent bind frame)
            public quaternion tgtA;       // inv(Jw) * F0_parent
            public quaternion tgtB;       // inv(F0_self) * Jw
            public quaternion spawnChildRot;
            public quaternion frameARot;  // builder-parity joint frames
            public quaternion frameBRot;
            // --- per-step observations (caller-filled) ---
            public float3 curDirW;        // pivot->tip observed (world)
            public float3 curRollDirW;    // chain refs: roll direction observed (world); zero = none
            // --- outputs ---
            public float twist;           // twist about the own bone axis vs bind (rad, parent-frame datum)
            public float obsTwist;        // hinge-child observation for this node (rad)
            public float obsConf;         // observation confidence 0..1
            public quaternion cmdRot;     // estimated absolute segment frame (root: measured/fallback)
            public quaternion relT;       // joint target in builder-parity joint frames (identity = spawn)
        }

        /// <summary>Bind-time chain topology: chains grow along aligned rest
        /// bones (child dir vs parent dir dot > 'chainAxisDot'), arc length
        /// accumulates along the chain; every joint then gets its nearest
        /// chain reference ABOVE it in the same chain, and the nearest
        /// reference on its ancestor path (the chain base). Requires
        /// parent/restPivotW/restDirW filled and parents ordered before
        /// children in 'order'.</summary>
        public static void BuildChainTopology(ChainSimNode[] nodes, int[] order, float chainAxisDot)
        {
            for (int oi = 0; oi < order.Length; oi++)
            {
                int i = order[oi];
                var n = nodes[i];
                if (n.parent < 0 || !n.hasJoint) { nodes[i] = n; continue; }
                var p = nodes[n.parent];
                bool sameAxis = math.abs(math.dot(n.restDirW, p.restDirW)) > chainAxisDot;
                if (p.parent >= 0 && p.chain >= 0 && sameAxis)
                {
                    n.chain = p.chain;
                    n.s = p.s + math.length(n.restPivotW - p.restPivotW);
                }
                else
                {
                    n.chain = i; // new chain keyed by its start node
                    n.s = 0f;
                }
                nodes[i] = n;
            }

            for (int oi = 0; oi < order.Length; oi++)
            {
                int i = order[oi];
                var n = nodes[i];
                if (!n.hasJoint) { nodes[i] = n; continue; }
                int best = -1; float bestS = float.MaxValue;
                for (int oj = oi + 1; oj < order.Length; oj++)
                {
                    int j = order[oj];
                    var r = nodes[j];   // j, not oj: order is a permutation
                    if (r.chain != n.chain || !r.isChainRef || r.s <= n.s) continue;
                    if (r.s < bestS) { bestS = r.s; best = j; }
                }
                n.nextRefIdx = best;
                n.nextRefS = best >= 0 ? nodes[best].s : -1f;

                int baseRef = -1;
                for (int p = n.parent; p >= 0; p = nodes[p].parent)
                    if (nodes[p].isChainRef) { baseRef = p; break; }
                n.baseRef = baseRef;
                nodes[i] = n;
            }
        }

        /// <summary>Bind segment frames F0 for every node, composed with the
        /// SAME recursion as the per-step pipeline (so bind == pose identity):
        /// root = measured chain-reference frame when the root carries a roll
        /// datum (REQUIRED for root-relative invariance; identity fallback -
        /// caller should warn), children = parent frame carried onto the
        /// child's authored direction with the bind roll datum. Also fills
        /// restRel and the target preconjugations tgtA/tgtB.
        /// Callers must set curDirW = restDirW and curRollDirW = bind roll
        /// direction (chain refs) before calling. Pure, allocation-free.</summary>
        public static void ComposeBindFrames(ChainSimNode[] nodes, int[] order)
        {
            for (int oi = 0; oi < order.Length; oi++)
            {
                int i = order[oi];
                var n = nodes[i];
                if (n.parent < 0)
                {
                    n.bindFrame = math.lengthsq(n.curRollDirW) > 1e-10f
                        ? FrameFromPrimaryRoll(n.restDirW, n.curRollDirW)
                        : quaternion.identity;
                }
                else
                {
                    var p = nodes[n.parent];
                    float3 restDirRel = math.mul(math.inverse(p.bindFrame), n.restDirW);
                    n.bindFrame = math.mul(p.bindFrame,
                        FrameFromPrimaryRoll(restDirRel, OrthoTo(restDirRel)));
                }
                n.restRel = n.parent >= 0
                    ? math.mul(math.inverse(nodes[n.parent].bindFrame), n.bindFrame)
                    : quaternion.identity;
                nodes[i] = n;
            }

            // Target preconjugations. The desired physical relative rotation
            // (identity = spawn pose) realizes the observed relative segment
            // frame Obs = inv(F_p) * F_i via
            //   relT_i = inv(Jw) * F0_p * Obs * inv(F0_i) * Jw
            // (derived from childRot = F_i * inv(F0_i) * sc and
            //  parentRot = F_p * inv(F0_p) * sp with builder-parity frames;
            //  all intermediate spawn factors cancel). Precompute
            //  tgtA = inv(Jw)*F0_p, tgtB = inv(F0_i)*Jw so the per-step cost
            //  is one conjugated compose. Jw = spawnChildRot * frameBRot.
            for (int i = 0; i < nodes.Length; i++)
            {
                var n = nodes[i];
                if (!n.hasJoint || n.parent < 0) { nodes[i] = n; continue; }
                quaternion jw = math.mul(n.spawnChildRot, n.frameBRot);
                n.tgtA = math.mul(math.inverse(jw), nodes[n.parent].bindFrame);
                n.tgtB = math.mul(math.inverse(n.bindFrame), jw);
                nodes[i] = n;
            }

            // Runtime bind constants. Everything below is invariant once the
            // bind frames exist; precomputing removes 2 inverses + 2 muls per
            // node per step from RetargetChains (they were recomputed for
            // every child of the same parent, in both passes).
            for (int i = 0; i < nodes.Length; i++)
            {
                var n = nodes[i];
                n.invBindFrame = math.inverse(n.bindFrame);
                n.restRelInv = math.inverse(n.restRel);
                if (n.parent >= 0)
                {
                    var p = nodes[n.parent];
                    n.restDirRel = math.mul(math.inverse(p.bindFrame), n.restDirW);
                    if (n.isHinge)
                    {
                        // Same quantities the runtime hinge observation used
                        // to derive per step: parent bone axis + rest bend,
                        // in the parent's bind frame. Guard replicates the
                        // old lengthsq gates so behavior is unchanged.
                        float3 axisP = math.mul(math.inverse(p.bindFrame), p.restDirW);
                        float3 bendP = math.mul(math.inverse(p.bindFrame), p.restBendW);
                        n.hingeObsValid = math.lengthsq(axisP) > 1e-8f
                                       && math.lengthsq(bendP) > 1e-8f;
                        if (n.hingeObsValid)
                        {
                            axisP = math.normalizesafe(axisP);
                            bendP -= axisP * math.dot(bendP, axisP);
                            n.hingeAxisRel = axisP;
                            n.hingeBendRel = math.normalizesafe(bendP);
                        }
                    }
                }
                nodes[i] = n;
            }
        }

        /// <summary>Swing from a KNOWN rest direction to the observed one.
        /// Same rotation as ShortestArc(rest, cur) but via a single axis-angle
        /// instead of a basis build: the rest direction is a bind constant,
        /// so the generic quaternion-difference formulation is wasted work.
        /// Degenerate handling matches ShortestArc (identity near-parallel,
        /// OrthoTo pi fallback near-antiparallel).</summary>
        public static quaternion ArcFromRest(float3 rest, float3 cur)
        {
            float3 axis = math.cross(rest, cur);
            float d = math.dot(rest, cur);
            if (d < -0.999999f)
                return quaternion.AxisAngle(OrthoTo(rest), math.PI);
            float l = math.length(axis);
            return l < 1e-8f ? quaternion.identity
                             : quaternion.AxisAngle(axis / l, math.atan2(l, d));
        }

        /// <summary>Root-relative retargeting, parent-first. Per joint node i
        /// (parent p), with F_p the parent's estimated segment frame:
        ///   dirRel    = inv(F_p) * curDir_i                    (bone dir, parent frame)
        ///   swing     = ArcFromRest(restDirRel_i, dirRel)      (pose-only swing)
        ///   twist tau: chain ref -> measured (full frame, rel to parent);
        ///              confident hinge observation -> observed parent roll;
        ///              otherwise inherit the upstream twist (zero-twist
        ///              prediction).
        ///   Obs_i     = swing * restRel_i * TwistLocal(tau)    (rel frame estimate)
        ///   relT_i    = tgtA_i * Obs_i * tgtB_i                (joint target, identity = spawn)
        /// Hinge children never receive a twist target (tau = 0) but observe
        /// their parent's roll from the bend plane (mod pi, confidence =
        /// bend magnitude). Root pose is never commanded. Pure - no Unity
        /// objects, no allocations. Requires topology + bind frames.
        /// 'hingeObservation' gates the swing-only pre-pass: it exists ONLY
        /// to feed parent twist observations from hinge children. When the
        /// rig derives twist from chain references instead, pass false and
        /// the whole first pass is skipped (halves the traversal).
        /// 'invCmdScratch' (length >= nodes.Length, optional): per-pass cache
        /// of inv(cmdRot); when provided, each node's inverse is computed
        /// once instead of once per child. Null = compute per access.</summary>
        public static void RetargetChains(ChainSimNode[] nodes, int[] order, float obsConfMin,
            bool hingeObservation, quaternion[] invCmdScratch = null)
        {
            // Pass A: swing-only absolute frames (twist-free datum), stored in
            // cmdRot as scratch (pass B overwrites). Root = measured frame
            // (covariant) or swing-only fallback; child = parent frame carried
            // by the observed swing. Covariant at every level.
            // Hinge children then observe their PARENT's roll from the bend
            // plane, in the parent's swing frame: there the parent's bone axis
            // is its bind-frame datum (constant), and the observed bend vs the
            // rest bend datum differs ONLY by the parent's twist. Confidence
            // = bend magnitude; comparison folded mod pi (WrapHalf).
            // GATED: this pass exists ONLY to feed the hinge twist
            // observation. When the caller derives twist from chain
            // references (hingeObservation = false) the pass is skipped
            // entirely - pass B recomputes dirRel from the parent's FINAL
            // frame, so it never reads pass A output.
            if (hingeObservation)
            for (int oi = 0; oi < order.Length; oi++)
            {
                int i = order[oi];
                var n = nodes[i];
                n.obsTwist = 0f;
                n.obsConf = 0f;
                n.twist = 0f;

                if (n.parent < 0)
                {
                    n.cmdRot = math.lengthsq(n.curRollDirW) > 1e-10f
                        ? FrameFromPrimaryRoll(n.curDirW, n.curRollDirW)
                        : math.mul(ArcFromRest(n.restDirW, n.curDirW), n.restFrame);
                    if (invCmdScratch != null) invCmdScratch[i] = math.inverse(n.cmdRot);
                    nodes[i] = n;
                    continue;
                }

                var p = nodes[n.parent];
                quaternion pInv = invCmdScratch != null ? invCmdScratch[n.parent]
                                                        : math.inverse(p.cmdRot);
                float3 dirRel = math.mul(pInv, n.curDirW); // parent swing-only frame here
                n.cmdRot = math.mul(math.mul(p.cmdRot, ArcFromRest(n.restDirRel, dirRel)), n.restRel);

                if (n.isHinge && n.hingeObsValid)
                {
                    var pp = nodes[n.parent];
                    // childRel == dirRel (same product); the old code
                    // recomputed inverse(p.cmdRot) for it.
                    float3 curBend = dirRel - n.hingeAxisRel * math.dot(dirRel, n.hingeAxisRel);
                    float c = math.length(curBend);
                    if (c > 1e-5f)
                    {
                        curBend /= c;
                        float tw = WrapHalf(SignedAngle(n.hingeBendRel, curBend, n.hingeAxisRel));
                        if (math.saturate(c) >= pp.obsConf)
                        {
                            pp.obsTwist = tw;
                            pp.obsConf = math.saturate(c);
                            nodes[n.parent] = pp;
                        }
                    }
                }
                if (invCmdScratch != null) invCmdScratch[i] = math.inverse(n.cmdRot);
                nodes[i] = n;
            }

            // Pass B: twist + estimated absolute frames + joint targets,
            // parent-first (a node's parent frame is final when reached).
            for (int oi = 0; oi < order.Length; oi++)
            {
                int i = order[oi];
                var n = nodes[i];

                if (n.parent < 0)
                {
                    // Root: fully measured frame when a roll datum exists
                    // (covariant under rigid source rotation); otherwise the
                    // swing-only fallback (NOT covariant - caller warns).
                    n.cmdRot = math.lengthsq(n.curRollDirW) > 1e-10f
                        ? FrameFromPrimaryRoll(n.curDirW, n.curRollDirW)
                        : math.mul(ArcFromRest(n.restDirW, n.curDirW), n.restFrame);
                    n.twist = 0f;
                    if (invCmdScratch != null) invCmdScratch[i] = math.inverse(n.cmdRot);
                    nodes[i] = n;
                    continue;
                }

                var p = nodes[n.parent];
                quaternion pInv = invCmdScratch != null ? invCmdScratch[n.parent]
                                                        : math.inverse(p.cmdRot);
                float3 dirRel = math.mul(pInv, n.curDirW);
                quaternion swing = ArcFromRest(n.restDirRel, dirRel);

                if (!n.hasJoint)
                {
                    n.cmdRot = math.mul(p.cmdRot, math.mul(swing, n.restRel));
                    n.relT = quaternion.identity;
                    if (invCmdScratch != null) invCmdScratch[i] = math.inverse(n.cmdRot);
                    nodes[i] = n;
                    continue;
                }

                quaternion obs;
                if (n.isChainRef && math.lengthsq(n.curRollDirW) > 1e-10f)
                {
                    // Fully measured relative frame; twist = own-axis
                    // component of the frame delta vs bind.
                    obs = math.mul(pInv,
                        FrameFromPrimaryRoll(n.curDirW, n.curRollDirW));
                    n.twist = TwistAngle(math.mul(obs, n.restRelInv), dirRel);
                }
                else
                {
                    if (n.isHinge)
                    {
                        n.twist = 0f; // hinge: no twist DOF, bend only
                    }
                    else if (n.obsConf >= obsConfMin)
                    {
                        n.twist = n.obsTwist;
                    }
                    else if (n.parent >= 0 && p.chain == n.chain)
                    {
                        n.twist = p.twist; // zero-twist prediction: inherit upstream
                    }
                    else if (n.baseRef >= 0)
                    {
                        n.twist = nodes[n.baseRef].twist;
                    }
                    else
                    {
                        n.twist = 0f;
                    }
                    // Obs = swing * restRel * twist(about own local bone axis).
                    quaternion twistLocal = quaternion.AxisAngle(new float3(1f, 0f, 0f), n.twist);
                    obs = math.mul(math.mul(swing, n.restRel), twistLocal);
                }

                // Joint target in builder-parity joint frames; identity = spawn.
                // Source-root rotation cancels: Obs is parent-relative by
                // construction, and tgtA/tgtB are bind constants.
                n.relT = math.mul(math.mul(n.tgtA, obs), n.tgtB);
                // cmdRot stays a SOURCE SEGMENT frame (observation datum for
                // children + ghost FK); body rotations are derived by the
                // caller: body = cmdRot * inv(bindFrame) * spawnChildRot.
                n.cmdRot = math.mul(p.cmdRot, obs);
                if (invCmdScratch != null) invCmdScratch[i] = math.inverse(n.cmdRot);
                nodes[i] = n;
            }
        }
    }
}
