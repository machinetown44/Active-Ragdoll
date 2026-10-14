using Box3D.Hybrid;
using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;

namespace Box3DArticulation
{
    /// <summary>Reward probe for the biped: computes the locomotion reward
    /// terms discussed for ML-Agents, but with NO agent attached - the
    /// uncontrolled biped (idle waves, no parameter overrides) just walks its
    /// scripted figure while the HUD ticks each term in play mode. Goal:
    /// verify the reward signals measure the right things before training.
    ///
    /// Terms (per step):
    ///  - tracking  : potential-based delta |torsoCOM.xz - target.xz|
    ///  - align     : potential-based delta shortestAngle(yaw, lookAngle)
    ///  - survival  : constant while alive
    ///  - effort    : small punishment on joint motor power |tau|*|omega| (both modes)
    ///  - tilt debt : fills below dotFall threshold, recovers above; r -= k*debt
    ///  - drift     : soft wall beyond driftLimit; hard backstop reset at limit
    /// Falls (tilt debt >= 1 or torso below floor fall height) reset the gauge.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class RewardProbe : MonoBehaviour
    {
        [Header("References")]
        [SerializeField, Tooltip("The PinnedBipedDemo owning the torso. Auto-found if empty.")]
        private PinnedBipedDemo demo;
        [SerializeField, Tooltip("The roaming target. Auto-found if empty.")]
        private LocomotionTarget target;

        [Header("Torso COM readout")]
        [SerializeField, Tooltip("Body-local point treated as the torso center of mass.")]
        private float3 comLocal = new float3(0f, 0f, 0f);
        /// <summary>Body-local COM offset (obs + reward read it).</summary>
        public float3 ComLocal => comLocal;

        [Header("Minimal mode (pursuit core)")]
        [SerializeField, Tooltip("ON: strip the stack to TWO terms - velocity match against a pursuit-corrected desired velocity (target velocity + homing bias toward the target) and alignment. No gait/balance/energy/debt/height terms. Deviation abort (agent) is the only early exit.")]
        private bool minimalMode = true;
        [SerializeField, Tooltip("Homing gain (1/s): desired velocity = target velocity + gain * (target - COM). 1.0: 0.5 m behind means +0.5 m/s homing component on top of the target's motion.")]
        private float pursuitGain = 1f;
        [SerializeField, Tooltip("Desired-velocity error (m/s) at which the velocity-match payout reaches zero.")]
        private float velTol = 1f;

        [Header("Tracking")]
        [SerializeField] private float trackingK = 3f;

        [Header("Alignment")]
        [SerializeField] private float alignK = 1f;

        [Header("Survival")]
        [SerializeField, Tooltip("Per-second alive bonus. Modest on purpose: it is behavior-agnostic baseline; too high and standing still out-earns risky learning.")]
        private float survival = 1f;

        [Header("Effort")]
        [SerializeField, Tooltip("Punishment on summed joint motor power: -k * saturate(|tau|*|omega_rel| / effortRef) over the rig's 8 servo joints, low-passed. Static holds are free (omega ~ 0), so the gravity baseline of standing costs nothing; flails are taxed. Texture term - keep k small.")]
        private float effortK = 0.3f;
        [SerializeField, Tooltip("Joint power (W, summed over all joints) that saturates the effort punishment.")]
        private float effortRef = 60f;
        [SerializeField, Range(0f, 1f), Tooltip("Leak factor for the effort low-pass (per physics step fraction of the new sample). Lower = smoother. MUST be <= 1: >1 makes lerp overshoot, the smoothed value can go negative and flip the term into a positive reward.")]
        private float energySmooth = 0.1f;

        [Header("Tilt debt gauge")]
        [SerializeField, Range(0f, 1f), Tooltip("dot(up, worldUp) below which debt fills.")]
        private float dotFall = 0.90f;
        [SerializeField, Tooltip("Debt fill rate at worst tilt (dot = -1), 1/s.")]
        private float debtFillRate = 0.5f;
        [SerializeField, Tooltip("Debt recovery rate at full upright (dot = 1), 1/s.")]
        private float debtRecoverRate = 2f;
        [SerializeField] private float tiltDebtK = 1f; // steep: debt punished hard so wobbling states read as costly, not mild

        [Header("Toppling rate (angular velocity toward the ground)")]
        [SerializeField, Tooltip("Punishment on toppling RATE: -k * saturate(-d(up.y)/dt / ref). Fires the moment the torso starts going over - a predictive gradient the position debt fills too slowly to deliver. Symmetric wobble averages ~0 (like the energy term); only net tipping is taxed. Debt stays as the fall detector and prone-state punishment.")]
        private float tipK = 1f;
        [SerializeField, Tooltip("Tipping rate (up.y per second) at which the punishment saturates. 0.5/s: from upright, ~2 s of sustained tip is full penalty; a stumble hits it instantly.")]
        private float tipRateRef = 0.5f;

        [Header("Drift soft wall")]
        [SerializeField, Tooltip("COM distance from target beyond which drift punishment starts (m).")]
        private float driftLimit = 0.5f;
        [SerializeField] private float driftK = 2f;
        [SerializeField, Tooltip("Velocity mismatch penalty: -k * |vCom - vTarget|^2 (planar). Static target: any COM speed costs. 0.1: 1 m/s wander = -0.1/s.")]
        private float velMatchK = 5f;
        [Header("Standing tall")]
        [SerializeField, Tooltip("Standing-tall reward: heightK * clamp(com.y / standHeight, 0, 1) per second. Full stand pays heightK/s, crouch pays proportionally less.")]
        private float heightK = 1f;
        [SerializeField, Tooltip("Reference standing COM height (m) for the height term.")]
        private float standHeight = 1.1f;

        [Header("Gait (periodic stride detection)")]
        [SerializeField, Tooltip("Payout at full gait: k * min(A_L, A_R, liftL, liftR) * antiPhase * gates, per second. Amplitudes in meters of locked excursion.")]
        private float gaitK = 15f;
        [SerializeField, Tooltip("Reference stride frequency (Hz) the lock-in correlator is tuned to. Only periodic foot motion near this frequency pays.")]
        private float strideFreq = 1f;
        [SerializeField, Tooltip("Lock-in window (s): leak time constant of the correlator. MUST hold >= 2 stride periods (>= 1.3 s at 1.5 Hz) or the correlator cannot discriminate periodic motion from noise.")]
        private float gaitWindow = 2f;
        [SerializeField, Tooltip("COM planar speed (m/s) at full gait payout. Scales linearly below - dancing in place earns nothing.")]
        private float gaitSpeedGate = 0.5f;

        [Header("Lateral COM balance (dynamic) - COM projection centered over the stance")]
        [SerializeField, Tooltip("Payout at center: k * saturate(1 - |offset|/margin) * gates, per second. offset = COM projection vs feet midpoint, perpendicular to travel. Symmetric sway averages ~0; only biased lean is punished.")]
        private float balanceK = 1.5f;
        [SerializeField, Tooltip("Lift differential (m) at which the balance target is fully the stance foot (single stance). Below it, target blends to feet midpoint (double stance).")]
        private float liftRef = 0.08f;
        [SerializeField, Tooltip("Lateral offset (m) at which the balance payout reaches zero.")]
        private float balanceMargin = 0.3f;
        [SerializeField, Tooltip("Stance width (m, torso frame, left minus right) where the balance payout starts fading - wider than a walking stance pays less, killing the static splits exploit (wide base = max balance reward, zero risk).")]
        private float stanceWidthRef = 0.25f;
        [SerializeField, Tooltip("Stance width excess (m) at which the balance payout reaches zero.")]
        private float stanceWidthMargin = 0.2f;

        [Header("Leg crossing guard - feet must stay on their own side of the torso")]
        [SerializeField, Tooltip("Punishment rate when the left foot passes right of the right foot (torso frame), saturating over crossMargin. Ungated by speed: turn-in-place crossings trip too.")]
        private float crossK = 0.5f;
        [SerializeField, Tooltip("Minimum lateral separation (m), left minus right, before the penalty ramps in.")]
        private float crossTol = 0.03f;
        [SerializeField, Tooltip("Separation deficit (m) at full penalty.")]
        private float crossMargin = 0.15f;

        [Header("Fall reset")]
        [SerializeField, Tooltip("Torso COM below this world height counts as fallen (m).")]
        private float fallHeight = 0.5f;

        [SerializeField,
         Tooltip(
             "Misery mode: on fall, do NOT reset gauges - the fallen rig keeps accruing penalties (debt pegged, height 0, no gait) until MaxStep. Escape ticket is gone; the policy must avoid the fall outright.")]
        private bool resetOnFall = false;

        [Header("Imitation (ghost pose match - reduced rig rep, DeepMimic exponential form)")]
        [SerializeField, Tooltip("Overall imitation scale. Terms: pose W*exp(-sharp*e^2) + tip-velocity W*exp(-sharp*e^2) + root-orientation W*exp(-sharp*e^2), each bounded [0,W].")]
        private float imitationK = 4f;
        [SerializeField, Tooltip("Pose term weight (DeepMimic w_p ~ 0.65).")]
        private float imitPoseW = 0.5f;
        [SerializeField, Tooltip("Tip-velocity term weight (DeepMimic w_v).")]
        private float imitVelW = 0.15f;
        [SerializeField, Tooltip("Root-orientation term weight (replaces the align term - DeepMimic's pose includes root rotation).")]
        private float imitRootW = 0.1f;
        [SerializeField, Tooltip("Pose sharpness k in exp(-k*e^2). e = weighted mean(1 - dot) of anchor->joint/tip unit directions; 15 makes a decent match (e=0.2) pay ~50%.")]
        private float imitPoseSharp = 15f;
        [SerializeField, Tooltip("Tip-velocity sharpness k. e in m/s, anchor-relative tip speeds; 1.5 makes a 1 m/s error pay ~22%.")]
        private float imitVelSharp = 1.5f;
        [SerializeField, Tooltip("Root-orientation sharpness k (same units as pose).")]
        private float imitRootSharp = 15f;
        [SerializeField, Tooltip("Per-limb pose/velocity weights: legs vs arms (DeepMimic joint weighting).")]
        private float legWeight = 1f;
        [SerializeField, Tooltip("Arm weight (see legWeight).")]
        private float armWeight = 0.6f;
        [SerializeField, Tooltip("Fixed torso-frame anchor for the rig side of the reduced representation. (0,0,0) = body origin - a fixed point inside the torso, no COM wobble. Only directions are compared, so anatomical equivalence with the ghost's Hips is not required.")]
        private Vector3 torsoAnchorLocal = Vector3.zero;
        [SerializeField, Tooltip("Sustained pose-fail abort (DeepMimic pose-error termination): mean pose error above this ends the episode.")]
        private float poseAbortError = 0.5f;
        [SerializeField, Tooltip("Pose error must stay above poseAbortError this long (s) before the abort fires - sustained, not instantaneous: no quick destructive exit.")]
        private float poseAbortTime = 1f;
        [SerializeField, Tooltip("Wired by BipedBatch: the kinematic ghost the rig should look like.")]
        private ImitationReference imitationRef;

        // --- reward state ---
        private Box3DBody m_torso;
        private float m_prevDist, m_prevUpright;
        private bool m_misery; // fallen-but-alive: fall flag fires once, gauges keep running
        private float m_effortSm;        // low-passed summed joint motor power (W)
        private readonly List<BackboneWaveProbe> m_limbs = new List<BackboneWaveProbe>(); // effort metric sources
        private float m_debt;
        private float m_episodeReward;

        // --- gait lock-in correlators (per foot, leaky demodulation at strideFreq) ---
        private struct GaitCorrelator { public float s, c; public float sy, cy; } // horizontal + vertical lock-in per foot
        private GaitCorrelator m_gaitL, m_gaitR;
        private float m_gaitAnti;    // 0..1: 1 = feet locked in anti-phase
        private float m_gaitT;       // correlator reference clock (rad/s = 2pi*strideFreq)
        private Box3DBody m_footL, m_footR;
        private float m_episodeTime;
        [HideInInspector] public float rImit;
        // Imitation error readouts (weighted means, per-step, inspector debug).
        [HideInInspector] public float ePose, eVel, eRoot;
        /// Sustained pose-fail gauge: true once poseAbortTime of accumulated
        /// violation (ePose > poseAbortError) has built up. Decays on recovery.
        public bool PoseFail => m_poseViol >= poseAbortTime;
        /// Episode clock since the last reset (s) - the agent's abort grace uses it.
        public float EpisodeTime => m_episodeTime;
        private float m_poseViol;
        private Animator m_ghostAnim;                      // resolved lazily off the ghost
        private Transform m_ghostHips;                     // cached Hips: GetBoneTransform re-resolves the humanoid map on EVERY call - never call it per step
        private readonly Transform[] m_ghostBones = new Transform[8]; // cached per-bone (joint,tip)x4
        private readonly float3[] m_prevTipsRig = new float3[4];   // tip velocity finite differences
        private readonly float3[] m_prevTipsGhost = new float3[4]; // (per limb: index i, not 2i+1)
        private float3 m_prevAnchorRig, m_prevAnchorG;
        private bool m_velInit; // velocity term silent on the first step after reset
        // Index-aligned with m_limbs (LegL, LegR, ArmL, ArmR).
        private static readonly HumanBodyBones[] kJointBone =
            { HumanBodyBones.LeftLowerLeg, HumanBodyBones.RightLowerLeg, HumanBodyBones.LeftLowerArm, HumanBodyBones.RightLowerArm };
        private static readonly HumanBodyBones[] kTipBone =
            { HumanBodyBones.LeftFoot, HumanBodyBones.RightFoot, HumanBodyBones.LeftHand, HumanBodyBones.RightHand };
        private int m_falls;
        private Vector3 m_spawnPos;

        // per-step term readouts (inspector, debug view)
        [HideInInspector] public float rTrack, rAlign, rSurvive, rEnergy, rTilt, rTip, rDrift, rVel, rHeight, rGait, rBalance, rCross;
        [HideInInspector] public float curDist, curUpright, curDrift;

        public float EpisodeReward => m_episodeReward;
        public float Debt => m_debt;

        private void OnEnable()
        {
            Acquire();
        }

        // Re-acquire references. Runs until the torso is valid: component
        // OnEnable order across GameObjects is not guaranteed, so the demo
        // may not have created its torso when this probe first enables.
        private void Acquire()
        {
            if (demo == null) demo = FindAnyObjectByType<PinnedBipedDemo>();
            if (target == null) target = FindAnyObjectByType<LocomotionTarget>();
            if (demo != null && (m_torso == null || !m_torso.Body.IsValid)) m_torso = demo.Torso; // re-resolve after a rig rebuild too
            if (m_torso != null && m_torso.Body.IsValid && target != null)
            {
                m_spawnPos = m_torso.transform.position;
                ResetGauges();
            }
        }

        private void ResetGauges()
        {
            // Called from the agent's OnEpisodeBegin, which can fire before the
            // probe has acquired a torso (Academy's forced first reset happens
            // before the demo's Awake may have created it). Skip safely; the
            // per-step path re-derives everything once references are valid.
            if (m_torso == null || !m_torso.Body.IsValid || target == null) return;
            RebuildRigCache();
            float3 com = TorsoCom();
            float3 tPos = target.Position;
            m_prevDist = math.length(new float2(com.x - tPos.x, com.z - tPos.z));
            m_prevUpright = Upright();
            m_debt = 0f;
            m_misery = false;
            m_effortSm = 0f;
            m_gaitL = m_gaitR = default; // correlators start empty: no phantom stride reward after a reset
            m_gaitAnti = 0f;
            m_gaitT = 0f;
            m_episodeReward = 0f;
            m_episodeTime = 0f;
            rImit = 0f;
            ePose = eVel = eRoot = 0f;
            m_poseViol = 0f;
            m_velInit = false; // velocity term silent on the first step after reset
            for (int i = 0; i < 4; i++) m_prevTipsRig[i] = m_prevTipsGhost[i] = new float3(float.PositiveInfinity, 0f, 0f);
        }

        private void FixedUpdate()
        {
            if (m_torso == null || !m_torso.Body.IsValid || target == null)
            {
                Acquire(); // demo/target may come alive later in the frame order
                if (m_torso == null || !m_torso.Body.IsValid || target == null) return;
            }
            float dt = Time.fixedDeltaTime;

            // --- torso pose ---
            float3 com = TorsoCom();
            float3 tPos = target.Position;
            float upright = Upright();
            float3 vCom = m_torso.Body.GetLinearVelocity();

            // Imitation runs in BOTH modes: matching the reference pose IS the
            // task, not optional shaping. Computed once, added to each sum.
            UpdateImitation(dt);

            // --- minimal mode: DeepMimic core ---
            if (minimalMode)
            {
                // DeepMimic structure: imitation (3 exponential terms) + ONE task
                // term (potential-based target tracking) + effort insurance at half
                // weight. The pursuit velocity-match and align terms are gone -
                // root heading is inside the imitation root-orientation term, and
                // the motion spec is the ghost itself.
                float distM = math.length(new float2(com.x - tPos.x, com.z - tPos.z));
                rTrack = trackingK * (m_prevDist - distM);
                m_prevDist = distM;
                rVel = rAlign = rSurvive = rTilt = rTip = rDrift = rHeight = rGait = rBalance = rCross = 0f;
                rEnergy = EffortTerm(dt); // effort is mode-agnostic: no free flails in minimal either
                curDist = math.distance(com, tPos + new float3(0f, standHeight, 0f)); // 3D deviation: planar + vertical in one number
                curUpright = upright;
                curDrift = 0f;
                LastStepReward = rTrack + 0.5f * rEnergy + rImit;
                m_episodeReward += LastStepReward;
                m_episodeTime += dt;
                m_debt = 0f;
                // Fall detection (minimal mode): the debt gauge is off, so the
                // fall flag comes from height alone. The agent ends the episode
                // on it (DeepMimic fail) - no misery accrual in this mode.
                if (com.y < fallHeight) m_fallFlag = true;
                return;
            }

            // 1. tracking: potential delta toward the roaming point
            float dist = math.length(new float2(com.x - tPos.x, com.z - tPos.z));
            rTrack = trackingK * (m_prevDist - dist);
            m_prevDist = dist;

            // 2. alignment: ABSOLUTE pose quality, not error-reduction shaping.
            // The potential-delta form paid for any error reduction, including
            // a 180-deg sweep THROUGH the anti-heading (the ±pi wrap jump
            // paid a 2·pi spike) - and the resulting spin destabilizes the
            // rig into episode-ending falls. cos(err) pays for facing the
            // look heading every step: 1 aligned, -1 anti-aligned, and no
            // wrap discontinuity to exploit. Scaled to [-1,1] * alignK.
            float err = AlignError();
            rAlign = alignK * math.cos(err);

            // 3. survival
            rSurvive = survival * dt;

            // 4. effort: -k * saturate(sum |tau|*|omega_rel| / ref), low-passed.
            // Measured at the 8 servo joints (motor torque x relative joint
            // speed) - a direct metabolic proxy. Static holds are free (omega
            // ~ 0): the gravity baseline of standing costs nothing, which a
            // raw torque penalty cannot do (it would teach lying flat).
            rEnergy = EffortTerm(dt);

            // 5. tilt debt: graded gauge, fills below threshold, recovers above
            float dDot = dotFall - upright;               // >0 when tilting past threshold
            if (dDot > 0f) m_debt += debtFillRate * dDot / math.max(dotFall, 1e-3f) * dt;
            else m_debt -= debtRecoverRate * (-dDot) / math.max(1f - dotFall, 1e-3f) * dt;
            m_debt = math.clamp(m_debt, 0f, 1f);
            rTilt = -tiltDebtK * m_debt;

            // 5b. toppling rate: -k * saturate(-d(up.y)/dt / ref). The debt
            // gauge is POSITION - it fills over tenths of a second, so the
            // gradient during the decisive toppling moment is weak. The rate
            // term fires instantly when the torso starts going over, giving
            // PPO a signal in the exact steps where avoidance is still
            // possible. Only negative rate is taxed: recovery (up.y rising)
            // is never punished, and symmetric wobble averages ~0.
            float dUp = (upright - m_prevUpright) / dt;
            m_prevUpright = upright;
            rTip = -tipK * math.saturate(-dUp / math.max(tipRateRef, 1e-3f));

            // 6. drift soft wall
            float drift = math.max(0f, dist - driftLimit);
            rDrift = -driftK * drift;

            // 7. velocity match: COM speed vs target speed, squared. Static
            // target (stage 0) -> any COM speed costs, killing the jitter/
            // orbit exploits the potential-based terms are blind to. Moving
            // target -> rewards staying in the target's frame.
            float3 vTar = target.Velocity;
            float2 dv = new float2(vCom.x - vTar.x, vCom.z - vTar.z); // planar only
            rVel = -velMatchK * math.lengthsq(dv);

            // 8. standing tall: continuous gradient toward full height, the
            // per-second signal the debt/fall pair lacks between them. Falls
            // to 0 near the ground; full stand pays heightK/s.
            rHeight = heightK * math.clamp(com.y / math.max(standHeight, 1e-3f), 0f, 1f);

            // 9. gait: periodic stride detection. Per foot, demodulate the foot
            // excursion along the travel axis (torso fwd XZ, relative to torso
            // COM) at the reference stride frequency with a leaky window. Only
            // periodic motion near strideFreq accumulates amplitude; flail/
            // drift/vibration average out. Payout = k * min(A_L, A_R) - both
            // feet must stride - times an anti-phase factor (in-phase hopping
            // pays ~0), times speed and uprightness gates.
            UpdateGait(com, vCom, upright, dt);

        float step = rTrack + rAlign + rSurvive + rEnergy + rTilt + rTip + rDrift + rVel + rHeight + rGait + rBalance + rCross + rImit;
        m_episodeReward += step;
        m_episodeTime += dt;
        curDist = math.distance(com, tPos + new float3(0f, standHeight, 0f)); // 3D deviation: planar + vertical in one number
        curUpright = upright;
        curDrift = drift;
        LastStepReward = step;

        // --- fall detection ---
        // In misery mode (resetOnFall = false) the gauge keeps running while
        // fallen: debt stays pegged at 1, height pays 0, gait gates to 0 - the
        // stream itself is the punishment, and a ONE-TIME flag tells the agent
        // the fall happened (no repeat penalties on every floor step).
        bool fallen = !m_misery && (m_debt >= 1f || com.y < fallHeight);
        if (fallen)
        {
        m_falls++;
        m_fallFlag = true;
        if (resetOnFall) ResetGauges();
        else m_misery = true; // stay fallen: keep accruing, flag only once
        }
        }

        /// Imitation block, DeepMimic form on the reduced rig representation:
        /// three bounded exponential terms, all scale-free (direction/dot based).
        /// - pose:   anchor->joint + anchor->tip unit directions, per-limb weighted
        /// - vel:    anchor-relative tip velocities (finite differences), so root
        ///           translation cancels and in-place/traveling clips are equivalent
        /// - root:   torso up+forward vs ghost pelvis orientation (DeepMimic's pose
        ///           includes root rotation)
        /// Anchors: fixed torso-frame point (no COM wobble) vs ghost Hips. Also
        /// accumulates the sustained pose-fail gauge for the DeepMimic-style abort.
        private void UpdateImitation(float dt)
        {
            rImit = 0f;
            ePose = eVel = eRoot = 0f;
            if (imitationK <= 0f || imitationRef == null) return;
            if (!ResolveGhost()) return; // prefab without an Animator - term silent
            Transform hips = m_ghostHips;
            if (hips == null) return; // avatar without a mapped humanoid skeleton

            var tb = m_torso.Body;
            float3 anchorRig = tb.Position + math.mul(tb.Rotation, (float3)torsoAnchorLocal);
            float3 anchorG = (float3)hips.position;
            float3 upR = math.mul(tb.Rotation, new float3(0f, 1f, 0f));
            float3 fwdR = math.mul(tb.Rotation, new float3(0f, 0f, 1f)); // rig forward = local +Z (authored facing; Unity-humanoid fwdG is +Z too)
            float3 upG = (float3)hips.up;
            float3 fwdG = (float3)hips.forward;
            eRoot = 0.5f * (1f - math.dot(upR, upG)) + 0.5f * (1f - math.dot(fwdR, fwdG));

            // Finite differences: anchors and tips, one step back. Velocities are
            // ANCHOR-RELATIVE on both sides - root translation cancels entirely.
            float3 vAnchorRig = m_velInit ? (anchorRig - m_prevAnchorRig) / dt : 0f;
            float3 vAnchorG = m_velInit ? (anchorG - m_prevAnchorG) / dt : 0f;

            if (m_limbs.Count == 0) ResolveLimbs();
            float errSum = 0f, wSum = 0f, velSum = 0f, velWSum = 0f;
            for (int i = 0; i < m_limbs.Count && i < 4; i++)
            {
                var limb = m_limbs[i];
                if (limb == null || !limb.ChainValid) continue;
                float w = i < 2 ? legWeight : armWeight;
                int j = 2 * i, t = 2 * i + 1;
                Transform bJoint = m_ghostBones[j] != null ? m_ghostBones[j] : (m_ghostBones[j] = m_ghostAnim.GetBoneTransform(kJointBone[i]));
                Transform bTip = m_ghostBones[t] != null ? m_ghostBones[t] : (m_ghostBones[t] = m_ghostAnim.GetBoneTransform(kTipBone[i]));
                if (bJoint != null)
                {
                    float3 g = (float3)bJoint.position - anchorG;
                    float3 r = limb.JointWorld - anchorRig;
                    if (math.lengthsq(g) > 1e-8f && math.lengthsq(r) > 1e-8f)
                    {
                        errSum += w * (1f - math.dot(math.normalizesafe(r), math.normalizesafe(g)));
                        wSum += w;
                    }
                }
                if (bTip != null)
                {
                    float3 gPos = (float3)bTip.position;
                    float3 g = gPos - anchorG;
                    float3 r = limb.TipWorld - anchorRig;
                    if (math.lengthsq(g) > 1e-8f && math.lengthsq(r) > 1e-8f)
                    {
                        errSum += w * (1f - math.dot(math.normalizesafe(r), math.normalizesafe(g)));
                        wSum += w;
                    }
                    if (m_velInit)
                    {
                        // tip velocities relative to each side's anchor (m/s)
                        float3 vR = (limb.TipWorld - m_prevTipsRig[i]) / dt - vAnchorRig;
                        float3 vG = (gPos - m_prevTipsGhost[i]) / dt - vAnchorG;
                        velSum += w * math.length(vR - vG);
                        velWSum += w;
                    }
                    m_prevTipsGhost[i] = gPos;
                    m_prevTipsRig[i] = limb.TipWorld;
                }
            }
            m_prevAnchorRig = anchorRig;
            m_prevAnchorG = anchorG;
            m_velInit = true;

            ePose = wSum > 1e-6f ? errSum / wSum : 0f;
            eVel = velWSum > 1e-6f ? velSum / velWSum : 0f;

            rImit = imitationK * (
                imitPoseW * math.exp(-imitPoseSharp * ePose * ePose)
                + imitVelW * math.exp(-imitVelSharp * eVel * eVel)
                + imitRootW * math.exp(-imitRootSharp * eRoot * eRoot));

            // Sustained pose-fail gauge: accumulates above the abort threshold,
            // decays at 2x on recovery so transient stumbles never trigger it.
            if (ePose > poseAbortError) m_poseViol += dt;
            else m_poseViol = math.max(0f, m_poseViol - 2f * dt);
        }

        /// Effort term, shared by both modes: low-passed summed joint motor
        /// power. Limb references resolve lazily (rig is rebuilt at reset).
        private float EffortTerm(float dt)
        {
            if (m_limbs.Count == 0) ResolveLimbs();
            float p = 0f;
            for (int i = 0; i < m_limbs.Count; i++)
            {
                var limb = m_limbs[i];
                if (limb != null) p += limb.JointPower();
            }
            m_effortSm = math.lerp(m_effortSm, p, energySmooth);
            return -effortK * math.saturate(m_effortSm / math.max(effortRef, 1e-3f));
        }

        private void ResolveLimbs()
        {
            m_limbs.Clear();
            if (demo == null) return;
            if (demo.LegL != null) m_limbs.Add(demo.LegL);
            if (demo.LegR != null) m_limbs.Add(demo.LegR);
            if (demo.ArmL != null) m_limbs.Add(demo.ArmL);
            if (demo.ArmR != null) m_limbs.Add(demo.ArmR);
        }

        /// Reward sum for the most recent physics step (agent pipes into AddReward).
        public float LastStepReward { get; private set; }        /// True once since the last consume - a fall happened.
        public bool ConsumeFall()
        {
        bool f = m_fallFlag;
        m_fallFlag = false;
        return f;
        }
        /// Misery mode active (resetOnFall = false): fallen rig stays in the
        /// episode, accrual continues, ConsumeFall fires once per fall.
        public bool MiseryMode => !resetOnFall;
        private bool m_fallFlag;
        /// The tracked torso body (agent observations).
        public Box3DBody TorsoBody => m_torso;
        /// The roaming target this probe tracks.
        public LocomotionTarget Target => target;
        /// Current tilt debt (agent observation + gauge).
        /// Deviation abort metric: 3D distance from the COM to the EXPECTED
        /// COM point (target position raised by standHeight). ONE distance for
        /// both axes: standing beside the target is planar-only, a jumper adds
        /// height^2, a fallen rig adds (standHeight - com.y)^2 - all three
        /// escape routes end the episode through the same number.
        public float TargetDist => curDist;

        /// Aim inputs for the agent's yaw assist.
        public bool MinimalMode => minimalMode;
        public float PursuitGain => pursuitGain;
        public Box3DBody TorsoBodyPublic => m_torso;

        /// <summary>Ghost-side limb pose for observations (the DeepMimic "goal"
        /// block): anchor (ghost Hips) + joint + tip world positions. Independent
        /// of whether the imitation reward is enabled - the observation side may
        /// poll this even with imitationK = 0. Bone mapping identical to the
        /// reward path (kJointBone / kTipBone).</summary>
        public struct GhostLimbPose
        {
            public float3 AnchorW, JointW, TipW;
            public bool Valid;
        }

        /// <summary>Resolve + read the ghost limb pose. Invalid (all zeros) when
        /// no ghost is wired, the prefab has no Animator, or the avatar lacks a
        /// mapped humanoid skeleton. Cheap: cached transforms only.</summary>
        public GhostLimbPose GetGhostLimbPose(int i)
        {
            var s = default(GhostLimbPose);
            if (imitationRef == null || i < 0 || i > 3) return s;
            if (!ResolveGhost()) return s;
            int j = 2 * i, t = 2 * i + 1;
            Transform bJoint = m_ghostBones[j] != null ? m_ghostBones[j] : (m_ghostBones[j] = m_ghostAnim.GetBoneTransform(kJointBone[i]));
            Transform bTip = m_ghostBones[t] != null ? m_ghostBones[t] : (m_ghostBones[t] = m_ghostAnim.GetBoneTransform(kTipBone[i]));
            Transform hips = m_ghostHips; // cached - no per-call humanoid map lookup
            if (bJoint == null || bTip == null || hips == null) return s;
            s.AnchorW = hips.position;
            s.JointW = bJoint.position;
            s.TipW = bTip.position;
            s.Valid = true;
            return s;
        }

        // Shared lazy ghost-animator resolve for the reward and observation
        // paths. GetBoneTransform is a humanoid map re-resolution PER CALL -
        // resolve everything ONCE here: animator, hips, then bones fill their
        // own caches on first use. If hips is missing the mapping is missing
        // for the avatar's lifetime; both consumers null-check per use.
        private bool ResolveGhost()
        {
            if (m_ghostAnim != null) return true;
            m_ghostAnim = imitationRef.GetComponentInChildren<Animator>();
            if (m_ghostAnim == null) return false; // prefab without an Animator
            m_ghostHips = m_ghostAnim.GetBoneTransform(HumanBodyBones.Hips);
            return true;
        }
        public float GaitAmpL => GaitAmp(m_gaitL);  // inspector gait diagnostics
        public float GaitAmpR => GaitAmp(m_gaitR);
        public float GaitLiftL => GaitAmpY(m_gaitL);
        public float GaitLiftR => GaitAmpY(m_gaitR);
        public float GaitAnti => m_gaitAnti;
        /// Streams the last step's reward through the given sink (agent.AddReward).
        public void RewardInto(System.Action<float> sink) => sink(LastStepReward);
        /// Public reset hook for the agent's OnEpisodeBegin.
        public void ResetGaugesPublic() => ResetGauges();

        /// Curriculum: loosen the tilt debt early so recovery is learnable.
        /// d=0: forgiving fill (long time-to-fall); d=1: tight (training endpoint).
        public void ApplyDifficulty(float d)
        {
            debtFillRate = math.lerp(2f, 0.6f, math.clamp(d, 0f, 1f)); // stage 0: ~2s worst-tilt to fall
        }

        // Torso COM in world space (COM readout = body position + rotated local point).
        private float3 TorsoCom()
        {
            var b = m_torso.Body;
            return b.Position + math.mul(b.Rotation, comLocal);
        }

        // --- whole-rig COM (mass-weighted over every dynamic body) ---
        // Cached once per episode: the rig is rebuilt at reset, so children
        // scan only there, never per step. Per-step cost is just 9 body reads.
        private readonly List<Box3DBody> m_rigBodies = new List<Box3DBody>();

        private void RebuildRigCache()
        {
            m_rigBodies.Clear();
            if (demo == null) return;
            demo.GetComponentsInChildren(true, m_rigBodies);
            for (int i = m_rigBodies.Count - 1; i >= 0; i--)
            {
                var body = m_rigBodies[i];
                if (body.BodyType != Box3D.Hybrid.Box3DBodyType.Dynamic || !body.Body.IsValid) m_rigBodies.RemoveAt(i);
            }
            // Gait feet: re-resolve with the rig (probes are rebuilt on reset too).
            if (demo.LegL != null) m_footL = demo.LegL.DistalBody;
            if (demo.LegR != null) m_footR = demo.LegR.DistalBody;
        }

        /// Gait correlators: leaky lock-in demodulation of foot excursion along
        /// the travel axis, per foot; amplitude, anti-phase factor, then gates.
        private void UpdateGait(float3 com, float3 vCom, float upright, float dt)
        {
            if (m_footL == null || !m_footL.Body.IsValid || m_footR == null || !m_footR.Body.IsValid) { rGait = 0f; rBalance = 0f; rCross = 0f; return; }
            // Torso forward (XZ) for the crossing guard's lateral axis.
            float3 tFwd = math.mul(m_torso.Body.Rotation, new float3(0f, 0f, 1f)); // torso forward = local +Z
            tFwd.y = 0f;
            float tLen = math.length(tFwd);
            float3 torsoFwdXz = tLen > 1e-4f ? tFwd / tLen : new float3(0f, 0f, 1f);
            // Travel axis: the COM's actual movement direction (XZ), NOT torso
            // forward - look and move are deliberately decoupled (the align
            // reward pays facing the target's look heading while walking along
            // MoveDirW). Projecting the stride on torso-forward would tax every
            // non-aligned walk. Backwards/reversing stays robust: the lock-in
            // pays amplitude (sign-blind), so only the axis alignment matters.
            float vLen = math.length(new float2(vCom.x, vCom.z));
            float3 travel;
            if (vLen > 0.05f)
            {
                travel = new float3(vCom.x, 0f, vCom.z) / vLen;
            }
            else
            {
                // Below the gate the payout is ~0 anyway; fall back to torso
                // forward so the correlator keeps a stable reference instead of
                // jittering with near-zero velocity noise.
                float3 fwd = math.mul(m_torso.Body.Rotation, new float3(0f, 0f, 1f)); // torso forward = local +Z
                fwd.y = 0f;
                float fLen = math.length(fwd);
                travel = fLen > 1e-4f ? fwd / fLen : new float3(0f, 0f, 1f);
            }

            m_gaitT += 2f * math.PI * strideFreq * dt;
            float sn = math.sin(m_gaitT), cs = math.cos(m_gaitT);
            float decay = 1f - math.clamp(dt / math.max(gaitWindow, 1e-2f), 0f, 0.9f);

            Demodulate(ref m_gaitL, m_footL, com, travel, sn, cs, decay, dt);
            Demodulate(ref m_gaitR, m_footR, com, travel, sn, cs, decay, dt);
            m_gaitAnti = AntiPhase(m_gaitL, m_gaitR);

            // Gates: speed (dance-in-place earns nothing) x uprightness (the
            // same smoothstep as the yaw assist: 0 below ~14 deg from horizontal).
            float speedGate = math.saturate(math.length(new float2(vCom.x, vCom.z)) / math.max(gaitSpeedGate, 1e-3f));
            float upGate = math.saturate((upright - 0.25f) / 0.5f);
            // Foot lift coupling (vertical lock-in): stride amplitude is min'd
            // with the locked vertical excursion relative to the COM - torso bob
            // cancels, so only real clearance counts. min per foot, then across
            // feet: dragging EITHER foot starves the term, same anti-hack logic
            // as the L/R travel min. Without it, a floor-shuffling fore-aft
            // scoot collects full stride credit.
            float amp = math.min(
                math.min(GaitAmp(m_gaitL), GaitAmpY(m_gaitL)),
                math.min(GaitAmp(m_gaitR), GaitAmpY(m_gaitR)));
            rGait = gaitK * amp * m_gaitAnti * speedGate * upGate;

            // Lateral COM balance (term 10), STANCE-ANCHORED: walking is an
            // inverted pendulum - the COM must shift over the PLANTED foot
            // during single stance, or the rig topples onto the swing side.
            // A midpoint reference (old form) taxes exactly that sway. Instead:
            // the lower foot (relative to COM, so bob cancels) is the stance;
            // the balance target blends from midpoint (double stance, COM
            // centered is correct) to the planted foot (single stance, shift is
            // correct), weighted by the lift differential. An aimless lean with
            // no lift differential still pays ~0 - so only gait-consistent
            // weight shift earns, and lift is reinforced rather than fought.
            float3 side = math.normalizesafe(math.cross(new float3(0f, 1f, 0f), travel));
            float3 pL = (float3)m_footL.Body.GetPosition();
            float3 pR = (float3)m_footR.Body.GetPosition();
            float yL = pL.y - com.y, yR = pR.y - com.y;
            bool lIsStance = yL <= yR;
            float liftDiff = (lIsStance ? yR - yL : yL - yR); // >= 0
            float singleStance = math.saturate(liftDiff / math.max(liftRef, 1e-3f));
            float3 mid = 0.5f * (pL + pR); mid.y = 0f;
            float3 stanceXz = lIsStance ? new float3(pL.x, 0f, pL.z) : new float3(pR.x, 0f, pR.z);
            float3 balanceTarget = math.lerp(mid, stanceXz, singleStance);
            float3 comXz = new float3(com.x, 0f, com.z);
            // Signed torso-frame foot separations, shared by the crossing guard
            // and the stance-width cap (below).
            // Rig-left axis. With fwd = +Z this is cross(fwd, up); the old -Z
            // fwd needed cross(up, fwd) to point the same way. Axis direction
            // is PRESERVED across the convention flip: sL/sR ("left minus
            // right", width >= 0 sane) keep their pre-flip semantics.
            float3 sideT = math.normalizesafe(math.cross(torsoFwdXz, new float3(0f, 1f, 0f)));
            float sL = math.dot(pL - comXz, sideT);
            float sR = math.dot(pR - comXz, sideT);
            float offset = math.abs(math.dot(comXz - balanceTarget, side));
            float balancePay = math.saturate(1f - offset / math.max(balanceMargin, 1e-3f));
            // Stance width cap: a wide planted base is STATICALLY stable (max
            // balance payout, no debt risk) - the splits exploit. Healthy gait
            // width ~0.2 m; beyond stanceWidthRef the payout fades to zero, so
            // balance only pays when the COM is centered over a DYNAMIC-width
            // stance.
            float width = sL - sR; // torso frame, >= 0 in a sane stance
            balancePay *= math.saturate(1f - math.max(0f, width - stanceWidthRef) / math.max(stanceWidthMargin, 1e-3f));
            rBalance = balanceK * balancePay * speedGate * upGate;

            // Leg crossing guard (term 11): crossing during a turn is the
            // classic trip - the feet swap lateral order and the swing leg
            // clips the stance one. sL/sR computed above (torso frame):
            // reversal or turn-in-place flips the motion axis, but "left foot
            // left of right foot" must not flip with it.
            float deficit = math.max(0f, crossTol - (sL - sR));
            rCross = -crossK * math.saturate(deficit / math.max(crossMargin, 1e-3f)) * upGate;
        }

        private void Demodulate(ref GaitCorrelator g, Box3DBody foot, float3 com,
            float3 travel, float sn, float cs, float decay, float dt)
        {
            float3 p = (float3)foot.Body.GetPosition();
            float x = math.dot(p - com, travel);
            g.s = g.s * decay + dt * x * sn;
            g.c = g.c * decay + dt * x * cs;
            // Vertical channel relative to COM: torso bob (whole-body rise/fall)
            // cancels in the subtraction, leaving pure foot clearance.
            float y = p.y - com.y;
            g.sy = g.sy * decay + dt * y * sn;
            g.cy = g.cy * decay + dt * y * cs;
        }

        /// Stride amplitude (m) from the leaky correlator: steady state of the
        /// demodulated sinusoid is a*window/2, so A = 2*|s|/window.
        private float GaitAmp(in GaitCorrelator g) => 2f * math.sqrt(g.s * g.s + g.c * g.c) / math.max(gaitWindow, 1e-2f);

        /// Locked vertical (lift) amplitude from the same clock, relative to COM.
        private float GaitAmpY(in GaitCorrelator g) => 2f * math.sqrt(g.sy * g.sy + g.cy * g.cy) / math.max(gaitWindow, 1e-2f);

        /// 0..1 anti-phase score: 1 when the feet's locked phases differ by pi
        /// (a walk), 0 when in phase (a hop). cos(dPhi) from the correlators.
        private static float AntiPhase(in GaitCorrelator a, in GaitCorrelator b)
        {
            float ma = math.length(new float2(a.s, a.c));
            float mb = math.length(new float2(b.s, b.c));
            if (ma < 1e-4f || mb < 1e-4f) return 0f;
            float cosD = (a.s * b.s + a.c * b.c) / (ma * mb);
            return 0.5f - 0.5f * cosD; // 1 - cos(phiL - phiR - pi) over 2
        }

        /// <summary>Mass-weighted COM of the whole rig (torso + all bones),
        /// world space. float3.zero if the cache is empty.</summary>
        public float3 RigComW()
        {
            float totalMass = 0f;
            float3 com = float3.zero;
            for (int i = 0; i < m_rigBodies.Count; i++)
            {
                var b = m_rigBodies[i].Body;
                if (!b.IsValid) continue;
                float m = b.GetMass();
                com += (float3)b.GetWorldCenterOfMass() * m;
                totalMass += m;
            }
            return totalMass > 0f ? com / totalMass : float3.zero;
        }

        private float Upright()
        {
            float3 up = math.mul(m_torso.Body.Rotation, new float3(0f, 1f, 0f));
            return up.y; // worldUp.y = 1, dot = up.y
        }

        // Shortest signed angle between torso yaw and the target's look heading (rad).
        private float AlignError()
        {
            // Rig forward = local +Z (matches the authored spawn facing / yaw-lock).
            // Flipped from the old -Z read after the rig's facing was re-verified;
            // the assist must flip with this - they are one convention.
            float3 fwd = math.mul(m_torso.Body.Rotation, new float3(0f, 0f, 1f));
            float yaw = math.atan2(fwd.x, fwd.z);
            float look = math.atan2(target.LookDirW.x, target.LookDirW.z);
            return math.atan2(math.sin(yaw - look), math.cos(yaw - look));
        }
    }
}
