using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>Kinematic imitation reference: an animated humanoid ghost that
    /// chases the same LocomotionTarget as its rig instance, inside the same
    /// cell. No physics, no colliders, no Box3D interaction - the ghost is
    /// transform-moved on the fixed step and its Animator steps with
    /// AnimatePhysics so reference and rig stay on the same clock.
    ///
    /// Step 1 of the imitation pipeline: movement + animation only. The
    /// (future) imitation reward reads skeleton anchors off this transform's
    /// Animator hierarchy; nothing here is rewarded or terminated yet.
    ///
    /// Time alignment for free: the ghost does the rig's task (chase the
    /// wandering target) at the same time in the same place, so any
    /// skeleton-to-rig comparison is phase-aligned by construction - no clip
    /// seeking, no retargeting pass. Jump support can slot in later by
    /// replacing the flat groundY mover; the chase/animation interface stays.</summary>
    public sealed class ImitationReference : MonoBehaviour
    {
        [Header("Chase (wired by BipedBatch)")]
        [SerializeField] private LocomotionTarget target;
        [SerializeField, Tooltip("Top speed (m/s). Must exceed the target's speedMax so chase lag stays small and bounded.")]
        private float maxSpeed = 3.2f;
        [SerializeField] private float accel = 3.0f;               // m/s^2
        [SerializeField, Tooltip("Heading turn clamp (rad/s).")]
        private float turnRate = 6.0f;
        [SerializeField, Tooltip("Ghost speed is capped to the target's own speed plus catchupGain * distance - starts slow, grows into pursuit, and never outruns the target by much.")]
        private float catchupGain = 0.4f;

        [Header("Placement")]
        [SerializeField, Tooltip("Persistent transform the spawn point anchors to (the Biped{i} instance root - NOT a child the rig rebuild sweeps). Wired by BipedBatch.")]
        private Transform spawnAnchor;
        [SerializeField, Tooltip("World Y the ghost walks on (batch floor top face).")]
        private float groundY = 0f;
        [SerializeField, Tooltip("Spawn offset from the instance origin (0 = same spot as the biped).")]
        private Vector3 spawnOffset = Vector3.zero;
        [SerializeField, Tooltip("Correction applied to the model's authored forward if it differs from Unity -Z (degrees around Y).")]
        private float facingOffsetDeg = 0f;

        [Header("Animation")]
        [SerializeField, Tooltip("Animator float parameter driven by planar speed (normalized 0..1). Used when the 2D input pair is absent.")]
        private string speedParam = "Speed";
        [SerializeField, Tooltip("2D locomotion blend tree (Huts Studio style): forward axis param. Auto-selected when both this and inputXParam exist.")]
        private string inputYParam = "InputY";
        [SerializeField] private string inputXParam = "InputX";
        [SerializeField, Tooltip("Undamped input memory the asset's turn animations read (optional).")]
        private string lastInputXParam = "LastInputX";
        [SerializeField] private string lastInputYParam = "LastInputY";
        [SerializeField, Tooltip("Animator bool set while parked but facing misaligned (idle-turn clips).")]
        private string isTurningParam = "IsTurning";
        [SerializeField, Tooltip("Damped turn direction (-1/+1) feeding the idle-turn blend.")]
        private string turnDirectionParam = "TurnDirection";
        [SerializeField, Tooltip("Animator bool set while planar speed exceeds the moving threshold (optional).")]
        private string isMovingParam = "IsMoving";
        [SerializeField, Tooltip("Animator bool forced true - the ghost is always kinematically grounded.")]
        private string isGroundedParam = "IsGrounded";
        [SerializeField, Tooltip("Animator float forced 0 - no vertical velocity in ghost mode (jump comes later via the mover).")]
        private string verticalVelocityParam = "VerticalVelocity";
        [SerializeField, Tooltip("Planar speed (m/s) that maps the animation parameter to 1 (full run blend). Matched to the curriculum's max-difficulty target speed.")]
        private float animSpeedAtOne = 3.0f;
        [SerializeField, Tooltip("Damping time for the animation inputs (the asset's own PlayerAnimator default is 0.1). This is what makes direction changes blend smoothly.")]
        private float dampTime = 0.1f;
        [SerializeField, Tooltip("Facing misalignment (deg) at which turn-in-place triggers while parked.")]
        private float turnStartAngleDeg = 30f;

        private Animator m_animator;
        private int m_speedParamId;
        private bool m_hasSpeedParam;
        private int m_inputXId, m_inputYId, m_isMovingId, m_isGroundedId, m_verticalId;
        private int m_lastInputXId, m_lastInputYId, m_isTurningId, m_turnDirId;
        private bool m_has2DParams, m_hasIsMoving, m_hasIsGrounded, m_hasVertical;
        private bool m_hasLastInput, m_hasIsTurning, m_hasTurnDir;
        // Damped animation inputs + their SmoothDamp velocity state (kept
        // across steps; zeroed on episode reset).
        private float m_inX, m_inY, m_inXV, m_inYV, m_turnD, m_turnDV;
        private float m_speed;
        private float m_heading;          // movement heading (rad, world, atan2(x, z) convention)
        private float m_facingYaw;        // smoothed body yaw
        private float3 m_spawnPos;
        private bool m_spawnCaptured;

        /// <summary>Current planar speed (m/s) - reads for the future reward layer.</summary>
        public float PlanarSpeed => m_speed;
        /// <summary>Body facing (world, horizontal, normalized) - the world
        /// direction the visible model looks along, including the authored
        /// forward correction. Read for the future imitation reward layer.</summary>
        public float3 FacingDirW
        {
            get
            {
                float yaw = m_facingYaw + facingOffsetDeg * Mathf.Deg2Rad;
                return new float3(math.sin(yaw), 0f, math.cos(yaw));
            }
        }

        private void Awake()
        {
            // A full controller prefab (Starter Assets-style) carries its own
            // locomotion/input/gravity components - all of it is replaced by
            // this kinematic mover. Disable rather than destroy so the user's
            // prefab stays intact for the driven-controller upgrade path.
            DisableForeignComponents();
            m_animator = GetComponentInChildren<Animator>();
            if (m_animator != null)
            {
                // Determinism with the fixed physics step + no culling under batch training.
                m_animator.updateMode = AnimatorUpdateMode.Fixed;
                m_animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;
                m_inputXId = ParamId(inputXParam);
                m_inputYId = ParamId(inputYParam);
                m_has2DParams = m_inputXId != 0 && m_inputYId != 0;
                m_speedParamId = ParamId(speedParam);
                m_hasSpeedParam = m_speedParamId != 0 && !m_has2DParams; // 2D tree wins when both exist
                m_isMovingId = ParamId(isMovingParam);
                m_hasIsMoving = m_isMovingId != 0;
                m_isGroundedId = ParamId(isGroundedParam);
                m_hasIsGrounded = m_isGroundedId != 0;
                m_verticalId = ParamId(verticalVelocityParam);
                m_hasVertical = m_verticalId != 0;
                m_lastInputXId = ParamId(lastInputXParam);
                m_lastInputYId = ParamId(lastInputYParam);
                m_hasLastInput = m_lastInputXId != 0 && m_lastInputYId != 0;
                m_isTurningId = ParamId(isTurningParam);
                m_hasIsTurning = m_isTurningId != 0;
                m_turnDirId = ParamId(turnDirectionParam);
                m_hasTurnDir = m_turnDirId != 0;
            }
        }

        /// <summary>StringToHash + existence check; 0 when the parameter is
        /// absent from the controller (all drives are then skipped per-param).</summary>
        private int ParamId(string name)
        {
            if (string.IsNullOrEmpty(name)) return 0;
            int id = Animator.StringToHash(name);
            var pars = m_animator.parameters;
            for (int i = 0; i < pars.Length; i++)
                if (pars[i].nameHash == id) return id;
            return 0;
        }

        // One shared log across the batch - 64 instances would spam otherwise.
        private static bool s_strippedLogged;

        private void DisableForeignComponents()
        {
            bool stripped = false;
            // Physics: never let Unity-side bodies/characters move the ghost.
            foreach (var rb in GetComponentsInChildren<Rigidbody>(true)) { rb.isKinematic = true; stripped = true; }
            foreach (var col in GetComponentsInChildren<Collider>(true)) { col.enabled = false; stripped = true; }
            // ALL user scripts off - the ghost only needs Animator + renderers.
            // Generic (not name-based) so any imported controller asset works:
            // locomotion/input/animator-driver/foot-IK/cursor scripts included.
            foreach (var b in GetComponentsInChildren<Behaviour>(true))
            {
                if (b is Animator || b is ImitationReference) continue;
                if (b is MonoBehaviour) { b.enabled = false; stripped = true; }
            }
            if (stripped && !s_strippedLogged)
            {
                s_strippedLogged = true;
                Debug.Log("[ImitationReference] Disabled controller/physics components on the reference prefab (kinematic ghost mode).");
            }
        }

        // Spawn anchor: the persistent instance root when wired, else the
        // ghost's own placement at activation. Re-derived every reset - the
        // anchor never moves, so this is stable across the batch lifetime.
        private float3 AnchorXZ => spawnAnchor != null
            ? new float3(spawnAnchor.position.x, groundY, spawnAnchor.position.z)
            : new float3(transform.position.x, groundY, transform.position.z);

        private void OnEnable()
        {
            // Initial placement: spawn pos + offset (same convention as
            // LocomotionTarget - transform movement never re-anchors it).
            if (!m_spawnCaptured)
            {
                m_spawnPos = AnchorXZ + new float3(spawnOffset.x, 0f, spawnOffset.z);
                transform.position = new Vector3(m_spawnPos.x, groundY, m_spawnPos.z);
                m_spawnCaptured = true;
            }
        }

        /// <summary>Per-episode reset: snap to the spawn point, zero motion,
        /// face the rig's spawn yaw (the target does the same, so ghost and rig
        /// open every episode aligned).</summary>
        public void EpisodeReset(float spawnYawRad)
        {
            m_spawnPos = AnchorXZ + new float3(spawnOffset.x, 0f, spawnOffset.z);
            m_speed = 0f;
            m_heading = spawnYawRad;
            m_facingYaw = spawnYawRad;
            m_inX = m_inY = m_inXV = m_inYV = m_turnD = m_turnDV = 0f;
            transform.position = new Vector3(m_spawnPos.x, groundY, m_spawnPos.z);
            transform.rotation = Quaternion.Euler(0f, (spawnYawRad + facingOffsetDeg * Mathf.Deg2Rad) * Mathf.Rad2Deg, 0f);
        }

        private void FixedUpdate()
        {
            if (target == null) return;
            float dt = Time.fixedDeltaTime;

            // --- chase: target XZ (its Position carries the marker height; ignore it) ---
            float3 tPos = target.Position;
            float3 pos = new float3(transform.position.x, groundY, transform.position.z);
            float3 toT = new float3(tPos.x - pos.x, 0f, tPos.z - pos.z);
            float dist = math.length(toT);

            // Braking-aware desired speed: v = sqrt(2*a*d) arrives exactly at
            // zero speed (kinematics), so the ghost SETTLES at the target
            // instead of overshooting and oscillating around it forever.
            // Catch-up cap: never chase faster than the target's own speed
            // plus a bounded term - the ghost starts slow (no initial lurch,
            // the target's RSI momentum is matched gradually) and the trailing
            // gap stabilizes at catchupGain-earned distance instead of zero.
            float targetSpd = target.Speed;
            float desiredSpeed = math.min(maxSpeed, math.sqrt(2f * accel * dist));
            desiredSpeed = math.min(desiredSpeed, targetSpd + catchupGain * dist);
            m_speed = MoveTowards(m_speed, desiredSpeed, accel * dt);

            if (dist > 1e-3f)
            {
                float targetHeading = math.atan2(toT.x, toT.z);
                m_heading = MoveTowardsAngle(m_heading, targetHeading, turnRate * dt);
            }

            pos += new float3(math.sin(m_heading), 0f, math.cos(m_heading)) * (m_speed * dt);
            transform.position = new Vector3(pos.x, groundY, pos.z);

            // --- facing: the body ALWAYS chases the target's look channel ---
            // (the asset's own convention: PlayerLocomotion faces the look
            // direction while moving too - strafe blends come from movement
            // projected into that facing, not from facing the movement)
            float3 look = target.LookDirW;
            float lookYaw = math.atan2(look.x, look.z);
            float faceYawErr = math.atan2(math.sin(lookYaw - m_facingYaw), math.cos(lookYaw - m_facingYaw));
            m_facingYaw = MoveTowardsAngle(m_facingYaw, lookYaw, turnRate * dt);
            transform.rotation = Quaternion.Euler(0f, (m_facingYaw + facingOffsetDeg * Mathf.Deg2Rad) * Mathf.Rad2Deg, 0f);

            if (m_animator == null) return;

            // --- animation drive: movement in the lagging body frame, damped ---
            // (same structure as the asset's own PlayerAnimator: damped 2D
            // input + booleans, so its authored blending is preserved)
            float3 fwd = new float3(math.sin(m_facingYaw), 0f, math.cos(m_facingYaw));
            float3 right = new float3(fwd.z, 0f, -fwd.x);          // fwd=+Z -> right=+X
            float3 moveDir = new float3(math.sin(m_heading), 0f, math.cos(m_heading));
            float norm = m_speed / math.max(animSpeedAtOne, 1e-3f);
            float tx = math.dot(moveDir, right) * norm;            // strafe axis
            float ty = math.dot(moveDir, fwd) * norm;              // forward axis
            m_inX = Mathf.SmoothDamp(m_inX, tx, ref m_inXV, dampTime, Mathf.Infinity, dt);
            m_inY = Mathf.SmoothDamp(m_inY, ty, ref m_inYV, dampTime, Mathf.Infinity, dt);

            bool moving = m_speed > 0.05f;
            if (m_has2DParams)
            {
                m_animator.SetFloat(m_inputXId, m_inX);
                m_animator.SetFloat(m_inputYId, m_inY);
                // Input memory for the asset's turn animations (undamped,
                // like its own driver writes them).
                if (m_hasLastInput && moving)
                {
                    m_animator.SetFloat(m_lastInputXId, tx);
                    m_animator.SetFloat(m_lastInputYId, ty);
                }
            }
            else if (m_hasSpeedParam)
            {
                m_animator.SetFloat(m_speedParamId, math.saturate(math.abs(m_inY)));
            }

            if (m_hasIsMoving) m_animator.SetBool(m_isMovingId, moving);
            if (m_hasIsGrounded) m_animator.SetBool(m_isGroundedId, true); // always kinematically grounded
            if (m_hasVertical) m_animator.SetFloat(m_verticalId, 0f);      // jump arrives via the mover later

            // Turn-in-place: parked while the look channel wanders keeps the
            // idle-turn state alive instead of a frozen stance.
            if (m_hasIsTurning || m_hasTurnDir)
            {
                bool turning = !moving && math.abs(faceYawErr) > turnStartAngleDeg * Mathf.Deg2Rad;
                m_turnD = Mathf.SmoothDamp(m_turnD, math.sign(faceYawErr), ref m_turnDV, dampTime, Mathf.Infinity, dt);
                if (m_hasIsTurning) m_animator.SetBool(m_isTurningId, turning);
                if (m_hasTurnDir) m_animator.SetFloat(m_turnDirId, turning ? m_turnD : 0f);
            }
        }

        private static float MoveTowards(float current, float target, float maxDelta)
        {
            float d = target - current;
            return math.abs(d) <= maxDelta ? target : current + math.sign(d) * maxDelta;
        }

        private static float MoveTowardsAngle(float current, float target, float maxDelta)
        {
            float delta = math.atan2(math.sin(target - current), math.cos(target - current));
            return MoveTowards(current, current + delta, maxDelta);
        }
    }
}
