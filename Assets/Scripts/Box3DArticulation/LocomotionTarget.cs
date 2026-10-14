using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>Roaming COM target for the locomotion task. The target point
    /// drifts on the floor with inertia: a movement direction wanders
    /// gradually (occasionally kicked by a larger, still-smoothed turn) and
    /// the speed eases between limits. Position is clamped to floor bounds.
    /// Look direction is independent: its own smoothed horizontal angle, with
    /// the same occasional-kick schedule. Pure task definition + visualization -
    /// no reward logic here, the (future) agent component reads from it.</summary>
    public class LocomotionTarget : MonoBehaviour
    {
        [Header("Floor bounds (world XZ, fixed at creation)")]
        [SerializeField] private Vector2 floorSize = new Vector2(14f, 14f);
        [SerializeField, Tooltip("Fixed world height of the target marker.")]
        private float height = 1.3f;

        [Header("Movement")]
        [SerializeField, Min(0.01f)] private float accel = 1.2f;       // m/s^2 speed authority
        [SerializeField, Tooltip("Spawn heading source: 0 = target faces the biped's yaw every reset (aligned). 1 = fully random. Between: random with probability d (curriculum blends).")]
        [Range(0f, 1f)] private float headingRandomizeAt = 0f;
        [SerializeField] private float speedMin = 0.2f;                // m/s
        [SerializeField, Tooltip("Speed range ceiling (m/s). With the curriculum at max difficulty the target reaches this - 3.0 is a slow run (walking is ~1.4). The ghost chaser and its run-blend speed are tuned to match.")]
        private float speedMax = 3.0f;               // m/s: max difficulty = running
        [SerializeField] private float dirWanderRate = 1.2f;           // rad/s noise bandwidth
        [SerializeField] private float dirMaxRate = 1.5f;              // rad/s normal turn clamp
        [SerializeField, Range(0f, 1f)] private float bigKickChance = 0.02f; // per second
        [SerializeField] private float bigKickAngle = 2.4f;            // rad, mean |kick|
        [SerializeField] private float kickSmoothTime = 1.2f;          // s, kick blends in over this
        [SerializeField] private float speedRetargetTime = 2.5f;       // s, speed setpoint hold
        [SerializeField, Tooltip("RSI-style (DeepMimic): sample the initial target speed per episode from [speedMin, speedMax] instead of starting frozen. The target walks from step 0 - standing still is never a max-return mode.")]
        private bool speedSampleAtReset = true;
        [SerializeField, Tooltip("Curriculum floor: at difficulty 0 the speed range is scaled to this fraction of the full range (stage 0 = slow walking, never frozen).")]
        [Range(0f, 1f)] private float startSpeedFrac = 0.25f;
        [SerializeField, Tooltip("Border margin: heading is steered away from the walls within this distance.")]
        private float borderMargin = 1.5f;

        [Header("Look direction (horizontal only)")]
        [SerializeField] private float lookWanderRate = 0.8f;          // rad/s noise bandwidth
        [SerializeField] private float lookMaxRate = 1.2f;             // rad/s normal turn clamp
        [SerializeField, Range(0f, 1f)] private float lookKickChance = 0.015f;
        [SerializeField] private float lookKickAngle = 2.8f;           // rad
        [SerializeField] private float lookKickSmoothTime = 1.0f;

        [Header("Visualization")]
        [SerializeField] private float markerRadius = 0.25f;

        // --- movement state ---
        private float m_speed;           // current |v|
        private float m_speedSetpoint;
        private float m_speedRetarget;
        private float m_moveAngle;       // heading (rad, world)
        private float m_dirKick;         // active kick rate (rad/s), decaying envelope
        private float m_dirKickT;
        // floor bounds captured at spawn (start position) - the gizmo rect must
        // not track the marker, only the marker itself moves.
        private float3 m_floorCenter;
        private float3 m_pos;

        // --- look state ---
        private float m_lookAngle;
        private float m_lookKick;
        private float m_lookKickT;

        // noise phases (band-limited wander via layered sines - deterministic,
        // allocation-free, good enough for a target wanderer)
        private float m_n1, m_n2, m_n3, m_n4;

        public float3 Position => m_pos;
        public float3 LookDirW => new float3(math.sin(m_lookAngle), 0f, math.cos(m_lookAngle));
        public Vector2 FloorSize => floorSize;
        public float3 MoveDirW => new float3(math.sin(m_moveAngle), 0f, math.cos(m_moveAngle));
        public float Speed => m_speed;
        /// <summary>Current planar velocity (world). Zero when frozen at d=0.
        /// For reward/obs consumers that need the target's frame of motion.</summary>
        public float3 Velocity => MoveDirW * m_speed;

        // --- curriculum ---
        // d = 0: frozen target (stand/stabilize only). d = 1: full field values.
        // Serialized fields keep their meaning as d=1 endpoints; difficulty is
        // re-read on every episode begin, so it applies to newly spawned agents.
        // --- curriculum ---
        // d = 0: frozen target (stand/stabilize only). d = 1: full field values.
        // Endpoint constants are captured from the serialized fields once
        // (OnEnable): ApplyDifficulty overwrites the fields every episode,
        // and lerping from the fields themselves would compound -
        // speedMax <- lerp(0, speedMax, d) each episode decays exponentially
        // to zero, which froze the target's motion entirely.
        private float kSpeedMax, kSpeedMin, kDirWanderRate, kDirMaxRate, kLookWanderRate, kLookMaxRate;
        private bool kCaptured;

        private void CaptureEndpoints()
        {
            if (kCaptured) return;
            kSpeedMax = speedMax; kSpeedMin = speedMin;
            kDirWanderRate = dirWanderRate; kDirMaxRate = dirMaxRate;
            kLookWanderRate = lookWanderRate; kLookMaxRate = lookMaxRate;
            kCaptured = true;
        }

        public void ApplyDifficulty(float d)
        {
            CaptureEndpoints();
            d = math.clamp(d, 0f, 1f);
            speedMax = math.lerp(kSpeedMax * startSpeedFrac, kSpeedMax, d);
            speedMin = math.lerp(kSpeedMin * startSpeedFrac, kSpeedMin, d);
            dirWanderRate = math.lerp(0f, kDirWanderRate, d);
            dirMaxRate = math.lerp(0f, kDirMaxRate, d);
            bigKickChance = 0.02f * d;
            lookWanderRate = math.lerp(0f, kLookWanderRate, d);
            lookMaxRate = math.lerp(0f, kLookMaxRate, d);
            lookKickChance = 0.015f * d;
            LastDifficulty = d; // remembered for ResetToSpawn
        }

        /// <summary>Per-episode reset: target back at the spawn point, all
        /// motion/look state zeroed (kick envelopes, noise phases, retarget
        /// timer). Without this the marker stays where the last episode's
        /// wander left it and every episode starts from a stale task state.
        /// spawnYaw: the biped's facing at reset - the target starts facing
        /// the same direction, so tracking and walking align from step 0.</summary>
        public void ResetToSpawn(float difficulty, float spawnYawRad)
        {
            // m_floorCenter is the ANCHOR, captured once (OnEnable / first
            // spawn) - never re-read from transform here: the marker's own
            // movement writes transform.position every step, so re-anchoring
            // from it let each target random-walk its floor rect across the
            // grid. The rect stays put; only the marker moves.
            m_pos = new float3(m_floorCenter.x, height, m_floorCenter.z);
            transform.position = new Vector3(m_pos.x, m_pos.y, m_pos.z);
            m_speed = m_speedSetpoint = 0f;
            m_speedRetarget = Time.time + speedRetargetTime;
            // RSI-style reset: the target starts ALREADY moving at a sampled
            // speed - every episode opens with the follow task live. Frozen
            // starts let the policy farm max return standing beside a still
            // target and never learn to walk (DeepMimic's RSI rationale: states
            // the policy must master must be on-policy from the start).
            if (speedSampleAtReset)
            {
                m_speedSetpoint = UnityEngine.Random.Range(speedMin, speedMax);
                m_speed = m_speedSetpoint; // no ease-in - momentum is the target's, not the rig's
            }
            else
            {
                m_speed = m_speedSetpoint = 0f;
            }
            // Face the biped's heading every episode by default; with
            // headingRandomizeAt > 0, blend in random headings (probability =
            // the field) so varied approach directions can be re-enabled
            // later without a code change.
            if (headingRandomizeAt > 0f && UnityEngine.Random.value < headingRandomizeAt)
            {
                m_moveAngle = UnityEngine.Random.value * 6.2831853f;
                m_lookAngle = UnityEngine.Random.value * 6.2831853f;
            }
            else
            {
                m_moveAngle = spawnYawRad;
                m_lookAngle = spawnYawRad;
            }
            m_dirKick = 0f; m_dirKickT = 0f;
            m_lookKick = 0f; m_lookKickT = 0f;
            m_n1 = m_n2 = m_n3 = m_n4 = 0f;
        }

        /// <summary>Last difficulty passed to ApplyDifficulty (curriculum state
        /// read-back for ResetToSpawn's heading policy).</summary>
        private float LastDifficulty { get; set; }

        private void OnEnable()
        {
            // Anchor once: the floor rect and spawn point are fixed at the
            // component's spawn position for the component's lifetime.
            m_floorCenter = transform.position;
            m_pos = new float3(transform.position.x, height, transform.position.z);
        }

        private void FixedUpdate()
        {
            float dt = Time.fixedDeltaTime;
            float t = Time.time;
            m_n1 = t * dirWanderRate;
            m_n2 = t * dirWanderRate * 0.53f + 17.3f;
            m_n3 = t * lookWanderRate;
            m_n4 = t * lookWanderRate * 0.61f + 43.7f;

            // --- movement heading: wander + occasional kick + border push ---
            float wander = (math.sin(m_n1) * 0.7f + math.sin(m_n2) * 0.3f) * dirMaxRate;
            MaybeKick(ref m_dirKick, ref m_dirKickT, bigKickChance, bigKickAngle, kickSmoothTime);
            // kick is a smoothed step in heading rate: triangle-blend to the
            // kick rate and back, so the heading response stays C1.
            float kickRate = m_dirKick * Mathf.SmoothStep(1f, 0f, m_dirKickT);
            // Border repulsion: within borderMargin of any wall, add a turn
            // rate steering the heading toward the floor center. Scales with
            // proximity (quadratic falloff), so deep incursions turn hard.
            float pushRate = BorderPushRate();
            m_moveAngle += (wander + kickRate + pushRate) * dt;

            // --- speed: ease toward a retargeted setpoint ---
            if (t >= m_speedRetarget)
            {
                m_speedSetpoint = UnityEngine.Random.Range(speedMin, speedMax);
                m_speedRetarget = t + speedRetargetTime * UnityEngine.Random.Range(0.6f, 1.4f);
            }
            m_speed = MoveTowards(m_speed, m_speedSetpoint, accel * dt);

            // --- integrate + clamp to floor bounds ---
            float3 v = MoveDirW * m_speed;
            m_pos += v * dt;
            float hx = floorSize.x * 0.5f, hz = floorSize.y * 0.5f;
            m_pos = math.clamp(m_pos, new float3(m_floorCenter.x - hx, height, m_floorCenter.z - hz),
                                     new float3(m_floorCenter.x + hx, height, m_floorCenter.z + hz));

            // --- look heading: same treatment, independent schedule ---
            float lookWander = (math.sin(m_n3) * 0.7f + math.sin(m_n4) * 0.3f) * lookMaxRate;
            MaybeKick(ref m_lookKick, ref m_lookKickT, lookKickChance, lookKickAngle, lookKickSmoothTime);
            float lookKickRate = m_lookKick * Mathf.SmoothStep(1f, 0f, m_lookKickT);
            m_lookAngle += (lookWander + lookKickRate) * dt;

            transform.position = new Vector3(m_pos.x, m_pos.y, m_pos.z);
        }

        private static float MoveTowards(float current, float target, float maxDelta)
        {
            float d = target - current;
            return math.abs(d) <= maxDelta ? target : current + math.sign(d) * maxDelta;
        }

        // Turn rate steering the heading toward the floor center when inside
        // the border margin (0 outside it). Quadratic in penetration depth,
        // capped well above the normal turn rate so it always wins.
        private float BorderPushRate()
        {
            float hx = floorSize.x * 0.5f, hz = floorSize.y * 0.5f;
            float dxIn = (hx - borderMargin) - math.abs(m_pos.x - m_floorCenter.x);
            float dzIn = (hz - borderMargin) - math.abs(m_pos.z - m_floorCenter.z);
            float d = math.min(dxIn, dzIn);                 // depth into the closest wall's margin
            if (d >= 0f) return 0f;
            float depth = math.clamp(-d / borderMargin, 0f, 1f);
            // signed turn: rotate heading toward the direction that faces the center
            float toCenter = math.atan2(m_floorCenter.x - m_pos.x, m_floorCenter.z - m_pos.z);
            // shortest signed angular difference to the center-facing heading
            float delta = math.atan2(math.sin(toCenter - m_moveAngle), math.cos(toCenter - m_moveAngle));
            return math.sign(delta) * dirMaxRate * 3f * depth * depth;
        }

        private void MaybeKick(ref float kick, ref float kickT, float chance, float angle, float smooth)
        {
            if (kickT > 0f) { kickT -= Time.fixedDeltaTime / math.max(smooth, 1e-3f); if (kickT <= 0f) { kick = 0f; return; } }
            if (UnityEngine.Random.value < chance * Time.fixedDeltaTime)
            {
                kick = UnityEngine.Random.value < 0.5f ? -angle : angle;
                kickT = 1f;
            }
        }

        private void OnDrawGizmos()
        {
            // floor bounds - fixed rect, drawn at spawn center, does NOT track the marker
            Gizmos.color = new Color(0f, 1f, 0.5f, 0.6f);
            Vector3 c = m_floorCenter;
            Vector3 sz = new Vector3(floorSize.x, 0.02f, floorSize.y);
            Gizmos.DrawWireCube(c, sz);

            // target point + look arrow (horizontal)
            Vector3 p = Application.isPlaying ? (Vector3)m_pos : transform.position;
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(p, markerRadius);
            Gizmos.DrawLine(p, p + (Vector3)(LookDirW * (markerRadius * 2.5f)));
            if (!Application.isPlaying) return; // heading only known while running
            Gizmos.color = new Color(1f, 0.6f, 0f);
            Gizmos.DrawLine(p, p + (Vector3)(MoveDirW * (markerRadius * 3.5f)));
        }
    }
}
