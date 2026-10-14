using Box3D.Hybrid;
using Unity.Mathematics;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Cheap inverted-pendulum lean control: rotates the axis-remap frame of
    /// one god-forces entry (typically the hips/pelvis) so the body chases a
    /// tilted desired orientation instead of the target's straight one.
    ///
    /// Two modes:
    /// - Manual: fixed lean angle toward a root-relative direction (tuning knob).
    /// - Upright: gravity-driven - measure the body's current up axis against
    ///   world up and command a lean that pulls it back upright (angle grows
    ///   with the tilt error, scaled by uprightGain, capped). No support or
    ///   COM knowledge needed - pure inverted-pendulum "fall toward gravity,
    ///   drive back toward gravity-up".
    ///
    /// The remap column is written in TARGET-LOCAL space: the desired world
    /// tilt is expressed through the animated root rotation (the driver-driven
    /// reference frame), so an upright animation yields the intended world
    /// tilt regardless of facing. God forces sanitizes the matrix (re-orthogonalize
    /// against X, derive Z) every step, so only Y needs writing.
    ///
    /// Pairs with the pose-bias dead-band: while the picked pose agrees with
    /// the resulting lean the bias injects nothing, so this override does not
    /// fight the animation or exaggerate itself.
    /// </summary>
    [DisallowMultipleComponent]
    public class Box3DLeanController : MonoBehaviour
    {
        public enum Mode { Manual, Upright }

        [SerializeField, Tooltip("God forces component whose entry is tilted. Auto-found when null.")]
        private Box3DGodForces godForces;
        [SerializeField, Tooltip("The body whose drive entry gets the tilted remap (usually the pelvis/hips body).")]
        private Box3DBody leanBody;
        [SerializeField, Tooltip("Mode: Manual = fixed angle toward root-relative direction; Upright = tilt-driven, lean back toward the slope reference in proportion to the current tilt error.")]
        private Mode mode = Mode.Manual;
        [SerializeField, Tooltip("Upright reference: the controller wrapper whose SmoothUp (low-passed predicted surface up) is the upright target. Null = world up - on slopes the recovery then fights the legitimate terrain tilt.")]
        private Box3DControllerWrapper upSource;

        [Header("Manual")]
        [SerializeField, Range(-30f, 30f), Tooltip("Manual lean angle (deg). Positive = lean toward leanDirection, negative = lean away from it. 0 = upright.")]
        private float leanAngle = 0f;
        [SerializeField, Tooltip("Lean direction in ROOT-LOCAL space (horizontal part is used, normalized). Default root-forward.")]
        private Vector3 leanDirection = new Vector3(0f, 0f, 1f);

        [Header("Upright")]
        [SerializeField, Tooltip("Upright gain: commanded recovery angle per degree of current tilt. 1 = cancel the tilt exactly, >1 = overcorrect, <1 = partial. Sustained lean requires >1 (else the drive only slows the fall).")]
        private float uprightGain = 1.2f;
        [SerializeField, Range(0f, 45f), Tooltip("Maximum commanded lean angle (deg), both modes.")]
        private float maxLeanAngle = 15f;

        [Header("General")]
        [SerializeField, Tooltip("Physical animator supplying the animated root rotation (the reference frame of the drive). Auto-found when null.")]
        private Box3DPhysicalAnimator physicalAnimator;
        [SerializeField, Tooltip("Lean angle low-pass rate (1/s): smooths the commanded tilt, both ways.")]
        private float leanRampRate = 8f;

        private float m_smoothed;       // low-passed lean angle (deg, signed in Manual mode)
        private float3 m_axisW;         // world tilt axis (Manual: from direction; Upright: from measured tilt)
        private Vector3 m_baseRemapX = new Vector3(1f, 0f, 0f);
        private Vector3 m_baseRemapY = new Vector3(0f, 1f, 0f);
        private Vector3 m_baseRemapZ = new Vector3(0f, 0f, 1f);
        private Box3DGodForces.Entry m_entry;
        private bool m_valid;

        private void OnEnable()
        {
            m_valid = false;
            if (godForces == null) godForces = FindFirstObjectByType<Box3DGodForces>();
            if (physicalAnimator == null) physicalAnimator = FindFirstObjectByType<Box3DPhysicalAnimator>();
            if (upSource == null) upSource = FindFirstObjectByType<Box3DControllerWrapper>();
            if (leanBody == null || godForces == null)
            {
                Debug.LogWarning("[Box3DLeanController] leanBody/godForces missing - disabled", this);
                enabled = false;
                return;
            }
            foreach (var e in godForces.Entries)
            {
                if (e.body == leanBody) { m_entry = e; break; }
            }
            if (m_entry == null)
            {
                Debug.LogWarning($"[Box3DLeanController] no god-forces entry drives '{leanBody.name}' - disabled", this);
                enabled = false;
                return;
            }
            m_baseRemapX = m_entry.axisRemapX;
            m_baseRemapY = m_entry.axisRemapY;
            m_baseRemapZ = m_entry.axisRemapZ;
            m_smoothed = 0f;
            m_valid = true;
        }

        private void OnDisable()
        {
            if (m_entry != null)
            {
                m_entry.axisRemapX = m_baseRemapX;
                m_entry.axisRemapY = m_baseRemapY;
                m_entry.axisRemapZ = m_baseRemapZ;
            }
            m_entry = null;
            m_valid = false;
        }

        private void FixedUpdate()
        {
            if (!m_valid) return;

            // Commanded angle + world tilt axis for this step.
            float angle;
            // Upright reference: the predicted surface frame when wired, world
            // up otherwise. Recovery toward the slope frame preserves legit
            // terrain tilt; recovery toward world up would fight the slope.
            float3 upW = upSource != null ? upSource.SmoothUp : new float3(0f, 1f, 0f);
            if (mode == Mode.Manual)
            {
                // Signed: negative leans away from leanDirection.
                angle = math.clamp(leanAngle, -maxLeanAngle, maxLeanAngle);
                float3 dirW = math.normalizesafe(
                    new float3(leanDirection.x, 0f, leanDirection.z));
                m_axisW = math.lengthsq(dirW) > 0.5f
                    ? math.normalizesafe(math.cross(upW, dirW), new float3(1f, 0f, 0f))
                    : upW; // degenerate direction: axis never used (angle >= threshold check below)
            }
            else
            {
                // Upright: recovery axis + angle from the MEASURED tilt of the
                // body's up axis against world up. cross(bodyUp, worldUp) gives
                // the axis that rotates bodyUp toward upright.
                float3 bodyUp = math.mul((quaternion)leanBody.Body.Rotation, upW);
                float tilt = math.degrees(math.acos(math.clamp(math.dot(bodyUp, upW), -1f, 1f)));
                angle = math.clamp(tilt * uprightGain, 0f, maxLeanAngle);
                float3 axis = math.cross(bodyUp, upW);
                m_axisW = math.lengthsq(axis) > 1e-6f
                    ? math.normalizesafe(axis)
                    : new float3(1f, 0f, 0f); // already upright: arbitrary axis, angle ~ 0
            }

            // Low-pass the commanded angle (axis follows instantly - it is the
            // angle magnitude that must not step).
            float a = 1f - math.exp(-math.max(leanRampRate, 0.01f) * Time.fixedDeltaTime);
            m_smoothed = math.lerp(m_smoothed, angle, a);
            if (math.abs(m_smoothed) < 0.05f)
            {
                // Upright: restore the authored frame.
                m_entry.axisRemapX = m_baseRemapX;
                m_entry.axisRemapY = m_baseRemapY;
                m_entry.axisRemapZ = m_baseRemapZ;
                return;
            }

            // Rigid tilt of the whole desired orientation: rotate the authored
            // remap frame by the tilt about the world axis, expressed in the
            // drive's reference frame (animated root rotation). X and Y are
            // written; the sanitizer derives Z from them (authored Z is
            // ignored), and a rotated orthonormal pair re-derives exactly the
            // rotated Z. Writing Y alone cannot express sideways lean - the
            // sanitizer projects Y against X and strips the tilt component.
            physicalAnimator.GetRootPose(out _, out quaternion rootRot, 0f);
            quaternion q = math.mul(math.inverse(rootRot),
                math.mul(quaternion.AxisAngle(m_axisW, math.radians(m_smoothed)), rootRot));
            m_entry.axisRemapX = (Vector3)math.normalizesafe(
                math.mul(q, (float3)m_baseRemapX), (float3)m_baseRemapX);
            m_entry.axisRemapY = (Vector3)math.normalizesafe(
                math.mul(q, (float3)m_baseRemapY), (float3)m_baseRemapY);
        }
    }
}