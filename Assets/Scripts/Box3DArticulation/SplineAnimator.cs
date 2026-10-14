using System;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Animates spline control points to test rig tracking of a moving rail.
    /// Attach to the same GameObject as SplineChainController.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(SplineChainController))]
    public sealed class SplineAnimator : MonoBehaviour
    {
        [Header("Animation Modes")]
        [SerializeField] private AnimationMode mode = AnimationMode.VerticalWave;
        [SerializeField] private bool animateOnAwake = true;
        [SerializeField] private float startDelay = 1f;

        [Header("Vertical Wave (climbing motion)")]
        [SerializeField] private float waveAmplitude = 1.0f; // Reduced to 1.0 for tracking test (use Static mode for static pose)
        [SerializeField] private float waveFrequency = 0.5f;      // Hz
        [SerializeField] private float waveSpeed = 1f;             // wave travel speed along rail
        [SerializeField] private float waveVerticalOffset = 0f;

        [Header("Horizontal Sway")]
        [SerializeField] private float swayAmplitude = 1f;
        [SerializeField] private float swayFrequency = 0.3f;
        [SerializeField] private float swayPhaseOffset = 0f;

        [Header("Perturbation Test")]
        [SerializeField] private float perturbationInterval = 3f;
        [SerializeField] private float perturbationMagnitude = 0.5f;
        [SerializeField] private float perturbationDuration = 0.5f;

        [Header("Circular Orbit")]
        [SerializeField] private float orbitRadius = 2f;
        [SerializeField] private float orbitFrequency = 0.2f;
        [SerializeField] private float orbitHeight = 0f;

        [Header("Reference (required)")]
        [SerializeField] private Transform[] controlPoints;  // Must match SplineChainController's controlPoints

        private SplineChainController controller;
        private Vector3[] initialPositions;
        private float startTime;
        private bool started;

        public enum AnimationMode
        {
            VerticalWave,    // Wave traveling up/down the rail
            HorizontalSway,  // Side-to-side sway
            Perturbation,    // Random kicks to test stability
            CircularOrbit,   // Rail moves in a circle
            Combined,        // Wave + sway
            Static          // No animation (static rail)
        }

        private void Awake()
        {
            controller = GetComponent<SplineChainController>();
            if (controller == null)
            {
                Debug.LogError($"{name}: SplineAnimator requires SplineChainController on same GameObject");
                enabled = false;
                return;
            }

            // Get control points from controller (legacy shared-rail field)
            var field = typeof(SplineChainController).GetField("controlPoints",
                System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
            if (field != null)
            {
                controlPoints = field.GetValue(controller) as Transform[];
            }

            // Per-chain rails: flatten every chain's railControlPoints into one array.
            if (controlPoints == null || controlPoints.Length == 0)
            {
                var chainsField = typeof(SplineChainController).GetField("chains",
                    System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
                var chains = chainsField?.GetValue(controller) as System.Collections.IEnumerable;
                if (chains != null)
                {
                    var points = new System.Collections.Generic.List<Transform>();
                    foreach (var chain in chains)
                    {
                        var rail = chain.GetType().GetField("railControlPoints")?.GetValue(chain) as Transform[];
                        if (rail != null) points.AddRange(rail);
                    }
                    controlPoints = points.ToArray();
                }
            }

            if (controlPoints == null || controlPoints.Length == 0)
            {
                Debug.LogError($"{name}: No control points found on SplineChainController");
                enabled = false;
                return;
            }

            initialPositions = new Vector3[controlPoints.Length];
            for (int i = 0; i < controlPoints.Length; i++)
            {
                if (controlPoints[i] != null)
                    initialPositions[i] = controlPoints[i].position;
            }

            if (animateOnAwake)
            {
                Invoke(nameof(StartAnimation), startDelay);
            }
        }

        public void StartAnimation()
        {
            startTime = Time.time;
            started = true;
        }

        public void StopAnimation()
        {
            started = false;
            // Reset to initial positions
            for (int i = 0; i < controlPoints.Length; i++)
            {
                if (controlPoints[i] != null)
                    controlPoints[i].position = initialPositions[i];
            }
        }

        private void Update()
        {
            if (!started || controlPoints == null || initialPositions == null) return;

            float t = Time.time - startTime;

            switch (mode)
            {
                case AnimationMode.VerticalWave:
                    AnimateVerticalWave(t);
                    break;
                case AnimationMode.HorizontalSway:
                    AnimateHorizontalSway(t);
                    break;
                case AnimationMode.Perturbation:
                    AnimatePerturbation(t);
                    break;
                case AnimationMode.CircularOrbit:
                    AnimateCircularOrbit(t);
                    break;
                case AnimationMode.Combined:
                    AnimateCombined(t);
                    break;
                case AnimationMode.Static:
                    // No animation — rail stays at initial positions
                    break;
            }

            // Force rail cache rebuild (per-chain CacheRails, private, parameterless)
            controller.SendMessage("CacheRails", SendMessageOptions.DontRequireReceiver);
        }

        private void AnimateVerticalWave(float t)
        {
            // Wave travels along the rail (index space)
            for (int i = 0; i < controlPoints.Length; i++)
            {
                if (controlPoints[i] == null) continue;

                // Normalized position along rail (0 to 1)
                float u = controlPoints.Length > 1 ? i / (float)(controlPoints.Length - 1) : 0.5f;
                
                // Phase varies along rail to create traveling wave
                float phase = t * waveFrequency * 2f * Mathf.PI - u * waveSpeed * 2f * Mathf.PI;
                float offset = Mathf.Sin(phase) * waveAmplitude;

                Vector3 pos = initialPositions[i];
                pos.y += offset + waveVerticalOffset;
                controlPoints[i].position = pos;
            }
        }

        private void AnimateHorizontalSway(float t)
        {
            float sway = Mathf.Sin(t * swayFrequency * 2f * Mathf.PI + swayPhaseOffset) * swayAmplitude;
            
            for (int i = 0; i < controlPoints.Length; i++)
            {
                if (controlPoints[i] == null) continue;
                Vector3 pos = initialPositions[i];
                pos.x += sway;
                controlPoints[i].position = pos;
            }
        }

        private void AnimatePerturbation(float t)
        {
            // Mostly static with periodic kicks
            int kickIndex = Mathf.FloorToInt(t / perturbationInterval);
            float kickPhase = (t % perturbationInterval) / perturbationDuration;
            
            float perturbation = 0f;
            if (kickPhase < 1f)
            {
                // Smooth kick: sin^2 for smooth onset/offset
                perturbation = Mathf.Sin(kickPhase * Mathf.PI) * perturbationMagnitude;
            }

            // Apply perturbation to middle control points (index 1 and 2 for 4-point rail)
            int midStart = Mathf.Max(1, controlPoints.Length / 2 - 1);
            int midEnd = Mathf.Min(controlPoints.Length - 1, controlPoints.Length / 2 + 1);

            for (int i = 0; i < controlPoints.Length; i++)
            {
                if (controlPoints[i] == null) continue;
                Vector3 pos = initialPositions[i];
                
                if (i >= midStart && i <= midEnd)
                {
                    // Random direction for each kick
                    float dir = (kickIndex % 2 == 0) ? 1f : -1f;
                    pos.x += dir * perturbation;
                    pos.z += dir * perturbation * 0.5f;
                }
                controlPoints[i].position = pos;
            }
        }

        private void AnimateCircularOrbit(float t)
        {
            float angle = t * orbitFrequency * 2f * Mathf.PI;
            float centerX = Mathf.Cos(angle) * orbitRadius;
            float centerZ = Mathf.Sin(angle) * orbitRadius;

            for (int i = 0; i < controlPoints.Length; i++)
            {
                if (controlPoints[i] == null) continue;
                Vector3 pos = initialPositions[i];
                pos.x += centerX;
                pos.z += centerZ;
                pos.y += orbitHeight;
                controlPoints[i].position = pos;
            }
        }

        private void AnimateCombined(float t)
        {
            // Vertical wave + horizontal sway
            for (int i = 0; i < controlPoints.Length; i++)
            {
                if (controlPoints[i] == null) continue;

                float u = controlPoints.Length > 1 ? i / (float)(controlPoints.Length - 1) : 0.5f;
                
                // Vertical wave
                float wavePhase = t * waveFrequency * 2f * Mathf.PI - u * waveSpeed * 2f * Mathf.PI;
                float waveOffset = Mathf.Sin(wavePhase) * waveAmplitude;
                
                // Horizontal sway (same phase for all points = rigid sway)
                float swayOffset = Mathf.Sin(t * swayFrequency * 2f * Mathf.PI + swayPhaseOffset) * swayAmplitude;

                Vector3 pos = initialPositions[i];
                pos.x += swayOffset;
                pos.y += waveOffset + waveVerticalOffset;
                controlPoints[i].position = pos;
            }
        }

        private void OnValidate()
        {
            // Clamp values
            waveAmplitude = Mathf.Max(0f, waveAmplitude);
            waveFrequency = Mathf.Max(0.01f, waveFrequency);
            swayAmplitude = Mathf.Max(0f, swayAmplitude);
            swayFrequency = Mathf.Max(0.01f, swayFrequency);
            perturbationInterval = Mathf.Max(0.1f, perturbationInterval);
            perturbationMagnitude = Mathf.Max(0f, perturbationMagnitude);
            perturbationDuration = Mathf.Max(0.01f, perturbationDuration);
            orbitRadius = Mathf.Max(0f, orbitRadius);
            orbitFrequency = Mathf.Max(0.01f, orbitFrequency);
        }

        // Public API for external control (e.g., from test script)
        public void SetMode(AnimationMode newMode) => mode = newMode;
        public AnimationMode GetMode() => mode;
        public void SetWaveParams(float amplitude, float frequency, float speed)
        {
            waveAmplitude = amplitude;
            waveFrequency = frequency;
            waveSpeed = speed;
        }
        public void SetPerturbationParams(float interval, float magnitude, float duration)
        {
            perturbationInterval = interval;
            perturbationMagnitude = magnitude;
            perturbationDuration = duration;
        }
    }
}