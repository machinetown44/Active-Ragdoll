using UnityEngine;

namespace Box3DArticulation
{
    [DisallowMultipleComponent]
    [ExecuteAlways]
    public sealed class DebugViewCapture : MonoBehaviour
    {
        [Header("Capture Settings")]
        [SerializeField] private int captureWidth = 1024;
        [SerializeField] private int captureHeight = 768;
        [SerializeField] private float captureInterval = 0.1f; // seconds
        [SerializeField] private bool accumulateTrajectories = true;

        [Header("Camera Settings")]
        [SerializeField] private float fieldOfView = 60f;

        [Header("Output")]
        [SerializeField] private string outputFolder = "DebugCaptures";
        [SerializeField] private bool saveToDisk = true;

        private Camera captureCamera;
        private RenderTexture renderTexture;
        private Texture2D accumulatedTexture;
        private float nextCaptureTime = 0f;
        private int frameCount = 0;

        private void Awake()
        {
            SetupCamera();
            SetupRenderTexture();
        }

        private void Start()
        {
            // Re-position camera after demo is fully created
            UpdateCameraTransform();
        }

        private void OnDestroy()
        {
            Cleanup();
        }

        private void SetupCamera()
        {
            var camObj = new GameObject("DebugCaptureCamera");
            camObj.transform.SetParent(transform);

            captureCamera = camObj.AddComponent<Camera>();
            captureCamera.clearFlags = CameraClearFlags.SolidColor;
            captureCamera.backgroundColor = new Color(0.1f, 0.1f, 0.12f, 1f);
            captureCamera.fieldOfView = fieldOfView;
            captureCamera.nearClipPlane = 0.1f;
            captureCamera.farClipPlane = 100f;
            captureCamera.cullingMask = ~0;
            captureCamera.useOcclusionCulling = false;
            captureCamera.allowHDR = false;
            captureCamera.allowMSAA = false;
            captureCamera.enabled = false;

            UpdateCameraTransform();
        }

        private void UpdateCameraTransform()
        {
            if (captureCamera == null) return;

            // Find all Box3DBody components to calculate bounds
            var bodies = GetComponentsInChildren<Box3D.Hybrid.Box3DBody>();
            if (bodies.Length == 0) return;

            var bounds = new Bounds(bodies[0].transform.position, Vector3.zero);
            foreach (var b in bodies)
                bounds.Encapsulate(b.transform.position);

            // Also check for rail points (children without bodies), but exclude the capture camera itself
            foreach (Transform child in transform)
            {
                if (child.GetComponent<Box3D.Hybrid.Box3DBody>() == null && child != captureCamera.transform)
                    bounds.Encapsulate(child.position);
            }

            Vector3 center = bounds.center;
            float maxDim = Mathf.Max(bounds.size.x, bounds.size.y, bounds.size.z);
            float distance = maxDim * 1.5f / Mathf.Tan(fieldOfView * 0.5f * Mathf.Deg2Rad);
            distance = Mathf.Max(distance, maxDim * 2f);

            captureCamera.transform.position = center + new Vector3(0, distance * 0.5f, distance);
            captureCamera.transform.LookAt(center);
        }

        private void SetupRenderTexture()
        {
            renderTexture = new RenderTexture(captureWidth, captureHeight, 24, RenderTextureFormat.ARGB32);
            renderTexture.filterMode = FilterMode.Bilinear;
            renderTexture.wrapMode = TextureWrapMode.Clamp;
            renderTexture.Create();

            accumulatedTexture = new Texture2D(captureWidth, captureHeight, TextureFormat.RGBA32, false);
            accumulatedTexture.filterMode = FilterMode.Bilinear;
            accumulatedTexture.wrapMode = TextureWrapMode.Clamp;
        }

        private void Cleanup()
        {
            if (renderTexture != null)
            {
                renderTexture.Release();
                DestroyImmediate(renderTexture);
            }
            if (accumulatedTexture != null)
                DestroyImmediate(accumulatedTexture);
            if (captureCamera != null)
                DestroyImmediate(captureCamera.gameObject);
        }

        private void Update()
        {
            if (Time.time >= nextCaptureTime)
            {
                CaptureFrame();
                nextCaptureTime = Time.time + captureInterval;
            }
        }

        public void CaptureFrame()
        {
            if (captureCamera == null || renderTexture == null) return;

            // Render to render texture
            RenderTexture.active = renderTexture;
            captureCamera.targetTexture = renderTexture;
            captureCamera.Render();
            captureCamera.targetTexture = null;

            // Simple copy to Texture2D
            RenderTexture.active = renderTexture;
            accumulatedTexture.ReadPixels(new Rect(0, 0, captureWidth, captureHeight), 0, 0);
            accumulatedTexture.Apply();
            RenderTexture.active = null;

            frameCount++;

            if (saveToDisk && frameCount % 10 == 0)
            {
                SaveFrame();
            }
        }

        public void SaveFrame(string filename = null)
        {
            if (accumulatedTexture == null) return;

            string folder = System.IO.Path.Combine(Application.dataPath, "..", outputFolder);
            if (!System.IO.Directory.Exists(folder))
                System.IO.Directory.CreateDirectory(folder);

            string path = System.IO.Path.Combine(folder, filename ?? $"debug_capture_{frameCount:D6}.png");
            byte[] bytes = accumulatedTexture.EncodeToPNG();
            System.IO.File.WriteAllBytes(path, bytes);

            Debug.Log($"[DebugViewCapture] Saved frame to {path}");
        }

        public void SaveCurrentFrame()
        {
            CaptureFrame();
            SaveFrame();
        }

        public RenderTexture GetRenderTexture() => renderTexture;
        public Texture2D GetAccumulatedTexture() => accumulatedTexture;
        public int FrameCount => frameCount;

        private void OnValidate()
        {
            if (captureCamera != null)
            {
                captureCamera.fieldOfView = fieldOfView;
                UpdateCameraTransform();
            }
        }
    }
}