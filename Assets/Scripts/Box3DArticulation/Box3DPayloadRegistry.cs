using System;
using System.Collections.Generic;
using Box3D.Hybrid;
using UnityEngine;

namespace Box3DArticulation
{
    /// <summary>
    /// Registry of gameplay-attached payload bodies (weights welded to the
    /// character at runtime - pickups, equipment). The rig's own gravity
    /// feedforward and playback feedback loops are closed over the rig node
    /// bodies only; payloads registered here are folded into both:
    ///  - HumanoidMotionDriver: payload appended to the host node's distal
    ///    list, so the joint's composite inertia and gravity load feedforward
    ///    include it.
    ///  - Box3DPhysicalAnimator: payload mass/velocity included in the COM
    ///    velocity the playback-speed feedback tracks.
    /// Auto-discovery (context menu) scans the rig for Box3DBodies that are
    /// not rig node bodies and registers them under their nearest ancestor
    /// node body.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class Box3DPayloadRegistry : MonoBehaviour
    {
        [Serializable]
        public sealed class Payload
        {
            [Tooltip("The attached (non-rig) body carrying the extra weight.")]
            public Box3DBody body;
            [Tooltip("Rig node the payload hangs from (its joint carries the load).")]
            public string hostNode;
        }

        [SerializeField, Tooltip("Rig the payloads attach to (discovery scope + node-name source).")]
        private Box3DRigBuilder rig;
        [SerializeField, Tooltip("Driver whose gravity feedforward should include payloads. Empty = resolved automatically.")]
        private HumanoidMotionDriver driver;
        [SerializeField, Tooltip("Physical animator whose playback feedback should include payloads. Empty = resolved automatically.")]
        private Box3DPhysicalAnimator physicalAnimator;

        [SerializeField, Tooltip("Registered payloads (edit-time persistence + runtime state).")]
        private List<Payload> payloads = new List<Payload>();

        public IReadOnlyList<Payload> Payloads => payloads;
        public Box3DRigBuilder Rig => rig;

        private void Awake() => ResolveRefs();

        private void OnEnable() => PushAll();

        private void ResolveRefs()
        {
            if (driver == null) driver = FindFirstObjectByType<HumanoidMotionDriver>();
            if (physicalAnimator == null) physicalAnimator = FindFirstObjectByType<Box3DPhysicalAnimator>();
        }

        /// <summary>Register (or update the host of) a payload body and push
        /// it to the consumer loops. Idempotent per body.</summary>
        public void Register(Box3DBody body, string hostNode)
        {
            if (body == null || string.IsNullOrEmpty(hostNode)) return;
            for (int i = 0; i < payloads.Count; i++)
            {
                if (payloads[i].body == body)
                {
                    payloads[i].hostNode = hostNode;
                    Push(i);
                    return;
                }
            }
            payloads.Add(new Payload { body = body, hostNode = hostNode });
            Push(payloads.Count - 1);
        }

        /// <summary>Remove a payload (e.g. dropped item) and update the loops.</summary>
        public void Unregister(Box3DBody body)
        {
            for (int i = payloads.Count - 1; i >= 0; i--)
            {
                if (payloads[i].body != body) continue;
                payloads.RemoveAt(i);
                driver?.DetachPayload(body);
            }
        }

        /// <summary>Drop every payload whose body has been destroyed.</summary>
        public void PruneDestroyed()
        {
            for (int i = payloads.Count - 1; i >= 0; i--)
            {
                if (payloads[i].body == null) payloads.RemoveAt(i);
            }
        }

        /// <summary>Re-push everything (bind refresh, ref re-resolution).</summary>
        public void PushAll()
        {
            ResolveRefs();
            PruneDestroyed();
            for (int i = 0; i < payloads.Count; i++) Push(i);
        }

        private void Push(int index)
        {
            var p = payloads[index];
            if (p.body == null) return;
            driver?.AttachPayload(p.body, p.hostNode);
            // The animator reads the registry list directly per-step.
        }

        // --- auto-discovery -------------------------------------------------

        [ContextMenu("Auto-Discover Attached Bodies")]
        private void AutoDiscover()
        {
            if (rig == null || !rig.IsBuilt)
            {
                Debug.LogWarning("[Box3DPayloadRegistry] no built rig assigned - discovery aborted", this);
                return;
            }
            ResolveRefs();

            // Rig node bodies, keyed by instance for ancestor matching.
            var nodeBodies = new HashSet<Box3DBody>(rig.Bodies);

            int added = 0;
            foreach (var body in rig.transform.GetComponentsInChildren<Box3DBody>(false))
            {
                if (body == null || nodeBodies.Contains(body)) continue;
                if (IndexOf(body) >= 0) continue; // already registered

                string host = FindHostNode(body.transform, nodeBodies);
                if (host == null)
                {
                    Debug.LogWarning($"[Box3DPayloadRegistry] '{body.name}' has no rig node body among its ancestors - skipped", this);
                    continue;
                }
                payloads.Add(new Payload { body = body, hostNode = host });
                Push(payloads.Count - 1);
                added++;
            }
            Debug.Log($"[Box3DPayloadRegistry] auto-discover: {added} new payload(s), {payloads.Count} total", this);
        }

        /// <summary>Nearest ancestor (inclusive of self) that is a rig node
        /// body; its GO name == node name (builder naming).</summary>
        private string FindHostNode(Transform t, HashSet<Box3DBody> nodeBodies)
        {
            for (var cur = t; cur != rig.transform; cur = cur.parent)
            {
                if (cur == null) return null;
                var b = cur.GetComponent<Box3DBody>();
                if (b != null && nodeBodies.Contains(b)) return cur.gameObject.name;
            }
            return null;
        }

        private int IndexOf(Box3DBody body)
        {
            for (int i = 0; i < payloads.Count; i++)
                if (payloads[i].body == body) return i;
            return -1;
        }
    }
}
