# RetargetChains Performance Report

Scope: `MotionTransform.RetargetChains` — profiler showed **~0.6 ms** of `math.mul` per FixedUpdate with **one rig** (~20–40 nodes). No fixes applied yet; this is a source-attribution report.

## Where the time goes

### Pass A — swing-only absolute frames (parent-first walk)

Per non-root node, the hot loop body:

| Expression | math.mul / inverse count |
| --- | --- |
| `math.inverse(p.bindFrame)` → `restDirRel` | 1 inv + 1 mul |
| `math.inverse(p.cmdRot)` → `dirRel` | 1 inv + 1 mul |
| `ShortestArc(restDirRel, dirRel)` | ~2 mul internally (basis build + quaternion from cross) |
| `p.cmdRot * swing * n.restRel` (two chained muls) | 2 mul |
| **Hinge branch (extra)** | |
| `inverse(pp.bindFrame)` ×2 (`axisP`, `restBendRel`) | 2 inv + 2 mul |
| `childRel = inverse(p.cmdRot) * curDirW` | 1 inv + 1 mul (recomputed — identical to `dirRel` above) |
| `SignedAngle(restBendRel, curBend, axisP)` | ~4 mul (2 cross + dot + basis) |

### Pass B — twist + joint targets

Per node with joint:

| Expression | count |
| --- | --- |
| `inverse(p.cmdRot)` → `dirRel` | 1 inv + 1 mul |
| `inverse(p.bindFrame)` → `restDirRel` | 1 inv + 1 mul |
| `ShortestArc` | ~2 mul |
| **Chain-ref branch**: `inverse(p.cmdRot) * FrameFromPrimaryRoll(...)` + `TwistAngle(...)` | 2 inv + ~3 mul |
| **Twist branch**: `swing * restRel * twistLocal` | 2 mul + 1 quaternion-from-axis |
| `relT = tgtA * obs * tgtB` | 2 mul |
| `cmdRot = p.cmdRot * obs` | 1 mul |
| **No-joint branch**: `p.cmdRot * swing * restRel` | 2 mul |

### Root nodes (both passes)

`FrameFromPrimaryRoll` (or `ShortestArc * restFrame` fallback) — ~3–4 mul each, ×2 passes.

## Structural observations (no action taken)

1. **Duplicate inverse/mul work.** `inverse(p.bindFrame)` is recomputed for every child of the same parent; same for `inverse(p.cmdRot)` — recomputed in both Pass A *and* Pass B per child, and twice inside Pass A's hinge branch (`dirRel` and `childRel` are the same product). With ~20–40 nodes this is the largest single multiplier: O(children × per-child inversions) instead of O(nodes) precomputed parent inverses.
2. **Quaternion chain depth.** Every node's final `relT`/`cmdRot` is a 4–6 factor quaternion chain; `math.mul` cost is dominated by count of muls, not the operands.
3. **`ShortestArc`** is called once per node per pass — internally it does a cross product + basis construction, which itself shows up as `math.mul`/`math.cross` in the profiler under the caller's frame.
4. **String-level cost estimate**: ~6–10 quaternion ops (mul/inv) per node per pass, ×2 passes, × ~30 nodes ≈ **400–600 quaternion operations per FixedUpdate**. At ~1 µs each (non-Burst Mono, JIT'd Unity math), that lines up with the observed 0.6 ms.
5. **Extra note:** the additional ~0.3 ms `math.mul` reported elsewhere traces to gizmo/FK expected-pose recomputation (`ComputeExpectedPose` / `OnDrawGizmos` path), which re-derives absolute frames purely for visualization — gated by editor/draw toggles, not needed in player builds.
