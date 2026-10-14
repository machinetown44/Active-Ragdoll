# ActiveRagdoll

Standalone Unity prototype: an active ragdoll character driven by Motion Matching,
simulated with Box3D. Extracted from the Prototyping workspace as a clean-room
project — it contains only what it needs and nothing cross-references the
original workspace.

**Status:** work in progress. The humanoid motion driver is dead code (it does
not work); the current approach is rigid rotation of articulated bodies with
root-velocity anticipation and root-COM coupling.

## Requirements

- Unity **6000.5.6f1** (URP).
- No other toolchain needed to run: native physics binaries ship in-repo.

## Layout

| Path | Contents |
|---|---|
| `Assets/Scripts/Box3DArticulation/` | Active ragdoll driver (Box3D articulation + MxM glue) |
| `Assets/Plugins/MotionMatching/` | MxM plugin + demo scenes (`Demo/Scenes`) and mocap data |
| `Assets/Settings/` | URP assets (PC/Mobile), volume profiles |
| `Assets/Plugins/Harmony/` | 0Harmony, needed by `id.elemtk` |
| `Packages/com.suvitruf.box3d` | Box3D physics — **embedded fork**, see below |
| `Packages/id.elemtk` | Embedded tooling package (WidgetMapProbe) |
| `Packages/com.popcron.gizmos` | Embedded gizmo drawing |

## Box3D fork provenance

`Packages/com.suvitruf.box3d` is a fork of [Suvitruf/box3d-unity](https://github.com/Suvitruf/box3d-unity),
three commits ahead (articulation rig hooks: `BuildDef` virtual + public `LiveShape`,
`MoverHit`/`PlaneResult` facet-normal mirroring). The native libraries in
`Plugins/` were built from a forked C source, pinned at
`v0.1.0-2-g29bf523` ("Fixes 02 (#31)") — recorded in
`Box3D.Native~/VERSION` together with rebuild instructions.

Upstream is **not sufficient**: the ragdoll driver depends on the fork's hooks.

### Editing the fork

Edit at the canonical location (`E:/Unity/box3d-unity`), then copy back into
`Packages/com.suvitruf.box3d` (exclude `.git` and `Box3D.Native~/build*`).
Rebuilding natives additionally needs the C-source fork checkout noted in
`VERSION`; this is only required after C-side changes.

## Privacy

Private repository. The tree carries MxM demo content and Mixamo-derived mocap,
which are not redistributable.
