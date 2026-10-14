# MxM Studio — review, architecture, implementation plan

Goal: turn the census+preview+timeline prototype into a scoping workbench over the full
MxM bake pipeline: authoring tree (left), preview + timeline (center), baked-data tree
(right), embedded inspector drawer (far right). Selection drives what every view shows.

## 1. Existing MxM tooling (reviewed)

| Tool | What it does | Clunky part we replace/absorb |
| --- | --- | --- |
| `MxMPreProcessDataInspector` (2834 L) | dataset config, module/category CRUD, **bake button** (writes `MxMAnimData`, sets `LastSavedAnimData`) | flat UX; no cross-view; bake buried in foldouts |
| `MxMTaggingWindow` (4038 L) | per-composite timeline: tag spans, events, footsteps, warps, contacts | one composite at a time, opened per asset; no dataset view |
| `MxMAnimationClipCompositeWindow` (745 L) | composite assembly (before/primary/after, speed mods) | per-asset again |
| `MxMBlendSpaceWindow` (1371 L) | 2D lattice editor + generated events/tracks | per-asset; no preview of member clip at lattice point |
| `MxMAnimationIdleSetWindow` (633 L) | idle set assembly | — |
| `MxMDebuggerWindow` (1848 L) | **runtime** pose/cost debugging (needs an MxMAnimator in scene) | authoring-side pose inspection missing |
| Inspectors (composite/blendspace/idleset/module) | thin, mostly foldouts into the windows above | embedded-inspector target |

Data model (verified in source): authoring root `MxMPreProcessData` → `AnimationModule[]`
(sub-datasets, same shape) → `CompositeCategory[]` → `MxMAnimationClipComposite` /
`MxMBlendSpace` / `MxMAnimationIdleSet` units; units own clips, `EventMarker[]`,
tag/favour/footstep/warp/curve tracks. Bake output `MxMAnimData` → flat `PoseData[]`
(10 Hz, joints+trajectory+favour+tags), `ClipData[]/CompositeData[]/BlendSpaceData[]/
IdleSetData[]` (pose ranges), `EventData[]` (pose-anchored), footstep data. Poses live
only in the bake; authoring assets keep transient `PoseList` (NonSerialized).

### 1.1 Editing-feature gap vs the MxM windows (Studio is read-only today)

From `MxMTaggingWindow` (Window/MxM/Timeline), the composite/blendspace/idleset windows,
and `MxMAnimConfigWindow` — grouped by edit surface. P4 candidates marked.

**Track editing** (core of the tagging window)

- Create tag spans: click/drag on a lane; drag to move; right-click delete; drag edges
  to re-time.
- Rename / add / remove tracks (tag, favour, user).
- Footstep steps: create + edit `Pace` / `Type` (data the Studio drops entirely today).

**Event marker editing**

- Create markers on a composite.
- Drag `EventTime` (the white fire-point line) and band edges (windup / each action /
  follow-through / recovery) to change durations.
- Multi-action markers: add/remove action segments.

**Composite structure** (`MxMAnimationClipCompositeWindow`)

- Create composite from clip (CONTEXT menu).
- Add/remove/reorder segment clips; set primary; edit segment boundaries/lengths.

**Blendspace** (`MxMBlendSpaceWindow`): member clip CRUD, 1D/2D axis parameter values,
 speed mods, generated-clip refresh.

**Idle set** (`MxMAnimationIdleSetWindow`): primary/secondary clips, pose ranges,
 transition timings.

**Curves & misc**: float curve tracks; user tag tracks + naming; global favour tags;
 timing presets (`MotionTimingPreset`).

**Config** (`MxMAnimConfigWindow`, a window not an inspector): pose joints
(= `MatchBones` — defines what matching compares), trajectory point times, calibration.

Out of parity scope: bake trigger (preprocess inspector), `MxMDebuggerWindow`
(runtime-only). Blendspace/idleset/config duplication is LOW value — stock windows
already handle them; Studio P4 focuses on batch ops + the timeline as edit surface.

## 2. Views and scoping semantics

Single `SelectionScope` (plain class, evented):

```csharp
class StudioSelection {
    MxMPreProcessData Dataset;      // root
    AnimationModule   Module;       // null = root-level content
    CompositeCategory Category;
    Object            Unit;         // composite | blendspace | idleset
    int               TrackIndex;   // tag/footstep/warp/curve row on the unit (-1)
    EventMarker       Marker;
    // baked side:
    MxMAnimData       Baked;        // = Dataset.LastSavedAnimData
    int               PoseId;       // -1 unless a pose row/tick selected
    bool              FullClip;     // timeline/preview scope expansion toggle
}
```

Every view subscribes to `SelectionChanged`; each renders what its level allows and
dims/ignores the rest. Scoping = *what the view frames*, fidelity stays full.

| Selection | Preview | Timeline | Baked tree | Inspector drawer |
| --- | --- | --- | --- | --- |
| dataset/module | first composite (entry point) | hidden (no time base) | full tree | module inspector |
| category | first composite in it | that composite | subtree highlighted | category defaults (readonly summary) |
| **composite** | composite segments (current behavior) | full composite; marker lanes; track overlays; pose ticks | its pose range linked | embedded `MxMCompositeInspector` + "Open in MxM Timeline" |
| **blendspace** | picked member clip (lattice → clip; phase 2: drag on lattice) | member clip + generated events | its pose range | embedded `MxMBlendSpaceInspector` |
| **idleset** | primary clip looping | primary clip (+secondary markers later) | its pose range | embedded `MxMIdleSetInspector` |
| marker/event | seek windup start (current behavior) | highlight lane + contacts | linked EventData | event fields inline |
| tag/footstep/warp track | track spans drawn in scene (warp targets as handles) | span overlay highlighted | — | track fields inline (left) |
| **pose (baked)** | pose snapshot + matched-bone skeleton overlay from `JointsData` | playhead snap to pose time | pose row highlighted | pose readout: velocity, favour, tags, trajectory points (right) |
| FullClip toggle on composite | samples whole primary clip | shows primary-clip extent beyond composite slice (dimmed) | — | — |

Dynamic scoping rules (the "fidelity" lever):

- Unit selected → preview/timeline time base = that unit (composite plan / member clip / primary clip).
- `FullClip` expands the time base past the composite slice to the full primary clip
  (the clip participates in MM in full — the composite is only an authoring slice).
- Marker/track selection re-frames but never changes the time base.
- Baked pose selection overrides playhead only when `Baked` maps to the current unit
  (pose → `PrimaryClipId` → clip → unit); otherwise it just previews the pose standalone.

## 3. Filtering

One `StudioFilter` shared by both trees:

- text query (name contains, case-insens),
- type toggles per tree: authoring {modules, categories, composites, events, tags,
  blendspaces, idlesets}; baked {clips, composites, blendspaces, idlesets, events, poses},
- category dropdown (existing) = tree node toggle in P1,
- jump-count filter (existing) stays as an event-marker filter.

Trees filter to matching nodes and reveal ancestors; top line counts are computed by a
single pass over the filtered sets, per type: `Modules 2/2 · Categories 5/7 · Composites
34/148 · Markers 34/204 · Tags 12 · IdleSets 3/3 · Clips 88/412` (visible/total, nonzero
only). Baked tree mirrors: `Clips 412 · Composites 148 · Poses 41,203`.

## 4. Architecture

Window = one `MxMStudioWindow` (replaces `MxMPreprocessWrapperWindow` layout). The **work
grid** is a pannable canvas carrying all tooling panels — trees included. The collapsible
side panes host **legacy editors only** (`EmbeddedInspector` wrapping the stock MxM
editors), stage-split like the original design:

```
[ LEFT pane (legacy)  ] [ WORK GRID (pannable)                    ] [ RIGHT pane (legacy) ]
[ Pre-Bake Editor     ] [ AuthoringTree | Preview | BakedTree    ] [ Post-Bake Editor    ]
[ EmbeddedInspector ⇅ ] [           Timeline (spans work cols)    ] [ EmbeddedInspector ⇅ ]
```

- Selecting an **authoring** node swaps the LEFT pane to its stock editor (unit asset,
  module, or dataset fallback for category-only picks).
- Selecting a **baked** node swaps the RIGHT pane to the `MxMAnimData` inspector.
- Trees live on the grid (not in the panes) so scrubbing width never competes with
  hierarchy navigation; panes collapse to a gutter and are per-side (`Side.Left` mirrors
  gutter/divider order, chevrons, drag direction).
- `InspectorDrawer` (custom) was superseded by the framework `EmbeddedInspector`; the
  file remains only until cleanup.

New/changed files (all `Assets/Scripts/MxMWrapper/Editor/`):

- `StudioSelection.cs` — scope struct + `SelectionChanged`/`TimeChanged` events. Pure data, no Unity UI.
- `StudioFilter.cs` — query + type toggles; `Matches(node)` helpers. Pure data.
- `CensusModel.cs` → grows `TreeModel`: builds authoring nodes (dataset→module→category→unit→tracks/events) with parent links and per-node counts; keeps segment-plan logic.
- `AuthoringTreePanel.cs` (replaces `CensusDataPanel` list) — `UnityEditor.IMGUI.Controls.TreeView` (generic `TreeView<int>`), lazy children, rename context menu, drag-to-category (P4). WORK GRID.
- `BakedTreePanel.cs` — TreeView over `MxMAnimData`: Clips[] → CompositeData/BlendSpaceData/IdleSetData (pose ranges as child rows) → EventData; pose rows virtualized (only render visible range); pose count per range on the row. WORK GRID.
- `CensusPreviewPanel.cs` — scope-aware sampling: add idle-set loop, blendspace member clip, `FullClip` mode, pose overlay (skeleton from `JointsData`, matched bones from dataset `PoseJoints`), track-span scene viz.
- `CensusTimelinePanel.cs` — multi-lane (one row per marker + track overlay lane + pose tick raster), time base follows scope (unit vs FullClip), click lane = select.
- `InspectorDrawer.cs` — SUPERSEDED by framework `EmbeddedInspector` (better lifetime,
  opaque backing, repaint coupling); left instance bound to authoring selection, right to
  baked selection. Delete with the P4 cleanup.
- `MxMStudioWindow.cs` — layout + wiring + top-line counts.

Baked↔authoring mapping (best-effort, flagged in UI): baked `Clips[i].ClipId` →
`AnimationClip` → authoring units referencing it. `CompositeData` has no name/back-ref,
so mapping is by clip identity; show a ⚠ when ambiguous (same clip in several composites).

Perf rules (editor code, but repaints are per-frame): no LINQ/allocs in repaint paths;
trees cache nodes; counts computed on filter/dataset change only; pose raster drawn as
one `EditorGUI.DrawRect` loop over visible ticks; `JointsData` overlay reuses buffers.

## 5. Implementation phases

**P1 — spine (selection + trees + counts) — SHIPPED**

1. ✅ `StudioSelection`, `StudioFilter`, `TreeModel` (authoring nodes).
2. ✅ `AuthoringTreePanel` TreeView replacing census list; dataset/module/category/unit rows;
   `ExpandAll` on load (or persisted expansion); children-list build (foldouts); empty-root guard.
3. ✅ Top-line typed counts; toolbar search field + Filter type-toggle menu (shared filter,
   both trees; baked tree filters clips + rides children on clip visibility; query persists).
4. ✅ Legacy editors in collapsible side panes via `EmbeddedInspector` (NOT the planned
   `InspectorDrawer` — framework component won; drawer file deleted). Pane collapse state,
   tree expansion, and query persist via EditorPrefs.
Also shipped: `BakedTreePanel` (P3.1 pulled early — overview rows only, no pose rows),
work-grid canvas layout, `CensusModel.CollectForUnit` (composites/blendspaces/idlesets
all preview-bindable).
Verify: LSP clean ✅; in-editor walkthrough pending (tree selection → editor swap, preview
binding per unit type).

**P2 — timeline & preview scoping — SHIPPED (unverified in-editor)**

1. ✅ Timeline rebind: unit = time base (`RebindCurrentUnit`); multi-marker lanes (one lane
   per composite event, focused marker highlighted, `+N lanes not drawn` overflow note);
   track overlay lanes (tag/favour/footstep/warp tracks as tag-id-colored spans);
   FullClip dimmed tail (everything past the composite slice dims — replaces the old
   "missing ending" behavior with the header toggle).
2. ✅ Preview: idle-set loop + blendspace member clip (markerless units loop the full
   playable range); FullClip sampling (primary clip whole, composite plan bypassed).
3. ✅ Track scene viz (revised after review): single root trajectory with a header
   color-source dropdown — Phase (default) or any authored tag/favour/footstep track
   (tag-id hue per segment, dim gray in gaps). Replaces the earlier parallel-band
   approach (same path drawn 3× with vertical offsets — confusing). Warp position
   handles NOT feasible from the asset — `WarpPositionTrack` is a plain `(time, id)`
   TagTrackBase; actual warp targets live in the MxM warp module's scene objects.
   Note: footstep tracks are events (`time range, Pace, Type`), NOT foot-bone data;
   spatial matching = root trajectory points + configured `PoseJoints` (`MatchBones`).
Verify: composites, blendspaces, idlesets each preview+scrub correctly; tag bands visible
under trajectory; FullClip toggle extends timeline + trajectory past the slice.

**P3 — baked panel + poses**

1. `BakedTreePanel` over `LastSavedAnimData` + right `InspectorDrawer` instance (`MxMAnimData` inspector / pose readout); empty-state = "bake first" button wired to preprocess inspector's bake flow.
2. Pose rows + range counts; pose → unit mapping (clip identity, ⚠ on ambiguity).
3. Pose selection → preview skeleton overlay (`JointsData` × `PoseJoints`), timeline snap, pose inspector (velocity/favour/tags/trajectory).
Verify: pose click shows skeleton matching the previewed frame; counts match bake log.

**P4 — editing tools (clunk-first)**

Editing parity inventory: §1.1. P4 deliverables, in clunk order:

1. Batch tag apply/clear across selected composites (SerializedObject multi-edit).
2. Batch event default edits (windup/FT/recovery across filtered markers).
3. Bake button + "stale bake" badge (compare LastSavedAnimData import/asset timestamps vs authoring dirty state).
4. Category drag-move for composites; rename propagation.
5. Later/direct-edit tier (timeline as edit surface): tag span CRUD + drag re-time on our
   lanes; event marker drag (`EventTime` + band edges); footstep `Pace`/`Type` readout
   at minimum. Composite assembly likely stays in the stock window.
Verify: edits round-trip through Unity serialization; bake badge flips on change.

Each phase lands LSP-clean and testable independently; P1–P3 deliver the views, P4 the
workflow relief. Old census entry list is superseded by P1 and removed with it.

## 6. P5 — Live pose-data view (trajectory-driven search, animator-decoupled)

A view **into the pose database** with manual trajectory-driven search playback. Tied to
the low-level data model (`MxMAnimData` → `PoseData[]` + `BestPoseSearchJob`), NOT to
`MxMAnimator` — controller/animator logic can change without re-authoring this view.

### Layout (work grid grows two columns)

```
baked  |live    |view
data   |view    |settings
(exists)|timeline|
```

- `LiveViewPanel` — top-down trajectory canvas + skeleton render target, own timeline
  below (independent seek; play = runner ticks).
- `LiveSettingsPanel` — search interval, blend-in time, window/speed params, cache
  controls (invalidate / rebuild).

### Runner model (`LiveSearchSession`, Editor-only)

Tick (fixed step, e.g. search every N frames):

1. `poseNow` = sample last chosen pose at current time (joints only; root ignored).
2. `rootNow` = trajectory spline evaluated at live-timeline seek (simulated root —
   decoupled from clip root motion).
3. `traj` = spline sample window ahead of seek, root-relative to `rootNow`; per-point
   orientation = spline tangent, overridden by manually pinned look directions.
4. `tags` = trigger-lane spans at seek time (required/favour toggles).
5. `BestPoseSearchJob` → `PoseData` (AnimId/AnimType/PrimaryClipId/Time); keep pose until
   next search step; blend-in lerp ~0.15 s on pose jump (else pops).

Render: sample chosen clip forward onto the preview rig; place rig at `rootNow`,
orientation from look pins / tangent.

### Cache

Decision sequence `[(stepTime, chosenPoseId, blendFrom)]`. Scrub = find last decision ≤
seek, replay sampling forward (no re-search). Invalidate on: spline edit, trigger edit,
seek-past-end, database reload. v1 may re-run the loop fast on scrub; sequence cache is
the optimization.

### Trajectory model

Ordered points on the live canvas (top-down). Each point: position (draggable), look
direction (auto = spline tangent; right-click pin → free angle, drawn differently,
never re-derived when spline updates). Velocity for the cost = spline derivative ×
speed parameter.

### Trigger timeline

Lanes of tag spans (required tags gate the search, favour bias it) — the same `ETags`
data the search consumes. Drag-create spans, drag-move, right-click delete (mirror
tagging-window interactions, clunk-first).

### Out of scope for v1

Events/contact warp (animator-level), foot phase/locking, idle-set loop handling,
blend spaces' runtime weighting pass (chosen member plays straight).

### Gotchas logged from design discussion

- Start pose bootstrap: t=0 has no previous pose — seed from selected baked clip or
  trajectory rest pose (convention TBD).
- Root-relative frame of the desired trajectory must be built from the *simulated* root
  (pos + yaw) per search — the classic silent-wrongness bug.
- Search interval defines cache granularity; make it a setting.
