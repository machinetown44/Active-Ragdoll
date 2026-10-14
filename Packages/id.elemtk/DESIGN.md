# DESIGN — Element Toolkit (`id.elemtk`)

A generic UI Toolkit (UIElements) framework of reusable editor primitives for
building polished tool windows **on top of existing Unity asset workflows** —
e.g. wrapping the Motion Matching (MxM) pre-processor with an automated
jump-replacement workflow.

## Why

Unity's editor-extension story has two extremes: raw `CustomEditor`/IMGUI
(one asset, one window, no composition) and fully custom UIFrameworks
(high cost, no reuse of existing editors). What is missing is the middle
ground: **wrapping** — hosting existing editors inside new layouts and
reacting to their changes without touching the wrapped asset's code.

Element Toolkit provides the generic, MxM-agnostic parts of that pattern.
Domain-specific logic (MxM composite/event editing) lives with the domain
tool, not here.

## Design philosophy

1. **Host, don't re-implement.**
   An inspector is just an `Editor` object. `Editor.CreateEditor(target)`
   gives the *same* editor Unity's own Inspector window would show, including
   its serialized-property machinery. Hosting it (`EmbeddedInspector`) means
   the wrapped asset keeps its authoring UI, and the wrapper adds layout,
   workflow, and automation around it — no re-implementation, no drift.

2. **React by observation.**
   Wrapper windows never modify the wrapped framework. Change detection is
   observation-only (`ChangeMonitor`): cheap version hashes polled on the
   editor update loop, plus `Undo.postprocessModifications` /
   `undoRedoPerformed` hooks. Both work across *any* editor that mutates the
   data — stock inspector, wrapper controls, undo — because everything shares
   the same object and the observer watches the data, not the UI.

3. **Data first.**
   Authoring hubs point downward (hub → modules → sub-assets). Runtime
   artifacts are self-contained with no back-links. Tools therefore walk the
   graph from the hub (`SubAssetUtility`) and treat baked/runtime assets as
   products, never as sources of authoring truth.

4. **Modules, not layers.**
   Feature sets are self-contained modules (`Modules/`), each with its own
   namespace, folders, and samples — not architectural layers. The original
   Element Animation Toolkit now lives as `Modules/Animation` (namespace
   `instance.id.EATK`, frozen — see Legacy below). The generic framework
   primitives live in `Runtime/Framework/` (namespace `id.elemtk.Framework`).

5. **Legacy is frozen, not migrated.**
   Code forked from instance.id/ElementAnimationToolkit keeps its original
   namespaces (`instance.id.EATK`, `instance.id.EATK.Extensions`) and lives
   in `Runtime/Legacy/` + `Modules/`. It is only touched to fix compile
   errors (Unity 6 API migrations, e.g. `GetInstanceID` → `GetEntityId`).
   No renaming, no "improvements" — zero-risk to keep working, trivial to
   delete later.

6. **Editor-only by construction.**
   Everything in `Framework/` is `#if UNITY_EDITOR`. The package is an
   editor-tooling library; no runtime scene code is planned here.

## Structure

```
Runtime/
  Framework/        Generic primitives (id.elemtk.Framework)
    EmbeddedInspector   Host an asset's stock inspector in any layout
    ChangeMonitor       Asset change events via polling + Undo hooks
    SubAssetUtility     Sub-asset census, container search, version hashes
  Modules/
    Animation/          Chainable UIElements animations (legacy EATK)
    Elements/           Animated elements (foldouts, labels, text fields)
    Tasks/              Editor task sequencing (legacy)
  Legacy/              Copied instance.id Extensions support code (frozen)
  Util/                Misc utilities
```

## Roadmap

- MxM wrapper (external tool project): jump census window over
  `MocapPreProcessData` — composites → event markers, strip/add, bake via
  the pre-process inspector, using EmbeddedInspector + ChangeMonitor.
- Framework additions happen **only when a consuming tool needs them**
  (requirement-driven, no speculative API).
