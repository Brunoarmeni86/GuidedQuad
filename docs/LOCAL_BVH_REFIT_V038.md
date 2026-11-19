# GuidedQuad V0.3.8 — local quad-BVH refit with conservative full fallback

**Experimental**; stacked on V0.3.7, with no change to the stable `main` branch or Blender Sculpt Extended.

## Correctness audit before optimization

The pre-existing `QuadBvh::refit` rebuilt all quad boxes and renormalized **every vertex** based on a potentially changing world-space bounding box. Naively recomputing only dirty quad boxes under a newly calculated normalization would mix two incompatible coordinate frames, which could silently misclassify intersections.

The new local path **keeps the current normalized frame fixed** while a dirty update remains within a conservative normalized envelope (`[-0.25, 1.25]` on all axes). The old frame is retained for unchanged vertices and unchanged BVH boxes. When dirty coordinates escape this envelope (or are not representable), it falls back to the fully validated/global refit, which recomputes the current normalization consistently. This threshold is a conservative *implementation heuristic*, not a universal numerical-error certification.

## Local refit design

- `QuadBvh` retains each BVH node's parent, a quad-to-leaf lookup, and cached coordinate normalization.
- `check_positions_dirty` already verifies that every actually changed vertex was reported by the caller, and knows which quads are incident to those vertices.
- Local refit transforms only changed vertex positions, validates only the affected quad normals/areas and calculates only their new AABBs.
- It then refits the dirty leaves and their ancestors in **bottom-up order**, preserving all unaffected boxes. Validation occurs before committing any geometry.
- A failed collision check automatically refits the previous verified-clean geometry; if a rollback would escape the active normalization frame, it also uses the full refit fallback. The next attempt starts from a clean state.
- Missing dirty vertices and invalid indices are rejected; an orphan vertex, which belongs to no faces, updates its accepted position without unnecessary BVH work.
- Persistent **epoch stamps** replace per-stroke O(F) allocation/scans for dirty face masks and dirty BVH query membership.

### Public diagnostics

`PreparedQuadCollisionChecker` now exposes `local_refit_count()`, `full_refit_fallback_count()`, and `local_bvh_node_updates()`, in addition to the existing `refit_count()` and `dirty_update_count()`. Full-check and dirty-check APIs retain the same call signatures.

## Measured Ubuntu Release reference benchmark

Synthetic 48×48 all-quad grid: **2,304 quads**, 24 updates to one interior grid vertex (four incident quad faces) in a single GitHub Actions runner:

| Method | Wall-clock time |
|---|---:|
| Full BVH refit + full collision query | **12.8163 ms** |
| Local BVH refit + dirty collision query | **1.68599 ms** |

The local check is **~7.60× faster** on this specific sparse-edit benchmark, a reduction of ~86.8% in elapsed time. During the 24 updates it recomputed **840 total BVH node boxes** instead of performing 24 full-tree refits, and traversed **6,048** BVH nodes rather than **225,408**. SAT triangle-pair tests remained **576 vs 576**; fewer BVH traversals and box updates account for the difference. This is a **single synthetic microbenchmark**, not a measured sculpt brush latency, proof of 7.6× performance on organic meshes, or evidence of superiority to Exoside.

## Verification/regressions

The new tests check:
- Correctness of accepted localized updates versus a rebuilt full BVH.
- Correct rollback after a rejected collision; later safe edits still work.
- Out-of-envelope updates force a consistent **full recenter**; subsequent small edits can return to local refit.
- A colliding out-of-envelope candidate is rejected and restored.
- No BVH work for an isolated orphan vertex while preserving accepted-position bookkeeping.
- More than one dirty vertex/quad, duplicate dirty IDs, and explicit detection of missing dirty vertex IDs.
- A 36×36 iterative deformation and a 48×48-quad sparse benchmark; 13 CTest cases cover the full engine.

The Linux/Windows/macOS Debug/Release matrix and Linux ASan+UBSan must pass for final acceptance. Every code commit is checked via GitHub Actions; see the PR CI report for the last commit status.

## Remaining bottlenecks and limits

- `check_positions_dirty` still **scans all input vertices, O(V)**, to prove caller-supplied dirty indices are complete. This validation is important for safety but leaves a per-call cost; true O(D) input handling would require a trusted change-set API with transaction/consistency guarantees.
- Local BVH box refits are O((dirty leaves + ancestors) log K) due to bottom-up node ordering; full fallback remains O(V+F) and can be preferable for broad pose changes.
- The BVH's initial spatial partition can degrade over very large deformations even when normalized positions remain within the envelope; future maintenance may rebuild adaptively.
- SAT triangle predicates are floating-point approximations (filtered double/long-double for 2D orientation); this is **not exact** robust computational geometry. Near-degenerate slivers, motion tunneling between poses and collision certification for arbitrary pathological inputs remain open.
- Automatic patch decomposition, guided edge flow / 4-RoSy, UV/attribute/detail transfer, full Blender addon, and real-sculpt p50/p95 wall-clock tests remain future milestones.
