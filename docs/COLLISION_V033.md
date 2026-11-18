# GuidedQuad V0.3.3 — global nonadjacent quad collision guard

**Experimental.** This is a geometric safety check, **not** a complete automatic retopology solver. It does not compute new edge flow or repair intersections by changing topology.

## Implemented

- `find_quad_self_intersections(QuadPatchMesh, max_reported_pairs=16)` uses a median-split BVH over quad AABBs, each quad split into two triangles.
- Narrowphase uses triangle-triangle separating axis tests (triangle face normals, all 9 edge-edge axes, plus edge-vs-normal axes for coplanar overlap).
- The mesh is normalized to a unit-scale bounding box for collision predicates (supports various reasonable input scales); near-touching cases are deliberately conservative.
- Returns pairs of intersecting **topologically nonadjacent** quads plus counts of BVH node-pair visits, candidate quad pairs and triangle-pair tests.
- Pairs sharing any vertex index are *excluded*. This avoids false alarms at legitimate mesh edges/corners, but the first implementation can miss an illegal overlap between topologically adjacent quads. That remains an acknowledged limitation.
- In `project_quad_patch_to_surface`, collision rejection is **enabled by default** for the initial quad mesh and each locally valid projection candidate. Invalid input fails with an actionable exception; a new collision rejects the candidate step and the backtracking loop tries a smaller displacement. A candidate without self-intersection is accepted.
- To disable these checks for diagnostics/performance experiments, set `SurfaceProjectionOptions::reject_self_intersections=false` explicitly. This removes the guarantee and is not recommended for final assets.

## Standalone validation

```bash
./build/guidedquad_cli check-quads tests/separated_quads.obj
./build/guidedquad_cli check-quads tests/colliding_quads.obj
```

The command supports positive-index all-quad OBJ faces and outputs detected pair indices and broadphase counts. Exit code **0** means no collision found; **3** means at least one colliding nonadjacent quad pair; malformed inputs and other failures return **1**. The 16-pair report limit bounds work on badly overlapping meshes.

## Tests

- Two nonadjacent quads intersecting through different 3D planes.
- Coplanar overlapping but independent quad faces.
- Spatially separated quads.
- Regular connected quad grid (no false alarm for shared vertices/edges).
- A two-turn annular regular chart that overlaps itself **without locally inverted quads** (detected and rejected as projection input).
- Valid but extremely small/large coordinate scales after normalization.
- Invalid indices and nonfinite source coordinates.
- CLI exit codes for clean and colliding quad OBJ fixtures.

The existing GitHub Actions suite runs Linux, Windows and macOS Debug/Release plus Linux ASan/UBSan; check the active run before treating this version as verified.

## Performance and accuracy

The broadphase prunes distant quad pairs, avoiding an unconditional O(Q²) scan. Construction is approximately O(Q log Q) time and O(Q) auxiliary storage. Worst-case highly overlapping geometry remains O(Q²) candidate comparisons. Current code rebuilds the quad BVH per projection candidate: this prioritizes correctness; local updates and BVH refitting will be needed for high-resolution real-time workflows.

These are floating-point SAT predicates with a fixed conservative tolerance **after normalization**, not exact robust predicates. Avoid using this result as a mathematically certified watertight/collision-free guarantee at arbitrary geometric scales, extreme slivers or near-contact tolerances.

## Remaining work

1. **Adjacent fold-through:** distinguish legitimate edge/vertex contact from overlaps involving topologically adjacent quads.
2. **Cross-patch collisions:** this detector checks a single supplied QuadPatchMesh; detecting intersections between separately projected patches requires assembling their quads (or an indexed set of patches).
3. **Numerical robustness:** exact/adaptive orientation predicates and pathological skinny faces.
4. **Feature-aware correspondence:** distance, normals and source connected-component selection for thin, similarly oriented disconnected scan layers.
5. **Performance:** persistent output-quad BVH/refit or local dirty-face collision checking, wall-clock benchmark and peak memory profiling on real sculpt models.
6. Automatic patch decomposition, field-driven edge flow, Blender standard extension and UV/detail transfer.

### Important distinction

`reject_self_intersections` prevents a locally valid projection step from *introducing* some kinds of nonadjacent quad overlaps. It does **not** guarantee that a deforming mesh is globally non-self-intersecting, nor does it repair already-invalid guide geometry. Unlike the reference-surface BVH, the output collision BVH is currently rebuilt on each candidate.
