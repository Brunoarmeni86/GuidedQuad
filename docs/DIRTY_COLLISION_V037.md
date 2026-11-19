# GuidedQuad V0.3.7 — dirty-quad collision checks (experimental)

This branch is stacked on the V0.3.6 refittable collision BVH. It is not a full automatic remesher, and it is not merged to the stable \`main\` branch or Blender Sculpt Extended.

## Safety invariant

If a quad mesh was verified **collision-free**, and only a subset of its vertices changes, **any newly introduced intersection must involve at least one quad incident to a changed vertex**. Unchanged/unchanged quad pairs were already safe. The new checker uses this invariant; it does not simply ignore arbitrary static triangles without a baseline.

- \`PreparedQuadCollisionChecker::check_initial()\` verifies the whole initial mesh.
- \`check_positions_dirty(candidate_positions, changed_vertex_ids, limit)\` enforces that **all actual coordinate changes** are included in \`changed_vertex_ids\` (using an exact comparison against its last accepted positions). Duplicate IDs and extra unchanged IDs are harmless; missing or out-of-range IDs are rejected.
- Each changed vertex maps to incident quads with a topology-only adjacency cache built once.
- The collision BVH is refitted to the candidate coordinates, then **each dirty quad** queries overlapping BVH nodes, testing it against **all** potentially intersecting quads, including unchanged ones. Dirty-dirty pairs are visited only once.
- An intersecting candidate is **not accepted**: the BVH rolls back to its last verified clean geometry; next backtracking attempts remain safe. Invalid geometry is rejected by the existing refit validation before touching the cached tree.
- \`check_positions\` still supports complete checks. A prior failed complete check requires another successful global check before incremental validation can resume.
- \`dirty_face_queries\` reports how many dirty quads were traversed. \`refit_count\` includes rollback refits, while \`dirty_update_count\` counts attempted nonempty dirty evaluations.

## Integration with the source-surface projection

The V0.3.7 projection automatically chooses the incremental query only when fewer than 1/6 of the patch's vertices actually move **and** the current collision-checker state is clean. Dense updates fall back to the complete BVH query, avoiding potentially excessive per-dirty-quad traversals. Collision BVH state is reused across accepted candidates; rejected dirty updates roll back before trying a smaller displacement.

Projection stats include \`collision_dirty_passes\` and \`collision_dirty_face_queries\`. The existing source-reference BVH and quad collision BVH remain independent.

## Performance: what is and is not optimized

- The expensive **global collision traversal** can now be replaced by localized dirty-face traversals.
- This implementation still performs **O(V) input validation** to ensure no changed vertex was omitted and **O(V+F) refit** of all BVH bounding boxes for each candidate. It is not yet a truly local BVH update. This protects correctness while establishing a reusable API for subsequent optimization.
- Dirty checking should help most on brushes or sparse deformation updates that change only a small part of the patch. For broad deformations the full BVH pass remains preferable.
- The test suite contains a reproducible 48×48-quad, 24-frame sparse-center update microbenchmark. The Ubuntu Release CI prints wall-clock durations, BVH node visits and SAT triangle tests for both methods. Treat a single-run timing as indicative, not an Exoside performance comparison.

## Precision

The earlier 2D orientation helper ran all calculations with \`long double\`. A new **filtered predicate** first estimates a double determinant; if it is well above a conservative floating-point cancellation bound, it uses the fast result, otherwise it recomputes with \`long double\`.

This is **not adaptive exact arithmetic**. In particular, Microsoft's C++ toolchain commonly provides only double-equivalent \`long double\` precision. Triangle-triangle SAT, edge-plane intersections, near-coplanar slivers and continuous collisions between frames remain unproven in the mathematical sense. Robust exact fallback predicates, as well as a feature-scale tolerance policy, remain separate work items.

## Regression coverage

- Dirty checks without a verified clean initial mesh must fail.
- Omitting an actually changed vertex must fail **before** changing the BVH state.
- Moving a dirty patch across a stationary patch must detect the intersection.
- A rejected collision attempt must leave the previous validated geometry unchanged; a later valid update must succeed.
- Two dirty faces are compared only once; all changed/unchanged interactions remain visible.
- Repeated sparse deformations of a 36×36 quad grid produce the same colliding pairs as a freshly rebuilt full BVH.
- The microbenchmark compares a refitted full query and a refitted dirty query on the same 48×48 quad mesh over 24 deformation frames; wall-clock timing is diagnostic, while the relative BVH node visits are regression-checked.

## Further work

1. Refit **only** dirty faces and their parent BVH nodes, rather than all quad boxes.
2. Provide a reliable caller-owned dirty-vertex source (e.g. Blender sculpt stroke changes) and optional conservative dirty-bound validation, rather than always scanning all vertices.
3. Add adaptive/exact orientation predicates, continuous collision detection and robust handling for invalid skinny faces and near-tangencies.
4. Benchmark large organic sculpts and mixed sparse/dense brush strokes including memory overhead and frame-time percentiles, not only tiny synthetic grids.
5. Automatic patch decomposition, field alignment (4-RoSy), feature guides, UV/color/detail transfer, standard Blender addon.
