# GuidedQuad V0.3 — guarded surface reprojection

**Experimental.** This is NOT an automatic remesher for arbitrary organic models and does NOT yet transfer UVs, color attributes, sculpt layers, or full Multires detail.

The V0.2 four-sided Coons chart can now be projected against a reference *triangulated* high-resolution surface while retaining all artist-defined boundary vertices.

## Algorithm

1. Validate the source reference triangle mesh. Invalid triangle indices or nonfinite vertices fail. Degenerate geometric triangles are skipped and counted; an entirely degenerate source fails.
2. Build one median-split triangle BVH with cached AABBs/centroids and 8 triangles per leaf. Query branches nearest-first and prune boxes farther than the current closest point.
3. Validate that the incoming quad mesh has the rectangular row-major topology and valid starting orientation expected from the V0.2 patch solver.
4. Query nearest points for interior vertices only. **Never move the outer ring**.
5. Try a full projection step, then halve step size when any quad becomes locally inverted, collapses below a specified fraction of its original triangle area, flips relative to its initial normal, or develops a signed corner Jacobian below threshold.
6. If a valid step exists, accept it and iterate. If not, keep the last valid mesh.
7. Measure exact BVH nearest-surface RMS before and after projection, count moving vertices, triangle tests, AABB checks, and rejected steps.

This uses O(T) reference surface storage, O(T log T) median-split BVH build time (approximately, subject to the standard partitioning implementation), and typically sublinear triangle visits per query on spatially localized geometry. **Worst-case query complexity is still O(T)**. The initial version rebuilds a BVH for each call; retaining a shared BVH across patches is a future optimization.

## Reproducible usage

Build the repository using CMake as described in the root README, then:

\`\`\`sh
./build/guidedquad_cli project-loop \
  tests/square.obj tests/curved_surface.obj \
  projected_quads.obj 0 1 2 3 8 8 2 1
\`\`\`

Parameters: \`border.obj\`, \`reference.obj\`, \`output.obj\`, four ordered corner indices in the extracted border path, U/V segment counts, optional projection iterations and strength. On Windows, the executable may be in \`build/Release/\`.

The command writes genuine \`f i j k l\` quad faces to OBJ and reports the reference RMS change and number of BVH triangle tests. The border OBJ must have exactly one closed outer boundary. Only triangles with positive absolute OBJ vertex indices are accepted as source meshes.

The programmatic C++ entry point is:

\`project_quad_patch_to_surface(reference_surface, coons_patch.mesh, u_segments, v_segments, options)\`

Default steps: 2, strength: 1, max backtracking: 10; quality barriers: min original triangle area 10%, min signed corner sine 0.05, initial-normal dot 0.1.

## Tests

The synthetic test reference surface is a triangulated height field \`z=0.16 sin(pi*x) sin(pi*y)\`, with a flat boundary. The 12×12 quad chart starts flat. After projection, its interior should move towards the reference surface while all boundary points remain unchanged. Regression tests cover: RMS decrease, sufficient BVH query pruning relative to a brute-force upper bound, no-boundary movement, invalid quad-grid topology, bad options, and skipped reference degenerates. A separate CTest CLI test writes the projected OBJ.

The reproducible CI workflow tests Debug and Release on Linux, Windows, macOS and prints query counts and RMS for the synthetic 12×12 case on Linux Release.

## Measured synthetic reference (GitHub Actions Ubuntu Release)

A 12×12 flat patch projected against a 28×28 triangulated height field (1,568 triangles) produced **484 BVH nearest-point queries**, examining **13,038 triangles** instead of **758,912 triangle visits** for a brute-force all-triangle scan at the same query count: **98.28% fewer triangle tests**, or about **58.2× fewer triangle tests** (not a wall-clock speedup measurement). Interior-vertex nearest-surface RMS was **0.0830842** before projection and **9.67397e−18** after projection; boundary vertices remained fixed.

The reference surface is smooth and well sampled, so the near-zero final error is not an indicator of equal accuracy on thin or self-occluding organic geometry. Full timings, peak RAM and matched Exoside comparisons remain unmeasured. These values are captured in GitHub Actions V0.3 projection test output.

## V0.3.1 audit: cached queries and bounded correspondence (experimental)

**Confirmed inefficiency fixed:** the first projection iteration used to repeat all the nearest-triangle queries that had just been used for the initial RMS measurement. We now cache the original nearest targets for the first iteration. When zero steps are requested, or no step is accepted, the initial RMS is reused as the final RMS without an extra BVH pass. Geometry is unchanged by this optimization.

**Measured on the same Linux Release CI synthetic fixture:** V0.3: 484 nearest queries, 13,038 triangle tests; V0.3.1: **363 queries, 7,639 triangle tests**, with the same final RMS (~9.67e-18). This represents 25% fewer queries and ~41.4% fewer triangle tests on that one case, **not an independently measured wall-clock speedup**; query traversal cost and BVH build cost still matter.

**Correspondence safety:** `SurfaceProjectionOptions::max_projection_distance` (positive real-world units, default infinity) can now limit each candidate closest-surface target's distance from the *original* patch vertex. Distant targets are skipped, not partially snapped; `distance_rejected_targets` counts rejections. This radius is checked against the original position on every iteration, preventing slow cross-surface drift across successive steps. Set a physically meaningful radius when processing thin parts and nearby disconnected scan components. The default unlimited behavior preserves existing API results unless the caller opts in.

**Regression coverage:** remote target surface rejected without moving the chart, valid targets allowed only within an original-space radius, no-op projection uses one query per interior vertex, and duplicate-first-query removal preserves test geometry.

**Remaining risks:** this distance guard cannot resolve two different source layers inside the allowed radius; an oriented/feature-aware correspondence strategy is still required. The current CPU BVH is reconstructed for each new patch; persistent multi-patch acceleration is planned. Also missing: global quad self-intersection detection (as opposed to local Jacobian guards), multiscale benchmark with measured runtime/RAM, UV and color transfer.

## What is still not solved

- Global mesh self-intersection, nearest-point ambiguity across folds and thin features, topology-sensitive nearest surface association or projection along intended normals.
- Optimal placement of extraordinary vertices and edge-loop continuation between patches.
- Construction of quad patches automatically from Face Sets, Polypaint or manual guide curves, including internal flow constraints and arbitrary boundary topology.
- Reuse of the same BVH across many adjacent patches.
- Certified two-sided Hausdorff distance between full reference surface and resulting quad mesh; current RMS is **from interior vertices to the reference surface only**, not a surface-wide symmetric error.
- UV / per-corner attributes / high-resolution detail transfer and Blender standard addon.

A single synthetic dataset and successful build is not evidence of superior quality or speed over Exoside QuadRemesher. Full geometry benchmarks on real organic meshes remain mandatory.
