# Solver V0.2 — constrained four-sided quad patches

Status: **experimental solver, not an automatic remesher for arbitrary meshes**.

## What has been implemented

1. Import one closed GuideGraph loop and four ordered path positions marking corners.
2. Split the boundary loop into four hard boundary polylines.
3. Resample each side with arc-length parameterization at user-selected U/V quad counts.
4. Generate an all-quad grid using a 3D Coons transfinite interpolation patch.
5. Preserve all four boundary polylines exactly at the resampled grid locations.
6. Optionally relax **interior vertices only** with Jacobi Laplacian iterations, never moving hard boundaries.
7. Reject collapsed, inverted or strongly folded adjacent quads.
8. Report boundary deviation, worst side aspect ratio, mean absolute corner cosine and minimum face area.
9. Export actual OBJ quad faces with `guidedquad_cli patch-loop`.

## Example

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure

./build/guidedquad_cli patch-loop tests/square.obj square_quads.obj 0 1 2 3 8 8
```

Output: 81 vertices and 64 genuine quad faces; source boundary path must have one closed loop and four ordered corners. **Corner indices refer to positions in the extracted guide path, not OBJ vertex numbers.** 

A Face Set or standard Blender Color Attribute adapter has not yet been developed. The CLI mode currently extracts the outer mesh border, not painted internal guide loops.

## What this proves — and does not prove

- **Proves**: a simple independently implemented solver can generate quad topology which follows arbitrary piecewise-linear border guides; it can preserve hard border constraints and reject common local degeneracies.
- **Does not prove**: superiority to Exoside; good automatic patch decomposition; global loop quality; automatic 4-RoSy singularity placement; exact curved source surface reprojection; UV preservation; local retopology editing.
- In the first unguarded curved-boundary example, 25 relaxation iterations reduced mean |cos(angle)| from **0.202441 to 0.159981** (better) but raised worst side aspect ratio from **1.22536 to 1.43089** (worse), with boundary deviation exactly 0. These are measured synthetic test data, not an Exoside comparison.
- Added quality-guarded **backtracking** relaxation: test smaller steps until angle quality improves without exceeding a configurable **2%** worst aspect growth; preserve baseline if none qualifies. On the same synthetic boundary example this reduced angle error to **0.198131** (a **2.1%** relative improvement) with worst aspect **1.24986** (within 2% of baseline) and zero boundary error. The more aggressive unguarded smoothing improves angles further but distorts lengths significantly. These values must be retested on real meshes.

## Next technical steps

- Add input from Blender standard per-corner attributes and Face Sets.
- Split complex painted loops into a compatible patch graph with corner insertion.
- Add orientation fields and singularity planning, independent from density fields.
- Implement hard/soft interior directional constraints and integral patch-side counts.
- Add source triangle closest-point projection accelerated by a BVH, then compare error/quality against non-projected patches.
- Improve the initial quality-guarded Laplacian iteration with surface-aware optimization, line search, element-quality objectives and reliable termination; current guard only limits aspect regression and checks local folds.
- Establish a fixed mesh corpus with curved mechanical/organic shapes and publish measured comparisons.

## Performance budget

The current patch generation uses O(U*V) memory and work, plus O(iterations*U*V) smoothing and O(U*V) validation; boundary curves are resampled in linear time in input points + output samples. This is only a local patch solver; a future global solver must avoid allocating every mesh-level object for each pixel or edge and must bound field iterations.

The Face Set guide extractor uses one sorted contiguous edge incidence array (O(F log F) sort, O(F) storage) rather than one `std::map` allocation per source edge. The contour extractor uses hash-based node deduplication and deterministic threshold tie handling. None of these changes has been measured as a speedup against an external reference yet.

## V0.2.1 guide fidelity and quality audit (experimental branch)

The original V0.2 sampler could remove an intermediate hard-guide vertex and still report `max_boundary_error=0`, because the metric compared the generated border only with the resampled border. Example: input bottom `(0,0) → (0.5,-0.2) → (1,0)` sampled at just one segment gives a straight line and incorrectly appears exact.

**Correction:** `preserve_boundary_vertices=true` by default. All input guide knots are retained at exact coordinates, with additional subdivisions apportioned by segment length. If a side has `N` nondegenerate input segments, `u_segments` or `v_segments` must be at least `N`; otherwise the solver rejects the request with an actionable error. This is not yet an automatic quad-budget redistribution solver.

**Optional approximation:** clients may set `preserve_boundary_vertices=false` explicitly, in which case the diagnostic now measures the maximum distance of original input vertices to the approximated output polyline, in addition to sample matching and any canonicalized corner offsets. This is a vertex-based directional deviation rather than a certified full continuous Hausdorff distance; a full bidirectional continuous metric remains future work. The optional nearest-segment check currently costs O(input-vertices × output-segments) once per boundary, so do not use approximation mode on enormous paths without spatial indexing.

**Quality and CPU:** include minimum signed corner scaled Jacobian (negative/collapsed corners are rejected), and split-triangle diagonal warpage angle. Samples and input deviation are computed once per solve and reused during the backtracking optimizer, removing four repeated boundary resamplings and their allocations per candidate evaluation. This is an algorithmic reduction in repeated work, not yet a measured wall-clock speed-up.

**Regression tests:** sharp corner with insufficient density must fail under strict policy; enough segments preserve the original corner; opt-in approximation must report ~0.2 lost displacement; asymmetric multiknot guides keep original source coordinates; twisted-but-valid quads expose non-zero warpage; bow-tie quads are rejected; identical consecutive guide points are rejected.

**Remaining blockers before precision sculpt remeshing:** global mesh self-intersection detection (BVH), source mesh reprojection and closest-point error, hard/soft interior flow guides, automatic patch construction and singularity placement. Do not interpret this patch solver as an automatic replacement for Exoside.
