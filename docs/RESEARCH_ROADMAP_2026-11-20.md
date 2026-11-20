# GuidedQuad: research-backed topology roadmap (20 November 2026)

**Scope:** research and implementation plan, not a claim of measured superiority over Exoside. The stable `main` branch is unaffected. This note targets the V0.3.9 stack plus the exploratory 4-RoSy PR #16.

## What the current code actually provides

- V0.1 `guide_graph`: region-boundary and scalar contour extraction; source anchors and path classification.
- V0.3.x experimental: a four-sided quad patch generator, strict boundary-knot preservation, guarded interior reprojection, patch-collection seam/collision checks, and a limited Blender diagnostic addon. Not a global automatic quadrangulator.
- PR #16: deterministic, per-triangle tangent 4-RoSy phase propagation with locked face **orientation** and weighted soft guides, plus OBJ preview. It does not constrain a future edge to coincide with any marked feature curve. Global integrability and singularity layout are unimplemented.

**Important blocker:** The strict sampler in `src/patch_solver.cpp` preserves every input boundary vertex and rejects requested segment counts smaller than the input segment count. On high-resolution Face Set boundaries, this can make an otherwise valid desired low-poly result impossible. "Preserve feature geometry" and "preserve all sampled high-res knots" must be separate controls. Do not silently downsample a locked corner or junction.

## Five research findings

1. **Hard feature constraints require chart topology, not just a direction field.** QuadWild (Pietroni et al., 2021) puts sharp feature lines on patch boundaries and permits polygonal patches and intermediate T-junctions, later generating pure quads. Research the *design*, not its GPL-3.0 code. https://github.com/nicopietroni/quadwild
2. **Place non-hard partition paths where the cross field is smooth and away from singularities.** Field Smoothness-Controlled Partition (Liang et al., 2025) reduces distortion from curved, singularity-adjacent chart boundaries. GPL-3.0 research only; its released reference also depends on Gurobi. https://ustc-gcl-f.github.io/projects/FSCQ/FSCQ.html
3. **Quantization and guide feasibility are first-class constraints.** Coudert-Osmont et al. (2023/2024), Quad Mesh Quantization Without a T-Mesh, provides a strategy adaptable to complex feature networks. Penner-coordinate feature-aligned parametrization (2025) explicitly discusses infeasible guide configurations and controlled residual error. https://doi.org/10.1111/cgf.14928 ; https://www.algohex.eu/publications/feature-aligned-parametrization-in-penner-coordinates/
4. **Topology editability should be measured, not guessed.** Microsoft SQuadGen (SIGGRAPH 2026) uses chart distance fields and loop-simplicity metrics for artist-like layout proposals. Its repository is MIT, but its neural pipeline is Linux/CUDA-based, high-resource and optional, not core. https://github.com/microsoft/SQuadGen
5. **Preserve layout connectivity through geometric optimization.** Embedding Optimization of Layouts via Distortion Minimization (2026) emphasizes connectivity-preserving, low-distortion surface embeddings and coarse-to-fine optimization. https://doi.org/10.1111/cgf.70421

Additional inspiration: QuadriFlow global minimal-singularity optimization (MIT): https://github.com/hjwdzh/QuadriFlow ; Instant Meshes interactive orientation and position fields (BSD-style): https://github.com/wjakob/instant-meshes ; NeurCross neural SDF regularization (AGPL-3.0/CUDA): https://github.com/QiujieDong/NeurCross .

## Proposed design contracts

### Constraint graph

Create stable, surface-anchored guide arcs with explicit roles. Do not infer the role solely from the RGB values:
- `HardEdgeLoop`: curve must appear in output as a connected edge path within a specified geometric tolerance (or exactly retain selected authored control points when requested).
- `EdgeRing`: crossing edge direction is constrained, but the original curve is not required to be a topological edge path.
- `SoftFlow`: weighted tangent orientation preference.
- `DensityField`: local target edge length/metric, *not* direction or an edge boundary.
- `PoleAvoidRegion` and optional `PoleSeed`: artist controls for singularity locations.

Input provenance includes Face Set boundary, color-region boundary, hand-drawn stroke **centerline**, sharp normal edge or authored curve. These are different interpretations; a wide painted stroke must not automatically generate two loop guides from its two borders. Preserve curve junctions, hard corners, loop closure and stable anchors separately from high-res intermediate sampling.

### Feasibility and tolerance model

- Strict mode: retain authored vertices/knots and all hard junctions. Reject incompatible low target counts with an actionable explanation.
- Geometric mode: simplify high-res guide samples adaptively against a surface-aware distance tolerance, preserving authored corners, branch nodes, topology, and winding. Reproject subdivision samples to the original feature curve. Report max/mean guide-to-edge deviation.
- Never treat a 4-RoSy orientation lock as a hard edge loop.
- Detect intersecting contradictory hard guides and shared patch boundary count mismatch *before* meshing. Report conflicts with guide IDs and required segment counts.
- Match opposite sides of a four-sided chart (`n0 == n2`, `n1 == n3`), shared adjacent patch edges, and ring count consistency. More general polygonal charts may require deliberate extraordinary vertices.

### Global topology pipeline

1. Extract and classify features; build source-anchored guide graph.
2. Resolve intersections, junction valences, open boundary cases, hard-point priorities; compute chart candidates whose boundaries contain all hard feature curves.
3. Solve a mesh-aware direction field (4-RoSy) using parallel transport, curvature smoothing at multiple scales, hard tangent preferences and soft artist guides. Detect field holonomy/singularities. Limit poles in artist-protected regions; respect global topological requirements.
4. Trace additional *soft* chart boundaries along low-rotation, singularity-avoiding paths (2025 FSCQ principle). Do not relax hard curve geometry.
5. Solve integer subdivision counts, shared-seam compatibility and region density globally. Evaluate optional MIT-licensed **HiGHS** as a MILP solver (https://highs.info/); do not introduce Gurobi runtime licensing.
6. Extend existing `solve_quad_patch` to a multi-chart layout, then support polygonal patches, extraordinary vertices and needed topology changes.
7. Project **interior** vertices to source via the existing guarded BVH, keep hard guide paths fixed, stitch seams, run manifold/collision/quality verification, and transfer UVs/colors/detail in a separate phase.
8. For local edits: mark changed guide-affected charts and recompute those, freezing compatible seam boundary conditions. If a singularity/segment-count change propagates globally, declare a broader solve; do not falsely promise constant-cost local updates.

### Priority gates

**Gate A — small, testable next code milestone:** `GuideArc` types, arc simplification in *geometric tolerance* mode, authored-junction preservation, distinct thick-stroke centerline vs painted-region border tests. Compare high-density guide with 5k/10k requested quads; ensure guide simplification does not silently break intended topological loops.

**Gate B:** two or more constrained adjacent four-sided patches, matching integer seam counts, explicit infeasibility errors, exact preserved geometric hard paths. Baseline CPU tests plus ASan/UBSan on Windows/Linux/macOS GitHub Actions.

**Gate C:** cross-field-driven chart tracing and purposeful placement of poles outside face/eyelid/mouth/joint protection masks. Include path continuity and cycle consistency verification.

**Gate D:** user-facing automatic remesher on at least one complete closed organic mesh, same-mesh Exoside side-by-side and Blender addon preview.

## Benchmark plan and competitive claim

Use a legally licensed, working Exoside QuadRemesher 1.4.x engine when feasible; the previously analyzed 1.2 binary is historical. Exoside documents engine 1.4.3 released 15 October 2026, primarily for platform compatibility fixes: https://exoside.com/quadremesher/quadremesher-troubleshooting/ . No inference about its proprietary remeshing internals.

A third-party UI reference is Rhino's distinct **Approximate / Create Edge Ring / Create Edge Loop** guide influence modes (not a claim that Rhino uses Exoside's engine): https://docs.mcneel.com/rhino/9/help/en-us/commands/quadremesh.htm .

Evaluate matched source, output budget, guide and adaptive settings on: (i) feature ring cylinder; (ii) organic eyelid + mouth ring topology; (iii) ears and branch junctions; (iv) bent limb, local density paint; (v) noisy multires surface; (vi) adversarial incompatible hard guides. Prefer real sculpt data alongside procedural fixtures.

Metrics: exact guide path completion and maximum deviation (mm and normalized by bounding box diagonal), number/placement of extraordinary vertices, loop simplicity/broken loops, flow alignment modulo 90 degrees, per-quad angle/aspect/warpage, self-intersections, manifoldness, Hausdorff-like reconstruction distance, density counts and distributions, wall clock/RAM, failure transparency and reproducibility. **Do not assert superiority without measured results.**

## Legal and integration hygiene

Keep GuidedQuad's core MIT. QuadriFlow and HiGHS are MIT; Instant Meshes BSD-style; Geometry Central MIT. QuadWild, FSCQ and libQEx are GPL-3.0; NeurCross AGPL-3.0. Do not copy code from copyleft repositories into the MIT core without a deliberate license decision. SQuadGen's top-level MIT does not obviate third-party submodule audits. Respect Exoside's licensing; study published information and compare observable behavior/results rather than decompiling protected internals.
