# GuidedQuad experimental 4-RoSy flow-field prototype (V0.2 branch)

This code is original C++17 with no external dependencies. It is a **direction solver**, **not yet a quad remesher**.

## Inputs and field representation

- Triangle mesh with consistent winding. Nonmanifold edges, zero-area faces, invalid indices and non-finite coordinates are rejected.
- `FlowConstraint{triangle, direction, weight, locked}`. Each direction is projected onto the face tangent plane. Two directions differing by 90 degrees are **equivalent** (4-RoSy cross orientation).
- `locked=true` means a fixed **face orientation**. Conflicting locked orientations on the *same face* trigger an error instead of being silently averaged. This is **not** an edge-preservation constraint; actual pinned edge loops require a later patch/integer solver.
- `locked=false` is a weighted soft constraint. Zero-weight constraints have no influence.
- Hard/soft **flow** is independent from future blue/red **density** and the target quad count.

## Algorithm

1. Construct orthonormal tangent charts for triangles and shared-edge adjacency.
2. Use the shared edge as a reference axis for phase-transport (`exp(4i(theta_i - theta_j))`), including folded meshes.
3. Accumulate weighted soft guides and exact face-orientation locks as complex 4-angle phases.
4. Initialize connected components using deterministic breadth-first transport from guide seeds.
5. Iteratively smooth with weighted Jacobi and damping; never update locked face orientations. An unseeded component emits a warning and has no semantically prescribed direction.
6. Decode the phase into one of the four equivalent in-plane directions. Report local coherence and final phase change; neither indicates an all-quad result.

The current scheme is a lightweight **reference field propagation**, not a globally optimal geometry-aware solver. It does not minimize singularities, guarantee globally integrable grids, solve integer edge counts, reproduce manual guide strokes as loops, output quads, or demonstrate superiority over Exoside.

## CLI preview

```
guidedquad_cli flow tests/square.obj tests/square_flow.txt preview_flow.obj
```

The text file has one six-column constraint per line: `triangle_index dx dy dz weight locked` with `locked` equal to `0` or `1`. Comment lines begin with `#`. The OBJ output contains one short direction `l` line per input triangle, suitable for inspection; it is NOT topology. This mode currently requires manually prepared per-face guide vectors.

## Tests

`guidedquad_cross_field_tests` covers planar propagation, 90-degree equivalence, exact locks, impossible/invalid constraints, folded triangles, reproducibility, empty guide fields and bad mesh topology.

## Next research

- Convert Face Set boundary curves and paint stroke centerlines to **separate** guide types. A guide may need junction handling and artist-authored singularity placement, not just per-face directions.
- Optimize a 4-RoSy field with explicit singularity placement and curvature, measuring guide residual and holonomy.
- Couple the field to a constrained patch decomposition, integer interval matching, adaptive density and quad extraction.
- Compare exact hard-guide displacement, loops, poles, aspect ratios, runtimes and RAM on the same licensed Exoside test meshes; never copy proprietary implementation.
