# GuidedQuad V0.3.6 — refittable collision BVH and precision audit

**Experimental**, stacked after V0.3.5. No merged changes to the stable `main` branch or Blender Sculpt Extended.

## Bottleneck verified

The previous `PreparedSurfaceProjector::project` rebuilt a complete output-quad BVH on **every** accepted or rejected candidate after the local geometry validation. This repeatedly allocates quad bounds, sorts the triangle/quad indexes and traverses a newly built hierarchy even though the patch's topology never changes during a brush/projection operation.

### Refittable BVH

New `PreparedQuadCollisionChecker`:
- Takes an independent copy of an indexed quad patch and builds one BVH topology.
- `check_initial()` checks the initial geometry.
- `check_positions(updated_vertices)` validates the same vertex count and quad topology, computes updated normalized bounds, and refits existing BVH leaves/internal nodes **bottom-up**; it never re-sorts the quad array.
- A malformed update is rejected **before** replacing the previous BVH geometry, allowing later valid updates.
- `refit_count()` is exposed for diagnostics. The checker is move-only and throws if called after it has been moved from.
- `SurfaceProjectionStats` now reports `collision_bvh_builds` and `collision_bvh_refits`. When collision protection is enabled, normal projection has **one BVH build** for its initial patch regardless of the number of backtracking steps.

C++ example:

```cpp
guidedquad::PreparedQuadCollisionChecker checker(initial_patch);
auto initial=checker.check_initial(1);
auto candidate=checker.check_positions(next_vertex_positions,1);
```

This API is not thread-safe. A substantial change in geometry may degrade pruning because the refitted tree retains its original partitioning. Rebuild when the mesh connectivity changes, and periodically rebuild when the spatial distribution changes substantially.

## Faster safe adjacency broadphase

Prior adjacent-pair checks often ran four triangle-triangle SAT narrowphase tests even when two quads' overlapping AABBs had **no geometric volume outside their shared edge or shared vertex**.

The validator can now prove this legal shared contact at the AABB level. It skips the SAT narrowphase only when the entire overlap bounding box is contained in the shared feature (within the conservative normalized tolerance). A special constant-time path handles axis-aligned shared edges; arbitrary oriented edges retain the complete bounding-box corner check and full SAT fallback. This cannot skip collisions that extend beyond a seam's shared feature. The report exposes `shared_feature_fastpath_skips`.

## More precise local geometry handling

Two independent numerical shortcomings were addressed:

1. The former 1e-13 absolute SAT separation margin (in normalized model units) could treat distinct near-parallel sheets separated by about 1e-14 as intersecting. The margin has been tightened to 4e-15. This resolves a reproducible *false positive* on representable unit-scale input coordinates.
2. The former 1e-13 coplanar overlap **area** cutoff was not scaled with individual face size and could miss overlaps among legitimate very small quads in an otherwise large model. Coplanar clipping now uses relative local projected-triangle areas; intermediate 2D orientation operations use `long double` where that type provides extra precision. Regression: ~2×10⁻⁷-wide overlapping adjacent quads in a unit-scale model must be detected, while a legal seam with similar local dimensions must remain clean.

The numerical methods are still floating-point approximations, **not exact predicates**. On some platforms `long double` has the same precision as `double`. Very thin, degenerate, badly scaled, or nearly tangent triangles, as well as continuous collisions *between* update frames, remain unsolved. There is no certified absolute precision figure such as “nanometer accuracy” independent of model scale.

## Verification and reproducible timings

`tests/test_refit_collision.cpp` compares refitted vs freshly rebuilt BVH results on changing quad positions, checks rollback after malformed candidates, and executes a 48×48-quad microbenchmark over 20 frames. CTest includes these tests on Windows, Linux, macOS, Debug and Release, plus Linux ASan/UBSan.

On GitHub Actions Ubuntu Release, using the new shared-feature pruning and refit code:

| Algorithm | 2,304 quads × 20 frames |
|---|---:|
| Rebuild a fresh BVH every frame | **199.610 ms** |
| Build once and refit 20 times | **186.558 ms** |

Thus the **refit itself** was about **6.5% faster** than repeated rebuilds in the same run. The earlier pre-optimization reference recorded ~259.374 ms for repeated rebuild and ~248.360 ms for refit, but those came from a different CI run and should not be treated as a controlled head-to-head speedup. No user-perceived real-time sculpting or Exoside comparison has been measured yet. Timings are single-run diagnostics and subject to runner noise. Check current CI status before merging.

## Remaining high-impact work

1. **Dirty-quad local collision queries**: avoid checking unchanged face pairs. Requires an explicit motion/dirty-vertex map and safe handling of moving vs stationary interactions.
2. **Adaptive/exact orientation predicates** (e.g. filtered floating-point + robust exact fallback) for very thin geometry, with care for third-party code licenses.
3. **Continuous collision detection** for large brush strokes / pose changes to avoid tunneling between accepted frames.
4. **Collision topology and seam manifoldness** (including duplicated coincident vertices or ambiguous seams), UV/seam preservation, quality metrics on projected organic assets.
5. The key next remeshing milestone remains automatic patch decomposition and a consistent directional field (4-RoSy), not just lower latency of this local safety guard.
