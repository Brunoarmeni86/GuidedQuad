# GuidedQuad V0.3.2 — reusable BVH and oriented correspondence

**Status: experimental**, stacked after the V0.3.1 distance-guard PR. This is not an automatic global quad remesher, not integrated into a Blender addon, and not merged into `main`.

## Persistent reference-surface index

Previously `project_quad_patch_to_surface(surface, patch, U, V)` constructed and discarded a CPU BVH on each call. The original compatibility function remains available. The new `PreparedSurfaceProjector` owns an immutable copy of the triangulated source and builds one BVH, which can be queried for any number of four-sided patches.

```cpp
guidedquad::PreparedSurfaceProjector prepared(reference_tri_mesh);
auto first = prepared.project(first_quad_patch, u1, v1);
auto second = prepared.project(second_quad_patch, u2, v2);
```

The prepared index retains the source geometry after the original `TriangleMesh` has gone out of scope, and can be moved (not copied). Each projection result has independent query counts and diagnostic metrics. The existing `project_quad_patch_to_surface` helper constructs a temporary prepared index to maintain its old semantics.

**Optimization model:** the first prepared projection pays the index construction cost; subsequent patch projections reuse it. We have tested output equivalence with the legacy helper on a synthetic height field and independent results on patches with different U/V subdivisions. We have **not measured wall-clock or peak RAM gains** on many-patch organic assets. The cached surface copy consumes memory and must be refreshed when source geometry changes.

## Optional source normal filtering

The new `SurfaceProjectionOptions::min_reference_normal_dot` is `-1.0` by default (disabled, preserving the previous nearest-point selection). Values greater than -1, up to 1, enforce an alignment threshold between a source triangle's normalized face normal and the *original* quad patch's tangent-frame normal for each interior vertex.

```cpp
guidedquad::SurfaceProjectionOptions options;
options.min_reference_normal_dot = 0.5; // require compatible source winding
options.max_projection_distance = 0.1; // optional absolute source-space radius
auto projected = prepared.project(patch, u, v, options);
```

When scanning nearby opposing layers, the nearest Euclidean triangle may belong to the wrong side of a surface. This filter searches for the closest **eligible** oriented triangle rather than blindly accepting the closest one. If none is eligible, it fails explicitly, rather than silently snapping the quad to a known incompatible layer. The filter is opt-in because incorrect or inconsistent source face winding can make orientation checks reject otherwise valid geometry. An aligned triangle can still be topologically unrelated; this is not a complete solution to thin-feature correspondence.

### Standalone CLI

The `project-loop` command now accepts two additional optional arguments after `[iterations] [strength]`: `[normal_dot] [max_distance]`.

```sh
./build/guidedquad_cli project-loop \
  tests/square.obj tests/curved_surface.obj projected_oriented.obj \
  0 1 2 3 8 8 2 1 0.5
```

This uses the winding-aware filter with `normal_dot=0.5`. It still constructs a one-shot BVH for the single CLI operation; **BVH persistence** is available through the C++ `PreparedSurfaceProjector` API when processing multiple patches. A standard Blender integration is still future work.

## Verified synthetic tests

- A prepared reference index can be used after its original `TriangleMesh` goes out of scope.
- Two quad patches with different resolutions can share one reference index.
- A prepared projection matches legacy temporary-index output at identical options.
- A pair of closely spaced, oppositely wound source sheets: default distance-only matching picks the nearer sheet, while the optional orientation filter selects the farther but correctly oriented sheet.
- If the only source sheet has incompatible winding and orientation matching is enabled, the solver fails explicitly instead of returning misleading geometry.

GitHub CI exercises Linux, Windows and macOS Debug/Release plus Linux AddressSanitizer/UndefinedBehaviorSanitizer. Report success only after the relevant run completes.

## Further work

1. Global self-intersections and collisions between disconnected quad patches, using a separate triangle/quad BVH.
2. Feature- or region-aware correspondence: topology, distance and normals simultaneously, not just individual candidate triangle orientation.
3. BVH refit instead of full rebuild for changing source vertex positions; cache ownership and invalidation policy.
4. Benchmark wall-clock/peak RAM on multiple organic patches, comparing reused vs rebuilt BVHs.
5. Automatic 4-RoSy flow, patch decomposition, UV and attribute/detail transfer and standalone Blender addon.
