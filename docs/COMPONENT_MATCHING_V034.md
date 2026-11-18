# GuidedQuad V0.3.4 — source component-aware projection

**Experimental.** The quad patch solver now offers an explicit source connected-component constraint in addition to the existing nearest-distance and face-normal checks. The connected-component ID is chosen by the caller; the tool does **not automatically decide** which surface layer is anatomically correct.

## Rationale

The normal filter alone cannot distinguish two close but disconnected scan shells that point in the same direction. An interior quad vertex might snap onto the wrong layer, even when both normals align. A strict source component selection prevents this particular cross-shell mistake.

## How it works

- On creation of `PreparedSurfaceProjector`, construct triangle components from **shared mesh EDGES** using disjoint-set union. Unwelded meshes with coincident coordinates are not considered connected; touching at one vertex without sharing an edge does not merge components.
- Assign stable component IDs from zero in the valid source triangle order (IDs depend on the source geometry and face ordering). Degenerate triangles are skipped as before.
- The BVH caches a homogeneous-component ID for its nodes and prunes nodes that cannot belong to the selected component. Mixed nodes filter individual face candidates.
- `PreparedSurfaceProjector::connected_component_count()` returns how many valid edge-connected components exist.
- `SurfaceProjectionOptions::source_component_id` defaults to UINT32_MAX (disabled). Set it to a valid ID to require the chosen connected shell. Invalid IDs throw `std::invalid_argument` instead of silently falling back to unrelated triangles.
- `SurfaceProjectionStats::component_rejected_triangles` counts individual triangle candidates filtered out inside mixed BVH leaves. Pruned homogeneous BVH nodes do not contribute to that count.

## C++ usage

```cpp
guidedquad::PreparedSurfaceProjector prepared(triangle_source);
const auto count = prepared.connected_component_count();
guidedquad::SurfaceProjectionOptions opts;
opts.source_component_id = 1; // select shell 1 if 0 <= 1 < count
opts.min_reference_normal_dot = 0.5; // optional simultaneous normal filter
opts.max_projection_distance = 0.2; // optional original-space search radius
auto result = prepared.project(quad_patch, u_segments, v_segments, opts);
```

The CLI `project-loop` accepts `[component_id]` after `[normal_dot] [max_distance]`:

```sh
./build/guidedquad_cli project-loop border.obj reference.obj projected.obj \
  0 1 2 3 8 8 2 1 0.5 2 0
```

This example selects source component zero. A Blender standard addon is **not yet implemented**.

## Regressions and limits

- Two disconnected **same-oriented** parallel sheets: default nearest point picks the nearer shell, while explicit component selection can pick the farther intended shell.
- Two triangles touching only at one vertex remain separate components.
- Out-of-range component selection must fail explicitly.
- Component API reports an error if accessed after moving away the owning `PreparedSurfaceProjector`.
- All standard projection tests, collision detection, Linux/Windows/macOS CI and sanitizer checks remain applicable.

This does not solve *which component to pick automatically*. A single connected shell can still contain nearby self-folded regions. Fixing that requires feature/guide anchoring, region-based correspondences and source-surface geodesic continuity. The initial union-find creates an edge hash map with O(T) expected memory/work during BVH construction, so benchmarking large, unwelded scans is important. The prepared BVH and component labels remain immutable until the reference surface is rebuilt.

## Future

Enable per-patch source region selection in the Blender extension; component editing and source visualization; robust topological/fold-aware nearest queries; BVH refit/partition caching and benchmark on high-resolution organic scans. Do not claim this experimental patch engine is a replacement for Exoside.
