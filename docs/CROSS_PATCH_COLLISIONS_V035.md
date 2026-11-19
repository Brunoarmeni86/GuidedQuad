# GuidedQuad V0.3.5 — adjacent and cross-patch collision audit

**Experimental correctness upgrade**, built on the connected-component and BVH projection branches. The stable `main` and Blender Sculpt Extended remain unchanged.

## Bugs addressed

1. The previous detector skipped any pair of quads that shared even one vertex index. This could conceal a real overlap: two quad faces sharing an edge but occupying the **same side** of it, or two noncoplanar faces passing through each other well beyond their shared corner.
2. Each call previously considered just one `QuadPatchMesh`, so two individually valid but overlapping patches would not be checked against each other.
3. Independent patches may have two different vertex indices at an intentional shared seam. Automatically welding coincident coordinates would be unsafe on sculpted folds, but never recognizing a seam would also produce false-positive boundary-contact reports.

## New adjacent-pair collision checks

`find_quad_self_intersections` now examines AABB-overlapping shared-vertex or shared-edge quad pairs rather than blindly skipping them.

- Positive-area **coplanar** triangle overlap is detected using projected 2D convex-polygon clipping.
- **Noncoplanar** triangle crossings are detected when a triangle edge pierces the other triangle outside the pair's legitimate common vertex or edge.
- A legal edge-sharing coplanar grid, a single-corner contact, or a right-angle hinge should not be reported.
- Results still use the existing quad BVH broadphase and pair report limit.
- New `adjacent_pairs_checked` tracks such investigations. `adjacent_pairs_ignored` now means a checked adjacent pair was found to have no illegal overlap, rather than skipping it automatically.

This is **not exact robust computational geometry**: extreme slivers, near-tangencies, coincident surfaces and coplanar edge configurations may remain ambiguous. For a strict watertight guarantee, robust adaptive predicates and a complete intersection witness construction remain necessary.

## Collision checking across patches

New function:

```cpp
std::vector<guidedquad::QuadPatchMesh> patches={patch_left,patch_right};
std::vector<guidedquad::SharedPatchVertex> seams={
  {0, left_vertex_index, 1, right_vertex_index}
};
auto report=guidedquad::find_patch_collection_intersections(patches,seams,16);
for (const auto &hit:report.collisions) {
  // hit.patch_a, hit.quad_a, hit.patch_b, hit.quad_b
}
```

- Builds a temporary combined indexed mesh and runs the same BVH collision detector.
- The caller provides **explicit seam correspondences** to identify legitimate shared vertices. Without seam correspondences, even legitimate touching patch borders can be reported as contact. Do not auto-weld by nearest coordinate without checking manifoldness and intended connectivity.
- Validates patch and vertex indices, requires seam endpoints to agree within a normalized tolerance, and keeps provenance of each detected quad pair.
- Reports both **within-patch** and **between-patch** collisions. The report preserves the existing global detection cap (default 16).
- Does **not** repair collisions, merge UVs or assign edge loops across patches. It provides validation before separately generated patches are accepted into a global mesh.

## Verification

Regression tests cover:
- Intersection of two independent quads with a fully crossing third-dimensional sheet.
- Overlapping and separate patches.
- Two adjacent patches meeting along an explicitly stitched edge (no false collision).
- Two stitched patches overlapping on the same side of the seam (detected).
- Contact at one shared corner (no false collision).
- A noncoplanar intersection extending beyond one shared corner (detected).
- Valid right-angle hinge with two shared vertices (no false collision).
- Invalid or geometrically mismatched seam mapping rejected.
- Full existing guide/patch/UV-independent projection and source-component regression suite.

Run `cmake -S . -B build`, `cmake --build build` and `ctest --test-dir build --output-on-failure`. GitHub Actions also runs Windows, Linux and macOS Debug/Release plus Linux ASan/UBSan. Only mark platforms verified once CI completes.

## Performance and roadmap

This version constructs one collision BVH for the temporary merged patch collection (instead of one BVH for every possible pair of patches). Expected broadphase is substantially more efficient than brute-force all-pairs for well-separated patches, but heavy overlaps can still produce quadratic checks. **No measured overall wall-clock speedup is claimed.** Stitching builds an additional combined mesh and uses O(total vertices + quads) memory.

Next correctness/performance priorities:
1. Persistent output collision BVH with incremental refitting and dirty-quad queries, instead of rebuilding it for every projection backtracking step.
2. Exact/adaptive intersection predicates, adjacent shared-edge tangency tests, better handling of zero-area slivers and self-intersecting single quads.
3. Automatic seam correspondence and multi-patch stitch topology validation.
4. Feature/region-guided source correspondence within folded connected surfaces; 4-RoSy global flow/patch decomposition, UV/attribute/detail transfer, real organic sculpt benchmark dataset.
