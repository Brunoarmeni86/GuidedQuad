// SPDX-License-Identifier: MIT
#pragma once

#include "guidedquad/patch_solver.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace guidedquad {

// Collision pairs refer to indices into QuadPatchMesh::quads.
// Ordinary edge/vertex contact is permitted. Adjacent quad pairs are
// also tested for positive-area coplanar overlap and noncoplanar crossings
// away from their shared geometric feature. Near-degenerate geometry and
// certain edge-in-plane overlap cases still require exact predicates.
struct QuadIntersectionReport {
  std::vector<std::array<std::uint32_t, 2>> pairs;
  std::uint64_t bvh_pair_tests = 0;
  std::uint64_t candidate_quad_pairs = 0;
  std::uint64_t triangle_pair_tests = 0;
  std::uint64_t adjacent_pairs_ignored = 0;
  std::uint64_t adjacent_pairs_checked = 0;
  bool truncated = false;
};

// Inspect nonadjacent quad pairs using a quad-AABB BVH and triangle-triangle
// separating axis tests (including coplanar overlap). Returns at most
// max_reported_pairs, stopping early to bound runtime for invalid meshes.
// This uses approximate floating-point predicates, not exact arithmetic.
// Small near-contact cases are conservatively classified as intersecting.
// Requires nondegenerate quads and valid indices, throws for malformed input.
QuadIntersectionReport find_quad_self_intersections(
    const QuadPatchMesh &mesh, std::size_t max_reported_pairs = 16);


// A stitched collection is tested as a SINGLE indexed mesh, so collisions
// between separately-generated patches are found as well as within patches.
// Seam endpoints must be explicitly identified by the caller, never welded
// merely because two coordinates happen to be equal (which could conceal
// genuine self-intersections on a folded or overlapping scan).
struct SharedPatchVertex {
  std::uint32_t patch_a, vertex_a, patch_b, vertex_b;
};
struct PatchCollision {
  std::uint32_t patch_a, quad_a, patch_b, quad_b;
};
struct PatchCollectionIntersectionReport {
  std::vector<PatchCollision> collisions;
  QuadIntersectionReport merged_report;
};

// All patches must contain valid all-quad faces. Explicitly stitched seam
// positions must match within a small normalized tolerance, otherwise fail.
// Reports both intra-patch and inter-patch intersections with patch provenance.
PatchCollectionIntersectionReport find_patch_collection_intersections(
    const std::vector<QuadPatchMesh> &patches,
    const std::vector<SharedPatchVertex> &seam_vertices = {},
    std::size_t max_reported_pairs = 16);

} // namespace guidedquad
