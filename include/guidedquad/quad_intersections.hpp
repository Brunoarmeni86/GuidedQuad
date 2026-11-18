// SPDX-License-Identifier: MIT
#pragma once

#include "guidedquad/patch_solver.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace guidedquad {

// Collision pairs refer to indices into QuadPatchMesh::quads.
// Pairs sharing ANY vertex index are deliberately excluded: ordinary grid
// neighbors touch at shared vertices/edges. This early detector does NOT
// certify that adjacent quads cannot fold through each other.
struct QuadIntersectionReport {
  std::vector<std::array<std::uint32_t, 2>> pairs;
  std::uint64_t bvh_pair_tests = 0;
  std::uint64_t candidate_quad_pairs = 0;
  std::uint64_t triangle_pair_tests = 0;
  std::uint64_t adjacent_pairs_ignored = 0;
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

} // namespace guidedquad
