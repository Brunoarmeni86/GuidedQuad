// SPDX-License-Identifier: MIT
#pragma once

#include "guidedquad/patch_solver.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
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
  std::uint64_t shared_feature_fastpath_skips = 0;
  std::uint64_t dirty_face_queries = 0; // individually queried moving quads
  bool truncated = false;
};

// Persistent BVH for repeated validation of the SAME quad topology.
// The build is performed once. Subsequent coordinate updates refit AABBs in
// linear time without re-sorting or rebuilding the tree. Not thread-safe.
// A failed update throws and leaves the last valid geometry unchanged.
class PreparedQuadCollisionChecker {
 public:
  explicit PreparedQuadCollisionChecker(const QuadPatchMesh &topology_and_positions);
  ~PreparedQuadCollisionChecker();
  PreparedQuadCollisionChecker(PreparedQuadCollisionChecker &&) noexcept;
  PreparedQuadCollisionChecker &operator=(PreparedQuadCollisionChecker &&) noexcept;
  PreparedQuadCollisionChecker(const PreparedQuadCollisionChecker &) = delete;
  PreparedQuadCollisionChecker &operator=(const PreparedQuadCollisionChecker &) = delete;

  QuadIntersectionReport check_initial(std::size_t max_reported_pairs=16);
  QuadIntersectionReport check_positions(
      const std::vector<Vec3> &positions,
      std::size_t max_reported_pairs=16);

  // Only query pairs involving a face incident to an actually changed
  // vertex. Requires a previously checked collision-free state. Every
  // changed vertex MUST appear in dirty_vertices, else throws. A positive
  // collision is rejected and the last clean checker state restored.
  // Full vertex validation + BVH refit are still O(V + F); only collision
  // queries are dirty-face-local. No collision can be missed solely because
  // two unchanged quads were omitted from a clean baseline.
  QuadIntersectionReport check_positions_dirty(
      const std::vector<Vec3> &positions,
      const std::vector<std::uint32_t> &dirty_vertices,
      std::size_t max_reported_pairs=16);
  std::uint64_t refit_count() const;
  std::uint64_t dirty_update_count() const;
  // Successful partial box/node updates; full fallback when geometry no
  // longer fits the checker's original normalized coordinate frame.
  std::uint64_t local_refit_count() const;
  std::uint64_t full_refit_fallback_count() const;
  std::uint64_t local_bvh_node_updates() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
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
