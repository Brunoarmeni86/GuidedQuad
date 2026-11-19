// SPDX-License-Identifier: MIT
#pragma once

#include "guidedquad/patch_solver.hpp"
#include <cstdint>
#include <limits>
#include <memory>

namespace guidedquad {

// Optional projection of an existing rectangular all-quad patch onto a
// reference triangle surface. Boundary vertices are ALWAYS immutable.
// This is an experimental local chart operation, not global remeshing.
struct SurfaceProjectionOptions {
  std::uint32_t iterations = 2;
  double strength = 1.0;          // (0, 1]
  std::uint32_t backtracking_steps = 10; // halve strength when rejected
  double min_area_ratio = 0.1;     // candidate triangle area / initial area
  double min_corner_sine = 0.05;   // signed scaled Jacobian wrt baseline normal
  double min_normal_dot = 0.1;     // candidate triangle normal vs initial
  // Optional strict correspondence radius from ORIGINAL patch positions.
  // A target beyond this distance is skipped, not partially projected.
  // Infinity (default) disables the limit; must be >0 if finite.
  double max_projection_distance = std::numeric_limits<double>::infinity();
  // Optional source winding filter. -1 disables. Values in (-1,1] require
  // consistent reference face winding aligned to the original quad patch.
  // If no eligible triangle exists, projection fails rather than snapping to
  // a known incompatible reference layer.
  double min_reference_normal_dot = -1.0;
  // Validate nonadjacent quad collisions at input and on every candidate.
  // Default safe; disabling this avoids BVH rebuilds but removes a guarantee.
  bool reject_self_intersections = true;
  // Optional connected source component (triangle edge connectivity).
  // UINT32_MAX = no component filter (default). IDs assigned in source
  // face order to nondegenerate connected components.
  std::uint32_t source_component_id = std::numeric_limits<std::uint32_t>::max();
};

struct SurfaceProjectionStats {
  std::uint64_t closest_queries = 0;
  std::uint64_t box_tests = 0;
  std::uint64_t triangle_tests = 0;
  std::uint64_t normal_rejected_triangles = 0;
  std::uint64_t component_rejected_triangles = 0;
  std::uint32_t skipped_degenerate_triangles = 0;
  std::uint32_t accepted_iterations = 0;
  std::uint32_t rejected_steps = 0;
  std::uint32_t collision_rejected_steps = 0;
  std::uint64_t collision_validation_passes = 0;
  std::uint64_t collision_bvh_builds = 0;
  std::uint64_t collision_bvh_refits = 0;
  std::uint64_t collision_dirty_passes = 0;
  std::uint64_t collision_dirty_face_queries = 0;
  std::uint64_t distance_rejected_targets = 0; // target queries outside original radius
  std::uint32_t moved_vertices = 0;
  double initial_rms_distance = 0.0; // vertices interior to the source surface
  double final_rms_distance = 0.0;   // exact BVH nearest-surface distance
  double max_vertex_displacement = 0.0;
};

struct SurfaceProjectionResult {
  QuadPatchMesh mesh;
  SurfaceProjectionStats stats;
};

// Build the reference triangle BVH once and reuse it across many patches.
// The reference surface is copied and remains immutable for the cache lifetime.
// Query counters belong to each projection result, not to the shared cache.
class PreparedSurfaceProjector {
 public:
  explicit PreparedSurfaceProjector(const TriangleMesh &reference_surface);
  ~PreparedSurfaceProjector();
  PreparedSurfaceProjector(PreparedSurfaceProjector &&) noexcept;
  PreparedSurfaceProjector &operator=(PreparedSurfaceProjector &&) noexcept;
  PreparedSurfaceProjector(const PreparedSurfaceProjector &) = delete;
  PreparedSurfaceProjector &operator=(const PreparedSurfaceProjector &) = delete;

  SurfaceProjectionResult project(const QuadPatchMesh &patch,
      std::uint32_t u_segments, std::uint32_t v_segments,
      const SurfaceProjectionOptions &options = {}) const;
  std::uint32_t active_triangle_count() const;
  std::uint32_t connected_component_count() const;
  std::uint32_t skipped_degenerate_triangles() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// The input MUST be a regular UxV quad grid returned by solve_quad_patch.
// Projection uses a reusable CPU BVH and closest-triangle queries.
// If no non-inverting step is possible, the input patch is returned unchanged.
// Rejects malformed topology and a reference mesh without valid triangles.
// Does NOT guarantee global non-self-intersection or UV/attribute transfer.
SurfaceProjectionResult project_quad_patch_to_surface(
    const TriangleMesh &reference_surface,
    const QuadPatchMesh &patch,
    std::uint32_t u_segments,
    std::uint32_t v_segments,
    const SurfaceProjectionOptions &options = {});

} // namespace guidedquad
