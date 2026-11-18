// SPDX-License-Identifier: MIT
#pragma once

#include "guidedquad/patch_solver.hpp"
#include <cstdint>

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
};

struct SurfaceProjectionStats {
  std::uint64_t closest_queries = 0;
  std::uint64_t box_tests = 0;
  std::uint64_t triangle_tests = 0;
  std::uint32_t skipped_degenerate_triangles = 0;
  std::uint32_t accepted_iterations = 0;
  std::uint32_t rejected_steps = 0;
  std::uint32_t moved_vertices = 0;
  double initial_rms_distance = 0.0; // vertices interior to the source surface
  double final_rms_distance = 0.0;   // exact BVH nearest-surface distance
  double max_vertex_displacement = 0.0;
};

struct SurfaceProjectionResult {
  QuadPatchMesh mesh;
  SurfaceProjectionStats stats;
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
