// SPDX-License-Identifier: MIT
#pragma once

#include "guidedquad/guide_graph.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace guidedquad {

// A quadrilateral chart has four explicitly guided boundary curves.
// Each sequence is ordered in positive patch coordinates:
// bottom = left -> right; right = bottom -> top;
// top = left -> right; left = bottom -> top.
struct QuadPatchBoundary {
  std::vector<Vec3> bottom, right, top, left;
};

struct QuadPatchOptions {
  std::uint32_t u_segments = 8;
  std::uint32_t v_segments = 8;
  // Optional interior-only Laplacian smoothing. Boundaries remain immutable.
  std::uint32_t smooth_iterations = 0;
  double smooth_strength = 0.0; // [0,1]
};

struct QuadPatchMesh {
  std::vector<Vec3> vertices;
  std::vector<std::array<std::uint32_t, 4>> quads;
};

struct QuadPatchQuality {
  double max_boundary_error = 0.0;     // world-space, sampled boundaries
  double worst_edge_aspect_ratio = 1.0; // longest / shortest quad side
  double mean_abs_orthogonality = 0.0;  // mean |cos(corner angle)|, 0 best
  double minimum_face_area = 0.0;      // approximate area (two triangles)
};

struct QuadPatchResult {
  QuadPatchMesh mesh;
  QuadPatchQuality quality;
};

// Construct one connected, all-quad chart on a four-sided patch.
// Hard boundaries are satisfied up to floating-point precision.
// No automatic projection to a curved triangle source is performed in V0.2.
// Throws on incompatible corners, self-folded/degenerate results or huge grids.
QuadPatchResult solve_quad_patch(const QuadPatchBoundary &boundary,
                                const QuadPatchOptions &options);

// Split a closed guide path at four ordered path indices to form the four
// hard boundaries. The guide path must contain a final repeated start node.
// The indices must be strictly increasing and less than path.nodes.size()-1.
QuadPatchBoundary patch_boundary_from_loop(const GuideGraph &graph,
                                           std::size_t path_index,
                                           std::array<std::size_t, 4> corners);

} // namespace guidedquad
