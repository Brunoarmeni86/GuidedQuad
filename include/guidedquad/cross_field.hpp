// SPDX-License-Identifier: MIT
#pragma once
#include "guidedquad/guide_graph.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace guidedquad {
// Tangent directions are 4-RoSy: rotating a direction by 90 degrees is equivalent.
// A locked constraint fixes the orientation of one input triangle; it does NOT
// promise that a future quad edge will coincide with a curve or surface feature.
struct FlowConstraint {
  std::uint32_t triangle = 0;
  Vec3 direction;
  double weight = 1.0; // Used only when locked == false.
  bool locked = false;
};

struct CrossFieldOptions {
  std::size_t max_iterations = 200;
  double smoothness = 1.0; // Relative strength of neighbor coupling.
  double tolerance = 1e-9;
};

struct CrossFieldSample {
  Vec3 direction; // Unit tangent, one of four equivalent cross axes.
  double coherence = 0.0; // Local smoothing support, 0..1; NOT quad quality.
};

struct CrossFieldResult {
  std::vector<CrossFieldSample> triangles;
  std::size_t iterations = 0;
  double max_change = 0.0; // Largest |delta| in complex four-angle representation.
  std::vector<std::string> warnings;
};

// CPU reference implementation, deterministic ordering, no external dependency.
// Rejects nonmanifold edges and degenerate triangles rather than hiding them.
CrossFieldResult solve_cross_field(const TriangleMesh &mesh,
                                  const std::vector<FlowConstraint> &constraints,
                                  const CrossFieldOptions &options = {});
} // namespace guidedquad
