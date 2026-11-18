// SPDX-License-Identifier: MIT
#include "guidedquad/guide_graph.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace guidedquad {
namespace {
using Id = std::uint32_t;
using Key = std::uint64_t;

// Pack two 32-bit indices collision-free. Sorting each pair also makes keys
// insensitive to triangle winding and edge traversal direction.
Key edge_key(Id a, Id b) {
  if (a > b) std::swap(a, b);
  return (Key(a) << 32) | Key(b);
}

void validate(const TriangleMesh &mesh) {
  if (mesh.vertices.size() > std::numeric_limits<Id>::max() ||
      mesh.triangles.size() > std::numeric_limits<Id>::max()) {
    throw std::invalid_argument("mesh exceeds supported 32-bit index range");
  }
  for (const Vec3 &p : mesh.vertices) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      throw std::invalid_argument("mesh contains non-finite coordinates");
    }
  }
  for (const auto &tri : mesh.triangles) {
    if (tri[0] >= mesh.vertices.size() || tri[1] >= mesh.vertices.size() ||
        tri[2] >= mesh.vertices.size() || tri[0] == tri[1] ||
        tri[1] == tri[2] || tri[2] == tri[0]) {
      throw std::invalid_argument("mesh contains invalid triangle indices");
    }
  }
}

// Avoid overflow in (threshold - a) / (b - a) for large finite scalars.
// Both endpoints must be on opposite sides of the chosen half-open contour.
double contour_fraction(double a, double b, double threshold) {
  if (a == threshold) return 0.0;
  if (b == threshold) return 1.0;
  const double scale = std::max({std::abs(a), std::abs(b), std::abs(threshold)});
  if (scale == 0.0) throw std::invalid_argument("contour has zero dynamic range");
  const double an = a / scale, bn = b / scale, level = threshold / scale;
  const double den = bn - an;
  if (den == 0.0) {
    throw std::invalid_argument("contour endpoints indistinguishable numerically");
  }
  const double t = (level - an) / den;
  if (!std::isfinite(t)) throw std::invalid_argument("non-finite contour interpolation");
  return std::clamp(t, 0.0, 1.0);
}

struct GraphAssembler {
  GuideGraph result;
  std::unordered_map<Key, Id> node_ids;
  std::unordered_set<Key> graph_edges;

  Id node(Id a, Id b, double t, const TriangleMesh &mesh) {
    if (!std::isfinite(t) || t < 0.0 || t > 1.0) {
      throw std::invalid_argument("invalid interpolation fraction");
    }
    // Snap *exact* endpoints only. Epsilon snapping can erase very small
    // painted features on dense meshes.
    if (t == 0.0) { b = a; }
    else if (t == 1.0) { a = b; t = 0.0; }
    if (a == b) t = 0.0;
    if (a > b) { std::swap(a, b); t = 1.0 - t; }
    const Key key = edge_key(a, b);
    const auto it = node_ids.find(key);
    if (it != node_ids.end()) return it->second;
    if (result.nodes.size() >= std::numeric_limits<Id>::max()) {
      throw std::overflow_error("too many guide nodes");
    }
    const Vec3 &pa = mesh.vertices[a], &pb = mesh.vertices[b];
    const Vec3 p{pa.x * (1.0 - t) + pb.x * t,
                 pa.y * (1.0 - t) + pb.y * t,
                 pa.z * (1.0 - t) + pb.z * t};
    const Id id = static_cast<Id>(result.nodes.size());
    result.nodes.push_back({p, a, b, t});
    node_ids.emplace(key, id);
    return id;
  }

  void connect(Id a, Id b) {
    if (a == b) return;
    const Key key = edge_key(a, b);
    if (graph_edges.insert(key).second) {
      result.edges.push_back({static_cast<Id>(key >> 32), static_cast<Id>(key)});
    }
  }
};

struct EdgeUse {
  Key key;
  Id face;
};
} // namespace

void build_paths(GuideGraph &graph) {
  graph.paths.clear();
  if (graph.nodes.size() > std::numeric_limits<Id>::max() ||
      graph.edges.size() > std::numeric_limits<Id>::max()) {
    throw std::invalid_argument("guide graph exceeds 32-bit index range");
  }

  std::vector<std::vector<Id>> incident(graph.nodes.size());
  std::unordered_set<Key> seen;
  seen.reserve(graph.edges.size());
  for (Id e = 0; e < graph.edges.size(); ++e) {
    const GuideEdge &edge = graph.edges[e];
    if (edge.a >= graph.nodes.size() || edge.b >= graph.nodes.size() ||
        edge.a == edge.b || !seen.insert(edge_key(edge.a, edge.b)).second) {
      throw std::invalid_argument("guide graph has an invalid or duplicate edge");
    }
    incident[edge.a].push_back(e);
    incident[edge.b].push_back(e);
  }
  // Edge indices are visited in ascending order, so incident lists are already
  // sorted: sorting every vertex again is unnecessary on high-resolution meshes.
  std::vector<bool> visited(graph.edges.size(), false);
  const auto walk = [&](Id first_node, Id first_edge) {
    GuidePath path;
    Id current = first_node, e = first_edge;
    path.nodes.push_back(current);
    for (;;) {
      if (visited[e]) break;
      visited[e] = true;
      const GuideEdge &edge = graph.edges[e];
      current = (edge.a == current) ? edge.b : edge.a;
      path.nodes.push_back(current);
      if (current == first_node) {
        path.closed = true;
        break;
      }
      if (incident[current].size() != 2) break;
      const Id next = incident[current][0] == e ? incident[current][1] : incident[current][0];
      if (visited[next]) break;
      e = next;
    }
    graph.paths.push_back(std::move(path));
  };
  for (Id n = 0; n < incident.size(); ++n) {
    if (incident[n].size() == 2) continue;
    for (const Id e : incident[n]) if (!visited[e]) walk(n, e);
  }
  for (Id e = 0; e < graph.edges.size(); ++e) {
    if (!visited[e]) walk(graph.edges[e].a, e);
  }
  std::size_t junctions = 0;
  for (const auto &list : incident) if (list.size() > 2) ++junctions;
  if (junctions) {
    graph.warnings.emplace_back(std::to_string(junctions) +
                                " guide junction(s) require manual or solver resolution");
  }
}

GuideGraph extract_region_boundaries(const TriangleMesh &mesh,
                                     const std::vector<int> &regions,
                                     bool include_mesh_border) {
  validate(mesh);
  if (regions.size() != mesh.triangles.size()) {
    throw std::invalid_argument("one region ID is required per triangle");
  }
  if (mesh.triangles.size() > std::numeric_limits<std::size_t>::max() / 3) {
    throw std::overflow_error("too many triangle edges");
  }

  // Contiguous sorted incidence list avoids thousands of per-edge heap
  // allocations from std::map<Key, vector<FaceId>> and retains stable output.
  std::vector<EdgeUse> uses;
  uses.reserve(mesh.triangles.size() * 3);
  for (Id f = 0; f < mesh.triangles.size(); ++f) {
    const auto &tri = mesh.triangles[f];
    for (int i = 0; i < 3; ++i) {
      uses.push_back({edge_key(tri[i], tri[(i + 1) % 3]), f});
    }
  }
  std::sort(uses.begin(), uses.end(), [](const EdgeUse &a, const EdgeUse &b) {
    return a.key < b.key;
  });

  GraphAssembler out;
  std::size_t nonmanifold = 0;
  for (std::size_t i = 0; i < uses.size();) {
    std::size_t j = i + 1;
    while (j < uses.size() && uses[j].key == uses[i].key) ++j;
    const std::size_t count = j - i;
    if (count > 2) {
      ++nonmanifold;
    } else {
      const bool boundary = (count == 2 && regions[uses[i].face] != regions[uses[i + 1].face]) ||
                            (include_mesh_border && count == 1);
      if (boundary) {
        const Id a = static_cast<Id>(uses[i].key >> 32);
        const Id b = static_cast<Id>(uses[i].key);
        const Id start = out.node(a, a, 0.0, mesh);
        const Id end = out.node(b, b, 0.0, mesh);
        out.connect(start, end);
      }
    }
    i = j;
  }
  if (nonmanifold) {
    out.result.warnings.emplace_back(std::to_string(nonmanifold) +
                                     " non-manifold source edge(s) skipped");
  }
  build_paths(out.result);
  return out.result;
}

GuideGraph extract_scalar_contours(const TriangleMesh &mesh,
                                   const std::vector<double> &values,
                                   double threshold) {
  validate(mesh);
  if (values.size() != mesh.vertices.size() || !std::isfinite(threshold)) {
    throw std::invalid_argument("invalid scalar array or contour threshold");
  }
  for (const double value : values) {
    if (!std::isfinite(value)) throw std::invalid_argument("non-finite scalar value");
  }
  // A fixed >= convention is independent of vertex numbering. Exact-threshold
  // plateaus are treated as inside; all-equal triangles emit no arbitrary lines.
  std::vector<bool> high(values.size());
  for (Id v = 0; v < values.size(); ++v) high[v] = values[v] >= threshold;

  GraphAssembler out;
  for (const auto &tri : mesh.triangles) {
    Id crossings[3];
    int count = 0;
    for (int i = 0; i < 3; ++i) {
      const Id a = tri[i], b = tri[(i + 1) % 3];
      if (high[a] == high[b]) continue;
      const double t = contour_fraction(values[a], values[b], threshold);
      crossings[count++] = out.node(a, b, t, mesh);
    }
    if (count == 2) out.connect(crossings[0], crossings[1]);
  }
  build_paths(out.result);
  return out.result;
}

} // namespace guidedquad
