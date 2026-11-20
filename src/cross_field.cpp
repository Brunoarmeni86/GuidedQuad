// SPDX-License-Identifier: MIT
#include "guidedquad/cross_field.hpp"
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace guidedquad {
namespace {
using Id = std::uint32_t;
using Complex = std::complex<double>;
using EdgeKey = std::pair<Id, Id>;
constexpr double kTiny = 1e-14;

Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator*(Vec3 a, double t) { return {a.x * t, a.y * t, a.z * t}; }
double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y,
          a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}
double length(Vec3 a) { return std::sqrt(dot(a, a)); }
Vec3 unit(Vec3 a) { return a * (1.0 / length(a)); }
bool finite(Vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
Complex phase(double angle) { return {std::cos(4.0 * angle), std::sin(4.0 * angle)}; }
Complex normalized(Complex z) { return z / std::abs(z); }
struct Frame { Vec3 u, v, n; };
struct Neighbor { Id face; Complex transport; double weight; };

// The first edge defines a local chart. The shared edge is used to parallel-
// transport the 4-RoSy phases from one face chart to the next.
Frame frame_for_triangle(const TriangleMesh &mesh, const std::array<Id, 3> &t) {
  const Vec3 a = mesh.vertices[t[0]];
  const Vec3 e1 = mesh.vertices[t[1]] - a;
  const Vec3 e2 = mesh.vertices[t[2]] - a;
  const double e1_len = length(e1);
  const Vec3 n = cross(e1, e2);
  const double n_len = length(n);
  const double scale = std::max({length(e1), length(e2),
                                 length(mesh.vertices[t[2]] - mesh.vertices[t[1]])});
  if (!(scale > 0.0) || !(e1_len > 0.0) || !(n_len > 1e-12 * scale * scale)) {
    throw std::invalid_argument("cross field: degenerate or nearly collinear triangle");
  }
  Frame f;
  f.u = unit(e1);
  f.n = n * (1.0 / n_len);
  f.v = cross(f.n, f.u);
  return f;
}

double angle_in_frame(Vec3 direction, const Frame &frame) {
  return std::atan2(dot(direction, frame.v), dot(direction, frame.u));
}
} // namespace

CrossFieldResult solve_cross_field(const TriangleMesh &mesh,
                                  const std::vector<FlowConstraint> &constraints,
                                  const CrossFieldOptions &options) {
  if (!std::isfinite(options.smoothness) || options.smoothness < 0.0 ||
      !std::isfinite(options.tolerance) || options.tolerance <= 0.0) {
    throw std::invalid_argument("cross field: invalid solver options");
  }
  const std::size_t nf = mesh.triangles.size();
  if (nf > std::numeric_limits<Id>::max() ||
      mesh.vertices.size() > std::numeric_limits<Id>::max()) {
    throw std::invalid_argument("cross field: 32-bit index limit exceeded");
  }
  CrossFieldResult result;
  if (nf == 0) return result;
  for (const Vec3 &p : mesh.vertices) {
    if (!finite(p)) throw std::invalid_argument("cross field: non-finite mesh vertex");
  }
  std::vector<Frame> frames;
  frames.reserve(nf);
  std::map<EdgeKey, std::vector<std::pair<Id, bool>>> edge_faces;
  for (Id f = 0; f < nf; ++f) {
    const auto &t = mesh.triangles[f];
    if (t[0] >= mesh.vertices.size() || t[1] >= mesh.vertices.size() ||
        t[2] >= mesh.vertices.size() || t[0] == t[1] ||
        t[1] == t[2] || t[2] == t[0]) {
      throw std::invalid_argument("cross field: invalid triangle indices");
    }
    frames.push_back(frame_for_triangle(mesh, t));
    for (int e = 0; e < 3; ++e) {
      const Id a = t[e], b = t[(e + 1) % 3];
      edge_faces[{std::min(a, b), std::max(a, b)}].push_back({f, a < b});
    }
  }
  std::vector<std::vector<Neighbor>> neighbors(nf);
  for (const auto &entry : edge_faces) {
    const auto &adjacent = entry.second;
    if (adjacent.size() > 2)
      throw std::invalid_argument("cross field: nonmanifold edge (>2 triangles)");
    if (adjacent.size() != 2) continue;
    if (adjacent[0].second == adjacent[1].second)
      throw std::invalid_argument("cross field: inconsistent orientation of neighboring triangles");
    const Id a = adjacent[0].first, b = adjacent[1].first;
    const Vec3 edge = unit(mesh.vertices[entry.first.second] -
                           mesh.vertices[entry.first.first]);
    const double beta_a = angle_in_frame(edge, frames[a]);
    const double beta_b = angle_in_frame(edge, frames[b]);
    const double edge_weight = length(mesh.vertices[entry.first.second] -
                                      mesh.vertices[entry.first.first]);
    const Complex b_to_a = phase(beta_a - beta_b);
    neighbors[a].push_back({b, b_to_a, edge_weight});
    neighbors[b].push_back({a, std::conj(b_to_a), edge_weight});
  }

  std::vector<Complex> anchored(nf, {0.0, 0.0});
  std::vector<double> anchor_weight(nf, 0.0);
  std::vector<bool> locked(nf, false);
  for (const FlowConstraint &c : constraints) {
    if (c.triangle >= nf || !finite(c.direction) ||
        !std::isfinite(c.weight) || c.weight < 0.0) {
      throw std::invalid_argument("cross field: invalid directional constraint");
    }
    const Frame &f = frames[c.triangle];
    const Vec3 tangent = c.direction - f.n * dot(c.direction, f.n);
    if (!(length(tangent) > kTiny * std::max(1.0, length(c.direction)))) {
      throw std::invalid_argument("cross field: constraint is perpendicular to its face");
    }
    const Complex q = phase(angle_in_frame(tangent, f));
    if (c.locked) {
      if (locked[c.triangle] && std::abs(anchored[c.triangle] - q) > 1e-6)
        throw std::invalid_argument("cross field: conflicting locked constraints");
      anchored[c.triangle] = q;
      anchor_weight[c.triangle] = 1.0;
      locked[c.triangle] = true;
    } else if (!locked[c.triangle] && c.weight > 0.0) {
      anchored[c.triangle] += q * c.weight;
      anchor_weight[c.triangle] += c.weight;
    }
  }

  // Initialize seeded components by deterministic breadth-first transport.
  // Never invent a meaningful direction for an unseeded connected component.
  std::vector<Complex> q(nf, {1.0, 0.0});
  std::vector<bool> reached(nf, false);
  std::deque<Id> queue;
  for (Id i = 0; i < nf; ++i) {
    if (std::abs(anchored[i]) > kTiny) {
      q[i] = normalized(anchored[i]);
      reached[i] = true;
      queue.push_back(i);
    }
  }
  while (!queue.empty()) {
    const Id from = queue.front();
    queue.pop_front();
    for (const Neighbor &link : neighbors[from]) {
      if (reached[link.face]) continue;
      reached[link.face] = true;
      // Link transports neighbor -> from; conjugate transports from -> neighbor.
      q[link.face] = normalized(q[from] * std::conj(link.transport));
      queue.push_back(link.face);
    }
  }
  if (std::find(reached.begin(), reached.end(), false) != reached.end()) {
    result.warnings.push_back("unconstrained component: reference axis is arbitrary");
  }
  if (constraints.empty()) result.warnings.push_back("no directional guides provided");

  std::vector<Complex> next = q;
  std::vector<double> coherence(nf, 1.0);
  for (std::size_t it = 0; it < options.max_iterations; ++it) {
    double max_delta = 0.0;
    for (Id i = 0; i < nf; ++i) {
      if (locked[i]) {
        next[i] = anchored[i];
        continue;
      }
      Complex sum = anchored[i];
      double total_weight = anchor_weight[i];
      double neighbor_weight = 0.0;
      for (const Neighbor &n : neighbors[i]) {
        const double w = n.weight * options.smoothness;
        sum += q[n.face] * n.transport * w;
        total_weight += w;
        neighbor_weight += w;
      }
      // Positive diagonal relaxation prevents two-face Jacobi ping-pong in
      // unconstrained components while leaving locked constraints exact.
      const double diagonal = neighbor_weight * 0.5;
      sum += q[i] * diagonal;
      total_weight += diagonal;
      if (std::abs(sum) <= kTiny * std::max(total_weight, 1.0)) {
        next[i] = q[i];
        coherence[i] = 0.0;
      } else {
        next[i] = normalized(sum);
        coherence[i] = total_weight > 0.0 ?
            std::min(1.0, std::abs(sum) / total_weight) : 0.0;
      }
      max_delta = std::max(max_delta, std::abs(next[i] - q[i]));
    }
    q.swap(next);
    result.iterations = it + 1;
    result.max_change = max_delta;
    if (max_delta < options.tolerance) break;
  }

  result.triangles.reserve(nf);
  for (Id i = 0; i < nf; ++i) {
    const double angle = std::atan2(q[i].imag(), q[i].real()) / 4.0;
    const Vec3 d = frames[i].u * std::cos(angle) + frames[i].v * std::sin(angle);
    result.triangles.push_back({d, locked[i] ? 1.0 : coherence[i]});
  }
  return result;
}
} // namespace guidedquad
