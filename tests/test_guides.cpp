// SPDX-License-Identifier: MIT
#include "guidedquad/guide_graph.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace guidedquad;

static TriangleMesh square() {
  return {{{0,0,0},{1,0,0},{1,1,0},{0,1,0}}, {{0,1,2},{0,2,3}}};
}

int main() {
  const auto mesh = square();
  {
    const auto g = extract_region_boundaries(mesh, {1,2});
    assert(g.edges.size() == 1 && g.nodes.size() == 2);
    assert(g.paths.size() == 1 && !g.paths[0].closed);
    for (const auto &node : g.nodes) assert(node.a == node.b);
  }
  {
    const auto g = extract_region_boundaries(mesh, {1,1});
    assert(g.edges.empty() && g.paths.empty());
  }
  {
    const auto g = extract_scalar_contours(mesh, {-1,1,1,-1}, 0);
    assert(g.nodes.size() == 3 && g.edges.size() == 2);
    assert(g.paths.size() == 1 && g.paths[0].nodes.size() == 3);
    for (const auto &node : g.nodes) assert(std::fabs(node.position.x - 0.5) < 1e-12);
  }
  {
    const TriangleMesh fan = {{{-1,-1,0},{1,-1,0},{1,1,0},{-1,1,0},{0,0,0}},
                              {{0,1,4},{1,2,4},{2,3,4},{3,0,4}}};
    const auto g = extract_scalar_contours(fan, {-1,-1,-1,-1,1}, 0);
    assert(g.nodes.size() == 4 && g.edges.size() == 4);
    assert(g.paths.size() == 1 && g.paths[0].closed);
  }
  {
    const auto g = extract_scalar_contours(mesh, {0,1,1,-1}, 0);
    assert(g.edges.size() >= 1);
  }
  {
    bool caught = false;
    try { extract_region_boundaries(mesh, {1}); }
    catch (const std::invalid_argument &) { caught = true; }
    assert(caught);
  }
  {
    const TriangleMesh nonmanifold = {{{0,0,0},{1,0,0},{0,1,0},{0,-1,0},{0,0,1}},
                                      {{0,1,2},{1,0,3},{0,1,4}}};
    const auto g = extract_region_boundaries(nonmanifold, {0,1,2});
    assert(!g.warnings.empty());
  }
  {
    auto invalid = mesh;
    invalid.triangles[0][0] = 100;
    bool caught = false;
    try { extract_region_boundaries(invalid, {1,2}); }
    catch (const std::invalid_argument &) { caught = true; }
    assert(caught);
  }
  {
    // Regression: exact-threshold plateau must not produce arbitrary guides.
    const auto g = extract_scalar_contours(mesh, {0,0,0,0}, 0);
    assert(g.nodes.empty() && g.edges.empty() && g.paths.empty());
  }
  {
    const TriangleMesh t = {{{0,0,0},{1,0,0},{0,1,0}}, {{0,1,2}}};
    // Regression: tiny real crossings must remain attached to their edges.
    const auto g = extract_scalar_contours(t, {-1e-12, 1, 1}, 0);
    assert(g.nodes.size() == 2 && g.edges.size() == 1);
    for (const auto &node : g.nodes) {
      assert(node.a != node.b);
      assert(node.t > 0.0 && node.t < 1e-10);
    }
  }
  {
    const TriangleMesh t = {{{0,0,0},{1,0,0},{0,1,0}}, {{0,1,2}}};
    const auto g = extract_scalar_contours(t, {-1e308, 1e308, 1e308}, 0);
    assert(g.nodes.size() == 2 && g.edges.size() == 1);
    for (const auto &node : g.nodes) {
      assert(std::isfinite(node.position.x) && std::isfinite(node.position.y));
      assert(std::fabs(node.t - 0.5) < 1e-12);
    }
  }
  {
    // Re-indexing the input must not change which source-space line is drawn.
    const TriangleMesh t = {{{0,0,0},{1,0,0},{0,1,0}}, {{0,1,2}}};
    const TriangleMesh swapped = {{{1,0,0},{0,0,0},{0,1,0}}, {{0,1,2}}};
    const auto g1 = extract_scalar_contours(t, {0,-1,1}, 0);
    const auto g2 = extract_scalar_contours(swapped, {-1,0,1}, 0);
    assert(g1.edges.size() == 1 && g2.edges.size() == 1);
    std::vector<double> xx1, xx2;
    for (const auto &node : g1.nodes) xx1.push_back(node.position.x);
    for (const auto &node : g2.nodes) xx2.push_back(node.position.x);
    std::sort(xx1.begin(),xx1.end()); std::sort(xx2.begin(),xx2.end());
    assert(xx1.size() == xx2.size());
    for (std::size_t i = 0; i < xx1.size(); ++i) {
      assert(std::fabs(xx1[i] - xx2[i]) < 1e-12);
    }
  }
  {
    // Duplicate graph edges would cause unpredictable path traversal.
    GuideGraph g;
    g.nodes.resize(2);
    g.edges.push_back({0,1});
    g.edges.push_back({1,0});
    bool caught = false;
    try { build_paths(g); } catch(const std::invalid_argument &) { caught = true; }
    assert(caught);
  }

  std::cout << "Guide graph tests passed.\n";
}
