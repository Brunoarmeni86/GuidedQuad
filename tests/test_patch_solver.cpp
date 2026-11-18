// SPDX-License-Identifier: MIT
#include "guidedquad/patch_solver.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace guidedquad;

static QuadPatchBoundary unit_square() {
  return {{{0,0,0},{1,0,0}},  // bottom
          {{1,0,0},{1,1,0}},  // right
          {{0,1,0},{1,1,0}},  // top
          {{0,0,0},{0,1,0}}}; // left
}

static QuadPatchBoundary curved_square() {
  return {{{0,0,0},{0.5,-0.12,0},{1,0,0}},
          {{1,0,0},{1.08,0.5,0},{1,1,0}},
          {{0,1,0},{0.5,1.14,0},{1,1,0}},
          {{0,0,0},{-0.1,0.5,0},{0,1,0}}};
}

int main() {
  {
    QuadPatchOptions options;
    options.u_segments=4;
    options.v_segments=4;
    const auto patch=solve_quad_patch(unit_square(),options);
    assert(patch.mesh.vertices.size()==25);
    assert(patch.mesh.quads.size()==16);
    assert(patch.quality.max_boundary_error < 1e-13);
    assert(patch.quality.worst_edge_aspect_ratio < 1.0+1e-12);
    assert(patch.quality.mean_abs_orthogonality < 1e-12);
    assert(std::fabs(patch.quality.minimum_face_area - 1.0/16) < 1e-12);
    for (const auto &q : patch.mesh.quads) {
      for (auto id : q) assert(id < patch.mesh.vertices.size());
    }
  }
  {
    // The Coons solver follows all four curved guides, whereas the bilinear
    // baseline would keep the top middle at y=1 and bottom middle at y=0.
    auto options=QuadPatchOptions{};
    options.u_segments=10; options.v_segments=10;
    options.smooth_iterations=25; options.smooth_strength=0.55;
    const auto patch=solve_quad_patch(curved_square(),options);
    assert(patch.mesh.quads.size()==100);
    assert(patch.quality.max_boundary_error < 1e-12);
    assert(patch.quality.minimum_face_area > 0);
    const auto &bottom_middle=patch.mesh.vertices[5];
    const auto &top_middle=patch.mesh.vertices[10*11+5];
    assert(bottom_middle.y < -0.11);
    assert(top_middle.y > 1.13);
  }
  {
    // Bridge to the existing Face Set/border guide extraction.
    TriangleMesh square={{{0,0,0},{1,0,0},{1,1,0},{0,1,0}},
                         {{0,1,2},{0,2,3}}};
    const GuideGraph graph=extract_region_boundaries(square,{1,1},true);
    assert(graph.paths.size()==1 && graph.paths[0].closed);
    auto boundary=patch_boundary_from_loop(graph,0,{0,1,2,3});
    QuadPatchOptions options;
    options.u_segments=3; options.v_segments=2;
    const auto result=solve_quad_patch(boundary,options);
    assert(result.mesh.quads.size()==6);
    assert(result.quality.max_boundary_error < 1e-13);
  }
  {
    auto incorrect=unit_square();
    incorrect.left.front().x += 0.02;
    bool failed=false;
    try { solve_quad_patch(incorrect,{}); }
    catch(const std::invalid_argument &) { failed=true; }
    assert(failed);
  }
  {
    auto degenerate=unit_square();
    degenerate.top={{0,0,0},{1,0,0}};
    degenerate.left={{0,0,0},{0,0,0}};
    degenerate.right={{1,0,0},{1,0,0}};
    bool failed=false;
    try { solve_quad_patch(degenerate,{}); }
    catch(const std::invalid_argument &) { failed=true; }
    assert(failed);
  }
  {
    auto invalid=QuadPatchOptions{};
    invalid.u_segments=0;
    bool failed=false;
    try { solve_quad_patch(unit_square(),invalid); }
    catch(const std::invalid_argument &) { failed=true; }
    assert(failed);
    invalid.u_segments=100000;
    invalid.v_segments=100000;
    failed=false;
    try { solve_quad_patch(unit_square(),invalid); }
    catch(const std::invalid_argument &) { failed=true; }
    assert(failed);
  }
  {
    auto invalid=QuadPatchOptions{};
    invalid.smooth_strength=std::numeric_limits<double>::quiet_NaN();
    bool failed=false;
    try { solve_quad_patch(unit_square(),invalid); }
    catch(const std::invalid_argument &) { failed=true; }
    assert(failed);
  }
  {
    GuideGraph graph;
    graph.paths.push_back({{0,1,2,3,0},true});
    graph.nodes={{{0,0,0}},{{1,0,0}},{{1,1,0}},{{0,1,0}}};
    bool failed=false;
    try { patch_boundary_from_loop(graph,0,{0,1,1,3}); }
    catch(const std::invalid_argument &) { failed=true; }
    assert(failed);
  }
  std::cout << "Constrained Coons quad patch tests passed.\n";
}
