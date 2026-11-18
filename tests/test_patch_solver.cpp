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
    assert(patch.quality.minimum_scaled_jacobian > 1.0-1e-12);
    assert(patch.quality.max_diagonal_warpage_deg < 1e-6);
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
    QuadPatchOptions baseline_opts=options;
    const auto baseline=solve_quad_patch(curved_square(),baseline_opts);
    options.smooth_iterations=25; options.smooth_strength=0.55;
    const auto patch=solve_quad_patch(curved_square(),options);
    std::cout << "Curved patch: Coons aspect=" << baseline.quality.worst_edge_aspect_ratio
              << ", smoothed aspect=" << patch.quality.worst_edge_aspect_ratio
              << ", Coons ortho=" << baseline.quality.mean_abs_orthogonality
              << ", smoothed ortho=" << patch.quality.mean_abs_orthogonality
              << ", max boundary error=" << patch.quality.max_boundary_error << "\\n";
    assert(patch.mesh.quads.size()==100);
    assert(patch.quality.max_boundary_error < 1e-12);
    assert(patch.quality.minimum_face_area > 0);
    // The optimizer must not accept a large aspect-ratio regression.
    assert(patch.quality.worst_edge_aspect_ratio <=
           baseline.quality.worst_edge_aspect_ratio * 1.020000001);
    assert(patch.quality.mean_abs_orthogonality <=
           baseline.quality.mean_abs_orthogonality + 1e-12);
    // Regression: adaptive line search finds a useful step in this case.
    assert(patch.quality.mean_abs_orthogonality <
           baseline.quality.mean_abs_orthogonality - 0.001);
    assert(patch.accepted_smooth_iterations > 0);
    assert(patch.initial_quality.worst_edge_aspect_ratio ==
           baseline.quality.worst_edge_aspect_ratio);
    const auto &bottom_middle=patch.mesh.vertices[5];
    const auto &top_middle=patch.mesh.vertices[10*11+5];
    assert(bottom_middle.y < -0.11);
    assert(top_middle.y > 1.13);
  }
  {
    // Regression: formerly one straight segment silently erased a sharp
    // painted guide vertex but still claimed zero boundary deviation.
    auto sharp=unit_square();
    sharp.bottom={{0,0,0},{0.5,-0.2,0},{1,0,0}};
    QuadPatchOptions strict;
    strict.u_segments=1; strict.v_segments=1;
    bool insufficient=false;
    try { solve_quad_patch(sharp,strict); }
    catch (const std::invalid_argument &) { insufficient=true; }
    assert(insufficient);

    strict.u_segments=2;
    const auto preserved=solve_quad_patch(sharp,strict);
    assert(preserved.mesh.quads.size()==2);
    assert(preserved.mesh.vertices[1].x==0.5);
    assert(preserved.mesh.vertices[1].y==-0.2);
    assert(preserved.quality.max_boundary_error < 1e-12);

    // Approximation is opt-in and reports source-to-output deviation.
    strict.u_segments=1;
    strict.preserve_boundary_vertices=false;
    const auto approximated=solve_quad_patch(sharp,strict);
    assert(approximated.mesh.quads.size()==1);
    assert(std::fabs(approximated.quality.max_boundary_error-0.2) < 1e-12);
  }
  {
    // Hard constraints are honored even for an asymmetric arc-length layout:
    // extra divisions must never skip an input knot.
    auto uneven=unit_square();
    uneven.bottom={{0,0,0},{0.03,-0.04,0},{0.7,-0.1,0},{1,0,0}};
    QuadPatchOptions options;
    options.u_segments=7; options.v_segments=4;
    const auto patch=solve_quad_patch(uneven,options);
    bool found_first=false, found_second=false;
    for (std::size_t i=0;i<=options.u_segments;++i) {
      const auto p=patch.mesh.vertices[i];
      found_first |= (p.x==0.03 && p.y==-0.04);
      found_second |= (p.x==0.7 && p.y==-0.1);
    }
    assert(found_first && found_second);
    assert(patch.quality.max_boundary_error < 1e-12);
  }
  {
    // Non-planarity must be visible in the metric even if faces stay valid.
    auto twisted=unit_square();
    twisted.right.back().z=0.2;
    twisted.top.back().z=0.2;
    QuadPatchOptions options;
    options.u_segments=1; options.v_segments=1;
    const auto patch=solve_quad_patch(twisted,options);
    assert(patch.quality.max_diagonal_warpage_deg > 1.0);
    assert(patch.quality.minimum_scaled_jacobian > 0.0);
  }
  {
    // Bow-tie with crossing edges is not a valid quad.
    QuadPatchBoundary crossing{
      {{0,0,0},{1,1,0}}, {{1,1,0},{1,0,0}},
      {{0,1,0},{1,0,0}}, {{0,0,0},{0,1,0}}};
    QuadPatchOptions options;
    options.u_segments=1; options.v_segments=1;
    bool invalid=false;
    try { solve_quad_patch(crossing,options); }
    catch (const std::runtime_error &) { invalid=true; }
    assert(invalid);
  }
  {
    // Identical consecutive guide points must fail rather than produce NaNs.
    auto degenerate=unit_square();
    degenerate.bottom={{0,0,0},{0,0,0},{1,0,0}};
    QuadPatchOptions options;
    options.u_segments=2; options.v_segments=1;
    bool invalid=false;
    try { solve_quad_patch(degenerate,options); }
    catch (const std::invalid_argument &) { invalid=true; }
    assert(invalid);
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
