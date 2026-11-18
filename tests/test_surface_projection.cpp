// SPDX-License-Identifier: MIT
#include "guidedquad/surface_projection.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace guidedquad;

static QuadPatchBoundary square_boundary() {
  return {{{0,0,0},{1,0,0}},{{1,0,0},{1,1,0}},
          {{0,1,0},{1,1,0}},{{0,0,0},{0,1,0}}};
}

static TriangleMesh source_grid(std::uint32_t n, double height) {
  TriangleMesh mesh;
  const double pi=3.14159265358979323846;
  for(std::uint32_t j=0;j<=n;++j) for(std::uint32_t i=0;i<=n;++i) {
    const double x=double(i)/n,y=double(j)/n;
    mesh.vertices.push_back({x,y,height*std::sin(pi*x)*std::sin(pi*y)});
  }
  for(std::uint32_t j=0;j<n;++j) for(std::uint32_t i=0;i<n;++i) {
    const std::uint32_t a=j*(n+1)+i,b=a+1,d=a+n+1,c=d+1;
    mesh.triangles.push_back({a,b,c});
    mesh.triangles.push_back({a,c,d});
  }
  return mesh;
}

static bool close(Vec3 a,Vec3 b,double eps=1e-13) {
  return std::abs(a.x-b.x)<=eps && std::abs(a.y-b.y)<=eps &&
         std::abs(a.z-b.z)<=eps;
}

static bool is_boundary(std::size_t k,std::uint32_t nu,std::uint32_t nv) {
  const std::size_t row=k/(nu+1),col=k%(nu+1);
  return row==0||row==nv||col==0||col==nu;
}

int main() {
  {
    QuadPatchOptions base;
    base.u_segments=12;
    base.v_segments=12;
    const QuadPatchMesh patch=solve_quad_patch(square_boundary(),base).mesh;
    const TriangleMesh surface=source_grid(28,0.16);

    SurfaceProjectionOptions options;
    options.iterations=3;
    options.strength=1.0;
    const auto projected=project_quad_patch_to_surface(
        surface,patch,base.u_segments,base.v_segments,options);
    assert(projected.mesh.vertices.size()==patch.vertices.size());
    assert(projected.mesh.quads==patch.quads);
    assert(projected.stats.accepted_iterations>0);
    assert(projected.stats.initial_rms_distance>0.02);
    assert(projected.stats.final_rms_distance<projected.stats.initial_rms_distance*0.25);
    assert(projected.stats.moved_vertices>50);
    assert(projected.stats.skipped_degenerate_triangles==0);
    for(std::size_t k=0;k<patch.vertices.size();++k) {
      if(is_boundary(k,base.u_segments,base.v_segments))
        assert(close(patch.vertices[k],projected.mesh.vertices[k]));
    }
    // Query pruning must substantially reduce triangle tests relative to
    // brute-force scanning every triangle for every nearest-point query.
    const std::uint64_t brute_tests=
        projected.stats.closest_queries*surface.triangles.size();
    assert(projected.stats.triangle_tests<brute_tests/3);
    std::cout<<"BVH: "<<projected.stats.closest_queries<<" nearest queries, "
             <<projected.stats.triangle_tests<<" triangle tests vs "
             <<brute_tests<<" brute-force, RMS "
             <<projected.stats.initial_rms_distance<<" -> "
             <<projected.stats.final_rms_distance<<'\n';
  }
  {
    // No interior vertices: no boundary vertex is ever moved.
    QuadPatchOptions options;
    options.u_segments=1;options.v_segments=1;
    const auto patch=solve_quad_patch(square_boundary(),options).mesh;
    const auto result=project_quad_patch_to_surface(source_grid(4,0.15),patch,1,1);
    assert(result.stats.closest_queries==0);
    assert(result.mesh.vertices.size()==patch.vertices.size());
    for(std::size_t i=0;i<patch.vertices.size();++i)
      assert(close(result.mesh.vertices[i],patch.vertices[i]));
  }
  {
    // Skip degenerate reference triangles, without accepting an empty BVH.
    QuadPatchOptions options;
    options.u_segments=3;options.v_segments=3;
    const auto patch=solve_quad_patch(square_boundary(),options).mesh;
    TriangleMesh surface=source_grid(5,0.05);
    surface.triangles.push_back({0,0,1});
    const auto result=project_quad_patch_to_surface(surface,patch,3,3);
    assert(result.stats.skipped_degenerate_triangles==1);
    bool failed=false;
    try {
      TriangleMesh empty={{{0,0,0},{1,0,0}},{{0,0,1}}};
      (void)project_quad_patch_to_surface(empty,patch,3,3);
    } catch(const std::invalid_argument&) { failed=true; }
    assert(failed);
  }
  {
    // Topology changes/incorrect subdivision counts must fail explicitly.
    QuadPatchOptions options;
    options.u_segments=3;options.v_segments=3;
    auto patch=solve_quad_patch(square_boundary(),options).mesh;
    bool failed=false;
    try { (void)project_quad_patch_to_surface(source_grid(4,0.1),patch,4,3); }
    catch(const std::invalid_argument&) { failed=true; }
    assert(failed);
    std::swap(patch.quads[0][1],patch.quads[0][2]);
    failed=false;
    try { (void)project_quad_patch_to_surface(source_grid(4,0.1),patch,3,3); }
    catch(const std::invalid_argument&) { failed=true; }
    assert(failed);
  }
  {
    QuadPatchOptions options;
    options.u_segments=3;options.v_segments=3;
    const auto patch=solve_quad_patch(square_boundary(),options).mesh;
    SurfaceProjectionOptions bad;
    bad.strength=std::numeric_limits<double>::quiet_NaN();
    bool failed=false;
    try { (void)project_quad_patch_to_surface(source_grid(4,0.1),patch,3,3,bad); }
    catch(const std::invalid_argument&) { failed=true; }
    assert(failed);
  }
  std::cout<<"Guarded surface projection tests passed.\n";
}
