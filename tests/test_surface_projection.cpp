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
    // Cached initialization must prevent the duplicate first-iteration
    // nearest queries that used to waste one search per interior vertex.
    const std::uint64_t interior=(base.u_segments-1)*(base.v_segments-1);
    assert(projected.stats.closest_queries<=interior*3);

    std::cout<<"BVH: "<<projected.stats.closest_queries<<" nearest queries, "
             <<projected.stats.triangle_tests<<" triangle tests vs "
             <<brute_tests<<" brute-force, RMS "
             <<projected.stats.initial_rms_distance<<" -> "
             <<projected.stats.final_rms_distance<<'\n';
  }
  {
    // PreparedBVH owns an immutable copy. Project multiple different patches
    // through one reference index even after the original mesh is destroyed.
    const auto build=[]() {
      const TriangleMesh source=source_grid(28,0.16);
      return PreparedSurfaceProjector(source);
    };
    PreparedSurfaceProjector shared=build();
    assert(shared.active_triangle_count()==28*28*2);
    assert(shared.skipped_degenerate_triangles()==0);
    QuadPatchOptions q;
    q.u_segments=12;q.v_segments=12;
    const auto large=solve_quad_patch(square_boundary(),q).mesh;
    q.u_segments=8;q.v_segments=8;
    const auto small=solve_quad_patch(square_boundary(),q).mesh;
    SurfaceProjectionOptions options;
    const auto a=shared.project(large,12,12,options);
    const auto b=shared.project(small,8,8,options);
    const auto fresh=project_quad_patch_to_surface(source_grid(28,0.16),
                                                 large,12,12,options);
    assert(a.mesh.quads==fresh.mesh.quads);
    assert(a.mesh.vertices.size()==fresh.mesh.vertices.size());
    for(std::size_t k=0;k<a.mesh.vertices.size();++k)
      assert(close(a.mesh.vertices[k],fresh.mesh.vertices[k]));
    assert(std::abs(a.stats.final_rms_distance-fresh.stats.final_rms_distance)<1e-13);
    assert(a.stats.closest_queries==fresh.stats.closest_queries);
    assert(b.mesh.quads.size()==64);
    assert(b.stats.final_rms_distance<b.stats.initial_rms_distance);
    assert(a.stats.skipped_degenerate_triangles==0);
  }
  {
    // A thin scanned object can contain nearby but oppositely oriented
    // surface layers. The optional filter must select the compatible layer,
    // not merely the closest one in Euclidean distance.
    TriangleMesh layers;
    layers.vertices={{0,0,0.012},{1,0,0.012},{1,1,0.012},{0,1,0.012},
                     {0,0,0.04},{1,0,0.04},{1,1,0.04},{0,1,0.04}};
    layers.triangles={{0,2,1},{0,3,2},{4,5,6},{4,6,7}};
    PreparedSurfaceProjector shared(layers);
    QuadPatchOptions q;
    q.u_segments=8;q.v_segments=8;
    const auto patch=solve_quad_patch(square_boundary(),q).mesh;
    SurfaceProjectionOptions opts;
    opts.iterations=2;
    const auto nearest=shared.project(patch,8,8,opts);
    opts.min_reference_normal_dot=0.5;
    const auto oriented=shared.project(patch,8,8,opts);
    const std::size_t mid=4*9+4;
    assert(nearest.mesh.vertices[mid].z>0.009);
    assert(nearest.mesh.vertices[mid].z<0.019);
    assert(oriented.mesh.vertices[mid].z>0.035);
    assert(oriented.mesh.vertices[mid].z<0.045);
    assert(oriented.stats.normal_rejected_triangles>0);
    for(std::size_t k=0;k<patch.vertices.size();++k) {
      if(is_boundary(k,8,8)) {
        assert(close(patch.vertices[k],nearest.mesh.vertices[k]));
        assert(close(patch.vertices[k],oriented.mesh.vertices[k]));
      }
    }
    // If ONLY the incompatible layer exists, do not silently fall back.
    layers.vertices.resize(4);
    layers.triangles.resize(2);
    PreparedSurfaceProjector inverted_only(layers);
    bool rejected=false;
    try {
      (void)inverted_only.project(patch,8,8,opts);
    } catch(const std::runtime_error&) {rejected=true;}
    assert(rejected);
  }
  {
    // Projection distance is measured against original positions, not the
    // current accepted step. A remote scan surface must not attract the chart
    // across an unrelated feature or separate layer.
    QuadPatchOptions base;
    base.u_segments=6; base.v_segments=6;
    const auto patch=solve_quad_patch(square_boundary(),base).mesh;
    TriangleMesh remote=source_grid(8,0.0);
    for(auto &p:remote.vertices) p.z+=5.0;

    SurfaceProjectionOptions options;
    options.iterations=5;
    options.max_projection_distance=0.1;
    auto projected=project_quad_patch_to_surface(
        remote,patch,base.u_segments,base.v_segments,options);
    assert(projected.stats.distance_rejected_targets>0);
    assert(projected.stats.accepted_iterations==0);
    assert(projected.stats.moved_vertices==0);
    assert(projected.stats.max_vertex_displacement==0);
    assert(projected.stats.closest_queries==25); // initial pass only; no motion
    assert(projected.stats.final_rms_distance==
           projected.stats.initial_rms_distance);
    for(std::size_t k=0;k<patch.vertices.size();++k)
      assert(close(projected.mesh.vertices[k],patch.vertices[k]));
  }
  {
    // A small allowed radius permits partial but bounded projection without
    // stepping past the original-position radius on later iterations.
    QuadPatchOptions base;
    base.u_segments=12; base.v_segments=12;
    const auto patch=solve_quad_patch(square_boundary(),base).mesh;
    SurfaceProjectionOptions options;
    options.iterations=5;
    options.max_projection_distance=0.025;
    const auto projected=project_quad_patch_to_surface(
        source_grid(28,0.16),patch,base.u_segments,base.v_segments,options);
    assert(projected.stats.distance_rejected_targets>0);
    assert(projected.stats.max_vertex_displacement<=0.025+1e-12);
    for(std::size_t k=0;k<patch.vertices.size();++k) {
      if(is_boundary(k,base.u_segments,base.v_segments))
        assert(close(patch.vertices[k],projected.mesh.vertices[k]));
    }
  }
  {
    // With zero iterations the initial nearest queries already provide the
    // final RMS. No duplicate final query should run on unchanged geometry.
    QuadPatchOptions base;
    base.u_segments=4; base.v_segments=4;
    const auto patch=solve_quad_patch(square_boundary(),base).mesh;
    SurfaceProjectionOptions options;
    options.iterations=0;
    const auto result=project_quad_patch_to_surface(
        source_grid(8,0.1),patch,base.u_segments,base.v_segments,options);
    assert(result.stats.accepted_iterations==0);
    assert(result.stats.moved_vertices==0);
    assert(result.stats.initial_rms_distance==result.stats.final_rms_distance);
    assert(result.stats.closest_queries==9);
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
    bad.strength=1.0;
    bad.max_projection_distance=0.0;
    failed=false;
    try { (void)project_quad_patch_to_surface(source_grid(4,0.1),patch,3,3,bad); }
    catch(const std::invalid_argument&) { failed=true; }
    assert(failed);
    bad.max_projection_distance=1.0;
    bad.min_reference_normal_dot=1.5;
    failed=false;
    try { (void)project_quad_patch_to_surface(source_grid(4,0.1),patch,3,3,bad); }
    catch(const std::invalid_argument&) { failed=true; }
    assert(failed);
  }
  std::cout<<"Guarded surface projection tests passed.\n";
}
