// SPDX-License-Identifier: MIT
#include "guidedquad/quad_intersections.hpp"
#include "guidedquad/surface_projection.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace guidedquad;

static QuadPatchBoundary square_boundary() {
  return {{{0,0,0},{1,0,0}},{{1,0,0},{1,1,0}},
          {{0,1,0},{1,1,0}},{{0,0,0},{0,1,0}}};
}
static QuadPatchMesh two_quads(bool coplanar, bool separated=false) {
  QuadPatchMesh m;
  m.vertices={{0,0,0},{1,0,0},{1,1,0},{0,1,0}};
  if(coplanar){
    const double dx=separated?2.0:0.25;
    m.vertices.insert(m.vertices.end(), {
      {dx,0.25,0},{dx+1,0.25,0},{dx+1,1.25,0},{dx,1.25,0}});
  } else {
    m.vertices.insert(m.vertices.end(),{
      {0.5,-0.1,-0.5},{0.5,1.1,-0.5},
      {0.5,1.1,0.5},{0.5,-0.1,0.5}});
  }
  m.quads={{0,1,2,3},{4,5,6,7}};
  return m;
}
static QuadPatchMesh overlapping_regular_grid() {
  // Smooth locally regular annular chart, with TWO overlapping turns.
  // This cannot be found by local quad Jacobian checks alone.
  const std::uint32_t nu=24,nv=2;
  const double pi=3.14159265358979323846;
  QuadPatchMesh m;
  for(std::uint32_t j=0;j<=nv;++j){
    const double r=1+0.2*j;
    for(std::uint32_t i=0;i<=nu;++i){
      const double t=4*pi*double(i)/nu;
      m.vertices.push_back({r*std::cos(t),r*std::sin(t),0});
    }
  }
  for(std::uint32_t j=0;j<nv;++j)for(std::uint32_t i=0;i<nu;++i){
    const std::uint32_t a=j*(nu+1)+i,b=a+1,d=a+nu+1,c=d+1;
    m.quads.push_back({a,b,c,d});
  }
  return m;
}
int main(){
  {
    const auto m=two_quads(false);
    const auto found=find_quad_self_intersections(m);
    assert(found.pairs.size()==1);
    assert(found.pairs[0][0]==0&&found.pairs[0][1]==1);
    assert(found.candidate_quad_pairs>0);
    assert(found.triangle_pair_tests>0);
    std::cout<<"Crossing quad pair detected; BVH visits "<<found.bvh_pair_tests<<'\n';
  }
  {
    const auto found=find_quad_self_intersections(two_quads(true));
    assert(found.pairs.size()==1); // coplanar overlapping faces
  }
  {
    const auto found=find_quad_self_intersections(two_quads(true,true));
    assert(found.pairs.empty()); // nonoverlapping AABBs
  }
  {
    // Precision regression: old 1e-13 SAT margin reported distinct sheets
    // separated by ~1e-14 as colliding even though the input double values
    // can represent that distance in this normalized coordinate system.
    auto layers=two_quads(true);
    for(std::size_t i=4;i<8;++i)layers.vertices[i].z=1e-14;
    assert(find_quad_self_intersections(layers).pairs.empty());
    for(std::size_t i=4;i<8;++i)layers.vertices[i].z=0.0;
    assert(find_quad_self_intersections(layers).pairs.size()==1);

    // Also distinguish separate nearly touching boundaries in-plane.
    auto separated=two_quads(true,true);
    for(std::size_t i=4;i<8;++i)
      separated.vertices[i].x-=1.0-1e-13;
    assert(find_quad_self_intersections(separated).pairs.empty());
    for(std::size_t i=4;i<8;++i)
      separated.vertices[i].x-=2e-9;
    assert(find_quad_self_intersections(separated).pairs.size()==1);
  }
  {
    // The predicates normalize the entire model to unit bounding scale.
    // Both very small and very large but representable coordinates must
    // give the same crossing and disjoint classifications.
    for(double scale:{1e-10,1e10,1e120}) {
      auto crossing=two_quads(false);
      auto disjoint=two_quads(true,true);
      for(auto *mesh:{&crossing,&disjoint})
        for(auto &p:mesh->vertices){p.x*=scale;p.y*=scale;p.z*=scale;}
      assert(find_quad_self_intersections(crossing).pairs.size()==1);
      assert(find_quad_self_intersections(disjoint).pairs.empty());
    }
  }
  {
    QuadPatchOptions o;
    o.u_segments=10;o.v_segments=10;
    const auto grid=solve_quad_patch(square_boundary(),o).mesh;
    const auto found=find_quad_self_intersections(grid);
    assert(found.pairs.empty());
    assert(found.adjacent_pairs_ignored>0);
    assert(found.shared_feature_fastpath_skips>0);
    // Candidate count is far below an exhaustive O(Q^2) scan.
    const std::uint64_t all_pairs=grid.quads.size()*(grid.quads.size()-1)/2;
    assert(found.candidate_quad_pairs<all_pairs/10);
    std::cout<<"Disjoint 10x10 quad grid: "<<found.candidate_quad_pairs
             <<" candidate pairs vs "<<all_pairs
             <<" exhaustive quad pairs, "<<found.bvh_pair_tests
             <<" BVH box-pair tests\n";
  }
  {
    // Two edge-sharing quads touch but must not be reported as collisions.
    QuadPatchMesh m;
    m.vertices={{0,0,0},{1,0,0},{1,1,0},{0,1,0},{2,0,0},{2,1,0}};
    m.quads={{0,1,2,3},{1,4,5,2}};
    const auto found=find_quad_self_intersections(m);
    assert(found.pairs.empty());
    assert(found.adjacent_pairs_ignored==1);
  }
  {
    // A globally overlapping two-turn mesh still passes ordinary per-quad
    // orientation checks, but must fail collision validation.
    const auto m=overlapping_regular_grid();
    const auto found=find_quad_self_intersections(m,1);
    assert(found.pairs.size()==1);
    assert(found.truncated);
    TriangleMesh source;
    source.vertices={{-2,-2,0},{2,-2,0},{2,2,0},{-2,2,0}};
    source.triangles={{0,1,2},{0,2,3}};
    bool rejected=false;
    try {
      (void)project_quad_patch_to_surface(source,m,24,2);
    } catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
    // Explicit opt-out is allowed only for diagnostics / benchmarking.
    SurfaceProjectionOptions unguarded;
    unguarded.reject_self_intersections=false;
    const auto unchecked=project_quad_patch_to_surface(source,m,24,2,unguarded);
    assert(unchecked.stats.collision_validation_passes==0);
  }
  {
    auto invalid=two_quads(false);
    invalid.quads[1][3]=9999999;
    bool rejected=false;
    try { (void)find_quad_self_intersections(invalid); }
    catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
    invalid=two_quads(false);
    invalid.vertices[0].z=std::numeric_limits<double>::quiet_NaN();
    rejected=false;
    try { (void)find_quad_self_intersections(invalid); }
    catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  std::cout<<"Global quad collision tests passed.\n";
}
