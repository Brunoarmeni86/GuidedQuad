// SPDX-License-Identifier: MIT
#include "guidedquad/quad_intersections.hpp"
#include "guidedquad/patch_solver.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace guidedquad;
static QuadPatchBoundary square() {
  return {{{0,0,0},{1,0,0}},{{1,0,0},{1,1,0}},
          {{0,1,0},{1,1,0}},{{0,0,0},{0,1,0}}};
}
static QuadPatchMesh separate_quads() {
  QuadPatchMesh mesh;
  mesh.vertices={{0,0,0},{1,0,0},{1,1,0},{0,1,0},
                 {2,0,0},{3,0,0},{3,1,0},{2,1,0}};
  mesh.quads={{0,1,2,3},{4,5,6,7}};
  return mesh;
}
static std::vector<std::array<std::uint32_t,2>> sorted_pairs(QuadIntersectionReport r) {
  std::sort(r.pairs.begin(),r.pairs.end());
  return std::move(r.pairs);
}
int main() {
  {
    auto patch=separate_quads();
    PreparedQuadCollisionChecker prepared(patch);
    bool no_baseline=false;
    try {(void)prepared.check_positions_dirty(patch.vertices,{4,5,6,7});}
    catch(const std::logic_error&) {no_baseline=true;}
    assert(no_baseline);
    assert(prepared.check_initial().pairs.empty());
    assert(prepared.check_positions_dirty(patch.vertices,{}).pairs.empty());
    assert(prepared.dirty_update_count()==0);

    // Move an independent quad into the first. A dirty-face check must
    // discover the overlap with a stationary face outside the dirty set.
    const auto pristine=patch.vertices;
    auto colliding=patch.vertices;
    for(std::size_t k=4;k<8;++k)colliding[k].x-=1.5;
    bool missing_index=false;
    try {(void)prepared.check_positions_dirty(colliding,{4,5,6});}
    catch(const std::invalid_argument&) {missing_index=true;}
    assert(missing_index);
    assert(prepared.refit_count()==0);
    assert(prepared.check_initial().pairs.empty());

    const auto found=prepared.check_positions_dirty(colliding,{4,5,6,7});
    assert(found.pairs.size()==1);
    assert(found.pairs.front()[0]==0 && found.pairs.front()[1]==1);
    assert(found.dirty_face_queries==1);
    assert(prepared.check_initial().pairs.empty()); // rolled back
    assert(prepared.dirty_update_count()==1);

    auto safely_moved=pristine;
    for(std::size_t k=4;k<8;++k)safely_moved[k].x+=0.2;
    const auto safe=prepared.check_positions_dirty(safely_moved,{4,5,6,7});
    assert(safe.pairs.empty());
    assert(prepared.check_initial().pairs.empty());
    const auto same=prepared.check_positions_dirty(safely_moved,{});
    assert(same.pairs.empty());
    assert(same.dirty_face_queries==0);
    assert(prepared.dirty_update_count()==2);
  }
  {
    // A globally invalid initial mesh cannot be incrementally validated,
    // because a collision between two unchanged faces could be missed.
    auto invalid=separate_quads();
    for(std::size_t k=4;k<8;++k)invalid.vertices[k].x-=1.5;
    PreparedQuadCollisionChecker checker(invalid);
    assert(checker.check_initial().pairs.size()==1);
    bool rejected=false;
    try {(void)checker.check_positions_dirty(invalid.vertices,{4});}
    catch(const std::logic_error&) {rejected=true;}
    assert(rejected);
  }
  {
    QuadPatchOptions opts;opts.u_segments=36;opts.v_segments=36;
    auto grid=solve_quad_patch(square(),opts).mesh;
    PreparedQuadCollisionChecker prepared(grid);
    assert(prepared.check_initial().pairs.empty());
    const std::uint32_t center=18*(opts.u_segments+1)+18;
    auto changed=grid.vertices;
    for(int i=0;i<12;++i) {
      changed[center].z=0.004*(i+1);
      grid.vertices=changed;
      const auto dirty=prepared.check_positions_dirty(changed,{center,center});
      const auto full=find_quad_self_intersections(grid);
      assert(sorted_pairs(dirty)==sorted_pairs(full));
      assert(dirty.pairs.empty());
      assert(dirty.dirty_face_queries==4);
      assert(dirty.candidate_quad_pairs < full.candidate_quad_pairs);
    }
    assert(prepared.dirty_update_count()==12);
  }
  {
    // Same exact frames: whole-tree query versus dirty-only query. The refit
    // itself remains O(V+F), but dirty traversals should prune global
    // unchanged pairs. Never fail CI on noisy wall-clock measurements.
    QuadPatchOptions opts;opts.u_segments=48;opts.v_segments=48;
    const auto original=solve_quad_patch(square(),opts).mesh;
    const std::uint32_t center=24*(opts.u_segments+1)+24;
    std::vector<std::vector<Vec3>> frames;
    for(int i=0;i<24;++i) {
      auto vertices=original.vertices;
      vertices[center].z=0.003+0.0001*i;
      frames.push_back(std::move(vertices));
    }
    using Clock=std::chrono::steady_clock;
    auto t0=Clock::now();
    PreparedQuadCollisionChecker full(original);
    assert(full.check_initial(1).pairs.empty());
    std::uint64_t full_triangle_tests=0;
    for(const auto &frame:frames) {
      const auto result=full.check_positions(frame,1);
      assert(result.pairs.empty());
      full_triangle_tests+=result.triangle_pair_tests;
    }
    auto t1=Clock::now();
    PreparedQuadCollisionChecker partial(original);
    assert(partial.check_initial(1).pairs.empty());
    std::uint64_t dirty_triangle_tests=0,dirty_faces=0;
    for(const auto &frame:frames) {
      const auto result=partial.check_positions_dirty(frame,{center},1);
      assert(result.pairs.empty());
      dirty_triangle_tests+=result.triangle_pair_tests;
      dirty_faces+=result.dirty_face_queries;
    }
    auto t2=Clock::now();
    const double full_ms=std::chrono::duration<double,std::milli>(t1-t0).count();
    const double dirty_ms=std::chrono::duration<double,std::milli>(t2-t1).count();
    assert(dirty_triangle_tests<full_triangle_tests);
    assert(dirty_faces==frames.size()*4);
    std::cout<<"Dirty collision benchmark (48x48 quads, 24 sparse frames): "
             <<full_ms<<" ms full vs "<<dirty_ms<<" ms dirty; SAT triangle pairs "
             <<full_triangle_tests<<" full vs "<<dirty_triangle_tests
             <<" dirty; dirty quad queries "<<dirty_faces<<'\n';
  }
  std::cout<<"Dirty collision BVH tests passed.\n";
}
