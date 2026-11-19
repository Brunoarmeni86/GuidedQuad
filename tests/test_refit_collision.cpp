// SPDX-License-Identifier: MIT
#include "guidedquad/quad_intersections.hpp"
#include "guidedquad/surface_projection.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace guidedquad;
static QuadPatchMesh crossing_pair() {
  QuadPatchMesh m;
  m.vertices={{0,0,0},{1,0,0},{1,1,0},{0,1,0},
              {0.5,-0.1,-0.5},{0.5,1.1,-0.5},
              {0.5,1.1,0.5},{0.5,-0.1,0.5}};
  m.quads={{0,1,2,3},{4,5,6,7}};
  return m;
}
static QuadPatchBoundary square() {
  return {{{0,0,0},{1,0,0}},{{1,0,0},{1,1,0}},
          {{0,1,0},{1,1,0}},{{0,0,0},{0,1,0}}};
}
int main() {
  {
    auto patch=crossing_pair();
    PreparedQuadCollisionChecker prepared(patch);
    assert(prepared.check_initial(2).pairs.size()==1);
    assert(prepared.refit_count()==0);

    auto disjoint=patch.vertices;
    for(std::size_t k=4;k<8;++k)disjoint[k].x+=2.0;
    const auto result=prepared.check_positions(disjoint,2);
    assert(result.pairs.empty());
    assert(prepared.refit_count()==1);
    patch.vertices=disjoint;
    assert(find_quad_self_intersections(patch).pairs.empty());
    assert(prepared.check_initial().pairs.empty());

    auto invalid=disjoint;
    invalid[0].z=std::numeric_limits<double>::quiet_NaN();
    bool failed=false;
    try { (void)prepared.check_positions(invalid,2); }
    catch(const std::invalid_argument&) { failed=true; }
    assert(failed);
    assert(prepared.refit_count()==1);
    // Failed candidate must not overwrite the last valid BVH.
    assert(prepared.check_initial().pairs.empty());

    patch=crossing_pair();
    const auto crossing=prepared.check_positions(patch.vertices,2);
    assert(crossing.pairs.size()==1);
    assert(crossing.pairs[0][0]==0&&crossing.pairs[0][1]==1);
    assert(prepared.refit_count()==2);
  }
  {
    // A normal rectangular patch under mild time-varying deformation:
    // refitting must match fresh BVH rebuild exactly for each candidate.
    QuadPatchOptions opts;
    opts.u_segments=32;
    opts.v_segments=32;
    QuadPatchMesh patch=solve_quad_patch(square(),opts).mesh;
    PreparedQuadCollisionChecker prepared(patch);
    assert(prepared.check_initial().pairs.empty());
    const double pi=3.14159265358979323846;
    for(int n=0;n<8;++n) {
      for(std::size_t k=0;k<patch.vertices.size();++k) {
        const auto original=patch.vertices[k];
        patch.vertices[k].z=0.03*std::sin(pi*original.x)*
                              std::sin(pi*original.y)*(0.2*n);
      }
      const auto refit=prepared.check_positions(patch.vertices);
      const auto rebuilt=find_quad_self_intersections(patch);
      assert(refit.pairs==rebuilt.pairs);
    }
    assert(prepared.refit_count()==8);
  }
  {
    // Reproducible microbenchmark; wall-clock is diagnostic only, no
    // performance claim or flaky timing assertion.
    QuadPatchOptions opts;
    opts.u_segments=48;opts.v_segments=48;
    QuadPatchMesh patch=solve_quad_patch(square(),opts).mesh;
    const double pi=3.14159265358979323846;
    std::vector<std::vector<Vec3>> frames;
    for(int n=0;n<20;++n) {
      auto positions=patch.vertices;
      for(auto &p:positions)
        p.z=0.02*std::sin(pi*p.x)*std::sin(pi*p.y)*std::sin(0.25*n);
      frames.push_back(std::move(positions));
    }
    using Clock=std::chrono::steady_clock;
    std::uint64_t total_pairs=0;
    const auto t0=Clock::now();
    for(const auto &frame:frames) {
      patch.vertices=frame;
      total_pairs+=find_quad_self_intersections(patch,1).pairs.size();
    }
    const auto t1=Clock::now();
    PreparedQuadCollisionChecker shared(patch);
    const auto t2=Clock::now();
    for(const auto &frame:frames)
      total_pairs+=shared.check_positions(frame,1).pairs.size();
    const auto t3=Clock::now();
    assert(total_pairs==0);
    assert(shared.refit_count()==frames.size());
    const double rebuild_ms=
        std::chrono::duration<double,std::milli>(t1-t0).count();
    const double refit_ms=
        std::chrono::duration<double,std::milli>(t3-t2).count();
    std::cout<<"Collision BVH benchmark (48x48 quads, 20 frames): "
             <<rebuild_ms<<" ms rebuilt vs "<<refit_ms
             <<" ms refitted; 20 builds vs 1 build + "
             <<shared.refit_count()<<" refits\n";
  }
  std::cout<<"Refittable collision BVH tests passed.\n";
}
