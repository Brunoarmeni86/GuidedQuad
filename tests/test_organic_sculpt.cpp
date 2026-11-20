// SPDX-License-Identifier: MIT
// Procedural organic-shaped benchmark, NOT a scanned or artist-produced asset.
#include "guidedquad/quad_intersections.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace guidedquad;

static constexpr double PI=3.1415926535897932384626433832795;

static QuadPatchMesh organic_chart(std::uint32_t n) {
  QuadPatchMesh out;
  out.vertices.reserve(std::size_t(n+1)*(n+1));
  out.quads.reserve(std::size_t(n)*n);
  for(std::uint32_t j=0;j<=n;++j)for(std::uint32_t i=0;i<=n;++i){
    const double u=double(i)/n,v=double(j)/n;
    // Nonplanar multi-frequency curved patch: large bulge, directional
    // folds, and detail; no random distribution or input external asset.
    const double du=u-0.65,dv=v-0.39;
    const double bump=0.19*std::exp(-74*(du*du+dv*dv));
    const double detail=0.028*std::sin(11*PI*u)*std::sin(9*PI*v);
    out.vertices.push_back({
      u+0.024*std::sin(2*PI*v)*std::sin(PI*u),
      v+0.018*std::sin(2*PI*u)*std::sin(PI*v),
      0.10*std::sin(2*PI*u)*std::sin(3*PI*v)+bump+detail
    });
  }
  for(std::uint32_t j=0;j<n;++j)for(std::uint32_t i=0;i<n;++i) {
    const std::uint32_t a=j*(n+1)+i,b=a+1,d=a+n+1,c=d+1;
    out.quads.push_back({a,b,c,d});
  }
  return out;
}

struct BrushFrame {
  std::vector<Vec3> positions;
  std::vector<std::uint32_t> changed;
};
static BrushFrame brush_frame(const QuadPatchMesh &base,std::uint32_t n,
                              int frame,double radius,bool broad) {
  BrushFrame out;
  out.positions=base.vertices;
  const double cx=0.48+0.025*std::sin(frame*0.3);
  const double cy=0.51;
  const double amplitude=0.004*(1+(frame%4));
  for(std::uint32_t j=1;j<n;++j)for(std::uint32_t i=1;i<n;++i) {
    const std::size_t k=std::size_t(j)*(n+1)+i;
    const double u=double(i)/n,v=double(j)/n;
    const double d=std::hypot(u-cx,v-cy);
    const double r=broad?0.32:radius;
    if(d>r)continue;
    const double t=d/r;
    const double influence=std::pow(1-t*t,2);
    out.positions[k].z+=amplitude*influence;
    out.positions[k].x+=0.0005*influence*std::sin(frame*0.17);
    if(out.positions[k].x!=base.vertices[k].x ||
       out.positions[k].y!=base.vertices[k].y ||
       out.positions[k].z!=base.vertices[k].z)
      out.changed.push_back(static_cast<std::uint32_t>(k));
  }
  return out;
}
static void measure_case(std::uint32_t n,int frames,bool broad) {
  QuadPatchMesh source=organic_chart(n);
  auto reference=source;
  PreparedQuadCollisionChecker full(source),local(source);
  const auto baseline=full.check_initial(2);
  if(!baseline.pairs.empty()) {
    const auto first=baseline.pairs.front();
    std::cerr<<"BAD BASELINE n="<<n<<" broad="<<broad
             <<" collision "<<first[0]<<","<<first[1]
             <<" adjacent checked "<<baseline.adjacent_pairs_checked
             <<" candidate pairs "<<baseline.candidate_quad_pairs<<"\n";
    for(auto qid:first) {
      const auto &q=source.quads[qid];
      std::cerr<<"quad "<<qid<<": ";
      for(auto v:q) {
        const auto &p=source.vertices[v];
        std::cerr<<v<<"("<<p.x<<","<<p.y<<","<<p.z<<") ";
      }
      std::cerr<<"\n";
    }
  }
  assert(baseline.pairs.empty());
  assert(local.check_initial(1).pairs.empty());
  using Clock=std::chrono::steady_clock;
  double full_ms=0,local_ms=0;
  std::uint64_t full_nodes=0,local_nodes=0,updated_box_nodes=0;
  std::uint64_t total_faces=0;
  for(int i=0;i<frames;++i) {
    const auto sample=brush_frame(source,n,i,0.07,broad);
    reference.vertices=sample.positions;
    const auto full_start=Clock::now();
    const auto f=full.check_positions(sample.positions,1);
    const auto mid=Clock::now();
    const auto d=local.check_positions_dirty(sample.positions,sample.changed,1);
    const auto done=Clock::now();
    full_ms+=std::chrono::duration<double,std::milli>(mid-full_start).count();
    local_ms+=std::chrono::duration<double,std::milli>(done-mid).count();
    // For this fixture every candidate should remain clean; if it doesn't,
    // investigate the shape rather than accepting a numerically flaky test.
    assert(f.pairs.empty());
    assert(d.pairs==f.pairs);
    assert(d.pairs==find_quad_self_intersections(reference,1).pairs);
    full_nodes+=f.bvh_pair_tests;
    local_nodes+=d.bvh_pair_tests;
    total_faces+=d.dirty_face_queries;
  }
  updated_box_nodes=local.local_bvh_node_updates();
  assert(local.full_refit_fallback_count()==0);
  assert(local.local_refit_count()==std::uint64_t(frames));
  if(!broad) {
    assert(local_nodes<full_nodes);
    assert(total_faces<source.quads.size()*std::uint64_t(frames)/3);
  }
  std::cout<<std::fixed<<std::setprecision(3)
           <<"Organic procedural chart: "<<n<<"x"<<n<<" quads, "
           <<frames<<" frames, "<<(broad?"broad":"local")<<" brush; "
           <<"full="<<full_ms<<" ms, dirty="<<local_ms
           <<" ms, ratio="<<(local_ms>0?full_ms/local_ms:0)
           <<"; BVH tests "<<full_nodes<<" vs "<<local_nodes
           <<"; dirty quad visits "<<total_faces
           <<"; refitted box nodes "<<updated_box_nodes<<'\n';
}
int main() {
  measure_case(24,8,false);
  measure_case(48,6,false);
  measure_case(72,4,false);
  measure_case(48,4,true);
  std::cout<<"Organic procedural collision regression passed.\n";
}
