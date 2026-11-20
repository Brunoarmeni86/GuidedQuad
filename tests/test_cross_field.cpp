// SPDX-License-Identifier: MIT
#include "guidedquad/cross_field.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace guidedquad;

static TriangleMesh square() {
  return {{{0,0,0},{1,0,0},{1,1,0},{0,1,0}}, {{0,1,2},{0,2,3}}};
}
static double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
static double length(Vec3 a) { return std::sqrt(dot(a,a)); }
static bool same_cross(Vec3 a, Vec3 b) {
  const double d=dot(a,b)/(length(a)*length(b));
  // Parallel OR perpendicular is the same four-way field orientation.
  return std::min(std::abs(1.0-std::abs(d)), std::abs(d)) < 1e-5;
}
static bool caught_invalid(const TriangleMesh &m, const std::vector<FlowConstraint> &g) {
  try { (void)solve_cross_field(m,g); }
  catch (const std::invalid_argument &) { return true; }
  return false;
}
int main() {
  const auto mesh=square();
  {
    auto out=solve_cross_field(mesh, {{0,{1,0,0},1,true}});
    assert(out.triangles.size()==2);
    assert(same_cross(out.triangles[0].direction,{1,0,0}));
    assert(same_cross(out.triangles[1].direction,{1,0,0}));
    for(const auto &s: out.triangles) assert(std::abs(length(s.direction)-1.0)<1e-10);
    assert(out.triangles[0].coherence==1.0);
    auto repeat=solve_cross_field(mesh, {{0,{1,0,0},1,true}});
    assert(out.triangles[1].direction.x==repeat.triangles[1].direction.x);
  }
  {
    // A 90-degree rotation is equivalent in a cross-field, including for locked guides.
    auto out=solve_cross_field(mesh, {{0,{1,0,0},1,true},{0,{0,1,0},1,true}});
    assert(same_cross(out.triangles[0].direction,{0,1,0}));
    assert(out.triangles.size()==2);
  }
  {
    assert(caught_invalid(mesh,{{0,{1,0,0},1,true},{0,{1,1,0},1,true}}));
    assert(caught_invalid(mesh,{{2,{1,0,0},1,true}}));
    assert(caught_invalid(mesh,{{0,{0,0,1},1,true}}));
    assert(caught_invalid(mesh,{{0,{1,0,0},-1,false}}));
  }
  {
    auto out=solve_cross_field(mesh, {});
    assert(out.triangles.size()==2);
    assert(!out.warnings.empty());
  }
  {
    // Folded mesh: guide aligned to the shared edge on one triangle is
    // transported consistently through different face tangent planes.
    TriangleMesh fold={{{0,0,0},{1,0,0},{0,1,0},{0,0,1}},{{0,1,2},{1,0,3}}};
    auto out=solve_cross_field(fold,{{0,{1,0,0},1,true}});
    assert(same_cross(out.triangles[0].direction,{1,0,0}));
    assert(same_cross(out.triangles[1].direction,{1,0,0}));
  }
  {
    TriangleMesh nm={{{0,0,0},{1,0,0},{0,1,0},{0,-1,0},{0,0,1}},
                     {{0,1,2},{1,0,3},{0,1,4}}};
    assert(caught_invalid(nm,{}));
    TriangleMesh bad={{{0,0,0},{1,0,0},{2,0,0}},{{0,1,2}}};
    assert(caught_invalid(bad,{}));
  }
  std::cout<<"Cross field tests passed.\n";
}
