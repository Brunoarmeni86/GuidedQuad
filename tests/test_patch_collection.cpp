// SPDX-License-Identifier: MIT
#include "guidedquad/quad_intersections.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace guidedquad;
static QuadPatchMesh one_quad(double x,double y,double z=0.0) {
  QuadPatchMesh q;
  q.vertices={{x,y,z},{x+1,y,z},{x+1,y+1,z},{x,y+1,z}};
  q.quads={{0,1,2,3}};
  return q;
}
int main() {
  {
    const auto a=one_quad(0,0);
    const auto b=one_quad(0.25,0.25);
    const auto result=find_patch_collection_intersections({a,b});
    assert(result.collisions.size()==1);
    const auto collision=result.collisions.front();
    assert(collision.patch_a==0&&collision.quad_a==0);
    assert(collision.patch_b==1&&collision.quad_b==0);
  }
  {
    const auto result=find_patch_collection_intersections({
      one_quad(0,0),one_quad(2,0)});
    assert(result.collisions.empty());
  }
  {
    // Explicit seam welding converts two independent chart boundaries into
    // legal topological neighbors, without blindly merging nearby scan layers.
    const auto a=one_quad(0,0),b=one_quad(1,0);
    const auto unwelded=find_patch_collection_intersections({a,b});
    assert(unwelded.collisions.size()==1); // geometric contact is conservative
    const std::vector<SharedPatchVertex> seam={
      {0,1,1,0},{0,2,1,3}
    };
    const auto welded=find_patch_collection_intersections({a,b},seam);
    assert(welded.collisions.empty());
    assert(welded.merged_report.adjacent_pairs_checked>0);
  }
  {
    // Same seam, but the two patches overlap on the SAME side of their
    // shared edge. Earlier "ignore any shared vertex" missed this defect.
    const auto a=one_quad(0,0);
    QuadPatchMesh b;
    b.vertices={{1,0,0},{0.3,0.2,0},{0.3,0.8,0},{1,1,0}};
    b.quads={{0,1,2,3}};
    const std::vector<SharedPatchVertex> seam={
      {0,1,1,0},{0,2,1,3}
    };
    const auto result=find_patch_collection_intersections({a,b},seam);
    assert(result.collisions.size()==1);
    assert(result.collisions[0].patch_a==0);
    assert(result.collisions[0].patch_b==1);
  }
  {
    // Two patches legitimately touch at a single shared corner.
    const auto a=one_quad(0,0),b=one_quad(1,1);
    const auto r=find_patch_collection_intersections({a,b},{{0,2,1,0}});
    assert(r.collisions.empty());
    // An overlapping quad sharing the corner must NOT be hidden by welding.
    QuadPatchMesh overlap;
    overlap.vertices={{1,1,0},{1,0.5,0},{0.5,0.5,0},{0.5,1,0}};
    overlap.quads={{0,1,2,3}};
    const auto bad=find_patch_collection_intersections({a,overlap},{{0,2,1,0}});
    assert(bad.collisions.size()==1);
  }
  {
    // Invalid seam mapping must be rejected rather than silently welding
    // physically separated coordinates.
    bool rejected=false;
    try {
      (void)find_patch_collection_intersections({
        one_quad(0,0),one_quad(2,0)},{{0,1,1,0}});
    } catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  {
    bool rejected=false;
    try { (void)find_patch_collection_intersections({}); }
    catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  std::cout << "Stitched cross-patch collision tests passed.\n";
}
