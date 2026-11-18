// SPDX-License-Identifier: MIT
#include "guidedquad/quad_intersections.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace guidedquad {
namespace {
using Id=std::uint32_t;
Vec3 sub(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3 cross(Vec3 a,Vec3 b){
  return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
bool finite(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
double axis(Vec3 p,int d){return d==0?p.x:(d==1?p.y:p.z);}
struct Aabb {
  Vec3 lo{1e300,1e300,1e300},hi{-1e300,-1e300,-1e300};
  void extend(Vec3 p) {
    lo.x=std::min(lo.x,p.x);lo.y=std::min(lo.y,p.y);lo.z=std::min(lo.z,p.z);
    hi.x=std::max(hi.x,p.x);hi.y=std::max(hi.y,p.y);hi.z=std::max(hi.z,p.z);
  }
  void extend(Aabb b){extend(b.lo);extend(b.hi);}
  bool overlaps(Aabb b) const {
    const double eps=2e-14; // conservative contact tolerance in normalized units
    return lo.x<=b.hi.x+eps && hi.x+eps>=b.lo.x &&
           lo.y<=b.hi.y+eps && hi.y+eps>=b.lo.y &&
           lo.z<=b.hi.z+eps && hi.z+eps>=b.lo.z;
  }
};

struct Triangle {std::array<Vec3,3> v;};
bool separated_on_axis(const Triangle &a,const Triangle &b,Vec3 direction) {
  const double length=std::hypot(direction.x,direction.y,direction.z);
  if(!(length>1e-14) || !std::isfinite(length))return false;
  direction={direction.x/length,direction.y/length,direction.z/length};
  double amin=dot(a.v[0],direction),amax=amin;
  double bmin=dot(b.v[0],direction),bmax=bmin;
  for(int i=1;i<3;++i){
    const double av=dot(a.v[i],direction),bv=dot(b.v[i],direction);
    amin=std::min(amin,av);amax=std::max(amax,av);
    bmin=std::min(bmin,bv);bmax=std::max(bmax,bv);
  }
  // Tolerance for round-off; touching is conservatively considered collision.
  return amax<bmin-1e-13 || bmax<amin-1e-13;
}

bool triangle_intersection(const Triangle &a,const Triangle &b) {
  const Vec3 ae[3]={sub(a.v[1],a.v[0]),sub(a.v[2],a.v[1]),sub(a.v[0],a.v[2])};
  const Vec3 be[3]={sub(b.v[1],b.v[0]),sub(b.v[2],b.v[1]),sub(b.v[0],b.v[2])};
  const Vec3 an=cross(ae[0],ae[1]),bn=cross(be[0],be[1]);
  if(separated_on_axis(a,b,an)||separated_on_axis(a,b,bn))return false;
  for(Vec3 e:ae)for(Vec3 f:be) {
    if(separated_on_axis(a,b,cross(e,f)))return false;
  }
  // When triangles are coplanar, the usual edge-edge axes are perpendicular
  // to their shared plane and cannot resolve 2D polygon separation.
  for(Vec3 e:ae)if(separated_on_axis(a,b,cross(an,e)))return false;
  for(Vec3 e:be)if(separated_on_axis(a,b,cross(bn,e)))return false;
  return true;
}
bool share_vertex(const std::array<Id,4> &a,const std::array<Id,4> &b){
  for(Id i:a)for(Id j:b)if(i==j)return true;
  return false;
}
class QuadBvh {
  struct Node {
    Aabb box;
    Id first=0,count=0,left=0,right=0;
    bool leaf=false;
  };
  const QuadPatchMesh &mesh_;
  std::vector<Vec3> positions_;
  std::vector<Aabb> boxes_;
  std::vector<Vec3> centers_;
  std::vector<Id> indices_;
  std::vector<Node> nodes_;
  QuadIntersectionReport report_;
  std::size_t limit_;

  Id build(Id first,Id end){
    const Id self=static_cast<Id>(nodes_.size());
    nodes_.push_back({});
    Aabb box,centroid;
    for(Id i=first;i<end;++i){
      const Id q=indices_[i];box.extend(boxes_[q]);centroid.extend(centers_[q]);
    }
    nodes_[self].box=box;nodes_[self].first=first;nodes_[self].count=end-first;
    if(end-first<=8){nodes_[self].leaf=true;return self;}
    const Vec3 extent=sub(centroid.hi,centroid.lo);
    int dim=extent.y>extent.x?1:0;
    if(extent.z>axis(extent,dim))dim=2;
    const Id mid=first+(end-first)/2;
    std::nth_element(indices_.begin()+first,indices_.begin()+mid,indices_.begin()+end,
      [&](Id a,Id b){
        const double av=axis(centers_[a],dim),bv=axis(centers_[b],dim);
        return av<bv||(av==bv&&a<b);
      });
    const Id left=build(first,mid),right=build(mid,end);
    nodes_[self].left=left;nodes_[self].right=right;
    return self;
  }

  Triangle tri(Id q,int diagonal) const{
    const auto &ids=mesh_.quads[q];
    if(diagonal==0)return {{{positions_[ids[0]],positions_[ids[1]],positions_[ids[2]]}}};
    return {{{positions_[ids[0]],positions_[ids[2]],positions_[ids[3]]}}};
  }

  bool process_pair(Id a,Id b){
    if(a==b)return true;
    if(a>b)std::swap(a,b);
    const auto &qa=mesh_.quads[a],&qb=mesh_.quads[b];
    if(share_vertex(qa,qb)){
      ++report_.adjacent_pairs_ignored;
      return true;
    }
    if(!boxes_[a].overlaps(boxes_[b]))return true;
    ++report_.candidate_quad_pairs;
    for(int ia=0;ia<2;++ia){
      const Triangle ta=tri(a,ia);
      for(int ib=0;ib<2;++ib){
        ++report_.triangle_pair_tests;
        if(triangle_intersection(ta,tri(b,ib))){
          if(report_.pairs.size()>=limit_){report_.truncated=true;return false;}
          report_.pairs.push_back({a,b});
          if(report_.pairs.size()==limit_) {report_.truncated=true;return false;}
          return true;
        }
      }
    }
    return true;
  }

  bool traverse(Id a,Id b){
    const Node &na=nodes_[a],&nb=nodes_[b];
    ++report_.bvh_pair_tests;
    if(!na.box.overlaps(nb.box))return true;
    if(a==b){
      if(na.leaf){
        for(Id i=na.first;i<na.first+na.count;++i)
          for(Id j=i+1;j<na.first+na.count;++j)
            if(!process_pair(indices_[i],indices_[j]))return false;
        return true;
      }
      return traverse(na.left,na.left)&&traverse(na.left,na.right)&&
             traverse(na.right,na.right);
    }
    if(na.leaf&&nb.leaf){
      for(Id i=na.first;i<na.first+na.count;++i)
        for(Id j=nb.first;j<nb.first+nb.count;++j)
          if(!process_pair(indices_[i],indices_[j]))return false;
      return true;
    }
    if(!na.leaf&&(nb.leaf||na.count>=nb.count))
      return traverse(na.left,b)&&traverse(na.right,b);
    return traverse(a,nb.left)&&traverse(a,nb.right);
  }

 public:
  QuadBvh(const QuadPatchMesh &mesh,std::size_t limit):mesh_(mesh),limit_(limit){
    if(mesh.vertices.empty()||mesh.quads.empty()||
       mesh.vertices.size()>std::numeric_limits<Id>::max()||
       mesh.quads.size()>2000000ULL||limit==0){
      throw std::invalid_argument("invalid quad mesh or collision report limit");
    }
    Vec3 lo=mesh.vertices.front(),hi=lo;
    for(Vec3 p:mesh.vertices) {
      if(!finite(p))throw std::invalid_argument("quad mesh contains nonfinite coordinates");
      lo.x=std::min(lo.x,p.x);lo.y=std::min(lo.y,p.y);lo.z=std::min(lo.z,p.z);
      hi.x=std::max(hi.x,p.x);hi.y=std::max(hi.y,p.y);hi.z=std::max(hi.z,p.z);
    }
    const double scale=std::max({hi.x-lo.x,hi.y-lo.y,hi.z-lo.z});
    if(!(scale>0)||!std::isfinite(scale))
      throw std::invalid_argument("quad mesh bounds are degenerate or unrepresentable");
    positions_.reserve(mesh.vertices.size());
    for(Vec3 p:mesh.vertices){
      // Normalize geometry to a unit-scale box to avoid tiny-scale SAT issues.
      positions_.push_back({(p.x-lo.x)/scale,(p.y-lo.y)/scale,(p.z-lo.z)/scale});
    }
    boxes_.resize(mesh.quads.size());
    centers_.resize(mesh.quads.size());
    indices_.resize(mesh.quads.size());
    std::iota(indices_.begin(),indices_.end(),Id{0});
    for(Id i=0;i<mesh.quads.size();++i){
      const auto &q=mesh.quads[i];
      for(Id v:q)if(v>=mesh.vertices.size())
        throw std::invalid_argument("quad has invalid vertex index");
      for(int k=0;k<4;++k)for(int m=k+1;m<4;++m)if(q[k]==q[m])
        throw std::invalid_argument("quad has duplicate vertex indices");
      Aabb box;
      for(Id id:q)box.extend(positions_[id]);
      boxes_[i]=box;
      centers_[i]={(box.lo.x+box.hi.x)*0.5,
                   (box.lo.y+box.hi.y)*0.5,
                   (box.lo.z+box.hi.z)*0.5};
      const Triangle a=tri(i,0),b=tri(i,1);
      const Vec3 na=cross(sub(a.v[1],a.v[0]),sub(a.v[2],a.v[0]));
      const Vec3 nb=cross(sub(b.v[1],b.v[0]),sub(b.v[2],b.v[0]));
      const double len_a=std::hypot(na.x,na.y,na.z);
      const double len_b=std::hypot(nb.x,nb.y,nb.z);
      if(!(len_a>1e-14&&len_b>1e-14)||dot(na,nb)<=0)
        throw std::invalid_argument("quad has degenerate or folded split triangles");
    }
    nodes_.reserve(mesh.quads.size()/4+16);
    build(0,static_cast<Id>(mesh.quads.size()));
  }
  QuadIntersectionReport query() {
    traverse(0,0);
    return std::move(report_);
  }
};
} // namespace

QuadIntersectionReport find_quad_self_intersections(
    const QuadPatchMesh &mesh,std::size_t max_reported_pairs) {
  QuadBvh bvh(mesh,max_reported_pairs);
  return bvh.query();
}

} // namespace guidedquad
