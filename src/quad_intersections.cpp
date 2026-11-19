// SPDX-License-Identifier: MIT
#include "guidedquad/quad_intersections.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
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
  // The geometry is normalized to unit-scale. The previous 1e-13 fixed
  // tolerance incorrectly classified distinct parallel layers separated by
  // ~1e-14 as collisions. Keep a smaller conservative numeric error margin.
  constexpr double epsilon=4e-15;
  return amax<bmin-epsilon || bmax<amin-epsilon;
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
struct SharedFeature {
  std::array<Id,4> indices{};
  std::uint32_t count=0;
};
SharedFeature shared_vertices(const std::array<Id,4> &a,
                              const std::array<Id,4> &b) {
  SharedFeature shared;
  for(Id i:a)for(Id j:b)if(i==j) {
    shared.indices[shared.count++]=i;
    break;
  }
  return shared;
}

struct Vec2 { double x,y; };
Vec2 project(Vec3 p,int drop) {
  return drop==0?Vec2{p.y,p.z}:
         drop==1?Vec2{p.x,p.z}:Vec2{p.x,p.y};
}
double orient(Vec2 a,Vec2 b,Vec2 c) {
  // Filtered 2D determinant. Most clearly noncollinear triples can be
  // resolved with double precision. Near cancellation uses extended
  // intermediates where supported, without claiming exact predicates.
  const double abx=b.x-a.x,aby=b.y-a.y;
  const double acx=c.x-a.x,acy=c.y-a.y;
  const double p=abx*acy,q=aby*acx,det=p-q;
  const double err=16.0*std::numeric_limits<double>::epsilon()*
                    (std::abs(p)+std::abs(q));
  if(std::isfinite(det)&&std::abs(det)>err)return det;
  const long double labx=static_cast<long double>(b.x)-a.x;
  const long double laby=static_cast<long double>(b.y)-a.y;
  const long double lacx=static_cast<long double>(c.x)-a.x;
  const long double lacy=static_cast<long double>(c.y)-a.y;
  return static_cast<double>(labx*lacy-laby*lacx);
}
double coplanar_overlap_area(const Triangle &a,const Triangle &b,Vec3 normal){
  int drop=0;
  if(std::abs(normal.y)>std::abs(normal.x))drop=1;
  if(std::abs(normal.z)>std::abs(axis(normal,drop)))drop=2;
  const Vec2 ba[3]={project(b.v[0],drop),project(b.v[1],drop),project(b.v[2],drop)};
  const double winding=orient(ba[0],ba[1],ba[2])>=0.0?1.0:-1.0;
  std::vector<Vec2> poly={project(a.v[0],drop),project(a.v[1],drop),project(a.v[2],drop)};
  const Vec2 aa[3]={project(a.v[0],drop),project(a.v[1],drop),project(a.v[2],drop)};
  const double area_a=0.5*std::abs(orient(aa[0],aa[1],aa[2]));
  const double area_b=0.5*std::abs(orient(ba[0],ba[1],ba[2]));
  // Area predicates must scale with *local triangle area*, not the whole
  // sculpt bounding box. A fixed 1e-12 tolerance erased 2e-7 sized quads.
  const double eps=std::max(1e-28,1e-12*std::min(area_a,area_b));
  for(int edge=0;edge<3;++edge) {
    const Vec2 p=ba[edge],q=ba[(edge+1)%3];
    std::vector<Vec2> clipped;
    if(poly.empty())return 0.0;
    clipped.reserve(poly.size()+2);
    for(std::size_t i=0;i<poly.size();++i) {
      const Vec2 before=poly[i],after=poly[(i+1)%poly.size()];
      const double d0=winding*orient(p,q,before);
      const double d1=winding*orient(p,q,after);
      const bool in0=d0>=-eps,in1=d1>=-eps;
      if(in0!=in1) {
        const double den=d0-d1;
        const double t=std::abs(den)>1e-30?
            std::clamp(d0/den,0.0,1.0):0.5;
        clipped.push_back({before.x+t*(after.x-before.x),
                           before.y+t*(after.y-before.y)});
      }
      if(in1)clipped.push_back(after);
    }
    poly.swap(clipped);
  }
  if(poly.size()<3)return 0.0;
  double twice_area=0;
  for(std::size_t i=0;i<poly.size();++i) {
    const Vec2 p=poly[i],q=poly[(i+1)%poly.size()];
    twice_area+=p.x*q.y-p.y*q.x;
  }
  return std::abs(twice_area)*0.5;
}
double feature_distance(Vec3 p,const SharedFeature &shared,
                        const std::vector<Vec3> &positions){
  if(shared.count==1) {
    const Vec3 d=sub(p,positions[shared.indices[0]]);
    return std::hypot(d.x,d.y,d.z);
  }
  if(shared.count==2) {
    const Vec3 a=positions[shared.indices[0]],b=positions[shared.indices[1]];
    const Vec3 ab=sub(b,a),ap=sub(p,a);
    const double den=dot(ab,ab);
    const double t=den>0?std::clamp(dot(ap,ab)/den,0.0,1.0):0.0;
    const Vec3 q{a.x+t*ab.x,a.y+t*ab.y,a.z+t*ab.z};
    const Vec3 d=sub(p,q);
    return std::hypot(d.x,d.y,d.z);
  }
  return 0.0;
}
// If the AABB intersection is contained wholly inside the exact shared
// vertex or shared edge, every possible geometric intersection must be a
// legal contact. This is conservative and only skips narrowphase when it can
// be proven unnecessary from bounds alone; warped/folded neighbors still go
// through the full triangle tests.
bool bounds_touch_only_shared_feature(
    const Aabb &a,const Aabb &b,const SharedFeature &shared,
    const std::vector<Vec3> &positions) {
  if(shared.count==0||shared.count>2)return false;
  const Vec3 lo{std::max(a.lo.x,b.lo.x),std::max(a.lo.y,b.lo.y),
                std::max(a.lo.z,b.lo.z)};
  const Vec3 hi{std::min(a.hi.x,b.hi.x),std::min(a.hi.y,b.hi.y),
                std::min(a.hi.z,b.hi.z)};
  // A small gap can be admitted by the broadphase tolerance; preserve the
  // full narrowphase for those cases to avoid numerical false negatives.
  if(lo.x>hi.x||lo.y>hi.y||lo.z>hi.z)return false;
  constexpr double tol=2e-15; // normalized unit extent, below SAT separation tolerance
  if(shared.count==1) {
    const Vec3 p=positions[shared.indices[0]];
    return std::abs(lo.x-p.x)<=tol&&std::abs(hi.x-p.x)<=tol&&
           std::abs(lo.y-p.y)<=tol&&std::abs(hi.y-p.y)<=tol&&
           std::abs(lo.z-p.z)<=tol&&std::abs(hi.z-p.z)<=tol;
  }
  // Fast exact-box test for an axis-aligned shared edge. Its AABB IS the
  // shared segment; the intersection AABB must fit entirely within it.
  // A tilted shared edge needs the more expensive 8-corner feature test.
  const Vec3 e0=positions[shared.indices[0]];
  const Vec3 e1=positions[shared.indices[1]];
  const Vec3 edge=sub(e1,e0);
  const int varying=(std::abs(edge.x)>tol)+(std::abs(edge.y)>tol)+
                    (std::abs(edge.z)>tol);
  if(varying<=1) {
    const Vec3 el{std::min(e0.x,e1.x),std::min(e0.y,e1.y),
                  std::min(e0.z,e1.z)};
    const Vec3 eh{std::max(e0.x,e1.x),std::max(e0.y,e1.y),
                  std::max(e0.z,e1.z)};
    return lo.x>=el.x-tol&&hi.x<=eh.x+tol &&
           lo.y>=el.y-tol&&hi.y<=eh.y+tol &&
           lo.z>=el.z-tol&&hi.z<=eh.z+tol;
  }
  for(int mask=0;mask<8;++mask) {
    const Vec3 p{mask&1?hi.x:lo.x,mask&2?hi.y:lo.y,
                 mask&4?hi.z:lo.z};
    if(feature_distance(p,shared,positions)>tol)return false;
  }
  return true;
}

bool point_in_triangle(Vec3 p,const Triangle &t,Vec3 n){
  for(int i=0;i<3;++i) {
    const Vec3 edge=sub(t.v[(i+1)%3],t.v[i]);
    const Vec3 ap=sub(p,t.v[i]);
    if(dot(cross(edge,ap),n)<-1e-14)return false;
  }
  return true;
}
bool edge_crosses_triangle_away(const Triangle &a,const Triangle &b,
                                const SharedFeature &shared,
                                const std::vector<Vec3> &positions) {
  const Vec3 n=cross(sub(b.v[1],b.v[0]),sub(b.v[2],b.v[0]));
  const double norm=std::hypot(n.x,n.y,n.z);
  if(!(norm>1e-14))return false;
  const Vec3 unit{n.x/norm,n.y/norm,n.z/norm};
  for(int i=0;i<3;++i) {
    const Vec3 p=a.v[i],q=a.v[(i+1)%3];
    const double d0=dot(sub(p,b.v[0]),unit);
    const double d1=dot(sub(q,b.v[0]),unit);
    if((d0>1e-12&&d1>1e-12)||(d0< -1e-12&&d1< -1e-12))
      continue;
    if(std::abs(d0-d1)<1e-14)continue; // coplanar segment handled separately
    const double t=d0/(d0-d1);
    if(t< -1e-12||t>1.0+1e-12)continue;
    const double u=std::clamp(t,0.0,1.0);
    const Vec3 hit{p.x+u*(q.x-p.x),p.y+u*(q.y-p.y),p.z+u*(q.z-p.z)};
    if(point_in_triangle(hit,b,n) &&
       feature_distance(hit,shared,positions)>1e-12)return true;
  }
  return false;
}
// Both triangles may touch at one legitimate shared vertex or shared edge.
// Only report a witness away from that allowed contact feature.
bool illegal_adjacent_intersection(const Triangle &a,const Triangle &b,
                                    const SharedFeature &shared,
                                    const std::vector<Vec3> &positions) {
  if(!triangle_intersection(a,b))return false;
  if(shared.count>=3)return true;
  const Vec3 n1=cross(sub(a.v[1],a.v[0]),sub(a.v[2],a.v[0]));
  const Vec3 n2=cross(sub(b.v[1],b.v[0]),sub(b.v[2],b.v[0]));
  const double len1=std::hypot(n1.x,n1.y,n1.z);
  const double len2=std::hypot(n2.x,n2.y,n2.z);
  if(!(len1>1e-14&&len2>1e-14))return false;
  const Vec3 unit{n1.x/len1,n1.y/len1,n1.z/len1};
  const double parallel=std::abs(dot(n1,n2)/(len1*len2));
  bool coplanar=parallel>1.0-1e-10;
  for(Vec3 p:b.v)if(std::abs(dot(sub(p,a.v[0]),unit))>1e-10)
    coplanar=false;
  if(coplanar) {
    int drop=0;
    if(std::abs(n1.y)>std::abs(n1.x))drop=1;
    if(std::abs(n1.z)>std::abs(axis(n1,drop)))drop=2;
    const auto area=[&](const Triangle &t){
      return 0.5*std::abs(orient(project(t.v[0],drop),
                                 project(t.v[1],drop),
                                 project(t.v[2],drop)));
    };
    const double threshold=std::max(1e-28,1e-12*std::min(area(a),area(b)));
    return coplanar_overlap_area(a,b,n1)>threshold;
  }
  return edge_crosses_triangle_away(a,b,shared,positions)||
         edge_crosses_triangle_away(b,a,shared,positions);
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
  // Scratch buffers retained across candidate checks. A rejected geometric
  // update cannot corrupt the last valid BVH.
  std::vector<Vec3> staged_positions_;
  std::vector<Aabb> staged_boxes_;
  std::vector<Vec3> centers_;
  std::vector<Id> indices_;
  std::vector<Node> nodes_;
  std::vector<Id> parent_node_, quad_leaf_;
  std::vector<std::uint32_t> node_epoch_, vertex_epoch_;
  std::vector<Vec3> local_candidate_positions_;
  std::vector<Id> affected_nodes_;
  std::uint32_t update_epoch_ = 0;
  Vec3 world_origin_{0,0,0};
  double world_scale_ = 1.0;
  std::uint64_t local_node_updates_ = 0;
  QuadIntersectionReport report_;
  std::size_t limit_;

  Id build(Id first,Id end,Id parent=std::numeric_limits<Id>::max()){
    const Id self=static_cast<Id>(nodes_.size());
    nodes_.push_back({});
    parent_node_.push_back(parent);
    Aabb box,centroid;
    for(Id i=first;i<end;++i){
      const Id q=indices_[i];box.extend(boxes_[q]);centroid.extend(centers_[q]);
    }
    nodes_[self].box=box;nodes_[self].first=first;nodes_[self].count=end-first;
    if(end-first<=8){
      nodes_[self].leaf=true;
      for(Id k=first;k<end;++k)quad_leaf_[indices_[k]]=self;
      return self;
    }
    const Vec3 extent=sub(centroid.hi,centroid.lo);
    int dim=extent.y>extent.x?1:0;
    if(extent.z>axis(extent,dim))dim=2;
    const Id mid=first+(end-first)/2;
    std::nth_element(indices_.begin()+first,indices_.begin()+mid,indices_.begin()+end,
      [&](Id a,Id b){
        const double av=axis(centers_[a],dim),bv=axis(centers_[b],dim);
        return av<bv||(av==bv&&a<b);
      });
    const Id left=build(first,mid,self),right=build(mid,end,self);
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
    if(!boxes_[a].overlaps(boxes_[b]))return true;
    const SharedFeature shared=shared_vertices(qa,qb);
    if(shared.count) {
      ++report_.adjacent_pairs_checked;
      if(bounds_touch_only_shared_feature(boxes_[a],boxes_[b],
                                          shared,positions_)) {
        ++report_.adjacent_pairs_ignored;
        ++report_.shared_feature_fastpath_skips;
        return true;
      }
    }
    ++report_.candidate_quad_pairs;
    for(int ia=0;ia<2;++ia){
      const Triangle ta=tri(a,ia);
      for(int ib=0;ib<2;++ib){
        ++report_.triangle_pair_tests;
        const Triangle tb=tri(b,ib);
        const bool colliding=shared.count?
            illegal_adjacent_intersection(ta,tb,shared,positions_):
            triangle_intersection(ta,tb);
        if(colliding){
          if(report_.pairs.size()>=limit_){report_.truncated=true;return false;}
          report_.pairs.push_back({a,b});
          if(report_.pairs.size()==limit_) {report_.truncated=true;return false;}
          return true;
        }
      }
    }
    if(shared.count)++report_.adjacent_pairs_ignored;
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
    world_origin_=lo;
    world_scale_=scale;
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
    const std::size_t estimated_nodes=mesh.quads.size()/4+16;
    nodes_.reserve(estimated_nodes);
    parent_node_.reserve(estimated_nodes);
    quad_leaf_.resize(mesh.quads.size());
    build(0,static_cast<Id>(mesh.quads.size()));
    node_epoch_.assign(nodes_.size(),0);
    vertex_epoch_.assign(positions_.size(),0);
    local_candidate_positions_.resize(positions_.size());
    affected_nodes_.reserve(nodes_.size());
  }
  void refit(const std::vector<Vec3> &vertices) {
    if(vertices.size()!=positions_.size()) {
      throw std::invalid_argument("BVH refit cannot change the vertex count");
    }
    Vec3 lo=vertices.front(),hi=lo;
    for(const Vec3 p:vertices) {
      if(!finite(p))throw std::invalid_argument("BVH refit contains nonfinite vertices");
      lo.x=std::min(lo.x,p.x);lo.y=std::min(lo.y,p.y);lo.z=std::min(lo.z,p.z);
      hi.x=std::max(hi.x,p.x);hi.y=std::max(hi.y,p.y);hi.z=std::max(hi.z,p.z);
    }
    const double scale=std::max({hi.x-lo.x,hi.y-lo.y,hi.z-lo.z});
    if(!(scale>0)||!std::isfinite(scale))
      throw std::invalid_argument("BVH refit has invalid geometric scale");
    staged_positions_.resize(vertices.size());
    for(std::size_t k=0;k<vertices.size();++k) {
      const Vec3 p=vertices[k];
      staged_positions_[k]={(p.x-lo.x)/scale,(p.y-lo.y)/scale,(p.z-lo.z)/scale};
    }
    staged_boxes_.resize(mesh_.quads.size());
    for(Id i=0;i<mesh_.quads.size();++i) {
      const auto &q=mesh_.quads[i];
      const Vec3 a=staged_positions_[q[0]],b=staged_positions_[q[1]];
      const Vec3 c=staged_positions_[q[2]],d=staged_positions_[q[3]];
      const Vec3 n1=cross(sub(b,a),sub(c,a));
      const Vec3 n2=cross(sub(c,a),sub(d,a));
      if(!(std::hypot(n1.x,n1.y,n1.z)>1e-14 &&
           std::hypot(n2.x,n2.y,n2.z)>1e-14) || dot(n1,n2)<=0.0) {
        throw std::invalid_argument("BVH refit would create a degenerate or folded quad");
      }
      Aabb bounds;
      bounds.extend(a);bounds.extend(b);bounds.extend(c);bounds.extend(d);
      staged_boxes_[i]=bounds;
    }
    // All validation completed: commit the new geometry, keeping the former
    // vector capacities available for the next trial. Topology and node order
    // remain immutable; only the stored geometry and BVH node boxes change.
    positions_.swap(staged_positions_);
    boxes_.swap(staged_boxes_);
    world_origin_=lo;
    world_scale_=scale;
    // Nodes are allocated parent-first, so reverse order is bottom-up.
    for(std::size_t idx=nodes_.size();idx>0;) {
      Node &node=nodes_[--idx];
      Aabb bounds;
      if(node.leaf) {
        for(Id j=node.first;j<node.first+node.count;++j)
          bounds.extend(boxes_[indices_[j]]);
      } else {
        bounds.extend(nodes_[node.left].box);
        bounds.extend(nodes_[node.right].box);
      }
      node.box=bounds;
    }
  }
  // Return false only when a candidate escapes the fixed normalization
  // frame; the caller must then use the full refit (which can recenter).
  // All validation completes BEFORE changing any live vertex or BVH box.
  bool refit_local(const std::vector<Vec3> &vertices,
                   const std::vector<Id> &dirty_vertices,
                   const std::vector<Id> &dirty_faces) {
    if(vertices.size()!=positions_.size())
      throw std::invalid_argument("local BVH refit cannot change vertex count");
    if(dirty_faces.empty())return true;
    if(update_epoch_==std::numeric_limits<std::uint32_t>::max()) {
      std::fill(vertex_epoch_.begin(),vertex_epoch_.end(),0);
      std::fill(node_epoch_.begin(),node_epoch_.end(),0);
      update_epoch_=0;
    }
    ++update_epoch_;
    for(Id id:dirty_vertices) {
      if(id>=positions_.size())
        throw std::invalid_argument("local BVH dirty vertex out of bounds");
      const Vec3 p=vertices[id];
      if(!finite(p))
        throw std::invalid_argument("local BVH dirty vertex is nonfinite");
      const Vec3 normalized{(p.x-world_origin_.x)/world_scale_,
                            (p.y-world_origin_.y)/world_scale_,
                            (p.z-world_origin_.z)/world_scale_};
      if(!finite(normalized) ||
         normalized.x < -0.25 || normalized.x > 1.25 ||
         normalized.y < -0.25 || normalized.y > 1.25 ||
         normalized.z < -0.25 || normalized.z > 1.25) {
        // Keep geometric scale reasonably close to unit size. Outside this
        // interval, recenter the *whole* model before any collision check.
        return false;
      }
      vertex_epoch_[id]=update_epoch_;
      local_candidate_positions_[id]=normalized;
    }
    const auto get=[&](Id id) -> Vec3 {
      return vertex_epoch_[id]==update_epoch_?
             local_candidate_positions_[id]:positions_[id];
    };
    // Staging is sparse: unchanged faces never recompute their positions,
    // normals or AABBs.
    if(staged_boxes_.size()!=boxes_.size())staged_boxes_.resize(boxes_.size());
    for(Id face:dirty_faces) {
      if(face>=mesh_.quads.size())
        throw std::invalid_argument("local BVH dirty face out of bounds");
      const auto &q=mesh_.quads[face];
      const Vec3 a=get(q[0]),b=get(q[1]),c=get(q[2]),d=get(q[3]);
      const Vec3 n1=cross(sub(b,a),sub(c,a));
      const Vec3 n2=cross(sub(c,a),sub(d,a));
      if(!(std::hypot(n1.x,n1.y,n1.z)>1e-14 &&
           std::hypot(n2.x,n2.y,n2.z)>1e-14) ||
         dot(n1,n2)<=0.0) {
        throw std::invalid_argument("local BVH refit would create a degenerate or folded quad");
      }
      Aabb bounds;
      bounds.extend(a);bounds.extend(b);bounds.extend(c);bounds.extend(d);
      staged_boxes_[face]=bounds;
    }
    // Commit: preallocate node scratch once; bottom-up tree recomputation
    // does not allocate or invoke any fallible operations.
    for(Id id:dirty_vertices)positions_[id]=local_candidate_positions_[id];
    for(Id face:dirty_faces)boxes_[face]=staged_boxes_[face];
    affected_nodes_.clear();
    for(Id face:dirty_faces) {
      Id node=quad_leaf_[face];
      while(node!=std::numeric_limits<Id>::max() &&
            node_epoch_[node]!=update_epoch_) {
        node_epoch_[node]=update_epoch_;
        affected_nodes_.push_back(node);
        node=parent_node_[node];
      }
    }
    std::sort(affected_nodes_.begin(),affected_nodes_.end(),
              [](Id a,Id b){return a>b;});
    for(Id id:affected_nodes_) {
      Node &node=nodes_[id];
      Aabb bounds;
      if(node.leaf) {
        for(Id k=node.first;k<node.first+node.count;++k)
          bounds.extend(boxes_[indices_[k]]);
      } else {
        bounds.extend(nodes_[node.left].box);
        bounds.extend(nodes_[node.right].box);
      }
      node.box=bounds;
    }
    local_node_updates_+=affected_nodes_.size();
    return true;
  }
  std::uint64_t local_node_updates() const {return local_node_updates_;}
  // Traverse only AABBs touched by dirty quads. If two dirty quads
  // overlap, the larger-index query skips the already-visited pair.
  bool query_against_node(Id moving,Id node_index,
                          const std::vector<std::uint8_t> &dirty) {
    const Node &node=nodes_[node_index];
    ++report_.bvh_pair_tests;
    if(!boxes_[moving].overlaps(node.box))return true;
    if(node.leaf) {
      for(Id i=node.first;i<node.first+node.count;++i) {
        const Id other=indices_[i];
        if(other==moving)continue;
        if(dirty[other] && other<moving)continue;
        if(!process_pair(moving,other))return false;
      }
      return true;
    }
    return query_against_node(moving,node.left,dirty) &&
           query_against_node(moving,node.right,dirty);
  }
  QuadIntersectionReport query_dirty(const std::vector<Id> &dirty_faces,
                                     std::size_t max_reported_pairs=16) {
    if(max_reported_pairs==0)
      throw std::invalid_argument("BVH collision report limit must be positive");
    limit_=max_reported_pairs;
    report_=QuadIntersectionReport{};
    if(dirty_faces.empty())return std::move(report_);
    std::vector<std::uint8_t> dirty(boxes_.size(),0);
    for(Id face:dirty_faces) {
      if(face>=boxes_.size())throw std::invalid_argument("dirty quad index is invalid");
      dirty[face]=1;
    }
    for(Id face=0;face<boxes_.size();++face)if(dirty[face]) {
      ++report_.dirty_face_queries;
      if(!query_against_node(face,0,dirty))break;
    }
    return std::move(report_);
  }
  QuadIntersectionReport query(std::size_t max_reported_pairs=16) {
    if(max_reported_pairs==0)
      throw std::invalid_argument("BVH collision report limit must be positive");
    limit_=max_reported_pairs;
    report_=QuadIntersectionReport{};
    traverse(0,0);
    return std::move(report_);
  }

};
} // namespace

struct PreparedQuadCollisionChecker::Impl {
  QuadPatchMesh mesh;
  QuadBvh bvh;
  std::vector<Vec3> accepted_positions;
  std::vector<std::vector<Id>> incident_quads;
  std::uint64_t refits=0,dirty_updates=0;
  std::uint64_t local_refits=0,full_refit_fallbacks=0;
  bool clean_baseline=false;
  explicit Impl(const QuadPatchMesh &input)
      : mesh(input), bvh(mesh,16), accepted_positions(input.vertices),
        incident_quads(input.vertices.size()) {
    for(Id face=0;face<input.quads.size();++face)
      for(Id vertex:input.quads[face])
        incident_quads[vertex].push_back(face);
  }
};
PreparedQuadCollisionChecker::PreparedQuadCollisionChecker(const QuadPatchMesh &patch)
    : impl_(new Impl(patch)) {}
PreparedQuadCollisionChecker::~PreparedQuadCollisionChecker()=default;
PreparedQuadCollisionChecker::PreparedQuadCollisionChecker(
    PreparedQuadCollisionChecker &&) noexcept=default;
PreparedQuadCollisionChecker &PreparedQuadCollisionChecker::operator=(
    PreparedQuadCollisionChecker &&) noexcept=default;
QuadIntersectionReport PreparedQuadCollisionChecker::check_initial(std::size_t limit) {
  if(!impl_)throw std::logic_error("collision checker was moved from");
  auto report=impl_->bvh.query(limit);
  impl_->clean_baseline=report.pairs.empty();
  return report;
}
QuadIntersectionReport PreparedQuadCollisionChecker::check_positions(
    const std::vector<Vec3> &positions,std::size_t limit) {
  if(!impl_)throw std::logic_error("collision checker was moved from");
  if(limit==0)throw std::invalid_argument("collision report limit must be positive");
  impl_->bvh.refit(positions);
  ++impl_->refits;
  impl_->accepted_positions=positions;
  auto report=impl_->bvh.query(limit);
  impl_->clean_baseline=report.pairs.empty();
  return report;
}
QuadIntersectionReport PreparedQuadCollisionChecker::check_positions_dirty(
    const std::vector<Vec3> &positions,
    const std::vector<Id> &dirty_vertices,std::size_t limit) {
  if(!impl_)throw std::logic_error("collision checker was moved from");
  if(limit==0)throw std::invalid_argument("collision report limit must be positive");
  if(!impl_->clean_baseline)
    throw std::logic_error("dirty check needs a verified collision-free baseline");
  if(positions.size()!=impl_->accepted_positions.size())
    throw std::invalid_argument("dirty update cannot change vertex count");
  std::vector<std::uint8_t> declared(positions.size(),0);
  for(Id v:dirty_vertices) {
    if(v>=positions.size())throw std::invalid_argument("dirty vertex index invalid");
    declared[v]=1;
  }
  std::vector<std::uint8_t> faces(impl_->mesh.quads.size(),0);
  std::vector<Id> changed_faces;
  std::vector<Id> changed_vertices;
  changed_vertices.reserve(dirty_vertices.size());
  for(std::size_t k=0;k<positions.size();++k) {
    const Vec3 p=positions[k],q=impl_->accepted_positions[k];
    if(!finite(p))throw std::invalid_argument("dirty update has nonfinite vertex");
    const bool changed=(p.x!=q.x || p.y!=q.y || p.z!=q.z);
    if(changed&&!declared[k])
      throw std::invalid_argument("dirty list omits an altered vertex");
    if(changed) {
      changed_vertices.push_back(static_cast<Id>(k));
      for(Id face:impl_->incident_quads[k]) {
        if(!faces[face]){faces[face]=1;changed_faces.push_back(face);}
      }
    }
  }
  if(changed_faces.empty()) {
    // Orphan vertices have no incident quad and cannot introduce a surface
    // collision; keep the accepted input snapshot synchronized regardless.
    impl_->accepted_positions=positions;
    return QuadIntersectionReport{};
  }
  // A clean prior state proves that a newly introduced intersection must
  // involve at least one changed face. The original normalization frame is
  // held fixed for local refits. Large motion falls back to a full recenter.
  const auto update_tree=[&](const std::vector<Vec3> &target) {
    if(impl_->bvh.refit_local(target,changed_vertices,changed_faces)) {
      ++impl_->local_refits;
    } else {
      impl_->bvh.refit(target);
      ++impl_->full_refit_fallbacks;
    }
    ++impl_->refits;
  };
  update_tree(positions);
  ++impl_->dirty_updates;
  try {
    auto report=impl_->bvh.query_dirty(changed_faces,limit);
    if(report.pairs.empty()) {
      impl_->accepted_positions=positions;
    } else {
      // Failed candidate: refit back to the last verified clean positions
      // before another backtracking attempt, without changing topology.
      update_tree(impl_->accepted_positions);
    }
    return report;
  } catch(...) {
    update_tree(impl_->accepted_positions);
    throw;
  }
}
std::uint64_t PreparedQuadCollisionChecker::refit_count() const {
  if(!impl_)throw std::logic_error("collision checker was moved from");
  return impl_->refits;
}
std::uint64_t PreparedQuadCollisionChecker::dirty_update_count() const {
  if(!impl_)throw std::logic_error("collision checker was moved from");
  return impl_->dirty_updates;
}
std::uint64_t PreparedQuadCollisionChecker::local_refit_count() const {
  if(!impl_)throw std::logic_error("collision checker was moved from");
  return impl_->local_refits;
}
std::uint64_t PreparedQuadCollisionChecker::full_refit_fallback_count() const {
  if(!impl_)throw std::logic_error("collision checker was moved from");
  return impl_->full_refit_fallbacks;
}
std::uint64_t PreparedQuadCollisionChecker::local_bvh_node_updates() const {
  if(!impl_)throw std::logic_error("collision checker was moved from");
  return impl_->bvh.local_node_updates();
}

QuadIntersectionReport find_quad_self_intersections(
    const QuadPatchMesh &mesh,std::size_t max_reported_pairs) {
  QuadBvh bvh(mesh,max_reported_pairs);
  return bvh.query(max_reported_pairs);
}

} // namespace guidedquad
