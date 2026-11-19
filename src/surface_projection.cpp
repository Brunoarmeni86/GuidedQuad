// SPDX-License-Identifier: MIT
#include "guidedquad/surface_projection.hpp"
#include "guidedquad/quad_intersections.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <stdexcept>
#include <vector>

namespace guidedquad {
namespace {
using Id = std::uint32_t;
Vec3 add(Vec3 a,Vec3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3 sub(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3 mul(Vec3 a,double t){return {a.x*t,a.y*t,a.z*t};}
double dot(Vec3 a,Vec3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec3 cross(Vec3 a,Vec3 b){
  return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
double norm(Vec3 a){return std::hypot(a.x,a.y,a.z);}
bool finite(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
double sqdist(Vec3 a,Vec3 b){const auto d=sub(a,b);return dot(d,d);}
double component(Vec3 p,int axis){return axis==0?p.x:(axis==1?p.y:p.z);}

struct Box {
  Vec3 lo{std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity()};
  Vec3 hi{-std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity()};
  void extend(Vec3 v) {
    lo.x=std::min(lo.x,v.x);lo.y=std::min(lo.y,v.y);lo.z=std::min(lo.z,v.z);
    hi.x=std::max(hi.x,v.x);hi.y=std::max(hi.y,v.y);hi.z=std::max(hi.z,v.z);
  }
  void extend(const Box &b){extend(b.lo);extend(b.hi);}
  double dist2(Vec3 p) const {
    const auto delta=[](double x,double lo,double hi){
      return x<lo?lo-x:(x>hi?x-hi:0.0);
    };
    const double x=delta(p.x,lo.x,hi.x),y=delta(p.y,lo.y,hi.y),z=delta(p.z,lo.z,hi.z);
    return x*x+y*y+z*z;
  }
};

// Closest-point regions from triangle barycentric Voronoi decomposition.
// All source triangles are validated as nondegenerate before querying.
Vec3 closest_triangle(Vec3 p,Vec3 a,Vec3 b,Vec3 c) {
  const Vec3 ab=sub(b,a),ac=sub(c,a),ap=sub(p,a);
  const double d1=dot(ab,ap),d2=dot(ac,ap);
  if(d1<=0&&d2<=0)return a;
  const Vec3 bp=sub(p,b);
  const double d3=dot(ab,bp),d4=dot(ac,bp);
  if(d3>=0&&d4<=d3)return b;
  const double vc=d1*d4-d3*d2;
  if(vc<=0&&d1>=0&&d3<=0){
    const double v=d1/(d1-d3);
    return add(a,mul(ab,v));
  }
  const Vec3 cp=sub(p,c);
  const double d5=dot(ab,cp),d6=dot(ac,cp);
  if(d6>=0&&d5<=d6)return c;
  const double vb=d5*d2-d1*d6;
  if(vb<=0&&d2>=0&&d6<=0){
    const double w=d2/(d2-d6);
    return add(a,mul(ac,w));
  }
  const double va=d3*d6-d5*d4;
  if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0){
    const double w=(d4-d3)/((d4-d3)+(d5-d6));
    return add(b,mul(sub(c,b),w));
  }
  const double inv=1.0/(va+vb+vc);
  const double v=vb*inv,w=vc*inv;
  return add(a,add(mul(ab,v),mul(ac,w)));
}

class SurfaceBvh {
  struct Node {
    Box box;
    Id begin=0,count=0;
    Id left=0,right=0;
    bool leaf=false;
    Id component=std::numeric_limits<Id>::max(); // mixed if unset
  };
  const TriangleMesh &mesh_;
  std::vector<Id> indices_;
  std::vector<Box> triangle_boxes_;
  std::vector<Vec3> centroids_;
  std::vector<Node> nodes_;
  std::vector<Vec3> face_normals_;
  std::vector<Id> face_components_;
  Id component_count_ = 0;
  std::uint32_t skipped_ = 0;

  // Components are defined by triangle EDGE connectivity, not by similar
  // coordinates or by touching only at one vertex. Unwelded scan shells stay
  // separate even if geometrically coincident and normals agree.
  void build_components() {
    std::vector<Id> parent(mesh_.triangles.size());
    std::iota(parent.begin(),parent.end(),Id{0});
    const auto root=[&](Id id) {
      while(parent[id]!=id) {
        parent[id]=parent[parent[id]];
        id=parent[id];
      }
      return id;
    };
    const auto join=[&](Id a,Id b) {
      a=root(a); b=root(b);
      if(a!=b)parent[std::max(a,b)]=std::min(a,b);
    };
    std::unordered_map<std::uint64_t,Id> first_edge_face;
    first_edge_face.reserve(indices_.size()*3);
    for(Id face:indices_) {
      const auto &t=mesh_.triangles[face];
      for(int k=0;k<3;++k) {
        const Id a=std::min(t[k],t[(k+1)%3]);
        const Id b=std::max(t[k],t[(k+1)%3]);
        const std::uint64_t key=(std::uint64_t(a)<<32)|b;
        const auto insertion=first_edge_face.emplace(key,face);
        if(!insertion.second)join(face,insertion.first->second);
      }
    }
    std::unordered_map<Id,Id> root_to_component;
    for(Id face:indices_) {
      const Id r=root(face);
      const auto [it,inserted]=root_to_component.emplace(r,component_count_);
      if(inserted)++component_count_;
      face_components_[face]=it->second;
    }
  }

  Id build(Id first,Id end) {
    const Id node_index=static_cast<Id>(nodes_.size());
    nodes_.emplace_back();
    Box bounds,centroid_bounds;
    for(Id i=first;i<end;++i){
      const Id face=indices_[i];
      bounds.extend(triangle_boxes_[face]);
      centroid_bounds.extend(centroids_[face]);
    }
    nodes_[node_index].box=bounds;
    Id homogeneous=face_components_[indices_[first]];
    for(Id i=first+1;i<end;++i)
      if(face_components_[indices_[i]]!=homogeneous){
        homogeneous=std::numeric_limits<Id>::max();break;
      }
    nodes_[node_index].component=homogeneous;
    if(end-first<=8) {
      nodes_[node_index].begin=first;
      nodes_[node_index].count=end-first;
      nodes_[node_index].leaf=true;
      return node_index;
    }
    const Vec3 e=sub(centroid_bounds.hi,centroid_bounds.lo);
    int axis=e.y>e.x?1:0;
    if(e.z>component(e,axis))axis=2;
    const Id mid=first+(end-first)/2;
    std::nth_element(indices_.begin()+first,indices_.begin()+mid,indices_.begin()+end,
                     [&](Id a,Id b){
                       const double ca=component(centroids_[a],axis);
                       const double cb=component(centroids_[b],axis);
                       return ca==cb?a<b:ca<cb;
                     });
    const Id left=build(first,mid);
    const Id right=build(mid,end);
    nodes_[node_index].left=left;
    nodes_[node_index].right=right;
    return node_index;
  }

  void find(Id n,Vec3 p,double &best,Vec3 &point,
            Vec3 reference_normal,double min_dot,Id desired_component,
            SurfaceProjectionStats &stats) const {
    const Node &node=nodes_[n];
    ++stats.box_tests;
    if(desired_component!=std::numeric_limits<Id>::max()&&
       node.component!=std::numeric_limits<Id>::max()&&
       node.component!=desired_component)return;
    if(node.box.dist2(p)>best)return;
    if(node.leaf) {
      for(Id i=node.begin;i<node.begin+node.count;++i){
        const Id face=indices_[i];
        if(desired_component!=std::numeric_limits<Id>::max() &&
           face_components_[face]!=desired_component){
          ++stats.component_rejected_triangles;
          continue;
        }
        if(min_dot > -1.0 && dot(face_normals_[face],reference_normal)<min_dot) {
          ++stats.normal_rejected_triangles;
          continue;
        }
        const auto &tri=mesh_.triangles[face];
        const Vec3 q=closest_triangle(p,mesh_.vertices[tri[0]],
                                      mesh_.vertices[tri[1]],mesh_.vertices[tri[2]]);
        ++stats.triangle_tests;
        const double d2=sqdist(p,q);
        if(d2<best){best=d2;point=q;}
      }
      return;
    }
    const double dl=nodes_[node.left].box.dist2(p);
    const double dr=nodes_[node.right].box.dist2(p);
    if(dl<=dr) {
      if(dl<=best)find(node.left,p,best,point,reference_normal,min_dot,desired_component,stats);
      if(dr<=best)find(node.right,p,best,point,reference_normal,min_dot,desired_component,stats);
    } else {
      if(dr<=best)find(node.right,p,best,point,reference_normal,min_dot,desired_component,stats);
      if(dl<=best)find(node.left,p,best,point,reference_normal,min_dot,desired_component,stats);
    }
  }

 public:
  explicit SurfaceBvh(const TriangleMesh &surface):mesh_(surface) {
    if(surface.vertices.size()>std::numeric_limits<Id>::max()||
       surface.triangles.size()>std::numeric_limits<Id>::max()||
       surface.triangles.size()>10000000ULL) {
      throw std::invalid_argument("surface exceeds projector triangle/index limit");
    }
    for(Vec3 p:surface.vertices)if(!finite(p))
      throw std::invalid_argument("surface has non-finite source vertices");
    triangle_boxes_.resize(surface.triangles.size());
    centroids_.resize(surface.triangles.size());
    face_normals_.resize(surface.triangles.size());
    face_components_.resize(surface.triangles.size(),std::numeric_limits<Id>::max());
    for(Id i=0;i<surface.triangles.size();++i) {
      const auto &t=surface.triangles[i];
      if(t[0]>=surface.vertices.size()||t[1]>=surface.vertices.size()||
         t[2]>=surface.vertices.size()) {
        throw std::invalid_argument("reference surface has invalid triangle indices");
      }
      const Vec3 a=surface.vertices[t[0]],b=surface.vertices[t[1]],c=surface.vertices[t[2]];
      const Vec3 n=cross(sub(b,a),sub(c,a));
      if(t[0]==t[1]||t[1]==t[2]||t[2]==t[0]||
         !(norm(n)>0.0) || !finite(n)){
        ++skipped_;
        continue;
      }
      indices_.push_back(i);
      face_normals_[i]=mul(n,1.0/norm(n));
      Box bounds;bounds.extend(a);bounds.extend(b);bounds.extend(c);
      triangle_boxes_[i]=bounds;
      centroids_[i]=mul(add(add(a,b),c),1.0/3.0);
    }
    if(indices_.empty())throw std::invalid_argument("surface has no nondegenerate triangle");
    build_components();
    nodes_.reserve(indices_.size()/4+16);
    build(0,static_cast<Id>(indices_.size()));
  }
  std::uint32_t skipped() const { return skipped_; }
  Id connected_components() const { return component_count_; }
  std::uint32_t active_count() const { return static_cast<std::uint32_t>(indices_.size()); }
  Vec3 nearest(Vec3 p,Vec3 reference_normal,double min_dot,
               Id desired_component,SurfaceProjectionStats &stats) const {
    if(!finite(p))throw std::invalid_argument("nearest query has invalid point");
    ++stats.closest_queries;
    double best=std::numeric_limits<double>::infinity();
    Vec3 closest{0,0,0};
    find(0,p,best,closest,reference_normal,min_dot,desired_component,stats);
    if(!std::isfinite(best))
      throw std::runtime_error("no compatible triangle or numeric overflow in nearest query");
    return closest;
  }
};

std::size_t index(Id i,Id j,Id nu) {
  return static_cast<std::size_t>(j)*(static_cast<std::size_t>(nu)+1)+i;
}

struct QualityReference {
  Vec3 normal;
  double area_a=0,area_b=0;
};

std::vector<QualityReference> validate_input(const QuadPatchMesh &mesh,Id nu,Id nv) {
  const std::uint64_t expected_vertices=(std::uint64_t(nu)+1)*(std::uint64_t(nv)+1);
  const std::uint64_t expected_quads=std::uint64_t(nu)*nv;
  if(nu==0||nv==0||expected_vertices>2000000ULL||
     mesh.vertices.size()!=expected_vertices||mesh.quads.size()!=expected_quads){
    throw std::invalid_argument("projection expects a complete regular quad patch");
  }
  for(Vec3 p:mesh.vertices)if(!finite(p))
    throw std::invalid_argument("quad patch has non-finite coordinates");
  std::vector<QualityReference> refs;
  refs.reserve(mesh.quads.size());
  for(Id j=0;j<nv;++j)for(Id i=0;i<nu;++i){
    const auto &q=mesh.quads[static_cast<std::size_t>(j)*nu+i];
    const std::array<Id,4> expected={
       static_cast<Id>(index(i,j,nu)),static_cast<Id>(index(i+1,j,nu)),
       static_cast<Id>(index(i+1,j+1,nu)),static_cast<Id>(index(i,j+1,nu))};
    if(q!=expected)throw std::invalid_argument("quad patch has non-grid topology or ordering");
    const Vec3 a=mesh.vertices[q[0]],b=mesh.vertices[q[1]],
               c=mesh.vertices[q[2]],d=mesh.vertices[q[3]];
    const Vec3 n1=cross(sub(b,a),sub(c,a));
    const Vec3 n2=cross(sub(c,a),sub(d,a));
    const double a1=norm(n1),a2=norm(n2);
    if(!(a1>0.0&&a2>0.0)||dot(n1,n2)<=0.0){
      throw std::invalid_argument("original quad patch is collapsed or folded");
    }
    const Vec3 n=add(n1,n2);
    const double length=norm(n);
    if(!(length>0.0)||!std::isfinite(length))
      throw std::invalid_argument("invalid reference quad normal");
    refs.push_back({mul(n,1.0/length),a1,a2});
  }
  return refs;
}

bool valid_candidate(const QuadPatchMesh &patch,
                     const std::vector<QualityReference> &original,
                     const SurfaceProjectionOptions &options) {
  for(std::size_t f=0;f<patch.quads.size();++f){
    const auto &q=patch.quads[f];
    const Vec3 p[4]={patch.vertices[q[0]],patch.vertices[q[1]],
                     patch.vertices[q[2]],patch.vertices[q[3]]};
    for(Vec3 v:p)if(!finite(v))return false;
    const Vec3 n1=cross(sub(p[1],p[0]),sub(p[2],p[0]));
    const Vec3 n2=cross(sub(p[2],p[0]),sub(p[3],p[0]));
    const double a1=norm(n1),a2=norm(n2);
    if(!(a1>=original[f].area_a*options.min_area_ratio&&
         a2>=original[f].area_b*options.min_area_ratio))return false;
    if(dot(n1,n2)<=0)return false;
    if(dot(n1,original[f].normal)<=options.min_normal_dot*a1||
       dot(n2,original[f].normal)<=options.min_normal_dot*a2)return false;
    for(int k=0;k<4;++k){
      const Vec3 outgoing=sub(p[(k+1)%4],p[k]);
      const Vec3 incoming=sub(p[(k+3)%4],p[k]);
      const double le=norm(outgoing),li=norm(incoming);
      if(!(le>0.0&&li>0.0))return false;
      if(dot(cross(outgoing,incoming),original[f].normal) <=
         options.min_corner_sine*le*li)return false;
    }
  }
  return true;
}
Vec3 original_vertex_normal(const QuadPatchMesh &patch, Id i, Id j, Id nu) {
  const Vec3 du=sub(patch.vertices[index(i+1,j,nu)],
                    patch.vertices[index(i-1,j,nu)]);
  const Vec3 dv=sub(patch.vertices[index(i,j+1,nu)],
                    patch.vertices[index(i,j-1,nu)]);
  const Vec3 n=cross(du,dv);
  const double l=norm(n);
  if(!(l>0.0)||!std::isfinite(l))
    throw std::invalid_argument("cannot define an original patch vertex normal");
  return mul(n,1.0/l);
}
} // namespace

struct PreparedSurfaceProjector::Impl {
  TriangleMesh surface;
  SurfaceBvh bvh;
  explicit Impl(const TriangleMesh &reference):surface(reference),bvh(surface) {}
};

PreparedSurfaceProjector::PreparedSurfaceProjector(const TriangleMesh &surface)
  :impl_(new Impl(surface)) {}
PreparedSurfaceProjector::~PreparedSurfaceProjector() = default;
PreparedSurfaceProjector::PreparedSurfaceProjector(PreparedSurfaceProjector &&) noexcept = default;
PreparedSurfaceProjector &PreparedSurfaceProjector::operator=(PreparedSurfaceProjector &&) noexcept = default;
std::uint32_t PreparedSurfaceProjector::active_triangle_count() const {
  if(!impl_) throw std::logic_error("projector was moved from");
  return impl_->bvh.active_count();
}
std::uint32_t PreparedSurfaceProjector::connected_component_count() const {
  if(!impl_) throw std::logic_error("projector was moved from");
  return impl_->bvh.connected_components();
}
std::uint32_t PreparedSurfaceProjector::skipped_degenerate_triangles() const {
  if(!impl_) throw std::logic_error("projector was moved from");
  return impl_->bvh.skipped();
}

SurfaceProjectionResult PreparedSurfaceProjector::project(
    const QuadPatchMesh &patch,
    Id nu,Id nv,const SurfaceProjectionOptions &opts) const {
  if(!impl_) throw std::logic_error("projector was moved from");
  if(!std::isfinite(opts.strength)||opts.strength<=0.0||opts.strength>1.0||
     !std::isfinite(opts.min_area_ratio)||opts.min_area_ratio<=0.0||
     opts.min_area_ratio>1.0||!std::isfinite(opts.min_corner_sine)||
     opts.min_corner_sine<=0.0||opts.min_corner_sine>=1.0||
     !std::isfinite(opts.min_normal_dot)||opts.min_normal_dot<0.0||
     opts.min_normal_dot>=1.0||opts.iterations>64||
     opts.backtracking_steps==0||opts.backtracking_steps>20||
     !(opts.max_projection_distance>0.0)||
     !std::isfinite(opts.min_reference_normal_dot)||
     opts.min_reference_normal_dot < -1.0||
     opts.min_reference_normal_dot > 1.0){
    throw std::invalid_argument("invalid guarded surface projection options");
  }
  SurfaceProjectionResult output;
  output.mesh=patch;
  const auto refs=validate_input(patch,nu,nv);
  // Keep one output-quad tree for every candidate and backtracking attempt;
  // only per-face bounds change, the patch quad indices remain fixed.
  std::unique_ptr<PreparedQuadCollisionChecker> collision;
  if(opts.reject_self_intersections) {
    collision.reset(new PreparedQuadCollisionChecker(patch));
    ++output.stats.collision_bvh_builds;
    ++output.stats.collision_validation_passes;
    if(!collision->check_initial(1).pairs.empty()) {
      throw std::invalid_argument("input patch contains quad self-intersections");
    }
  }
  const std::uint64_t interior=nu>1&&nv>1?std::uint64_t(nu-1)*(nv-1):0;
  if(interior*opts.iterations>20000000ULL)
    throw std::invalid_argument("projection iterations exceed CPU budget");

  const SurfaceBvh &bvh=impl_->bvh;
  if(opts.source_component_id!=std::numeric_limits<Id>::max()&&
     opts.source_component_id>=bvh.connected_components()) {
    throw std::invalid_argument("requested source component does not exist");
  }
  output.stats.skipped_degenerate_triangles=bvh.skipped();
  if(!interior)return output;
  std::vector<Vec3> original_normals;
  if(opts.min_reference_normal_dot > -1.0) {
    original_normals.resize(patch.vertices.size());
    for(Id j=1;j<nv;++j)for(Id i=1;i<nu;++i)
      original_normals[index(i,j,nu)]=original_vertex_normal(patch,i,j,nu);
  }
  const Vec3 no_normal{0,0,0};
  std::vector<Vec3> targets(patch.vertices.size());
  QuadPatchMesh trial=patch;

  // Collect the initial closest points once. Previously the same nearest
  // queries ran twice: first to calculate initial RMS, immediately again to
  // calculate the first iteration's unchanged targets.
  double initial_sum=0.0;
  for(Id j=1;j<nv;++j)for(Id i=1;i<nu;++i){
    const std::size_t idx=index(i,j,nu);
    targets[idx]=bvh.nearest(patch.vertices[idx],
        original_normals.empty()?no_normal:original_normals[idx],
        opts.min_reference_normal_dot,opts.source_component_id,output.stats);
    initial_sum+=sqdist(patch.vertices[idx],targets[idx]);
  }
  output.stats.initial_rms_distance=std::sqrt(initial_sum/static_cast<double>(interior));
  if(opts.iterations==0) {
    output.stats.final_rms_distance=output.stats.initial_rms_distance;
    return output;
  }

  for(Id iter=0;iter<opts.iterations;++iter) {
    double max_residual=0.0;
    for(Id j=1;j<nv;++j)for(Id i=1;i<nu;++i){
      const std::size_t idx=index(i,j,nu);
      // First iteration reuses the initial RMS queries. Re-query only after
      // geometry actually changes and the previous closest targets become stale.
      if(iter>0)targets[idx]=bvh.nearest(output.mesh.vertices[idx],
          original_normals.empty()?no_normal:original_normals[idx],
          opts.min_reference_normal_dot,opts.source_component_id,output.stats);
      if(norm(sub(targets[idx],patch.vertices[idx]))>opts.max_projection_distance){
        // The radius is measured against the ORIGINAL grid, not the last
        // accepted position; consecutive iterations cannot drift across it.
        targets[idx]=output.mesh.vertices[idx];
        ++output.stats.distance_rejected_targets;
      }
      max_residual=std::max(max_residual,norm(sub(targets[idx],output.mesh.vertices[idx])));
    }
    if(max_residual<=1e-14)break;
    bool accepted=false;
    double alpha=opts.strength;
    for(Id attempt=0;attempt<opts.backtracking_steps;++attempt) {
      for(Id j=1;j<nv;++j)for(Id i=1;i<nu;++i){
        const std::size_t idx=index(i,j,nu);
        trial.vertices[idx]=add(mul(output.mesh.vertices[idx],1.0-alpha),
                                 mul(targets[idx],alpha));
      }
      const bool locally_valid=valid_candidate(trial,refs,opts);
      bool globally_valid=true;
      if(locally_valid && opts.reject_self_intersections) {
        ++output.stats.collision_validation_passes;
        // Full triangle-local validity was checked immediately above; the
        // updated BVH checks non-local intersections without re-sorting quads.
        globally_valid=collision->check_positions(trial.vertices,1).pairs.empty();
        ++output.stats.collision_bvh_refits;
        if(!globally_valid) ++output.stats.collision_rejected_steps;
      }
      if(locally_valid && globally_valid){
        output.mesh.vertices.swap(trial.vertices);
        ++output.stats.accepted_iterations;
        accepted=true;
        break;
      }
      ++output.stats.rejected_steps;
      alpha*=0.5;
    }
    if(!accepted)break;
  }

  // No accepted iteration means the mesh is unchanged: the initial nearest
  // distances are still valid. Avoid an unnecessary full BVH query pass.
  if(output.stats.accepted_iterations==0) {
    output.stats.final_rms_distance=output.stats.initial_rms_distance;
    return output;
  }

  double final_sum=0.0;
  for(Id j=1;j<nv;++j)for(Id i=1;i<nu;++i) {
    const std::size_t idx=index(i,j,nu);
    const Vec3 q=bvh.nearest(output.mesh.vertices[idx],
        original_normals.empty()?no_normal:original_normals[idx],
        opts.min_reference_normal_dot,opts.source_component_id,output.stats);
    final_sum+=sqdist(output.mesh.vertices[idx],q);
    const double moved=norm(sub(output.mesh.vertices[idx],patch.vertices[idx]));
    if(moved>1e-14)++output.stats.moved_vertices;
    output.stats.max_vertex_displacement=std::max(output.stats.max_vertex_displacement,moved);
  }
  output.stats.final_rms_distance=std::sqrt(final_sum/static_cast<double>(interior));
  return output;
}

SurfaceProjectionResult project_quad_patch_to_surface(
    const TriangleMesh &surface,const QuadPatchMesh &patch,
    Id nu,Id nv,const SurfaceProjectionOptions &opts) {
  const PreparedSurfaceProjector prepared(surface);
  return prepared.project(patch,nu,nv,opts);
}

} // namespace guidedquad
