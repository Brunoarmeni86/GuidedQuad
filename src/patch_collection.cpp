// SPDX-License-Identifier: MIT
#include "guidedquad/quad_intersections.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace guidedquad {
namespace {
using Id = std::uint32_t;
double distance(Vec3 a,Vec3 b) {
  return std::hypot(a.x-b.x,a.y-b.y,a.z-b.z);
}
struct UnionFind {
  std::vector<Id> parent;
  explicit UnionFind(std::size_t count) : parent(count) {
    std::iota(parent.begin(),parent.end(),Id{0});
  }
  Id root(Id i) {
    while(parent[i]!=i) {
      parent[i]=parent[parent[i]];
      i=parent[i];
    }
    return i;
  }
  void unite(Id a,Id b) {
    a=root(a);b=root(b);
    if(a!=b)parent[std::max(a,b)]=std::min(a,b);
  }
};
}

PatchCollectionIntersectionReport find_patch_collection_intersections(
    const std::vector<QuadPatchMesh> &patches,
    const std::vector<SharedPatchVertex> &seam_vertices,
    std::size_t max_reported_pairs) {
  if(patches.empty() || patches.size()>std::numeric_limits<Id>::max())
    throw std::invalid_argument("patch collection must contain at least one patch");
  if(max_reported_pairs==0)
    throw std::invalid_argument("collision report limit must be positive");

  std::vector<Id> vertex_offsets;
  vertex_offsets.reserve(patches.size()+1);
  vertex_offsets.push_back(0);
  std::uint64_t total_vertices=0,total_quads=0;
  Vec3 lo{},hi{};
  bool has_point=false;
  for(const auto &patch:patches) {
    if(patch.vertices.empty() || patch.quads.empty())
      throw std::invalid_argument("patch collection has an empty patch");
    if(total_vertices+patch.vertices.size()>std::numeric_limits<Id>::max() ||
       total_quads+patch.quads.size()>2000000ULL)
      throw std::invalid_argument("patch collection exceeds collision engine index limit");
    for(const Vec3 &p:patch.vertices) {
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))
        throw std::invalid_argument("patch collection contains non-finite vertex");
      if(!has_point){lo=hi=p;has_point=true;}
      lo.x=std::min(lo.x,p.x);lo.y=std::min(lo.y,p.y);lo.z=std::min(lo.z,p.z);
      hi.x=std::max(hi.x,p.x);hi.y=std::max(hi.y,p.y);hi.z=std::max(hi.z,p.z);
    }
    for(const auto &q:patch.quads)
      for(Id v:q)if(v>=patch.vertices.size())
        throw std::invalid_argument("patch collection has invalid face index");
    total_vertices+=patch.vertices.size();
    total_quads+=patch.quads.size();
    vertex_offsets.push_back(static_cast<Id>(total_vertices));
  }
  const double scale=std::max({hi.x-lo.x,hi.y-lo.y,hi.z-lo.z});
  if(!(scale>0.0)||!std::isfinite(scale))
    throw std::invalid_argument("invalid patch collection extent");

  UnionFind dsu(static_cast<std::size_t>(total_vertices));
  for(const auto &seam:seam_vertices) {
    if(seam.patch_a>=patches.size()||seam.patch_b>=patches.size()||
       seam.patch_a==seam.patch_b ||
       seam.vertex_a>=patches[seam.patch_a].vertices.size()||
       seam.vertex_b>=patches[seam.patch_b].vertices.size()) {
      throw std::invalid_argument("invalid cross-patch seam vertex pair");
    }
    const Vec3 a=patches[seam.patch_a].vertices[seam.vertex_a];
    const Vec3 b=patches[seam.patch_b].vertices[seam.vertex_b];
    if(distance(a,b)>scale*1e-10)
      throw std::invalid_argument("stitched seam positions are not coincident");
    dsu.unite(vertex_offsets[seam.patch_a]+seam.vertex_a,
              vertex_offsets[seam.patch_b]+seam.vertex_b);
  }

  QuadPatchMesh combined;
  combined.vertices.reserve(static_cast<std::size_t>(total_vertices));
  combined.quads.reserve(static_cast<std::size_t>(total_quads));
  const Id missing=std::numeric_limits<Id>::max();
  std::vector<Id> root_to_vertex(static_cast<std::size_t>(total_vertices),missing);
  std::vector<Id> remap(static_cast<std::size_t>(total_vertices),missing);
  for(Id patch=0;patch<patches.size();++patch) {
    const auto &src=patches[patch];
    for(Id v=0;v<src.vertices.size();++v) {
      const Id original=vertex_offsets[patch]+v;
      const Id root=dsu.root(original);
      if(root_to_vertex[root]==missing){
        root_to_vertex[root]=static_cast<Id>(combined.vertices.size());
        // Use first representative's position; accepted seam differences are
        // only a small fraction of normalized tolerance.
        combined.vertices.push_back(src.vertices[v]);
      }
      remap[original]=root_to_vertex[root];
    }
  }

  std::vector<std::array<Id,2>> provenance;
  provenance.reserve(static_cast<std::size_t>(total_quads));
  for(Id patch=0;patch<patches.size();++patch) {
    const auto &src=patches[patch];
    for(Id q=0;q<src.quads.size();++q) {
      const auto &quad=src.quads[q];
      combined.quads.push_back({
        remap[vertex_offsets[patch]+quad[0]],
        remap[vertex_offsets[patch]+quad[1]],
        remap[vertex_offsets[patch]+quad[2]],
        remap[vertex_offsets[patch]+quad[3]]});
      provenance.push_back({patch,q});
    }
  }
  PatchCollectionIntersectionReport output;
  output.merged_report=find_quad_self_intersections(combined,max_reported_pairs);
  output.collisions.reserve(output.merged_report.pairs.size());
  for(const auto &pair:output.merged_report.pairs) {
    const auto &a=provenance[pair[0]],&b=provenance[pair[1]];
    output.collisions.push_back({a[0],a[1],b[0],b[1]});
  }
  return output;
}
} // namespace guidedquad
