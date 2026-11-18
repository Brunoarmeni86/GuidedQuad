// SPDX-License-Identifier: MIT
#include "guidedquad/patch_solver.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace guidedquad {
namespace {
Vec3 add(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vec3 sub(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
Vec3 scale(Vec3 a, double v) { return {a.x*v, a.y*v, a.z*v}; }
double dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3 cross(Vec3 a, Vec3 b) {
  return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
double length(Vec3 a) { return std::hypot(a.x, a.y, a.z); }
double distance(Vec3 a, Vec3 b) { return length(sub(a,b)); }
bool finite(Vec3 p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }

void check_line(const std::vector<Vec3> &line) {
  if (line.size() < 2) throw std::invalid_argument("patch boundary needs at least two points");
  for (Vec3 p : line) if (!finite(p)) {
    throw std::invalid_argument("patch boundary has non-finite point");
  }
}

std::vector<Vec3> resample(const std::vector<Vec3> &line, std::uint32_t count) {
  std::vector<double> cumulative(line.size(), 0.0);
  for (std::size_t i=1; i<line.size(); ++i) {
    cumulative[i] = cumulative[i-1] + distance(line[i], line[i-1]);
  }
  const double total = cumulative.back();
  if (!(total > 0.0) || !std::isfinite(total)) {
    throw std::invalid_argument("patch boundary has zero or invalid arc length");
  }
  std::vector<Vec3> sampled;
  sampled.reserve(static_cast<std::size_t>(count)+1);
  sampled.push_back(line.front());
  std::size_t segment = 1;
  for (std::uint32_t i=1; i<count; ++i) {
    const double target = total * (double(i) / count);
    while (segment+1 < line.size() && cumulative[segment] < target) ++segment;
    while (segment+1 < line.size() && cumulative[segment] == cumulative[segment-1]) ++segment;
    const double span = cumulative[segment] - cumulative[segment-1];
    if (!(span > 0.0)) {
      throw std::invalid_argument("boundary interpolation has zero-length span");
    }
    const double t = std::clamp((target - cumulative[segment-1])/span, 0.0, 1.0);
    sampled.push_back(add(scale(line[segment-1], 1.0-t), scale(line[segment],t)));
  }
  sampled.push_back(line.back());
  return sampled;
}

void ensure_matching_corners(QuadPatchBoundary &b) {
  check_line(b.bottom); check_line(b.right); check_line(b.top); check_line(b.left);
  Vec3 minp = b.bottom.front(), maxp=minp;
  for (const auto *side : {&b.bottom,&b.right,&b.top,&b.left}) {
    for (Vec3 p : *side) {
      minp.x=std::min(minp.x,p.x); minp.y=std::min(minp.y,p.y);
      minp.z=std::min(minp.z,p.z);
      maxp.x=std::max(maxp.x,p.x); maxp.y=std::max(maxp.y,p.y);
      maxp.z=std::max(maxp.z,p.z);
    }
  }
  const double scale_size = distance(maxp,minp);
  if (!(scale_size > 0.0) || !std::isfinite(scale_size)) {
    throw std::invalid_argument("patch has zero or non-finite extent");
  }
  const double tolerance = 1e-10*scale_size;
  if (distance(b.bottom.front(),b.left.front()) > tolerance ||
      distance(b.bottom.back(),b.right.front()) > tolerance ||
      distance(b.top.front(),b.left.back()) > tolerance ||
      distance(b.top.back(),b.right.back()) > tolerance) {
    throw std::invalid_argument("patch boundary corners do not match");
  }
  // Canonical corners chosen from horizontal curves. Snap only when already
  // within the declared input tolerance, never deform intermediate guides.
  b.left.front() = b.bottom.front();
  b.right.front() = b.bottom.back();
  b.left.back() = b.top.front();
  b.right.back() = b.top.back();
}

std::size_t index(std::uint32_t i, std::uint32_t j, std::uint32_t nu) {
  return static_cast<std::size_t>(j)*(static_cast<std::size_t>(nu)+1)+i;
}

QuadPatchQuality inspect(const QuadPatchBoundary &boundary, const QuadPatchMesh &mesh,
                         std::uint32_t nu, std::uint32_t nv) {
  const auto bottom=resample(boundary.bottom,nu), top=resample(boundary.top,nu);
  const auto left=resample(boundary.left,nv), right=resample(boundary.right,nv);
  QuadPatchQuality metrics;
  for (std::uint32_t i=0; i<=nu; ++i) {
    metrics.max_boundary_error=std::max(metrics.max_boundary_error,
        distance(mesh.vertices[index(i,0,nu)],bottom[i]));
    metrics.max_boundary_error=std::max(metrics.max_boundary_error,
        distance(mesh.vertices[index(i,nv,nu)],top[i]));
  }
  for (std::uint32_t j=0; j<=nv; ++j) {
    metrics.max_boundary_error=std::max(metrics.max_boundary_error,
        distance(mesh.vertices[index(0,j,nu)],left[j]));
    metrics.max_boundary_error=std::max(metrics.max_boundary_error,
        distance(mesh.vertices[index(nu,j,nu)],right[j]));
  }
  Vec3 minp=mesh.vertices.front(), maxp=minp;
  for (Vec3 p : mesh.vertices) {
    if (!finite(p)) throw std::runtime_error("patch solver produced non-finite vertex");
    minp.x=std::min(minp.x,p.x);minp.y=std::min(minp.y,p.y);minp.z=std::min(minp.z,p.z);
    maxp.x=std::max(maxp.x,p.x);maxp.y=std::max(maxp.y,p.y);maxp.z=std::max(maxp.z,p.z);
  }
  const double size=distance(minp,maxp);
  const double area_floor=(size*size)*1e-14 / (double(nu)*double(nv));
  std::vector<Vec3> prev_row(nu);
  double sum_orthogonality=0.0;
  metrics.minimum_face_area=std::numeric_limits<double>::infinity();
  for (std::uint32_t j=0; j<nv; ++j) {
    Vec3 prev_in_row{0,0,0};
    for (std::uint32_t i=0; i<nu; ++i) {
      const auto &q=mesh.quads[static_cast<std::size_t>(j)*nu+i];
      const Vec3 a=mesh.vertices[q[0]], b=mesh.vertices[q[1]];
      const Vec3 c=mesh.vertices[q[2]], d=mesh.vertices[q[3]];
      const Vec3 n1=cross(sub(b,a),sub(c,a));
      const Vec3 n2=cross(sub(c,a),sub(d,a));
      const double aa=length(n1), ab=length(n2);
      if (!(aa > area_floor) || !(ab > area_floor) || dot(n1,n2) <= 0.0) {
        throw std::runtime_error("patch contains collapsed or locally inverted quad");
      }
      const Vec3 n=add(n1,n2);
      if (i > 0 && dot(prev_in_row,n) <= 0.0) {
        throw std::runtime_error("patch fold detected across quad columns");
      }
      if (j > 0 && dot(prev_row[i],n) <= 0.0) {
        throw std::runtime_error("patch fold detected across quad rows");
      }
      prev_in_row=n;
      prev_row[i]=n;
      metrics.minimum_face_area=std::min(metrics.minimum_face_area,0.5*(aa+ab));
      const Vec3 vertices[4]={a,b,c,d};
      double shortest=std::numeric_limits<double>::infinity(), longest=0;
      for (int k=0; k<4; ++k) {
        const Vec3 outgoing=sub(vertices[(k+1)%4],vertices[k]);
        const Vec3 incoming=sub(vertices[(k+3)%4],vertices[k]);
        const double le=length(outgoing), li=length(incoming);
        shortest=std::min(shortest,le);longest=std::max(longest,le);
        if (!(le > 0.0 && li > 0.0)) {
          throw std::runtime_error("patch contains a collapsed edge");
        }
        sum_orthogonality+=std::abs(dot(outgoing,incoming)/(le*li));
      }
      metrics.worst_edge_aspect_ratio=std::max(metrics.worst_edge_aspect_ratio,longest/shortest);
    }
  }
  metrics.mean_abs_orthogonality=sum_orthogonality/(4.0*mesh.quads.size());
  return metrics;
}
} // namespace

QuadPatchResult solve_quad_patch(const QuadPatchBoundary &boundary,
                                const QuadPatchOptions &options) {
  const std::uint32_t nu=options.u_segments, nv=options.v_segments;
  const std::uint64_t vertex_count=(std::uint64_t(nu)+1)*(std::uint64_t(nv)+1);
  const std::uint64_t quad_count=std::uint64_t(nu)*nv;
  if (nu==0 || nv==0 || vertex_count > 2000000 || quad_count > 2000000) {
    throw std::invalid_argument("quad patch grid size is zero or exceeds V0.2 limit");
  }
  if (!std::isfinite(options.smooth_strength) ||
      options.smooth_strength < 0.0 || options.smooth_strength > 1.0 ||
      options.smooth_iterations > 5000 ||
      vertex_count * options.smooth_iterations > 50000000ULL) {
    throw std::invalid_argument("invalid patch smoothing options");
  }
  auto b=boundary;
  ensure_matching_corners(b);
  const auto bottom=resample(b.bottom,nu),top=resample(b.top,nu);
  const auto left=resample(b.left,nv),right=resample(b.right,nv);
  const Vec3 p00=bottom.front(), p10=bottom.back();
  const Vec3 p01=top.front(), p11=top.back();
  QuadPatchResult output;
  auto &mesh=output.mesh;
  mesh.vertices.resize(static_cast<std::size_t>(vertex_count));
  mesh.quads.reserve(static_cast<std::size_t>(quad_count));

  // Four-sided Coons patch: all four boundary curves are satisfied exactly.
  for (std::uint32_t j=0; j<=nv; ++j) {
    const double v=double(j)/nv;
    for (std::uint32_t i=0; i<=nu; ++i) {
      const double u=double(i)/nu;
      const Vec3 curveblend=add(
          add(scale(bottom[i],1-v),scale(top[i],v)),
          add(scale(left[j],1-u),scale(right[j],u)));
      const Vec3 cornerblend=add(
          add(scale(p00,(1-u)*(1-v)),scale(p10,u*(1-v))),
          add(scale(p01,(1-u)*v),scale(p11,u*v)));
      mesh.vertices[index(i,j,nu)]=sub(curveblend,cornerblend);
    }
  }
  // Copy the boundary samples back rather than depending on cancellation in
  // the Coons formula. Never smooth these hard guides.
  for (std::uint32_t i=0;i<=nu;++i) {
    mesh.vertices[index(i,0,nu)]=bottom[i];
    mesh.vertices[index(i,nv,nu)]=top[i];
  }
  for (std::uint32_t j=0;j<=nv;++j) {
    mesh.vertices[index(0,j,nu)]=left[j];
    mesh.vertices[index(nu,j,nu)]=right[j];
  }

  if (options.smooth_iterations && options.smooth_strength && nu>1 && nv>1) {
    std::vector<Vec3> next=mesh.vertices;
    for (std::uint32_t iteration=0;iteration<options.smooth_iterations;++iteration) {
      for (std::uint32_t j=1;j<nv;++j) for (std::uint32_t i=1;i<nu;++i) {
        const std::size_t k=index(i,j,nu);
        const Vec3 average=scale(add(add(mesh.vertices[index(i-1,j,nu)],
                                          mesh.vertices[index(i+1,j,nu)]),
                                     add(mesh.vertices[index(i,j-1,nu)],
                                         mesh.vertices[index(i,j+1,nu)])),0.25);
        next[k]=add(scale(mesh.vertices[k],1.0-options.smooth_strength),
                    scale(average,options.smooth_strength));
      }
      mesh.vertices.swap(next);
    }
  }
  for (std::uint32_t j=0;j<nv;++j) for (std::uint32_t i=0;i<nu;++i) {
    mesh.quads.push_back({static_cast<std::uint32_t>(index(i,j,nu)),
                          static_cast<std::uint32_t>(index(i+1,j,nu)),
                          static_cast<std::uint32_t>(index(i+1,j+1,nu)),
                          static_cast<std::uint32_t>(index(i,j+1,nu))});
  }
  output.quality=inspect(b,mesh,nu,nv);
  return output;
}

QuadPatchBoundary patch_boundary_from_loop(const GuideGraph &graph,
                                           std::size_t path_index,
                                           std::array<std::size_t,4> corners) {
  if (path_index >= graph.paths.size()) {
    throw std::invalid_argument("patch guide path index is out of range");
  }
  const GuidePath &path=graph.paths[path_index];
  if (!path.closed || path.nodes.size()<5 ||
      path.nodes.front()!=path.nodes.back()) {
    throw std::invalid_argument("quad patch requires a closed guide path");
  }
  const std::size_t n=path.nodes.size()-1;
  if (corners[0]>=corners[1] || corners[1]>=corners[2] ||
      corners[2]>=corners[3] || corners[3]>=n) {
    throw std::invalid_argument("four ordered distinct loop corners required");
  }
  const auto extract=[&](std::size_t start,std::size_t end) {
    std::vector<Vec3> result;
    result.reserve(end-start+1);
    for (std::size_t k=start;k<=end;++k) {
      const auto vertex_id=path.nodes[k%n];
      if (vertex_id>=graph.nodes.size()) {
        throw std::invalid_argument("guide loop references an invalid node");
      }
      result.push_back(graph.nodes[vertex_id].position);
    }
    return result;
  };
  auto bottom=extract(corners[0],corners[1]);
  auto right=extract(corners[1],corners[2]);
  auto top=extract(corners[2],corners[3]);
  auto left=extract(corners[3],corners[0]+n);
  std::reverse(top.begin(),top.end());
  std::reverse(left.begin(),left.end());
  return {std::move(bottom),std::move(right),std::move(top),std::move(left)};
}

} // namespace guidedquad
