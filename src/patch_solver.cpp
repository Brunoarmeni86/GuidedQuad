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

// Approximate sampling is deliberately opt-in: it may erase guide corners.
// This function is also used as an explicit comparison baseline in tests.
std::vector<Vec3> resample_arclength(const std::vector<Vec3> &line, std::uint32_t count) {
  std::vector<double> cumulative(line.size(), 0.0);
  for (std::size_t i=1; i<line.size(); ++i) {
    const double segment=distance(line[i], line[i-1]);
    if (!(segment > 0.0) || !std::isfinite(segment)) {
      throw std::invalid_argument("boundary has coincident or non-finite consecutive vertices");
    }
    cumulative[i] = cumulative[i-1] + segment;
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
    const double span = cumulative[segment] - cumulative[segment-1];
    const double t = std::clamp((target - cumulative[segment-1])/span, 0.0, 1.0);
    sampled.push_back(add(scale(line[segment-1], 1.0-t), scale(line[segment],t)));
  }
  sampled.push_back(line.back());
  return sampled;
}

// The strict sampler inserts *every* source knot exactly. It can subdivide
// original segments but never jumps across a painted corner or inflection.
std::vector<Vec3> resample_strict(const std::vector<Vec3> &line, std::uint32_t count) {
  const std::size_t n=line.size()-1;
  if (n > count) {
    throw std::invalid_argument("hard guide requires at least "+std::to_string(n)+
                                " segments; requested "+std::to_string(count));
  }
  std::vector<double> lengths(n);
  double total=0.0;
  for (std::size_t k=0;k<n;++k) {
    lengths[k]=distance(line[k],line[k+1]);
    if (!(lengths[k]>0.0) || !std::isfinite(lengths[k])) {
      throw std::invalid_argument("hard guide contains coincident or invalid consecutive points");
    }
    total+=lengths[k];
  }
  if (!std::isfinite(total)) throw std::invalid_argument("boundary arc length overflows");

  // Extra subdivisions are apportioned by arc length. Every original segment
  // receives at least one division. O(N log N + count) time and O(N) scratch.
  std::vector<std::uint32_t> splits(n,1);
  const std::uint32_t extras=count-static_cast<std::uint32_t>(n);
  std::uint32_t assigned=0;
  struct Fractional { double remainder; std::size_t index; };
  std::vector<Fractional> fractions;
  fractions.reserve(n);
  for (std::size_t k=0;k<n;++k) {
    const double share=static_cast<double>(extras)*(lengths[k]/total);
    const auto add_count=static_cast<std::uint32_t>(std::min(
        std::floor(share),static_cast<double>(extras-assigned)));
    splits[k]+=add_count;
    assigned+=add_count;
    fractions.push_back({share-std::floor(share),k});
  }
  std::sort(fractions.begin(),fractions.end(),[](const Fractional &a,const Fractional &b) {
    if (a.remainder != b.remainder) return a.remainder > b.remainder;
    return a.index < b.index;
  });
  for (std::uint32_t i=0;i<extras-assigned;++i) ++splits[fractions[i].index];

  std::vector<Vec3> result;
  result.reserve(static_cast<std::size_t>(count)+1);
  result.push_back(line.front());
  for (std::size_t k=0;k<n;++k) {
    const std::uint32_t divisions=splits[k];
    for (std::uint32_t i=1;i<divisions;++i) {
      const double t=double(i)/divisions;
      result.push_back(add(scale(line[k],1.0-t),scale(line[k+1],t)));
    }
    result.push_back(line[k+1]); // retain exact original guide knot
  }
  if (result.size()!=static_cast<std::size_t>(count)+1) {
    throw std::logic_error("strict resampling failed to allocate requested segments");
  }
  return result;
}

std::vector<Vec3> sample_boundary(const std::vector<Vec3> &line,
                                  std::uint32_t count, bool strict) {
  return strict ? resample_strict(line,count) : resample_arclength(line,count);
}

// The opt-in approximation mode reports max displacement of ORIGINAL source
// vertices from the generated boundary polyline (not just the endpoints).
// Source-vertex-to-output-segment deviation catches lost sharp corners.
double source_vertex_deviation(const std::vector<Vec3> &original,
                               const std::vector<Vec3> &sampled) {
  double maximum=0.0;
  for (const Vec3 p : original) {
    double nearest=std::numeric_limits<double>::infinity();
    for (std::size_t i=1;i<sampled.size();++i) {
      const Vec3 ab=sub(sampled[i],sampled[i-1]);
      const double squared=dot(ab,ab);
      if (!(squared>0.0)) {
        throw std::runtime_error("generated boundary has a collapsed segment");
      }
      const double t=std::clamp(dot(sub(p,sampled[i-1]),ab)/squared,0.0,1.0);
      nearest=std::min(nearest,distance(p,add(sampled[i-1],scale(ab,t))));
    }
    maximum=std::max(maximum,nearest);
  }
  return maximum;
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

QuadPatchQuality inspect(const QuadPatchBoundary &sampled, const QuadPatchMesh &mesh,
                         std::uint32_t nu, std::uint32_t nv, double input_error) {
  // Boundary samples and their source error are cached once per solve.
  // Quality search may evaluate this function many times per iteration.
  const auto &bottom=sampled.bottom, &top=sampled.top;
  const auto &left=sampled.left, &right=sampled.right;
  QuadPatchQuality metrics;
  metrics.max_boundary_error=input_error;
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
      vertex_count * options.smooth_iterations > 50000000ULL ||
      !std::isfinite(options.max_aspect_growth) ||
      options.max_aspect_growth < 0.0 || options.max_aspect_growth > 1.0) {
    throw std::invalid_argument("invalid patch smoothing options");
  }
  auto b=boundary;
  ensure_matching_corners(b);
  const bool strict=options.preserve_boundary_vertices;
  const auto bottom=sample_boundary(b.bottom,nu,strict);
  const auto top=sample_boundary(b.top,nu,strict);
  const auto left=sample_boundary(b.left,nv,strict);
  const auto right=sample_boundary(b.right,nv,strict);
  const QuadPatchBoundary sampled{bottom,right,top,left};
  // Canonicalizing nearly matching corners is a small real input deviation.
  double input_error=std::max({
      distance(boundary.left.front(),b.left.front()),
      distance(boundary.left.back(),b.left.back()),
      distance(boundary.right.front(),b.right.front()),
      distance(boundary.right.back(),b.right.back())});
  if (!strict) {
    input_error=std::max({input_error,
        source_vertex_deviation(b.bottom,bottom),
        source_vertex_deviation(b.top,top),
        source_vertex_deviation(b.left,left),
        source_vertex_deviation(b.right,right)});
  }
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

  for (std::uint32_t j=0;j<nv;++j) for (std::uint32_t i=0;i<nu;++i) {
    mesh.quads.push_back({static_cast<std::uint32_t>(index(i,j,nu)),
                          static_cast<std::uint32_t>(index(i+1,j,nu)),
                          static_cast<std::uint32_t>(index(i+1,j+1,nu)),
                          static_cast<std::uint32_t>(index(i,j+1,nu))});
  }
  // First verify the unoptimized Coons topology. A bad initial layout must
  // not be concealed by smoothing, because the boundary chart is invalid.
  output.initial_quality=inspect(sampled,mesh,nu,nv,input_error);
  output.quality=output.initial_quality;

  if (options.smooth_iterations && options.smooth_strength && nu>1 && nv>1) {
    QuadPatchMesh trial=mesh; // same topology; only interior coordinates change
    const double aspect_limit=output.initial_quality.worst_edge_aspect_ratio*
                              (1.0+options.max_aspect_growth);
    for (std::uint32_t iteration=0;iteration<options.smooth_iterations;++iteration) {
      bool accepted=false;
      double alpha=options.smooth_strength;
      // Backtracking prevents a large relaxation step from destroying side
      // aspect or flipping cells. A smaller legal step is preferred to no step.
      const int max_attempts=options.quality_guard ? 10 : 1;
      for (int attempt=0;attempt<max_attempts;++attempt) {
        for (std::uint32_t j=1;j<nv;++j) for (std::uint32_t i=1;i<nu;++i) {
          const std::size_t k=index(i,j,nu);
          const Vec3 average=scale(add(add(mesh.vertices[index(i-1,j,nu)],
                                            mesh.vertices[index(i+1,j,nu)]),
                                       add(mesh.vertices[index(i,j-1,nu)],
                                           mesh.vertices[index(i,j+1,nu)])),0.25);
          trial.vertices[k]=add(scale(mesh.vertices[k],1.0-alpha),
                                scale(average,alpha));
        }
        QuadPatchQuality candidate;
        bool valid=true;
        try {
          candidate=inspect(sampled,trial,nu,nv,input_error);
        } catch (const std::runtime_error &) {
          valid=false;
          if (!options.quality_guard) throw;
        }
        const bool better=valid && (!options.quality_guard ||
            (candidate.worst_edge_aspect_ratio <= aspect_limit &&
             candidate.mean_abs_orthogonality+1e-12 < output.quality.mean_abs_orthogonality));
        if (better) {
          mesh.vertices.swap(trial.vertices);
          output.quality=candidate;
          ++output.accepted_smooth_iterations;
          accepted=true;
          break;
        }
        alpha*=0.5;
      }
      if (!accepted) break; // stationary point or constraints saturated
    }
  }
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
