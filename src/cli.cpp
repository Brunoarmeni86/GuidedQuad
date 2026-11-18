// SPDX-License-Identifier: MIT
#include "guidedquad/guide_graph.hpp"
#include "guidedquad/quad_intersections.hpp"
#include "guidedquad/patch_solver.hpp"
#include "guidedquad/surface_projection.hpp"
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using guidedquad::TriangleMesh;

static TriangleMesh read_obj(const std::string &name) {
  std::ifstream file(name);
  if (!file) throw std::runtime_error("cannot open input OBJ");
  TriangleMesh mesh;
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream in(line);
    std::string tag;
    in >> tag;
    if (tag == "v") {
      guidedquad::Vec3 p;
      if (!(in >> p.x >> p.y >> p.z)) throw std::runtime_error("invalid OBJ vertex");
      mesh.vertices.push_back(p);
    } else if (tag == "f") {
      std::array<std::uint32_t, 3> f{};
      for (int i = 0; i < 3; ++i) {
        std::string token;
        if (!(in >> token)) throw std::runtime_error("only triangulated OBJ is supported");
        const std::string raw_index = token.substr(0, token.find('/'));
        if (raw_index.empty()) throw std::runtime_error("OBJ face index is missing");
        std::size_t consumed = 0;
        const unsigned long long index = std::stoull(raw_index, &consumed, 10);
        if (consumed != raw_index.size() || index == 0 ||
            index > std::numeric_limits<std::uint32_t>::max()) {
          throw std::runtime_error("OBJ face index is invalid or out of range");
        }
        f[i] = static_cast<std::uint32_t>(index - 1);
      }
      std::string extra;
      if (in >> extra && extra[0] != '#') throw std::runtime_error("triangulate OBJ faces first");
      mesh.triangles.push_back(f);
    }
  }
  return mesh;
}

static guidedquad::QuadPatchMesh read_quad_obj(const std::string &name) {
  std::ifstream file(name);
  if (!file) throw std::runtime_error("cannot open quad OBJ input");
  guidedquad::QuadPatchMesh mesh;
  std::string line;
  while(std::getline(file,line)) {
    std::istringstream in(line);
    std::string tag;
    in>>tag;
    if(tag=="v") {
      guidedquad::Vec3 p;
      if(!(in>>p.x>>p.y>>p.z))throw std::runtime_error("invalid OBJ position");
      mesh.vertices.push_back(p);
    } else if(tag=="f") {
      std::array<std::uint32_t,4> q{};
      for(int i=0;i<4;++i) {
        std::string token;
        if(!(in>>token))throw std::runtime_error("check-quads requires only quad faces");
        const std::string index_text=token.substr(0,token.find('/'));
        std::size_t consumed=0;
        const unsigned long long index=std::stoull(index_text,&consumed,10);
        if(index_text.empty()||consumed!=index_text.size()||index==0||
           index>std::numeric_limits<std::uint32_t>::max())
          throw std::runtime_error("bad positive OBJ face index");
        q[i]=static_cast<std::uint32_t>(index-1);
      }
      std::string extra;
      if(in>>extra && extra[0]!='#')
        throw std::runtime_error("non-quad OBJ face encountered");
      mesh.quads.push_back(q);
    }
  }
  return mesh;
}

template<typename T>
static std::vector<T> read_values(const std::string &name) {
  std::ifstream file(name);
  if (!file) throw std::runtime_error("cannot open values file");
  std::vector<T> values;
  T val;
  while (file >> val) values.push_back(val);
  if (!file.eof()) throw std::runtime_error("invalid numeric values file");
  return values;
}

static void write_obj(const guidedquad::GuideGraph &g, const std::string &name) {
  std::ofstream file(name);
  if (!file) throw std::runtime_error("cannot write output OBJ");
  file << std::setprecision(std::numeric_limits<double>::max_digits10);
  file << "# GuidedQuad V0.1: guide polylines only, not a quad mesh\n";
  for (const auto &n : g.nodes) {
    file << "v " << n.position.x << ' ' << n.position.y << ' ' << n.position.z << '\n';
  }
  for (const auto &p : g.paths) {
    if (p.nodes.size() < 2) continue;
    file << 'l';
    for (std::uint32_t n : p.nodes) file << ' ' << n + 1;
    file << '\n';
  }
  file.flush();
  if (!file) throw std::runtime_error("failed to write complete output OBJ");
}

static std::uint32_t parse_u32(const std::string &value) {
  std::size_t consumed=0;
  const unsigned long long parsed=std::stoull(value,&consumed,10);
  if (consumed!=value.size() || parsed>std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("invalid 32-bit unsigned integer argument");
  }
  return static_cast<std::uint32_t>(parsed);
}

static void write_quads(const guidedquad::QuadPatchMesh &mesh,
                        const std::string &name) {
  std::ofstream file(name);
  if (!file) throw std::runtime_error("cannot open quad OBJ output");
  file << std::setprecision(std::numeric_limits<double>::max_digits10);
  file << "# GuidedQuad constrained four-sided quad patch\n";
  for (const auto &p : mesh.vertices) {
    file << "v " << p.x << ' ' << p.y << ' ' << p.z << '\n';
  }
  for (const auto &q : mesh.quads) {
    file << "f " << q[0]+1 << ' ' << q[1]+1 << ' '
         << q[2]+1 << ' ' << q[3]+1 << '\n';
  }
  file.flush();
  if (!file) throw std::runtime_error("failed to finish quad OBJ output");
}

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: guidedquad_cli facesets input.obj face_ids.txt output.obj\n"
              << "       guidedquad_cli contours input.obj scalar_per_vertex.txt output.obj [threshold]\n"
              << "       guidedquad_cli patch-loop input.obj output.obj c0 c1 c2 c3 nu nv\n"
              << "       guidedquad_cli check-quads quad_mesh.obj\n"
              << "       guidedquad_cli project-loop border.obj reference.obj output.obj c0 c1 c2 c3 nu nv [iterations] [strength] [normal_dot] [max_distance] [component_id]\n"
              << "Patch corners are four ordered indices in the extracted closed boundary path.\n";
    return 2;
  }
  try {
    const std::string mode(argv[1]);
    if(mode=="check-quads") {
      if(argc!=3)throw std::invalid_argument("check-quads takes exactly one quad OBJ");
      const auto report=guidedquad::find_quad_self_intersections(read_quad_obj(argv[2]),16);
      std::cout<<"Quad collision check: "<<report.pairs.size()<<" detected pair(s), "
               <<report.bvh_pair_tests<<" BVH node-pair tests, "
               <<report.candidate_quad_pairs<<" candidate quad pairs";
      if(report.truncated)std::cout<<" (search stopped at report limit)";
      std::cout<<'\n';
      for(const auto &pair:report.pairs)
        std::cerr<<"Colliding quads: "<<pair[0]<<" and "<<pair[1]<<" (zero-based)\n";
      return report.pairs.empty()?0:3;
    }
    if(argc<4)throw std::invalid_argument("missing arguments for quad patch or guide extraction");
    const auto mesh = read_obj(argv[2]);
    if (mode == "patch-loop" || mode == "project-loop") {
      const bool projecting=(mode=="project-loop");
      const int shift=projecting?1:0;
      if ((!projecting && argc != 10) || (projecting && (argc<11||argc>16))) {
        throw std::invalid_argument("patch mode needs 4 corner positions and U/V segment counts");
      }
      const auto border=guidedquad::extract_region_boundaries(
          mesh,std::vector<int>(mesh.triangles.size(),0),true);
      if (border.paths.size()!=1 || !border.paths[0].closed) {
        throw std::invalid_argument("patch requires exactly one closed outer boundary loop");
      }
      const auto boundary=guidedquad::patch_boundary_from_loop(
          border,0,{parse_u32(argv[4+shift]),parse_u32(argv[5+shift]),
                    parse_u32(argv[6+shift]),parse_u32(argv[7+shift])});
      guidedquad::QuadPatchOptions options;
      options.u_segments=parse_u32(argv[8+shift]);
      options.v_segments=parse_u32(argv[9+shift]);
      auto result=guidedquad::solve_quad_patch(boundary,options);
      if (projecting) {
        const TriangleMesh reference=read_obj(argv[3]);
        guidedquad::SurfaceProjectionOptions project_options;
        if (argc>=12) project_options.iterations=parse_u32(argv[11]);
        if (argc>=13) {
          std::size_t consumed=0;
          project_options.strength=std::stod(argv[12],&consumed);
          if(consumed!=std::string(argv[12]).size()){
            throw std::invalid_argument("projection strength contains invalid characters");
          }
        }
        if (argc>=14) {
          std::size_t consumed=0;
          project_options.min_reference_normal_dot=std::stod(argv[13],&consumed);
          if(consumed!=std::string(argv[13]).size()) {
            throw std::invalid_argument("reference normal dot contains invalid characters");
          }
        }
        if (argc>=15) {
          std::size_t consumed=0;
          project_options.max_projection_distance=std::stod(argv[14],&consumed);
          if(consumed!=std::string(argv[14]).size()) {
            throw std::invalid_argument("max projection distance contains invalid characters");
          }
        }
        if (argc>=16) {
          project_options.source_component_id=parse_u32(argv[15]);
        }
        auto projected=guidedquad::project_quad_patch_to_surface(
            reference,result.mesh,options.u_segments,options.v_segments,project_options);
        write_quads(projected.mesh,argv[4]);
        std::cout<<"Projected quad patch: "<<projected.mesh.quads.size()
                 <<" quads, nearest-point RMS "
                 <<projected.stats.initial_rms_distance<<" -> "
                 <<projected.stats.final_rms_distance
                 <<", moved interior vertices "<<projected.stats.moved_vertices
                 <<", normal-filtered triangles "<<projected.stats.normal_rejected_triangles
                 <<", triangle tests "<<projected.stats.triangle_tests
                 <<" across "<<projected.stats.closest_queries
                 <<" BVH queries, accepted steps "<<projected.stats.accepted_iterations
                 <<'\n';
      } else {
        write_quads(result.mesh,argv[3]);
        std::cout << "Quad patch: " << result.mesh.vertices.size() << " vertices, "
                  << result.mesh.quads.size() << " quads; boundary deviation "
                  << result.quality.max_boundary_error << "; worst aspect ratio "
                  << result.quality.worst_edge_aspect_ratio
                  << "; min scaled Jacobian " << result.quality.minimum_scaled_jacobian
                  << "; max warpage " << result.quality.max_diagonal_warpage_deg
                  << " deg\n";
      }
      return 0;
    }
    if (argc<5) throw std::invalid_argument("facesets/contours mode requires input attributes and output path");
    guidedquad::GuideGraph graph;
    if (mode == "facesets") {
      graph = guidedquad::extract_region_boundaries(mesh, read_values<int>(argv[3]));
    } else if (mode == "contours") {
      graph = guidedquad::extract_scalar_contours(mesh, read_values<double>(argv[3]),
                                                   argc > 5 ? std::stod(argv[5]) : 0.0);
    } else {
      throw std::invalid_argument("mode must be facesets or contours");
    }
    write_obj(graph, argv[4]);
    for (const auto &warning : graph.warnings) std::cerr << "GuidedQuad warning: " << warning << '\n';
    std::cout << "Guides: " << graph.nodes.size() << " nodes, "
              << graph.edges.size() << " edges, " << graph.paths.size()
              << " paths, " << graph.warnings.size() << " warnings\n";
  } catch (const std::exception &e) {
    std::cerr << "GuidedQuad error: " << e.what() << '\n';
    return 1;
  }
}
