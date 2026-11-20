// SPDX-License-Identifier: MIT
#include "guidedquad/guide_graph.hpp"
#include "guidedquad/cross_field.hpp"
#include <cstdlib>
#include <cmath>
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
        long index = std::stol(token.substr(0, token.find('/')));
        if (index <= 0) throw std::runtime_error("OBJ indices must be positive");
        f[i] = static_cast<std::uint32_t>(index - 1);
      }
      std::string extra;
      if (in >> extra && extra[0] != '#') throw std::runtime_error("triangulate OBJ faces first");
      mesh.triangles.push_back(f);
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
}

static std::vector<guidedquad::FlowConstraint> read_flow_constraints(const std::string &name) {
  std::ifstream file(name);
  if (!file) throw std::runtime_error("cannot open directional constraints file");
  std::vector<guidedquad::FlowConstraint> guides;
  std::string line;
  unsigned row = 0;
  while (std::getline(file, line)) {
    ++row;
    std::istringstream in(line);
    std::string first;
    if (!(in >> first) || first[0] == '#') continue;
    guidedquad::FlowConstraint c;
    unsigned long long face = 0;
    int lock = 0;
    try {
      std::size_t consumed = 0;
      face = std::stoull(first, &consumed);
      if (consumed != first.size() || face > std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("bad face index");
    } catch (const std::exception &) {
      throw std::runtime_error("invalid face index on flow constraint line " + std::to_string(row));
    }
    if (!(in >> c.direction.x >> c.direction.y >> c.direction.z >> c.weight >> lock)
        || (lock != 0 && lock != 1)) {
      throw std::runtime_error("invalid flow constraint on line " + std::to_string(row));
    }
    std::string extra;
    if (in >> extra && extra[0] != '#')
      throw std::runtime_error("extra flow constraint columns on line " + std::to_string(row));
    c.triangle = static_cast<std::uint32_t>(face);
    c.locked = (lock == 1);
    guides.push_back(c);
  }
  return guides;
}

static double distance(guidedquad::Vec3 a, guidedquad::Vec3 b) {
  return std::hypot(std::hypot(a.x-b.x, a.y-b.y), a.z-b.z);
}

static void write_flow_obj(const TriangleMesh &mesh,
                           const guidedquad::CrossFieldResult &field,
                           const std::string &name) {
  std::ofstream out(name);
  if (!out) throw std::runtime_error("cannot write cross-field OBJ");
  out << "# GuidedQuad cross-field preview. Lines only, NOT an all-quad mesh.\n";
  for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
    const auto &t = mesh.triangles[i];
    const auto &a=mesh.vertices[t[0]], &b=mesh.vertices[t[1]], &c=mesh.vertices[t[2]];
    const guidedquad::Vec3 center{(a.x+b.x+c.x)/3.0,(a.y+b.y+c.y)/3.0,(a.z+b.z+c.z)/3.0};
    const auto &dir=field.triangles[i].direction;
    const double half_length=(distance(a,b)+distance(b,c)+distance(c,a))/18.0;
    const std::size_t index=i*2+1;
    out << "# triangle " << i << " coherence " << field.triangles[i].coherence << '\n';
    out << "v " << center.x-dir.x*half_length << ' ' << center.y-dir.y*half_length << ' '
        << center.z-dir.z*half_length << '\n';
    out << "v " << center.x+dir.x*half_length << ' ' << center.y+dir.y*half_length << ' '
        << center.z+dir.z*half_length << '\n';
    out << "l " << index << ' ' << index+1 << '\n';
  }
}

int main(int argc, char **argv) {
  if (argc < 5) {
    std::cerr << "Usage: guidedquad_cli facesets input.obj face_ids.txt output.obj\n"
              << "       guidedquad_cli contours input.obj scalar_per_vertex.txt output.obj [threshold]\n"
              << "       guidedquad_cli flow input.obj constraints.txt output.obj [iterations]\n";
    return 2;
  }
  try {
    const std::string mode(argv[1]);
    const auto mesh = read_obj(argv[2]);
    if (mode == "flow") {
      guidedquad::CrossFieldOptions options;
      if (argc > 5) options.max_iterations = std::stoul(argv[5]);
      const auto field = guidedquad::solve_cross_field(
          mesh, read_flow_constraints(argv[3]), options);
      write_flow_obj(mesh, field, argv[4]);
      std::cout << "Cross field: " << field.triangles.size() << " faces, "
                << field.iterations << " iterations, " << field.warnings.size()
                << " warnings (preview lines, no quads)\n";
      for (const auto &w : field.warnings) std::cout << "warning: " << w << '\n';
      return 0;
    }
    guidedquad::GuideGraph graph;
    if (mode == "facesets") {
      graph = guidedquad::extract_region_boundaries(mesh, read_values<int>(argv[3]));
    } else if (mode == "contours") {
      graph = guidedquad::extract_scalar_contours(mesh, read_values<double>(argv[3]),
                                                   argc > 5 ? std::stod(argv[5]) : 0.0);
    } else {
      throw std::invalid_argument("mode must be facesets, contours or flow");
    }
    write_obj(graph, argv[4]);
    std::cout << "Guides: " << graph.nodes.size() << " nodes, "
              << graph.edges.size() << " edges, " << graph.paths.size()
              << " paths, " << graph.warnings.size() << " warnings\n";
  } catch (const std::exception &e) {
    std::cerr << "GuidedQuad error: " << e.what() << '\n';
    return 1;
  }
}
