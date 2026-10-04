/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 52 (2026-08-21, new only — do not restore deleted ids):
 *  - Visibility Graph 2D (Overmars–Welzl chords of XY obstacle segments)
 *  - Terrain TIN (XY CDT, keep per-vertex Z)
 *  - Min Circle 3D (smallest enclosing circle in the LSQ plane)
 *
 * Principal Direction Lines is node-side (existing principal curvature intern).
 *
 * Skipped: OpenGR, Kinetic EPECK, Mesh_3 volume, PolyFit SCIP, Ridges_3,
 * Classification, OSQP, Hyperbolic Delaunay, weighted Delaunay / Lloyd,
 * banned measure/query/remesh, previously deleted nodes.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Min_circle_2.h>
#include <CGAL/Min_circle_2_traits_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/assertions_behaviour.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/number_utils.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using FT = Kernel::FT;
using Segment_2 = Kernel::Segment_2;
using Vb = CGAL::Delaunay_mesh_vertex_base_2<Kernel>;
using Fb = CGAL::Delaunay_mesh_face_base_2<Kernel>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, Tds, CGAL::Exact_predicates_tag>;

static Point_2 to_p2_xy(const float *xyz)
{
  return Point_2(double(xyz[0]), double(xyz[1]));
}

static Point_3 to_p3(const float *xyz)
{
  return Point_3(double(xyz[0]), double(xyz[1]), double(xyz[2]));
}

static void push_p3(std::vector<float> &pos, const Point_3 &p)
{
  pos.push_back(float(CGAL::to_double(p.x())));
  pos.push_back(float(CGAL::to_double(p.y())));
  pos.push_back(float(CGAL::to_double(p.z())));
}

static std::pair<int, int> undirected(int a, int b)
{
  return (a < b) ? std::pair<int, int>(a, b) : std::pair<int, int>(b, a);
}

}  // namespace

WireResult mesh_visibility_graph_2(const MeshIn &mesh)
{
  WireResult result;
  CgalThrowGuard guard;
  try {
    if (!mesh.positions || mesh.verts_num < 2) {
      result.error = "Visibility Graph 2D needs at least 2 verts";
      return result;
    }
    std::vector<std::pair<int, int>> obstacles;
    obstacles.reserve(size_t(std::max(mesh.edges_num, mesh.corners_num)));
    auto add_seg = [&](int a, int b) {
      if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
        return;
      }
      obstacles.push_back(undirected(a, b));
    };
    if (mesh.edges_num > 0 && mesh.edge_v0 && mesh.edge_v1) {
      for (int e = 0; e < mesh.edges_num; e++) {
        add_seg(mesh.edge_v0[e], mesh.edge_v1[e]);
      }
    }
    if (mesh.faces_num > 0 && mesh.face_offsets && mesh.corner_verts) {
      for (int f = 0; f < mesh.faces_num; f++) {
        const int a = mesh.face_offsets[f];
        const int b = mesh.face_offsets[f + 1];
        const int n = b - a;
        if (n < 2) {
          continue;
        }
        for (int i = 0; i < n; i++) {
          add_seg(mesh.corner_verts[a + i], mesh.corner_verts[a + ((i + 1) % n)]);
        }
      }
    }
    std::sort(obstacles.begin(), obstacles.end());
    obstacles.erase(std::unique(obstacles.begin(), obstacles.end()), obstacles.end());
    if (obstacles.empty()) {
      result.error = "Visibility Graph 2D needs obstacle edges (mesh edges or face borders)";
      return result;
    }
    std::vector<char> used;
    used.resize(size_t(mesh.verts_num), 0);
    for (const std::pair<int, int> &e : obstacles) {
      used[size_t(e.first)] = 1;
      used[size_t(e.second)] = 1;
    }
    std::vector<int> verts;
    for (int i = 0; i < mesh.verts_num; i++) {
      if (used[size_t(i)]) {
        verts.push_back(i);
      }
    }
    if (int(verts.size()) > 1500) {
      result.error = "Visibility Graph 2D: too many obstacle verts (max 1500)";
      return result;
    }
    std::vector<Point_2> p2;
    p2.resize(size_t(mesh.verts_num));
    for (int i = 0; i < mesh.verts_num; i++) {
      p2[size_t(i)] = to_p2_xy(mesh.positions + i * 3);
    }
    std::vector<Segment_2> wall;
    wall.reserve(obstacles.size());
    for (const std::pair<int, int> &e : obstacles) {
      wall.push_back(Segment_2(p2[size_t(e.first)], p2[size_t(e.second)]));
    }
    std::set<std::pair<int, int>> vis(obstacles.begin(), obstacles.end());
    const int nv = int(verts.size());
    for (int i = 0; i < nv; i++) {
      const int a = verts[size_t(i)];
      for (int j = i + 1; j < nv; j++) {
        const int b = verts[size_t(j)];
        const std::pair<int, int> key = undirected(a, b);
        if (vis.find(key) != vis.end()) {
          continue;
        }
        if (p2[size_t(a)] == p2[size_t(b)]) {
          continue;
        }
        const Segment_2 chord(p2[size_t(a)], p2[size_t(b)]);
        bool blocked = false;
        for (int w = 0; w < int(obstacles.size()); w++) {
          const int c = obstacles[size_t(w)].first;
          const int d = obstacles[size_t(w)].second;
          if (c == a || c == b || d == a || d == b) {
            continue;
          }
          if (CGAL::do_intersect(chord, wall[size_t(w)])) {
            blocked = true;
            break;
          }
        }
        if (!blocked) {
          vis.insert(key);
        }
      }
    }
    result.positions.resize(size_t(mesh.verts_num) * 3);
    for (int i = 0; i < mesh.verts_num; i++) {
      result.positions[size_t(i) * 3 + 0] = mesh.positions[i * 3 + 0];
      result.positions[size_t(i) * 3 + 1] = mesh.positions[i * 3 + 1];
      result.positions[size_t(i) * 3 + 2] = mesh.positions[i * 3 + 2];
    }
    result.edge_v0.reserve(vis.size());
    result.edge_v1.reserve(vis.size());
    for (const std::pair<int, int> &e : vis) {
      result.edge_v0.push_back(e.first);
      result.edge_v1.push_back(e.second);
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Visibility Graph 2D produced no edges";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Visibility Graph 2D failed";
  }
  return result;
}

MeshResult mesh_terrain_tin(const MeshIn &mesh, int fill_rule)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    if (!mesh.positions || mesh.verts_num < 3) {
      result.error = "Terrain TIN needs at least 3 verts";
      return result;
    }
    CDT cdt;
    std::map<CDT::Vertex_handle, float> zmap;
    for (int i = 0; i < mesh.verts_num; i++) {
      const float *p = mesh.positions + i * 3;
      CDT::Vertex_handle vh = cdt.insert(to_p2_xy(p));
      if (zmap.find(vh) == zmap.end()) {
        zmap[vh] = p[2];
      }
    }
    auto insert_con = [&](int a, int b) {
      if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
        return;
      }
      const float *pa = mesh.positions + a * 3;
      const float *pb = mesh.positions + b * 3;
      CDT::Vertex_handle va = cdt.insert(to_p2_xy(pa));
      CDT::Vertex_handle vb = cdt.insert(to_p2_xy(pb));
      if (va != vb) {
        cdt.insert_constraint(va, vb);
      }
    };
    if (mesh.edges_num > 0 && mesh.edge_v0 && mesh.edge_v1) {
      for (int e = 0; e < mesh.edges_num; e++) {
        insert_con(mesh.edge_v0[e], mesh.edge_v1[e]);
      }
    }
    if (mesh.faces_num > 0 && mesh.face_offsets && mesh.corner_verts) {
      for (int f = 0; f < mesh.faces_num; f++) {
        const int a = mesh.face_offsets[f];
        const int b = mesh.face_offsets[f + 1];
        const int n = b - a;
        if (n < 2) {
          continue;
        }
        for (int i = 0; i < n; i++) {
          insert_con(mesh.corner_verts[a + i], mesh.corner_verts[a + ((i + 1) % n)]);
        }
      }
    }
    const bool even_odd = (fill_rule == 1) && (mesh.faces_num > 0 || mesh.edges_num > 0);
    if (even_odd) {
      CGAL::mark_domain_in_triangulation(cdt);
    }
    for (CDT::Finite_faces_iterator fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end();
         ++fit)
    {
      if (even_odd && !fit->is_in_domain()) {
        continue;
      }
      const int vbase = result.verts_num();
      bool ok = true;
      for (int k = 0; k < 3; k++) {
        CDT::Vertex_handle vh = fit->vertex(k);
        auto z = zmap.find(vh);
        if (z == zmap.end()) {
          ok = false;
          break;
        }
        result.positions.push_back(float(CGAL::to_double(vh->point().x())));
        result.positions.push_back(float(CGAL::to_double(vh->point().y())));
        result.positions.push_back(z->second);
      }
      if (!ok) {
        result.positions.resize(size_t(vbase) * 3);
        continue;
      }
      result.corner_verts.push_back(vbase);
      result.corner_verts.push_back(vbase + 1);
      result.corner_verts.push_back(vbase + 2);
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Terrain TIN produced no triangles";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Terrain TIN failed";
  }
  return result;
}

MeshResult points_min_circle_3(const float *positions, int n, int segments)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    if (!positions || n < 2) {
      result.error = "Min Circle 3D needs at least 2 points";
      return result;
    }
    if (segments < 8) {
      segments = 8;
    }
    if (segments > 128) {
      segments = 128;
    }
    double cx = 0.0, cy = 0.0, cz = 0.0;
    for (int i = 0; i < n; i++) {
      cx += double(positions[i * 3 + 0]);
      cy += double(positions[i * 3 + 1]);
      cz += double(positions[i * 3 + 2]);
    }
    cx /= double(n);
    cy /= double(n);
    cz /= double(n);
    double xx = 0.0, xy = 0.0, xz = 0.0, yy = 0.0, yz = 0.0, zz = 0.0;
    for (int i = 0; i < n; i++) {
      const double dx = double(positions[i * 3 + 0]) - cx;
      const double dy = double(positions[i * 3 + 1]) - cy;
      const double dz = double(positions[i * 3 + 2]) - cz;
      xx += dx * dx;
      xy += dx * dy;
      xz += dx * dz;
      yy += dy * dy;
      yz += dy * dz;
      zz += dz * dz;
    }
    /* LSQ plane normal ≈ smallest covariance eigenvector: cross of the two
     * covariance rows with the largest parallelogram area. */
    const double ev[3][3] = {{xx, xy, xz}, {xy, yy, yz}, {xz, yz, zz}};
    double best = -1.0;
    double fnx = 0.0, fny = 0.0, fnz = 1.0;
    for (int a = 0; a < 3; a++) {
      const int b = (a + 1) % 3;
      const double axv = ev[a][0], ayv = ev[a][1], azv = ev[a][2];
      const double bxv = ev[b][0], byv = ev[b][1], bzv = ev[b][2];
      const double cxv = ayv * bzv - azv * byv;
      const double cyv = azv * bxv - axv * bzv;
      const double czv = axv * byv - ayv * bxv;
      const double area = cxv * cxv + cyv * cyv + czv * czv;
      if (area > best) {
        best = area;
        fnx = cxv;
        fny = cyv;
        fnz = czv;
      }
    }
    double nlen = std::sqrt(fnx * fnx + fny * fny + fnz * fnz);
    if (nlen < 1e-20) {
      fnx = 0.0;
      fny = 0.0;
      fnz = 1.0;
      nlen = 1.0;
    }
    fnx /= nlen;
    fny /= nlen;
    fnz /= nlen;
    double ux = 0.0, uy = 1.0, uz = 0.0;
    if (std::abs(fny) > 0.9) {
      ux = 1.0;
      uy = 0.0;
      uz = 0.0;
    }
    double tx = fny * uz - fnz * uy;
    double ty = fnz * ux - fnx * uz;
    double tz = fnx * uy - fny * ux;
    const double tlen = std::sqrt(tx * tx + ty * ty + tz * tz);
    if (tlen < 1e-20) {
      result.error = "Min Circle 3D: degenerate plane";
      return result;
    }
    tx /= tlen;
    ty /= tlen;
    tz /= tlen;
    const double vx = fny * tz - fnz * ty;
    const double vy = fnz * tx - fnx * tz;
    const double vz = fnx * ty - fny * tx;
    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      const double dx = double(positions[i * 3 + 0]) - cx;
      const double dy = double(positions[i * 3 + 1]) - cy;
      const double dz = double(positions[i * 3 + 2]) - cz;
      pts.push_back(Point_2(dx * tx + dy * ty + dz * tz, dx * vx + dy * vy + dz * vz));
    }
    using Traits = CGAL::Min_circle_2_traits_2<Kernel>;
    using Min_circle = CGAL::Min_circle_2<Traits>;
    Min_circle mc(pts.begin(), pts.end(), true);
    if (mc.is_empty()) {
      result.error = "Min Circle 3D is empty";
      return result;
    }
    const auto circ = mc.circle();
    const double ocx = CGAL::to_double(circ.center().x());
    const double ocy = CGAL::to_double(circ.center().y());
    const double r = std::sqrt(CGAL::to_double(circ.squared_radius()));
    const double wx = cx + ocx * tx + ocy * vx;
    const double wy = cy + ocx * ty + ocy * vy;
    const double wz = cz + ocx * tz + ocy * vz;
    result.center[0] = float(wx);
    result.center[1] = float(wy);
    result.center[2] = float(wz);
    result.radius = float(r);
    const Point_3 ctr = to_p3(result.center);
    push_p3(result.positions, ctr);
    for (int i = 0; i < segments; i++) {
      const double ang = (2.0 * 3.14159265358979323846 * double(i)) / double(segments);
      const double ca = std::cos(ang);
      const double sa = std::sin(ang);
      Point_3 rp = Point_3(FT(wx + r * (ca * tx + sa * vx)),
                           FT(wy + r * (ca * ty + sa * vy)),
                           FT(wz + r * (ca * tz + sa * vz)));
      push_p3(result.positions, rp);
    }
    for (int i = 0; i < segments; i++) {
      const int i1 = 1 + i;
      const int i2 = 1 + ((i + 1) % segments);
      result.corner_verts.push_back(0);
      result.corner_verts.push_back(i1);
      result.corner_verts.push_back(i2);
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Min Circle 3D produced no faces";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Min Circle 3D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
