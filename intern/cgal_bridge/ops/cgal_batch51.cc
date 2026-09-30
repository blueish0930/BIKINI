/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 51 (2026-08-21, new only — do not restore deleted ids):
 *  - Offset Mesh 3D (signed-distance isosurface of AABB distance)
 *  - CDT Hole Fill (triangulate_hole 2D CDT, no fair / no refine)
 *  - Bisector Surface (Voronoi faces of DT edges between two labeled sets)
 *  - Projected Outline (boolean union of projected triangles)
 *
 * Skipped: OpenGR, Kinetic EPECK, Mesh_3 volume, PolyFit SCIP, Ridges_3,
 * Classification, OSQP, Hyperbolic Delaunay, weighted Delaunay / Lloyd,
 * banned measure/query/remesh, previously deleted nodes.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Lazy_exact_nt.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Quotient.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>

#include <CGAL/assertions_behaviour.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/number_utils.h>

#include <boost/property_map/property_map.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point = Point_3;
using FT = Kernel::FT;
using Vector_3 = Kernel::Vector_3;
using Point_2 = Kernel::Point_2;

static Point to_p3(double x, double y, double z)
{
  return Point(FT(x), FT(y), FT(z));
}

static void push_xyz(std::vector<float> &pos, const Point &p)
{
  pos.push_back(float(CGAL::to_double(p.x())));
  pos.push_back(float(CGAL::to_double(p.y())));
  pos.push_back(float(CGAL::to_double(p.z())));
}

static int add_vert(MeshResult &r, const Point &p)
{
  const int i = r.verts_num();
  push_xyz(r.positions, p);
  return i;
}

static void add_tri(MeshResult &r, int a, int b, int c, int tag)
{
  if (a == b || b == c || c == a) {
    return;
  }
  r.corner_verts.push_back(a);
  r.corner_verts.push_back(b);
  r.corner_verts.push_back(c);
  r.face_tag.push_back(tag);
}

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

static Point lerp_iso(const Point &a, const Point &b, double sa, double sb, double iso)
{
  const double den = sb - sa;
  double t = (std::abs(den) < 1e-18) ? 0.5 : (iso - sa) / den;
  t = std::min(1.0 - 1e-6, std::max(1e-6, t));
  return Point(a.x() + FT(t) * (b.x() - a.x()),
               a.y() + FT(t) * (b.y() - a.y()),
               a.z() + FT(t) * (b.z() - a.z()));
}

static void emit_iso_tri(MeshResult &r, Point p0, Point p1, Point p2, const Point &toward)
{
  Vector_3 n = CGAL::cross_product(p1 - p0, p2 - p0);
  const Vector_3 to = toward - p0;
  if (CGAL::to_double(n * to) < 0.0) {
    std::swap(p1, p2);
  }
  add_tri(r, add_vert(r, p0), add_vert(r, p1), add_vert(r, p2), 0);
}

static void march_tet(MeshResult &r, const Point p[4], const double s[4], double iso)
{
  int above[4];
  int na = 0;
  for (int i = 0; i < 4; i++) {
    if (s[i] >= iso) {
      above[na++] = i;
    }
  }
  if (na == 0 || na == 4) {
    return;
  }
  if (na == 1 || na == 3) {
    int lone = above[0];
    if (na == 3) {
      bool used[4] = {false, false, false, false};
      for (int k = 0; k < 3; k++) {
        used[above[k]] = true;
      }
      for (int i = 0; i < 4; i++) {
        if (!used[i]) {
          lone = i;
          break;
        }
      }
    }
    Point q[3];
    int t = 0;
    for (int i = 0; i < 4; i++) {
      if (i != lone) {
        q[t++] = lerp_iso(p[lone], p[i], s[lone], s[i], iso);
      }
    }
    Point toward = p[lone];
    if (s[lone] < iso) {
      toward = Point(0, 0, 0);
      int c = 0;
      for (int i = 0; i < 4; i++) {
        if (s[i] >= iso) {
          toward = Point(toward.x() + p[i].x(), toward.y() + p[i].y(), toward.z() + p[i].z());
          c++;
        }
      }
      if (c > 0) {
        toward = Point(toward.x() / FT(c), toward.y() / FT(c), toward.z() / FT(c));
      }
    }
    emit_iso_tri(r, q[0], q[1], q[2], toward);
  }
  else {
    const int a = above[0];
    const int b = above[1];
    int outv[2];
    int no = 0;
    for (int i = 0; i < 4; i++) {
      if (i != a && i != b) {
        outv[no++] = i;
      }
    }
    const int c = outv[0];
    const int d = outv[1];
    const Point qac = lerp_iso(p[a], p[c], s[a], s[c], iso);
    const Point qad = lerp_iso(p[a], p[d], s[a], s[d], iso);
    const Point qbc = lerp_iso(p[b], p[c], s[b], s[c], iso);
    const Point qbd = lerp_iso(p[b], p[d], s[b], s[d], iso);
    const Point toward((p[a].x() + p[b].x()) / FT(2),
                       (p[a].y() + p[b].y()) / FT(2),
                       (p[a].z() + p[b].z()) / FT(2));
    emit_iso_tri(r, qac, qad, qbc, toward);
    emit_iso_tri(r, qad, qbd, qbc, toward);
  }
}

}  // namespace

MeshResult mesh_offset_sdf(const MeshIn &mesh, double offset, int resolution, bool signed_distance)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Offset Mesh 3D needs a triangle mesh" : error;
    return result;
  }
  resolution = std::clamp(resolution, 8, 64);
  try {
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
    using Tree = CGAL::AABB_tree<CGAL::AABB_traits_3<Kernel, Primitive>>;
    Tree tree(faces(sm).first, faces(sm).second, sm);
    tree.accelerate_distance_queries();

    const bool closed = CGAL::is_closed(sm);
    if (signed_distance && !closed) {
      result.error = "Signed Offset Mesh 3D needs a closed mesh (uncheck Signed for open shells)";
      return result;
    }

    double bmin[3] = {1e300, 1e300, 1e300};
    double bmax[3] = {-1e300, -1e300, -1e300};
    for (auto v : sm.vertices()) {
      const Point &p = sm.point(v);
      const double x = CGAL::to_double(p.x());
      const double y = CGAL::to_double(p.y());
      const double z = CGAL::to_double(p.z());
      bmin[0] = std::min(bmin[0], x);
      bmin[1] = std::min(bmin[1], y);
      bmin[2] = std::min(bmin[2], z);
      bmax[0] = std::max(bmax[0], x);
      bmax[1] = std::max(bmax[1], y);
      bmax[2] = std::max(bmax[2], z);
    }
    const double dx = std::max(1e-9, bmax[0] - bmin[0]);
    const double dy = std::max(1e-9, bmax[1] - bmin[1]);
    const double dz = std::max(1e-9, bmax[2] - bmin[2]);
    const double longest = std::max(dx, std::max(dy, dz));
    const double cell = longest / double(resolution);
    const double pad = std::abs(offset) + cell * 2.0;
    const double gmin[3] = {bmin[0] - pad, bmin[1] - pad, bmin[2] - pad};
    const double gmax[3] = {bmax[0] + pad, bmax[1] + pad, bmax[2] + pad};
    int nx = std::max(4, int(std::ceil((gmax[0] - gmin[0]) / cell)));
    int ny = std::max(4, int(std::ceil((gmax[1] - gmin[1]) / cell)));
    int nz = std::max(4, int(std::ceil((gmax[2] - gmin[2]) / cell)));
    nx = std::min(nx, 64);
    ny = std::min(ny, 64);
    nz = std::min(nz, 64);
    const int sx = nx + 1;
    const int sy = ny + 1;
    const int sz = nz + 1;
    const double hx = (gmax[0] - gmin[0]) / double(nx);
    const double hy = (gmax[1] - gmin[1]) / double(ny);
    const double hz = (gmax[2] - gmin[2]) / double(nz);

    std::unique_ptr<CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel>> side;
    if (signed_distance) {
      side = std::make_unique<CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel>>(sm);
    }

    std::vector<double> values(size_t(sx * sy * sz));
    std::vector<Point> pts(size_t(sx * sy * sz));
    auto at = [sx, sy](int i, int j, int k) -> int {
      return (k * sy + j) * sx + i;
    };
    for (int k = 0; k < sz; k++) {
      for (int j = 0; j < sy; j++) {
        for (int i = 0; i < sx; i++) {
          const Point p = to_p3(gmin[0] + double(i) * hx,
                                gmin[1] + double(j) * hy,
                                gmin[2] + double(k) * hz);
          const int id = at(i, j, k);
          pts[size_t(id)] = p;
          const double dist = std::sqrt(CGAL::to_double(tree.squared_distance(p)));
          if (signed_distance) {
            const auto loc = (*side)(p);
            double sgn = 1.0;
            if (loc == CGAL::ON_BOUNDED_SIDE) {
              sgn = -1.0;
            }
            else if (loc == CGAL::ON_BOUNDARY) {
              sgn = 0.0;
            }
            values[size_t(id)] = sgn * dist;
          }
          else {
            values[size_t(id)] = dist;
          }
        }
      }
    }

    const double iso = signed_distance ? offset : std::abs(offset);
    /* Freudenthal 6-tet split of each cube along the 000–111 diagonal. */
    const int tet[6][4] = {
        {0, 1, 3, 7},
        {0, 1, 5, 7},
        {0, 2, 3, 7},
        {0, 2, 6, 7},
        {0, 4, 5, 7},
        {0, 4, 6, 7},
    };
    const int corner[8][3] = {
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {1, 1, 0},
        {0, 0, 1},
        {1, 0, 1},
        {0, 1, 1},
        {1, 1, 1},
    };
    for (int k = 0; k < nz; k++) {
      for (int j = 0; j < ny; j++) {
        for (int i = 0; i < nx; i++) {
          int cid[8];
          Point cp[8];
          double cv[8];
          for (int c = 0; c < 8; c++) {
            cid[c] = at(i + corner[c][0], j + corner[c][1], k + corner[c][2]);
            cp[c] = pts[size_t(cid[c])];
            cv[c] = values[size_t(cid[c])];
          }
          for (int t = 0; t < 6; t++) {
            Point p[4];
            double s[4];
            for (int v = 0; v < 4; v++) {
              p[v] = cp[tet[t][v]];
              s[v] = cv[tet[t][v]];
            }
            march_tet(result, p, s, iso);
          }
        }
      }
    }
    result.ok = result.corners_num() >= 3;
    if (!result.ok) {
      result.error = "Offset Mesh 3D: no isosurface at this offset/resolution";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Offset Mesh 3D failed";
  }
  return result;
}

MeshResult mesh_cdt_hole_fill(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true) || sm.number_of_faces() == 0) {
    result.error = error.empty() ? "CDT Hole Fill input failed" : error;
    return result;
  }
  try {
    if (!CGAL::is_triangle_mesh(sm)) {
      triangulate_faces_keep_ids(sm);
    }
    std::vector<Surface_mesh::Halfedge_index> border_loops;
    CGAL::extract_boundary_cycles(sm, std::back_inserter(border_loops));
    int filled = 0;
    for (const auto h : border_loops) {
      if (!CGAL::is_valid_halfedge_descriptor(h, sm) || !sm.is_border(h)) {
        continue;
      }
      PMP::triangulate_hole(sm,
                            h,
                            CGAL::parameters::use_2d_constrained_delaunay_triangulation(true)
                                .use_delaunay_triangulation(true)
                                .do_not_use_cubic_algorithm(true));
      filled++;
    }
    if (filled == 0) {
      result.error = "CDT Hole Fill found no boundary loops";
      return result;
    }
    fill_missing_vert_maps_from_edges(sm);
    detriangulate_by_original_face(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CDT Hole Fill failed";
  }
  return result;
}

MeshResult points_bisector_surface(const float *a_xyz, int a_n, const float *b_xyz, int b_n)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!a_xyz || !b_xyz || a_n < 1 || b_n < 1) {
    result.error = "Bisector Surface needs points on both sides";
    return result;
  }
  try {
    using DT = CGAL::Delaunay_triangulation_3<Kernel>;
    DT dt;
    std::map<typename DT::Vertex_handle, int> label;
    auto insert_cloud = [&](const float *xyz, int n, int lab) {
      for (int i = 0; i < n; i++) {
        typename DT::Vertex_handle vh = dt.insert(to_p3(double(xyz[i * 3 + 0]),
                                                       double(xyz[i * 3 + 1]),
                                                       double(xyz[i * 3 + 2])));
        label[vh] = lab;
      }
    };
    insert_cloud(a_xyz, a_n, 0);
    insert_cloud(b_xyz, b_n, 1);
    if (dt.dimension() != 3) {
      result.error = "Bisector Surface: Delaunay tetrahedralization failed";
      return result;
    }

    if (result.face_offsets.empty()) {
      result.face_offsets.push_back(0);
    }
    for (auto eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
      typename DT::Edge edge = *eit;
      typename DT::Vertex_handle va = edge.first->vertex(edge.second);
      typename DT::Vertex_handle vb = edge.first->vertex(edge.third);
      auto ia = label.find(va);
      auto ib = label.find(vb);
      if (ia == label.end() || ib == label.end() || ia->second == ib->second) {
        continue;
      }
      typename DT::Cell_circulator circ = dt.incident_cells(edge);
      typename DT::Cell_circulator start = circ;
      std::vector<Point> duals;
      bool unbounded = false;
      do {
        if (dt.is_infinite(circ)) {
          unbounded = true;
          break;
        }
        duals.push_back(dt.dual(circ));
        ++circ;
      } while (circ != start);
      if (unbounded || duals.size() < 3) {
        continue;
      }
      Vector_3 axis = vb->point() - va->point();
      const double alen = std::sqrt(CGAL::to_double(axis.squared_length()));
      if (alen < 1e-18) {
        continue;
      }
      axis = axis / FT(alen);
      Vector_3 tmp(FT(1), FT(0), FT(0));
      if (std::abs(CGAL::to_double(axis * tmp)) > 0.9) {
        tmp = Vector_3(FT(0), FT(1), FT(0));
      }
      Vector_3 u = CGAL::cross_product(axis, tmp);
      const double ulen = std::sqrt(CGAL::to_double(u.squared_length()));
      if (ulen < 1e-18) {
        continue;
      }
      u = u / FT(ulen);
      const Vector_3 vv = CGAL::cross_product(axis, u);
      const Point origin = duals[0];
      std::vector<std::pair<double, int>> order;
      order.resize(duals.size());
      for (int i = 0; i < int(duals.size()); i++) {
        const Vector_3 d = duals[size_t(i)] - origin;
        const double ang = std::atan2(CGAL::to_double(d * vv), CGAL::to_double(d * u));
        order[size_t(i)] = {ang, i};
      }
      std::sort(order.begin(), order.end(), [](const auto &p, const auto &q) {
        return p.first < q.first;
      });
      const int base = result.verts_num();
      int emitted = 0;
      Point last = to_p3(1e300, 1e300, 1e300);
      for (const auto &item : order) {
        const Point &p = duals[size_t(item.second)];
        if (emitted > 0 && CGAL::squared_distance(p, last) < FT(1e-20)) {
          continue;
        }
        push_xyz(result.positions, p);
        last = p;
        emitted++;
      }
      if (emitted < 3) {
        result.positions.resize(size_t(base) * 3);
        continue;
      }
      for (int i = 0; i < emitted; i++) {
        result.corner_verts.push_back(base + i);
      }
      result.face_offsets.push_back(int(result.corner_verts.size()));
      result.face_tag.push_back(0);
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Bisector Surface: no finite mixed Voronoi faces";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Bisector Surface failed";
  }
  return result;
}

MeshResult mesh_projected_outline(const MeshIn &mesh, float dx, float dy, float dz)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!mesh.positions || mesh.faces_num <= 0 || !mesh.corner_verts) {
    result.error = "Projected Outline needs a mesh with faces";
    return result;
  }
  try {
    const double ndx = double(dx);
    const double ndy = double(dy);
    const double ndz = double(dz);
    Vector_3 n = Vector_3(FT(ndx), FT(ndy), FT(ndz));
    const double nlen = std::sqrt(CGAL::to_double(n.squared_length()));
    if (nlen < 1e-12) {
      n = Vector_3(FT(0), FT(0), FT(1));
    }
    else {
      n = n / FT(nlen);
    }
    Vector_3 tmp(FT(0), FT(0), FT(1));
    if (std::abs(CGAL::to_double(n * tmp)) > 0.9) {
      tmp = Vector_3(FT(1), FT(0), FT(0));
    }
    Vector_3 u = CGAL::cross_product(n, tmp);
    const double ulen = std::sqrt(CGAL::to_double(u.squared_length()));
    u = u / FT(ulen);
    const Vector_3 vv = CGAL::cross_product(n, u);

    double cx = 0.0, cy = 0.0, cz = 0.0;
    for (int i = 0; i < mesh.verts_num; i++) {
      cx += double(mesh.positions[i * 3 + 0]);
      cy += double(mesh.positions[i * 3 + 1]);
      cz += double(mesh.positions[i * 3 + 2]);
    }
    if (mesh.verts_num > 0) {
      cx /= double(mesh.verts_num);
      cy /= double(mesh.verts_num);
      cz /= double(mesh.verts_num);
    }
    const Point origin = to_p3(cx, cy, cz);

    auto project = [&](const float *xyz) -> Point_2 {
      const double px = double(xyz[0]) - cx;
      const double py = double(xyz[1]) - cy;
      const double pz = double(xyz[2]) - cz;
      const Vector_3 d = Vector_3(FT(px), FT(py), FT(pz));
      return Point_2(d * u, d * vv);
    };
    auto unproject = [&](const Point_2 &p) -> Point {
      return origin + u * p.x() + vv * p.y();
    };

    using EFT = CGAL::Lazy_exact_nt<CGAL::Quotient<CGAL::MP_Float>>;
    using EKernel = CGAL::Simple_cartesian<EFT>;
    using EPoint_2 = EKernel::Point_2;
    using EPolygon_2 = CGAL::Polygon_2<EKernel>;
    using EPolygon_with_holes_2 = CGAL::Polygon_with_holes_2<EKernel>;
    using EPolygon_set_2 = CGAL::Polygon_set_2<EKernel>;

    const CGAL::Failure_behaviour old_err = CGAL::set_error_behaviour(CGAL::THROW_EXCEPTION);
    const CGAL::Failure_behaviour old_warn = CGAL::set_warning_behaviour(CGAL::CONTINUE);
    auto restore = [&]() {
      CGAL::set_error_behaviour(old_err);
      CGAL::set_warning_behaviour(old_warn);
    };

    EPolygon_set_2 ps;
    int joined = 0;
    auto face_corners = [&](int f, int &start, int &end) {
      if (mesh.face_offsets) {
        start = mesh.face_offsets[f];
        end = mesh.face_offsets[f + 1];
      }
      else {
        start = f * 3;
        end = start + 3;
      }
    };
    for (int f = 0; f < mesh.faces_num; f++) {
      int start = 0, end = 0;
      face_corners(f, start, end);
      if (end - start < 3) {
        continue;
      }
      std::vector<EPoint_2> raw;
      raw.reserve(size_t(end - start));
      bool bad = false;
      for (int c = start; c < end; c++) {
        const int v = mesh.corner_verts[c];
        if (v < 0 || v >= mesh.verts_num) {
          bad = true;
          break;
        }
        const Point_2 q = project(mesh.positions + v * 3);
        raw.emplace_back(EFT(CGAL::to_double(q.x())), EFT(CGAL::to_double(q.y())));
      }
      if (bad || raw.size() < 3) {
        continue;
      }
      try {
        EPolygon_2 poly(raw.begin(), raw.end());
        if (poly.is_clockwise_oriented()) {
          poly.reverse_orientation();
        }
        if (poly.is_simple() && poly.size() >= 3) {
          ps.join(poly);
          joined++;
        }
      }
      catch (...) {
        continue;
      }
    }
    if (joined == 0) {
      restore();
      result.error = "Projected Outline: no simple projected faces";
      return result;
    }
    std::vector<EPolygon_with_holes_2> outlines;
    ps.polygons_with_holes(std::back_inserter(outlines));
    restore();
    if (outlines.empty()) {
      result.error = "Projected Outline: union was empty";
      return result;
    }

    using Vb = CGAL::Triangulation_vertex_base_2<Kernel>;
    using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
    using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, CGAL::Exact_predicates_tag>;

    auto to_epick = [](const EPoint_2 &p) {
      return Point_2(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
    };

    int island = 0;
    for (const EPolygon_with_holes_2 &pwh : outlines) {
      CDT cdt;
      auto insert_ring = [&](const EPolygon_2 &poly) {
        if (poly.size() < 3) {
          return;
        }
        std::vector<typename CDT::Vertex_handle> vhs;
        for (auto vit = poly.vertices_begin(); vit != poly.vertices_end(); ++vit) {
          try {
            vhs.push_back(cdt.insert(to_epick(*vit)));
          }
          catch (...) {
            return;
          }
        }
        if (vhs.size() >= 2 && vhs.front() == vhs.back()) {
          vhs.pop_back();
        }
        const int n = int(vhs.size());
        if (n < 3) {
          return;
        }
        for (int i = 0; i < n; i++) {
          if (vhs[size_t(i)] != vhs[size_t((i + 1) % n)]) {
            try {
              cdt.insert_constraint(vhs[size_t(i)], vhs[size_t((i + 1) % n)]);
            }
            catch (...) {
            }
          }
        }
      };
      insert_ring(pwh.outer_boundary());
      for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        insert_ring(*hit);
      }
      if (cdt.dimension() != 2) {
        continue;
      }
      std::map<typename CDT::Face_handle, bool> in_domain_map;
      boost::associative_property_map<std::map<typename CDT::Face_handle, bool>> in_domain(
          in_domain_map);
      CGAL::mark_domain_in_triangulation(cdt, in_domain);
      std::map<typename CDT::Vertex_handle, int> v2i;
      auto vert_index = [&](typename CDT::Vertex_handle vh) -> int {
        auto it = v2i.find(vh);
        if (it != v2i.end()) {
          return it->second;
        }
        const int idx = add_vert(result, unproject(vh->point()));
        v2i[vh] = idx;
        return idx;
      };
      for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
        if (!in_domain_map[fit]) {
          continue;
        }
        add_tri(result,
                vert_index(fit->vertex(0)),
                vert_index(fit->vertex(1)),
                vert_index(fit->vertex(2)),
                island);
      }
      island++;
    }
    result.ok = result.corners_num() >= 3;
    if (!result.ok) {
      result.error = "Projected Outline: no filled cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Projected Outline failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
