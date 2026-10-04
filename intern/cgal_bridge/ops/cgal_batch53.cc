/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 53 (2026-08-21, new only — do not restore deleted ids):
 *  - Interior Tets (DT tets whose circumcenter is inside a closed mesh)
 *  - Surface Delaunay Graph (tangent-plane DT 1-ring, not Euclidean KNN)
 *  - Split Charts (cut along sharp dihedrals into disconnected islands)
 *  - Restricted Voronoi (3D Voronoi cells clipped to the surface mesh)
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

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/assertions_behaviour.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/property_map.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <set>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using FT = Kernel::FT;
using DT3 = CGAL::Delaunay_triangulation_3<Kernel>;
using Vb2 = CGAL::Triangulation_vertex_base_with_info_2<int, Kernel>;
using Fb2 = CGAL::Triangulation_face_base_2<Kernel>;
using Tds2 = CGAL::Triangulation_data_structure_2<Vb2, Fb2>;
using DT2 = CGAL::Delaunay_triangulation_2<Kernel, Tds2>;

static Point_3 mk_p3(double x, double y, double z)
{
  const FT fx(x);
  const FT fy(y);
  const FT fz(z);
  return Point_3(fx, fy, fz);
}

static Point_2 mk_p2(double x, double y)
{
  const FT fx(x);
  const FT fy(y);
  return Point_2(fx, fy);
}

static void push_p3(std::vector<float> &pos, const Point_3 &p)
{
  pos.push_back(float(CGAL::to_double(p.x())));
  pos.push_back(float(CGAL::to_double(p.y())));
  pos.push_back(float(CGAL::to_double(p.z())));
}

using DVert = std::array<double, 3>;

static void clip_poly_halfspace(std::vector<DVert> &poly,
                                double nx,
                                double ny,
                                double nz,
                                double dkeep)
{
  const size_t n = poly.size();
  if (n < 3) {
    poly.clear();
    return;
  }
  auto side = [&](const DVert &p) { return nx * p[0] + ny * p[1] + nz * p[2] - dkeep; };
  std::vector<DVert> out;
  out.reserve(n + 2);
  for (size_t i = 0; i < n; i++) {
    const DVert &a = poly[i];
    const DVert &b = poly[(i + 1) % n];
    const double sa = side(a);
    const double sb = side(b);
    const bool ina = sa <= 1e-12;
    const bool inb = sb <= 1e-12;
    if (ina) {
      out.push_back(a);
    }
    if (ina != inb) {
      const double den = sa - sb;
      double t = (std::abs(den) < 1e-30) ? 0.5 : (sa / den);
      if (t < 0.0) {
        t = 0.0;
      }
      if (t > 1.0) {
        t = 1.0;
      }
      DVert h;
      h[0] = a[0] + t * (b[0] - a[0]);
      h[1] = a[1] + t * (b[1] - a[1]);
      h[2] = a[2] + t * (b[2] - a[2]);
      out.push_back(h);
    }
  }
  poly.swap(out);
}

static void emit_ngon(MeshResult &result, const std::vector<DVert> &poly, int tag)
{
  if (poly.size() < 3) {
    return;
  }
  const int vbase = result.verts_num();
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  for (size_t i = 0; i < poly.size(); i++) {
    result.positions.push_back(float(poly[i][0]));
    result.positions.push_back(float(poly[i][1]));
    result.positions.push_back(float(poly[i][2]));
    result.corner_verts.push_back(vbase + int(i));
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
}

}  // namespace

MeshResult mesh_interior_tets(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    if (!mesh.positions || mesh.verts_num < 4 || mesh.faces_num < 4) {
      result.error = "Interior Tets needs a closed mesh with ≥4 verts and faces";
      return result;
    }
    Surface_mesh sm;
    std::string io_err;
    if (!mesh_in_to_surface_mesh(mesh, sm, io_err, false) || sm.number_of_faces() < 4) {
      result.error = io_err.empty() ? "Interior Tets: failed to import mesh" : io_err;
      return result;
    }
    if (!CGAL::is_closed(sm)) {
      result.error = "Interior Tets needs a closed mesh";
      return result;
    }
    using Inside = CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel>;
    Inside inside(sm);
    DT3 dt;
    for (int i = 0; i < mesh.verts_num; i++) {
      const float *p = mesh.positions + i * 3;
      dt.insert(mk_p3(double(p[0]), double(p[1]), double(p[2])));
    }
    if (dt.number_of_finite_cells() < 1) {
      result.error = "Interior Tets: Delaunay produced no tetrahedra";
      return result;
    }
    for (DT3::Finite_cells_iterator cit = dt.finite_cells_begin(); cit != dt.finite_cells_end();
         ++cit)
    {
      const Point_3 c = dt.dual(cit);
      if (inside(c) != CGAL::ON_BOUNDED_SIDE) {
        continue;
      }
      const int vbase = result.verts_num();
      for (int k = 0; k < 4; k++) {
        push_p3(result.positions, cit->vertex(k)->point());
      }
      const int faces[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
      for (int f = 0; f < 4; f++) {
        result.corner_verts.push_back(vbase + faces[f][0]);
        result.corner_verts.push_back(vbase + faces[f][1]);
        result.corner_verts.push_back(vbase + faces[f][2]);
      }
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Interior Tets: no tetrahedron has its circumcenter inside";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Interior Tets failed";
  }
  return result;
}

WireResult points_surface_delaunay_graph(const float *positions, int n, int knn)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!positions || n < 4) {
    result.error = "Surface Delaunay Graph needs at least 4 points";
    return result;
  }
  knn = std::max(6, std::min(knn, n - 1));
  try {
    using Pair = std::pair<Point_3, int>;
    using Traits_base = CGAL::Search_traits_3<Kernel>;
    using Traits = CGAL::Search_traits_adapter<Pair,
                                               CGAL::First_of_pair_property_map<Pair>,
                                               Traits_base>;
    using KSearch = CGAL::Orthogonal_k_neighbor_search<Traits>;
    using Tree = typename KSearch::Tree;

    std::vector<Pair> items;
    items.reserve(size_t(n));
    result.positions.assign(positions, positions + n * 3);
    for (int i = 0; i < n; i++) {
      items.emplace_back(mk_p3(double(positions[i * 3 + 0]),
                               double(positions[i * 3 + 1]),
                               double(positions[i * 3 + 2])),
                         i);
    }
    Tree tree(items.begin(), items.end());
    std::set<std::pair<int, int>> seen;
    for (int i = 0; i < n; i++) {
      KSearch search(tree, items[size_t(i)].first, unsigned(knn + 1));
      std::vector<int> nbs;
      nbs.reserve(size_t(knn));
      double cx = 0.0, cy = 0.0, cz = 0.0;
      int count = 0;
      for (auto it = search.begin(); it != search.end(); ++it) {
        const int j = it->first.second;
        nbs.push_back(j);
        cx += double(positions[j * 3 + 0]);
        cy += double(positions[j * 3 + 1]);
        cz += double(positions[j * 3 + 2]);
        count++;
      }
      if (count < 4) {
        continue;
      }
      cx /= double(count);
      cy /= double(count);
      cz /= double(count);
      double xx = 0, cov_xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
      for (int j : nbs) {
        const double dx = double(positions[j * 3 + 0]) - cx;
        const double dy = double(positions[j * 3 + 1]) - cy;
        const double dz = double(positions[j * 3 + 2]) - cz;
        xx += dx * dx;
        cov_xy += dx * dy;
        xz += dx * dz;
        yy += dy * dy;
        yz += dy * dz;
        zz += dz * dz;
      }
      const double ev[3][3] = {{xx, cov_xy, xz}, {cov_xy, yy, yz}, {xz, yz, zz}};
      double best = -1.0;
      double nx = 0.0, ny = 0.0, nz = 1.0;
      for (int a = 0; a < 3; a++) {
        const int b = (a + 1) % 3;
        const double cxv = ev[a][1] * ev[b][2] - ev[a][2] * ev[b][1];
        const double cyv = ev[a][2] * ev[b][0] - ev[a][0] * ev[b][2];
        const double czv = ev[a][0] * ev[b][1] - ev[a][1] * ev[b][0];
        const double area = cxv * cxv + cyv * cyv + czv * czv;
        if (area > best) {
          best = area;
          nx = cxv;
          ny = cyv;
          nz = czv;
        }
      }
      double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (nlen < 1e-20) {
        nx = 0.0;
        ny = 0.0;
        nz = 1.0;
        nlen = 1.0;
      }
      nx /= nlen;
      ny /= nlen;
      nz /= nlen;
      double ux = 0.0, uy = 1.0, uz = 0.0;
      if (std::abs(ny) > 0.9) {
        ux = 1.0;
        uy = 0.0;
        uz = 0.0;
      }
      double tx = ny * uz - nz * uy;
      double ty = nz * ux - nx * uz;
      double tz = nx * uy - ny * ux;
      const double tlen = std::sqrt(tx * tx + ty * ty + tz * tz);
      if (tlen < 1e-20) {
        continue;
      }
      tx /= tlen;
      ty /= tlen;
      tz /= tlen;
      const double vx = ny * tz - nz * ty;
      const double vy = nz * tx - nx * tz;
      const double vz = nx * ty - ny * tx;
      DT2 dt2;
      DT2::Vertex_handle self_vh = DT2::Vertex_handle();
      bool have_self = false;
      for (int j : nbs) {
        const double dx = double(positions[j * 3 + 0]) - cx;
        const double dy = double(positions[j * 3 + 1]) - cy;
        const double dz = double(positions[j * 3 + 2]) - cz;
        DT2::Vertex_handle vh = dt2.insert(
            mk_p2(dx * tx + dy * ty + dz * tz, dx * vx + dy * vy + dz * vz));
        vh->info() = j;
        if (j == i) {
          self_vh = vh;
          have_self = true;
        }
      }
      if (!have_self || dt2.dimension() < 2) {
        continue;
      }
      DT2::Vertex_circulator circ = dt2.incident_vertices(self_vh);
      DT2::Vertex_circulator done = circ;
      do {
        if (!dt2.is_infinite(circ)) {
          const int j = circ->info();
          if (j != i) {
            const int a = std::min(i, j);
            const int b = std::max(i, j);
            if (seen.insert({a, b}).second) {
              result.edge_v0.push_back(a);
              result.edge_v1.push_back(b);
            }
          }
        }
        ++circ;
      } while (circ != done);
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Surface Delaunay Graph produced no edges (increase K)";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Surface Delaunay Graph failed";
  }
  return result;
}

MeshResult mesh_split_charts(const MeshIn &mesh, double angle_deg)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    if (!mesh.positions || mesh.faces_num < 1) {
      result.error = "Split Charts needs a mesh with faces";
      return result;
    }
    Surface_mesh sm;
    std::string io_err;
    if (!mesh_in_to_surface_mesh(mesh, sm, io_err, false) || sm.number_of_faces() < 1) {
      result.error = io_err.empty() ? "Split Charts: failed to import mesh" : io_err;
      return result;
    }
    angle_deg = std::max(0.0, std::min(angle_deg, 180.0));
    auto ecm = sm.add_property_map<Surface_mesh::Edge_index, bool>("e:chart_cut", false).first;
    PMP::detect_sharp_edges(sm, angle_deg, ecm);
    std::vector<Surface_mesh> parts;
    PMP::split_connected_components(sm, parts, CGAL::parameters::edge_is_constrained_map(ecm));
    if (parts.empty()) {
      result.error = "Split Charts produced no charts";
      return result;
    }
    Surface_mesh out;
    for (const Surface_mesh &part : parts) {
      CGAL::copy_face_graph(part, out);
    }
    result = surface_mesh_to_result(out);
    if (!result.ok) {
      result.error = result.error.empty() ? "Split Charts: export failed" : result.error;
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Split Charts failed";
  }
  return result;
}

MeshResult mesh_restricted_voronoi(const MeshIn &mesh, const float *sites, int n_sites)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!mesh.positions || mesh.faces_num < 1) {
    result.error = "Restricted Voronoi needs a mesh with faces";
    return result;
  }
  if (!sites || n_sites < 1) {
    result.error = "Restricted Voronoi needs at least 1 site";
    return result;
  }
  try {
    using Pair = std::pair<Point_3, int>;
    using Traits_base = CGAL::Search_traits_3<Kernel>;
    using Traits = CGAL::Search_traits_adapter<Pair,
                                               CGAL::First_of_pair_property_map<Pair>,
                                               Traits_base>;
    using KSearch = CGAL::Orthogonal_k_neighbor_search<Traits>;
    using Tree = typename KSearch::Tree;

    std::vector<Pair> items;
    items.reserve(size_t(n_sites));
    for (int i = 0; i < n_sites; i++) {
      items.emplace_back(mk_p3(double(sites[i * 3 + 0]),
                               double(sites[i * 3 + 1]),
                               double(sites[i * 3 + 2])),
                         i);
    }
    Tree tree(items.begin(), items.end());
    const int knn = std::min(n_sites, 16);

    auto nearest_ids = [&](double x, double y, double z, std::set<int> &ids) {
      KSearch search(tree, mk_p3(x, y, z), unsigned(knn));
      for (auto it = search.begin(); it != search.end(); ++it) {
        ids.insert(it->first.second);
      }
    };

    const int *offsets = mesh.face_offsets;
    auto face_begin = [&](int f) -> int {
      if (offsets) {
        return offsets[f];
      }
      return f * 3;
    };
    auto face_end = [&](int f) -> int {
      if (offsets) {
        return offsets[f + 1];
      }
      return f * 3 + 3;
    };

    for (int f = 0; f < mesh.faces_num; f++) {
      const int b = face_begin(f);
      const int e = face_end(f);
      if (e - b < 3) {
        continue;
      }
      for (int t = b + 1; t + 1 < e; t++) {
        const int ia = mesh.corner_verts[b];
        const int ib = mesh.corner_verts[t];
        const int ic = mesh.corner_verts[t + 1];
        if (ia < 0 || ib < 0 || ic < 0 || ia >= mesh.verts_num || ib >= mesh.verts_num ||
            ic >= mesh.verts_num)
        {
          continue;
        }
        const DVert pa{double(mesh.positions[ia * 3 + 0]),
                       double(mesh.positions[ia * 3 + 1]),
                       double(mesh.positions[ia * 3 + 2])};
        const DVert pb{double(mesh.positions[ib * 3 + 0]),
                       double(mesh.positions[ib * 3 + 1]),
                       double(mesh.positions[ib * 3 + 2])};
        const DVert pc{double(mesh.positions[ic * 3 + 0]),
                       double(mesh.positions[ic * 3 + 1]),
                       double(mesh.positions[ic * 3 + 2])};
        const double cx = (pa[0] + pb[0] + pc[0]) / 3.0;
        const double cy = (pa[1] + pb[1] + pc[1]) / 3.0;
        const double cz = (pa[2] + pb[2] + pc[2]) / 3.0;
        std::set<int> cand;
        nearest_ids(pa[0], pa[1], pa[2], cand);
        nearest_ids(pb[0], pb[1], pb[2], cand);
        nearest_ids(pc[0], pc[1], pc[2], cand);
        nearest_ids(cx, cy, cz, cand);
        if (cand.empty()) {
          continue;
        }
        std::vector<int> ids(cand.begin(), cand.end());
        if (n_sites <= 24) {
          ids.clear();
          ids.reserve(size_t(n_sites));
          for (int s = 0; s < n_sites; s++) {
            ids.push_back(s);
          }
        }
        for (int s : ids) {
          std::vector<DVert> poly = {pa, pb, pc};
          const double sx = double(sites[s * 3 + 0]);
          const double sy = double(sites[s * 3 + 1]);
          const double sz = double(sites[s * 3 + 2]);
          for (int tsite : ids) {
            if (tsite == s) {
              continue;
            }
            const double tx = double(sites[tsite * 3 + 0]);
            const double ty = double(sites[tsite * 3 + 1]);
            const double tz = double(sites[tsite * 3 + 2]);
            const double nx = tx - sx;
            const double ny = ty - sy;
            const double nz = tz - sz;
            const double n2 = nx * nx + ny * ny + nz * nz;
            if (n2 < 1e-24) {
              continue;
            }
            const double mx = 0.5 * (sx + tx);
            const double my = 0.5 * (sy + ty);
            const double mz = 0.5 * (sz + tz);
            const double dkeep = nx * mx + ny * my + nz * mz;
            clip_poly_halfspace(poly, nx, ny, nz, dkeep);
            if (poly.size() < 3) {
              break;
            }
          }
          if (poly.size() >= 3) {
            emit_ngon(result, poly, s);
          }
        }
      }
    }
    /* emit_ngon uses &p - poly.data() which is correct; fix corner indices if needed. */
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Restricted Voronoi produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Restricted Voronoi failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
