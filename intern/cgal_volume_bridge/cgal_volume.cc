/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Heavy tetrahedral helpers isolated from bf_intern_cgal_bridge.
 *
 * Tet remesh / CDT3: interior Delaunay tets with Steiner points
 * (Mesh_3 / CDT3 6.1 skipped: 65535-export / missing).
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Triangulation_data_structure_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace blender::cgal_bridge {
namespace {

using Point_3 = Kernel::Point_3;
using FT = Kernel::FT;
using VI = Surface_mesh::Vertex_index;
using DT3 = CGAL::Delaunay_triangulation_3<Kernel>;

static Point_3 mk_p3(double x, double y, double z)
{
  return Point_3(FT(x), FT(y), FT(z));
}

static void push_p3(std::vector<float> &pos, const Point_3 &p)
{
  pos.push_back(float(CGAL::to_double(p.x())));
  pos.push_back(float(CGAL::to_double(p.y())));
  pos.push_back(float(CGAL::to_double(p.z())));
}

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  sm.collect_garbage();
  return sm.number_of_faces() > 0;
}

static Point_3 tet_shrink_vert(const Point_3 &p, double gx, double gy, double gz, double shrink)
{
  return mk_p3(gx + shrink * (CGAL::to_double(p.x()) - gx),
               gy + shrink * (CGAL::to_double(p.y()) - gy),
               gz + shrink * (CGAL::to_double(p.z()) - gz));
}

/**
 * Emit one tetrahedron as 4 triangles with duplicated verts (an isolated island).
 * shrink < 1 pulls vertices toward the tet centroid so neighboring tets no longer
 * share coincident faces — otherwise a tet filling looks like one closed surface.
 * Vertex order a,b,c,d is preserved so vert_src0 stays aligned.
 */
static void append_tet(MeshResult &result,
                       Point_3 a,
                       Point_3 b,
                       Point_3 c,
                       Point_3 d,
                       int tag,
                       double shrink)
{
  if (shrink < 0.999) {
    const double gx = 0.25 * (CGAL::to_double(a.x()) + CGAL::to_double(b.x()) +
                              CGAL::to_double(c.x()) + CGAL::to_double(d.x()));
    const double gy = 0.25 * (CGAL::to_double(a.y()) + CGAL::to_double(b.y()) +
                              CGAL::to_double(c.y()) + CGAL::to_double(d.y()));
    const double gz = 0.25 * (CGAL::to_double(a.z()) + CGAL::to_double(b.z()) +
                              CGAL::to_double(c.z()) + CGAL::to_double(d.z()));
    a = tet_shrink_vert(a, gx, gy, gz, shrink);
    b = tet_shrink_vert(b, gx, gy, gz, shrink);
    c = tet_shrink_vert(c, gx, gy, gz, shrink);
    d = tet_shrink_vert(d, gx, gy, gz, shrink);
  }
  const Point_3 P[4] = {a, b, c, d};
  const int vbase = result.verts_num();
  for (int i = 0; i < 4; i++) {
    push_p3(result.positions, P[i]);
  }
  int faces[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  const int opp[4] = {3, 2, 1, 0};
  for (int f = 0; f < 4; f++) {
    const Point_3 &pa = P[faces[f][0]];
    const Point_3 &pb = P[faces[f][1]];
    const Point_3 &pc = P[faces[f][2]];
    const Point_3 &po = P[opp[f]];
    const double abx = CGAL::to_double(pb.x()) - CGAL::to_double(pa.x());
    const double aby = CGAL::to_double(pb.y()) - CGAL::to_double(pa.y());
    const double abz = CGAL::to_double(pb.z()) - CGAL::to_double(pa.z());
    const double acx = CGAL::to_double(pc.x()) - CGAL::to_double(pa.x());
    const double acy = CGAL::to_double(pc.y()) - CGAL::to_double(pa.y());
    const double acz = CGAL::to_double(pc.z()) - CGAL::to_double(pa.z());
    const double nx = aby * acz - abz * acy;
    const double ny = abz * acx - abx * acz;
    const double nz = abx * acy - aby * acx;
    const double ox = CGAL::to_double(po.x()) - CGAL::to_double(pa.x());
    const double oy = CGAL::to_double(po.y()) - CGAL::to_double(pa.y());
    const double oz = CGAL::to_double(po.z()) - CGAL::to_double(pa.z());
    /* Outward: flip if the opposite vertex is on the positive side of the face. */
    if (nx * ox + ny * oy + nz * oz > 0.0) {
      std::swap(faces[f][1], faces[f][2]);
    }
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  for (int f = 0; f < 4; f++) {
    result.corner_verts.push_back(vbase + faces[f][0]);
    result.corner_verts.push_back(vbase + faces[f][1]);
    result.corner_verts.push_back(vbase + faces[f][2]);
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_tag.push_back(tag);
  }
}

/* mode: 1 denser remesh, 3 vertices only. */
static MeshResult interior_tets_with_steiner(const MeshIn &mesh, double spacing, int mode)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error) || !CGAL::is_closed(sm)) {
    result.error = error.empty() ? "Needs a closed triangle mesh" : error;
    return result;
  }
  CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel> inside(sm);
  DT3 dt;
  CGAL::Bbox_3 bb;
  for (const VI v : sm.vertices()) {
    dt.insert(sm.point(v));
    bb += sm.point(v).bbox();
  }
  const double dx = std::max(bb.xmax() - bb.xmin(), 1e-6);
  const double dy = std::max(bb.ymax() - bb.ymin(), 1e-6);
  const double dz = std::max(bb.zmax() - bb.zmin(), 1e-6);
  double h = spacing;
  if (!(h > 0)) {
    h = ((mode == 1) ? 0.04 : 0.08) * std::max({dx, dy, dz});
  }
  h = std::max(h, 1e-6);
  const int cap = (mode == 1) ? 36 : 24;
  const int nx = std::max(1, std::min(cap, int(std::ceil(dx / h))));
  const int ny = std::max(1, std::min(cap, int(std::ceil(dy / h))));
  const int nz = std::max(1, std::min(cap, int(std::ceil(dz / h))));
  if (mode != 3) {
    for (int k = 0; k <= nz; k++) {
      for (int j = 0; j <= ny; j++) {
        for (int i = 0; i <= nx; i++) {
          const Point_3 p = mk_p3(bb.xmin() + dx * i / double(nx),
                                  bb.ymin() + dy * j / double(ny),
                                  bb.zmin() + dz * k / double(nz));
          if (inside(p) == CGAL::ON_BOUNDED_SIDE) {
            dt.insert(p);
          }
        }
      }
    }
  }
  int tag = 0;
  for (auto cit = dt.finite_cells_begin(); cit != dt.finite_cells_end(); ++cit) {
    const Point_3 c = dt.dual(cit);
    if (inside(c) != CGAL::ON_BOUNDED_SIDE) {
      continue;
    }
    append_tet(result,
               cit->vertex(0)->point(),
               cit->vertex(1)->point(),
               cit->vertex(2)->point(),
               cit->vertex(3)->point(),
               tag++,
               1.0);
  }
  result.ok = result.faces_num() > 0;
  if (!result.ok) {
    result.error = "No tetrahedron has its circumcenter inside the domain";
  }
  return result;
}

}  // namespace

MeshResult mesh_tet_remesh(const MeshIn &mesh, double target_edge)
{
  CgalThrowGuard guard;
  try {
    return interior_tets_with_steiner(mesh, target_edge, 1);
  }
  catch (const std::exception &e) {
    MeshResult result;
    result.error = e.what();
    return result;
  }
}

MeshResult mesh_constrained_delaunay_3(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error) || !CGAL::is_closed(sm) || !mesh.positions || mesh.verts_num < 4)
  {
    result.error = error.empty() ? "Needs a closed triangle mesh" : error;
    return result;
  }
  try {
    using Vb = CGAL::Triangulation_vertex_base_with_info_3<int, Kernel>;
    using Cb = CGAL::Delaunay_triangulation_cell_base_3<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_3<Vb, Cb>;
    using DT3i = CGAL::Delaunay_triangulation_3<Kernel, Tds>;

    CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel> inside(sm);
    DT3i dt;
    std::map<DT3i::Vertex_handle, int> v2i;
    for (int i = 0; i < mesh.verts_num; i++) {
      const Point_3 p = mk_p3(mesh.positions[i * 3],
                              mesh.positions[i * 3 + 1],
                              mesh.positions[i * 3 + 2]);
      const DT3i::Vertex_handle vh = dt.insert(p);
      if (v2i.find(vh) == v2i.end()) {
        v2i[vh] = i;
      }
    }
    /* One disconnected tet island per cell so the result reads as tetrahedra,
     * while vert_src0 maps each copy back to the original vertex for attributes. */
    int tag = 0;
    for (auto cit = dt.finite_cells_begin(); cit != dt.finite_cells_end(); ++cit) {
      const Point_3 c = dt.dual(cit);
      if (inside(c) != CGAL::ON_BOUNDED_SIDE) {
        continue;
      }
      int ids[4];
      bool ok = true;
      Point_3 pts[4];
      for (int j = 0; j < 4; j++) {
        const auto found = v2i.find(cit->vertex(j));
        if (found == v2i.end()) {
          ok = false;
          break;
        }
        ids[j] = found->second;
        pts[j] = cit->vertex(j)->point();
      }
      if (!ok) {
        continue;
      }
      append_tet(result, pts[0], pts[1], pts[2], pts[3], tag, 1.0);
      for (int j = 0; j < 4; j++) {
        result.vert_src0.push_back(ids[j]);
      }
      tag++;
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "No tetrahedron has its circumcenter inside the domain";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Constrained DT3 failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
