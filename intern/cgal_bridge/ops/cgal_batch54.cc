/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 54 (2026-08-22, new only — do not restore deleted ids):
 *  - Walk Tets (DT cells stabbed by a query segment)
 *  - Skeleton Spokes (MCF skeleton-to-surface correspondence)
 *  - Radial Slices (planes that all contain an axis)
 *  - Intersection Band (faces incident to a corefine seam)
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

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/Triangulation_segment_traverser_3.h>
#include <CGAL/assertions_behaviour.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Plane_3 = Kernel::Plane_3;
using FT = Kernel::FT;
using DT3 = CGAL::Delaunay_triangulation_3<Kernel>;
using VI = Surface_mesh::Vertex_index;
using FI = Surface_mesh::Face_index;
using EI = Surface_mesh::Edge_index;

static Point_3 mk_p3(double x, double y, double z)
{
  const FT fx(x);
  const FT fy(y);
  const FT fz(z);
  return Point_3(fx, fy, fz);
}

static void push_p3(std::vector<float> &pos, const Point_3 &p)
{
  pos.push_back(float(CGAL::to_double(p.x())));
  pos.push_back(float(CGAL::to_double(p.y())));
  pos.push_back(float(CGAL::to_double(p.z())));
}

static void emit_tet(MeshResult &result, DT3::Cell_handle c)
{
  const int vbase = result.verts_num();
  for (int k = 0; k < 4; k++) {
    push_p3(result.positions, c->vertex(k)->point());
  }
  const int faces[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  for (int f = 0; f < 4; f++) {
    result.corner_verts.push_back(vbase + faces[f][0]);
    result.corner_verts.push_back(vbase + faces[f][1]);
    result.corner_verts.push_back(vbase + faces[f][2]);
  }
}

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  sm.collect_garbage();
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

static void pack_point(std::vector<float> &xyz, const Point_3 &p)
{
  xyz.push_back(float(CGAL::to_double(p.x())));
  xyz.push_back(float(CGAL::to_double(p.y())));
  xyz.push_back(float(CGAL::to_double(p.z())));
}

static void append_faces_of_constrained_edges(const Surface_mesh &sm,
                                              Surface_mesh::Property_map<EI, bool> ecm,
                                              Surface_mesh &out)
{
  std::set<FI> keep;
  for (const EI e : sm.edges()) {
    if (sm.is_removed(e) || !ecm[e]) {
      continue;
    }
    const auto h = sm.halfedge(e);
    const FI f0 = sm.face(h);
    const FI f1 = sm.face(sm.opposite(h));
    if (f0 != Surface_mesh::null_face() && !sm.is_removed(f0)) {
      keep.insert(f0);
    }
    if (f1 != Surface_mesh::null_face() && !sm.is_removed(f1)) {
      keep.insert(f1);
    }
  }
  std::map<VI, VI> vmap;
  auto addv = [&](VI v) {
    const auto it = vmap.find(v);
    if (it != vmap.end()) {
      return it->second;
    }
    const VI nv = out.add_vertex(sm.point(v));
    vmap[v] = nv;
    return nv;
  };
  for (const FI f : keep) {
    std::vector<VI> vs;
    vs.reserve(3);
    for (const VI v : CGAL::vertices_around_face(sm.halfedge(f), sm)) {
      vs.push_back(addv(v));
    }
    if (vs.size() >= 3) {
      out.add_face(vs);
    }
  }
}

}  // namespace

MeshResult points_walk_tets(const float *positions,
                            int n,
                            double ax,
                            double ay,
                            double az,
                            double bx,
                            double by,
                            double bz)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 4) {
    result.error = "Walk Tets needs at least 4 points";
    return result;
  }
  try {
    DT3 dt;
    for (int i = 0; i < n; i++) {
      dt.insert(mk_p3(double(positions[i * 3 + 0]),
                      double(positions[i * 3 + 1]),
                      double(positions[i * 3 + 2])));
    }
    if (dt.dimension() < 3) {
      result.error = "Walk Tets: points are not 3D (Delaunay dimension < 3)";
      return result;
    }
    const Point_3 pa = mk_p3(ax, ay, az);
    const Point_3 pb = mk_p3(bx, by, bz);
    if (pa == pb) {
      result.error = "Walk Tets: Start and End must be distinct";
      return result;
    }
    auto cit = dt.segment_traverser_cells_begin(pa, pb);
    const auto cend = dt.segment_traverser_cells_end();
    for (; cit != cend; ++cit) {
      const DT3::Cell_handle c = cit;
      if (c == DT3::Cell_handle() || dt.is_infinite(c)) {
        continue;
      }
      emit_tet(result, c);
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Walk Tets: the segment misses every finite tetrahedron";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Walk Tets failed";
  }
  return result;
}

WireResult mesh_skeleton_spokes(const MeshIn &mesh)
{
  WireResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string io_err;
  if (!load_tri(mesh, sm, io_err) || !CGAL::is_closed(sm)) {
    result.error = io_err.empty() ? "Skeleton Spokes needs a closed triangle mesh" : io_err;
    return result;
  }
  try {
    using Skel = CGAL::Mean_curvature_flow_skeletonization<Surface_mesh>::Skeleton;
    using SkelV = boost::graph_traits<Skel>::vertex_descriptor;
    using SkelE = boost::graph_traits<Skel>::edge_descriptor;
    Skel skeleton;
    CGAL::extract_mean_curvature_flow_skeleton(sm, skeleton);
    std::map<SkelV, int> v2i;
    for (const SkelV v : CGAL::make_range(vertices(skeleton))) {
      v2i[v] = int(result.positions.size() / 3);
      push_p3(result.positions, skeleton[v].point);
    }
    for (const SkelE e : CGAL::make_range(edges(skeleton))) {
      result.edge_v0.push_back(v2i[source(e, skeleton)]);
      result.edge_v1.push_back(v2i[target(e, skeleton)]);
    }
    int spoke_n = 0;
    for (const SkelV v : CGAL::make_range(vertices(skeleton))) {
      const int si = v2i[v];
      for (const VI vd : skeleton[v].vertices) {
        if (sm.is_removed(vd)) {
          continue;
        }
        const int ti = int(result.positions.size() / 3);
        push_p3(result.positions, sm.point(vd));
        result.edge_v0.push_back(si);
        result.edge_v1.push_back(ti);
        spoke_n++;
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Skeleton Spokes produced no edges";
    }
    else if (spoke_n == 0) {
      result.error = "Skeleton Spokes: centerline only (no surface correspondence)";
      result.ok = true;
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Skeleton Spokes failed";
  }
  return result;
}

bool mesh_radial_slices(const MeshIn &mesh,
                        float ox,
                        float oy,
                        float oz,
                        float ax,
                        float ay,
                        float az,
                        int count,
                        std::vector<std::vector<float>> &out_polylines,
                        std::string &error)
{
  out_polylines.clear();
  CgalThrowGuard guard;
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    error = error.empty() ? "Radial Slices needs a mesh with faces" : error;
    return false;
  }
  count = std::clamp(count, 1, 180);
  try {
    const FT fax(ax);
    const FT fay(ay);
    const FT faz(az);
    Vector_3 axis = Vector_3(fax, fay, faz);
    const double alen = std::sqrt(CGAL::to_double(axis.squared_length()));
    if (alen < 1e-20) {
      const FT z1(1);
      const FT z0(0);
      axis = Vector_3(z0, z0, z1);
    }
    else {
      const FT inv(alen);
      axis = axis / inv;
    }
    const FT one(1);
    const FT zero(0);
    Vector_3 u = Vector_3(one, zero, zero);
    if (std::abs(CGAL::to_double(axis.z())) < 0.9) {
      u = Vector_3(zero, zero, one);
    }
    Vector_3 t = CGAL::cross_product(axis, u);
    const double tlen = std::sqrt(CGAL::to_double(t.squared_length()));
    if (tlen < 1e-20) {
      error = "Radial Slices: could not build a plane around Axis";
      return false;
    }
    const FT tinv(tlen);
    t = t / tinv;
    const Vector_3 bvec = CGAL::cross_product(axis, t);
    const Point_3 origin = mk_p3(double(ox), double(oy), double(oz));
    CGAL::Polygon_mesh_slicer<Surface_mesh, Kernel> slicer(sm);
    const double step = 3.14159265358979323846 / double(count);
    for (int i = 0; i < count; i++) {
      const double ang = step * double(i);
      const FT c(std::cos(ang));
      const FT s(std::sin(ang));
      const Vector_3 n = t * c + bvec * s;
      const Plane_3 plane(origin, n);
      std::vector<std::vector<Point_3>> polylines;
      slicer(plane, std::back_inserter(polylines));
      for (const auto &pl : polylines) {
        if (pl.size() < 2) {
          continue;
        }
        std::vector<float> xyz;
        xyz.reserve(pl.size() * 3);
        for (const Point_3 &p : pl) {
          pack_point(xyz, p);
        }
        out_polylines.push_back(std::move(xyz));
      }
    }
    if (out_polylines.empty()) {
      error = "Radial Slices produced no curves";
      return false;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
  }
  catch (...) {
    error = "Radial Slices failed";
  }
  return false;
}

MeshResult mesh_intersection_band(const MeshIn &a, const MeshIn &b)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm_a, sm_b;
  std::string io_err;
  if (!load_tri(a, sm_a, io_err)) {
    result.error = "Intersection Band mesh A: " + (io_err.empty() ? "failed" : io_err);
    return result;
  }
  if (!load_tri(b, sm_b, io_err)) {
    result.error = "Intersection Band mesh B: " + (io_err.empty() ? "failed" : io_err);
    return result;
  }
  try {
    auto ecm_a = sm_a.add_property_map<EI, bool>("e:band", false).first;
    auto ecm_b = sm_b.add_property_map<EI, bool>("e:band", false).first;
    PMP::corefine(sm_a,
                  sm_b,
                  CGAL::parameters::edge_is_constrained_map(ecm_a),
                  CGAL::parameters::edge_is_constrained_map(ecm_b));
    Surface_mesh out;
    append_faces_of_constrained_edges(sm_a, ecm_a, out);
    append_faces_of_constrained_edges(sm_b, ecm_b, out);
    result = surface_mesh_to_result(out);
    if (!result.ok || result.faces_num() == 0) {
      result.ok = false;
      result.error = result.error.empty() ? "Intersection Band: meshes do not cut each other" :
                                            result.error;
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Intersection Band failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
