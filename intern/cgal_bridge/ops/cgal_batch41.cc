/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 41 (after user deletions):
 *  - Pullout Directions 2D
 *  - SSAB Partition 2D
 *  - Snap Borders (cross-island only)
 *  - Autorefine Clean (autorefine + local SI repair, keep all faces)
 * Deleted this turn (legacy reserved): Split Crossings 2D (2547),
 * Crossing Points 2D (2548), Random Polygon 2D (2553), Random Convex Set 2D (2554).
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

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Dynamic_property_map.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/autorefinement.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/internal/Snapping/snap.h>
#include <CGAL/Polygon_mesh_processing/repair_self_intersections.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/Face_filtered_graph.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/Random.h>
#include <CGAL/Set_movable_separability_2/Single_mold_translational_casting/top_edges.h>
#include <CGAL/Small_side_angle_bisector_decomposition_2.h>
#include <CGAL/Surface_sweep_2_algorithms.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Random_convex_set_traits_2.h>
#include <CGAL/random_convex_set_2.h>
#include <CGAL/random_polygon_2.h>

#include <algorithm>
#include <cmath>
#include <list>
#include <map>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Segment_2 = Kernel::Segment_2;
using Vector_2 = Kernel::Vector_2;
using Direction_2 = Kernel::Direction_2;
using FT = Kernel::FT;
using Polygon_2 = CGAL::Polygon_2<Kernel>;

static Point_2 xy(double x, double y)
{
  return Point_2(FT(x), FT(y));
}

static void push_xy0(std::vector<float> &pos, const Point_2 &p, float z)
{
  pos.push_back(float(CGAL::to_double(p.x())));
  pos.push_back(float(CGAL::to_double(p.y())));
  pos.push_back(z);
}

static float mid_z(const float *positions, int n)
{
  if (!positions || n <= 0) {
    return 0.0f;
  }
  double s = 0.0;
  for (int i = 0; i < n; i++) {
    s += double(positions[i * 3 + 2]);
  }
  return float(s / double(n));
}

static double signed_area_2(const std::vector<Point_2> &loop)
{
  double a = 0.0;
  const size_t n = loop.size();
  for (size_t i = 0; i < n; i++) {
    const Point_2 &p = loop[i];
    const Point_2 &q = loop[(i + 1) % n];
    a += CGAL::to_double(p.x()) * CGAL::to_double(q.y()) -
         CGAL::to_double(q.x()) * CGAL::to_double(p.y());
  }
  return 0.5 * a;
}

static double xy_snap_scale(const MeshIn &mesh)
{
  if (!mesh.positions || mesh.verts_num <= 0) {
    return 1.0e6;
  }
  double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
  for (int i = 0; i < mesh.verts_num; i++) {
    const double x = double(mesh.positions[i * 3 + 0]);
    const double y = double(mesh.positions[i * 3 + 1]);
    if (!std::isfinite(x) || !std::isfinite(y)) {
      continue;
    }
    minx = std::min(minx, x);
    miny = std::min(miny, y);
    maxx = std::max(maxx, x);
    maxy = std::max(maxy, y);
  }
  const double diag = std::hypot(std::max(0.0, maxx - minx), std::max(0.0, maxy - miny));
  if (!(diag > 1e-18)) {
    return 1.0e6;
  }
  return std::min(1.0e9, std::max(1.0e4, 1.0e7 / diag));
}

static Point_2 snap_pt2(const Point_2 &p, double scale)
{
  const double x = CGAL::to_double(p.x());
  const double y = CGAL::to_double(p.y());
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return p;
  }
  return xy(std::round(x * scale) / scale, std::round(y * scale) / scale);
}

static void collect_segments_xy(const MeshIn &mesh, std::vector<Segment_2> &segs)
{
  const double scale = xy_snap_scale(mesh);
  auto add = [&](int a, int b) {
    if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
      return;
    }
    const Point_2 p = snap_pt2(
        xy(double(mesh.positions[a * 3 + 0]), double(mesh.positions[a * 3 + 1])), scale);
    const Point_2 q = snap_pt2(
        xy(double(mesh.positions[b * 3 + 0]), double(mesh.positions[b * 3 + 1])), scale);
    if (p == q) {
      return;
    }
    segs.emplace_back(p, q);
  };
  if (mesh.edges_num > 0 && mesh.edge_v0 && mesh.edge_v1) {
    for (int i = 0; i < mesh.edges_num; i++) {
      add(mesh.edge_v0[i], mesh.edge_v1[i]);
    }
    return;
  }
  if (!mesh.corner_verts || mesh.corners_num < 3) {
    return;
  }
  const bool have_off = mesh.face_offsets != nullptr && mesh.faces_num > 0;
  const int faces = have_off ? mesh.faces_num : mesh.corners_num / 3;
  for (int f = 0; f < faces; f++) {
    const int begin = have_off ? mesh.face_offsets[f] : f * 3;
    const int end = have_off ? mesh.face_offsets[f + 1] : begin + 3;
    if (end - begin < 2) {
      continue;
    }
    for (int i = begin; i < end; i++) {
      const int j = (i + 1 == end) ? begin : i + 1;
      add(mesh.corner_verts[i], mesh.corner_verts[j]);
    }
  }
}

static void collect_face_rings_xy(const MeshIn &mesh, std::vector<std::vector<Point_2>> &rings)
{
  rings.clear();
  if (!mesh.corner_verts || mesh.corners_num < 3) {
    return;
  }
  const double scale = xy_snap_scale(mesh);
  const bool have_off = mesh.face_offsets != nullptr && mesh.faces_num > 0;
  const int faces = have_off ? mesh.faces_num : mesh.corners_num / 3;
  for (int f = 0; f < faces; f++) {
    const int begin = have_off ? mesh.face_offsets[f] : f * 3;
    const int end = have_off ? mesh.face_offsets[f + 1] : begin + 3;
    if (end - begin < 3) {
      continue;
    }
    std::vector<Point_2> ring;
    ring.reserve(size_t(end - begin));
    for (int i = begin; i < end; i++) {
      const int v = mesh.corner_verts[i];
      if (v < 0 || v >= mesh.verts_num) {
        ring.clear();
        break;
      }
      const Point_2 p = snap_pt2(
          xy(double(mesh.positions[v * 3 + 0]), double(mesh.positions[v * 3 + 1])), scale);
      if (ring.empty() || ring.back() != p) {
        ring.push_back(p);
      }
    }
    if (ring.size() >= 3 && ring.front() == ring.back()) {
      ring.pop_back();
    }
    if (ring.size() >= 3 && std::abs(signed_area_2(ring)) > 1e-16) {
      rings.push_back(std::move(ring));
    }
  }
}

static void strip_collinear(std::vector<Point_2> &ring)
{
  if (ring.size() < 4) {
    return;
  }
  std::vector<Point_2> out;
  out.reserve(ring.size());
  const int n = int(ring.size());
  for (int i = 0; i < n; i++) {
    const Point_2 &a = ring[size_t((i + n - 1) % n)];
    const Point_2 &b = ring[size_t(i)];
    const Point_2 &c = ring[size_t((i + 1) % n)];
    if (CGAL::orientation(a, b, c) == CGAL::COLLINEAR) {
      continue;
    }
    out.push_back(b);
  }
  if (out.size() >= 3) {
    ring.swap(out);
  }
}

static bool ring_to_polygon(std::vector<Point_2> ring, Polygon_2 &poly, std::string &error)
{
  strip_collinear(ring);
  if (ring.size() < 3) {
    error = "Need a simple XY polygon with at least 3 non-collinear vertices";
    return false;
  }
  if (signed_area_2(ring) < 0.0) {
    std::reverse(ring.begin(), ring.end());
  }
  poly = Polygon_2(ring.begin(), ring.end());
  if (!poly.is_simple()) {
    error = "Input polygon is not simple (self-intersecting)";
    return false;
  }
  return true;
}

static void append_isolated_ngon(MeshResult &result,
                                 const std::vector<Point_2> &ring,
                                 float z,
                                 int tag)
{
  if (ring.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  const int base = result.verts_num();
  for (const Point_2 &p : ring) {
    push_xy0(result.positions, p, z);
  }
  for (int i = 0; i < int(ring.size()); i++) {
    result.corner_verts.push_back(base + i);
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
}

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  PMP::triangulate_faces(sm);
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

static int vert_of(std::map<std::pair<long long, long long>, int> &vmap,
                   MeshResult &result,
                   const Point_2 &p,
                   float z)
{
  const std::pair<long long, long long> key{llround(CGAL::to_double(p.x()) * 1e7),
                                            llround(CGAL::to_double(p.y()) * 1e7)};
  const auto it = vmap.find(key);
  if (it != vmap.end()) {
    return it->second;
  }
  const int idx = result.verts_num();
  vmap[key] = idx;
  push_xy0(result.positions, p, z);
  return idx;
}

}  // namespace

WireResult mesh_split_crossings_2(const MeshIn &mesh)
{
  WireResult result;
  CgalThrowGuard guard;
  try {
    std::vector<Segment_2> segs;
    collect_segments_xy(mesh, segs);
    if (segs.size() < 1) {
      result.error = "Split Crossings 2D needs at least one XY edge";
      return result;
    }
    std::vector<Segment_2> subs;
    CGAL::compute_subcurves(segs.begin(), segs.end(), std::back_inserter(subs), false);
    if (subs.empty()) {
      result.error = "Split Crossings 2D produced no subcurves";
      return result;
    }
    const float z = mid_z(mesh.positions, mesh.verts_num);
    std::map<std::pair<long long, long long>, int> vmap;
    auto vid = [&](const Point_2 &p) {
      const std::pair<long long, long long> key{llround(CGAL::to_double(p.x()) * 1e7),
                                                llround(CGAL::to_double(p.y()) * 1e7)};
      const auto it = vmap.find(key);
      if (it != vmap.end()) {
        return it->second;
      }
      const int idx = int(result.positions.size() / 3);
      vmap[key] = idx;
      push_xy0(result.positions, p, z);
      return idx;
    };
    for (const Segment_2 &s : subs) {
      const int a = vid(s.source());
      const int b = vid(s.target());
      if (a != b) {
        result.edge_v0.push_back(a);
        result.edge_v1.push_back(b);
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Split Crossings 2D: empty after welding";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Split Crossings 2D failed";
  }
  return result;
}

MeshResult mesh_crossing_points_2(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<Segment_2> segs;
    collect_segments_xy(mesh, segs);
    if (segs.size() < 2) {
      result.error = "Crossing Points 2D needs at least two XY edges";
      return result;
    }
    std::vector<Point_2> pts;
    CGAL::compute_intersection_points(segs.begin(), segs.end(), std::back_inserter(pts), false);
    if (pts.empty()) {
      result.error = "Crossing Points 2D: no proper intersections (edges do not cross)";
      return result;
    }
    const float z = mid_z(mesh.positions, mesh.verts_num);
    std::map<std::pair<long long, long long>, int> seen;
    for (const Point_2 &p : pts) {
      vert_of(seen, result, p, z);
    }
    result.ok = result.verts_num() > 0;
    if (!result.ok) {
      result.error = "Crossing Points 2D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Crossing Points 2D failed";
  }
  return result;
}

MeshResult mesh_pullout_directions_2(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<std::vector<Point_2>> rings;
    collect_face_rings_xy(mesh, rings);
    if (rings.empty()) {
      result.error = "Pullout Directions 2D needs an XY face";
      return result;
    }
    /* Largest-area ring as the casting part. */
    size_t best = 0;
    double best_a = 0.0;
    for (size_t i = 0; i < rings.size(); i++) {
      const double a = std::abs(signed_area_2(rings[i]));
      if (a > best_a) {
        best_a = a;
        best = i;
      }
    }
    Polygon_2 poly;
    std::string err;
    if (!ring_to_polygon(rings[best], poly, err)) {
      result.error = err;
      return result;
    }
    namespace SMS = CGAL::Set_movable_separability_2::Single_mold_translational_casting;
    using Top = std::pair<Polygon_2::Edge_const_iterator, std::pair<Direction_2, Direction_2>>;
    std::list<Top> tops;
    SMS::top_edges(poly, std::back_inserter(tops));
    if (tops.empty()) {
      result.error = "Pullout Directions 2D: no castable top edge (not single-mold translational)";
      return result;
    }

    const float z = mid_z(mesh.positions, mesh.verts_num);
    std::vector<Point_2> outer(poly.vertices_begin(), poly.vertices_end());
    append_isolated_ngon(result, outer, z, 0);

    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (const Point_2 &p : outer) {
      minx = std::min(minx, CGAL::to_double(p.x()));
      miny = std::min(miny, CGAL::to_double(p.y()));
      maxx = std::max(maxx, CGAL::to_double(p.x()));
      maxy = std::max(maxy, CGAL::to_double(p.y()));
    }
    const double scale = std::max(1e-6, std::hypot(maxx - minx, maxy - miny));

    auto dir_unit = [](const Direction_2 &d) {
      const double dx = CGAL::to_double(d.dx());
      const double dy = CGAL::to_double(d.dy());
      const double len = std::hypot(dx, dy);
      if (!(len > 1e-18)) {
        return Vector_2(FT(0), FT(0));
      }
      return Vector_2(FT(dx / len), FT(dy / len));
    };

    int tag = 1;
    for (const Top &t : tops) {
      const Segment_2 edge = *t.first;
      const Point_2 mid = CGAL::midpoint(edge.source(), edge.target());
      const Vector_2 u0 = dir_unit(t.second.first);
      const Vector_2 u1 = dir_unit(t.second.second);
      const Point_2 a = mid + u0 * FT(scale);
      const Point_2 b = mid + u1 * FT(scale);
      append_isolated_ngon(result, {mid, a, b}, z, tag++);
    }
    result.ok = result.faces_num() > 0;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Pullout Directions 2D failed";
  }
  return result;
}

MeshResult mesh_ssab_partition_2(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<std::vector<Point_2>> rings;
    collect_face_rings_xy(mesh, rings);
    if (rings.empty()) {
      result.error = "SSAB Partition 2D needs an XY face";
      return result;
    }
    const float z = mid_z(mesh.positions, mesh.verts_num);
    CGAL::Small_side_angle_bisector_decomposition_2<Kernel> decomp;
    int tag = 0;
    for (auto &ring : rings) {
      Polygon_2 poly;
      std::string err;
      if (!ring_to_polygon(ring, poly, err)) {
        continue;
      }
      std::list<Polygon_2> pieces;
      decomp(poly, std::back_inserter(pieces));
      for (const Polygon_2 &piece : pieces) {
        if (piece.size() < 3) {
          continue;
        }
        std::vector<Point_2> pts(piece.vertices_begin(), piece.vertices_end());
        append_isolated_ngon(result, pts, z, tag++);
      }
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = result.error.empty() ? "SSAB Partition 2D produced no pieces" : result.error;
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "SSAB Partition 2D failed";
  }
  return result;
}

MeshResult mesh_snap_borders(const MeshIn &mesh, double tolerance)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Snap Borders needs a mesh with faces" : error;
    return result;
  }
  try {
    auto fccmap = sm.add_property_map<Surface_mesh::Face_index, std::size_t>("f:cc", 0).first;
    const std::size_t ncc = PMP::connected_components(sm, fccmap);
    if (ncc < 2) {
      result.error =
          "Snap Borders only joins different islands. A single connected piece is left unchanged "
          "(it will not snap a border onto itself). Join / Realize other islands first.";
      return result;
    }

    std::vector<Surface_mesh> parts(ncc);
    for (std::size_t i = 0; i < ncc; i++) {
      CGAL::Face_filtered_graph<Surface_mesh> ffg(sm, i, fccmap);
      CGAL::copy_face_graph(ffg, parts[i]);
    }

    const FT tol(std::max(0.0, tolerance));
    for (std::size_t i = 0; i < ncc; i++) {
      if (parts[i].number_of_faces() == 0) {
        continue;
      }
      for (std::size_t j = i + 1; j < ncc; j++) {
        if (parts[j].number_of_faces() == 0) {
          continue;
        }
        auto ta = parts[i].add_property_map<Surface_mesh::Vertex_index, FT>("v:tol", tol).first;
        auto tb = parts[j].add_property_map<Surface_mesh::Vertex_index, FT>("v:tol", tol).first;
        /* Two-mesh snap: never self-snap an island onto its own border. */
        PMP::experimental::snap_borders(parts[i], ta, parts[j], tb);
      }
    }

    Surface_mesh out;
    for (Surface_mesh &part : parts) {
      if (part.number_of_faces() > 0) {
        CGAL::copy_face_graph(part, out);
      }
    }
    /* After lips meet, stitch matching border pairs so islands actually join. */
    PMP::stitch_borders(out);
    result = surface_mesh_to_result(out);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Snap Borders failed";
  }
  return result;
}

MeshResult mesh_autorefine_clean(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Autorefine Clean needs a mesh with faces" : error;
    return result;
  }
  try {
    /* Keep every face. Do not use autorefine_and_remove_self_intersections:
     * that builder deletes "extra patches" and can leave only the intersection
     * region. Cut intersections first, then locally repair leftover SI. */
    PMP::autorefine(sm);
    PMP::experimental::remove_self_intersections(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Autorefine Clean failed";
  }
  return result;
}

MeshResult points_random_polygon_2(int count, double half_size, int seed)
{
  MeshResult result;
  CgalThrowGuard guard;
  count = std::clamp(count, 3, 10000);
  if (!(half_size > 0.0) || !std::isfinite(half_size)) {
    result.error = "Random Polygon 2D: Size must be > 0";
    return result;
  }
  try {
    std::vector<Point_2> pts;
    bool ok = false;
    for (int attempt = 0; attempt < 8 && !ok; attempt++) {
      pts.clear();
      CGAL::Random rng(static_cast<unsigned int>(seed) + unsigned(attempt) * 997u);
      CGAL::Random_points_in_square_2<Point_2> gen(half_size, rng);
      try {
        CGAL::random_polygon_2(std::size_t(count), std::back_inserter(pts), gen);
        ok = pts.size() >= 3;
      }
      catch (...) {
        ok = false;
      }
    }
    if (!ok) {
      result.error = "Random Polygon 2D failed (try another Seed or more Count)";
      return result;
    }
    if (signed_area_2(pts) < 0.0) {
      std::reverse(pts.begin(), pts.end());
    }
    append_isolated_ngon(result, pts, 0.0f, 0);
    result.ok = result.faces_num() > 0;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Random Polygon 2D failed";
  }
  return result;
}

MeshResult points_random_convex_set_2(int count, double half_size, int seed)
{
  MeshResult result;
  CgalThrowGuard guard;
  count = std::clamp(count, 3, 10000);
  if (!(half_size > 0.0) || !std::isfinite(half_size)) {
    result.error = "Random Convex Set 2D: Size must be > 0";
    return result;
  }
  try {
    CGAL::Random rng(static_cast<unsigned int>(seed));
    CGAL::Random_points_in_square_2<Point_2> gen(half_size, rng);
    std::vector<Point_2> pts;
    CGAL::random_convex_set_2(std::size_t(count),
                              std::back_inserter(pts),
                              gen,
                              CGAL::Random_convex_set_traits_2<Kernel>());
    if (pts.size() < 3) {
      result.error = "Random Convex Set 2D produced fewer than 3 points";
      return result;
    }
    if (signed_area_2(pts) < 0.0) {
      std::reverse(pts.begin(), pts.end());
    }
    append_isolated_ngon(result, pts, 0.0f, 0);
    result.ok = result.faces_num() > 0;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Random Convex Set 2D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
