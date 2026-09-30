/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * 2D shape overlay for flattened meshes.
 *
 * A flattened 3D mesh is treated as the union of its non-degenerate projected
 * faces (side / edge-on faces are ignored). Arrangement rebuilds those 2D
 * polygons (split at crossings). Boolean classifies cells with the non-zero
 * winding rule (inside iff winding != 0) and fills the keep region.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_types.hh"

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <boost/property_map/property_map.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;
using Traits = CGAL::Arr_segment_traits_2<Kernel>;
using Arr = CGAL::Arrangement_2<Traits>;

static Point_2 project_xy(const float *xyz)
{
  return Point_2(double(xyz[0]), double(xyz[1]));
}

static double snap_scale_xy(const MeshIn &mesh)
{
  if (!mesh.positions || mesh.verts_num <= 0) {
    return 1.0e6;
  }
  double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
  bool any = false;
  for (int i = 0; i < mesh.verts_num; i++) {
    const double x = double(mesh.positions[i * 3 + 0]);
    const double y = double(mesh.positions[i * 3 + 1]);
    if (!std::isfinite(x) || !std::isfinite(y)) {
      continue;
    }
    any = true;
    minx = std::min(minx, x);
    miny = std::min(miny, y);
    maxx = std::max(maxx, x);
    maxy = std::max(maxy, y);
  }
  if (!any) {
    return 1.0e6;
  }
  const double diag = std::hypot(std::max(0.0, maxx - minx), std::max(0.0, maxy - miny));
  if (!(diag > 1e-18)) {
    return 1.0e6;
  }
  /* Coarse enough that opposite faces of a flattened solid weld together. */
  return std::min(1.0e8, std::max(1.0e4, 1.0e6 / diag));
}

static Point_2 snap_pt(const Point_2 &p, double scale)
{
  const double x = CGAL::to_double(p.x());
  const double y = CGAL::to_double(p.y());
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return p;
  }
  return Point_2(std::round(x * scale) / scale, std::round(y * scale) / scale);
}

static double signed_area(const std::vector<Point_2> &ring)
{
  double a = 0.0;
  const size_t n = ring.size();
  for (size_t i = 0; i < n; i++) {
    const Point_2 &p = ring[i];
    const Point_2 &q = ring[(i + 1) % n];
    a += CGAL::to_double(p.x()) * CGAL::to_double(q.y()) -
         CGAL::to_double(q.x()) * CGAL::to_double(p.y());
  }
  return 0.5 * a;
}

static void clean_ring(std::vector<Point_2> &ring)
{
  if (ring.empty()) {
    return;
  }
  std::vector<Point_2> out;
  out.reserve(ring.size());
  for (const Point_2 &p : ring) {
    if (out.empty() || out.back() != p) {
      out.push_back(p);
    }
  }
  if (out.size() >= 2 && out.front() == out.back()) {
    out.pop_back();
  }
  ring.swap(out);
}

static std::vector<Point_2> collapse_collinear(std::vector<Point_2> ring)
{
  if (ring.size() < 4) {
    return ring;
  }
  for (int pass = 0; pass < 8; pass++) {
    const int n = int(ring.size());
    if (n < 3) {
      break;
    }
    std::vector<Point_2> out;
    out.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      const Point_2 &a = ring[size_t((i + n - 1) % n)];
      const Point_2 &b = ring[size_t(i)];
      const Point_2 &c = ring[size_t((i + 1) % n)];
      const double ax = CGAL::to_double(a.x());
      const double ay = CGAL::to_double(a.y());
      const double bx = CGAL::to_double(b.x());
      const double by = CGAL::to_double(b.y());
      const double cx = CGAL::to_double(c.x());
      const double cy = CGAL::to_double(c.y());
      const double abx = bx - ax;
      const double aby = by - ay;
      const double bcx = cx - bx;
      const double bcy = cy - by;
      const double cross = abx * bcy - aby * bcx;
      const double scale = std::sqrt(std::max(0.0, (abx * abx + aby * aby) * (bcx * bcx + bcy * bcy)));
      if (scale > 1e-30 && std::abs(cross) <= 1e-7 * scale) {
        continue;
      }
      out.push_back(b);
    }
    if (out.size() < 3 || out.size() == ring.size()) {
      if (out.size() >= 3) {
        ring.swap(out);
      }
      break;
    }
    ring.swap(out);
  }
  return ring;
}

struct Ring2 {
  std::vector<Point_2> pts;
  double minx = 0, miny = 0, maxx = 0, maxy = 0;
};

static void ring_bbox(Ring2 &r)
{
  r.minx = r.miny = 1e300;
  r.maxx = r.maxy = -1e300;
  for (const Point_2 &p : r.pts) {
    const double x = CGAL::to_double(p.x());
    const double y = CGAL::to_double(p.y());
    r.minx = std::min(r.minx, x);
    r.miny = std::min(r.miny, y);
    r.maxx = std::max(r.maxx, x);
    r.maxy = std::max(r.maxy, y);
  }
}

/* Non-zero winding of one ring (Hormann / AGP). CCW contributes +1 inside. */
static int winding_in_ring(const Point_2 &q, const Ring2 &r)
{
  const double qx = CGAL::to_double(q.x());
  const double qy = CGAL::to_double(q.y());
  if (qx < r.minx || qx > r.maxx || qy < r.miny || qy > r.maxy) {
    return 0;
  }
  const size_t n = r.pts.size();
  if (n < 3) {
    return 0;
  }
  int wn = 0;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = CGAL::to_double(r.pts[i].x());
    const double yi = CGAL::to_double(r.pts[i].y());
    const double xj = CGAL::to_double(r.pts[j].x());
    const double yj = CGAL::to_double(r.pts[j].y());
    if (yj <= qy) {
      if (yi > qy) {
        const double cross = (xi - xj) * (qy - yj) - (qx - xj) * (yi - yj);
        if (cross > 0.0) {
          wn++;
        }
      }
    }
    else if (yi <= qy) {
      const double cross = (xi - xj) * (qy - yj) - (qx - xj) * (yi - yj);
      if (cross < 0.0) {
        wn--;
      }
    }
  }
  return wn;
}

static int winding_at(const Point_2 &q, const std::vector<Ring2> &rings)
{
  int w = 0;
  for (const Ring2 &r : rings) {
    w += winding_in_ring(q, r);
  }
  return w;
}

static bool covered(const Point_2 &q, const std::vector<Ring2> &rings)
{
  return winding_at(q, rings) != 0;
}

static bool collect_shape_rings(const MeshIn &mesh, double scale, std::vector<Ring2> &rings)
{
  rings.clear();
  if (!mesh.positions || mesh.verts_num < 3 || !mesh.corner_verts || mesh.corners_num < 3) {
    return false;
  }
  const bool have_off = mesh.face_offsets != nullptr && mesh.faces_num > 0;
  const int nf = have_off ? mesh.faces_num : mesh.corners_num / 3;
  if (nf <= 0) {
    return false;
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
  const double bbox_area = std::max(0.0, maxx - minx) * std::max(0.0, maxy - miny);
  const double min_area = std::max(1e-16, 1e-12 * std::max(bbox_area, 1.0));

  for (int f = 0; f < nf; f++) {
    const int begin = have_off ? mesh.face_offsets[f] : f * 3;
    const int end = have_off ? mesh.face_offsets[f + 1] : begin + 3;
    if (end - begin < 3 || begin < 0 || end > mesh.corners_num) {
      continue;
    }
    Ring2 ring;
    ring.pts.reserve(size_t(end - begin));
    bool bad = false;
    for (int i = begin; i < end; i++) {
      const int v = mesh.corner_verts[i];
      if (v < 0 || v >= mesh.verts_num) {
        bad = true;
        break;
      }
      const Point_2 p = snap_pt(project_xy(mesh.positions + v * 3), scale);
      const double x = CGAL::to_double(p.x());
      const double y = CGAL::to_double(p.y());
      if (!std::isfinite(x) || !std::isfinite(y)) {
        bad = true;
        break;
      }
      ring.pts.push_back(p);
    }
    if (bad) {
      continue;
    }
    clean_ring(ring.pts);
    if (ring.pts.size() < 3 || std::abs(signed_area(ring.pts)) <= min_area) {
      continue;
    }
    ring_bbox(ring);
    rings.push_back(std::move(ring));
  }
  return !rings.empty();
}

static void collect_all_xy_edges(const MeshIn &mesh, double scale, std::vector<Segment_2> &segs)
{
  segs.clear();
  auto add = [&](int a, int b) {
    if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
      return;
    }
    const Point_2 p = snap_pt(project_xy(mesh.positions + a * 3), scale);
    const Point_2 q = snap_pt(project_xy(mesh.positions + b * 3), scale);
    if (p != q) {
      segs.emplace_back(p, q);
    }
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
  const int nf = have_off ? mesh.faces_num : mesh.corners_num / 3;
  for (int f = 0; f < nf; f++) {
    const int begin = have_off ? mesh.face_offsets[f] : f * 3;
    const int end = have_off ? mesh.face_offsets[f + 1] : begin + 3;
    if (end - begin < 2) {
      continue;
    }
    for (int i = begin; i < end; i++) {
      add(mesh.corner_verts[i], mesh.corner_verts[(i + 1 == end) ? begin : i + 1]);
    }
  }
}

static void collect_ring_segments(const std::vector<Ring2> &rings, std::vector<Segment_2> &segs)
{
  for (const Ring2 &r : rings) {
    const int n = int(r.pts.size());
    for (int i = 0; i < n; i++) {
      const Point_2 &a = r.pts[size_t(i)];
      const Point_2 &b = r.pts[size_t((i + 1) % n)];
      if (a != b) {
        segs.emplace_back(a, b);
      }
    }
  }
}

static void dedup_segments(std::vector<Segment_2> &segs)
{
  auto key = [](const Segment_2 &s) {
    Point_2 a = s.source();
    Point_2 b = s.target();
    if (b < a) {
      std::swap(a, b);
    }
    return std::make_pair(a, b);
  };
  std::sort(segs.begin(), segs.end(), [&](const Segment_2 &a, const Segment_2 &b) {
    const auto ka = key(a);
    const auto kb = key(b);
    if (ka.first < kb.first) {
      return true;
    }
    if (kb.first < ka.first) {
      return false;
    }
    return ka.second < kb.second;
  });
  segs.erase(std::unique(segs.begin(),
                         segs.end(),
                         [&](const Segment_2 &a, const Segment_2 &b) { return key(a) == key(b); }),
             segs.end());
}

static bool build_arr(const std::vector<Segment_2> &segs, Arr &arr)
{
  arr.clear();
  if (segs.empty()) {
    return false;
  }
  CgalThrowGuard guard;
  try {
    CGAL::insert(arr, segs.begin(), segs.end());
  }
  catch (...) {
    arr.clear();
    for (const Segment_2 &s : segs) {
      try {
        if (s.source() != s.target()) {
          CGAL::insert(arr, s);
        }
      }
      catch (...) {
      }
    }
  }
  return arr.number_of_vertices() >= 3 && arr.number_of_faces() > 0;
}

static Point_2 face_seed(Arr::Ccb_halfedge_circulator circ)
{
  const Point_2 a = circ->source()->point();
  const Point_2 b = circ->target()->point();
  const double ax = CGAL::to_double(a.x());
  const double ay = CGAL::to_double(a.y());
  const double bx = CGAL::to_double(b.x());
  const double by = CGAL::to_double(b.y());
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len = std::hypot(dx, dy);
  const double eps = (len > 1e-18) ? (1e-6 * len) : 1e-8;
  const double inv = (len > 1e-18) ? (1.0 / len) : 1.0;
  /* Face is to the left of the halfedge. */
  return Point_2(0.5 * (ax + bx) - dy * inv * eps, 0.5 * (ay + by) + dx * inv * eps);
}

static void mark_faces(Arr &arr,
                       const std::vector<Ring2> &rings,
                       std::map<Arr::Face_handle, bool> &keep)
{
  keep.clear();
  for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
    Arr::Face_handle f = fit;
    if (f->is_unbounded() || !f->has_outer_ccb()) {
      keep[f] = false;
      continue;
    }
    try {
      keep[f] = covered(face_seed(f->outer_ccb()), rings);
    }
    catch (...) {
      keep[f] = false;
    }
  }
}

static bool is_keep(const std::map<Arr::Face_handle, bool> &keep, Arr::Face_handle f)
{
  if (f->is_unbounded() || !f->has_outer_ccb()) {
    return false;
  }
  const auto it = keep.find(f);
  return it != keep.end() && it->second;
}

static Arr::Halfedge_handle next_outline(Arr::Halfedge_handle he,
                                         const std::map<Arr::Face_handle, bool> &keep)
{
  Arr::Halfedge_handle out = he->next();
  int guard = 0;
  while (is_keep(keep, out->twin()->face()) && ++guard < 256) {
    out = out->twin()->next();
  }
  return out;
}

static void uniquify_idx_ring(std::vector<int> &ring)
{
  if (ring.empty()) {
    return;
  }
  std::vector<int> out;
  out.reserve(ring.size());
  for (int v : ring) {
    if (out.empty() || out.back() != v) {
      out.push_back(v);
    }
  }
  if (out.size() >= 2 && out.front() == out.back()) {
    out.pop_back();
  }
  ring.swap(out);
}

static void emit_welded_cell(MeshResult &result,
                             const std::map<Arr::Vertex_handle, int> &vmap,
                             Arr::Ccb_halfedge_circulator circ,
                             int tag,
                             int max_steps)
{
  std::vector<int> ring;
  auto cur = circ;
  int steps = 0;
  do {
    const auto it = vmap.find(cur->source());
    if (it != vmap.end()) {
      ring.push_back(it->second);
    }
    ++cur;
  } while (cur != circ && ++steps < max_steps);
  if (steps >= max_steps) {
    return;
  }
  uniquify_idx_ring(ring);
  if (ring.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  result.corner_verts.insert(result.corner_verts.end(), ring.begin(), ring.end());
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
}

static void emit_isolated_ring(MeshResult &result, const std::vector<Point_2> &ring, int tag)
{
  if (ring.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  const int base = result.verts_num();
  for (const Point_2 &p : ring) {
    result.positions.push_back(float(CGAL::to_double(p.x())));
    result.positions.push_back(float(CGAL::to_double(p.y())));
    result.positions.push_back(0.0f);
  }
  for (int i = 0; i < int(ring.size()); i++) {
    result.corner_verts.push_back(base + i);
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
}

static bool point_on_seg_xy(double px,
                            double py,
                            double ax,
                            double ay,
                            double bx,
                            double by,
                            double tol2)
{
  const double abx = bx - ax;
  const double aby = by - ay;
  const double apx = px - ax;
  const double apy = py - ay;
  const double ab2 = abx * abx + aby * aby;
  if (ab2 < 1e-30) {
    return apx * apx + apy * apy <= tol2;
  }
  const double t = (apx * abx + apy * aby) / ab2;
  if (t < -1e-6 || t > 1.0 + 1e-6) {
    return false;
  }
  const double dx = px - (ax + t * abx);
  const double dy = py - (ay + t * aby);
  return dx * dx + dy * dy <= tol2;
}

/* Directed ring edges + spatial bins. Crossing L→R of he src→tgt:
 * Δw = −count(same dir) + count(opposite). Unbounded w = 0; inside iff w != 0. */
struct DirSegIndex {
  struct Seg {
    double ax = 0, ay = 0, bx = 0, by = 0;
    int count = 0;
  };
  std::vector<Seg> segs;
  double ox = 0, oy = 0, invx = 1, invy = 1;
  int gw = 1, gh = 1;
  std::vector<std::vector<int>> bins;

  int ix(double x) const
  {
    return std::clamp(int(std::floor((x - ox) * invx)), 0, gw - 1);
  }
  int iy(double y) const
  {
    return std::clamp(int(std::floor((y - oy) * invy)), 0, gh - 1);
  }

  void build(const std::vector<Ring2> &rings)
  {
    std::map<std::pair<Point_2, Point_2>, int> cnt;
    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (const Ring2 &r : rings) {
      const int n = int(r.pts.size());
      for (int i = 0; i < n; i++) {
        const Point_2 &a = r.pts[size_t(i)];
        const Point_2 &b = r.pts[size_t((i + 1) % n)];
        if (a == b) {
          continue;
        }
        cnt[std::make_pair(a, b)]++;
        const double ax = CGAL::to_double(a.x());
        const double ay = CGAL::to_double(a.y());
        const double bx = CGAL::to_double(b.x());
        const double by = CGAL::to_double(b.y());
        minx = std::min(minx, std::min(ax, bx));
        miny = std::min(miny, std::min(ay, by));
        maxx = std::max(maxx, std::max(ax, bx));
        maxy = std::max(maxy, std::max(ay, by));
      }
    }
    segs.clear();
    segs.reserve(cnt.size());
    for (const auto &kv : cnt) {
      Seg s;
      s.ax = CGAL::to_double(kv.first.first.x());
      s.ay = CGAL::to_double(kv.first.first.y());
      s.bx = CGAL::to_double(kv.first.second.x());
      s.by = CGAL::to_double(kv.first.second.y());
      s.count = kv.second;
      segs.push_back(s);
    }
    ox = minx;
    oy = miny;
    const double dx = std::max(maxx - minx, 1e-18);
    const double dy = std::max(maxy - miny, 1e-18);
    gw = segs.empty() ? 1 : 48;
    gh = segs.empty() ? 1 : 48;
    invx = double(gw) / dx;
    invy = double(gh) / dy;
    bins.assign(size_t(gw * gh), {});
    for (int i = 0; i < int(segs.size()); i++) {
      const Seg &s = segs[size_t(i)];
      const int x0 = ix(s.ax);
      const int y0 = iy(s.ay);
      const int x1 = ix(s.bx);
      const int y1 = iy(s.by);
      const int xa = std::min(x0, x1);
      const int xb = std::max(x0, x1);
      const int ya = std::min(y0, y1);
      const int yb = std::max(y0, y1);
      for (int y = ya; y <= yb; y++) {
        for (int x = xa; x <= xb; x++) {
          bins[size_t(y * gw + x)].push_back(i);
        }
      }
    }
  }

  int crossing_delta(const Point_2 &src, const Point_2 &tgt, double tol2) const
  {
    if (segs.empty()) {
      return 0;
    }
    const double sx = CGAL::to_double(src.x());
    const double sy = CGAL::to_double(src.y());
    const double tx = CGAL::to_double(tgt.x());
    const double ty = CGAL::to_double(tgt.y());
    const double mx = 0.5 * (sx + tx);
    const double my = 0.5 * (sy + ty);
    const double hx = tx - sx;
    const double hy = ty - sy;
    const std::vector<int> &bin = bins[size_t(iy(my) * gw + ix(mx))];
    int delta = 0;
    for (int idx : bin) {
      const Seg &s = segs[size_t(idx)];
      if (!point_on_seg_xy(mx, my, s.ax, s.ay, s.bx, s.by, tol2)) {
        continue;
      }
      const double dot = hx * (s.bx - s.ax) + hy * (s.by - s.ay);
      if (dot > 0.0) {
        delta -= s.count;
      }
      else if (dot < 0.0) {
        delta += s.count;
      }
    }
    return delta;
  }
};

static void flood_inside(Arr &arr,
                         const DirSegIndex &directed,
                         double tol2,
                         std::map<Arr::Face_handle, bool> &inside)
{
  inside.clear();
  std::map<Arr::Face_handle, bool> seen;
  std::map<Arr::Face_handle, int> wind;
  std::queue<Arr::Face_handle> q;
  Arr::Face_handle ub = arr.unbounded_face();
  wind[ub] = 0;
  inside[ub] = false;
  seen[ub] = true;
  q.push(ub);

  auto walk_ccb = [&](Arr::Face_handle f, Arr::Ccb_halfedge_circulator circ) {
    auto cur = circ;
    do {
      Arr::Face_handle g = cur->twin()->face();
      if (!seen[g]) {
        const int dw = directed.crossing_delta(
            cur->source()->point(), cur->target()->point(), tol2);
        const int wg = wind[f] + dw;
        wind[g] = wg;
        inside[g] = (wg != 0);
        seen[g] = true;
        q.push(g);
      }
      ++cur;
    } while (cur != circ);
  };

  while (!q.empty()) {
    Arr::Face_handle f = q.front();
    q.pop();
    if (f->has_outer_ccb()) {
      walk_ccb(f, f->outer_ccb());
    }
    for (auto hit = f->holes_begin(); hit != f->holes_end(); ++hit) {
      walk_ccb(f, *hit);
    }
  }
  for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
    if (seen.find(fit) == seen.end()) {
      inside[fit] = false;
    }
  }
}

static bool emit_keep_region(Arr &arr,
                             const std::map<Arr::Face_handle, bool> &keep,
                             MeshResult &result)
{
  std::map<Arr::Vertex_handle, int> vmap;
  std::map<Point_2, int> p2i;
  for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {
    const int idx = result.verts_num();
    vmap[vit] = idx;
    p2i[vit->point()] = idx;
    result.positions.push_back(float(CGAL::to_double(vit->point().x())));
    result.positions.push_back(float(CGAL::to_double(vit->point().y())));
    result.positions.push_back(0.0f);
  }

  using Vb = CGAL::Triangulation_vertex_base_2<Kernel>;
  using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
  using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
  using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, CGAL::Exact_predicates_tag>;

  const int max_steps = int(arr.number_of_halfedges()) + 8;
  CgalThrowGuard guard;
  for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
    if (!is_keep(keep, fit)) {
      continue;
    }
    try {
      if (fit->holes_begin() == fit->holes_end()) {
        emit_welded_cell(result, vmap, fit->outer_ccb(), 0, max_steps);
        continue;
      }
      /* Keep-face with holes: fill outer minus holes so holes are actually empty. */
      CDT cdt;
      auto insert_ccb = [&](Arr::Ccb_halfedge_circulator circ) {
        std::vector<typename CDT::Vertex_handle> vhs;
        auto cur = circ;
        int steps = 0;
        do {
          vhs.push_back(cdt.insert(cur->source()->point()));
          ++cur;
        } while (cur != circ && ++steps < max_steps);
        if (steps >= max_steps || vhs.size() < 3) {
          return;
        }
        const size_t n = vhs.size();
        for (size_t i = 0; i < n; i++) {
          if (vhs[i] != vhs[(i + 1) % n]) {
            cdt.insert_constraint(vhs[i], vhs[(i + 1) % n]);
          }
        }
      };
      insert_ccb(fit->outer_ccb());
      for (auto hit = fit->holes_begin(); hit != fit->holes_end(); ++hit) {
        insert_ccb(*hit);
      }
      if (cdt.dimension() != 2) {
        emit_welded_cell(result, vmap, fit->outer_ccb(), 0, max_steps);
        continue;
      }
      std::map<typename CDT::Face_handle, bool> in_domain_map;
      boost::associative_property_map<std::map<typename CDT::Face_handle, bool>> in_domain(
          in_domain_map);
      CGAL::mark_domain_in_triangulation(cdt, in_domain);
      bool any = false;
      for (auto cfit = cdt.finite_faces_begin(); cfit != cdt.finite_faces_end(); ++cfit) {
        if (!in_domain_map[cfit]) {
          continue;
        }
        int ids[3];
        bool ok = true;
        for (int i = 0; i < 3; i++) {
          auto pit = p2i.find(cfit->vertex(i)->point());
          if (pit == p2i.end()) {
            ok = false;
            break;
          }
          ids[i] = pit->second;
        }
        if (!ok) {
          continue;
        }
        if (result.face_offsets.empty()) {
          result.face_offsets.push_back(0);
        }
        result.corner_verts.push_back(ids[0]);
        result.corner_verts.push_back(ids[1]);
        result.corner_verts.push_back(ids[2]);
        result.face_offsets.push_back(int(result.corner_verts.size()));
        result.face_tag.push_back(0);
        any = true;
      }
      if (!any) {
        emit_welded_cell(result, vmap, fit->outer_ccb(), 0, max_steps);
      }
    }
    catch (...) {
      try {
        emit_welded_cell(result, vmap, fit->outer_ccb(), 0, max_steps);
      }
      catch (...) {
      }
    }
  }
  return result.faces_num() > 0;
}

static bool extract_outline(Arr &arr,
                            const std::map<Arr::Face_handle, bool> &keep,
                            MeshResult &result)
{
  std::set<Arr::Halfedge_handle> seen;
  const int max_steps = int(arr.number_of_halfedges()) + 8;
  std::vector<std::vector<Point_2>> outers;
  std::vector<std::vector<Point_2>> holes;
  for (auto he = arr.halfedges_begin(); he != arr.halfedges_end(); ++he) {
    if (!is_keep(keep, he->face()) || is_keep(keep, he->twin()->face())) {
      continue;
    }
    if (seen.count(he)) {
      continue;
    }
    std::vector<Point_2> cycle;
    auto cur = he;
    int steps = 0;
    do {
      seen.insert(cur);
      cycle.push_back(cur->source()->point());
      cur = next_outline(cur, keep);
    } while (cur != he && ++steps < max_steps);
    if (cur != he || cycle.size() < 3) {
      continue;
    }
    clean_ring(cycle);
    cycle = collapse_collinear(std::move(cycle));
    if (cycle.size() < 3) {
      continue;
    }
    const double area = signed_area(cycle);
    if (std::abs(area) <= 1e-16) {
      continue;
    }
    if (area > 0.0) {
      outers.push_back(std::move(cycle));
    }
    else {
      std::reverse(cycle.begin(), cycle.end());
      holes.push_back(std::move(cycle));
    }
  }
  for (const auto &r : outers) {
    emit_isolated_ring(result, r, 0);
  }
  for (const auto &r : holes) {
    emit_isolated_ring(result, r, 1);
  }
  return result.faces_num() > 0;
}

}  // namespace

MeshResult mesh_arrangement_2(const MeshIn &mesh)
{
  MeshResult result;
  try {
    if (!mesh.positions || mesh.verts_num < 2) {
      result.error = "Find All Cells 2D needs edges";
      return result;
    }
    const double scale = snap_scale_xy(mesh);
    std::vector<Segment_2> segs;
    collect_all_xy_edges(mesh, scale, segs);
    dedup_segments(segs);
    if (segs.empty()) {
      result.error = "Find All Cells 2D: no usable edges";
      return result;
    }

    Arr arr;
    if (!build_arr(segs, arr)) {
      result.error = "Find All Cells 2D failed to insert edges";
      return result;
    }

    std::map<Arr::Vertex_handle, int> vmap;
    std::map<Point_2, int> p2i;
    for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {
      const int idx = result.verts_num();
      vmap[vit] = idx;
      p2i[vit->point()] = idx;
      result.positions.push_back(float(CGAL::to_double(vit->point().x())));
      result.positions.push_back(float(CGAL::to_double(vit->point().y())));
      result.positions.push_back(0.0f);
    }

    using Vb = CGAL::Triangulation_vertex_base_2<Kernel>;
    using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
    using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, CGAL::Exact_predicates_tag>;

    const int max_steps = int(arr.number_of_halfedges()) + 8;
    int tag = 0;
    CgalThrowGuard guard;
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
      try {
        if (fit->is_unbounded() || !fit->has_outer_ccb()) {
          continue;
        }
        if (fit->holes_begin() == fit->holes_end()) {
          emit_welded_cell(result, vmap, fit->outer_ccb(), tag++, max_steps);
          continue;
        }
        /* Face with holes: fill the loop-minus-holes so every closed cell exists. */
        CDT cdt;
        auto insert_ccb = [&](Arr::Ccb_halfedge_circulator circ) {
          std::vector<typename CDT::Vertex_handle> vhs;
          auto cur = circ;
          int steps = 0;
          do {
            vhs.push_back(cdt.insert(cur->source()->point()));
            ++cur;
          } while (cur != circ && ++steps < max_steps);
          if (steps >= max_steps) {
            return;
          }
          const size_t n = vhs.size();
          for (size_t i = 0; i < n; i++) {
            if (vhs[i] != vhs[(i + 1) % n]) {
              cdt.insert_constraint(vhs[i], vhs[(i + 1) % n]);
            }
          }
        };
        insert_ccb(fit->outer_ccb());
        for (auto hit = fit->holes_begin(); hit != fit->holes_end(); ++hit) {
          insert_ccb(*hit);
        }
        if (cdt.dimension() != 2) {
          emit_welded_cell(result, vmap, fit->outer_ccb(), tag++, max_steps);
          continue;
        }
        std::map<typename CDT::Face_handle, bool> in_domain_map;
        boost::associative_property_map<std::map<typename CDT::Face_handle, bool>> in_domain(
            in_domain_map);
        CGAL::mark_domain_in_triangulation(cdt, in_domain);
        bool any = false;
        for (auto cfit = cdt.finite_faces_begin(); cfit != cdt.finite_faces_end(); ++cfit) {
          if (!in_domain_map[cfit]) {
            continue;
          }
          int ids[3];
          bool ok = true;
          for (int i = 0; i < 3; i++) {
            auto pit = p2i.find(cfit->vertex(i)->point());
            if (pit == p2i.end()) {
              ok = false;
              break;
            }
            ids[i] = pit->second;
          }
          if (!ok) {
            continue;
          }
          if (result.face_offsets.empty()) {
            result.face_offsets.push_back(0);
          }
          result.corner_verts.push_back(ids[0]);
          result.corner_verts.push_back(ids[1]);
          result.corner_verts.push_back(ids[2]);
          result.face_offsets.push_back(int(result.corner_verts.size()));
          result.face_tag.push_back(tag);
          any = true;
        }
        if (any) {
          tag++;
        }
        else {
          emit_welded_cell(result, vmap, fit->outer_ccb(), tag++, max_steps);
        }
      }
      catch (...) {
      }
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Find All Cells 2D: no closed loops";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Find All Cells 2D failed";
  }
  return result;
}

MeshResult mesh_boolean_ops_2(const MeshIn &mesh_a, const MeshIn &mesh_b, int mode)
{
  MeshResult result;
  mode = std::clamp(mode, 0, 3);
  try {
    const double scale = snap_scale_xy(mesh_a);
    std::vector<Ring2> rings_a;
    if (!collect_shape_rings(mesh_a, scale, rings_a)) {
      result.error =
          "Boolean Ops 2D: mesh A needs faces with non-zero XY area (flatten onto XY)";
      return result;
    }
    std::vector<Ring2> rings_b;
    const bool has_b = mesh_b.positions && mesh_b.verts_num >= 3 && mesh_b.corner_verts &&
                       collect_shape_rings(mesh_b, scale, rings_b);

    std::vector<Segment_2> segs;
    collect_ring_segments(rings_a, segs);
    if (has_b) {
      collect_ring_segments(rings_b, segs);
    }
    dedup_segments(segs);
    if (segs.empty()) {
      result.error = "Boolean Ops 2D: no usable 2D edges";
      return result;
    }

    Arr arr;
    if (!build_arr(segs, arr)) {
      result.error = "Boolean Ops 2D failed to overlay 2D faces";
      return result;
    }

    DirSegIndex dir_a;
    DirSegIndex dir_b;
    dir_a.build(rings_a);
    if (has_b) {
      dir_b.build(rings_b);
    }
    const double tol = std::max(2.0 / std::max(scale, 1.0), 1e-9);
    const double tol2 = tol * tol;

    std::map<Arr::Face_handle, bool> in_a;
    std::map<Arr::Face_handle, bool> in_b;
    /* Non-zero winding flood (not even-odd). Overlapping same-orientation
     * faces stay inside so Difference punches those regions. */
    flood_inside(arr, dir_a, tol2, in_a);
    if (has_b) {
      flood_inside(arr, dir_b, tol2, in_b);
    }
    /* Fallback if flood found nothing (open / non-closed rings). */
    bool any_a = false;
    for (const auto &kv : in_a) {
      if (kv.second) {
        any_a = true;
        break;
      }
    }
    if (!any_a) {
      mark_faces(arr, rings_a, in_a);
    }
    if (has_b) {
      bool any_b = false;
      for (const auto &kv : in_b) {
        if (kv.second) {
          any_b = true;
          break;
        }
      }
      if (!any_b) {
        mark_faces(arr, rings_b, in_b);
      }
    }

    std::map<Arr::Face_handle, bool> keep;
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
      const bool a = is_keep(in_a, fit);
      const bool b = has_b && is_keep(in_b, fit);
      bool k = a;
      if (has_b) {
        switch (mode) {
          case 0:
            k = a || b;
            break;
          case 1:
            k = a && b;
            break;
          case 2:
            k = a && !b;
            break;
          default:
            k = a != b;
            break;
        }
      }
      keep[fit] = k;
    }

    /* Fill kept arrangement faces (holes stay empty — not extra covering n-gons). */
    if (!emit_keep_region(arr, keep, result)) {
      result.error = "Boolean Ops 2D: empty 2D region (no silhouette)";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = std::string("Boolean Ops 2D failed: ") + e.what();
  }
  catch (...) {
    result.error = "Boolean Ops 2D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
