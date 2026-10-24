/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#include <CGAL/boost/graph/helpers.h>
#include <CGAL/convex_hull_3.h>

#include <algorithm>
#include <array>
#include <map>
#include <numeric>
#include <vector>

namespace blender::cgal_bridge {

TriangleMeshResult convex_hull_3(const float *positions, int points_num)
{
  TriangleMeshResult result;
  if (!positions || points_num < 4) {
    result.error = "Need at least 4 points for a 3D convex hull";
    return result;
  }

  try {
    std::vector<Point_3> cgal_pts;
    cgal_pts.reserve(size_t(points_num));
    for (int i = 0; i < points_num; i++) {
      const float *p = positions + i * 3;
      cgal_pts.emplace_back(p[0], p[1], p[2]);
    }

    Surface_mesh sm;
    CGAL::convex_hull_3(cgal_pts.begin(), cgal_pts.end(), sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "CGAL convex_hull_3 failed";
    result.ok = false;
  }
  return result;
}

MeshResult convex_hull_3_polygons(const float *positions, const int points_num)
{
  MeshResult result;
  if (!positions || points_num < 4) {
    result.error = "Need at least 4 points for a 3D convex hull";
    return result;
  }

  try {
    std::vector<Point_3> cgal_pts;
    cgal_pts.reserve(size_t(points_num));
    /* The hull's points are copies of input points, so they find their source index exactly. */
    std::map<std::array<double, 3>, int> src_of_point;
    for (int i = 0; i < points_num; i++) {
      const float *p = positions + i * 3;
      cgal_pts.emplace_back(p[0], p[1], p[2]);
      src_of_point.emplace(std::array<double, 3>{p[0], p[1], p[2]}, i);
    }

    Surface_mesh sm;
    CGAL::convex_hull_3(cgal_pts.begin(), cgal_pts.end(), sm);
    if (sm.number_of_faces() < 4 || !CGAL::is_closed(sm)) {
      result.error = "Points are flat or degenerate";
      return result;
    }

    /* Triangles in one plane form one polygon: join neighbors that are exactly coplanar. */
    std::vector<int> group(sm.number_of_faces());
    std::iota(group.begin(), group.end(), 0);
    auto find = [&](int f) {
      while (group[f] != f) {
        group[f] = group[group[f]];
        f = group[f];
      }
      return f;
    };
    for (const auto e : sm.edges()) {
      const auto h = sm.halfedge(e);
      const auto o = sm.opposite(h);
      if (CGAL::coplanar(sm.point(sm.source(h)),
                         sm.point(sm.target(h)),
                         sm.point(sm.target(sm.next(h))),
                         sm.point(sm.target(sm.next(o)))))
      {
        group[find(int(sm.face(h)))] = find(int(sm.face(o)));
      }
    }
    auto group_of = [&](const Surface_mesh::Halfedge_index h) { return find(int(sm.face(h))); };

    /* Each group's outline: start at a halfedge whose other side is another group and walk
     * around, rotating through the group's inner edges at every corner. */
    std::vector<bool> emitted(sm.number_of_faces(), false);
    std::map<Surface_mesh::Vertex_index, int> vert_index;
    size_t boundary_halfedges = 0;
    result.face_offsets.push_back(0);
    for (const auto h : sm.halfedges()) {
      const int g = group_of(h);
      if (g == group_of(sm.opposite(h))) {
        continue;
      }
      boundary_halfedges++;
      if (emitted[g]) {
        continue;
      }
      emitted[g] = true;
      auto cur = h;
      size_t guard = 0;
      do {
        const auto v = sm.source(cur);
        auto [it, added] = vert_index.emplace(v, int(vert_index.size()));
        if (added) {
          const Point_3 &p = sm.point(v);
          const auto src = src_of_point.find({p.x(), p.y(), p.z()});
          if (src == src_of_point.end()) {
            result.error = "Hull point is not an input point";
            return result;
          }
          result.positions.push_back(float(p.x()));
          result.positions.push_back(float(p.y()));
          result.positions.push_back(float(p.z()));
          result.vert_src0.push_back(src->second);
        }
        result.corner_verts.push_back(it->second);
        auto next = sm.next(cur);
        while (group_of(sm.opposite(next)) == g) {
          next = sm.next(sm.opposite(next));
        }
        cur = next;
        if (++guard > sm.number_of_halfedges()) {
          result.error = "Coplanar region outline does not close";
          return result;
        }
      } while (cur != h);
      result.face_offsets.push_back(int(result.corner_verts.size()));
    }
    if (boundary_halfedges != result.corner_verts.size()) {
      /* A group with more than one outline (it never is for a convex polytope). */
      result.error = "Coplanar region has several outlines";
      return result;
    }

    /* The hull is unique but CGAL's element order depends on memory addresses: sort vertices by
     * input index, start every polygon at its lowest vertex and sort the polygons. */
    const int verts_num = int(result.vert_src0.size());
    std::vector<int> order(verts_num);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](const int a, const int b) {
      return result.vert_src0[a] < result.vert_src0[b];
    });
    std::vector<int> new_index(verts_num);
    std::vector<float> positions(result.positions.size());
    std::vector<int> vert_src(verts_num);
    for (int i = 0; i < verts_num; i++) {
      new_index[order[i]] = i;
      std::copy_n(&result.positions[order[i] * 3], 3, &positions[i * 3]);
      vert_src[i] = result.vert_src0[order[i]];
    }
    std::vector<std::vector<int>> polygons;
    for (size_t f = 0; f + 1 < result.face_offsets.size(); f++) {
      std::vector<int> poly;
      for (int c = result.face_offsets[f]; c < result.face_offsets[f + 1]; c++) {
        poly.push_back(new_index[result.corner_verts[c]]);
      }
      std::rotate(poly.begin(), std::min_element(poly.begin(), poly.end()), poly.end());
      polygons.push_back(std::move(poly));
    }
    std::sort(polygons.begin(), polygons.end());
    result.positions = std::move(positions);
    result.vert_src0 = std::move(vert_src);
    result.corner_verts.clear();
    result.face_offsets.assign(1, 0);
    for (const std::vector<int> &poly : polygons) {
      result.corner_verts.insert(result.corner_verts.end(), poly.begin(), poly.end());
      result.face_offsets.push_back(int(result.corner_verts.size()));
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "CGAL convex_hull_3 failed";
    result.ok = false;
  }
  return result;
}

}  // namespace blender::cgal_bridge
