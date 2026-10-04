/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Triangulation_data_structure_3.h>
#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

TriangleMeshResult alpha_shape_3(const float *positions,
                                 int points_num,
                                 double alpha,
                                 bool use_optimal_alpha,
                                 int solid_components)
{
  TriangleMeshResult result;
  if (!positions || points_num < 4) {
    result.error = "Need at least 4 points for Alpha Shape 3D";
    return result;
  }

  try {
    using Vb = CGAL::Alpha_shape_vertex_base_3<Kernel>;
    using Fb = CGAL::Alpha_shape_cell_base_3<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_3<Vb, Fb>;
    using Triangulation = CGAL::Delaunay_triangulation_3<Kernel, Tds>;
    using Alpha_shape = CGAL::Alpha_shape_3<Triangulation>;

    std::vector<Point_3> cgal_pts;
    cgal_pts.reserve(size_t(points_num));
    for (int i = 0; i < points_num; i++) {
      const float *p = positions + i * 3;
      cgal_pts.emplace_back(p[0], p[1], p[2]);
    }

    /* REGULARIZED drops singular (dangling) faces -> fewer "broken" shreds. */
    Alpha_shape as(cgal_pts.begin(), cgal_pts.end(), 0, Alpha_shape::REGULARIZED);
    solid_components = std::max(solid_components, 1);

    if (use_optimal_alpha) {
      auto opt_it = as.find_optimal_alpha(solid_components);
      as.set_alpha(*opt_it);
      result.alpha_used = CGAL::to_double(*opt_it);
    }
    else {
      /* If user leaves Alpha at 0, estimate from median nearest-neighbor spacing. */
      double alpha_val = alpha;
      if (!(alpha_val > 0.0)) {
        /* crude bbox diagonal / 20 as fallback */
        double minx = cgal_pts[0].x(), maxx = minx;
        double miny = cgal_pts[0].y(), maxy = miny;
        double minz = cgal_pts[0].z(), maxz = minz;
        for (const Point_3 &p : cgal_pts) {
          minx = std::min(minx, p.x());
          maxx = std::max(maxx, p.x());
          miny = std::min(miny, p.y());
          maxy = std::max(maxy, p.y());
          minz = std::min(minz, p.z());
          maxz = std::max(maxz, p.z());
        }
        const double dx = maxx - minx, dy = maxy - miny, dz = maxz - minz;
        const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
        alpha_val = (diag > 0.0) ? (diag * diag) * 0.01 : 1.0;
      }
      as.set_alpha(Alpha_shape::NT(alpha_val));
      result.alpha_used = alpha_val;
    }

    std::vector<Alpha_shape::Facet> facets;
    /*
     * REGULAR facets form the solid/void interface. SINGULAR facets are dangling
     * and cause broken shreds — skip them (REGULARIZED mode already reduces them).
     */
    as.get_alpha_shape_facets(std::back_inserter(facets), Alpha_shape::REGULAR);

    std::vector<Point_3> soup_points;
    std::vector<std::vector<std::size_t>> soup_faces;
    std::map<Point_3, std::size_t> p2i;
    auto get_index = [&](const Point_3 &p) -> std::size_t {
      auto it = p2i.find(p);
      if (it != p2i.end()) {
        return it->second;
      }
      const std::size_t idx = soup_points.size();
      soup_points.push_back(p);
      p2i[p] = idx;
      return idx;
    };

    std::set<std::array<std::size_t, 3>> unique_tris;
    for (const Alpha_shape::Facet &f : facets) {
      const auto &cell = f.first;
      const int i = f.second;
      /* Prefer solid-side orientation when classification is available. */
      Point_3 p0 = cell->vertex((i + 1) & 3)->point();
      Point_3 p1 = cell->vertex((i + 2) & 3)->point();
      Point_3 p2 = cell->vertex((i + 3) & 3)->point();
      const Point_3 &opp = cell->vertex(i)->point();
      /*
       * If the cell is INTERIOR/SOLID-ish, put opposite vertex on the negative
       * side so the normal points outward of the solid. Fall back for exterior.
       */
      const auto cls = as.classify(cell);
      const bool want_opp_negative = (cls == Alpha_shape::INTERIOR);
      const CGAL::Orientation ori = CGAL::orientation(p0, p1, p2, opp);
      if (want_opp_negative) {
        if (ori != CGAL::NEGATIVE) {
          std::swap(p1, p2);
        }
      }
      else {
        if (ori != CGAL::POSITIVE) {
          std::swap(p1, p2);
        }
      }
      const std::size_t i0 = get_index(p0);
      const std::size_t i1 = get_index(p1);
      const std::size_t i2 = get_index(p2);
      if (i0 == i1 || i1 == i2 || i0 == i2) {
        continue;
      }
      std::array<std::size_t, 3> key = {i0, i1, i2};
      std::sort(key.begin(), key.end());
      if (!unique_tris.insert(key).second) {
        continue;
      }
      soup_faces.push_back({i0, i1, i2});
    }

    if (soup_points.empty() || soup_faces.empty()) {
      result.error =
          "Alpha shape produced no surface facets. Try manual Alpha around "
          "(point spacing)^2, denser points, or more solid components.";
      return result;
    }

    const double alpha_used_val = CGAL::to_double(as.get_alpha());

    /* Same robustness stack as Advancing Front / closer to Alpha Wrap usability. */
    try {
      PMP::repair_polygon_soup(soup_points, soup_faces);
    }
    catch (...) {
    }
    try {
      PMP::orient_polygon_soup(soup_points, soup_faces);
    }
    catch (...) {
    }

    Surface_mesh sm;
    bool rebuilt = false;
    try {
      PMP::polygon_soup_to_polygon_mesh(soup_points, soup_faces, sm);
      rebuilt = sm.number_of_faces() > 0;
    }
    catch (...) {
      rebuilt = false;
    }
    if (rebuilt) {
      try {
        if (CGAL::is_closed(sm)) {
          PMP::orient_to_bound_a_volume(sm);
        }
      }
      catch (...) {
      }
      MeshResult cleaned = surface_mesh_to_result(sm);
      if (cleaned.ok) {
        cleaned.alpha_used = alpha_used_val;
        return cleaned;
      }
    }

    /* Soup fallback. */
    result.positions.clear();
    result.corner_verts.clear();
    result.face_offsets.clear();
    for (const Point_3 &p : soup_points) {
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
    }
    for (const auto &poly : soup_faces) {
      if (poly.size() < 3) {
        continue;
      }
      for (size_t i = 1; i + 1 < poly.size(); i++) {
        result.corner_verts.push_back(int(poly[0]));
        result.corner_verts.push_back(int(poly[i]));
        result.corner_verts.push_back(int(poly[i + 1]));
      }
    }
    if (result.corner_verts.empty()) {
      result.error =
          "Alpha shape produced no surface facets. Try manual Alpha around "
          "(point spacing)^2, denser points, or more solid components.";
      return result;
    }
    result.alpha_used = alpha_used_val;
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL Alpha Shape 3D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
