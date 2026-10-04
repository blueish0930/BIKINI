/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_deformation.h>
#include <CGAL/Surface_mesh_parameterization/Error_code.h>
#include <CGAL/Surface_mesh_parameterization/ARAP_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Barycentric_mapping_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Circular_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_authalic_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_conformal_map_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Iterative_authalic_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/LSCM_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Mean_value_coordinates_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/boost/graph/Seam_mesh.h>
#include <boost/property_map/property_map.hpp>
#include <CGAL/Shape_detection/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Point_set/K_neighbor_query.h>
#include <CGAL/Shape_detection/Region_growing/Point_set/Least_squares_plane_fit_region.h>
#include <CGAL/Shape_detection/Region_growing/Point_set/Least_squares_plane_fit_sorting.h>
#include <CGAL/Shape_detection/Region_growing/Point_set/Sphere_neighbor_query.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Random.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <boost/graph/graph_traits.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

/* Nth_of_tuple_property_map lives in CGAL/property_map.h (already included). */

namespace blender::cgal_bridge {
namespace {

namespace PMP = CGAL::Polygon_mesh_processing;
namespace SMP = CGAL::Surface_mesh_parameterization;

using Point_2 = Kernel::Point_2;
using Vector_3 = Kernel::Vector_3;

template<CGAL::Deformation_algorithm_tag TAG>
static MeshResult arap_deform_tag(Surface_mesh &sm,
                                  const uint8_t *roi_mask,
                                  const uint8_t *control_mask,
                                  const float *target_xyz,
                                  int iterations,
                                  double tolerance)
{
  MeshResult result;
  using Deform = CGAL::Surface_mesh_deformation<Surface_mesh, CGAL::Default, CGAL::Default, TAG>;
  Deform deform(sm);

  std::vector<typename Surface_mesh::Vertex_index> controls;
  controls.reserve(sm.number_of_vertices());

  for (const auto v : sm.vertices()) {
    const int i = int(v.idx());
    if (roi_mask && roi_mask[i]) {
      deform.insert_roi_vertex(v);
    }
    if (control_mask && control_mask[i]) {
      deform.insert_control_vertex(v);
      controls.push_back(v);
    }
  }

  if (controls.empty()) {
    result.error = "ARAP Deform needs at least one control vertex";
    return result;
  }

  if (!deform.preprocess()) {
    result.error = "ARAP preprocess failed (check connectivity / ROI)";
    return result;
  }

  for (const auto v : controls) {
    const int i = int(v.idx());
    const Point_3 t(target_xyz[i * 3 + 0], target_xyz[i * 3 + 1], target_xyz[i * 3 + 2]);
    deform.set_target_position(v, t);
  }

  const unsigned int iters = unsigned(std::max(1, iterations));
  const double tol = std::max(0.0, tolerance);
  deform.deform(iters, tol);

  if (!surface_mesh_positions_to_buffer(sm, result.positions)) {
    result.error = "Failed to read deformed positions";
    return result;
  }
  result.positions_only = true;
  result.ok = true;
  return result;
}

/* Point + normal + original index (survives RANSAC in-place reorder). */
using IndexedPwn = std::tuple<Point_3, Vector_3, int>;

/** Lvalue UV property map for Seam_mesh parameterization (MSVC: not a local class). */
template<typename SM_vertex> struct LscmUVMap {
  struct Hash {
    std::size_t operator()(const SM_vertex &v) const
    {
      return hash_value(v);
    }
  };
  std::unordered_map<SM_vertex, Point_2, Hash> *storage = nullptr;
  using key_type = SM_vertex;
  using value_type = Point_2;
  using reference = Point_2 &;
  using category = boost::lvalue_property_map_tag;

  reference operator[](const key_type &k) const
  {
    auto it = storage->find(k);
    if (it == storage->end()) {
      it = storage->emplace(k, Point_2(0, 0)).first;
    }
    return it->second;
  }
};

template<typename SM_vertex>
typename LscmUVMap<SM_vertex>::reference get(const LscmUVMap<SM_vertex> &m,
                                             const typename LscmUVMap<SM_vertex>::key_type &k)
{
  return m[k];
}

template<typename SM_vertex>
void put(const LscmUVMap<SM_vertex> &m,
         const typename LscmUVMap<SM_vertex>::key_type &k,
         const typename LscmUVMap<SM_vertex>::value_type &v)
{
  (*m.storage)[k] = v;
}

/** Point+normal pair for point-set region growing. */
using PwnRG = std::pair<Point_3, Vector_3>;
using PwnRGIt = std::vector<PwnRG>::const_iterator;

struct PwnRGPointMap {
  using key_type = PwnRGIt;
  using value_type = Point_3;
  using reference = const Point_3 &;
  using category = boost::readable_property_map_tag;
};

struct PwnRGNormalMap {
  using key_type = PwnRGIt;
  using value_type = Vector_3;
  using reference = const Vector_3 &;
  using category = boost::readable_property_map_tag;
};

inline const Point_3 &get(const PwnRGPointMap & /*m*/, const PwnRGIt it)
{
  return it->first;
}
inline const Vector_3 &get(const PwnRGNormalMap & /*m*/, const PwnRGIt it)
{
  return it->second;
}

}  // namespace

MeshResult mesh_arap_deform(const MeshIn &mesh,
                            const uint8_t *roi_mask,
                            const uint8_t *control_mask,
                            const float *target_xyz,
                            int algorithm,
                            int iterations,
                            double tolerance)
{
  MeshResult result;
  if (mesh.faces_num <= 0 || mesh.verts_num < 3 || !target_xyz || !control_mask) {
    result.error = "ARAP Deform needs a mesh, control mask, and targets";
    return result;
  }
  Surface_mesh sm;
  std::string error;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, false)) {
    result.error = error.empty() ? "Failed to import mesh" : error;
    return result;
  }
  try {
    PMP::triangulate_faces(sm);
    if (algorithm == 0) {
      return arap_deform_tag<CGAL::ORIGINAL_ARAP>(
          sm, roi_mask, control_mask, target_xyz, iterations, tolerance);
    }
    return arap_deform_tag<CGAL::SPOKES_AND_RIMS>(
        sm, roi_mask, control_mask, target_xyz, iterations, tolerance);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "ARAP deformation failed";
  }
  return result;
}

bool mesh_parameterize_uv_corners(const MeshIn &mesh,
                                  const int *edge_v0,
                                  const int *edge_v1,
                                  int edges_num,
                                  const uint8_t *seam_edge,
                                  const uint8_t *face_selected,
                                  bool normalize,
                                  int method,
                                  int energy_iterations,
                                  double lambda,
                                  std::vector<float> &out_corner_uv,
                                  std::string &error)
{
  out_corner_uv.assign(size_t(std::max(0, mesh.corners_num)) * 2, 0.0f);
  if (mesh.faces_num <= 0 || mesh.verts_num < 3 || mesh.corners_num < 3) {
    error = "UV parameterize needs a non-empty mesh";
    return false;
  }

  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }

  try {
    /* Fan-triangulate; vertex indices stay 0..verts_num-1 (no new verts). */
    PMP::triangulate_faces(sm);
    if (sm.number_of_faces() == 0 || sm.number_of_vertices() == 0) {
      error = "Empty mesh after triangulation";
      return false;
    }

    using vertex_descriptor = boost::graph_traits<Surface_mesh>::vertex_descriptor;
    using halfedge_descriptor = boost::graph_traits<Surface_mesh>::halfedge_descriptor;
    using edge_descriptor = boost::graph_traits<Surface_mesh>::edge_descriptor;
    using face_descriptor = boost::graph_traits<Surface_mesh>::face_descriptor;

    std::vector<bool> face_ok(size_t(mesh.faces_num), true);
    if (face_selected) {
      for (int f = 0; f < mesh.faces_num; f++) {
        face_ok[size_t(f)] = face_selected[f] != 0;
      }
    }

    auto f_src_opt = sm.property_map<face_descriptor, int>(PROP_F_SRC);
    /* Parent original face for a triangulated face, or -1 if unknown.
     * IMPORTANT: do NOT fall back to fd.idx() — after triangulation indices can
     * exceed mesh.faces_num and would drop half the faces (UV collapse). */
    auto orig_face_of = [&](face_descriptor fd) -> int {
      if (f_src_opt.has_value()) {
        const int s = f_src_opt.value()[fd];
        if (s >= 0 && s < mesh.faces_num) {
          return s;
        }
      }
      return -1;
    };
    auto tri_face_selected = [&](face_descriptor fd) -> bool {
      const int of = orig_face_of(fd);
      if (of >= 0) {
        return face_ok[size_t(of)];
      }
      /* No parent map: keep the triangle (full mesh path). */
      return true;
    };

    /* Face-connected components on the triangulated mesh.
     * CGAL parameterize() only handles ONE connected patch at a time; we must
     * unwrap each component separately or the rest collapse to (0,0). */
    auto fcc_pair = sm.add_property_map<face_descriptor, std::size_t>("f:cc", 0);
    auto fcc = fcc_pair.first;
    const std::size_t n_cc = PMP::connected_components(sm, fcc);
    if (n_cc == 0) {
      error = "No face components";
      return false;
    }

    std::vector<std::vector<face_descriptor>> faces_by_cc(n_cc);
    for (const face_descriptor fd : faces(sm)) {
      if (!tri_face_selected(fd)) {
        continue;
      }
      faces_by_cc[fcc[fd]].push_back(fd);
    }

    int n_islands = 0;
    for (std::size_t cc = 0; cc < n_cc; cc++) {
      if (!faces_by_cc[cc].empty()) {
        n_islands++;
      }
    }
    if (n_islands == 0) {
      error = "No selected faces to parameterize";
      return false;
    }

    /* Seams as undirected edge keys. */
    std::set<std::pair<int, int>> seam_keys;
    if (seam_edge && edge_v0 && edge_v1 && edges_num > 0) {
      for (int i = 0; i < edges_num; i++) {
        if (!seam_edge[i]) {
          continue;
        }
        const int a = edge_v0[i];
        const int b = edge_v1[i];
        if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
          continue;
        }
        seam_keys.insert({std::min(a, b), std::max(a, b)});
      }
    }

    auto parameterize_one = [&](Surface_mesh &patch,
                                std::vector<float> &pu,
                                std::vector<float> &pv) -> bool {
      const int nv = int(patch.number_of_vertices());
      pu.assign(size_t(nv), 0.0f);
      pv.assign(size_t(nv), 0.0f);
      const auto border = PMP::longest_border(patch);
      halfedge_descriptor bhd = border.first;
      if (bhd == Surface_mesh::null_halfedge() || !is_border(bhd, patch)) {
        return false;
      }
      auto uv_pair = patch.add_property_map<vertex_descriptor, Point_2>("v:uv", Point_2(0, 0));
      auto uv_pmap = uv_pair.first;
      /* 0=LSCM 1=ARAP 2=Discrete Conformal 3=Mean Value 4=Discrete Authalic
       * 5=Barycentric 6=Iterative Authalic */
      const int meth = (method < 0 || method > 6) ? 0 : method;
      SMP::Error_code err = SMP::ERROR_EMPTY_MESH;
      if (meth == 1) {
        const double lam = (lambda > 0.0) ? lambda : 1000.0;
        const unsigned int iters = unsigned(energy_iterations > 0 ? energy_iterations : 50);
        using Parameterizer = SMP::ARAP_parameterizer_3<Surface_mesh>;
        err = SMP::parameterize(
            patch,
            Parameterizer(typename Parameterizer::Border_parameterizer(),
                          typename Parameterizer::Solver_traits(),
                          lam,
                          iters),
            bhd,
            uv_pmap);
      }
      else if (meth == 2) {
        using Border = SMP::Circular_border_arc_length_parameterizer_3<Surface_mesh>;
        using Parameterizer = SMP::Discrete_conformal_map_parameterizer_3<Surface_mesh, Border>;
        err = SMP::parameterize(patch, Parameterizer(), bhd, uv_pmap);
      }
      else if (meth == 3) {
        using Border = SMP::Circular_border_arc_length_parameterizer_3<Surface_mesh>;
        using Parameterizer = SMP::Mean_value_coordinates_parameterizer_3<Surface_mesh, Border>;
        err = SMP::parameterize(patch, Parameterizer(), bhd, uv_pmap);
      }
      else if (meth == 4) {
        using Border = SMP::Circular_border_arc_length_parameterizer_3<Surface_mesh>;
        using Parameterizer = SMP::Discrete_authalic_parameterizer_3<Surface_mesh, Border>;
        err = SMP::parameterize(patch, Parameterizer(), bhd, uv_pmap);
      }
      else if (meth == 5) {
        using Border = SMP::Circular_border_arc_length_parameterizer_3<Surface_mesh>;
        using Parameterizer = SMP::Barycentric_mapping_parameterizer_3<Surface_mesh, Border>;
        err = SMP::parameterize(patch, Parameterizer(), bhd, uv_pmap);
      }
      else if (meth == 6) {
        using Parameterizer = SMP::Iterative_authalic_parameterizer_3<Surface_mesh>;
        Parameterizer param;
        const unsigned int iters = unsigned(energy_iterations > 0 ? energy_iterations : 15);
        err = param.parameterize(patch, bhd, uv_pmap, iters);
      }
      else {
        using Parameterizer = SMP::LSCM_parameterizer_3<Surface_mesh>;
        err = SMP::parameterize(patch, Parameterizer(), bhd, uv_pmap);
      }
      if (err != SMP::OK) {
        return false;
      }
      for (const vertex_descriptor vd : vertices(patch)) {
        const int i = int(vd.idx());
        if (i < 0 || i >= nv) {
          continue;
        }
        const Point_2 uv = uv_pmap[vd];
        pu[size_t(i)] = float(uv.x());
        pv[size_t(i)] = float(uv.y());
      }
      return true;
    };

    /* One parameterized island before packing.
     * UVs are translated to origin and scaled so UV area ≈ 3D surface area
     * (uniform texel density across islands of different mesh sizes). */
    struct IslandUV {
      std::vector<int> patch_to_orig;
      std::vector<float> u;
      std::vector<float> v;
      std::vector<uint8_t> valid;
      float width = 0.0f;
      float height = 0.0f;
      float ox = 0.0f; /* pack offset */
      float oy = 0.0f;
    };

    /* Translate UV to origin, scale by sqrt(A3d / Auv), fill width/height. */
    auto finalize_island_uv = [](Surface_mesh &patch,
                                 const std::vector<int> &patch_to_orig,
                                 std::vector<float> &pu,
                                 std::vector<float> &pv,
                                 const std::vector<int> *valid_cnt, /* null = all verts valid */
                                 IslandUV &out) -> bool {
      const std::size_t n = pu.size();
      if (n == 0 || n != pv.size()) {
        return false;
      }
      auto vert_ok = [&](std::size_t i) -> bool {
        if (i >= n) {
          return false;
        }
        if (valid_cnt && (i >= valid_cnt->size() || (*valid_cnt)[i] <= 0)) {
          return false;
        }
        return true;
      };

      float umin = 1e30f, umax = -1e30f, vmin = 1e30f, vmax = -1e30f;
      int n_valid = 0;
      for (std::size_t i = 0; i < n; i++) {
        if (!vert_ok(i)) {
          continue;
        }
        umin = std::min(umin, pu[i]);
        umax = std::max(umax, pu[i]);
        vmin = std::min(vmin, pv[i]);
        vmax = std::max(vmax, pv[i]);
        n_valid++;
      }
      if (n_valid == 0) {
        return false;
      }

      /* 3D surface area of the patch. */
      double area_3d = 0.0;
      try {
        area_3d = CGAL::to_double(PMP::area(patch));
      }
      catch (...) {
        area_3d = 0.0;
      }
      if (!(area_3d > 0.0)) {
        /* Fallback: sum triangle areas from 3D points. */
        for (const face_descriptor fd : faces(patch)) {
          halfedge_descriptor hd = halfedge(fd, patch);
          const Point_3 &p0 = patch.point(target(hd, patch));
          const Point_3 &p1 = patch.point(target(next(hd, patch), patch));
          const Point_3 &p2 = patch.point(target(next(next(hd, patch), patch), patch));
          const Vector_3 c = CGAL::cross_product(p1 - p0, p2 - p0);
          area_3d += 0.5 * std::sqrt(CGAL::to_double(c.squared_length()));
        }
      }

      /* UV area after translation (scale is applied next). */
      double area_uv = 0.0;
      for (const face_descriptor fd : faces(patch)) {
        halfedge_descriptor hd = halfedge(fd, patch);
        const int i0 = int(target(hd, patch).idx());
        const int i1 = int(target(next(hd, patch), patch).idx());
        const int i2 = int(target(next(next(hd, patch), patch), patch).idx());
        if (i0 < 0 || i1 < 0 || i2 < 0 || !vert_ok(size_t(i0)) || !vert_ok(size_t(i1)) ||
            !vert_ok(size_t(i2)))
        {
          continue;
        }
        const float u0 = pu[size_t(i0)] - umin;
        const float v0 = pv[size_t(i0)] - vmin;
        const float u1 = pu[size_t(i1)] - umin;
        const float v1 = pv[size_t(i1)] - vmin;
        const float u2 = pu[size_t(i2)] - umin;
        const float v2 = pv[size_t(i2)] - vmin;
        area_uv += 0.5 * std::abs(double(u0 * (v1 - v2) + u1 * (v2 - v0) + u2 * (v0 - v1)));
      }

      /* Uniform scale: UV area matches 3D area → same texel density per island. */
      float sc = 1.0f;
      if (area_uv > 1e-30 && area_3d > 1e-30) {
        sc = float(std::sqrt(area_3d / area_uv));
      }
      else {
        /* Degenerate UV: fall back to bbox unit-fit (still better than crash). */
        const float du = std::max(1e-12f, umax - umin);
        const float dv = std::max(1e-12f, vmax - vmin);
        sc = 1.0f / std::max(du, dv);
        if (area_3d > 1e-30) {
          sc *= float(std::sqrt(area_3d));
        }
      }

      out.patch_to_orig = patch_to_orig;
      out.u.assign(n, 0.0f);
      out.v.assign(n, 0.0f);
      out.valid.assign(n, 0);
      float w = 0.0f, h = 0.0f;
      for (std::size_t i = 0; i < n; i++) {
        if (!vert_ok(i)) {
          continue;
        }
        const float u = (pu[i] - umin) * sc;
        const float v = (pv[i] - vmin) * sc;
        out.u[i] = u;
        out.v[i] = v;
        out.valid[i] = 1;
        w = std::max(w, u);
        h = std::max(h, v);
      }
      out.width = std::max(w, 1e-12f);
      out.height = std::max(h, 1e-12f);
      out.ox = 0.0f;
      out.oy = 0.0f;
      return true;
    };

    std::vector<IslandUV> islands;
    islands.reserve(size_t(n_islands));
    int ok_islands = 0;
    std::string last_err;

    for (std::size_t cc = 0; cc < n_cc; cc++) {
      const auto &flist = faces_by_cc[cc];
      if (flist.empty()) {
        continue;
      }

      /* Build a patch mesh from this component's faces. */
      Surface_mesh patch;
      std::map<int, vertex_descriptor> orig_to_patch;
      std::vector<int> patch_to_orig; /* patch vertex idx -> original vert */

      for (const face_descriptor fd : flist) {
        std::vector<vertex_descriptor> poly;
        halfedge_descriptor hd = halfedge(fd, sm);
        const halfedge_descriptor end = hd;
        do {
          const int ov = int(target(hd, sm).idx());
          auto it = orig_to_patch.find(ov);
          if (it == orig_to_patch.end()) {
            const Point_3 &p = sm.point(target(hd, sm));
            const vertex_descriptor nv = patch.add_vertex(p);
            orig_to_patch[ov] = nv;
            if (int(nv.idx()) >= int(patch_to_orig.size())) {
              patch_to_orig.resize(size_t(nv.idx()) + 1, -1);
            }
            patch_to_orig[size_t(nv.idx())] = ov;
            poly.push_back(nv);
          }
          else {
            poly.push_back(it->second);
          }
          hd = next(hd, sm);
        } while (hd != end);
        if (poly.size() >= 3) {
          patch.add_face(poly);
        }
      }
      if (patch.number_of_faces() == 0) {
        continue;
      }

      /* Apply seams that lie inside this patch (open closed components). */
      if (!seam_keys.empty()) {
        auto seam_edge_pm = patch.add_property_map<edge_descriptor, bool>("e:on_seam", false).first;
        auto seam_vertex_pm =
            patch.add_property_map<vertex_descriptor, bool>("v:on_seam", false).first;
        using Seam_mesh =
            CGAL::Seam_mesh<Surface_mesh, decltype(seam_edge_pm), decltype(seam_vertex_pm)>;
        Seam_mesh seam_mesh(patch, seam_edge_pm, seam_vertex_pm);
        int seams_added = 0;
        for (const edge_descriptor e : edges(patch)) {
          const int a = patch_to_orig[size_t(source(e, patch).idx())];
          const int b = patch_to_orig[size_t(target(e, patch).idx())];
          if (a < 0 || b < 0) {
            continue;
          }
          if (seam_keys.count({std::min(a, b), std::max(a, b)})) {
            if (seam_mesh.add_seam(source(e, patch), target(e, patch))) {
              seams_added++;
            }
          }
        }
        /* If seams open a border, parameterize via Seam_mesh UV map then collapse
         * to average per original vert (good enough for closed shells with cuts). */
        if (seams_added > 0) {
          using SM_vertex = typename boost::graph_traits<Seam_mesh>::vertex_descriptor;
          using SM_halfedge = typename boost::graph_traits<Seam_mesh>::halfedge_descriptor;
          struct SMVHash {
            std::size_t operator()(const SM_vertex &v) const
            {
              return hash_value(v);
            }
          };
          std::unordered_map<SM_vertex, Point_2, SMVHash> uv_umap;
          boost::associative_property_map<std::unordered_map<SM_vertex, Point_2, SMVHash>> uv_pmap(
              uv_umap);
          SM_halfedge bhd{};
          bool found = false;
          for (const SM_halfedge hd : halfedges(seam_mesh)) {
            if (is_border(hd, seam_mesh)) {
              bhd = hd;
              found = true;
              break;
            }
          }
          if (found) {
            SMP::Error_code err = SMP::ERROR_EMPTY_MESH;
            const int meth = (method < 0 || method > 6) ? 0 : method;
            if (meth == 1) {
              const double lam = (lambda > 0.0) ? lambda : 1000.0;
              const unsigned int iters = unsigned(energy_iterations > 0 ? energy_iterations : 50);
              using Parameterizer = SMP::ARAP_parameterizer_3<Seam_mesh>;
              err = SMP::parameterize(
                  seam_mesh,
                  Parameterizer(typename Parameterizer::Border_parameterizer(),
                                typename Parameterizer::Solver_traits(),
                                lam,
                                iters),
                  bhd,
                  uv_pmap);
            }
            else if (meth == 2) {
              using Border = SMP::Circular_border_arc_length_parameterizer_3<Seam_mesh>;
              using Parameterizer = SMP::Discrete_conformal_map_parameterizer_3<Seam_mesh, Border>;
              err = SMP::parameterize(seam_mesh, Parameterizer(), bhd, uv_pmap);
            }
            else if (meth == 3) {
              using Border = SMP::Circular_border_arc_length_parameterizer_3<Seam_mesh>;
              using Parameterizer =
                  SMP::Mean_value_coordinates_parameterizer_3<Seam_mesh, Border>;
              err = SMP::parameterize(seam_mesh, Parameterizer(), bhd, uv_pmap);
            }
            else if (meth == 4) {
              using Border = SMP::Circular_border_arc_length_parameterizer_3<Seam_mesh>;
              using Parameterizer = SMP::Discrete_authalic_parameterizer_3<Seam_mesh, Border>;
              err = SMP::parameterize(seam_mesh, Parameterizer(), bhd, uv_pmap);
            }
            else if (meth == 5) {
              using Border = SMP::Circular_border_arc_length_parameterizer_3<Seam_mesh>;
              using Parameterizer = SMP::Barycentric_mapping_parameterizer_3<Seam_mesh, Border>;
              err = SMP::parameterize(seam_mesh, Parameterizer(), bhd, uv_pmap);
            }
            else if (meth == 6) {
              using Parameterizer = SMP::Iterative_authalic_parameterizer_3<Seam_mesh>;
              Parameterizer param;
              const unsigned int iters = unsigned(energy_iterations > 0 ? energy_iterations : 15);
              err = param.parameterize(seam_mesh, bhd, uv_pmap, iters);
            }
            else {
              using Parameterizer = SMP::LSCM_parameterizer_3<Seam_mesh>;
              err = SMP::parameterize(seam_mesh, Parameterizer(), bhd, uv_pmap);
            }
            if (err == SMP::OK) {
              std::vector<float> sum_u(patch_to_orig.size(), 0.0f);
              std::vector<float> sum_v(patch_to_orig.size(), 0.0f);
              std::vector<int> cnt(patch_to_orig.size(), 0);
              for (const auto fd : faces(patch)) {
                halfedge_descriptor hd = halfedge(fd, patch);
                const halfedge_descriptor end = hd;
                do {
                  SM_halfedge shd(hd);
                  const SM_vertex svd = target(shd, seam_mesh);
                  auto it = uv_umap.find(svd);
                  if (it != uv_umap.end()) {
                    const int pi = int(target(hd, patch).idx());
                    if (pi >= 0 && pi < int(sum_u.size())) {
                      sum_u[size_t(pi)] += float(it->second.x());
                      sum_v[size_t(pi)] += float(it->second.y());
                      cnt[size_t(pi)]++;
                    }
                  }
                  hd = next(hd, patch);
                } while (hd != end);
              }
              std::vector<float> pu(patch_to_orig.size(), 0.0f);
              std::vector<float> pv(patch_to_orig.size(), 0.0f);
              for (std::size_t i = 0; i < patch_to_orig.size(); i++) {
                if (cnt[i] > 0) {
                  pu[i] = sum_u[i] / float(cnt[i]);
                  pv[i] = sum_v[i] / float(cnt[i]);
                }
              }
              IslandUV island;
              if (finalize_island_uv(patch, patch_to_orig, pu, pv, &cnt, island)) {
                islands.push_back(std::move(island));
                ok_islands++;
              }
              else {
                last_err = "Seam UV area finalize failed";
              }
              continue;
            }
            last_err = "Seam UV failed (error " + std::to_string(int(err)) + ")";
          }
        }
      }

      /* Open-border path (no seams or seam failed): parameterize patch as-is. */
      std::vector<float> pu, pv;
      if (!parameterize_one(patch, pu, pv)) {
        last_err = "Component has no boundary (mark Seams on closed shells)";
        continue;
      }

      IslandUV island;
      if (finalize_island_uv(patch, patch_to_orig, pu, pv, nullptr, island)) {
        islands.push_back(std::move(island));
        ok_islands++;
      }
      else {
        last_err = "Open UV area finalize failed";
      }
    }

    if (ok_islands == 0 || islands.empty()) {
      error = last_err.empty() ? "UV parameterization failed for all components" : last_err;
      return false;
    }

    /* Shelf-pack islands by their area-scaled bounding boxes.
     * Relative sizes already encode 3D surface area; do NOT force equal cells. */
    {
      /* Sort largest-first for a tighter shelf pack (stable for single island). */
      std::vector<std::size_t> order(islands.size());
      for (std::size_t i = 0; i < order.size(); i++) {
        order[i] = i;
      }
      std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return islands[a].height * islands[a].width > islands[b].height * islands[b].width;
      });

      /* Target shelf width: sqrt of total UV area (approx square atlas). */
      double total_area = 0.0;
      float max_w = 0.0f;
      for (const IslandUV &is : islands) {
        total_area += double(is.width) * double(is.height);
        max_w = std::max(max_w, is.width);
      }
      const float shelf_limit = std::max(max_w, float(std::sqrt(std::max(total_area, 1e-30))));
      const float gap = 0.02f * shelf_limit / float(std::max<std::size_t>(1, islands.size()));

      float cursor_x = 0.0f;
      float cursor_y = 0.0f;
      float row_h = 0.0f;
      float atlas_w = 0.0f;
      float atlas_h = 0.0f;
      for (const std::size_t idx : order) {
        IslandUV &is = islands[idx];
        if (cursor_x > 0.0f && cursor_x + is.width + gap > shelf_limit) {
          cursor_x = 0.0f;
          cursor_y += row_h + gap;
          row_h = 0.0f;
        }
        is.ox = cursor_x;
        is.oy = cursor_y;
        cursor_x += is.width + gap;
        row_h = std::max(row_h, is.height);
        atlas_w = std::max(atlas_w, is.ox + is.width);
        atlas_h = std::max(atlas_h, is.oy + is.height);
      }

      /* Optional: fit whole atlas into [0,1]^2 (uniform — preserves area ratios). */
      float atlas_sc = 1.0f;
      if (normalize) {
        const float extent = std::max(atlas_w, atlas_h);
        if (extent > 1e-12f) {
          atlas_sc = 1.0f / extent;
        }
      }

      std::vector<float> vert_u(size_t(mesh.verts_num), 0.0f);
      std::vector<float> vert_v(size_t(mesh.verts_num), 0.0f);
      std::vector<uint8_t> vert_has(size_t(mesh.verts_num), 0);

      for (const IslandUV &is : islands) {
        for (std::size_t i = 0; i < is.patch_to_orig.size(); i++) {
          const int ov = is.patch_to_orig[i];
          if (ov < 0 || ov >= mesh.verts_num) {
            continue;
          }
          if (i >= is.u.size() || i >= is.valid.size() || !is.valid[i]) {
            continue;
          }
          const float u = (is.u[i] + is.ox) * atlas_sc;
          const float v = (is.v[i] + is.oy) * atlas_sc;
          vert_u[size_t(ov)] = u;
          vert_v[size_t(ov)] = v;
          vert_has[size_t(ov)] = 1;
        }
      }

      /* Expand vertex UV to selected face corners. */
      for (int f = 0; f < mesh.faces_num; f++) {
        if (!face_ok[size_t(f)]) {
          continue;
        }
        int c0, c1;
        if (mesh.face_offsets) {
          c0 = mesh.face_offsets[f];
          c1 = mesh.face_offsets[f + 1];
        }
        else {
          c0 = f * 3;
          c1 = c0 + 3;
        }
        for (int c = c0; c < c1; c++) {
          const int vert = mesh.corner_verts[c];
          if (vert < 0 || vert >= mesh.verts_num || !vert_has[size_t(vert)]) {
            continue;
          }
          out_corner_uv[size_t(c) * 2 + 0] = vert_u[size_t(vert)];
          out_corner_uv[size_t(c) * 2 + 1] = vert_v[size_t(vert)];
        }
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "UV parameterization failed";
    return false;
  }
}

bool points_efficient_ransac(const float *in_xyz,
                             const float *in_nxyz,
                             int n,
                             double epsilon,
                             double cluster_epsilon,
                             double normal_threshold,
                             int min_points,
                             double probability,
                             int shape_flags,
                             unsigned int random_seed,
                             std::vector<int> &out_shape_id,
                             std::vector<int> &out_shape_type,
                             int &out_shape_count,
                             std::string &error)
{
  out_shape_id.assign(size_t(std::max(0, n)), -1);
  out_shape_type.assign(size_t(std::max(0, n)), -1);
  out_shape_count = 0;
  if (n < 10 || !in_xyz) {
    error = "Efficient RANSAC needs at least 10 points";
    return false;
  }
  if (shape_flags == 0) {
    shape_flags = 1; /* plane */
  }

  try {
    /* RANSAC samples minimal sets via CGAL::get_default_random(). Reset each
     * call so Geometry Nodes re-evaluation is deterministic for a fixed seed. */
    CGAL::get_default_random() = CGAL::Random(random_seed);

    std::vector<float> estimated;
    const float *normals = in_nxyz;
    if (!normals) {
      using Pair = std::pair<Point_3, Vector_3>;
      std::vector<Pair> tmp;
      tmp.reserve(size_t(n));
      for (int i = 0; i < n; i++) {
        tmp.emplace_back(Point_3(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                         Vector_3(0, 0, 1));
      }
      const int k = std::min(24, std::max(3, n - 1));
      CGAL::jet_estimate_normals<CGAL::Sequential_tag>(
          tmp,
          k,
          CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pair>()).normal_map(
              CGAL::Second_of_pair_property_map<Pair>()));
      CGAL::mst_orient_normals(
          tmp,
          k,
          CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pair>()).normal_map(
              CGAL::Second_of_pair_property_map<Pair>()));
      estimated.resize(size_t(n) * 3);
      for (int i = 0; i < n; i++) {
        estimated[size_t(i) * 3 + 0] = float(tmp[size_t(i)].second.x());
        estimated[size_t(i) * 3 + 1] = float(tmp[size_t(i)].second.y());
        estimated[size_t(i) * 3 + 2] = float(tmp[size_t(i)].second.z());
      }
      normals = estimated.data();
    }

    std::vector<IndexedPwn> points;
    points.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      points.emplace_back(Point_3(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                          Vector_3(normals[i * 3], normals[i * 3 + 1], normals[i * 3 + 2]),
                          i);
    }

    using Point_map = CGAL::Nth_of_tuple_property_map<0, IndexedPwn>;
    using Normal_map = CGAL::Nth_of_tuple_property_map<1, IndexedPwn>;
    using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel,
                                                                   std::vector<IndexedPwn>,
                                                                   Point_map,
                                                                   Normal_map>;
    using Efficient_ransac = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
    using Plane = CGAL::Shape_detection::Plane<Traits>;
    using Sphere = CGAL::Shape_detection::Sphere<Traits>;
    using Cylinder = CGAL::Shape_detection::Cylinder<Traits>;
    using Cone = CGAL::Shape_detection::Cone<Traits>;
    using Torus = CGAL::Shape_detection::Torus<Traits>;

    Efficient_ransac ransac;
    ransac.set_input(points, Point_map(), Normal_map());
    if (shape_flags & 1) {
      ransac.template add_shape_factory<Plane>();
    }
    if (shape_flags & 2) {
      ransac.template add_shape_factory<Sphere>();
    }
    if (shape_flags & 4) {
      ransac.template add_shape_factory<Cylinder>();
    }
    if (shape_flags & 8) {
      ransac.template add_shape_factory<Cone>();
    }
    if (shape_flags & 16) {
      ransac.template add_shape_factory<Torus>();
    }

    Efficient_ransac::Parameters params;
    if (probability > 0.0 && probability < 1.0) {
      params.probability = probability;
    }
    if (min_points > 0) {
      params.min_points = std::size_t(min_points);
    }
    if (epsilon > 0.0) {
      params.epsilon = epsilon;
    }
    if (cluster_epsilon > 0.0) {
      params.cluster_epsilon = cluster_epsilon;
    }
    if (normal_threshold > 0.0 && normal_threshold <= 1.0) {
      params.normal_threshold = normal_threshold;
    }

    ransac.detect(params);

    int sid = 0;
    for (const auto &shape_ptr : ransac.shapes()) {
      int stype = -1;
      if (std::dynamic_pointer_cast<Plane>(shape_ptr)) {
        stype = 0;
      }
      else if (std::dynamic_pointer_cast<Sphere>(shape_ptr)) {
        stype = 1;
      }
      else if (std::dynamic_pointer_cast<Cylinder>(shape_ptr)) {
        stype = 2;
      }
      else if (std::dynamic_pointer_cast<Cone>(shape_ptr)) {
        stype = 3;
      }
      else if (std::dynamic_pointer_cast<Torus>(shape_ptr)) {
        stype = 4;
      }
      for (const std::size_t idx : shape_ptr->indices_of_assigned_points()) {
        if (idx >= points.size()) {
          continue;
        }
        const int orig = std::get<2>(points[idx]);
        if (orig >= 0 && orig < n) {
          out_shape_id[size_t(orig)] = sid;
          out_shape_type[size_t(orig)] = stype;
        }
      }
      sid++;
    }
    out_shape_count = sid;
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Efficient RANSAC failed";
    return false;
  }
}

bool points_region_growing_planes(const float *in_xyz,
                                  const float *in_nxyz,
                                  int n,
                                  double neighbor_radius,
                                  double max_distance,
                                  double max_angle_deg,
                                  int min_region_size,
                                  std::vector<int> &out_region,
                                  int &out_region_count,
                                  std::string &error)
{
  out_region.assign(size_t(std::max(0, n)), -1);
  out_region_count = 0;
  if (n < 3 || !in_xyz) {
    error = "Point region growing needs at least 3 points";
    return false;
  }
  if (neighbor_radius <= 0.0) {
    error = "Neighbor radius must be > 0";
    return false;
  }

  try {
    std::vector<float> estimated;
    const float *nxyz = in_nxyz;
    if (!nxyz) {
      using Pair = std::pair<Point_3, Vector_3>;
      std::vector<Pair> tmp;
      tmp.reserve(size_t(n));
      for (int i = 0; i < n; i++) {
        tmp.emplace_back(Point_3(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                         Vector_3(0, 0, 1));
      }
      const int k = std::min(24, std::max(3, n - 1));
      CGAL::jet_estimate_normals<CGAL::Sequential_tag>(
          tmp,
          k,
          CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pair>()).normal_map(
              CGAL::Second_of_pair_property_map<Pair>()));
      CGAL::mst_orient_normals(
          tmp,
          k,
          CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pair>()).normal_map(
              CGAL::Second_of_pair_property_map<Pair>()));
      estimated.resize(size_t(n) * 3);
      for (int i = 0; i < n; i++) {
        estimated[size_t(i) * 3 + 0] = float(tmp[size_t(i)].second.x());
        estimated[size_t(i) * 3 + 1] = float(tmp[size_t(i)].second.y());
        estimated[size_t(i) * 3 + 2] = float(tmp[size_t(i)].second.z());
      }
      nxyz = estimated.data();
    }

    std::vector<PwnRG> pwn;
    pwn.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pwn.emplace_back(Point_3(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                       Vector_3(nxyz[i * 3], nxyz[i * 3 + 1], nxyz[i * 3 + 2]));
    }

    using Neighbor_query = CGAL::Shape_detection::Point_set::Sphere_neighbor_query<Kernel,
                                                                                   PwnRGIt,
                                                                                   PwnRGPointMap>;
    using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<
        Kernel,
        PwnRGIt,
        PwnRGPointMap,
        PwnRGNormalMap>;
    using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

    Neighbor_query neighbor_query(
        pwn,
        CGAL::parameters::sphere_radius(neighbor_radius).point_map(PwnRGPointMap()));
    Region_type region_type(CGAL::parameters::maximum_distance(max_distance)
                                .maximum_angle(max_angle_deg)
                                .minimum_region_size(std::max(1, min_region_size))
                                .point_map(PwnRGPointMap())
                                .normal_map(PwnRGNormalMap()));
    Region_growing region_growing(pwn, neighbor_query, region_type);

    std::vector<typename Region_growing::Primitive_and_region> regions;
    region_growing.detect(std::back_inserter(regions));

    int rid = 0;
    for (const auto &pr : regions) {
      for (const PwnRGIt it : pr.second) {
        const int idx = int(std::distance(pwn.cbegin(), it));
        if (idx >= 0 && idx < n) {
          out_region[size_t(idx)] = rid;
        }
      }
      rid++;
    }
    out_region_count = rid;
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Point region growing failed";
    return false;
  }
}

}  // namespace blender::cgal_bridge
