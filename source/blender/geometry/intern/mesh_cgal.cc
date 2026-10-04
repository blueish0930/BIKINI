/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_cgal.hh"

#include "BLI_array.hh"
#include "BLI_bounds.hh"
#include "BLI_index_mask.hh"
#include "BLI_index_range.hh"
#include "BLI_kdopbvh.hh"
#include "BLI_map.hh"
#include "BLI_math_base.hh"
#include "BLI_math_geom.hh"
#include "BLI_math_vector.hh"
#include "BLI_set.hh"
#include "BLI_offset_indices.hh"
#include "BLI_span.hh"
#include "BLI_string_ref.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_filter.hh"
#include "BKE_attribute_filters.hh"
#include "BKE_attribute_math.hh"
#include "BKE_bvh.hh"
#include "BKE_bvhutils.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_fair.hh"
#include "BKE_mesh_mapping.hh"
#include "BKE_mesh_remesh_voxel.hh"
#include "BKE_pointcloud.hh"

#include "eigen_capi.h"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#ifdef WITH_CGAL
#  include "cgal_bridge.hh"
#endif

namespace blender::geometry {

Mesh *cgal_triangle_mesh_from_buffers(Span<float3> positions, Span<int> corner_verts)
{
  if (positions.is_empty() || corner_verts.size() < 3 || corner_verts.size() % 3 != 0) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  const int faces_num = int(corner_verts.size() / 3);
  const int verts_num = int(positions.size());

  for (const int v : corner_verts) {
    if (v < 0 || v >= verts_num) {
      return BKE_mesh_new_nomain(0, 0, 0, 0);
    }
  }

  Mesh *mesh = BKE_mesh_new_nomain(verts_num, 0, faces_num, faces_num * 3);
  mesh->vert_positions_for_write().copy_from(positions);

  MutableSpan<int> face_offsets = mesh->face_offsets_for_write();
  MutableSpan<int> out_corners = mesh->corner_verts_for_write();
  for (int f = 0; f < faces_num; f++) {
    face_offsets[f] = f * 3;
    out_corners[f * 3 + 0] = corner_verts[f * 3 + 0];
    out_corners[f * 3 + 1] = corner_verts[f * 3 + 1];
    out_corners[f * 3 + 2] = corner_verts[f * 3 + 2];
  }
  face_offsets[faces_num] = faces_num * 3;

  bke::mesh_calc_edges(*mesh, false, false);
  mesh->tag_overlapping_none();
  return mesh;
}

#ifndef WITH_CGAL

Mesh *cgal_convex_hull_3(Span<float3> /*points*/, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_alpha_shape_3(
    Span<float3>, float, bool, int, float &r_alpha_used, std::string &r_error)
{
  r_alpha_used = 0.0f;
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_simplify(const Mesh &, float, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_isotropic_remesh(const Mesh &, float, int, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_smooth_shape(const Mesh &, float, int, bool, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_boolean(const Mesh &, const Mesh &, CgalBooleanOperation, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_hole_fill(const Mesh &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_fair(const Mesh &, int, std::string &r_error, Span<uint8_t>)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_refine(const Mesh &, float, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_clip_plane(const Mesh &, const float3 &, const float3 &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_subdivision(const Mesh &, CgalSubdivisionMode, int, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_repair(const Mesh &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_keep_largest(const Mesh &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_alpha_wrap(const Mesh &, float, float, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_corefine(const Mesh &,
                         const Mesh &,
                         std::string &r_error,
                         Array<bool> * /*r_seam_edge_selection*/)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}

Mesh *cgal_mesh_detect_features(const Mesh &, float, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_angle_area_smooth(const Mesh &, int, bool, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_tangential_relaxation(const Mesh &, int, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_extrude(const Mesh &, float, const float3 &, bool, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_remesh_planar_patches(const Mesh &, float, float, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_random_perturbation(const Mesh &, float, bool, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_triangulate(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_orient_outward(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_repair_self_intersections(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_split_long_edges(const Mesh &, float, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_connected_component_keep(const Mesh &, int, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_merge_border_vertices(const Mesh &, float, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
bool cgal_mesh_does_self_intersect(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
float cgal_mesh_volume(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return 0.0f; }
float cgal_mesh_area(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return 0.0f; }
Mesh *cgal_alpha_wrap_points(Span<float3>, float, float, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_advancing_front(Span<float3>, float, float, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_delaunay_3d(Span<float3>, const PointCloud *, const Mesh *, bool, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_min_sphere(Span<float3>, int, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_optimal_bbox(Span<float3>, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
float cgal_points_average_spacing(Span<float3>, int, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return 0.0f; }
PointCloud *cgal_points_jet_smooth(Span<float3>, int, int, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_bilateral_smooth(Span<float3>, int, int, float, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_remove_outliers(Span<float3>, int, float, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_grid_simplify(Span<float3>, float, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_random_simplify(Span<float3>, float, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_estimate_normals(
    Span<float3>, int, const PointCloud *, const Mesh *, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_wlop(
    Span<float3>, float, float, int, bool, const PointCloud *, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_hierarchy_simplify(
    Span<float3>, int, float, const PointCloud *, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_edge_aware_upsample(Span<float3>,
                                            Span<float3>,
                                            int,
                                            float,
                                            float,
                                            float,
                                            int,
                                            const PointCloud *,
                                            const Mesh *,
                                            std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_vcm_estimate_normals(
    Span<float3>, float, float, const PointCloud *, const Mesh *, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
Mesh *cgal_points_poisson(Span<float3>, Span<float3>, float, int, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
PointCloud *cgal_mesh_side_of(
    const Mesh &, Span<float3>, const PointCloud *, const Mesh *, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
bool cgal_mesh_side_of_query(const Mesh &, Span<float3>, MutableSpan<bool> r_inside, std::string &r_error)
{
  r_inside.fill(false);
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
PointCloud *cgal_mesh_distance_to(const Mesh &, Span<float3>, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_mesh_sample_points(const Mesh &, int, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
Mesh *cgal_mesh_skeleton(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_extract_border(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_geodesic_distance(const Mesh &, Span<float3>, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_segmentation(const Mesh &, int, float, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_min_ellipsoid(Span<float3>, int, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }

bool cgal_mesh_is_closed(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
bool cgal_mesh_centroid(const Mesh &, float3 &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
Mesh *cgal_mesh_autorefine(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_remove_degenerate(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
bool cgal_mesh_mean_curvature(const Mesh &, MutableSpan<float>, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
bool cgal_mesh_gaussian_curvature(const Mesh &, MutableSpan<float>, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
Mesh *cgal_points_scale_space(Span<float3>, int, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
bool cgal_mesh_principal_curvatures(const Mesh &,
                                    MutableSpan<float>,
                                    MutableSpan<float>,
                                    MutableSpan<float3>,
                                    MutableSpan<float3>,
                                    std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
bool cgal_mesh_shape_diameter(const Mesh &, MutableSpan<float>, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
bool cgal_mesh_mark_self_intersect(const Mesh &,
                                   MutableSpan<bool> r_intersect,
                                   MutableSpan<bool> r_inside,
                                   std::string &r_error)
{
  r_intersect.fill(false);
  r_inside.fill(false);
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
bool cgal_mesh_is_outward_oriented(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
Mesh *cgal_mesh_reverse_orientation(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
int cgal_mesh_hole_count(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return 0; }
bool cgal_mesh_border_edges(const Mesh &, MutableSpan<bool>, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
bool cgal_mesh_dihedral_angles(const Mesh &, MutableSpan<float>, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
Mesh *cgal_mesh_duplicate_non_manifold(const Mesh &, std::string &r_error)
{ r_error = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }

PointCloud *cgal_points_pca_estimate_normals(Span<float3>, int, const PointCloud *, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_mst_orient_normals(
    Span<float3>, Span<float3>, int, const PointCloud *, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_radial_orient_normals(
    Span<float3>, Span<float3>, const PointCloud *, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_points_cluster(Span<float3>, float, const PointCloud *, int &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
PointCloud *cgal_mesh_polyhedral_envelope(
    const Mesh &, float, Span<float3>, const PointCloud *, const Mesh *, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
Mesh *cgal_mesh_vertex_normals(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_face_normals(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_face_aspect_ratio(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_vertex_valence(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_mean_edge_length(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_border_vertex(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_face_quality(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_region_growing(const Mesh &, float, float, int, int &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_surface_shortest_path(const Mesh &, const float3 &, const float3 &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
PointCloud *cgal_mesh_locate(
    const Mesh &, Span<float3>, const PointCloud *, const Mesh *, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }

Mesh *cgal_mesh_edge_length(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_face_perimeter(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
bool cgal_points_centroid(Span<float3>, float3 &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
Mesh *cgal_points_fit_plane(Span<float3>, float3 &, float3 &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
bool cgal_points_neighbor_scale(Span<float3>, int &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
PointCloud *cgal_points_scanline_orient_normals(
    Span<float3>, Span<float3>, const PointCloud *, const Mesh *, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }

PointCloud *cgal_points_local_neighbor_scales(
    Span<float3>, const PointCloud *, const Mesh *, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
Mesh *cgal_mesh_remove_small_components(const Mesh &, int, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_points_fit_line(Span<float3>, float3 &, float3 &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
bool cgal_points_diameter(Span<float3>, float &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
Mesh *cgal_mesh_orient_polygon_soup(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
bool cgal_mesh_self_intersection_count(const Mesh &, int &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
PointCloud *cgal_points_local_density(
    Span<float3>, int, const PointCloud *, const Mesh *, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return nullptr; }
bool cgal_mesh_compactness(const Mesh &, float &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return false; }
Mesh *cgal_mesh_component_size(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
Mesh *cgal_mesh_face_planarity(const Mesh &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return BKE_mesh_new_nomain(0, 0, 0, 0); }
bool cgal_mesh_is_triangle_mesh(const Mesh &, bool &, std::string &e)
{ e = "Built without CGAL (WITH_CGAL=OFF)"; return false; }

Mesh *cgal_mesh_arap_deform(const Mesh &,
                            Span<uint8_t>,
                            Span<uint8_t>,
                            Span<float3>,
                            int,
                            int,
                            float,
                            std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_mesh_parameterize_uv_corners(const Mesh &,
                                       Span<bool>,
                                       Span<bool>,
                                       bool,
                                       int,
                                       int,
                                       float,
                                       MutableSpan<float2>,
                                       std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
bool cgal_points_region_growing_planes(Span<float3>,
                                       Span<float3>,
                                       float,
                                       float,
                                       float,
                                       int,
                                       MutableSpan<int>,
                                       int &,
                                       std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
PointCloud *cgal_points_efficient_ransac(Span<float3>,
                                         Span<float3>,
                                         float,
                                         float,
                                         float,
                                         int,
                                         float,
                                         int,
                                         int,
                                         const PointCloud *,
                                         const Mesh *,
                                         int &,
                                         std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return nullptr;
}

Mesh *cgal_points_min_ellipse_2(Span<float3>, int, int, float3 &, float &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_min_parallelogram_2(Span<float3>, int, float3 &, float &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_convex_hull_2(Span<float3>, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_mesh_plane_slice(const Mesh &, float3, float3, Vector<Vector<float3>> &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
PointCloud *cgal_points_vcm_feature_edges(Span<float3>,
                                          float,
                                          float,
                                          float,
                                          bool,
                                          const PointCloud *,
                                          const Mesh *,
                                          int &count,
                                          std::string &e)
{
  count = 0;
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return nullptr;
}
Mesh *cgal_points_delaunay_2(Span<float3>, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_alpha_shape_2(Span<float3>, int, float, bool, float &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_voronoi_2(Span<float3>, int, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_straight_skeleton_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_polygon_offset_2(const Mesh &, float, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_medial_axis_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_min_annulus_2(Span<float3>, int, int, float3 &, float &, float &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_convex_partition_2(const Mesh &, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_y_monotone_partition_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_max_area_k_gon_2(Span<float3>, int, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_polyline_simplify_2(const Mesh &, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_width_3(Span<float3>, float, float &, float3 &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_minkowski_sum_2(const Mesh &, const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_extrude_skeleton(const Mesh &, float, bool, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_polygon_fill_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_mesh_do_intersect_2(const Mesh &, const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_mesh_boolean_ops_2(const Mesh &, const Mesh *, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_largest_empty_iso_rectangle_2(
    Span<float3>, int, float3 &, float &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_polygon_repair_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
PointCloud *cgal_points_monge_jet_fit_pc(
    Span<float3>, int, int, int, const PointCloud *, const Mesh *, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return nullptr;
}

Mesh *cgal_mesh_convex_decomposition_3(const Mesh &, int &r_piece_count, std::string &e)
{
  r_piece_count = 0;
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_laplace_deform(const Mesh &,
                               Span<uint8_t>,
                               Span<uint8_t>,
                               Span<float3>,
                               float,
                               std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_surface_delaunay_remesh(
    const Mesh &, float, float, float, float, bool, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_otr_reconstruct_2(Span<float3>, int, float, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_regular_triangulation_2(Span<float3>, Span<float>, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_power_diagram_2(Span<float3>, Span<float>, int, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_voronoi_3(Span<float3>, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_visibility_2(const Mesh &, const float3 &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_refine_2(const Mesh &, float, float, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_alpha_complex_3(
    Span<float3>, float, bool, bool, float &alpha, int &tets, std::string &e)
{
  alpha = 0.0f;
  tets = 0;
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_minkowski_sum_3_convex(const Mesh &, const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_regular_triangulation_3(
    Span<float3>, Span<float>, bool, int &tets, std::string &e)
{
  tets = 0;
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_power_diagram_3(Span<float3>, Span<float>, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_largest_empty_circle_2(
    Span<float3>, int, int, float3 &, float &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_largest_inscribed_circle_2(const Mesh &, int, float3 &, float &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_min_width_2(Span<float3>, int, float, float &, float3 &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_periodic_delaunay_2(Span<float3>, int, float, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_constrained_voronoi_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_arrangement_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_delaunay_on_sphere(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_periodic_delaunay_3(
    Span<float3>, float, float, float, bool, int &tets, std::string &e)
{
  tets = 0;
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_mesh_do_intersect_3(const Mesh &, const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_mesh_split_by_mesh(const Mesh &, const Mesh &, int &pieces, std::string &e)
{
  pieces = 0;
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
CgalNaturalNeighbor2::~CgalNaturalNeighbor2() = default;
CgalNaturalNeighbor2::CgalNaturalNeighbor2(CgalNaturalNeighbor2 &&) noexcept = default;
CgalNaturalNeighbor2 &CgalNaturalNeighbor2::operator=(CgalNaturalNeighbor2 &&) noexcept = default;
bool CgalNaturalNeighbor2::build(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
bool CgalNaturalNeighbor2::is_valid() const
{
  return false;
}
int CgalNaturalNeighbor2::site_count() const
{
  return 0;
}
void CgalNaturalNeighbor2::query_many(Span<float3>,
                                      std::vector<int> &,
                                      std::vector<int> &,
                                      std::vector<float> &,
                                      MutableSpan<bool>) const
{
}
CgalNaturalNeighbor3::~CgalNaturalNeighbor3() = default;
CgalNaturalNeighbor3::CgalNaturalNeighbor3(CgalNaturalNeighbor3 &&) noexcept = default;
CgalNaturalNeighbor3 &CgalNaturalNeighbor3::operator=(CgalNaturalNeighbor3 &&) noexcept = default;
bool CgalNaturalNeighbor3::build(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
bool CgalNaturalNeighbor3::is_valid() const
{
  return false;
}
int CgalNaturalNeighbor3::site_count() const
{
  return 0;
}
void CgalNaturalNeighbor3::query_many(Span<float3>,
                                      std::vector<int> &,
                                      std::vector<int> &,
                                      std::vector<float> &,
                                      MutableSpan<bool>) const
{
}
Mesh *cgal_points_proximity_graph_2(Span<float3>, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_fill_polyline(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_halfspace_intersection_3(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_points_lloyd_relax(
    const Mesh &, Span<float3>, int, float, MutableSpan<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_mesh_conforming_delaunay_2(const Mesh &, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_max_perimeter_k_gon_2(Span<float3>, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_constrained_delaunay_2(const Mesh &, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_exterior_skeleton_2(const Mesh &, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_voronoi_on_sphere(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_radius_graph_3(Span<float3>, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_knn_graph_3(Span<float3>, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_points_natural_neighbor_3(Span<float3>,
                                    Span<float>,
                                    Span<float3>,
                                    MutableSpan<float>,
                                    std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
bool cgal_points_sibson_gradient_2(Span<float3>, Span<float>, MutableSpan<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_points_octree_3(Span<float3>, int, int, bool, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_hilbert_path_3(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_shortest_cycle(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_periodic_voronoi_2(Span<float3>, float, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_periodic_voronoi_3(Span<float3>, float, float, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_gabriel_graph_3(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_euclidean_mst_3(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_euclidean_mst_3(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_beta_skeleton_2(Span<float3>, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_convex_layers_2(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_farthest_voronoi_2(Span<float3>, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_convex_layers_3(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_overlay_2(const Mesh &, const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_polygon_kernel_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_mesh_self_intersection_curves(const Mesh &, Vector<Vector<float3>> &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_points_crust_2(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_geodesic_voronoi(const Mesh &, Span<bool>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_isosurface_3(Span<float3>, Span<float>, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_line_arrangement_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_vertical_decomposition_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_circle_arrangement_2(Span<float3>, Span<float>, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_crust_3(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_clipped_voronoi_3(Span<float3>, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_complement_2(const Mesh &, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_simple_polygon_2(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_points_regularize_planes(Span<float3>,
                                   Span<int>,
                                   bool,
                                   bool,
                                   bool,
                                   bool,
                                   float,
                                   float,
                                   MutableSpan<float3>,
                                   int &planes,
                                   std::string &e)
{
  planes = 0;
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}

Mesh *cgal_mesh_split_crossings_2(const Mesh &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
PointCloud *cgal_mesh_crossing_points_2(const Mesh &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_pointcloud_new_nomain(PointCloudType::Points, 0);
}
Mesh *cgal_mesh_pullout_directions_2(const Mesh &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_ssab_partition_2(const Mesh &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_snap_borders(const Mesh &, float, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_autorefine_clean(const Mesh &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_random_polygon_2(int, float, int, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_random_convex_set_2(int, float, int, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_largest_empty_sphere_3(Span<float3>, int, float3 &, float &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_ransac_primitives(Span<float3>,
                                    Span<float3>,
                                    float,
                                    float,
                                    float,
                                    int,
                                    float,
                                    int,
                                    int,
                                    int,
                                    int &,
                                    std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_voronoi_slice_3(Span<float3>, const float3 &, const float3 &, float, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_convex_offset_3(const Mesh &, float, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_volume_components(const Mesh &, int &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_inscribed_sphere_3(const Mesh &, int, float3 &, float &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_min_cylinder_3(Span<float3>, int, float3 &, float &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_simplify_polyline_xyz(Span<float3>,
                                bool,
                                float,
                                bool,
                                Vector<float3> &,
                                Vector<int> &,
                                std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_points_shape_fitting(Span<float3>,
                                Span<float3>,
                                int,
                                float,
                                float,
                                float,
                                int,
                                float,
                                float,
                                int,
                                MutableSpan<int>,
                                int &r_count,
                                std::string &r_error)
{
  r_count = 0;
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_fair_hole_fill(const Mesh &, int, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_simplify_polyline_3(const Mesh &, float, bool, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_sphere_region_growing(Span<float3>,
                                        Span<float3>,
                                        float,
                                        float,
                                        float,
                                        int,
                                        float,
                                        float,
                                        int,
                                        MutableSpan<int>,
                                        int &r_count,
                                        std::string &r_error)
{
  r_count = 0;
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_cylinder_region_growing(Span<float3>,
                                          Span<float3>,
                                          float,
                                          float,
                                          float,
                                          int,
                                          float,
                                          float,
                                          int,
                                          MutableSpan<int>,
                                          int &r_count,
                                          std::string &r_error)
{
  r_count = 0;
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_circle_region_growing(Span<float3>,
                                        Span<float3>,
                                        float,
                                        float,
                                        float,
                                        int,
                                        float,
                                        float,
                                        int,
                                        MutableSpan<int>,
                                        int &r_count,
                                        std::string &r_error)
{
  r_count = 0;
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_line_region_growing(Span<float3>,
                                      Span<float3>,
                                      float,
                                      float,
                                      int,
                                      MutableSpan<int>,
                                      int &r_count,
                                      std::string &r_error)
{
  r_count = 0;
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_min_annulus_3(Span<float3>,
                                int,
                                float3 &,
                                float &,
                                float &,
                                std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_collapse_short_edges(const Mesh &, float, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_mesh_cone_slice(const Mesh &,
                          float3,
                          float3,
                          float,
                          float,
                          int,
                          Vector<Vector<float3>> &,
                          std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_mesh_clip_box(const Mesh &, float3, float3, bool, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_clip_by_mesh(const Mesh &, const Mesh &, bool, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_regularize_open_polyline_xy(
    Span<float3>, bool, float, float, Vector<float3> &, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_mesh_constrained_simplify(
    const Mesh &, float, bool, Span<uint8_t>, std::string &r_error)
{
  r_error = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_rectangular_p_center_2(Span<float3>, int, float &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_polyline_hull_2(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_mesh_curve_intersect(const Mesh &,
                               Span<float3>,
                               Span<int>,
                               Vector<Vector<float3>> &,
                               Vector<float3> &,
                               std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
bool cgal_mesh_geodesic_isolines(
    const Mesh &, Span<bool>, int, float, Vector<Vector<float3>> &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_mesh_overlap_faces(const Mesh &, const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_mesh_contour_stack(
    const Mesh &, float3, float3, int, float, Vector<Vector<float3>> &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_points_alpha_edges_3(Span<float3>, float, bool, int, float &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_delaunay_edges_3(Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_mesh_edge_path(const Mesh &, Span<bool>, Span<bool>, Vector<float3> &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
bool cgal_mesh_curvature_isolines(
    const Mesh &, int, int, Vector<Vector<float3>> &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_mesh_offset_sdf(const Mesh &, float, int, bool, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_cdt_hole_fill(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_bisector_surface(Span<float3>, Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_projected_outline(const Mesh &, float3, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_interior_tets(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_surface_delaunay_graph(Span<float3>, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_split_charts(const Mesh &, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_restricted_voronoi(const Mesh &, Span<float3>, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_points_walk_tets(Span<float3>, float3, float3, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_skeleton_spokes(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_mesh_radial_slices(
    const Mesh &, float3, float3, int, Vector<Vector<float3>> &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_mesh_intersection_band(const Mesh &, const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
bool cgal_points_sphere_intersect(
    Span<float3>, Span<float>, float, int, Vector<Vector<float3>> &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return false;
}
Mesh *cgal_mesh_sphere_arrangement(const Mesh &, float3, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_graphcut_segment(const Mesh &, float, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
PointCloud *cgal_points_classification_features(
    Span<float3>, int, const PointCloud *, const Mesh *, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return nullptr;
}
PointCloud *cgal_points_register_icp(
    Span<float3>, Span<float3>, int, const PointCloud *, const Mesh *, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return nullptr;
}
PointCloud *cgal_points_register_4pcs(
    Span<float3>, Span<float3>, int, int, const PointCloud *, const Mesh *, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return nullptr;
}
Mesh *cgal_points_alpha_wrap_2(Span<float3>, const Mesh *, float, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_cage_deform_3(const Mesh &, const Mesh &, const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_dual_contour_grid(Span<float>, int, int, int, float3, float3, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_tet_remesh(const Mesh &, float, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_constrained_delaunay_3(const Mesh &, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_approximate_convex_decomposition(const Mesh &, int, int, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_marching_cubes_grid(
    Span<float>, int, int, int, float3, float3, float, bool, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
PointCloud *cgal_points_extreme_point_3(Span<float3>, float3, std::string &e)
{
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return nullptr;
}
PointCloud *cgal_mesh_barycentric_3(
    const Mesh &, Span<float3>, int, PointCloud **r_weights, std::string &e)
{
  if (r_weights) {
    *r_weights = nullptr;
  }
  e = "Built without CGAL (WITH_CGAL=OFF)";
  return nullptr;
}


#else

static void pointcloud_copy_point_attributes(const PointCloud &src,
                                            PointCloud &dst,
                                            const Span<StringRef> skip_names);
static void pointcloud_transfer_attributes_nearest(const PointCloud &src,
                                                  PointCloud &dst,
                                                  const Span<StringRef> skip_names);
static void pointcloud_copy_from_mesh_point_attributes(const Mesh &src,
                                                      PointCloud &dst,
                                                      const Span<StringRef> skip_names);

/**
 * Build a Blender Mesh from CGAL buffers, preserving n-gons when face_offsets is set.
 * face_offsets empty => pure triangles (corner_verts.size() == faces*3).
 */
static Mesh *result_to_mesh(cgal_bridge::MeshResult &result, std::string &r_error)
{
  if (!result.ok) {
    r_error = result.error.empty() ? "CGAL operation failed" : result.error;
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  const int verts_num = result.verts_num();
  const int corners_num = result.corners_num();
  if (verts_num <= 0 || corners_num < 3) {
    r_error = result.error.empty() ? "CGAL produced empty mesh" : result.error;
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }

  for (const int v : result.corner_verts) {
    if (v < 0 || v >= verts_num) {
      r_error = "CGAL result has invalid corner indices";
      return BKE_mesh_new_nomain(0, 0, 0, 0);
    }
  }

  /* Pure triangle encoding (no face_offsets). */
  if (result.face_offsets.empty()) {
    if (corners_num % 3 != 0) {
      r_error = "CGAL triangle result has non-multiple-of-3 corners";
      return BKE_mesh_new_nomain(0, 0, 0, 0);
    }
    Array<float3> positions(verts_num);
    for (int i = 0; i < verts_num; i++) {
      positions[i] = float3(result.positions[size_t(i) * 3 + 0],
                            result.positions[size_t(i) * 3 + 1],
                            result.positions[size_t(i) * 3 + 2]);
    }
    return cgal_triangle_mesh_from_buffers(
        positions.as_span(), Span(result.corner_verts.data(), result.corner_verts.size()));
  }

  /* Polygon mesh (triangles and/or n-gons). */
  const int faces_num = result.faces_num();
  if (faces_num <= 0 || int(result.face_offsets.size()) != faces_num + 1) {
    r_error = "CGAL polygon result has invalid face_offsets";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  if (result.face_offsets.front() != 0 || result.face_offsets.back() != corners_num) {
    r_error = "CGAL face_offsets do not match corner count";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  for (int f = 0; f < faces_num; f++) {
    const int begin = result.face_offsets[f];
    const int end = result.face_offsets[f + 1];
    if (end - begin < 3 || begin < 0 || end > corners_num) {
      r_error = "CGAL face has fewer than 3 corners";
      return BKE_mesh_new_nomain(0, 0, 0, 0);
    }
  }

  Mesh *mesh = BKE_mesh_new_nomain(verts_num, 0, faces_num, corners_num);
  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  for (int i = 0; i < verts_num; i++) {
    positions[i] = float3(result.positions[size_t(i) * 3 + 0],
                          result.positions[size_t(i) * 3 + 1],
                          result.positions[size_t(i) * 3 + 2]);
  }
  mesh->face_offsets_for_write().copy_from(
      Span(result.face_offsets.data(), result.face_offsets.size()));
  mesh->corner_verts_for_write().copy_from(
      Span(result.corner_verts.data(), result.corner_verts.size()));

  bke::mesh_calc_edges(*mesh, false, false);
  mesh->tag_overlapping_none();
  return mesh;
}

/**
 * Topology-preserving path: copy full source mesh (all attributes) and only
 * replace positions. Used for fair / smooth when positions_only is set, or when
 * counts match exactly.
 */
static Mesh *result_to_mesh_or_positions(cgal_bridge::MeshResult &result,
                                         const Mesh &topology_src,
                                         std::string &r_error)
{
  if (!result.ok) {
    r_error = result.error.empty() ? "CGAL operation failed" : result.error;
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  const bool counts_match = result.verts_num() == topology_src.verts_num &&
                            (result.positions_only ||
                             (result.faces_num() == topology_src.faces_num &&
                              result.corners_num() == topology_src.corners_num));
  if (result.positions_only || counts_match) {
    if (result.verts_num() != topology_src.verts_num ||
        int(result.positions.size()) < topology_src.verts_num * 3)
    {
      r_error = "Position-only result vertex count mismatch";
      return BKE_mesh_new_nomain(0, 0, 0, 0);
    }
    Mesh *out = BKE_mesh_copy_for_eval(topology_src);
    MutableSpan<float3> positions = out->vert_positions_for_write();
    for (int i = 0; i < topology_src.verts_num; i++) {
      positions[i] = float3(result.positions[size_t(i) * 3 + 0],
                            result.positions[size_t(i) * 3 + 1],
                            result.positions[size_t(i) * 3 + 2]);
    }
    out->tag_positions_changed();
    return out;
  }
  return result_to_mesh(result, r_error);
}

/** Official Convex Hull style: every face flat (all sharp). */
static void shade_all_sharp(Mesh &mesh)
{
  bke::mesh_smooth_set(mesh, false);
}

static bool is_builtin_topology_attr(const StringRef name)
{
  return ELEM(name, "position", ".edge_verts", ".corner_vert", ".corner_edge");
}

/**
 * Interpolate attributes using topology maps from the CGAL op (face parent,
 * vertex copy / edge lerp). This is NOT nearest-surface sampling.
 */
static void interpolate_attributes_from_maps(Mesh &dst,
                                             const Mesh &src,
                                             const cgal_bridge::MeshResult &maps)
{
  if (dst.faces_num == 0) {
    return;
  }
  BKE_mesh_copy_parameters_for_eval(&dst, &src);

  const bke::AttributeAccessor src_attrs = src.attributes();
  bke::MutableAttributeAccessor dst_attrs = dst.attributes_for_write();

  if (maps.has_face_map()) {
    src_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
      if (iter.domain != bke::AttrDomain::Face || is_builtin_topology_attr(iter.name)) {
        return;
      }
      const GVArraySpan src_data = *iter.get();
      bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_only_span(
          iter.name, bke::AttrDomain::Face, iter.data_type);
      if (!dst_w) {
        return;
      }
      bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
        const Span<T> s = src_data.typed<T>();
        MutableSpan<T> d = dst_w.span.typed<T>();
        const T def = T();
        for (const int f : d.index_range()) {
          const int sf = maps.face_src[f];
          d[f] = (sf >= 0 && sf < s.size()) ? s[sf] : def;
        }
      });
      dst_w.finish();
    });
  }

  if (maps.has_vert_map()) {
    src_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
      if (iter.domain != bke::AttrDomain::Point || is_builtin_topology_attr(iter.name) ||
          iter.name == "position")
      {
        return;
      }
      const GVArraySpan src_data = *iter.get();
      bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_only_span(
          iter.name, bke::AttrDomain::Point, iter.data_type);
      if (!dst_w) {
        return;
      }
      bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
        const Span<T> s = src_data.typed<T>();
        MutableSpan<T> d = dst_w.span.typed<T>();
        const T def = T();
        for (const int v : d.index_range()) {
          const int a = maps.vert_src0[v];
          const int b = (v < int(maps.vert_src1.size())) ? maps.vert_src1[v] : -1;
          const float t = (v < int(maps.vert_factor.size())) ? maps.vert_factor[v] : 0.0f;
          if (a >= 0 && a < s.size() && b >= 0 && b < s.size()) {
            if constexpr (std::is_same_v<T, bool> || std::is_same_v<T, int> ||
                          std::is_same_v<T, int8_t> || std::is_same_v<T, int2> ||
                          std::is_same_v<T, short2>)
            {
              d[v] = (t < 0.5f) ? s[a] : s[b];
            }
            else {
              d[v] = bke::attribute_math::mix2(t, s[a], s[b]);
            }
          }
          else if (a >= 0 && a < s.size()) {
            d[v] = s[a];
          }
          else {
            d[v] = def;
          }
        }
      });
      dst_w.finish();
    });
  }

  if (maps.has_face_map()) {
    const OffsetIndices src_faces = src.faces();
    const Span<int> src_corner_verts = src.corner_verts();
    const OffsetIndices dst_faces = dst.faces();
    const Span<int> dst_corner_verts = dst.corner_verts();

    src_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
      if (iter.domain != bke::AttrDomain::Corner || is_builtin_topology_attr(iter.name)) {
        return;
      }
      const GVArraySpan src_data = *iter.get();
      bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_only_span(
          iter.name, bke::AttrDomain::Corner, iter.data_type);
      if (!dst_w) {
        return;
      }
      bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
        const Span<T> s = src_data.typed<T>();
        MutableSpan<T> d = dst_w.span.typed<T>();
        const T def = T();
        for (const int f : dst_faces.index_range()) {
          const int sf = maps.face_src[f];
          for (const int corner : dst_faces[f]) {
            const int dv = dst_corner_verts[corner];
            T value = def;
            bool found = false;
            if (sf >= 0 && sf < src_faces.size()) {
              int want = -1;
              if (maps.has_vert_map() && dv < int(maps.vert_src0.size())) {
                want = maps.vert_src0[dv];
                if (dv < int(maps.vert_src1.size()) && maps.vert_src1[dv] >= 0 &&
                    dv < int(maps.vert_factor.size()) && maps.vert_factor[dv] >= 0.5f)
                {
                  want = maps.vert_src1[dv];
                }
              }
              for (const int sc : src_faces[sf]) {
                if (want >= 0 && src_corner_verts[sc] == want) {
                  value = s[sc];
                  found = true;
                  break;
                }
              }
              if (!found) {
                value = s[src_faces[sf][0]];
                found = true;
              }
            }
            d[corner] = found ? value : def;
          }
        }
      });
      dst_w.finish();
    });
  }

  if (maps.has_vert_map() && src.edges_num > 0) {
    const Span<int2> src_edges = src.edges();
    const Span<int2> dst_edges = dst.edges();
    Map<int2, int> src_edge_lookup;
    src_edge_lookup.reserve(src_edges.size());
    for (const int e : src_edges.index_range()) {
      const int2 ev = src_edges[e];
      src_edge_lookup.add(int2(math::min(ev[0], ev[1]), math::max(ev[0], ev[1])), e);
    }

    Array<int> dst_to_src_edge(dst.edges_num, -1);
    for (const int e : dst_edges.index_range()) {
      const int2 de = dst_edges[e];
      const int a0 = maps.vert_src0[de[0]];
      const int a1 = maps.vert_src0[de[1]];
      const int b0 = (de[0] < int(maps.vert_src1.size())) ? maps.vert_src1[de[0]] : -1;
      const int b1 = (de[1] < int(maps.vert_src1.size())) ? maps.vert_src1[de[1]] : -1;
      if (a0 >= 0 && a1 >= 0 && b0 < 0 && b1 < 0) {
        if (const int *se = src_edge_lookup.lookup_ptr(
                int2(math::min(a0, a1), math::max(a0, a1))))
        {
          dst_to_src_edge[e] = *se;
        }
      }
    }

    src_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
      if (iter.domain != bke::AttrDomain::Edge || is_builtin_topology_attr(iter.name)) {
        return;
      }
      const GVArraySpan src_data = *iter.get();
      bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_only_span(
          iter.name, bke::AttrDomain::Edge, iter.data_type);
      if (!dst_w) {
        return;
      }
      bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
        const Span<T> s = src_data.typed<T>();
        MutableSpan<T> d = dst_w.span.typed<T>();
        const T def = T();
        for (const int e : d.index_range()) {
          const int se = dst_to_src_edge[e];
          d[e] = (se >= 0 && se < s.size()) ? s[se] : def;
        }
      });
      dst_w.finish();
    });
  }
}

/**
 * Boolean / corefine: interpolate from A or B using face_src_mesh + face_src.
 * Mark A|B interface edges sharp. No nearest-surface sampling.
 */
static void interpolate_attributes_boolean(Mesh &dst,
                                           const Mesh &src_a,
                                           const Mesh &src_b,
                                           const cgal_bridge::MeshResult &maps)
{
  if (dst.faces_num == 0) {
    return;
  }
  BKE_mesh_copy_parameters_for_eval(&dst, &src_a);
  if (!maps.has_face_map()) {
    return;
  }

  const bool has_mesh_id = int(maps.face_src_mesh.size()) == dst.faces_num;
  bke::MutableAttributeAccessor dst_attrs = dst.attributes_for_write();

  auto apply_face_from = [&](const Mesh &src_mesh, const Span<int> face_indices) {
    const bke::AttributeAccessor src_attrs = src_mesh.attributes();
    src_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
      if (iter.domain != bke::AttrDomain::Face || is_builtin_topology_attr(iter.name)) {
        return;
      }
      const GVArraySpan src_data = *iter.get();
      bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_span(
          iter.name, bke::AttrDomain::Face, iter.data_type);
      if (!dst_w) {
        return;
      }
      bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
        const Span<T> s = src_data.typed<T>();
        MutableSpan<T> d = dst_w.span.typed<T>();
        for (const int f : face_indices) {
          const int sf = maps.face_src[f];
          if (sf >= 0 && sf < s.size()) {
            d[f] = s[sf];
          }
        }
      });
      dst_w.finish();
    });
  };

  Vector<int> faces_a, faces_b;
  Array<int8_t> face_owner(dst.faces_num, 0);
  for (const int f : IndexRange(dst.faces_num)) {
    const int8_t mid = has_mesh_id ? maps.face_src_mesh[f] : int8_t(0);
    face_owner[f] = mid;
    if (mid == 0) {
      faces_a.append(f);
    }
    else {
      faces_b.append(f);
    }
  }
  apply_face_from(src_a, faces_a);
  apply_face_from(src_b, faces_b);

  if (maps.has_vert_map()) {
    auto apply_point_from = [&](const Mesh &src_mesh, const Span<int> verts) {
      if (verts.is_empty()) {
        return;
      }
      const bke::AttributeAccessor src_attrs = src_mesh.attributes();
      src_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
        if (iter.domain != bke::AttrDomain::Point || is_builtin_topology_attr(iter.name) ||
            iter.name == "position")
        {
          return;
        }
        const GVArraySpan src_data = *iter.get();
        bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_span(
            iter.name, bke::AttrDomain::Point, iter.data_type);
        if (!dst_w) {
          return;
        }
        bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
          const Span<T> s = src_data.typed<T>();
          MutableSpan<T> d = dst_w.span.typed<T>();
          for (const int v : verts) {
            const int a = maps.vert_src0[v];
            const int b = (v < int(maps.vert_src1.size())) ? maps.vert_src1[v] : -1;
            const float t = (v < int(maps.vert_factor.size())) ? maps.vert_factor[v] : 0.0f;
            if (a >= 0 && a < s.size() && b >= 0 && b < s.size()) {
              if constexpr (std::is_same_v<T, bool> || std::is_same_v<T, int> ||
                            std::is_same_v<T, int8_t> || std::is_same_v<T, int2> ||
                            std::is_same_v<T, short2>)
              {
                d[v] = (t < 0.5f) ? s[a] : s[b];
              }
              else {
                d[v] = bke::attribute_math::mix2(t, s[a], s[b]);
              }
            }
            else if (a >= 0 && a < s.size()) {
              d[v] = s[a];
            }
          }
        });
        dst_w.finish();
      });
    };

    Array<int> count_a(dst.verts_num, 0);
    Array<int> count_b(dst.verts_num, 0);
    const OffsetIndices dst_faces = dst.faces();
    const Span<int> dst_corner_verts = dst.corner_verts();
    for (const int f : dst_faces.index_range()) {
      for (const int c : dst_faces[f]) {
        const int v = dst_corner_verts[c];
        if (face_owner[f] == 0) {
          count_a[v]++;
        }
        else {
          count_b[v]++;
        }
      }
    }
    Vector<int> verts_a, verts_b;
    for (const int v : IndexRange(dst.verts_num)) {
      if (count_b[v] > count_a[v]) {
        verts_b.append(v);
      }
      else {
        verts_a.append(v);
      }
    }
    apply_point_from(src_a, verts_a);
    apply_point_from(src_b, verts_b);
  }

  {
    const OffsetIndices dst_faces = dst.faces();
    const Span<int> dst_corner_edges = dst.corner_edges();
    Array<int2> edge_faces(dst.edges_num, int2(-1, -1));
    for (const int f : dst_faces.index_range()) {
      for (const int corner : dst_faces[f]) {
        const int e = dst_corner_edges[corner];
        if (edge_faces[e][0] < 0) {
          edge_faces[e][0] = f;
        }
        else if (edge_faces[e][1] < 0 && edge_faces[e][0] != f) {
          edge_faces[e][1] = f;
        }
      }
    }
    bke::SpanAttributeWriter<bool> sharp_w = dst_attrs.lookup_or_add_for_write_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    bool any_seam = false;
    for (const int e : IndexRange(dst.edges_num)) {
      const int f0 = edge_faces[e][0];
      const int f1 = edge_faces[e][1];
      if (f0 >= 0 && f1 >= 0 && face_owner[f0] != face_owner[f1]) {
        sharp_w.span[e] = true;
        any_seam = true;
      }
    }
    sharp_w.finish();
    if (!any_seam) {
      bool keep = false;
      const VArraySpan sharp = *dst_attrs.lookup_or_default<bool>(
          "sharp_edge", bke::AttrDomain::Edge, false);
      for (const bool s : sharp) {
        if (s) {
          keep = true;
          break;
        }
      }
      if (!keep) {
        dst_attrs.remove("sharp_edge");
      }
    }
  }
}

/**
 * Pass original Blender mesh topology (n-gons included) into CGAL.
 * Algorithms that require triangles triangulate inside the bridge only.
 */
static cgal_bridge::MeshIn mesh_to_cgal_in(const Mesh &mesh)
{
  cgal_bridge::MeshIn in;
  if (mesh.verts_num < 1) {
    return in;
  }
  const Span<float3> positions = mesh.vert_positions();
  in.positions = positions.cast<float>().data();
  in.verts_num = int(positions.size());
  /* Faces (optional). */
  if (mesh.faces_num > 0 && mesh.corners_num >= 3) {
    in.corner_verts = mesh.corner_verts().data();
    in.corners_num = mesh.corners_num;
    in.face_offsets = mesh.face_offsets().data();
    in.faces_num = mesh.faces_num;
  }
  /* Loose / topology edges — needed for pure wire polygon loops. */
  if (mesh.edges_num > 0) {
    std::vector<int> e0(size_t(mesh.edges_num));
    std::vector<int> e1(size_t(mesh.edges_num));
    const Span<int2> edges = mesh.edges();
    for (int i = 0; i < mesh.edges_num; i++) {
      e0[size_t(i)] = edges[i][0];
      e1[size_t(i)] = edges[i][1];
    }
    in.set_owned_edges(std::move(e0), std::move(e1));
  }
  return in;
}

Mesh *cgal_convex_hull_3(Span<float3> points, std::string &r_error)
{
  cgal_bridge::TriangleMeshResult result = cgal_bridge::convex_hull_3(
      points.cast<float>().data(), int(points.size()));
  Mesh *mesh = result_to_mesh(result, r_error);
  if (mesh && mesh->faces_num > 0) {
    /* Same as official GeometryNodeConvexHull: fully sharp / flat. */
    shade_all_sharp(*mesh);
  }
  return mesh;
}

Mesh *cgal_alpha_shape_3(Span<float3> points,
                         float alpha,
                         bool use_optimal_alpha,
                         int solid_components,
                         float &r_alpha_used,
                         std::string &r_error)
{
  cgal_bridge::TriangleMeshResult result = cgal_bridge::alpha_shape_3(points.cast<float>().data(),
                                                                     int(points.size()),
                                                                     double(alpha),
                                                                     use_optimal_alpha,
                                                                     solid_components);
  r_alpha_used = float(result.alpha_used);
  Mesh *mesh = result_to_mesh(result, r_error);
  if (mesh && mesh->faces_num > 0) {
    bke::mesh_smooth_set(*mesh, true);
  }
  return mesh;
}

Mesh *cgal_mesh_simplify(const Mesh &mesh, float keep_ratio, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  /* Simplify rebuilds connectivity (edge collapse); reproject is acceptable. */
  cgal_bridge::MeshResult result = cgal_bridge::mesh_simplify(in, double(keep_ratio));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    BKE_mesh_copy_parameters_for_eval(out, &mesh);
    bke::mesh_remesh_reproject_attributes(mesh, *out);
  }
  return out;
}

Mesh *cgal_mesh_isotropic_remesh(const Mesh &mesh,
                                 float edge_length,
                                 int iterations,
                                 std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_isotropic_remesh(
      in, double(edge_length), iterations);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    /* Remesh has no parent topology — surface reproject only here. */
    BKE_mesh_copy_parameters_for_eval(out, &mesh);
    bke::mesh_remesh_reproject_attributes(mesh, *out);
  }
  return out;
}

Mesh *cgal_mesh_smooth_shape(const Mesh &mesh,
                             float time,
                             int iterations,
                             bool do_scale,
                             std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  /* MCF triangulates only for the solver; vertex positions map back to original n-gons. */
  cgal_bridge::MeshResult result = cgal_bridge::mesh_smooth_shape(
      in, double(time), iterations, do_scale);
  return result_to_mesh_or_positions(result, mesh, r_error);
}

Mesh *cgal_mesh_boolean(const Mesh &a,
                        const Mesh &b,
                        CgalBooleanOperation op,
                        std::string &r_error)
{
  cgal_bridge::MeshIn in_a = mesh_to_cgal_in(a);
  cgal_bridge::MeshIn in_b = mesh_to_cgal_in(b);
  if (in_a.faces_num <= 0 || in_b.faces_num <= 0) {
    r_error = "Both inputs need a non-empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  /* Corefinement needs triangles for the compute step; export keeps any polygons. */
  cgal_bridge::MeshResult result = cgal_bridge::mesh_boolean(in_a, in_b, int(op));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    interpolate_attributes_boolean(*out, a, b, result);
  }
  return out;
}

Mesh *cgal_mesh_hole_fill(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_hole_fill(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  return out;
}

static Mesh *run_mesh_op_with_attr_maps(
    const Mesh &mesh,
    const std::function<cgal_bridge::MeshResult(const cgal_bridge::MeshIn &)> &op,
    std::string &r_error,
    bool try_topology_preserve = false,
    bool allow_reproject_fallback = false)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = op(in);
  Mesh *out = (try_topology_preserve || result.positions_only) ?
                  result_to_mesh_or_positions(result, mesh, r_error) :
                  result_to_mesh(result, r_error);
  if (!out || out->faces_num == 0) {
    return out;
  }
  /* positions_only / full topology copy already keeps every attribute. */
  if (result.positions_only ||
      (try_topology_preserve && out->verts_num == mesh.verts_num &&
       out->faces_num == mesh.faces_num && out->corners_num == mesh.corners_num))
  {
    return out;
  }
  if (result.has_face_map() || result.has_vert_map()) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (allow_reproject_fallback) {
    /* Only remesh-like ops (refine densify) may reproject. */
    BKE_mesh_copy_parameters_for_eval(out, &mesh);
    bke::mesh_remesh_reproject_attributes(mesh, *out);
  }
  return out;
}

static void fair_pin_constraint_rings(const Mesh &mesh,
                                      MutableSpan<bool> affected,
                                      const int extra_rings)
{
  if (extra_rings <= 0) {
    return;
  }
  Array<int> v2e_off;
  Array<int> v2e_idx;
  const GroupedSpan<int> v2e = bke::mesh::build_vert_to_edge_map(
      mesh.edges(), mesh.verts_num, v2e_off, v2e_idx);
  const Span<int2> edges = mesh.edges();

  Vector<int> frontier;
  for (int v = 0; v < mesh.verts_num; v++) {
    if (!affected[v]) {
      frontier.append(v);
    }
  }
  for (int ring = 0; ring < extra_rings; ring++) {
    Vector<int> next;
    for (const int v : frontier) {
      for (const int e : v2e[v]) {
        const int nb = bke::mesh::edge_other_vert(edges[e], v);
        if (affected[nb]) {
          affected[nb] = false;
          next.append(nb);
        }
      }
    }
    frontier = std::move(next);
    if (frontier.is_empty()) {
      break;
    }
  }
}

Mesh *cgal_mesh_fair(const Mesh &mesh,
                     int continuity,
                     std::string &r_error,
                     Span<uint8_t> free_mask)
{
  continuity = std::clamp(continuity, 0, 8);
  if (mesh.verts_num < 4 || mesh.faces_num < 1) {
    r_error = "Fair needs a mesh with interior vertices";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  if (!free_mask.is_empty() && int(free_mask.size()) != mesh.verts_num) {
    r_error = "Fair selection size mismatch";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }

  Mesh *out = BKE_mesh_copy_for_eval(mesh);

  Array<int> e2f_off;
  Array<int> e2f_idx;
  const GroupedSpan<int> e2f = bke::mesh::build_edge_to_face_map(
      out->faces(), out->corner_edges(), out->edges_num, e2f_off, e2f_idx);
  const Span<int2> edges = out->edges();

  Array<bool> is_boundary(out->verts_num, false);
  for (const int e : edges.index_range()) {
    if (e2f[e].size() < 2) {
      is_boundary[edges[e][0]] = true;
      is_boundary[edges[e][1]] = true;
    }
  }

  Array<bool> affected(out->verts_num, false);
  int n_free = 0;
  int n_pinned = 0;
  for (int i = 0; i < out->verts_num; i++) {
    const bool want_free = free_mask.is_empty() || free_mask[i] != 0;
    if (want_free && !is_boundary[i]) {
      affected[i] = true;
      n_free++;
    }
    else {
      n_pinned++;
    }
  }
  if (n_free == 0) {
    r_error = "Fair needs interior vertices to move (all verts are pinned or on the boundary)";
    BKE_id_free(nullptr, out);
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  if (n_pinned == 0) {
    r_error = "Fair needs some fixed vertices; deselect the ports or use an open mesh";
    BKE_id_free(nullptr, out);
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }

  /*
   * Continuity c solves L^{c+1} x = 0. Pin the original c-ring around already
   * fixed verts (border + unselected) so C^0 keeps positions, C^1 the existing
   * tangent, C^2 curvature, etc. Do not synthesize a cylinder from the loop.
   */
  if (continuity >= 1) {
    fair_pin_constraint_rings(*out, affected, continuity);
  }

  n_free = 0;
  for (int i = 0; i < out->verts_num; i++) {
    if (affected[i]) {
      n_free++;
    }
  }
  if (n_free == 0) {
    r_error = "Fair: not enough free vertices after pinning constraint rings";
    BKE_id_free(nullptr, out);
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }

  const int order = continuity + 1;
  BKE_mesh_prefair_and_fair_verts(
      out, {}, affected.data(), eMeshFairingDepth(order), false);
  out->tag_positions_changed();
  return out;
}

Mesh *cgal_mesh_refine(const Mesh &mesh, float density_factor, std::string &r_error)
{
  /* Densify remesh: no parent maps; reproject only for this remesh-like op. */
  return run_mesh_op_with_attr_maps(
      mesh,
      [density_factor](const cgal_bridge::MeshIn &in) {
        return cgal_bridge::mesh_refine(in, double(density_factor));
      },
      r_error,
      false,
      true);
}

Mesh *cgal_mesh_clip_plane(const Mesh &mesh,
                           const float3 &plane_point,
                           const float3 &plane_normal,
                           std::string &r_error)
{
  return run_mesh_op_with_attr_maps(
      mesh,
      [&](const cgal_bridge::MeshIn &in) {
        return cgal_bridge::mesh_clip_plane(in,
                                            plane_point.x,
                                            plane_point.y,
                                            plane_point.z,
                                            plane_normal.x,
                                            plane_normal.y,
                                            plane_normal.z);
      },
      r_error);
}

Mesh *cgal_mesh_subdivision(const Mesh &mesh,
                            CgalSubdivisionMode mode,
                            int steps,
                            std::string &r_error)
{
  /* Catmull-Clark / DooSabin keep quads/n-gons; Loop/Sqrt3 need tris (bridge-side). */
  return run_mesh_op_with_attr_maps(
      mesh,
      [mode, steps](const cgal_bridge::MeshIn &in) {
        return cgal_bridge::mesh_subdivision(in, int(mode), steps);
      },
      r_error);
}

Mesh *cgal_mesh_repair(const Mesh &mesh, std::string &r_error)
{
  return run_mesh_op_with_attr_maps(
      mesh, [](const cgal_bridge::MeshIn &in) { return cgal_bridge::mesh_repair(in); }, r_error);
}

Mesh *cgal_mesh_keep_largest(const Mesh &mesh, std::string &r_error)
{
  return run_mesh_op_with_attr_maps(
      mesh,
      [](const cgal_bridge::MeshIn &in) { return cgal_bridge::mesh_keep_largest_component(in); },
      r_error);
}

Mesh *cgal_mesh_alpha_wrap(const Mesh &mesh, float alpha, float offset, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_alpha_wrap(
      in, double(alpha), double(offset));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    bke::mesh_smooth_set(*out, true);
  }
  return out;
}

static void apply_seam_edge_selection(Mesh &mesh,
                                      const cgal_bridge::MeshResult &result,
                                      Array<bool> &r_selection)
{
  r_selection.reinitialize(mesh.edges_num);
  r_selection.fill(false);
  if (result.seam_vert_a.empty() || result.seam_vert_a.size() != result.seam_vert_b.size()) {
    return;
  }
  /* Build set of unordered endpoint pairs. */
  Map<int2, bool> seam_pairs;
  seam_pairs.reserve(result.seam_vert_a.size());
  for (const int i : IndexRange(int(result.seam_vert_a.size()))) {
    const int va = result.seam_vert_a[i];
    const int vb = result.seam_vert_b[i];
    if (va < 0 || vb < 0 || va >= mesh.verts_num || vb >= mesh.verts_num) {
      continue;
    }
    seam_pairs.add(int2(math::min(va, vb), math::max(va, vb)), true);
  }
  const Span<int2> edges = mesh.edges();
  for (const int e : edges.index_range()) {
    const int2 ev = edges[e];
    if (seam_pairs.contains(int2(math::min(ev[0], ev[1]), math::max(ev[0], ev[1])))) {
      r_selection[e] = true;
    }
  }
}

Mesh *cgal_mesh_corefine(const Mesh &a,
                         const Mesh &b,
                         std::string &r_error,
                         Array<bool> *r_seam_edge_selection)
{
  cgal_bridge::MeshIn in_a = mesh_to_cgal_in(a);
  cgal_bridge::MeshIn in_b = mesh_to_cgal_in(b);
  if (in_a.faces_num <= 0 || in_b.faces_num <= 0) {
    r_error = "Both inputs need a non-empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_corefine(in_a, in_b);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    interpolate_attributes_boolean(*out, a, b, result);
    if (r_seam_edge_selection) {
      apply_seam_edge_selection(*out, result, *r_seam_edge_selection);
    }
  }
  return out;
}

Mesh *cgal_alpha_wrap_points(Span<float3> points,
                             float alpha,
                             float offset,
                             std::string &r_error)
{
  cgal_bridge::TriangleMeshResult result = cgal_bridge::alpha_wrap_points(
      points.cast<float>().data(), int(points.size()), double(alpha), double(offset));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    bke::mesh_smooth_set(*out, true);
  }
  return out;
}

Mesh *cgal_advancing_front(Span<float3> points,
                           float radius_ratio,
                           float beta,
                           std::string &r_error)
{
  cgal_bridge::TriangleMeshResult result = cgal_bridge::advancing_front_surface(
      points.cast<float>().data(), int(points.size()), double(radius_ratio), double(beta));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    bke::mesh_smooth_set(*out, true);
  }
  return out;
}

Mesh *cgal_delaunay_3d(Span<float3> points,
                       const PointCloud *src_pointcloud,
                       const Mesh *src_mesh,
                       bool separate_tets,
                       std::string &r_error)
{
  cgal_bridge::TriangleMeshResult result = cgal_bridge::delaunay_3d(
      points.cast<float>().data(), int(points.size()), separate_tets);
  Mesh *out = result_to_mesh(result, r_error);
  if (!out || out->faces_num == 0) {
    return out;
  }
  shade_all_sharp(*out);

  /* Transfer source point attributes to mesh vertices by nearest position. */
  if (src_pointcloud && src_pointcloud->totpoint > 0) {
    PointCloud *tmp = BKE_pointcloud_new_nomain(PointCloudType::Points, out->verts_num);
    tmp->positions_for_write().copy_from(out->vert_positions());
    tmp->tag_positions_changed();
    pointcloud_transfer_attributes_nearest(*src_pointcloud, *tmp, {"position"});
    bke::copy_attributes(tmp->attributes(),
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         bke::attribute_filter_from_skip_ref(Span<StringRef>({"position"})),
                         out->attributes_for_write());
    BKE_id_free(nullptr, tmp);
  }
  else if (src_mesh && src_mesh->verts_num > 0) {
    /* Nearest vertex on source mesh. */
    const bke::bvh::Tree &tree = src_mesh->bvh_verts();
    Array<int> nearest(out->verts_num);
    const Span<float3> dst_pos = out->vert_positions();
    for (const int i : dst_pos.index_range()) {
      if (const std::optional<bke::bvh::ClosestPointResult> hit = tree.closest_point(dst_pos[i]))
      {
        nearest[i] = int(hit->index);
      }
      else {
        nearest[i] = 0;
      }
    }
      const bke::AttributeAccessor src_attrs = src_mesh->attributes();
      bke::MutableAttributeAccessor dst_attrs = out->attributes_for_write();
      src_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
        if (iter.domain != bke::AttrDomain::Point || is_builtin_topology_attr(iter.name) ||
            iter.name == "position")
        {
          return;
        }
        const GVArraySpan src_data = *iter.get();
        bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_only_span(
            iter.name, bke::AttrDomain::Point, iter.data_type);
        if (!dst_w) {
          return;
        }
        bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
          const Span<T> s = src_data.typed<T>();
          MutableSpan<T> d = dst_w.span.typed<T>();
          for (const int i : d.index_range()) {
            const int si = nearest[i];
            d[i] = (si >= 0 && si < s.size()) ? s[si] : T();
          }
        });
        dst_w.finish();
      });
  }
  return out;
}

Mesh *cgal_min_sphere(Span<float3> points, int segments, std::string &r_error)
{
  cgal_bridge::TriangleMeshResult result = cgal_bridge::min_sphere(
      points.cast<float>().data(), int(points.size()), segments);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    bke::mesh_smooth_set(*out, true);
  }
  return out;
}

Mesh *cgal_optimal_bbox(Span<float3> points, std::string &r_error)
{
  cgal_bridge::TriangleMeshResult result = cgal_bridge::optimal_bbox(points.cast<float>().data(),
                                                                    int(points.size()));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}


Mesh *cgal_mesh_detect_features(const Mesh &mesh, float angle, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_detect_features(in, double(angle));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (out && out->faces_num > 0 && result.seam_vert_a.size() > 0) {
    /* Feature detection: write sharp_edge from seams. */
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    w.span.copy_from(seams);
    w.finish();
  }
  return out;
}


Mesh *cgal_mesh_angle_area_smooth(const Mesh &mesh, int iterations, bool safety, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_angle_area_smooth(in, iterations, safety);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (out && out->faces_num > 0 && result.seam_vert_a.size() > 0) {
    /* Feature detection: write sharp_edge from seams. */
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    w.span.copy_from(seams);
    w.finish();
  }
  return out;
}


Mesh *cgal_mesh_tangential_relaxation(const Mesh &mesh, int iterations, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_tangential_relaxation(in, iterations);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (out && out->faces_num > 0 && result.seam_vert_a.size() > 0) {
    /* Feature detection: write sharp_edge from seams. */
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    w.span.copy_from(seams);
    w.finish();
  }
  return out;
}


Mesh *cgal_mesh_extrude(const Mesh &mesh,
                        float distance,
                        const float3 &offset,
                        bool along_normals,
                        std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_extrude(
      in, distance, offset.x, offset.y, offset.z, along_normals);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  return out;
}


Mesh *cgal_mesh_remesh_planar_patches(const Mesh &mesh, float cos_angle, float max_dist, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_remesh_planar_patches(in, double(cos_angle), double(max_dist));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (out && out->faces_num > 0 && result.seam_vert_a.size() > 0) {
    /* Feature detection: write sharp_edge from seams. */
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    w.span.copy_from(seams);
    w.finish();
  }
  return out;
}


Mesh *cgal_mesh_random_perturbation(const Mesh &mesh, float max_move, bool project, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_random_perturbation(in, double(max_move), project);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (out && out->faces_num > 0 && result.seam_vert_a.size() > 0) {
    /* Feature detection: write sharp_edge from seams. */
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    w.span.copy_from(seams);
    w.finish();
  }
  return out;
}


Mesh *cgal_mesh_triangulate(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_triangulate(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (out && out->faces_num > 0 && result.seam_vert_a.size() > 0) {
    /* Feature detection: write sharp_edge from seams. */
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    w.span.copy_from(seams);
    w.finish();
  }
  return out;
}


/** True if B is the same cyclic order as A (any start). */
static bool poly_cyclic_equal(const Span<int> a, const Span<int> b)
{
  if (a.size() != b.size() || a.is_empty()) {
    return false;
  }
  const int n = int(a.size());
  for (int start = 0; start < n; start++) {
    bool ok = true;
    for (int i = 0; i < n; i++) {
      if (a[i] != b[(start + i) % n]) {
        ok = false;
        break;
      }
    }
    if (ok) {
      return true;
    }
  }
  return false;
}

/** True if B is reverse cyclic order of A. */
static bool poly_cyclic_reverse(const Span<int> a, const Span<int> b)
{
  if (a.size() != b.size() || a.is_empty()) {
    return false;
  }
  const int n = int(a.size());
  for (int start = 0; start < n; start++) {
    bool ok = true;
    for (int i = 0; i < n; i++) {
      if (a[i] != b[(start - i + n * 2) % n]) {
        ok = false;
        break;
      }
    }
    if (ok) {
      return true;
    }
  }
  return false;
}

Mesh *cgal_mesh_orient_outward(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_orient_outward(in);
  if (!result.ok) {
    r_error = result.error.empty() ? "CGAL orient failed" : result.error;
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }

  /*
   * Preferred path: same vertices + same face set → copy mesh and only flip
   * faces whose winding changed. Preserves all point/edge/face/corner attributes
   * (via mesh_flip_faces for corner data).
   */
  const bool same_verts = result.verts_num() == mesh.verts_num && result.has_vert_map();
  const bool same_faces = result.faces_num() == mesh.faces_num && result.has_face_map();
  if (same_verts && same_faces) {
    bool maps_identity = true;
    for (int i = 0; i < mesh.verts_num; i++) {
      if (result.vert_src0[size_t(i)] != i) {
        maps_identity = false;
        break;
      }
    }
    for (int f = 0; f < mesh.faces_num && maps_identity; f++) {
      if (result.face_src[size_t(f)] != f) {
        maps_identity = false;
      }
    }
    if (maps_identity && !result.face_offsets.empty()) {
      const OffsetIndices src_faces = mesh.faces();
      const Span<int> src_corners = mesh.corner_verts();
      Vector<int> faces_to_flip;
      faces_to_flip.reserve(mesh.faces_num);
      bool topology_ok = true;
      for (int f = 0; f < mesh.faces_num; f++) {
        const int b0 = result.face_offsets[size_t(f)];
        const int b1 = result.face_offsets[size_t(f + 1)];
        if (b1 - b0 != src_faces[f].size()) {
          topology_ok = false;
          break;
        }
        const Span<int> oriented(result.corner_verts.data() + b0, b1 - b0);
        const Span<int> original = src_corners.slice(src_faces[f]);
        if (poly_cyclic_equal(original, oriented)) {
          continue;
        }
        if (poly_cyclic_reverse(original, oriented)) {
          faces_to_flip.append(f);
          continue;
        }
        topology_ok = false;
        break;
      }
      if (topology_ok) {
        Mesh *out = BKE_mesh_copy_for_eval(mesh);
        if (!faces_to_flip.is_empty()) {
          IndexMaskMemory memory;
          const IndexMask mask = IndexMask::from_indices<int>(faces_to_flip.as_span(), memory);
          bke::mesh_flip_faces(*out, mask);
        }
        return out;
      }
    }
  }

  /* Fallback: rebuild mesh and interpolate attributes via maps. */
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  return out;
}


Mesh *cgal_mesh_repair_self_intersections(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_repair_self_intersections(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (out && out->faces_num > 0 && result.seam_vert_a.size() > 0) {
    /* Feature detection: write sharp_edge from seams. */
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    w.span.copy_from(seams);
    w.finish();
  }
  return out;
}


Mesh *cgal_mesh_split_long_edges(const Mesh &mesh, float max_length, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_split_long_edges(in, double(max_length));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (out && out->faces_num > 0 && result.seam_vert_a.size() > 0) {
    /* Feature detection: write sharp_edge from seams. */
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    w.span.copy_from(seams);
    w.finish();
  }
  return out;
}


Mesh *cgal_mesh_connected_component_keep(const Mesh &mesh, int index, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_connected_component_keep(in, index);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (out && out->faces_num > 0 && result.seam_vert_a.size() > 0) {
    /* Feature detection: write sharp_edge from seams. */
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    w.span.copy_from(seams);
    w.finish();
  }
  return out;
}


Mesh *cgal_mesh_merge_border_vertices(const Mesh &mesh, float distance, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_merge_border_vertices(in, double(distance));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else if (out && out->faces_num > 0 && result.seam_vert_a.size() > 0) {
    /* Feature detection: write sharp_edge from seams. */
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "sharp_edge", bke::AttrDomain::Edge);
    w.span.copy_from(seams);
    w.finish();
  }
  return out;
}


bool cgal_mesh_does_self_intersect(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return false;
  }
  return cgal_bridge::mesh_does_self_intersect(in, r_error);
}


float cgal_mesh_volume(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return 0.0f;
  }
  return float(cgal_bridge::mesh_volume(in, r_error));
}


float cgal_mesh_area(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return 0.0f;
  }
  return float(cgal_bridge::mesh_area(in, r_error));
}

static PointCloud *pointcloud_from_xyz_buffer(const std::vector<float> &xyz)
{
  const int n = int(xyz.size() / 3);
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, n);
  MutableSpan<float3> pos = pc->positions_for_write();
  for (int i = 0; i < n; i++) {
    pos[i] = float3(xyz[size_t(i) * 3 + 0], xyz[size_t(i) * 3 + 1], xyz[size_t(i) * 3 + 2]);
  }
  pc->tag_positions_changed();
  return pc;
}

static PointCloud *pointcloud_from_positions_copy(Span<float3> points)
{
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, int(points.size()));
  pc->positions_for_write().copy_from(points);
  pc->tag_positions_changed();
  return pc;
}

/** Copy point-domain attributes when source and destination have the same point count. */
static void pointcloud_copy_point_attributes(const PointCloud &src,
                                            PointCloud &dst,
                                            const Span<StringRef> skip_names)
{
  if (src.totpoint != dst.totpoint || src.totpoint == 0) {
    return;
  }
  bke::copy_attributes(src.attributes(),
                       bke::AttrDomain::Point,
                       bke::AttrDomain::Point,
                       bke::attribute_filter_from_skip_ref(skip_names),
                       dst.attributes_for_write());
}

/**
 * Transfer point attributes by nearest source position (for remapped / densified clouds).
 * Skips "position" and any names in \a skip_names.
 */
static void pointcloud_transfer_attributes_nearest(const PointCloud &src,
                                                  PointCloud &dst,
                                                  const Span<StringRef> skip_names)
{
  if (src.totpoint == 0 || dst.totpoint == 0) {
    return;
  }
  if (src.totpoint == dst.totpoint) {
    pointcloud_copy_point_attributes(src, dst, skip_names);
    return;
  }

  const bke::bvh::Tree &tree_data = src.bvh_tree();
  Array<int> nearest(dst.totpoint);
  const Span<float3> dst_pos = dst.positions();
  for (const int i : dst_pos.index_range()) {
    if (const std::optional<bke::bvh::ClosestPointResult> hit = tree_data.closest_point(
            dst_pos[i]))
    {
      nearest[i] = int(hit->index);
    }
    else {
      nearest[i] = 0;
    }
  }

  const bke::AttributeAccessor src_attrs = src.attributes();
  bke::MutableAttributeAccessor dst_attrs = dst.attributes_for_write();
  src_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != bke::AttrDomain::Point || iter.name == "position") {
      return;
    }
    for (const StringRef skip : skip_names) {
      if (iter.name == skip) {
        return;
      }
    }
    const GVArraySpan src_data = *iter.get();
    bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_only_span(
        iter.name, bke::AttrDomain::Point, iter.data_type);
    if (!dst_w) {
      return;
    }
    bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
      const Span<T> s = src_data.typed<T>();
      MutableSpan<T> d = dst_w.span.typed<T>();
      for (const int i : d.index_range()) {
        const int si = nearest[i];
        d[i] = (si >= 0 && si < s.size()) ? s[si] : T();
      }
    });
    dst_w.finish();
  });
}

/** Copy mesh point attributes onto a point cloud of matching size (mesh verts → points). */
static void pointcloud_copy_from_mesh_point_attributes(const Mesh &src,
                                                      PointCloud &dst,
                                                      const Span<StringRef> skip_names)
{
  if (src.verts_num != dst.totpoint || src.verts_num == 0) {
    return;
  }
  bke::copy_attributes(src.attributes(),
                       bke::AttrDomain::Point,
                       bke::AttrDomain::Point,
                       bke::attribute_filter_from_skip_ref(skip_names),
                       dst.attributes_for_write());
}

float cgal_points_average_spacing(Span<float3> points, int neighbors, std::string &r_error)
{
  return float(cgal_bridge::points_average_spacing(
      points.cast<float>().data(), int(points.size()), neighbors, r_error));
}

PointCloud *cgal_points_jet_smooth(Span<float3> points,
                                   int neighbors,
                                   int iterations,
                                   std::string &r_error)
{
  const int n = int(points.size());
  std::vector<float> out(size_t(n) * 3);
  if (!cgal_bridge::points_jet_smooth(
          points.cast<float>().data(), n, neighbors, iterations, out.data(), r_error))
  {
    return nullptr;
  }
  return pointcloud_from_xyz_buffer(out);
}

PointCloud *cgal_points_bilateral_smooth(Span<float3> points,
                                         int neighbors,
                                         int iterations,
                                         float sharpness_deg,
                                         std::string &r_error)
{
  const int n = int(points.size());
  std::vector<float> out(size_t(n) * 3);
  if (!cgal_bridge::points_bilateral_smooth(points.cast<float>().data(),
                                            n,
                                            neighbors,
                                            iterations,
                                            double(sharpness_deg),
                                            out.data(),
                                            r_error))
  {
    return nullptr;
  }
  return pointcloud_from_xyz_buffer(out);
}

PointCloud *cgal_points_remove_outliers(Span<float3> points,
                                        int neighbors,
                                        float percent,
                                        std::string &r_error)
{
  std::vector<float> out;
  if (!cgal_bridge::points_remove_outliers(
          points.cast<float>().data(), int(points.size()), neighbors, double(percent), out, r_error))
  {
    return nullptr;
  }
  return pointcloud_from_xyz_buffer(out);
}

PointCloud *cgal_points_grid_simplify(Span<float3> points, float cell_size, std::string &r_error)
{
  std::vector<float> out;
  if (!cgal_bridge::points_grid_simplify(
          points.cast<float>().data(), int(points.size()), double(cell_size), out, r_error))
  {
    return nullptr;
  }
  return pointcloud_from_xyz_buffer(out);
}

PointCloud *cgal_points_random_simplify(Span<float3> points, float percent, std::string &r_error)
{
  std::vector<float> out;
  if (!cgal_bridge::points_random_simplify(
          points.cast<float>().data(), int(points.size()), double(percent), out, r_error))
  {
    return nullptr;
  }
  return pointcloud_from_xyz_buffer(out);
}

PointCloud *cgal_points_estimate_normals(Span<float3> points,
                                         int neighbors,
                                         const PointCloud *src_pc,
                                         const Mesh *src_mesh,
                                         std::string &r_error)
{
  const int n = int(points.size());
  std::vector<float> nrm(size_t(n) * 3);
  if (!cgal_bridge::points_estimate_normals(
          points.cast<float>().data(), n, neighbors, nrm.data(), r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_pc) {
    pointcloud_copy_point_attributes(*src_pc, *pc, {"position", "normal"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(*src_mesh, *pc, {"position", "normal"});
  }
  bke::MutableAttributeAccessor attrs = pc->attributes_for_write();
  bke::SpanAttributeWriter<float3> w = attrs.lookup_or_add_for_write_only_span<float3>(
      "normal", bke::AttrDomain::Point);
  for (int i = 0; i < n; i++) {
    w.span[i] = float3(nrm[size_t(i) * 3 + 0], nrm[size_t(i) * 3 + 1], nrm[size_t(i) * 3 + 2]);
  }
  w.finish();
  return pc;
}

PointCloud *cgal_points_wlop(Span<float3> points,
                             float select_percentage,
                             float neighbor_radius,
                             int iterations,
                             bool require_uniform,
                             const PointCloud *src_attrs,
                             std::string &r_error)
{
  std::vector<float> out;
  if (!cgal_bridge::points_wlop(points.cast<float>().data(),
                                int(points.size()),
                                double(select_percentage),
                                double(neighbor_radius),
                                iterations,
                                require_uniform,
                                out,
                                r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_xyz_buffer(out);
  if (src_attrs && pc) {
    pointcloud_transfer_attributes_nearest(*src_attrs, *pc, {"position"});
  }
  return pc;
}

PointCloud *cgal_points_hierarchy_simplify(Span<float3> points,
                                           int cluster_size,
                                           float max_variation,
                                           const PointCloud *src_attrs,
                                           std::string &r_error)
{
  std::vector<float> out;
  if (!cgal_bridge::points_hierarchy_simplify(points.cast<float>().data(),
                                              int(points.size()),
                                              cluster_size,
                                              double(max_variation),
                                              out,
                                              r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_xyz_buffer(out);
  if (src_attrs && pc) {
    pointcloud_transfer_attributes_nearest(*src_attrs, *pc, {"position"});
  }
  return pc;
}

PointCloud *cgal_points_edge_aware_upsample(Span<float3> points,
                                            Span<float3> normals,
                                            int output_count,
                                            float sharpness_angle_deg,
                                            float edge_sensitivity,
                                            float neighbor_radius,
                                            int normal_neighbors,
                                            const PointCloud *src_pc,
                                            const Mesh *src_mesh,
                                            std::string &r_error)
{
  std::vector<float> out_xyz, out_n;
  const float *nptr = nullptr;
  std::vector<float> nbuf;
  if (normals.size() == points.size() && !normals.is_empty()) {
    nbuf.resize(size_t(points.size()) * 3);
    for (const int i : points.index_range()) {
      nbuf[size_t(i) * 3 + 0] = normals[i].x;
      nbuf[size_t(i) * 3 + 1] = normals[i].y;
      nbuf[size_t(i) * 3 + 2] = normals[i].z;
    }
    nptr = nbuf.data();
  }
  if (!cgal_bridge::points_edge_aware_upsample(points.cast<float>().data(),
                                               nptr,
                                               int(points.size()),
                                               output_count,
                                               double(sharpness_angle_deg),
                                               double(edge_sensitivity),
                                               double(neighbor_radius),
                                               normal_neighbors,
                                               out_xyz,
                                               out_n,
                                               r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_xyz_buffer(out_xyz);
  const int n_out = int(out_xyz.size() / 3);
  const int n_in = int(points.size());
  /* Originals are first n_in points (same order): copy attrs by index, then nearest for rest. */
  if (pc && src_pc && src_pc->totpoint == n_in && n_out >= n_in) {
    PointCloud *prefix = BKE_pointcloud_new_nomain(PointCloudType::Points, n_in);
    prefix->positions_for_write().copy_from(src_pc->positions());
    pointcloud_copy_point_attributes(*src_pc, *prefix, {"position", "normal"});
    pointcloud_transfer_attributes_nearest(*src_pc, *pc, {"position", "normal"});
    const bke::AttributeAccessor pref_attrs = prefix->attributes();
    bke::MutableAttributeAccessor dst_attrs = pc->attributes_for_write();
    pref_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
      if (iter.domain != bke::AttrDomain::Point || iter.name == "position" ||
          iter.name == "normal")
      {
        return;
      }
      const GVArraySpan src_data = *iter.get();
      bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_only_span(
          iter.name, bke::AttrDomain::Point, iter.data_type);
      if (!dst_w) {
        return;
      }
      bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
        const Span<T> s = src_data.typed<T>();
        MutableSpan<T> d = dst_w.span.typed<T>();
        for (int i = 0; i < n_in && i < d.size(); i++) {
          d[i] = s[i];
        }
      });
      dst_w.finish();
    });
    BKE_id_free(nullptr, prefix);
  }
  else if (pc && src_pc) {
    pointcloud_transfer_attributes_nearest(*src_pc, *pc, {"position", "normal"});
  }
  else if (pc && src_mesh && src_mesh->verts_num == n_in && n_out >= n_in) {
    /* Mesh verts → densified cloud: exact index for original prefix, nearest for new. */
    PointCloud *tmp = BKE_pointcloud_new_nomain(PointCloudType::Points, n_in);
    tmp->positions_for_write().copy_from(src_mesh->vert_positions());
    pointcloud_copy_from_mesh_point_attributes(*src_mesh, *tmp, {"position", "normal"});
    pointcloud_transfer_attributes_nearest(*tmp, *pc, {"position", "normal"});
    const bke::AttributeAccessor pref_attrs = tmp->attributes();
    bke::MutableAttributeAccessor dst_attrs = pc->attributes_for_write();
    pref_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
      if (iter.domain != bke::AttrDomain::Point || iter.name == "position" ||
          iter.name == "normal")
      {
        return;
      }
      const GVArraySpan src_data = *iter.get();
      bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_only_span(
          iter.name, bke::AttrDomain::Point, iter.data_type);
      if (!dst_w) {
        return;
      }
      bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
        const Span<T> s = src_data.typed<T>();
        MutableSpan<T> d = dst_w.span.typed<T>();
        for (int i = 0; i < n_in && i < d.size(); i++) {
          d[i] = s[i];
        }
      });
      dst_w.finish();
    });
    BKE_id_free(nullptr, tmp);
  }
  bke::SpanAttributeWriter<float3> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<float3>("normal",
                                                                           bke::AttrDomain::Point);
  for (int i = 0; i < n_out; i++) {
    w.span[i] = float3(out_n[size_t(i) * 3 + 0], out_n[size_t(i) * 3 + 1], out_n[size_t(i) * 3 + 2]);
  }
  w.finish();
  return pc;
}

PointCloud *cgal_points_vcm_estimate_normals(Span<float3> points,
                                             float offset_radius,
                                             float convolution_radius,
                                             const PointCloud *src_pc,
                                             const Mesh *src_mesh,
                                             std::string &r_error)
{
  const int n = int(points.size());
  std::vector<float> nrm(size_t(n) * 3);
  if (!cgal_bridge::points_vcm_estimate_normals(points.cast<float>().data(),
                                                n,
                                                double(offset_radius),
                                                double(convolution_radius),
                                                nrm.data(),
                                                r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_pc) {
    pointcloud_copy_point_attributes(*src_pc, *pc, {"position", "normal"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(*src_mesh, *pc, {"position", "normal"});
  }
  bke::SpanAttributeWriter<float3> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<float3>("normal",
                                                                           bke::AttrDomain::Point);
  for (int i = 0; i < n; i++) {
    w.span[i] = float3(nrm[size_t(i) * 3 + 0], nrm[size_t(i) * 3 + 1], nrm[size_t(i) * 3 + 2]);
  }
  w.finish();
  return pc;
}

Mesh *cgal_points_poisson(Span<float3> points,
                          Span<float3> normals,
                          float spacing,
                          int neighbors,
                          std::string &r_error)
{
  const int n = int(points.size());
  std::vector<float> nrm(size_t(n) * 3);
  if (normals.size() == points.size()) {
    for (int i = 0; i < n; i++) {
      nrm[size_t(i) * 3 + 0] = normals[i].x;
      nrm[size_t(i) * 3 + 1] = normals[i].y;
      nrm[size_t(i) * 3 + 2] = normals[i].z;
    }
  }
  else {
    if (!cgal_bridge::points_estimate_normals(
            points.cast<float>().data(), n, neighbors, nrm.data(), r_error))
    {
      return BKE_mesh_new_nomain(0, 0, 0, 0);
    }
  }
  double sp = double(spacing);
  if (!(sp > 0.0)) {
    std::string spacing_err;
    const double avg = cgal_bridge::points_average_spacing(
        points.cast<float>().data(), n, std::max(neighbors, 6), spacing_err);
    sp = (avg > 0.0) ? (avg * 0.5) : 0.05;
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_poisson_reconstruct(
      points.cast<float>().data(), nrm.data(), n, sp);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

PointCloud *cgal_mesh_side_of(const Mesh &mesh,
                              Span<float3> query,
                              const PointCloud *src_pointcloud,
                              const Mesh *src_mesh,
                              std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0 || query.is_empty()) {
    r_error = "Empty mesh or query";
    return nullptr;
  }
  std::vector<int8_t> side;
  if (!cgal_bridge::mesh_side_of(
          in, query.cast<float>().data(), int(query.size()), side, r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(query);
  if (src_pointcloud) {
    pointcloud_copy_point_attributes(*src_pointcloud, *pc, {"position", "Inside"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(*src_mesh, *pc, {"position", "Inside"});
  }
  bke::MutableAttributeAccessor attrs = pc->attributes_for_write();
  bke::SpanAttributeWriter<bool> inside_w = attrs.lookup_or_add_for_write_only_span<bool>(
      "Inside", bke::AttrDomain::Point);
  for (const int i : query.index_range()) {
    inside_w.span[i] = side[size_t(i)] > 0;
  }
  inside_w.finish();
  return pc;
}

bool cgal_mesh_side_of_query(const Mesh &mesh,
                             Span<float3> query,
                             MutableSpan<bool> r_inside,
                             std::string &r_error)
{
  if (r_inside.size() != query.size()) {
    r_error = "Query size mismatch";
    return false;
  }
  r_inside.fill(false);
  if (query.is_empty()) {
    return true;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return false;
  }
  std::vector<int8_t> side;
  if (!cgal_bridge::mesh_side_of(
          in, query.cast<float>().data(), int(query.size()), side, r_error))
  {
    return false;
  }
  const int n = int(query.size());
  for (int i = 0; i < n; i++) {
    r_inside[i] = side[size_t(i)] > 0;
  }
  return true;
}

PointCloud *cgal_mesh_distance_to(const Mesh &mesh, Span<float3> query, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0 || query.is_empty()) {
    r_error = "Empty mesh or query";
    return nullptr;
  }
  std::vector<float> dist;
  if (!cgal_bridge::mesh_distance_to(
          in, query.cast<float>().data(), int(query.size()), dist, r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(query);
  bke::MutableAttributeAccessor attrs = pc->attributes_for_write();
  bke::SpanAttributeWriter<float> w = attrs.lookup_or_add_for_write_only_span<float>(
      "Distance", bke::AttrDomain::Point);
  w.span.copy_from(Span(dist.data(), dist.size()));
  w.finish();
  return pc;
}

PointCloud *cgal_mesh_sample_points(const Mesh &mesh, int count, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return nullptr;
  }
  std::vector<float> xyz;
  if (!cgal_bridge::mesh_sample_points(in, count, xyz, r_error)) {
    return nullptr;
  }
  return pointcloud_from_xyz_buffer(xyz);
}

static Mesh *wire_result_to_mesh(const cgal_bridge::WireResult &wire, std::string &r_error)
{
  if (!wire.ok) {
    r_error = wire.error.empty() ? "Wire mesh failed" : wire.error;
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  const int verts_num = int(wire.positions.size() / 3);
  const int edges_num = int(wire.edge_v0.size());
  if (verts_num <= 0 || edges_num <= 0 || wire.edge_v1.size() != wire.edge_v0.size()) {
    r_error = "Invalid wire result";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  for (int i = 0; i < edges_num; i++) {
    if (wire.edge_v0[size_t(i)] < 0 || wire.edge_v0[size_t(i)] >= verts_num ||
        wire.edge_v1[size_t(i)] < 0 || wire.edge_v1[size_t(i)] >= verts_num)
    {
      r_error = "Wire edge has invalid vertex index";
      return BKE_mesh_new_nomain(0, 0, 0, 0);
    }
  }
  Mesh *mesh = BKE_mesh_new_nomain(verts_num, edges_num, 0, 0);
  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  for (int i = 0; i < verts_num; i++) {
    positions[i] = float3(wire.positions[size_t(i) * 3 + 0],
                          wire.positions[size_t(i) * 3 + 1],
                          wire.positions[size_t(i) * 3 + 2]);
  }
  MutableSpan<int2> edges = mesh->edges_for_write();
  for (int i = 0; i < edges_num; i++) {
    edges[i] = int2(wire.edge_v0[size_t(i)], wire.edge_v1[size_t(i)]);
  }
  mesh->tag_topology_changed();
  return mesh;
}

Mesh *cgal_mesh_skeleton(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(cgal_bridge::mesh_skeleton(in), r_error);
}

Mesh *cgal_mesh_extract_border(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(cgal_bridge::mesh_extract_border(in), r_error);
}

Mesh *cgal_mesh_geodesic_distance(const Mesh &mesh,
                                  Span<float3> source_points,
                                  std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0 || in.verts_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  std::vector<int> sources;
  if (source_points.is_empty()) {
    sources.push_back(0);
  }
  else {
    const Span<float3> verts = mesh.vert_positions();
    sources.reserve(source_points.size());
    for (const float3 &sp : source_points) {
      int best = 0;
      float best_d = math::distance_squared(verts[0], sp);
      for (int i = 1; i < verts.size(); i++) {
        const float d = math::distance_squared(verts[i], sp);
        if (d < best_d) {
          best_d = d;
          best = i;
        }
      }
      sources.push_back(best);
    }
  }
  std::vector<float> dist;
  if (!cgal_bridge::mesh_geodesic_distances(in, sources, dist, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  if (int(dist.size()) != out->verts_num) {
    r_error = "Geodesic distance count mismatch";
    BKE_id_free(nullptr, out);
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  bke::MutableAttributeAccessor attrs = out->attributes_for_write();
  bke::SpanAttributeWriter<float> w = attrs.lookup_or_add_for_write_only_span<float>(
      "Geodesic", bke::AttrDomain::Point);
  w.span.copy_from(Span(dist.data(), dist.size()));
  w.finish();
  return out;
}

Mesh *cgal_mesh_segmentation(const Mesh &mesh,
                             int clusters,
                             float smoothing,
                             std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_segmentation(in, clusters, double(smoothing));
  Mesh *out = result_to_mesh(result, r_error);
  if (!out || out->faces_num == 0) {
    return out;
  }
  /* face_src keeps original face parents; transfer attrs first. */
  if (result.has_face_map() || result.has_vert_map()) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else {
    BKE_mesh_copy_parameters_for_eval(out, &mesh);
    bke::mesh_remesh_reproject_attributes(mesh, *out);
  }
  if (int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> seg_w = attrs.lookup_or_add_for_write_only_span<int>(
        "Segment", bke::AttrDomain::Face);
    seg_w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    seg_w.finish();
  }
  if (int(result.face_scalar.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<float> sdf_w = attrs.lookup_or_add_for_write_only_span<float>(
        "SDF", bke::AttrDomain::Face);
    sdf_w.span.copy_from(Span(result.face_scalar.data(), result.face_scalar.size()));
    sdf_w.finish();
  }
  return out;
}

Mesh *cgal_min_ellipsoid(Span<float3> points, int segments, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::min_ellipsoid(
      points.cast<float>().data(), int(points.size()), segments);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

bool cgal_mesh_is_closed(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return false;
  }
  return cgal_bridge::mesh_is_closed(in, r_error);
}

bool cgal_mesh_centroid(const Mesh &mesh, float3 &r_centroid, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return false;
  }
  float xyz[3];
  if (!cgal_bridge::mesh_centroid(in, xyz, r_error)) {
    return false;
  }
  r_centroid = float3(xyz[0], xyz[1], xyz[2]);
  return true;
}

Mesh *cgal_mesh_autorefine(const Mesh &mesh, std::string &r_error)
{
  return run_mesh_op_with_attr_maps(
      mesh,
      [](const cgal_bridge::MeshIn &in) { return cgal_bridge::mesh_autorefine(in); },
      r_error,
      false,
      true);
}

Mesh *cgal_mesh_remove_degenerate(const Mesh &mesh, std::string &r_error)
{
  return run_mesh_op_with_attr_maps(
      mesh,
      [](const cgal_bridge::MeshIn &in) { return cgal_bridge::mesh_remove_degenerate(in); },
      r_error,
      false,
      true);
}

bool cgal_mesh_mean_curvature(const Mesh &mesh, MutableSpan<float> r_values, std::string &r_error)
{
  if (r_values.size() != mesh.verts_num) {
    r_error = "Attribute size mismatch";
    return false;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    r_values.fill(0.0f);
    return false;
  }
  std::vector<float> h;
  if (!cgal_bridge::mesh_mean_curvature(in, h, r_error)) {
    r_values.fill(0.0f);
    return false;
  }
  if (int(h.size()) != mesh.verts_num) {
    r_error = "Curvature size mismatch";
    r_values.fill(0.0f);
    return false;
  }
  r_values.copy_from(Span(h.data(), h.size()));
  return true;
}

bool cgal_mesh_gaussian_curvature(const Mesh &mesh,
                                  MutableSpan<float> r_values,
                                  std::string &r_error)
{
  if (r_values.size() != mesh.verts_num) {
    r_error = "Attribute size mismatch";
    return false;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    r_values.fill(0.0f);
    return false;
  }
  std::vector<float> k;
  if (!cgal_bridge::mesh_gaussian_curvature(in, k, r_error)) {
    r_values.fill(0.0f);
    return false;
  }
  if (int(k.size()) != mesh.verts_num) {
    r_error = "Curvature size mismatch";
    r_values.fill(0.0f);
    return false;
  }
  r_values.copy_from(Span(k.data(), k.size()));
  return true;
}

Mesh *cgal_points_scale_space(Span<float3> points, int iterations, std::string &r_error)
{
  return cgal_points_scale_space_ex(points, iterations, 0, 0, r_error);
}

Mesh *cgal_points_scale_space_ex(Span<float3> points,
                                 int iterations,
                                 int smoother,
                                 int mesher,
                                 std::string &r_error)
{
  if (points.size() < 4) {
    r_error = "Scale Space needs at least 4 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_scale_space_reconstruct_ex(
      points.cast<float>().data(), int(points.size()), iterations, smoother, mesher);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_repair_degeneracies(const Mesh &mesh,
                                    float cap_threshold,
                                    float needle_threshold,
                                    float collapse_length,
                                    std::string &r_error)
{
  /* Same attribute path as Remove Degenerate: topology maps + reproject fallback. */
  return run_mesh_op_with_attr_maps(
      mesh,
      [cap_threshold, needle_threshold, collapse_length](const cgal_bridge::MeshIn &in) {
        return cgal_bridge::mesh_repair_degeneracies(
            in, double(cap_threshold), double(needle_threshold), double(collapse_length));
      },
      r_error,
      false,
      true);
}

bool cgal_mesh_surface_intersection(const Mesh &a,
                                    const Mesh &b,
                                    Vector<Vector<float3>> &r_polylines,
                                    std::string &r_error)
{
  r_polylines.clear();
  cgal_bridge::MeshIn in_a = mesh_to_cgal_in(a);
  cgal_bridge::MeshIn in_b = mesh_to_cgal_in(b);
  if (in_a.faces_num <= 0 || in_b.faces_num <= 0) {
    r_error = "Both meshes need faces";
    return false;
  }
  std::vector<std::vector<float>> raw;
  if (!cgal_bridge::mesh_surface_intersection_polylines(in_a, in_b, raw, r_error)) {
    return false;
  }
  r_polylines.reserve(raw.size());
  for (const std::vector<float> &poly : raw) {
    if (poly.size() < 6) {
      continue;
    }
    Vector<float3> pts;
    pts.reserve(poly.size() / 3);
    for (std::size_t i = 0; i + 2 < poly.size(); i += 3) {
      pts.append(float3(poly[i], poly[i + 1], poly[i + 2]));
    }
    if (pts.size() >= 2) {
      r_polylines.append(std::move(pts));
    }
  }
  return true;
}

Mesh *cgal_mesh_refine_at_isolevel(const Mesh &mesh,
                                   Span<float> vertex_values,
                                   float isovalue,
                                   Array<bool> *r_isoline_edges,
                                   std::string &r_error)
{
  if (vertex_values.size() != mesh.verts_num) {
    r_error = "Vertex value count must match vertex count";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_refine_at_isolevel(
      in, vertex_values.data(), double(isovalue));
  Mesh *out = result_to_mesh(result, r_error);
  if (!out || out->faces_num == 0) {
    return out;
  }
  if (result.has_face_map() || result.has_vert_map()) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  else {
    BKE_mesh_copy_parameters_for_eval(out, &mesh);
    bke::mesh_remesh_reproject_attributes(mesh, *out);
  }
  if (r_isoline_edges && result.seam_vert_a.size() == result.seam_vert_b.size() &&
      !result.seam_vert_a.empty())
  {
    Array<bool> seams;
    apply_seam_edge_selection(*out, result, seams);
    *r_isoline_edges = std::move(seams);
  }
  return out;
}

bool cgal_mesh_exact_geodesic_distances(const Mesh &mesh,
                                        Span<bool> source_mask,
                                        MutableSpan<float> r_distance,
                                        std::string &r_error)
{
  if (r_distance.size() != mesh.verts_num) {
    r_error = "Distance buffer size mismatch";
    return false;
  }
  if (source_mask.size() != mesh.verts_num) {
    r_error = "Source mask size mismatch";
    return false;
  }
  std::vector<int> sources;
  for (const int i : IndexRange(mesh.verts_num)) {
    if (source_mask[i]) {
      sources.push_back(i);
    }
  }
  if (sources.empty()) {
    r_distance.fill(-1.0f);
    r_error = "No source vertices selected";
    return false;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<float> dist;
  if (!cgal_bridge::mesh_exact_geodesic_distances(in, sources, dist, r_error)) {
    r_distance.fill(-1.0f);
    return false;
  }
  if (int(dist.size()) != mesh.verts_num) {
    r_error = "Geodesic distance count mismatch";
    r_distance.fill(-1.0f);
    return false;
  }
  r_distance.copy_from(Span(dist.data(), dist.size()));
  return true;
}

Mesh *cgal_points_skin_surface(Span<float3> points,
                               Span<float> radii,
                               float default_radius,
                               float shrink_factor,
                               int subdivisions,
                               std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "Skin Surface needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  std::vector<float> rad_buf;
  const float *rad_ptr = nullptr;
  if (!radii.is_empty() && radii.size() == points.size()) {
    rad_ptr = radii.data();
  }
  else {
    rad_buf.assign(size_t(points.size()), std::max(default_radius, 1e-6f));
    rad_ptr = rad_buf.data();
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_skin_surface(points.cast<float>().data(),
                                                                    rad_ptr,
                                                                    int(points.size()),
                                                                    double(shrink_factor),
                                                                    subdivisions);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_union_of_balls(Span<float3> points,
                                 Span<float> radii,
                                 float default_radius,
                                 int subdivisions,
                                 std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "Union of Balls needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  std::vector<float> rad_buf;
  const float *rad_ptr = nullptr;
  if (!radii.is_empty() && radii.size() == points.size()) {
    rad_ptr = radii.data();
  }
  else {
    rad_buf.assign(size_t(points.size()), std::max(default_radius, 1e-6f));
    rad_ptr = rad_buf.data();
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_union_of_balls(
      points.cast<float>().data(), rad_ptr, int(points.size()), subdivisions);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_min_sphere_of_spheres(Span<float3> points,
                                        Span<float> radii,
                                        float default_radius,
                                        int segments,
                                        float3 &r_center,
                                        float &r_radius,
                                        std::string &r_error)
{
  r_center = float3(0.0f);
  r_radius = 0.0f;
  if (points.is_empty()) {
    r_error = "Min Sphere of Spheres needs at least 1 ball";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  std::vector<float> rad_buf;
  const float *rad_ptr = nullptr;
  if (!radii.is_empty() && radii.size() == points.size()) {
    rad_ptr = radii.data();
  }
  else {
    rad_buf.assign(size_t(points.size()), std::max(default_radius, 0.0f));
    rad_ptr = rad_buf.data();
  }
  cgal_bridge::MeshResult result = cgal_bridge::min_sphere_of_spheres(
      points.cast<float>().data(), rad_ptr, int(points.size()), segments);
  if (result.ok) {
    r_center = float3(result.center[0], result.center[1], result.center[2]);
    r_radius = result.radius;
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_vsa_approximate(const Mesh &mesh,
                                int max_proxies,
                                int iterations,
                                int seeding_method,
                                int &r_proxy_count,
                                std::string &r_error)
{
  r_proxy_count = 0;
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_vsa_approximate(
      in, max_proxies, iterations, seeding_method);
  if (result.ok) {
    if (!result.face_tag.empty()) {
      r_proxy_count = result.face_tag[0];
    }
    else {
      r_proxy_count = int(result.radius);
    }
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_min_circle_2(Span<float3> points,
                               int plane,
                               int segments,
                               float3 &r_center,
                               float &r_radius,
                               std::string &r_error)
{
  r_center = float3(0.0f);
  r_radius = 0.0f;
  if (points.size() < 2) {
    r_error = "Min Circle 2D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_min_circle_2(
      points.cast<float>().data(), int(points.size()), plane, segments);
  if (result.ok) {
    r_center = float3(result.center[0], result.center[1], result.center[2]);
    r_radius = result.radius;
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_min_rectangle_2(Span<float3> points,
                                  int plane,
                                  float3 &r_center,
                                  float &r_radius,
                                  std::string &r_error)
{
  r_center = float3(0.0f);
  r_radius = 0.0f;
  if (points.size() < 2) {
    r_error = "Min Rectangle 2D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_min_rectangle_2(
      points.cast<float>().data(), int(points.size()), plane);
  if (result.ok) {
    r_center = float3(result.center[0], result.center[1], result.center[2]);
    r_radius = result.radius;
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

PointCloud *cgal_points_structure(Span<float3> points,
                                  Span<float3> normals,
                                  float epsilon,
                                  float attraction_factor,
                                  float ransac_epsilon,
                                  float ransac_cluster_epsilon,
                                  int min_points,
                                  std::string &r_error)
{
  const float *nptr = nullptr;
  if (!normals.is_empty() && normals.size() == points.size()) {
    nptr = normals.cast<float>().data();
  }
  std::vector<float> out_xyz, out_n;
  if (!cgal_bridge::points_structure(points.cast<float>().data(),
                                     nptr,
                                     int(points.size()),
                                     double(epsilon),
                                     double(attraction_factor),
                                     double(ransac_epsilon),
                                     double(ransac_cluster_epsilon),
                                     min_points,
                                     out_xyz,
                                     out_n,
                                     r_error))
  {
    return nullptr;
  }
  const int n = int(out_xyz.size() / 3);
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, n);
  MutableSpan<float3> pos = pc->positions_for_write();
  for (int i = 0; i < n; i++) {
    pos[i] = float3(out_xyz[size_t(i) * 3 + 0], out_xyz[size_t(i) * 3 + 1], out_xyz[size_t(i) * 3 + 2]);
  }
  bke::MutableAttributeAccessor attrs = pc->attributes_for_write();
  bke::SpanAttributeWriter<float3> nw = attrs.lookup_or_add_for_write_only_span<float3>(
      "normal", bke::AttrDomain::Point);
  for (int i = 0; i < n; i++) {
    nw.span[i] = float3(out_n[size_t(i) * 3 + 0], out_n[size_t(i) * 3 + 1], out_n[size_t(i) * 3 + 2]);
  }
  nw.finish();
  return pc;
}

bool cgal_mesh_principal_curvatures(const Mesh &mesh,
                                    MutableSpan<float> r_kmin,
                                    MutableSpan<float> r_kmax,
                                    MutableSpan<float3> r_dmin,
                                    MutableSpan<float3> r_dmax,
                                    std::string &r_error)
{
  if (r_kmin.size() != mesh.verts_num || r_kmax.size() != mesh.verts_num ||
      r_dmin.size() != mesh.verts_num || r_dmax.size() != mesh.verts_num)
  {
    r_error = "Vertex count mismatch";
    return false;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    r_kmin.fill(0.0f);
    r_kmax.fill(0.0f);
    r_dmin.fill(float3(0.0f));
    r_dmax.fill(float3(0.0f));
    return false;
  }
  std::vector<float> kmin, kmax, dmin, dmax;
  if (!cgal_bridge::mesh_principal_curvatures(in, kmin, kmax, dmin, dmax, r_error)) {
    r_kmin.fill(0.0f);
    r_kmax.fill(0.0f);
    r_dmin.fill(float3(0.0f));
    r_dmax.fill(float3(0.0f));
    return false;
  }
  if (int(kmin.size()) != mesh.verts_num || int(kmax.size()) != mesh.verts_num ||
      int(dmin.size()) != mesh.verts_num * 3 || int(dmax.size()) != mesh.verts_num * 3)
  {
    r_error = "Principal curvature size mismatch";
    r_kmin.fill(0.0f);
    r_kmax.fill(0.0f);
    r_dmin.fill(float3(0.0f));
    r_dmax.fill(float3(0.0f));
    return false;
  }
  r_kmin.copy_from(Span(kmin.data(), kmin.size()));
  r_kmax.copy_from(Span(kmax.data(), kmax.size()));
  for (int i = 0; i < mesh.verts_num; i++) {
    r_dmin[i] = float3(dmin[size_t(i) * 3 + 0], dmin[size_t(i) * 3 + 1], dmin[size_t(i) * 3 + 2]);
    r_dmax[i] = float3(dmax[size_t(i) * 3 + 0], dmax[size_t(i) * 3 + 1], dmax[size_t(i) * 3 + 2]);
  }
  return true;
}

bool cgal_mesh_shape_diameter(const Mesh &mesh, MutableSpan<float> r_sdf, std::string &r_error)
{
  if (r_sdf.size() != mesh.faces_num) {
    r_error = "Face count mismatch";
    return false;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    r_sdf.fill(0.0f);
    return false;
  }
  std::vector<float> sdf;
  if (!cgal_bridge::mesh_shape_diameter(in, sdf, r_error)) {
    r_sdf.fill(0.0f);
    return false;
  }
  if (int(sdf.size()) != mesh.faces_num) {
    r_error = "SDF size mismatch";
    r_sdf.fill(0.0f);
    return false;
  }
  r_sdf.copy_from(Span(sdf.data(), sdf.size()));
  return true;
}

bool cgal_mesh_mark_self_intersect(const Mesh &mesh,
                                   MutableSpan<bool> r_intersect,
                                   MutableSpan<bool> r_inside,
                                   std::string &r_error)
{
  r_intersect.fill(false);
  r_inside.fill(false);
  if (r_intersect.size() != mesh.faces_num) {
    r_error = "Face count mismatch";
    return false;
  }
  if (!r_inside.is_empty() && r_inside.size() != mesh.faces_num) {
    r_error = "Face count mismatch";
    return false;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return false;
  }
  std::vector<int8_t> hit;
  try {
    if (!cgal_bridge::mesh_mark_self_intersect_faces(in, hit, r_error)) {
      return false;
    }
  }
  catch (const std::exception &e) {
    r_error = e.what();
    return false;
  }
  catch (...) {
    r_error = "Self-intersect mark failed";
    return false;
  }
  if (int(hit.size()) != mesh.faces_num) {
    r_error = "Self-intersect face count mismatch";
    return false;
  }
  for (const int i : IndexRange(mesh.faces_num)) {
    r_intersect[i] = hit[size_t(i)] != 0;
  }
  if (r_inside.is_empty()) {
    return true;
  }

  /* Generalized winding number (solid angle). Does not use CGAL Side_of —
   * that AVs on self-intersections. A face is in the self-intersection
   * interior when a sample just off the face has |w| >= 2 (inside two
   * sheets). Intersection faces stay false. */
  const int nfaces = mesh.faces_num;
  const OffsetIndices<int> faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<float3> positions = mesh.vert_positions();
  const Span<float3> normals = mesh.face_normals();
  const Span<int3> tris = mesh.corner_tris();
  const Span<int> tri_faces = mesh.corner_tri_faces();
  if (tris.is_empty()) {
    return true;
  }

  float scale = 1.0f;
  if (const std::optional<Bounds<float3>> bounds = mesh.bounds_min_max()) {
    scale = math::length(bounds->size());
  }
  const float eps = math::max(1e-5f, 1e-4f * math::max(scale, 1e-8f));
  constexpr float four_pi = 4.0f * float(M_PI);
  constexpr float k_overlap = 1.5f;

  auto solid_angle = [](const float3 &p, const float3 &a, const float3 &b, const float3 &c) {
    float3 va = a - p;
    float3 vb = b - p;
    float3 vc = c - p;
    const float la = math::length(va);
    const float lb = math::length(vb);
    const float lc = math::length(vc);
    if (la < 1e-12f || lb < 1e-12f || lc < 1e-12f) {
      return 0.0f;
    }
    va /= la;
    vb /= lb;
    vc /= lc;
    const float triple = math::dot(va, math::cross(vb, vc));
    const float denom = 1.0f + math::dot(va, vb) + math::dot(vb, vc) + math::dot(vc, va);
    return 2.0f * std::atan2(triple, denom);
  };
  auto winding_at = [&](const float3 &p, const int skip_face) {
    float sum = 0.0f;
    for (const int t : tris.index_range()) {
      if (tri_faces[t] == skip_face) {
        continue;
      }
      const int3 &tri = tris[t];
      sum += solid_angle(p,
                         positions[corner_verts[tri[0]]],
                         positions[corner_verts[tri[1]]],
                         positions[corner_verts[tri[2]]]);
    }
    return sum / four_pi;
  };

  threading::parallel_for(IndexRange(nfaces), 32, [&](const IndexRange range) {
    for (const int f : range) {
      const float3 c = bke::mesh::face_center_calc(positions, corner_verts.slice(faces[f]));
      const float3 n = normals[f];
      const float wp = winding_at(c + n * eps, f);
      const float wn = winding_at(c - n * eps, f);
      /* |w|>=2: this sheet sits in another part of the solid (overlap interior). */
      r_inside[f] = math::max(math::abs(wp), math::abs(wn)) >= k_overlap;
    }
  });
  /* Intersection faces are the crossing set, not the interior. */
  for (int f = 0; f < nfaces; f++) {
    if (r_inside[f]) {
      r_intersect[f] = false;
    }
  }
  return true;
}

bool cgal_mesh_is_outward_oriented(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return false;
  }
  return cgal_bridge::mesh_is_outward_oriented(in, r_error);
}

Mesh *cgal_mesh_reverse_orientation(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_reverse_orientation(in);
  return result_to_mesh(result, r_error);
}

int cgal_mesh_hole_count(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return 0;
  }
  return cgal_bridge::mesh_hole_count(in, r_error);
}

static void fill_edge_vert_buffers(const Mesh &mesh, Array<int> &v0, Array<int> &v1)
{
  const Span<int2> edges = mesh.edges();
  v0.reinitialize(edges.size());
  v1.reinitialize(edges.size());
  for (const int i : edges.index_range()) {
    v0[i] = edges[i][0];
    v1[i] = edges[i][1];
  }
}

bool cgal_mesh_border_edges(const Mesh &mesh, MutableSpan<bool> r_border, std::string &r_error)
{
  if (r_border.size() != mesh.edges_num) {
    r_error = "Edge count mismatch";
    return false;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    r_border.fill(false);
    return false;
  }
  Array<int> v0, v1;
  fill_edge_vert_buffers(mesh, v0, v1);
  std::vector<int8_t> flags;
  if (!cgal_bridge::mesh_border_edge_flags(
          in, v0.data(), v1.data(), mesh.edges_num, flags, r_error))
  {
    r_border.fill(false);
    return false;
  }
  for (const int i : r_border.index_range()) {
    r_border[i] = flags[size_t(i)] != 0;
  }
  return true;
}

bool cgal_mesh_dihedral_angles(const Mesh &mesh, MutableSpan<float> r_angle_rad, std::string &r_error)
{
  if (r_angle_rad.size() != mesh.edges_num) {
    r_error = "Edge count mismatch";
    return false;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    r_angle_rad.fill(0.0f);
    return false;
  }
  Array<int> v0, v1;
  fill_edge_vert_buffers(mesh, v0, v1);
  std::vector<float> angles;
  if (!cgal_bridge::mesh_dihedral_angles(
          in, v0.data(), v1.data(), mesh.edges_num, angles, r_error))
  {
    r_angle_rad.fill(0.0f);
    return false;
  }
  r_angle_rad.copy_from(Span(angles.data(), angles.size()));
  return true;
}

Mesh *cgal_mesh_duplicate_non_manifold(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_duplicate_non_manifold_vertices(in);
  return result_to_mesh(result, r_error);
}

/* ---------- Batch 18 GEO wrappers ---------- */

PointCloud *cgal_points_pca_estimate_normals(Span<float3> points,
                                             int neighbors,
                                             const PointCloud *src_attrs,
                                             std::string &r_error)
{
  const int n = int(points.size());
  std::vector<float> nrm(size_t(n) * 3);
  if (!cgal_bridge::points_pca_estimate_normals(
          points.cast<float>().data(), n, neighbors, nrm.data(), r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_attrs) {
    pointcloud_copy_point_attributes(*src_attrs, *pc, {"position", "normal"});
  }
  bke::SpanAttributeWriter<float3> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<float3>("normal",
                                                                           bke::AttrDomain::Point);
  for (int i = 0; i < n; i++) {
    w.span[i] = float3(nrm[size_t(i) * 3 + 0], nrm[size_t(i) * 3 + 1], nrm[size_t(i) * 3 + 2]);
  }
  w.finish();
  return pc;
}

PointCloud *cgal_points_mst_orient_normals(Span<float3> points,
                                           Span<float3> normals,
                                           int neighbors,
                                           const PointCloud *src_attrs,
                                           std::string &r_error)
{
  if (normals.size() != points.size() || points.is_empty()) {
    r_error = "Normals must match point count";
    return nullptr;
  }
  const int n = int(points.size());
  std::vector<float> nbuf(size_t(n) * 3), out(size_t(n) * 3);
  for (int i = 0; i < n; i++) {
    nbuf[size_t(i) * 3 + 0] = normals[i].x;
    nbuf[size_t(i) * 3 + 1] = normals[i].y;
    nbuf[size_t(i) * 3 + 2] = normals[i].z;
  }
  if (!cgal_bridge::points_mst_orient_normals(
          points.cast<float>().data(), nbuf.data(), n, neighbors, out.data(), r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_attrs) {
    pointcloud_copy_point_attributes(*src_attrs, *pc, {"position", "normal"});
  }
  bke::SpanAttributeWriter<float3> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<float3>("normal",
                                                                           bke::AttrDomain::Point);
  for (int i = 0; i < n; i++) {
    w.span[i] = float3(out[size_t(i) * 3 + 0], out[size_t(i) * 3 + 1], out[size_t(i) * 3 + 2]);
  }
  w.finish();
  return pc;
}

PointCloud *cgal_points_radial_orient_normals(Span<float3> points,
                                              Span<float3> normals,
                                              const PointCloud *src_attrs,
                                              std::string &r_error)
{
  if (normals.size() != points.size() || points.is_empty()) {
    r_error = "Normals must match point count";
    return nullptr;
  }
  const int n = int(points.size());
  std::vector<float> nbuf(size_t(n) * 3), out(size_t(n) * 3);
  for (int i = 0; i < n; i++) {
    nbuf[size_t(i) * 3 + 0] = normals[i].x;
    nbuf[size_t(i) * 3 + 1] = normals[i].y;
    nbuf[size_t(i) * 3 + 2] = normals[i].z;
  }
  if (!cgal_bridge::points_radial_orient_normals(
          points.cast<float>().data(), nbuf.data(), n, out.data(), r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_attrs) {
    pointcloud_copy_point_attributes(*src_attrs, *pc, {"position", "normal"});
  }
  bke::SpanAttributeWriter<float3> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<float3>("normal",
                                                                           bke::AttrDomain::Point);
  for (int i = 0; i < n; i++) {
    w.span[i] = float3(out[size_t(i) * 3 + 0], out[size_t(i) * 3 + 1], out[size_t(i) * 3 + 2]);
  }
  w.finish();
  return pc;
}

PointCloud *cgal_points_cluster(Span<float3> points,
                                float neighbor_radius,
                                const PointCloud *src_attrs,
                                int &r_cluster_count,
                                std::string &r_error)
{
  std::vector<int> clusters;
  if (!cgal_bridge::points_cluster(points.cast<float>().data(),
                                   int(points.size()),
                                   double(neighbor_radius),
                                   clusters,
                                   r_cluster_count,
                                   r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_attrs) {
    pointcloud_copy_point_attributes(*src_attrs, *pc, {"position", "Cluster"});
  }
  bke::SpanAttributeWriter<int> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<int>("Cluster",
                                                                        bke::AttrDomain::Point);
  w.span.copy_from(Span(clusters.data(), clusters.size()));
  w.finish();
  return pc;
}

PointCloud *cgal_mesh_polyhedral_envelope(const Mesh &mesh,
                                          float epsilon,
                                          Span<float3> query,
                                          const PointCloud *src_pointcloud,
                                          const Mesh *src_mesh,
                                          std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<int8_t> inside;
  if (!cgal_bridge::mesh_polyhedral_envelope_contains(
          in, double(epsilon), query.cast<float>().data(), int(query.size()), inside, r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(query);
  if (src_pointcloud) {
    pointcloud_copy_point_attributes(*src_pointcloud, *pc, {"position", "Inside"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(*src_mesh, *pc, {"position", "Inside"});
  }
  bke::SpanAttributeWriter<bool> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<bool>("Inside",
                                                                         bke::AttrDomain::Point);
  for (const int i : query.index_range()) {
    w.span[i] = inside[size_t(i)] != 0;
  }
  w.finish();
  return pc;
}

static Mesh *mesh_copy_write_point_float3(const Mesh &mesh,
                                          Span<float> nxyz,
                                          const char *attr_name,
                                          std::string &r_error)
{
  if (int(nxyz.size()) != mesh.verts_num * 3) {
    r_error = "Vertex attribute size mismatch";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<float3> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<float3>(attr_name,
                                                                            bke::AttrDomain::Point);
  for (int i = 0; i < mesh.verts_num; i++) {
    w.span[i] = float3(nxyz[size_t(i) * 3 + 0], nxyz[size_t(i) * 3 + 1], nxyz[size_t(i) * 3 + 2]);
  }
  w.finish();
  return out;
}

Mesh *cgal_mesh_vertex_normals(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<float> nxyz;
  if (!cgal_bridge::mesh_vertex_normals(in, nxyz, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return mesh_copy_write_point_float3(mesh, nxyz, "normal", r_error);
}

Mesh *cgal_mesh_face_normals(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<float> nxyz;
  if (!cgal_bridge::mesh_face_normals(in, nxyz, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  if (int(nxyz.size()) != mesh.faces_num * 3) {
    r_error = "Face normal size mismatch";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<float3> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<float3>("normal",
                                                                            bke::AttrDomain::Face);
  for (int i = 0; i < mesh.faces_num; i++) {
    w.span[i] = float3(nxyz[size_t(i) * 3 + 0], nxyz[size_t(i) * 3 + 1], nxyz[size_t(i) * 3 + 2]);
  }
  w.finish();
  return out;
}

Mesh *cgal_mesh_face_aspect_ratio(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<float> aspect;
  if (!cgal_bridge::mesh_face_aspect_ratio(in, aspect, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<float> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<float>("Aspect Ratio",
                                                                           bke::AttrDomain::Face);
  w.span.copy_from(Span(aspect.data(), aspect.size()));
  w.finish();
  return out;
}

Mesh *cgal_mesh_vertex_valence(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<int> val;
  if (!cgal_bridge::mesh_vertex_valence(in, val, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<int> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<int>("Valence",
                                                                         bke::AttrDomain::Point);
  w.span.copy_from(Span(val.data(), val.size()));
  w.finish();
  return out;
}

Mesh *cgal_mesh_mean_edge_length(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<float> len;
  if (!cgal_bridge::mesh_mean_edge_length(in, len, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<float> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<float>("Mean Edge Length",
                                                                           bke::AttrDomain::Point);
  w.span.copy_from(Span(len.data(), len.size()));
  w.finish();
  return out;
}

Mesh *cgal_mesh_border_vertex(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<int8_t> border;
  if (!cgal_bridge::mesh_border_vertices(in, border, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<bool> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<bool>("Border",
                                                                          bke::AttrDomain::Point);
  for (const int i : w.span.index_range()) {
    w.span[i] = border[size_t(i)] != 0;
  }
  w.finish();
  return out;
}

Mesh *cgal_mesh_face_quality(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<float> q;
  if (!cgal_bridge::mesh_face_quality(in, q, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<float> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<float>("Quality",
                                                                           bke::AttrDomain::Face);
  w.span.copy_from(Span(q.data(), q.size()));
  w.finish();
  return out;
}

Mesh *cgal_mesh_region_growing(const Mesh &mesh,
                               float max_distance,
                               float max_angle_deg,
                               int min_region_size,
                               int &r_region_count,
                               std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<int> region;
  if (!cgal_bridge::mesh_region_growing(in,
                                        double(max_distance),
                                        double(max_angle_deg),
                                        min_region_size,
                                        region,
                                        r_region_count,
                                        r_error))
  {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  if (int(region.size()) == out->faces_num) {
    bke::SpanAttributeWriter<int> w =
        out->attributes_for_write().lookup_or_add_for_write_only_span<int>("Region",
                                                                           bke::AttrDomain::Face);
    w.span.copy_from(Span(region.data(), region.size()));
    w.finish();
  }
  return out;
}

Mesh *cgal_mesh_surface_shortest_path(const Mesh &mesh,
                                      const float3 &source,
                                      const float3 &target,
                                      std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  const float s[3] = {source.x, source.y, source.z};
  const float t[3] = {target.x, target.y, target.z};
  return wire_result_to_mesh(cgal_bridge::mesh_surface_shortest_path(in, s, t), r_error);
}

PointCloud *cgal_mesh_locate(const Mesh &mesh,
                             Span<float3> query,
                             const PointCloud *src_pointcloud,
                             const Mesh *src_mesh,
                             std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<int> faces;
  std::vector<float> bary;
  if (!cgal_bridge::mesh_locate_points(
          in, query.cast<float>().data(), int(query.size()), faces, bary, r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(query);
  if (src_pointcloud) {
    pointcloud_copy_point_attributes(
        *src_pointcloud, *pc, {"position", "Face Index", "Barycentric"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(
        *src_mesh, *pc, {"position", "Face Index", "Barycentric"});
  }
  bke::MutableAttributeAccessor attrs = pc->attributes_for_write();
  {
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "Face Index", bke::AttrDomain::Point);
    w.span.copy_from(Span(faces.data(), faces.size()));
    w.finish();
  }
  {
    bke::SpanAttributeWriter<float3> w = attrs.lookup_or_add_for_write_only_span<float3>(
        "Barycentric", bke::AttrDomain::Point);
    for (const int i : query.index_range()) {
      w.span[i] = float3(
          bary[size_t(i) * 3 + 0], bary[size_t(i) * 3 + 1], bary[size_t(i) * 3 + 2]);
    }
    w.finish();
  }
  return pc;
}

Mesh *cgal_mesh_edge_length(const Mesh &mesh, std::string &r_error)
{
  if (mesh.edges_num <= 0 || mesh.verts_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  const Span<float3> pos = mesh.vert_positions();
  const Span<int2> edges = mesh.edges();
  std::vector<int> edge_verts(size_t(mesh.edges_num) * 2);
  for (const int e : edges.index_range()) {
    edge_verts[size_t(e) * 2 + 0] = edges[e][0];
    edge_verts[size_t(e) * 2 + 1] = edges[e][1];
  }
  std::vector<float> lens;
  if (!cgal_bridge::mesh_edge_lengths(pos.cast<float>().data(),
                                      mesh.verts_num,
                                      edge_verts.data(),
                                      mesh.edges_num,
                                      lens,
                                      r_error))
  {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<float> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<float>("Edge Length",
                                                                           bke::AttrDomain::Edge);
  w.span.copy_from(Span(lens.data(), lens.size()));
  w.finish();
  return out;
}

Mesh *cgal_mesh_face_perimeter(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<float> perim;
  if (!cgal_bridge::mesh_face_perimeters(in, perim, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<float> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<float>("Perimeter",
                                                                           bke::AttrDomain::Face);
  w.span.copy_from(Span(perim.data(), perim.size()));
  w.finish();
  return out;
}

bool cgal_points_centroid(Span<float3> points, float3 &r_centroid, std::string &r_error)
{
  float c[3];
  if (!cgal_bridge::points_centroid(points.cast<float>().data(), int(points.size()), c, r_error)) {
    return false;
  }
  r_centroid = float3(c[0], c[1], c[2]);
  return true;
}

Mesh *cgal_points_fit_plane(Span<float3> points,
                            float3 &r_center,
                            float3 &r_normal,
                            std::string &r_error)
{
  float center[3], normal[3];
  float half_extent = 1.0f;
  if (!cgal_bridge::points_fit_plane(points.cast<float>().data(),
                                     int(points.size()),
                                     center,
                                     normal,
                                     half_extent,
                                     r_error))
  {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  r_center = float3(center[0], center[1], center[2]);
  r_normal = float3(normal[0], normal[1], normal[2]);

  /* Build a unit-frame in the plane for a single quad. */
  float3 n = math::normalize(r_normal);
  float3 t1 = math::cross(n, float3(0, 0, 1));
  if (math::length_squared(t1) < 1e-10f) {
    t1 = math::cross(n, float3(0, 1, 0));
  }
  t1 = math::normalize(t1);
  const float3 t2 = math::normalize(math::cross(n, t1));
  const float h = half_extent;
  const float3 c = r_center;

  Mesh *mesh = BKE_mesh_new_nomain(4, 0, 1, 4);
  MutableSpan<float3> vpos = mesh->vert_positions_for_write();
  vpos[0] = c - t1 * h - t2 * h;
  vpos[1] = c + t1 * h - t2 * h;
  vpos[2] = c + t1 * h + t2 * h;
  vpos[3] = c - t1 * h + t2 * h;
  MutableSpan<int> offsets = mesh->face_offsets_for_write();
  offsets[0] = 0;
  offsets[1] = 4;
  MutableSpan<int> corners = mesh->corner_verts_for_write();
  corners[0] = 0;
  corners[1] = 1;
  corners[2] = 2;
  corners[3] = 3;
  bke::mesh_calc_edges(*mesh, false, false);
  mesh->tag_overlapping_none();
  shade_all_sharp(*mesh);
  return mesh;
}

bool cgal_points_neighbor_scale(Span<float3> points, int &r_scale_k, std::string &r_error)
{
  return cgal_bridge::points_neighbor_scale(
      points.cast<float>().data(), int(points.size()), r_scale_k, r_error);
}

PointCloud *cgal_points_scanline_orient_normals(Span<float3> points,
                                                Span<float3> normals,
                                                const PointCloud *src_pc,
                                                const Mesh *src_mesh,
                                                std::string &r_error)
{
  const int n = int(points.size());
  if (normals.size() != points.size()) {
    r_error = "Scanline Orient Normals needs normals matching point count";
    return nullptr;
  }
  std::vector<float> nbuf(size_t(n) * 3);
  for (int i = 0; i < n; i++) {
    nbuf[size_t(i) * 3 + 0] = normals[i].x;
    nbuf[size_t(i) * 3 + 1] = normals[i].y;
    nbuf[size_t(i) * 3 + 2] = normals[i].z;
  }
  std::vector<float> out_n(size_t(n) * 3);
  if (!cgal_bridge::points_scanline_orient_normals(
          points.cast<float>().data(), nbuf.data(), n, out_n.data(), r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_pc) {
    pointcloud_copy_point_attributes(*src_pc, *pc, {"position", "normal"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(*src_mesh, *pc, {"position", "normal"});
  }
  bke::SpanAttributeWriter<float3> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<float3>("normal",
                                                                           bke::AttrDomain::Point);
  for (int i = 0; i < n; i++) {
    w.span[i] = float3(
        out_n[size_t(i) * 3 + 0], out_n[size_t(i) * 3 + 1], out_n[size_t(i) * 3 + 2]);
  }
  w.finish();
  return pc;
}

PointCloud *cgal_points_local_neighbor_scales(Span<float3> points,
                                              const PointCloud *src_pc,
                                              const Mesh *src_mesh,
                                              std::string &r_error)
{
  std::vector<int> scales;
  if (!cgal_bridge::points_local_neighbor_scales(
          points.cast<float>().data(), int(points.size()), scales, r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_pc) {
    pointcloud_copy_point_attributes(*src_pc, *pc, {"position", "Local Scale K"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(*src_mesh, *pc, {"position", "Local Scale K"});
  }
  bke::SpanAttributeWriter<int> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<int>("Local Scale K",
                                                                        bke::AttrDomain::Point);
  w.span.copy_from(Span(scales.data(), scales.size()));
  w.finish();
  return pc;
}

Mesh *cgal_mesh_remove_small_components(const Mesh &mesh, int min_faces, std::string &r_error)
{
  return run_mesh_op_with_attr_maps(
      mesh,
      [min_faces](const cgal_bridge::MeshIn &in) {
        return cgal_bridge::mesh_remove_small_components(in, min_faces);
      },
      r_error,
      false,
      true);
}

Mesh *cgal_points_fit_line(Span<float3> points,
                           float3 &r_center,
                           float3 &r_direction,
                           std::string &r_error)
{
  float center[3], direction[3];
  float half_extent = 1.0f;
  if (!cgal_bridge::points_fit_line(points.cast<float>().data(),
                                    int(points.size()),
                                    center,
                                    direction,
                                    half_extent,
                                    r_error))
  {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  r_center = float3(center[0], center[1], center[2]);
  r_direction = float3(direction[0], direction[1], direction[2]);
  const float3 d = math::normalize(r_direction);
  const float3 a = r_center - d * half_extent;
  const float3 b = r_center + d * half_extent;
  Mesh *mesh = BKE_mesh_new_nomain(2, 1, 0, 0);
  MutableSpan<float3> vpos = mesh->vert_positions_for_write();
  vpos[0] = a;
  vpos[1] = b;
  MutableSpan<int2> edges = mesh->edges_for_write();
  edges[0] = int2(0, 1);
  mesh->tag_positions_changed();
  return mesh;
}

bool cgal_points_diameter(Span<float3> points, float &r_diameter, std::string &r_error)
{
  return cgal_bridge::points_diameter(
      points.cast<float>().data(), int(points.size()), r_diameter, r_error);
}

Mesh *cgal_mesh_orient_polygon_soup(const Mesh &mesh, std::string &r_error)
{
  return run_mesh_op_with_attr_maps(
      mesh,
      [](const cgal_bridge::MeshIn &in) { return cgal_bridge::mesh_orient_polygon_soup(in); },
      r_error,
      false,
      true);
}

bool cgal_mesh_self_intersection_count(const Mesh &mesh, int &r_count, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  return cgal_bridge::mesh_self_intersection_count(in, r_count, r_error);
}

PointCloud *cgal_points_local_density(Span<float3> points,
                                      int neighbors,
                                      const PointCloud *src_pc,
                                      const Mesh *src_mesh,
                                      std::string &r_error)
{
  std::vector<float> dens;
  if (!cgal_bridge::points_local_density(
          points.cast<float>().data(), int(points.size()), neighbors, dens, r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_pc) {
    pointcloud_copy_point_attributes(*src_pc, *pc, {"position", "Local Density"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(*src_mesh, *pc, {"position", "Local Density"});
  }
  bke::SpanAttributeWriter<float> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<float>("Local Density",
                                                                          bke::AttrDomain::Point);
  w.span.copy_from(Span(dens.data(), dens.size()));
  w.finish();
  return pc;
}

bool cgal_mesh_compactness(const Mesh &mesh, float &r_compactness, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  return cgal_bridge::mesh_compactness(in, r_compactness, r_error);
}

Mesh *cgal_mesh_component_size(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<int> sizes;
  if (!cgal_bridge::mesh_face_component_sizes(in, sizes, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<int> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<int>("Component Size",
                                                                         bke::AttrDomain::Face);
  w.span.copy_from(Span(sizes.data(), sizes.size()));
  w.finish();
  return out;
}

Mesh *cgal_mesh_face_planarity(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<float> plan;
  if (!cgal_bridge::mesh_face_planarity(in, plan, r_error)) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  Mesh *out = BKE_mesh_copy_for_eval(mesh);
  bke::SpanAttributeWriter<float> w =
      out->attributes_for_write().lookup_or_add_for_write_only_span<float>("Planarity",
                                                                           bke::AttrDomain::Face);
  w.span.copy_from(Span(plan.data(), plan.size()));
  w.finish();
  return out;
}

bool cgal_mesh_is_triangle_mesh(const Mesh &mesh, bool &r_is_triangle, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  return cgal_bridge::mesh_is_triangle_mesh(in, r_is_triangle, r_error);
}

Mesh *cgal_mesh_arap_deform(const Mesh &mesh,
                            Span<uint8_t> roi_mask,
                            Span<uint8_t> control_mask,
                            Span<float3> target_xyz,
                            int algorithm,
                            int iterations,
                            float tolerance,
                            std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  if (roi_mask.size() != mesh.verts_num || control_mask.size() != mesh.verts_num ||
      target_xyz.size() != mesh.verts_num)
  {
    r_error = "ARAP mask/target size mismatch";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_arap_deform(
      in,
      roi_mask.data(),
      control_mask.data(),
      target_xyz.cast<float>().data(),
      algorithm,
      iterations,
      double(tolerance));
  return result_to_mesh_or_positions(result, mesh, r_error);
}

bool cgal_mesh_parameterize_uv_corners(const Mesh &mesh,
                                       Span<bool> seam_per_edge,
                                       Span<bool> face_selection,
                                       bool normalize,
                                       int method,
                                       int energy_iterations,
                                       float lambda,
                                       MutableSpan<float2> r_corner_uv,
                                       std::string &r_error)
{
  if (r_corner_uv.size() != mesh.corners_num) {
    r_error = "Corner UV buffer size mismatch";
    return false;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return false;
  }

  const Span<int2> edges = mesh.edges();
  std::vector<int> edge_v0(size_t(edges.size()));
  std::vector<int> edge_v1(size_t(edges.size()));
  for (const int i : edges.index_range()) {
    edge_v0[size_t(i)] = edges[i][0];
    edge_v1[size_t(i)] = edges[i][1];
  }

  std::vector<uint8_t> seam_u8;
  const uint8_t *seam_ptr = nullptr;
  if (!seam_per_edge.is_empty()) {
    if (seam_per_edge.size() != edges.size()) {
      r_error = "Seam mask size must match edge count";
      return false;
    }
    seam_u8.resize(size_t(edges.size()));
    for (const int i : edges.index_range()) {
      seam_u8[size_t(i)] = seam_per_edge[i] ? 1 : 0;
    }
    seam_ptr = seam_u8.data();
  }

  std::vector<uint8_t> face_u8;
  const uint8_t *face_ptr = nullptr;
  if (!face_selection.is_empty()) {
    if (face_selection.size() != mesh.faces_num) {
      r_error = "Face selection size must match face count";
      return false;
    }
    face_u8.resize(size_t(mesh.faces_num));
    for (const int i : IndexRange(mesh.faces_num)) {
      face_u8[size_t(i)] = face_selection[i] ? 1 : 0;
    }
    face_ptr = face_u8.data();
  }

  std::vector<float> uv;
  if (!cgal_bridge::mesh_parameterize_uv_corners(in,
                                                 edge_v0.data(),
                                                 edge_v1.data(),
                                                 int(edges.size()),
                                                 seam_ptr,
                                                 face_ptr,
                                                 normalize,
                                                 method,
                                                 energy_iterations,
                                                 double(lambda),
                                                 uv,
                                                 r_error))
  {
    return false;
  }
  if (int(uv.size()) != mesh.corners_num * 2) {
    r_error = "UV corner count mismatch";
    return false;
  }
  for (const int c : IndexRange(mesh.corners_num)) {
    r_corner_uv[c] = float2(uv[size_t(c) * 2 + 0], uv[size_t(c) * 2 + 1]);
  }
  return true;
}

bool cgal_points_region_growing_planes(Span<float3> points,
                                       Span<float3> normals,
                                       float neighbor_radius,
                                       float max_distance,
                                       float max_angle_deg,
                                       int min_region_size,
                                       MutableSpan<int> r_region,
                                       int &r_region_count,
                                       std::string &r_error)
{
  r_region_count = 0;
  const int n = int(points.size());
  if (r_region.size() != points.size()) {
    r_error = "Region buffer size mismatch";
    return false;
  }
  if (n < 3) {
    r_error = "Need at least 3 points";
    return false;
  }
  const float *nxyz = nullptr;
  if (!normals.is_empty()) {
    if (normals.size() != points.size()) {
      r_error = "Normals count must match points";
      return false;
    }
    nxyz = normals.cast<float>().data();
  }
  std::vector<int> regions;
  if (!cgal_bridge::points_region_growing_planes(points.cast<float>().data(),
                                                 nxyz,
                                                 n,
                                                 double(neighbor_radius),
                                                 double(max_distance),
                                                 double(max_angle_deg),
                                                 min_region_size,
                                                 regions,
                                                 r_region_count,
                                                 r_error))
  {
    return false;
  }
  r_region.copy_from(Span(regions.data(), regions.size()));
  return true;
}

PointCloud *cgal_points_efficient_ransac(Span<float3> points,
                                         Span<float3> normals,
                                         float epsilon,
                                         float cluster_epsilon,
                                         float normal_threshold,
                                         int min_points,
                                         float probability,
                                         int shape_flags,
                                         int random_seed,
                                         const PointCloud *src_pc,
                                         const Mesh *src_mesh,
                                         int &r_shape_count,
                                         std::string &r_error)
{
  r_shape_count = 0;
  const int n = int(points.size());
  if (n < 10) {
    r_error = "Need at least 10 points";
    return nullptr;
  }
  const float *nxyz = nullptr;
  if (!normals.is_empty()) {
    if (normals.size() != points.size()) {
      r_error = "Normals count must match points";
      return nullptr;
    }
    nxyz = normals.cast<float>().data();
  }
  std::vector<int> shape_id;
  std::vector<int> shape_type;
  if (!cgal_bridge::points_efficient_ransac(points.cast<float>().data(),
                                            nxyz,
                                            n,
                                            double(epsilon),
                                            double(cluster_epsilon),
                                            double(normal_threshold),
                                            min_points,
                                            double(probability),
                                            shape_flags,
                                            unsigned(std::max(0, random_seed)),
                                            shape_id,
                                            shape_type,
                                            r_shape_count,
                                            r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_pc && src_pc->totpoint == n) {
    pointcloud_copy_point_attributes(*src_pc, *pc, {"position", "Shape", "Shape Type"});
  }
  else if (src_mesh && src_mesh->verts_num == n) {
    pointcloud_copy_from_mesh_point_attributes(
        *src_mesh, *pc, {"position", "Shape", "Shape Type"});
  }
  {
    bke::SpanAttributeWriter<int> w = pc->attributes_for_write().lookup_or_add_for_write_only_span<
        int>("Shape", bke::AttrDomain::Point);
    w.span.copy_from(Span(shape_id.data(), shape_id.size()));
    w.finish();
  }
  {
    bke::SpanAttributeWriter<int> w = pc->attributes_for_write().lookup_or_add_for_write_only_span<
        int>("Shape Type", bke::AttrDomain::Point);
    w.span.copy_from(Span(shape_type.data(), shape_type.size()));
    w.finish();
  }
  return pc;
}

/* --- Catalog batch 27 --- */

Mesh *cgal_points_min_ellipse_2(Span<float3> points,
                                int plane,
                                int segments,
                                float3 &r_center,
                                float &r_radius,
                                std::string &r_error)
{
  r_center = float3(0.0f);
  r_radius = 0.0f;
  if (points.size() < 3) {
    r_error = "Min Ellipse 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_min_ellipse_2(
      points.cast<float>().data(), int(points.size()), plane, segments);
  if (result.ok) {
    r_center = float3(result.center[0], result.center[1], result.center[2]);
    r_radius = result.radius;
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_min_parallelogram_2(Span<float3> points,
                                      int plane,
                                      float3 &r_center,
                                      float &r_radius,
                                      std::string &r_error)
{
  r_center = float3(0.0f);
  r_radius = 0.0f;
  if (points.size() < 2) {
    r_error = "Min Parallelogram 2D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_min_parallelogram_2(
      points.cast<float>().data(), int(points.size()), plane);
  if (result.ok) {
    r_center = float3(result.center[0], result.center[1], result.center[2]);
    r_radius = result.radius;
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_convex_hull_2(Span<float3> points, int plane, std::string &r_error)
{
  if (points.size() < 3) {
    r_error = "Convex Hull 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_convex_hull_2(
      points.cast<float>().data(), int(points.size()), plane);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

bool cgal_mesh_plane_slice(const Mesh &mesh,
                           float3 origin,
                           float3 normal,
                           Vector<Vector<float3>> &r_polylines,
                           std::string &r_error)
{
  r_polylines.clear();
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return false;
  }
  std::vector<std::vector<float>> packed;
  if (!cgal_bridge::mesh_plane_slice(in,
                                     origin.x,
                                     origin.y,
                                     origin.z,
                                     normal.x,
                                     normal.y,
                                     normal.z,
                                     packed,
                                     r_error))
  {
    return false;
  }
  r_polylines.reserve(int(packed.size()));
  for (const auto &pl : packed) {
    if (pl.size() < 6) {
      continue;
    }
    Vector<float3> curve;
    curve.reserve(int(pl.size() / 3));
    for (size_t i = 0; i + 2 < pl.size(); i += 3) {
      curve.append(float3(pl[i], pl[i + 1], pl[i + 2]));
    }
    if (curve.size() >= 2) {
      r_polylines.append(std::move(curve));
    }
  }
  return true;
}

PointCloud *cgal_points_vcm_feature_edges(Span<float3> points,
                                          float offset_radius,
                                          float convolution_radius,
                                          float threshold,
                                          bool only_features,
                                          const PointCloud *src_pc,
                                          const Mesh *src_mesh,
                                          int &r_feature_count,
                                          std::string &r_error)
{
  r_feature_count = 0;
  const int n = int(points.size());
  if (n < 3) {
    r_error = "VCM Feature Edges needs at least 3 points";
    return nullptr;
  }
  std::vector<int8_t> feature;
  std::vector<float> ratio;
  if (!cgal_bridge::points_vcm_feature_edges(points.cast<float>().data(),
                                             n,
                                             double(offset_radius),
                                             double(convolution_radius),
                                             double(threshold),
                                             feature,
                                             ratio,
                                             r_feature_count,
                                             r_error))
  {
    return nullptr;
  }

  if (only_features && r_feature_count > 0) {
    Array<float3> kept(r_feature_count);
    Array<float> kept_ratio(r_feature_count);
    int w = 0;
    for (int i = 0; i < n; i++) {
      if (feature[size_t(i)]) {
        kept[w] = points[i];
        kept_ratio[w] = ratio[size_t(i)];
        w++;
      }
    }
    PointCloud *pc = pointcloud_from_positions_copy(kept.as_span());
    {
      bke::SpanAttributeWriter<float> a =
          pc->attributes_for_write().lookup_or_add_for_write_only_span<float>(
              "Feature", bke::AttrDomain::Point);
      a.span.fill(1.0f);
      a.finish();
    }
    {
      bke::SpanAttributeWriter<float> a =
          pc->attributes_for_write().lookup_or_add_for_write_only_span<float>(
              "Feature Ratio", bke::AttrDomain::Point);
      a.span.copy_from(kept_ratio.as_span());
      a.finish();
    }
    {
      bke::SpanAttributeWriter<float> a =
          pc->attributes_for_write().lookup_or_add_for_write_only_span<float>(
              ".selection", bke::AttrDomain::Point);
      a.span.fill(1.0f);
      a.finish();
    }
    return pc;
  }

  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_pc && src_pc->totpoint == n) {
    pointcloud_copy_point_attributes(
        *src_pc, *pc, {"position", "Feature", "Feature Ratio", ".selection"});
  }
  else if (src_mesh && src_mesh->verts_num == n) {
    pointcloud_copy_from_mesh_point_attributes(
        *src_mesh, *pc, {"position", "Feature", "Feature Ratio", ".selection"});
  }
  {
    bke::SpanAttributeWriter<float> a =
        pc->attributes_for_write().lookup_or_add_for_write_only_span<float>(
            "Feature", bke::AttrDomain::Point);
    for (int i = 0; i < n; i++) {
      a.span[i] = feature[size_t(i)] ? 1.0f : 0.0f;
    }
    a.finish();
  }
  {
    bke::SpanAttributeWriter<float> a =
        pc->attributes_for_write().lookup_or_add_for_write_only_span<float>(
            "Feature Ratio", bke::AttrDomain::Point);
    a.span.copy_from(Span(ratio.data(), ratio.size()));
    a.finish();
  }
  {
    bke::SpanAttributeWriter<float> a =
        pc->attributes_for_write().lookup_or_add_for_write_only_span<float>(
            ".selection", bke::AttrDomain::Point);
    for (int i = 0; i < n; i++) {
      a.span[i] = feature[size_t(i)] ? 1.0f : 0.0f;
    }
    a.finish();
  }
  return pc;
}

Mesh *cgal_points_delaunay_2(Span<float3> points, int plane, std::string &r_error)
{
  if (points.size() < 3) {
    r_error = "Delaunay 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_delaunay_2(
      points.cast<float>().data(), int(points.size()), plane);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_alpha_shape_2(Span<float3> points,
                                int plane,
                                float alpha,
                                bool use_optimal_alpha,
                                float &r_alpha_used,
                                std::string &r_error)
{
  r_alpha_used = 0.0f;
  if (points.size() < 3) {
    r_error = "Alpha Shape 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_alpha_shape_2(
      points.cast<float>().data(), int(points.size()), plane, double(alpha), use_optimal_alpha);
  if (result.ok) {
    r_alpha_used = float(result.alpha_used);
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_voronoi_2(Span<float3> points,
                            int plane,
                            float clip_margin,
                            std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "Voronoi 2D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_voronoi_2(
      points.cast<float>().data(), int(points.size()), plane, clip_margin);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "cell_id", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  else if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_straight_skeleton_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 3) {
    r_error = "Straight Skeleton 2D needs a mesh with vertices";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(cgal_bridge::mesh_straight_skeleton_2(in), r_error);
}

Mesh *cgal_mesh_polygon_offset_2(const Mesh &mesh,
                                 float offset_distance,
                                 int mode,
                                 std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  /* Open edge chains need only 2 verts; closed polygons need 3+. */
  if (in.verts_num < 2) {
    r_error = "Polygon Offset 2D needs a mesh with vertices";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_polygon_offset_2(
      in, double(offset_distance), mode);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_medial_axis_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 3) {
    r_error = "Medial Axis 2D needs a mesh with vertices";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(cgal_bridge::mesh_medial_axis_2(in), r_error);
}

Mesh *cgal_points_min_annulus_2(Span<float3> points,
                                int plane,
                                int segments,
                                float3 &r_center,
                                float &r_outer_radius,
                                float &r_inner_radius,
                                std::string &r_error)
{
  r_center = float3(0.0f);
  r_outer_radius = 0.0f;
  r_inner_radius = 0.0f;
  if (points.size() < 3) {
    r_error = "Min Annulus 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_min_annulus_2(
      points.cast<float>().data(), int(points.size()), plane, segments);
  if (result.ok) {
    r_center = float3(result.center[0], result.center[1], result.center[2]);
    r_outer_radius = result.radius;
    r_inner_radius = float(result.alpha_used);
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_convex_partition_2(const Mesh &mesh, int method, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 3) {
    r_error = "Convex Partition 2D needs a mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_convex_partition_2(in, method);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    if (int(result.face_tag.size()) == out->faces_num) {
      bke::MutableAttributeAccessor attrs = out->attributes_for_write();
      bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
          "piece", bke::AttrDomain::Face);
      w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
      w.finish();
    }
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_y_monotone_partition_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 3) {
    r_error = "Y-Monotone Partition 2D needs a mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_y_monotone_partition_2(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_max_area_k_gon_2(Span<float3> points, int plane, int k, std::string &r_error)
{
  if (points.size() < 3) {
    r_error = "Max Area K-gon 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_max_area_k_gon_2(
      points.cast<float>().data(), int(points.size()), plane, k);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_polyline_simplify_2(const Mesh &mesh, float cost_threshold, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 3) {
    r_error = "Shape Simplify 2D needs a mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_polyline_simplify_2(
      in, double(std::max(cost_threshold, 1.0e-8f)));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_width_3(Span<float3> points,
                          float plane_scale,
                          float &r_width,
                          float3 &r_center,
                          std::string &r_error)
{
  r_width = 0.0f;
  r_center = float3(0.0f);
  if (points.size() < 4) {
    r_error = "Width 3D needs at least 4 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_width_3(
      points.cast<float>().data(), int(points.size()), plane_scale);
  if (result.ok) {
    r_width = result.radius;
    r_center = float3(result.center[0], result.center[1], result.center[2]);
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_minkowski_sum_2(const Mesh &mesh_a, const Mesh &mesh_b, std::string &r_error)
{
  cgal_bridge::MeshIn a = mesh_to_cgal_in(mesh_a);
  cgal_bridge::MeshIn b = mesh_to_cgal_in(mesh_b);
  if (a.verts_num < 3 || b.verts_num < 3) {
    r_error = "Minkowski Sum 2D needs two meshes";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_minkowski_sum_2(a, b);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_extrude_skeleton(const Mesh &mesh,
                                 float height,
                                 bool outward,
                                 std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 3) {
    r_error = "Extrude Skeleton needs a mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_extrude_skeleton(
      in, double(height), outward);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_polygon_fill_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 3) {
    r_error = "Polygon Fill 2D needs a mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_polygon_fill_2(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

bool cgal_mesh_do_intersect_2(const Mesh &mesh_a, const Mesh &mesh_b, std::string &r_error)
{
  cgal_bridge::MeshIn a = mesh_to_cgal_in(mesh_a);
  cgal_bridge::MeshIn b = mesh_to_cgal_in(mesh_b);
  if (a.verts_num < 3 || b.verts_num < 3) {
    r_error = "Do Intersect 2D needs two meshes";
    return false;
  }
  return cgal_bridge::mesh_do_intersect_2(a, b, r_error);
}

Mesh *cgal_mesh_boolean_ops_2(const Mesh &mesh_a,
                              const Mesh *mesh_b,
                              int mode,
                              std::string &r_error)
{
  cgal_bridge::MeshIn a = mesh_to_cgal_in(mesh_a);
  cgal_bridge::MeshIn b;
  if (mesh_b && mesh_b->verts_num >= 3) {
    b = mesh_to_cgal_in(*mesh_b);
  }
  if (a.verts_num < 3) {
    r_error = "Boolean Ops 2D needs mesh A";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_boolean_ops_2(a, b, mode);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        "is_hole", bke::AttrDomain::Face);
    for (int i = 0; i < out->faces_num; i++) {
      w.span[i] = result.face_tag[size_t(i)] != 0;
    }
    w.finish();
    shade_all_sharp(*out);
  }
  else if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_largest_empty_iso_rectangle_2(Span<float3> points,
                                                int plane,
                                                float3 &r_center,
                                                float &r_area_sqrt,
                                                std::string &r_error)
{
  r_center = float3(0.0f);
  r_area_sqrt = 0.0f;
  if (points.size() < 1) {
    r_error = "Largest Empty Iso Rectangle needs points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_largest_empty_iso_rectangle_2(
      points.cast<float>().data(), int(points.size()), plane);
  if (result.ok) {
    r_center = float3(result.center[0], result.center[1], result.center[2]);
    r_area_sqrt = result.radius;
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_polygon_repair_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 3) {
    r_error = "Polygon Repair 2D needs a mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_polygon_repair_2(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

PointCloud *cgal_points_monge_jet_fit_pc(Span<float3> points,
                                         int knn,
                                         int degree_fitting,
                                         int degree_monge,
                                         const PointCloud *src_pc,
                                         const Mesh *src_mesh,
                                         std::string &r_error)
{
  const int n = int(points.size());
  std::vector<float> k1, k2, d1, d2, nn;
  if (!cgal_bridge::points_monge_jet_fit(points.cast<float>().data(),
                                         n,
                                         knn,
                                         degree_fitting,
                                         degree_monge,
                                         k1,
                                         k2,
                                         d1,
                                         d2,
                                         nn,
                                         r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_pc) {
    pointcloud_copy_point_attributes(
        *src_pc, *pc, {"position", "monge_k1", "monge_k2", "monge_d1", "monge_d2", "monge_normal"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(
        *src_mesh, *pc, {"position", "monge_k1", "monge_k2", "monge_d1", "monge_d2", "monge_normal"});
  }
  bke::MutableAttributeAccessor attrs = pc->attributes_for_write();
  bke::SpanAttributeWriter<float> w_k1 = attrs.lookup_or_add_for_write_only_span<float>(
      "monge_k1", bke::AttrDomain::Point);
  bke::SpanAttributeWriter<float> w_k2 = attrs.lookup_or_add_for_write_only_span<float>(
      "monge_k2", bke::AttrDomain::Point);
  bke::SpanAttributeWriter<float3> w_d1 = attrs.lookup_or_add_for_write_only_span<float3>(
      "monge_d1", bke::AttrDomain::Point);
  bke::SpanAttributeWriter<float3> w_d2 = attrs.lookup_or_add_for_write_only_span<float3>(
      "monge_d2", bke::AttrDomain::Point);
  bke::SpanAttributeWriter<float3> w_n = attrs.lookup_or_add_for_write_only_span<float3>(
      "monge_normal", bke::AttrDomain::Point);
  for (int i = 0; i < n; i++) {
    w_k1.span[i] = k1[size_t(i)];
    w_k2.span[i] = k2[size_t(i)];
    w_d1.span[i] = float3(d1[size_t(i) * 3 + 0], d1[size_t(i) * 3 + 1], d1[size_t(i) * 3 + 2]);
    w_d2.span[i] = float3(d2[size_t(i) * 3 + 0], d2[size_t(i) * 3 + 1], d2[size_t(i) * 3 + 2]);
    w_n.span[i] = float3(nn[size_t(i) * 3 + 0], nn[size_t(i) * 3 + 1], nn[size_t(i) * 3 + 2]);
  }
  w_k1.finish();
  w_k2.finish();
  w_d1.finish();
  w_d2.finish();
  w_n.finish();
  return pc;
}


Mesh *cgal_mesh_convex_decomposition_3(const Mesh &mesh, int &r_piece_count, std::string &r_error)
{
  r_piece_count = 0;
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_convex_decomposition_3(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (!out || out->faces_num == 0) {
    return out;
  }
  if (int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "convex_piece", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    int max_id = -1;
    for (const int t : result.face_tag) {
      max_id = std::max(max_id, t);
    }
    r_piece_count = max_id + 1;
  }
  else if (result.alpha_used > 0.0) {
    r_piece_count = int(result.alpha_used + 0.5);
  }
  shade_all_sharp(*out);
  return out;
}

Mesh *cgal_mesh_laplace_deform(const Mesh &mesh,
                               Span<uint8_t> roi_mask,
                               Span<uint8_t> control_mask,
                               Span<float3> target_xyz,
                               float weight,
                               std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  if (roi_mask.size() != mesh.verts_num || control_mask.size() != mesh.verts_num ||
      target_xyz.size() != mesh.verts_num)
  {
    r_error = "Laplace mask/target size mismatch";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_laplace_deform(
      in,
      roi_mask.data(),
      control_mask.data(),
      target_xyz.cast<float>().data(),
      double(weight));
  return result_to_mesh_or_positions(result, mesh, r_error);
}

Mesh *cgal_mesh_surface_delaunay_remesh(const Mesh &mesh,
                                        float facet_size,
                                        float facet_angle,
                                        float facet_distance,
                                        float features_angle_bound,
                                        bool protect_constraints,
                                        std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_surface_delaunay_remesh(
      in,
      double(facet_size),
      double(facet_angle),
      double(facet_distance),
      double(features_angle_bound),
      protect_constraints);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_otr_reconstruct_2(Span<float3> points,
                                    int plane,
                                    float keep_percent,
                                    int relocation,
                                    std::string &r_error)
{
  return wire_result_to_mesh(
      cgal_bridge::points_otr_reconstruct_2(
          points.cast<float>().data(), int(points.size()), plane, double(keep_percent), relocation),
      r_error);
}

Mesh *cgal_points_regular_triangulation_2(Span<float3> points,
                                          Span<float> radii,
                                          int plane,
                                          std::string &r_error)
{
  const float *rptr = (radii.size() == points.size()) ? radii.data() : nullptr;
  cgal_bridge::MeshResult result = cgal_bridge::points_regular_triangulation_2(
      points.cast<float>().data(), rptr, int(points.size()), plane);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "cell_id", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  else if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_power_diagram_2(Span<float3> points,
                                  Span<float> radii,
                                  int plane,
                                  float clip_margin,
                                  std::string &r_error)
{
  const float *rptr = (radii.size() == points.size()) ? radii.data() : nullptr;
  cgal_bridge::MeshResult result = cgal_bridge::points_power_diagram_2(
      points.cast<float>().data(), rptr, int(points.size()), plane, clip_margin);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "cell_id", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  else if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_voronoi_3(Span<float3> points, float clip_margin, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::points_voronoi_3(
      points.cast<float>().data(), int(points.size()), clip_margin);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "cell_id", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  else if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_visibility_2(const Mesh &mesh, const float3 &query, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_visibility_2(
      in, query.x, query.y, query.z);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_refine_2(const Mesh &mesh,
                         float max_edge,
                         float shape_bound,
                         int lloyd_iters,
                         std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 3) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_refine_2(
      in, double(max_edge), double(shape_bound), lloyd_iters);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_alpha_complex_3(Span<float3> points,
                                  float alpha,
                                  bool use_optimal_alpha,
                                  bool separate_tets,
                                  float &r_alpha_used,
                                  int &r_tet_count,
                                  std::string &r_error)
{
  r_alpha_used = 0.0f;
  r_tet_count = 0;
  cgal_bridge::MeshResult result = cgal_bridge::points_alpha_complex_3(
      points.cast<float>().data(),
      int(points.size()),
      double(alpha),
      use_optimal_alpha,
      separate_tets);
  Mesh *out = result_to_mesh(result, r_error);
  r_alpha_used = float(result.alpha_used);
  r_tet_count = int(result.radius + 0.5);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "alpha_tet", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_minkowski_sum_3_convex(const Mesh &mesh_a,
                                       const Mesh &mesh_b,
                                       std::string &r_error)
{
  cgal_bridge::MeshIn a = mesh_to_cgal_in(mesh_a);
  cgal_bridge::MeshIn b = mesh_to_cgal_in(mesh_b);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_minkowski_sum_3_convex(a, b);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_regular_triangulation_3(Span<float3> points,
                                          Span<float> radii,
                                          bool separate_tets,
                                          int &r_tet_count,
                                          std::string &r_error)
{
  r_tet_count = 0;
  const float *rptr = (radii.size() == points.size()) ? radii.data() : nullptr;
  cgal_bridge::MeshResult result = cgal_bridge::points_regular_triangulation_3(
      points.cast<float>().data(), rptr, int(points.size()), separate_tets);
  Mesh *out = result_to_mesh(result, r_error);
  r_tet_count = int(result.radius + 0.5f);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "regular_tet", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_power_diagram_3(Span<float3> points,
                                  Span<float> radii,
                                  float clip_margin,
                                  std::string &r_error)
{
  const float *rptr = (radii.size() == points.size()) ? radii.data() : nullptr;
  cgal_bridge::MeshResult result = cgal_bridge::points_power_diagram_3(
      points.cast<float>().data(), rptr, int(points.size()), clip_margin);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "cell_id", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  else if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_largest_empty_circle_2(Span<float3> points,
                                         int plane,
                                         int segments,
                                         float3 &r_center,
                                         float &r_radius,
                                         std::string &r_error)
{
  r_center = float3(0.0f);
  r_radius = 0.0f;
  cgal_bridge::MeshResult result = cgal_bridge::points_largest_empty_circle_2(
      points.cast<float>().data(), int(points.size()), plane, segments);
  if (result.ok) {
    r_center = float3(result.center[0], result.center[1], result.center[2]);
    r_radius = result.radius;
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_largest_inscribed_circle_2(const Mesh &mesh,
                                           int segments,
                                           float3 &r_center,
                                           float &r_radius,
                                           std::string &r_error)
{
  r_center = float3(0.0f);
  r_radius = 0.0f;
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_largest_inscribed_circle_2(in, segments);
  if (result.ok) {
    r_center = float3(result.center[0], result.center[1], result.center[2]);
    r_radius = result.radius;
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_min_width_2(Span<float3> points,
                              int plane,
                              float plane_scale,
                              float &r_width,
                              float3 &r_center,
                              std::string &r_error)
{
  r_width = 0.0f;
  r_center = float3(0.0f);
  cgal_bridge::MeshResult result = cgal_bridge::points_min_width_2(
      points.cast<float>().data(), int(points.size()), plane, plane_scale);
  if (result.ok) {
    r_width = result.radius;
    r_center = float3(result.center[0], result.center[1], result.center[2]);
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_periodic_delaunay_2(Span<float3> points,
                                      int plane,
                                      float domain_x,
                                      float domain_y,
                                      std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::points_periodic_delaunay_2(
      points.cast<float>().data(), int(points.size()), plane, double(domain_x), double(domain_y));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_constrained_voronoi_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  return wire_result_to_mesh(cgal_bridge::mesh_constrained_voronoi_2(in), r_error);
}

Mesh *cgal_mesh_arrangement_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_arrangement_2(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "cell_id", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  else if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_delaunay_on_sphere(Span<float3> points, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::points_delaunay_on_sphere(
      points.cast<float>().data(), int(points.size()));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_periodic_delaunay_3(Span<float3> points,
                                      float domain_x,
                                      float domain_y,
                                      float domain_z,
                                      bool separate_tets,
                                      int &r_tet_count,
                                      std::string &r_error)
{
  r_tet_count = 0;
  cgal_bridge::MeshResult result = cgal_bridge::points_periodic_delaunay_3(
      points.cast<float>().data(),
      int(points.size()),
      double(domain_x),
      double(domain_y),
      double(domain_z),
      separate_tets);
  Mesh *out = result_to_mesh(result, r_error);
  r_tet_count = int(result.radius + 0.5f);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "periodic_tet", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  else if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

bool cgal_mesh_do_intersect_3(const Mesh &mesh_a, const Mesh &mesh_b, std::string &r_error)
{
  cgal_bridge::MeshIn a = mesh_to_cgal_in(mesh_a);
  cgal_bridge::MeshIn b = mesh_to_cgal_in(mesh_b);
  return cgal_bridge::mesh_do_intersect_3(a, b, r_error);
}

Mesh *cgal_mesh_split_by_mesh(const Mesh &mesh,
                              const Mesh &cutter,
                              int &r_piece_count,
                              std::string &r_error)
{
  r_piece_count = 0;
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshIn cut = mesh_to_cgal_in(cutter);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_split_by_mesh(in, cut);
  Mesh *out = result_to_mesh(result, r_error);
  r_piece_count = int(result.radius + 0.5f);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "piece_id", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  else if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

CgalNaturalNeighbor2::~CgalNaturalNeighbor2()
{
  cgal_bridge::natural_neighbor_2_free(static_cast<cgal_bridge::NaturalNeighbor2 *>(impl_));
  impl_ = nullptr;
}

CgalNaturalNeighbor2::CgalNaturalNeighbor2(CgalNaturalNeighbor2 &&other) noexcept : impl_(other.impl_)
{
  other.impl_ = nullptr;
}

CgalNaturalNeighbor2 &CgalNaturalNeighbor2::operator=(CgalNaturalNeighbor2 &&other) noexcept
{
  if (this != &other) {
    cgal_bridge::natural_neighbor_2_free(static_cast<cgal_bridge::NaturalNeighbor2 *>(impl_));
    impl_ = other.impl_;
    other.impl_ = nullptr;
  }
  return *this;
}

bool CgalNaturalNeighbor2::build(Span<float3> sites, std::string &r_error)
{
  cgal_bridge::natural_neighbor_2_free(static_cast<cgal_bridge::NaturalNeighbor2 *>(impl_));
  impl_ = nullptr;
  impl_ = cgal_bridge::natural_neighbor_2_new(
      sites.cast<float>().data(), int(sites.size()), r_error);
  return impl_ != nullptr;
}

bool CgalNaturalNeighbor2::is_valid() const
{
  return impl_ != nullptr;
}

int CgalNaturalNeighbor2::site_count() const
{
  return cgal_bridge::natural_neighbor_2_site_count(
      static_cast<const cgal_bridge::NaturalNeighbor2 *>(impl_));
}

void CgalNaturalNeighbor2::query_many(Span<float3> query,
                                      std::vector<int> &r_offsets,
                                      std::vector<int> &r_site_index,
                                      std::vector<float> &r_weight,
                                      MutableSpan<bool> r_valid) const
{
  cgal_bridge::natural_neighbor_2_query_many(
      static_cast<const cgal_bridge::NaturalNeighbor2 *>(impl_),
      query.cast<float>().data(),
      int(query.size()),
      r_offsets,
      r_site_index,
      r_weight,
      r_valid.is_empty() ? nullptr : r_valid.data());
}

CgalNaturalNeighbor3::~CgalNaturalNeighbor3()
{
  cgal_bridge::natural_neighbor_3_free(static_cast<cgal_bridge::NaturalNeighbor3 *>(impl_));
  impl_ = nullptr;
}

CgalNaturalNeighbor3::CgalNaturalNeighbor3(CgalNaturalNeighbor3 &&other) noexcept : impl_(other.impl_)
{
  other.impl_ = nullptr;
}

CgalNaturalNeighbor3 &CgalNaturalNeighbor3::operator=(CgalNaturalNeighbor3 &&other) noexcept
{
  if (this != &other) {
    cgal_bridge::natural_neighbor_3_free(static_cast<cgal_bridge::NaturalNeighbor3 *>(impl_));
    impl_ = other.impl_;
    other.impl_ = nullptr;
  }
  return *this;
}

bool CgalNaturalNeighbor3::build(Span<float3> sites, std::string &r_error)
{
  cgal_bridge::natural_neighbor_3_free(static_cast<cgal_bridge::NaturalNeighbor3 *>(impl_));
  impl_ = nullptr;
  impl_ = cgal_bridge::natural_neighbor_3_new(
      sites.cast<float>().data(), int(sites.size()), r_error);
  return impl_ != nullptr;
}

bool CgalNaturalNeighbor3::is_valid() const
{
  return impl_ != nullptr;
}

int CgalNaturalNeighbor3::site_count() const
{
  return cgal_bridge::natural_neighbor_3_site_count(
      static_cast<const cgal_bridge::NaturalNeighbor3 *>(impl_));
}

void CgalNaturalNeighbor3::query_many(Span<float3> query,
                                      std::vector<int> &r_offsets,
                                      std::vector<int> &r_site_index,
                                      std::vector<float> &r_weight,
                                      MutableSpan<bool> r_valid) const
{
  cgal_bridge::natural_neighbor_3_query_many(
      static_cast<const cgal_bridge::NaturalNeighbor3 *>(impl_),
      query.cast<float>().data(),
      int(query.size()),
      r_offsets,
      r_site_index,
      r_weight,
      r_valid.is_empty() ? nullptr : r_valid.data());
}

Mesh *cgal_points_proximity_graph_2(Span<float3> points, int mode, std::string &r_error)
{
  return wire_result_to_mesh(
      cgal_bridge::points_proximity_graph_2(points.cast<float>().data(), int(points.size()), mode),
      r_error);
}

Mesh *cgal_mesh_fill_polyline(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_fill_polyline(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    if (result.has_vert_map() || result.has_face_map()) {
      interpolate_attributes_from_maps(*out, mesh, result);
    }
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_halfspace_intersection_3(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_halfspace_intersection_3(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

bool cgal_points_lloyd_relax(const Mesh &surface,
                             Span<float3> sites,
                             int max_iters,
                             float convergence,
                             MutableSpan<float3> r_positions,
                             std::string &r_error)
{
  if (sites.size() != r_positions.size() || sites.is_empty()) {
    r_error = "Lloyd Relax: empty or mismatched point cloud";
    return false;
  }
  cgal_bridge::MeshIn in = mesh_to_cgal_in(surface);
  std::vector<float> out;
  if (!cgal_bridge::points_lloyd_relax(in,
                                       sites.cast<float>().data(),
                                       int(sites.size()),
                                       max_iters,
                                       double(convergence),
                                       out,
                                       r_error))
  {
    return false;
  }
  if (int(out.size()) != int(sites.size()) * 3) {
    r_error = "Lloyd Relax produced a different point count";
    return false;
  }
  for (const int i : sites.index_range()) {
    r_positions[i] = float3(out[size_t(i) * 3 + 0], out[size_t(i) * 3 + 1], out[size_t(i) * 3 + 2]);
  }
  return true;
}

Mesh *cgal_mesh_conforming_delaunay_2(const Mesh &mesh, int mode, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_conforming_delaunay_2(in, mode);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_max_perimeter_k_gon_2(Span<float3> points, int k, std::string &r_error)
{
  if (points.size() < 3) {
    r_error = "Max Perimeter K-gon 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_max_perimeter_k_gon_2(
      points.cast<float>().data(), int(points.size()), k);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_constrained_delaunay_2(const Mesh &mesh, int fill_rule, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_constrained_delaunay_2(in, fill_rule);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_exterior_skeleton_2(const Mesh &mesh, float max_offset, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  return wire_result_to_mesh(cgal_bridge::mesh_exterior_skeleton_2(in, double(max_offset)),
                             r_error);
}

Mesh *cgal_points_voronoi_on_sphere(Span<float3> points, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::points_voronoi_on_sphere(
      points.cast<float>().data(), int(points.size()));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "cell_id", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
    shade_all_sharp(*out);
  }
  else if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_radius_graph_3(Span<float3> points, float radius, std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "Radius Graph 3D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(
      cgal_bridge::points_radius_graph_3(
          points.cast<float>().data(), int(points.size()), double(radius)),
      r_error);
}

Mesh *cgal_points_knn_graph_3(Span<float3> points, int k, std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "KNN Graph 3D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(
      cgal_bridge::points_knn_graph_3(points.cast<float>().data(), int(points.size()), k),
      r_error);
}

bool cgal_points_natural_neighbor_3(Span<float3> sites,
                                    Span<float> site_values,
                                    Span<float3> queries,
                                    MutableSpan<float> r_values,
                                    std::string &r_error)
{
  if (sites.size() < 4 || site_values.size() != sites.size() ||
      r_values.size() != queries.size())
  {
    r_error = "Natural Neighbor 3D size mismatch";
    return false;
  }
  std::vector<float> out;
  const bool ok = cgal_bridge::points_natural_neighbor_3(
      sites.cast<float>().data(),
      site_values.data(),
      int(sites.size()),
      queries.cast<float>().data(),
      int(queries.size()),
      out,
      r_error);
  if (!ok || int(out.size()) != r_values.size()) {
    return false;
  }
  r_values.copy_from(Span(out.data(), int(out.size())));
  return true;
}

bool cgal_points_sibson_gradient_2(Span<float3> points,
                                   Span<float> values,
                                   MutableSpan<float3> r_gradients,
                                   std::string &r_error)
{
  if (points.size() < 3 || values.size() != points.size() ||
      r_gradients.size() != points.size())
  {
    r_error = "Sibson Gradient 2D size mismatch";
    return false;
  }
  std::vector<float> out;
  const bool ok = cgal_bridge::points_sibson_gradient_2(
      points.cast<float>().data(), values.data(), int(points.size()), out, r_error);
  if (!ok || int(out.size()) != int(points.size()) * 3) {
    return false;
  }
  for (int i = 0; i < int(points.size()); i++) {
    r_gradients[i] = float3(out[size_t(i) * 3 + 0], out[size_t(i) * 3 + 1], out[size_t(i) * 3 + 2]);
  }
  return true;
}

Mesh *cgal_points_octree_3(
    Span<float3> points, int max_depth, int bucket, bool solid, std::string &r_error)
{
  if (points.is_empty()) {
    r_error = "Octree 3D needs points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_octree_3(
      points.cast<float>().data(), int(points.size()), max_depth, bucket, solid);
  if (!solid) {
    cgal_bridge::WireResult wire;
    wire.positions = std::move(result.positions);
    wire.edge_v0 = std::move(result.seam_vert_a);
    wire.edge_v1 = std::move(result.seam_vert_b);
    wire.ok = result.ok;
    wire.error = result.error;
    return wire_result_to_mesh(wire, r_error);
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_hilbert_path_3(Span<float3> points, std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "Hilbert Path 3D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(
      cgal_bridge::points_hilbert_path_3(points.cast<float>().data(), int(points.size())),
      r_error);
}

Mesh *cgal_mesh_shortest_cycle(const Mesh &mesh, std::string &r_error)
{
  return wire_result_to_mesh(cgal_bridge::mesh_shortest_cycle(mesh_to_cgal_in(mesh)), r_error);
}

Mesh *cgal_points_periodic_voronoi_2(Span<float3> points,
                                     float domain_x,
                                     float domain_y,
                                     std::string &r_error)
{
  if (points.size() < 3) {
    r_error = "Periodic Voronoi 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_periodic_voronoi_2(
      points.cast<float>().data(), int(points.size()), double(domain_x), double(domain_y));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    if (int(result.face_tag.size()) == out->faces_num) {
      bke::SpanAttributeWriter<int> w = out->attributes_for_write().lookup_or_add_for_write_only_span<int>(
          "cell_id", bke::AttrDomain::Face);
      w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
      w.finish();
    }
  }
  return out;
}

Mesh *cgal_points_periodic_voronoi_3(Span<float3> points,
                                     float domain_x,
                                     float domain_y,
                                     float domain_z,
                                     std::string &r_error)
{
  if (points.size() < 4) {
    r_error = "Periodic Voronoi 3D needs at least 4 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_periodic_voronoi_3(
      points.cast<float>().data(),
      int(points.size()),
      double(domain_x),
      double(domain_y),
      double(domain_z));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    if (int(result.face_tag.size()) == out->faces_num) {
      bke::SpanAttributeWriter<int> w = out->attributes_for_write().lookup_or_add_for_write_only_span<int>(
          "cell_id", bke::AttrDomain::Face);
      w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
      w.finish();
    }
  }
  return out;
}

Mesh *cgal_points_gabriel_graph_3(Span<float3> points, std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "Gabriel Graph 3D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(
      cgal_bridge::points_gabriel_graph_3(points.cast<float>().data(), int(points.size())),
      r_error);
}

Mesh *cgal_points_euclidean_mst_3(Span<float3> points, std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "Euclidean MST 3D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(
      cgal_bridge::points_euclidean_mst_3(points.cast<float>().data(), int(points.size())),
      r_error);
}

Mesh *cgal_mesh_euclidean_mst_3(const Mesh &mesh, std::string &r_error)
{
  if (mesh.verts_num < 2) {
    r_error = "Euclidean MST 3D needs at least 2 vertices";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(cgal_bridge::mesh_euclidean_mst_3(mesh_to_cgal_in(mesh)), r_error);
}

Mesh *cgal_points_beta_skeleton_2(Span<float3> points, float beta, std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "Beta Skeleton 2D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(
      cgal_bridge::points_beta_skeleton_2(
          points.cast<float>().data(), int(points.size()), double(beta)),
      r_error);
}

Mesh *cgal_points_convex_layers_2(Span<float3> points, std::string &r_error)
{
  if (points.size() < 3) {
    r_error = "Convex Layers 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_convex_layers_2(
      points.cast<float>().data(), int(points.size()));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    if (int(result.face_tag.size()) == out->faces_num) {
      bke::SpanAttributeWriter<int> w = out->attributes_for_write().lookup_or_add_for_write_only_span<int>(
          "layer_id", bke::AttrDomain::Face);
      w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
      w.finish();
    }
  }
  return out;
}

static void write_face_tag(Mesh &mesh, const cgal_bridge::MeshResult &result, StringRef name)
{
  if (int(result.face_tag.size()) == mesh.faces_num) {
    bke::SpanAttributeWriter<int> w =
        mesh.attributes_for_write().lookup_or_add_for_write_only_span<int>(name,
                                                                          bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
  }
}

Mesh *cgal_points_farthest_voronoi_2(Span<float3> points, float clip_margin, std::string &r_error)
{
  if (points.size() < 3) {
    r_error = "Farthest Voronoi 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_farthest_voronoi_2(
      points.cast<float>().data(), int(points.size()), double(clip_margin));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "cell_id");
  }
  return out;
}

Mesh *cgal_points_convex_layers_3(Span<float3> points, std::string &r_error)
{
  if (points.size() < 4) {
    r_error = "Convex Layers 3D needs at least 4 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_convex_layers_3(
      points.cast<float>().data(), int(points.size()));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "layer_id");
  }
  return out;
}

Mesh *cgal_mesh_overlay_2(const Mesh &mesh_a, const Mesh &mesh_b, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_overlay_2(mesh_to_cgal_in(mesh_a),
                                                               mesh_to_cgal_in(mesh_b));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "overlay_id");
  }
  return out;
}

Mesh *cgal_mesh_polygon_kernel_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_polygon_kernel_2(mesh_to_cgal_in(mesh));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "island_id");
  }
  return out;
}

bool cgal_mesh_self_intersection_curves(const Mesh &mesh,
                                        Vector<Vector<float3>> &r_polylines,
                                        std::string &r_error)
{
  r_polylines.clear();
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Self Intersection Curves needs faces";
    return false;
  }
  std::vector<std::vector<float>> raw;
  if (!cgal_bridge::mesh_self_intersection_polylines(in, raw, r_error)) {
    return false;
  }
  r_polylines.reserve(raw.size());
  for (const std::vector<float> &poly : raw) {
    if (poly.size() < 6) {
      continue;
    }
    Vector<float3> pts;
    pts.reserve(poly.size() / 3);
    for (std::size_t i = 0; i + 2 < poly.size(); i += 3) {
      pts.append(float3(poly[i], poly[i + 1], poly[i + 2]));
    }
    if (pts.size() >= 2) {
      r_polylines.append(std::move(pts));
    }
  }
  return true;
}

Mesh *cgal_points_crust_2(Span<float3> points, std::string &r_error)
{
  if (points.size() < 3) {
    r_error = "Crust 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(
      cgal_bridge::points_crust_2(points.cast<float>().data(), int(points.size())), r_error);
}

Mesh *cgal_mesh_geodesic_voronoi(const Mesh &mesh, Span<bool> sources, std::string &r_error)
{
  if (mesh.verts_num < 3 || mesh.faces_num < 1) {
    r_error = "Geodesic Voronoi needs a mesh with faces";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  std::vector<uint8_t> mask(size_t(mesh.verts_num), 0);
  const int n = std::min(mesh.verts_num, int(sources.size()));
  for (int i = 0; i < n; i++) {
    mask[size_t(i)] = sources[i] ? 1 : 0;
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_geodesic_voronoi(mesh_to_cgal_in(mesh),
                                                                      mask.data());
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "cell_id");
  }
  return out;
}

Mesh *cgal_points_isosurface_3(Span<float3> points,
                               Span<float> values,
                               float isolevel,
                               std::string &r_error)
{
  if (points.size() < 4 || values.size() < points.size()) {
    r_error = "Isosurface 3D needs at least 4 points with values";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_isosurface_3(
      points.cast<float>().data(), values.data(), int(points.size()), double(isolevel));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    bke::mesh_smooth_set(*out, true);
  }
  return out;
}

Mesh *cgal_mesh_line_arrangement_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_line_arrangement_2(mesh_to_cgal_in(mesh));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "island_id");
  }
  return out;
}

Mesh *cgal_mesh_vertical_decomposition_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_vertical_decomposition_2(
      mesh_to_cgal_in(mesh));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "island_id");
  }
  return out;
}

Mesh *cgal_points_circle_arrangement_2(Span<float3> points,
                                       Span<float> radii,
                                       int segments,
                                       std::string &r_error)
{
  if (points.size() < 1) {
    r_error = "Circle Arrangement 2D needs at least 1 point";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  const float *rad = (radii.size() >= points.size()) ? radii.data() : nullptr;
  cgal_bridge::MeshResult result = cgal_bridge::points_circle_arrangement_2(
      points.cast<float>().data(), rad, int(points.size()), segments);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "overlap");
  }
  return out;
}

Mesh *cgal_points_crust_3(Span<float3> points, std::string &r_error)
{
  if (points.size() < 4) {
    r_error = "Crust 3D needs at least 4 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_crust_3(points.cast<float>().data(),
                                                              int(points.size()));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_clipped_voronoi_3(Span<float3> points, float clip_margin, std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "Clipped Voronoi 3D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_clipped_voronoi_3(
      points.cast<float>().data(), int(points.size()), clip_margin);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "cell_id");
  }
  return out;
}

Mesh *cgal_mesh_complement_2(const Mesh &mesh, float padding, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_complement_2(mesh_to_cgal_in(mesh), padding);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "island_id");
  }
  return out;
}

Mesh *cgal_points_simple_polygon_2(Span<float3> points, std::string &r_error)
{
  if (points.size() < 3) {
    r_error = "Simple Polygon 2D needs at least 3 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_simple_polygon_2(
      points.cast<float>().data(), int(points.size()));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

bool cgal_points_regularize_planes(Span<float3> points,
                                   Span<int> plane_id,
                                   bool parallelism,
                                   bool orthogonality,
                                   bool coplanarity,
                                   bool axis_symmetry,
                                   float angle_deg,
                                   float coplanar_tol,
                                   MutableSpan<float3> r_positions,
                                   int &r_plane_count,
                                   std::string &r_error)
{
  r_plane_count = 0;
  if (points.size() != plane_id.size() || points.size() != r_positions.size()) {
    r_error = "Regularize Planes: size mismatch";
    return false;
  }
  std::vector<float> out;
  if (!cgal_bridge::points_regularize_planes(points.cast<float>().data(),
                                             plane_id.data(),
                                             int(points.size()),
                                             parallelism,
                                             orthogonality,
                                             coplanarity,
                                             axis_symmetry,
                                             double(angle_deg),
                                             double(coplanar_tol),
                                             out,
                                             r_plane_count,
                                             r_error))
  {
    return false;
  }
  if (int(out.size()) != points.size() * 3) {
    r_error = "Regularize Planes: unexpected output size";
    return false;
  }
  for (const int i : points.index_range()) {
    r_positions[i] = float3(out[size_t(i) * 3 + 0], out[size_t(i) * 3 + 1], out[size_t(i) * 3 + 2]);
  }
  return true;
}

Mesh *cgal_mesh_split_crossings_2(const Mesh &mesh, std::string &r_error)
{
  return wire_result_to_mesh(cgal_bridge::mesh_split_crossings_2(mesh_to_cgal_in(mesh)), r_error);
}

PointCloud *cgal_mesh_crossing_points_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_crossing_points_2(mesh_to_cgal_in(mesh));
  if (!result.ok || result.verts_num() <= 0) {
    r_error = result.error.empty() ? "Crossing Points 2D failed" : result.error;
    return BKE_pointcloud_new_nomain(PointCloudType::Points, 0);
  }
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, result.verts_num());
  MutableSpan<float3> pos = pc->positions_for_write();
  for (int i = 0; i < result.verts_num(); i++) {
    pos[i] = float3(result.positions[size_t(i) * 3 + 0],
                    result.positions[size_t(i) * 3 + 1],
                    result.positions[size_t(i) * 3 + 2]);
  }
  return pc;
}

Mesh *cgal_mesh_pullout_directions_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_pullout_directions_2(mesh_to_cgal_in(mesh));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "island_id");
  }
  return out;
}

Mesh *cgal_mesh_ssab_partition_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_ssab_partition_2(mesh_to_cgal_in(mesh));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "island_id");
  }
  return out;
}

Mesh *cgal_mesh_snap_borders(const Mesh &mesh, float tolerance, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_snap_borders(mesh_to_cgal_in(mesh),
                                                                  double(tolerance));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    BKE_mesh_copy_parameters_for_eval(out, &mesh);
    bke::mesh_remesh_reproject_attributes(mesh, *out);
  }
  return out;
}

Mesh *cgal_mesh_autorefine_clean(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_autorefine_clean(mesh_to_cgal_in(mesh));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    BKE_mesh_copy_parameters_for_eval(out, &mesh);
    bke::mesh_remesh_reproject_attributes(mesh, *out);
  }
  return out;
}

Mesh *cgal_points_random_polygon_2(int count, float size, int seed, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::points_random_polygon_2(
      count, double(size), seed);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_random_convex_set_2(int count, float size, int seed, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::points_random_convex_set_2(
      count, double(size), seed);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_largest_empty_sphere_3(Span<float3> points,
                                         int segments,
                                         float3 &r_center,
                                         float &r_radius,
                                         std::string &r_error)
{
  r_center = float3(0.0f);
  r_radius = 0.0f;
  cgal_bridge::MeshResult result = cgal_bridge::points_largest_empty_sphere_3(
      points.cast<float>().data(), int(points.size()), segments);
  r_center = float3(result.center[0], result.center[1], result.center[2]);
  r_radius = result.radius;
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    bke::mesh_smooth_set(*out, true);
  }
  return out;
}

Mesh *cgal_points_ransac_primitives(Span<float3> points,
                                    Span<float3> normals,
                                    float epsilon,
                                    float cluster_epsilon,
                                    float normal_threshold,
                                    int min_points,
                                    float probability,
                                    int shape_flags,
                                    int random_seed,
                                    int segments,
                                    int &r_shape_count,
                                    std::string &r_error)
{
  r_shape_count = 0;
  const float *nptr = (normals.size() == points.size() && !normals.is_empty()) ?
                          normals.cast<float>().data() :
                          nullptr;
  cgal_bridge::MeshResult result = cgal_bridge::points_ransac_primitives(
      points.cast<float>().data(),
      nptr,
      int(points.size()),
      double(epsilon),
      double(cluster_epsilon),
      double(normal_threshold),
      min_points,
      double(probability),
      shape_flags,
      unsigned(std::max(0, random_seed)),
      segments);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "shape_id");
    int max_id = -1;
    for (const int t : result.face_tag) {
      max_id = std::max(max_id, t);
    }
    r_shape_count = max_id + 1;
  }
  return out;
}

Mesh *cgal_points_voronoi_slice_3(Span<float3> points,
                                  const float3 &origin,
                                  const float3 &normal,
                                  float clip_margin,
                                  std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::points_voronoi_slice_3(
      points.cast<float>().data(),
      int(points.size()),
      origin.x,
      origin.y,
      origin.z,
      normal.x,
      normal.y,
      normal.z,
      clip_margin);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "cell_id");
  }
  return out;
}

Mesh *cgal_mesh_convex_offset_3(const Mesh &mesh, float offset, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_convex_offset_3(mesh_to_cgal_in(mesh),
                                                                    double(offset));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_volume_components(const Mesh &mesh, int &r_volume_count, std::string &r_error)
{
  r_volume_count = 0;
  cgal_bridge::MeshResult result = cgal_bridge::mesh_volume_components(mesh_to_cgal_in(mesh));
  r_volume_count = int(result.alpha_used);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "volume_id");
  }
  return out;
}

Mesh *cgal_mesh_inscribed_sphere_3(const Mesh &mesh,
                                   int segments,
                                   float3 &r_center,
                                   float &r_radius,
                                   std::string &r_error)
{
  r_center = float3(0.0f);
  r_radius = 0.0f;
  cgal_bridge::MeshResult result = cgal_bridge::mesh_inscribed_sphere_3(mesh_to_cgal_in(mesh),
                                                                       segments);
  r_center = float3(result.center[0], result.center[1], result.center[2]);
  r_radius = result.radius;
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    bke::mesh_smooth_set(*out, true);
  }
  return out;
}

Mesh *cgal_points_min_cylinder_3(Span<float3> points,
                                 int segments,
                                 float3 &r_center,
                                 float &r_radius,
                                 std::string &r_error)
{
  r_center = float3(0.0f);
  r_radius = 0.0f;
  cgal_bridge::MeshResult result = cgal_bridge::points_min_cylinder_3(
      points.cast<float>().data(), int(points.size()), segments);
  r_center = float3(result.center[0], result.center[1], result.center[2]);
  r_radius = result.radius;
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

/**
 * Dirichlet verts are copies of original mesh vertices (single parent).
 * Edge-lerp / hole Steiner verts are free and get a cotangent harmonic fill.
 */
static bool is_original_dirichlet_vert(const cgal_bridge::MeshResult &maps, const int v)
{
  if (!maps.has_vert_map() || v < 0 || v >= int(maps.vert_src0.size())) {
    return false;
  }
  if (maps.vert_src0[v] < 0) {
    return false;
  }
  if (v < int(maps.vert_src1.size()) && maps.vert_src1[v] >= 0) {
    return false;
  }
  return true;
}

template<typename T> static constexpr int harmonic_float_channels()
{
  if constexpr (std::is_same_v<T, float>) {
    return 1;
  }
  else if constexpr (std::is_same_v<T, float2>) {
    return 2;
  }
  else if constexpr (std::is_same_v<T, float3>) {
    return 3;
  }
  else if constexpr (std::is_same_v<T, float4> || std::is_same_v<T, ColorGeometry4f> ||
                     std::is_same_v<T, ColorGeometry4b>)
  {
    return 4;
  }
  else {
    return 0;
  }
}

template<typename T> static void harmonic_load_channels(const T &v, double out[4])
{
  if constexpr (std::is_same_v<T, float>) {
    out[0] = double(v);
  }
  else if constexpr (std::is_same_v<T, ColorGeometry4b>) {
    out[0] = double(v.r) / 255.0;
    out[1] = double(v.g) / 255.0;
    out[2] = double(v.b) / 255.0;
    out[3] = double(v.a) / 255.0;
  }
  else {
    constexpr int n = harmonic_float_channels<T>();
    for (int c = 0; c < n; c++) {
      out[c] = double(v[c]);
    }
  }
}

template<typename T> static T harmonic_store_channels(const double in[4])
{
  if constexpr (std::is_same_v<T, float>) {
    return float(in[0]);
  }
  else if constexpr (std::is_same_v<T, ColorGeometry4b>) {
    auto quant = [](const double x) -> uint8_t {
      return uint8_t(std::clamp(int(std::lround(x * 255.0)), 0, 255));
    };
    return T(quant(in[0]), quant(in[1]), quant(in[2]), quant(in[3]));
  }
  else {
    T out{};
    constexpr int n = harmonic_float_channels<T>();
    for (int c = 0; c < n; c++) {
      out[c] = float(in[c]);
    }
    return out;
  }
}

static void add_cot_triangle(const Span<float3> pos,
                             const int i0,
                             const int i1,
                             const int i2,
                             const Span<char> is_free,
                             LinearSolver *solver,
                             Array<Vector<std::pair<int, float>>> &gs_nbr,
                             MutableSpan<float> gs_diag)
{
  const int n = int(pos.size());
  if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= n || i1 >= n || i2 >= n) {
    return;
  }
  if (i0 == i1 || i1 == i2 || i2 == i0) {
    return;
  }
  if (!is_free[i0] && !is_free[i1] && !is_free[i2]) {
    return;
  }
  /* Pinkall–Polthier: edge opposite vertex k gets ½ cot(angle at k). */
  const float w0 = 0.5f * math::cotangent_tri_weight(pos[i0], pos[i1], pos[i2]);
  const float w1 = 0.5f * math::cotangent_tri_weight(pos[i1], pos[i2], pos[i0]);
  const float w2 = 0.5f * math::cotangent_tri_weight(pos[i2], pos[i0], pos[i1]);

  auto add_edge = [&](const int a, const int b, const float w) {
    if (w == 0.0f) {
      return;
    }
    EIG_linear_solver_matrix_add(solver, a, a, double(w));
    EIG_linear_solver_matrix_add(solver, b, b, double(w));
    EIG_linear_solver_matrix_add(solver, a, b, double(-w));
    EIG_linear_solver_matrix_add(solver, b, a, double(-w));
    const float wp = math::max(w, 0.0f);
    if (wp > 0.0f) {
      gs_nbr[a].append({b, wp});
      gs_nbr[b].append({a, wp});
      gs_diag[a] += wp;
      gs_diag[b] += wp;
    }
  };
  add_edge(i1, i2, w0);
  add_edge(i2, i0, w1);
  add_edge(i0, i1, w2);
}

static void cotan_harmonic_fill_point_attrs(Mesh &dst, const cgal_bridge::MeshResult &maps)
{
  if (!maps.has_vert_map() || dst.verts_num <= 0 || dst.faces_num <= 0) {
    return;
  }
  const int n = dst.verts_num;
  Array<char> is_free(n, 0);
  int free_n = 0;
  for (int v = 0; v < n; v++) {
    if (!is_original_dirichlet_vert(maps, v)) {
      is_free[v] = 1;
      free_n++;
    }
  }
  if (free_n == 0) {
    return;
  }

  LinearSolver *solver = EIG_linear_solver_new(0, n, 4);
  if (!solver) {
    return;
  }
  for (int v = 0; v < n; v++) {
    if (!is_free[v]) {
      EIG_linear_solver_variable_lock(solver, v);
    }
  }

  const Span<float3> pos = dst.vert_positions();
  const OffsetIndices faces = dst.faces();
  const Span<int> corner_verts = dst.corner_verts();
  Array<Vector<std::pair<int, float>>> gs_nbr(n);
  Array<float> gs_diag(n, 0.0f);

  for (const int f : faces.index_range()) {
    const IndexRange face = faces[f];
    if (face.size() < 3) {
      continue;
    }
    const int v0 = corner_verts[face[0]];
    for (int i = 1; i + 1 < face.size(); i++) {
      add_cot_triangle(pos,
                       v0,
                       corner_verts[face[i]],
                       corner_verts[face[i + 1]],
                       is_free,
                       solver,
                       gs_nbr,
                       gs_diag);
    }
  }
  constexpr double diag_eps = 1.0e-10;
  for (int v = 0; v < n; v++) {
    if (is_free[v]) {
      EIG_linear_solver_matrix_add(solver, v, v, diag_eps);
    }
  }

  bke::MutableAttributeAccessor attrs = dst.attributes_for_write();
  attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != bke::AttrDomain::Point || is_builtin_topology_attr(iter.name) ||
        iter.name == "position")
    {
      return;
    }
    bke::GSpanAttributeWriter w = attrs.lookup_for_write_span(iter.name);
    if (!w) {
      return;
    }
    bke::attribute_math::to_static_type(w.span.type(), [&]<typename T>() {
      constexpr int channels = harmonic_float_channels<T>();
      if constexpr (channels > 0) {
      MutableSpan<T> data = w.span.typed<T>();
      for (int v = 0; v < n; v++) {
        if (is_free[v]) {
          continue;
        }
        double ch[4] = {0, 0, 0, 0};
        harmonic_load_channels(data[v], ch);
        for (int c = 0; c < channels; c++) {
          EIG_linear_solver_variable_set(solver, c, v, ch[c]);
        }
      }
      const bool solved = EIG_linear_solver_solve(solver);
      if (solved) {
        for (int v = 0; v < n; v++) {
          if (!is_free[v]) {
            continue;
          }
          double ch[4] = {0, 0, 0, 0};
          for (int c = 0; c < channels; c++) {
            ch[c] = EIG_linear_solver_variable_get(solver, c, v);
          }
          data[v] = harmonic_store_channels<T>(ch);
        }
      }
      else {
      /* SparseLU failed (obtuse / degenerate patch): positive-cot Gauss–Seidel. */
      Array<double> cur(n * 4, 0.0);
      Array<double> nxt(n * 4, 0.0);
      for (int v = 0; v < n; v++) {
        double ch[4] = {0, 0, 0, 0};
        harmonic_load_channels(data[v], ch);
        for (int c = 0; c < 4; c++) {
          cur[v * 4 + c] = ch[c];
        }
      }
      nxt = cur;
      for (int it = 0; it < 256; it++) {
        bool changed = false;
        for (int v = 0; v < n; v++) {
          if (!is_free[v] || gs_diag[v] < 1.0e-20f) {
            continue;
          }
          double acc[4] = {0, 0, 0, 0};
          for (const std::pair<int, float> &nb : gs_nbr[v]) {
            for (int c = 0; c < channels; c++) {
              acc[c] += double(nb.second) * cur[nb.first * 4 + c];
            }
          }
          const double inv = 1.0 / double(gs_diag[v]);
          for (int c = 0; c < channels; c++) {
            const double val = acc[c] * inv;
            nxt[v * 4 + c] = val;
            if (val != cur[v * 4 + c]) {
              changed = true;
            }
          }
        }
        cur = nxt;
        if (!changed) {
          break;
        }
      }
      for (int v = 0; v < n; v++) {
        if (!is_free[v]) {
          continue;
        }
        data[v] = harmonic_store_channels<T>(cur.data() + v * 4);
      }
      }
      }
    });
    w.finish();
  });

  EIG_linear_solver_delete(solver);
}

Mesh *cgal_mesh_fair_hole_fill(const Mesh &mesh, int continuity, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.faces_num <= 0) {
    r_error = "Empty mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_fair_hole_fill(in, continuity);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    interpolate_attributes_from_maps(*out, mesh, result);
    cotan_harmonic_fill_point_attrs(*out, result);
  }
  return out;
}

Mesh *cgal_mesh_simplify_polyline_3(const Mesh &mesh,
                                    float max_distance,
                                    bool iterative,
                                    std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 2) {
    r_error = "Simplify Polyline 3D needs at least 2 vertices";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(
      cgal_bridge::mesh_simplify_polyline_3(in, double(max_distance), iterative), r_error);
}

static const float *span_xyz_or_null(Span<float3> s)
{
  return s.is_empty() ? nullptr : s.cast<float>().data();
}

static std::vector<int> *region_buf(MutableSpan<int> out_region, std::vector<int> &storage, int n)
{
  if (out_region.size() != n) {
    return nullptr;
  }
  storage.assign(size_t(n), -1);
  return &storage;
}

static void copy_region_buf(const std::vector<int> &storage, MutableSpan<int> out_region)
{
  if (out_region.size() == storage.size()) {
    out_region.copy_from(Span(storage.data(), storage.size()));
  }
}

Mesh *cgal_points_sphere_region_growing(Span<float3> points,
                                        Span<float3> normals,
                                        float neighbor_radius,
                                        float max_distance,
                                        float max_angle_deg,
                                        int min_region_size,
                                        float min_radius,
                                        float max_radius,
                                        int segments,
                                        MutableSpan<int> out_region,
                                        int &r_count,
                                        std::string &r_error)
{
  r_count = 0;
  std::vector<int> storage;
  std::vector<int> *region = region_buf(out_region, storage, int(points.size()));
  cgal_bridge::MeshResult result = cgal_bridge::points_sphere_region_growing(
      points.cast<float>().data(),
      span_xyz_or_null(normals),
      int(points.size()),
      double(neighbor_radius),
      double(max_distance),
      double(max_angle_deg),
      min_region_size,
      double(min_radius),
      double(max_radius),
      segments,
      region);
  r_count = int(result.alpha_used);
  copy_region_buf(storage, out_region);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "region_id");
  }
  return out;
}

Mesh *cgal_points_cylinder_region_growing(Span<float3> points,
                                          Span<float3> normals,
                                          float neighbor_radius,
                                          float max_distance,
                                          float max_angle_deg,
                                          int min_region_size,
                                          float min_radius,
                                          float max_radius,
                                          int segments,
                                          MutableSpan<int> out_region,
                                          int &r_count,
                                          std::string &r_error)
{
  r_count = 0;
  std::vector<int> storage;
  std::vector<int> *region = region_buf(out_region, storage, int(points.size()));
  cgal_bridge::MeshResult result = cgal_bridge::points_cylinder_region_growing(
      points.cast<float>().data(),
      span_xyz_or_null(normals),
      int(points.size()),
      double(neighbor_radius),
      double(max_distance),
      double(max_angle_deg),
      min_region_size,
      double(min_radius),
      double(max_radius),
      segments,
      region);
  r_count = int(result.alpha_used);
  copy_region_buf(storage, out_region);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "region_id");
  }
  return out;
}

Mesh *cgal_points_circle_region_growing(Span<float3> points,
                                        Span<float3> normals,
                                        float neighbor_radius,
                                        float max_distance,
                                        float max_angle_deg,
                                        int min_region_size,
                                        float min_radius,
                                        float max_radius,
                                        int segments,
                                        MutableSpan<int> out_region,
                                        int &r_count,
                                        std::string &r_error)
{
  r_count = 0;
  std::vector<int> storage;
  std::vector<int> *region = region_buf(out_region, storage, int(points.size()));
  cgal_bridge::MeshResult result = cgal_bridge::points_circle_region_growing(
      points.cast<float>().data(),
      span_xyz_or_null(normals),
      int(points.size()),
      double(neighbor_radius),
      double(max_distance),
      double(max_angle_deg),
      min_region_size,
      double(min_radius),
      double(max_radius),
      segments,
      region);
  r_count = int(result.alpha_used);
  copy_region_buf(storage, out_region);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "region_id");
  }
  return out;
}

Mesh *cgal_points_line_region_growing(Span<float3> points,
                                      Span<float3> normals,
                                      float neighbor_radius,
                                      float max_distance,
                                      int min_region_size,
                                      MutableSpan<int> out_region,
                                      int &r_count,
                                      std::string &r_error)
{
  r_count = 0;
  std::vector<int> storage;
  std::vector<int> *region = region_buf(out_region, storage, int(points.size()));
  cgal_bridge::WireResult wire = cgal_bridge::points_line_region_growing(
      points.cast<float>().data(),
      span_xyz_or_null(normals),
      int(points.size()),
      double(neighbor_radius),
      double(max_distance),
      min_region_size,
      region);
  r_count = 0;
  if (region) {
    int mx = -1;
    for (int id : storage) {
      mx = std::max(mx, id);
    }
    r_count = mx + 1;
  }
  copy_region_buf(storage, out_region);
  return wire_result_to_mesh(wire, r_error);
}

bool cgal_simplify_polyline_xyz(Span<float3> points,
                                bool closed,
                                float max_distance,
                                bool iterative,
                                Vector<float3> &r_out,
                                Vector<int> &r_src,
                                std::string &r_error)
{
  r_out.clear();
  r_src.clear();
  std::vector<float> out_xyz;
  std::vector<int> src;
  if (!cgal_bridge::simplify_polyline_xyz(points.cast<float>().data(),
                                          int(points.size()),
                                          closed,
                                          double(max_distance),
                                          iterative,
                                          out_xyz,
                                          src,
                                          r_error))
  {
    return false;
  }
  const int n = int(out_xyz.size() / 3);
  r_out.reserve(n);
  r_src.reserve(n);
  for (int i = 0; i < n; i++) {
    r_out.append(float3(out_xyz[size_t(i) * 3 + 0],
                        out_xyz[size_t(i) * 3 + 1],
                        out_xyz[size_t(i) * 3 + 2]));
    r_src.append(src[size_t(i)]);
  }
  return r_out.size() >= 2;
}

Mesh *cgal_points_shape_fitting(Span<float3> points,
                                Span<float3> normals,
                                int mode,
                                float neighbor_radius,
                                float max_distance,
                                float max_angle_deg,
                                int min_region_size,
                                float min_radius,
                                float max_radius,
                                int segments,
                                MutableSpan<int> out_region,
                                int &r_count,
                                std::string &r_error)
{
  r_count = 0;
  std::vector<int> storage;
  std::vector<int> *region = region_buf(out_region, storage, int(points.size()));
  cgal_bridge::MeshResult result = cgal_bridge::points_shape_fitting(
      points.cast<float>().data(),
      span_xyz_or_null(normals),
      int(points.size()),
      mode,
      double(neighbor_radius),
      double(max_distance),
      double(max_angle_deg),
      min_region_size,
      double(min_radius),
      double(max_radius),
      segments,
      region);
  r_count = int(result.alpha_used);
  copy_region_buf(storage, out_region);
  if (mode == 4) {
    cgal_bridge::WireResult wire;
    wire.ok = result.ok;
    wire.error = result.error;
    wire.positions = std::move(result.positions);
    wire.edge_v0 = std::move(result.seam_vert_a);
    wire.edge_v1 = std::move(result.seam_vert_b);
    return wire_result_to_mesh(wire, r_error);
  }
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "region_id");
  }
  return out;
}

Mesh *cgal_points_min_annulus_3(Span<float3> points,
                                int segments,
                                float3 &r_center,
                                float &r_outer,
                                float &r_inner,
                                std::string &r_error)
{
  r_center = float3(0.0f);
  r_outer = 0.0f;
  r_inner = 0.0f;
  cgal_bridge::MeshResult result = cgal_bridge::points_min_annulus_3(
      points.cast<float>().data(), int(points.size()), segments);
  r_center = float3(result.center[0], result.center[1], result.center[2]);
  r_outer = result.radius;
  r_inner = float(result.alpha_used);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_collapse_short_edges(const Mesh &mesh, float max_length, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_collapse_short_edges(in, double(max_length));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  return out;
}

bool cgal_regularize_open_polyline_xy(Span<float3> points,
                                      bool closed,
                                      float max_offset,
                                      float min_length,
                                      Vector<float3> &r_out,
                                      std::string &r_error)
{
  r_out.clear();
  std::vector<float> out_xyz;
  if (!cgal_bridge::regularize_open_polyline_xy(points.cast<float>().data(),
                                                int(points.size()),
                                                closed,
                                                double(max_offset),
                                                double(min_length),
                                                out_xyz,
                                                r_error))
  {
    return false;
  }
  const int n = int(out_xyz.size() / 3);
  r_out.reserve(n);
  for (int i = 0; i < n; i++) {
    r_out.append(float3(out_xyz[size_t(i) * 3 + 0],
                        out_xyz[size_t(i) * 3 + 1],
                        out_xyz[size_t(i) * 3 + 2]));
  }
  return r_out.size() >= 2;
}

Mesh *cgal_mesh_constrained_simplify(const Mesh &mesh,
                                     float keep_ratio,
                                     bool keep_boundary,
                                     Span<uint8_t> preserve,
                                     std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_constrained_simplify(
      in,
      double(keep_ratio),
      keep_boundary,
      preserve.data(),
      int(preserve.size()));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  return out;
}

static void packed_to_polylines(const std::vector<std::vector<float>> &packed,
                                Vector<Vector<float3>> &r_polylines)
{
  r_polylines.clear();
  r_polylines.reserve(int(packed.size()));
  for (const auto &pl : packed) {
    if (pl.size() < 6) {
      continue;
    }
    Vector<float3> curve;
    curve.reserve(int(pl.size() / 3));
    for (size_t i = 0; i + 2 < pl.size(); i += 3) {
      curve.append(float3(pl[i], pl[i + 1], pl[i + 2]));
    }
    if (curve.size() >= 2) {
      r_polylines.append(std::move(curve));
    }
  }
}

bool cgal_mesh_cone_slice(const Mesh &mesh,
                          float3 apex,
                          float3 axis,
                          float radius,
                          float height,
                          int segments,
                          Vector<Vector<float3>> &r_polylines,
                          std::string &r_error)
{
  r_polylines.clear();
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  std::vector<std::vector<float>> packed;
  if (!cgal_bridge::mesh_cone_slice(in,
                                    apex.x,
                                    apex.y,
                                    apex.z,
                                    axis.x,
                                    axis.y,
                                    axis.z,
                                    radius,
                                    height,
                                    segments,
                                    packed,
                                    r_error))
  {
    return false;
  }
  packed_to_polylines(packed, r_polylines);
  return !r_polylines.is_empty();
}

Mesh *cgal_mesh_clip_box(const Mesh &mesh,
                         float3 center,
                         float3 size,
                         bool clip_volume,
                         std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_clip_box(
      in, center.x, center.y, center.z, size.x, size.y, size.z, clip_volume);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  return out;
}

Mesh *cgal_mesh_clip_by_mesh(const Mesh &mesh,
                             const Mesh &clipper,
                             bool clip_volume,
                             std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  cgal_bridge::MeshIn cut = mesh_to_cgal_in(clipper);
  cgal_bridge::MeshResult result = cgal_bridge::mesh_clip_by_mesh(in, cut, clip_volume);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0 && (result.has_face_map() || result.has_vert_map())) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  return out;
}

Mesh *cgal_points_rectangular_p_center_2(Span<float3> points,
                                         int p,
                                         float &r_radius,
                                         std::string &r_error)
{
  r_radius = 0.0f;
  if (points.size() < 2) {
    r_error = "Rectangular P-Center 2D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_rectangular_p_center_2(
      points.cast<float>().data(), int(points.size()), p);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "square_id");
    r_radius = result.radius;
  }
  return out;
}

Mesh *cgal_mesh_polyline_hull_2(const Mesh &mesh, std::string &r_error)
{
  cgal_bridge::MeshIn in = mesh_to_cgal_in(mesh);
  if (in.verts_num < 3) {
    r_error = "Polyline Hull 2D needs a polygon or polyline mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_polyline_hull_2(in);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    write_face_tag(*out, result, "island_id");
  }
  return out;
}

static bool packed_polylines_to_vectors(const std::vector<std::vector<float>> &packed,
                                        Vector<Vector<float3>> &r_polylines)
{
  r_polylines.clear();
  r_polylines.reserve(int(packed.size()));
  for (const auto &pl : packed) {
    if (pl.size() < 6) {
      continue;
    }
    Vector<float3> curve;
    curve.reserve(int(pl.size() / 3));
    for (size_t i = 0; i + 2 < pl.size(); i += 3) {
      curve.append(float3(pl[i], pl[i + 1], pl[i + 2]));
    }
    if (curve.size() >= 2) {
      r_polylines.append(std::move(curve));
    }
  }
  return !r_polylines.is_empty();
}

bool cgal_mesh_curve_intersect(const Mesh &mesh,
                               Span<float3> curve_xyz,
                               Span<int> curve_offsets,
                               Vector<Vector<float3>> &r_segments,
                               Vector<float3> &r_points,
                               std::string &r_error)
{
  r_segments.clear();
  r_points.clear();
  if (mesh.faces_num <= 0 || curve_xyz.size() < 2 || curve_offsets.size() < 2) {
    r_error = "Curve Mesh Intersect needs a mesh and polylines";
    return false;
  }
  std::vector<int> offsets(curve_offsets.begin(), curve_offsets.end());
  std::vector<std::vector<float>> packed;
  std::vector<float> pts;
  if (!cgal_bridge::mesh_curve_intersect(mesh_to_cgal_in(mesh),
                                         curve_xyz.cast<float>().data(),
                                         int(curve_xyz.size()),
                                         offsets.data(),
                                         int(offsets.size()) - 1,
                                         packed,
                                         pts,
                                         r_error))
  {
    return false;
  }
  packed_polylines_to_vectors(packed, r_segments);
  r_points.reserve(int(pts.size() / 3));
  for (size_t i = 0; i + 2 < pts.size(); i += 3) {
    r_points.append(float3(pts[i], pts[i + 1], pts[i + 2]));
  }
  return !r_segments.is_empty() || !r_points.is_empty();
}

bool cgal_mesh_geodesic_isolines(const Mesh &mesh,
                                 Span<bool> sources,
                                 int count,
                                 float max_distance,
                                 Vector<Vector<float3>> &r_polylines,
                                 std::string &r_error)
{
  r_polylines.clear();
  if (mesh.verts_num < 3 || mesh.faces_num < 1) {
    r_error = "Geodesic Isolines needs a mesh with faces";
    return false;
  }
  std::vector<uint8_t> mask(size_t(mesh.verts_num), 0);
  const int n = std::min(mesh.verts_num, int(sources.size()));
  for (int i = 0; i < n; i++) {
    mask[size_t(i)] = sources[i] ? 1 : 0;
  }
  std::vector<std::vector<float>> packed;
  if (!cgal_bridge::mesh_geodesic_isolines(mesh_to_cgal_in(mesh),
                                           mask.data(),
                                           count,
                                           max_distance,
                                           packed,
                                           r_error))
  {
    return false;
  }
  return packed_polylines_to_vectors(packed, r_polylines);
}

Mesh *cgal_mesh_overlap_faces(const Mesh &mesh, const Mesh &other, std::string &r_error)
{
  if (mesh.faces_num <= 0 || other.faces_num <= 0) {
    r_error = "Overlap Faces needs two meshes with faces";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_overlap_faces(mesh_to_cgal_in(mesh),
                                                                   mesh_to_cgal_in(other));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    if (result.has_face_map() || result.has_vert_map()) {
      interpolate_attributes_from_maps(*out, mesh, result);
    }
  }
  return out;
}

bool cgal_mesh_contour_stack(const Mesh &mesh,
                             float3 origin,
                             float3 direction,
                             int count,
                             float spacing,
                             Vector<Vector<float3>> &r_polylines,
                             std::string &r_error)
{
  r_polylines.clear();
  if (mesh.faces_num <= 0) {
    r_error = "Contour Stack needs a mesh with faces";
    return false;
  }
  std::vector<std::vector<float>> packed;
  if (!cgal_bridge::mesh_contour_stack(mesh_to_cgal_in(mesh),
                                       origin.x,
                                       origin.y,
                                       origin.z,
                                       direction.x,
                                       direction.y,
                                       direction.z,
                                       count,
                                       spacing,
                                       packed,
                                       r_error))
  {
    return false;
  }
  return packed_polylines_to_vectors(packed, r_polylines);
}

Mesh *cgal_points_alpha_edges_3(Span<float3> points,
                                float alpha,
                                bool optimal,
                                int solid_components,
                                float &r_alpha_used,
                                std::string &r_error)
{
  r_alpha_used = 0.0f;
  if (points.size() < 4) {
    r_error = "Alpha Edges 3D needs at least 4 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::WireResult wire = cgal_bridge::points_alpha_edges_3(
      points.cast<float>().data(), int(points.size()), double(alpha), optimal, solid_components);
  r_alpha_used = 0.0f;
  return wire_result_to_mesh(wire, r_error);
}

Mesh *cgal_points_delaunay_edges_3(Span<float3> points, std::string &r_error)
{
  if (points.size() < 2) {
    r_error = "Delaunay Edges 3D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(
      cgal_bridge::points_delaunay_edges_3(points.cast<float>().data(), int(points.size())),
      r_error);
}

bool cgal_mesh_edge_path(const Mesh &mesh,
                         Span<bool> sources,
                         Span<bool> targets,
                         Vector<float3> &r_polyline,
                         std::string &r_error)
{
  r_polyline.clear();
  if (mesh.verts_num < 2 || mesh.faces_num < 1) {
    r_error = "Mesh Edge Path needs a mesh with faces";
    return false;
  }
  std::vector<uint8_t> src(size_t(mesh.verts_num), 0);
  std::vector<uint8_t> tgt(size_t(mesh.verts_num), 0);
  const int ns = std::min(mesh.verts_num, int(sources.size()));
  const int nt = std::min(mesh.verts_num, int(targets.size()));
  for (int i = 0; i < ns; i++) {
    src[size_t(i)] = sources[i] ? 1 : 0;
  }
  for (int i = 0; i < nt; i++) {
    tgt[size_t(i)] = targets[i] ? 1 : 0;
  }
  std::vector<float> xyz;
  if (!cgal_bridge::mesh_edge_path(mesh_to_cgal_in(mesh), src.data(), tgt.data(), xyz, r_error)) {
    return false;
  }
  r_polyline.reserve(int(xyz.size() / 3));
  for (size_t i = 0; i + 2 < xyz.size(); i += 3) {
    r_polyline.append(float3(xyz[i], xyz[i + 1], xyz[i + 2]));
  }
  return r_polyline.size() >= 2;
}

bool cgal_mesh_curvature_isolines(const Mesh &mesh,
                                  int mode,
                                  int count,
                                  Vector<Vector<float3>> &r_polylines,
                                  std::string &r_error)
{
  r_polylines.clear();
  if (mesh.faces_num <= 0) {
    r_error = "Curvature Isolines needs a mesh with faces";
    return false;
  }
  std::vector<std::vector<float>> packed;
  if (!cgal_bridge::mesh_curvature_isolines(
          mesh_to_cgal_in(mesh), mode, count, packed, r_error))
  {
    return false;
  }
  return packed_polylines_to_vectors(packed, r_polylines);
}

Mesh *cgal_mesh_offset_sdf(const Mesh &mesh,
                           float offset,
                           int resolution,
                           bool signed_distance,
                           std::string &r_error)
{
  if (mesh.faces_num <= 0) {
    r_error = "Offset Mesh 3D needs a mesh with faces";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_offset_sdf(
      mesh_to_cgal_in(mesh), double(offset), resolution, signed_distance);
  return result_to_mesh(result, r_error);
}

Mesh *cgal_mesh_cdt_hole_fill(const Mesh &mesh, std::string &r_error)
{
  if (mesh.faces_num <= 0) {
    r_error = "CDT Hole Fill needs a mesh with faces";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_cdt_hole_fill(mesh_to_cgal_in(mesh));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    interpolate_attributes_from_maps(*out, mesh, result);
  }
  return out;
}

Mesh *cgal_points_bisector_surface(Span<float3> a, Span<float3> b, std::string &r_error)
{
  if (a.size() < 1 || b.size() < 1) {
    r_error = "Bisector Surface needs points on both sides";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_bisector_surface(
      a.cast<float>().data(), int(a.size()), b.cast<float>().data(), int(b.size()));
  return result_to_mesh(result, r_error);
}

Mesh *cgal_mesh_visibility_graph_2(const Mesh &mesh, std::string &r_error)
{
  if (mesh.verts_num < 2) {
    r_error = "Visibility Graph 2D needs at least 2 verts";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(cgal_bridge::mesh_visibility_graph_2(mesh_to_cgal_in(mesh)), r_error);
}

Mesh *cgal_mesh_terrain_tin(const Mesh &mesh, int fill_rule, std::string &r_error)
{
  if (mesh.verts_num < 3) {
    r_error = "Terrain TIN needs at least 3 verts";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_terrain_tin(mesh_to_cgal_in(mesh), fill_rule);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_min_circle_3(Span<float3> points,
                               int segments,
                               float3 &r_center,
                               float3 &r_normal,
                               float &r_radius,
                               std::string &r_error)
{
  r_center = float3(0.0f);
  r_normal = float3(0.0f, 0.0f, 1.0f);
  r_radius = 0.0f;
  if (points.size() < 2) {
    r_error = "Min Circle 3D needs at least 2 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_min_circle_3(
      points.cast<float>().data(), int(points.size()), segments);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
    r_center = float3(result.center[0], result.center[1], result.center[2]);
    r_radius = result.radius;
    if (out->verts_num >= 3) {
      const Span<float3> p = out->vert_positions();
      r_normal = math::normalize(math::cross(p[1] - p[0], p[2] - p[0]));
      if (math::is_zero(r_normal)) {
        r_normal = float3(0.0f, 0.0f, 1.0f);
      }
    }
  }
  return out;
}

Mesh *cgal_mesh_interior_tets(const Mesh &mesh, std::string &r_error)
{
  if (mesh.verts_num < 4 || mesh.faces_num < 4) {
    r_error = "Interior Tets needs a closed mesh with ≥4 verts and faces";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_interior_tets(mesh_to_cgal_in(mesh));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_points_surface_delaunay_graph(Span<float3> points, int knn, std::string &r_error)
{
  if (points.size() < 4) {
    r_error = "Surface Delaunay Graph needs at least 4 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(
      cgal_bridge::points_surface_delaunay_graph(
          points.cast<float>().data(), int(points.size()), knn),
      r_error);
}

Mesh *cgal_mesh_split_charts(const Mesh &mesh, float angle_deg, std::string &r_error)
{
  if (mesh.faces_num < 1) {
    r_error = "Split Charts needs a mesh with faces";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_split_charts(
      mesh_to_cgal_in(mesh), double(angle_deg));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_restricted_voronoi(const Mesh &mesh, Span<float3> sites, std::string &r_error)
{
  if (mesh.faces_num < 1) {
    r_error = "Restricted Voronoi needs a mesh with faces";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  if (sites.size() < 1) {
    r_error = "Restricted Voronoi needs at least 1 site";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_restricted_voronoi(
      mesh_to_cgal_in(mesh), sites.cast<float>().data(), int(sites.size()));
  Mesh *out = result_to_mesh(result, r_error);
  if (!out || out->faces_num == 0) {
    return out;
  }
  if (int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "cell_id", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
  }
  shade_all_sharp(*out);
  return out;
}

Mesh *cgal_points_walk_tets(Span<float3> points, float3 start, float3 end, std::string &r_error)
{
  if (points.size() < 4) {
    r_error = "Walk Tets needs at least 4 points";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_walk_tets(points.cast<float>().data(),
                                                                 int(points.size()),
                                                                 double(start.x),
                                                                 double(start.y),
                                                                 double(start.z),
                                                                 double(end.x),
                                                                 double(end.y),
                                                                 double(end.z));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_skeleton_spokes(const Mesh &mesh, std::string &r_error)
{
  if (mesh.faces_num < 1) {
    r_error = "Skeleton Spokes needs a closed mesh";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  return wire_result_to_mesh(cgal_bridge::mesh_skeleton_spokes(mesh_to_cgal_in(mesh)), r_error);
}

bool cgal_mesh_radial_slices(const Mesh &mesh,
                             float3 origin,
                             float3 axis,
                             int count,
                             Vector<Vector<float3>> &r_polylines,
                             std::string &r_error)
{
  r_polylines.clear();
  if (mesh.faces_num <= 0) {
    r_error = "Radial Slices needs a mesh with faces";
    return false;
  }
  std::vector<std::vector<float>> packed;
  if (!cgal_bridge::mesh_radial_slices(mesh_to_cgal_in(mesh),
                                       origin.x,
                                       origin.y,
                                       origin.z,
                                       axis.x,
                                       axis.y,
                                       axis.z,
                                       count,
                                       packed,
                                       r_error))
  {
    return false;
  }
  return packed_polylines_to_vectors(packed, r_polylines);
}

Mesh *cgal_mesh_intersection_band(const Mesh &a, const Mesh &b, std::string &r_error)
{
  if (a.faces_num < 1 || b.faces_num < 1) {
    r_error = "Intersection Band needs two meshes with faces";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_intersection_band(mesh_to_cgal_in(a),
                                                                       mesh_to_cgal_in(b));
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_projected_outline(const Mesh &mesh, float3 direction, std::string &r_error)
{
  if (mesh.faces_num <= 0) {
    r_error = "Projected Outline needs a mesh with faces";
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  cgal_bridge::MeshResult result = cgal_bridge::mesh_projected_outline(
      mesh_to_cgal_in(mesh), direction.x, direction.y, direction.z);
  return result_to_mesh(result, r_error);
}

#include "mesh_cgal_batch55.inc"
#include "mesh_cgal_batch56.inc"
#include "mesh_cgal_batch57.inc"

#endif

using ConvexTri = std::array<float3, 3>;

struct ConvexPart {
  Vector<ConvexTri> tris;
};

static float3 convex_pts_centroid(Span<float3> pts)
{
  float3 c(0.0f);
  if (pts.is_empty()) {
    return c;
  }
  for (const float3 &p : pts) {
    c += p;
  }
  return c / float(pts.size());
}

static float3 convex_pts_pca_axis(Span<float3> pts, const float3 &c, const bool xy_only)
{
  float xx = 0.0f, xy = 0.0f, xz = 0.0f, yy = 0.0f, yz = 0.0f, zz = 0.0f;
  for (const float3 &p : pts) {
    float3 d = p - c;
    if (xy_only) {
      d.z = 0.0f;
    }
    xx += d.x * d.x;
    xy += d.x * d.y;
    xz += d.x * d.z;
    yy += d.y * d.y;
    yz += d.y * d.z;
    zz += d.z * d.z;
  }
  float3 v = xy_only ? float3(1.0f, 0.2f, 0.0f) : float3(1.0f, 0.2f, 0.1f);
  for (int i = 0; i < 16; i++) {
    float3 av(xx * v.x + xy * v.y + xz * v.z,
              xy * v.x + yy * v.y + yz * v.z,
              xz * v.x + yz * v.y + zz * v.z);
    if (xy_only) {
      av.z = 0.0f;
    }
    const float len = math::length(av);
    if (len < 1.0e-20f) {
      return xy_only ? float3(1.0f, 0.0f, 0.0f) : float3(1.0f, 0.0f, 0.0f);
    }
    v = av / len;
  }
  return v;
}

static Vector<float3> convex_part_points(const ConvexPart &part)
{
  Vector<float3> pts;
  Set<uint64_t> seen;
  constexpr float scale = 1.0e5f;
  auto add = [&](const float3 &p) {
    const int ix = int(std::lround(double(p.x) * scale));
    const int iy = int(std::lround(double(p.y) * scale));
    const int iz = int(std::lround(double(p.z) * scale));
    const uint64_t key = (uint64_t(uint32_t(ix) & 0x1FFFFFu) << 42) |
                         (uint64_t(uint32_t(iy) & 0x1FFFFFu) << 21) |
                         uint64_t(uint32_t(iz) & 0x1FFFFFu);
    if (seen.add(key)) {
      pts.append(p);
    }
  };
  for (const ConvexTri &t : part.tris) {
    add(t[0]);
    add(t[1]);
    add(t[2]);
  }
  return pts;
}

static void convex_emit_poly(const Vector<float3> &poly, Vector<ConvexTri> &out)
{
  for (int i = 1; i + 1 < int(poly.size()); i++) {
    out.append(ConvexTri{poly[0], poly[i], poly[i + 1]});
  }
}

static void convex_clip_tri(const ConvexTri &tri,
                            const float3 &n,
                            const float pd,
                            const float eps,
                            Vector<ConvexTri> &neg,
                            Vector<ConvexTri> &pos)
{
  float sd[3];
  int sg[3];
  for (int i = 0; i < 3; i++) {
    sd[i] = math::dot(n, tri[i]) + pd;
    sg[i] = (sd[i] > eps) ? 1 : ((sd[i] < -eps) ? -1 : 0);
  }
  if (sg[0] <= 0 && sg[1] <= 0 && sg[2] <= 0) {
    neg.append(tri);
    if (sg[0] == 0 && sg[1] == 0 && sg[2] == 0) {
      pos.append(tri);
    }
    return;
  }
  if (sg[0] >= 0 && sg[1] >= 0 && sg[2] >= 0) {
    pos.append(tri);
    return;
  }

  Vector<float3> npoly;
  Vector<float3> ppoly;
  npoly.reserve(5);
  ppoly.reserve(5);
  for (int i = 0; i < 3; i++) {
    const int j = (i + 1) % 3;
    if (sg[i] <= 0) {
      npoly.append(tri[i]);
    }
    if (sg[i] >= 0) {
      ppoly.append(tri[i]);
    }
    if (sg[i] * sg[j] < 0) {
      const float t = sd[i] / (sd[i] - sd[j]);
      const float3 x = math::interpolate(tri[i], tri[j], math::clamp(t, 0.0f, 1.0f));
      npoly.append(x);
      ppoly.append(x);
    }
  }
  convex_emit_poly(npoly, neg);
  convex_emit_poly(ppoly, pos);
}

static bool convex_choose_plane(const ConvexPart &part,
                                const bool xy_only,
                                float3 &r_n,
                                float &r_d)
{
  const Vector<float3> pts = convex_part_points(part);
  if (pts.size() < 4) {
    return false;
  }
  const float3 c = convex_pts_centroid(pts);
  r_n = convex_pts_pca_axis(pts, c, xy_only);
  if (math::length_squared(r_n) < 1.0e-12f) {
    r_n = xy_only ? float3(1.0f, 0.0f, 0.0f) : float3(1.0f, 0.0f, 0.0f);
  }
  r_n = math::normalize(r_n);
  r_d = -math::dot(r_n, c);
  return true;
}

static bool convex_split_part(const ConvexPart &in,
                              const bool xy_only,
                              ConvexPart &left,
                              ConvexPart &right)
{
  left.tris.clear();
  right.tris.clear();
  float3 n;
  float d;
  if (!convex_choose_plane(in, xy_only, n, d)) {
    return false;
  }
  for (const ConvexTri &t : in.tris) {
    convex_clip_tri(t, n, d, 1.0e-6f, left.tris, right.tris);
  }
  if (left.tris.size() < 1 || right.tris.size() < 1) {
    left.tris.clear();
    right.tris.clear();
    float3 mn(1.0e30f);
    float3 mx(-1.0e30f);
    for (const ConvexTri &t : in.tris) {
      for (int k = 0; k < 3; k++) {
        mn = math::min(mn, t[k]);
        mx = math::max(mx, t[k]);
      }
    }
    float3 ext = mx - mn;
    if (xy_only) {
      ext.z = 0.0f;
    }
    int ax = 0;
    if (ext.y > ext.x) {
      ax = 1;
    }
    if (!xy_only && ext.z > ext[ax]) {
      ax = 2;
    }
    n = float3(0.0f);
    n[ax] = 1.0f;
    d = -0.5f * (mn[ax] + mx[ax]);
    for (const ConvexTri &t : in.tris) {
      convex_clip_tri(t, n, d, 1.0e-6f, left.tris, right.tris);
    }
  }
  return left.tris.size() >= 1 && right.tris.size() >= 1;
}

static ConvexPart convex_part_from_mesh(const Mesh &mesh)
{
  ConvexPart part;
  const Span<float3> pos = mesh.vert_positions();
  const Span<int> cv = mesh.corner_verts();
  const Span<int3> tris = mesh.corner_tris();
  part.tris.reserve(tris.size());
  for (const int3 t : tris) {
    const int v0 = cv[t[0]];
    const int v1 = cv[t[1]];
    const int v2 = cv[t[2]];
    if (v0 < 0 || v1 < 0 || v2 < 0 || v0 >= mesh.verts_num || v1 >= mesh.verts_num ||
        v2 >= mesh.verts_num)
    {
      continue;
    }
    part.tris.append(ConvexTri{pos[v0], pos[v1], pos[v2]});
  }
  return part;
}

static bool part_already_convex(Span<float3> pts)
{
  if (pts.size() < 5) {
    return true;
  }
  std::string err;
  Mesh *hull = cgal_convex_hull_3(pts, err);
  if (!hull || hull->faces_num == 0) {
    if (hull) {
      BKE_id_free(nullptr, hull);
    }
    return false;
  }
  const int hull_verts = hull->verts_num;
  BKE_id_free(nullptr, hull);
  return hull_verts >= int(pts.size()) - 1;
}

static void convex_split_to_count(Vector<ConvexPart> &parts, const int max_hulls, const bool xy_only)
{
  const int target = math::max(max_hulls, 1);
  while (int(parts.size()) < target) {
    int best = -1;
    int best_n = 0;
    for (const int i : parts.index_range()) {
      const int n = int(parts[i].tris.size());
      if (n > best_n && !part_already_convex(convex_part_points(parts[i]))) {
        best_n = n;
        best = i;
      }
    }
    if (best < 0 || best_n < 2) {
      break;
    }
    ConvexPart left;
    ConvexPart right;
    if (!convex_split_part(parts[best], xy_only, left, right)) {
      break;
    }
    parts[best] = std::move(left);
    parts.append(std::move(right));
  }
}

static Mesh *convex_mesh_from_ring_xy(Span<float2> ring, const float z)
{
  const int n = int(ring.size());
  if (n < 3) {
    return nullptr;
  }
  Mesh *mesh = BKE_mesh_new_nomain(n, 0, 1, n);
  MutableSpan<float3> pos = mesh->vert_positions_for_write();
  for (int i = 0; i < n; i++) {
    pos[i] = float3(ring[i].x, ring[i].y, z);
  }
  mesh->face_offsets_for_write()[0] = 0;
  mesh->face_offsets_for_write()[1] = n;
  MutableSpan<int> corners = mesh->corner_verts_for_write();
  for (int i = 0; i < n; i++) {
    corners[i] = i;
  }
  bke::mesh_calc_edges(*mesh, false, false);
  mesh->tag_overlapping_none();
  bke::mesh_smooth_set(*mesh, false);
  return mesh;
}

static Vector<float2> convex_hull_2d_ring(Span<float2> in)
{
  Vector<float2> pts(in);
  if (pts.size() < 3) {
    return pts;
  }
  std::sort(pts.begin(), pts.end(), [](const float2 &a, const float2 &b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });
  Vector<float2> unique;
  unique.reserve(pts.size());
  for (const float2 &p : pts) {
    if (unique.is_empty() || math::distance_squared(unique.last(), p) > 1.0e-16f) {
      unique.append(p);
    }
  }
  if (unique.size() < 3) {
    return unique;
  }
  auto cross = [](const float2 &o, const float2 &a, const float2 &b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  };
  Vector<float2> hull;
  for (const float2 &p : unique) {
    while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.last(), p) <= 0.0f) {
      hull.remove_last();
    }
    hull.append(p);
  }
  const int lower = int(hull.size()) + 1;
  for (int i = int(unique.size()) - 2; i >= 0; i--) {
    while (int(hull.size()) >= lower && cross(hull[hull.size() - 2], hull.last(), unique[i]) <= 0.0f)
    {
      hull.remove_last();
    }
    hull.append(unique[i]);
  }
  if (!hull.is_empty()) {
    hull.remove_last();
  }
  return hull;
}

static void tag_convex_piece(Mesh &mesh, const int piece)
{
  bke::MutableAttributeAccessor attrs = mesh.attributes_for_write();
  bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
      "convex_piece", bke::AttrDomain::Face);
  w.span.fill(piece);
  w.finish();
}

Vector<Mesh *> cgal_mesh_convex_hulls_3(const Mesh &mesh, int max_hulls, std::string &r_error)
{
  Vector<Mesh *> hulls;
  ConvexPart root = convex_part_from_mesh(mesh);
  if (root.tris.size() < 1) {
    r_error = "Convex Decomposition 3D needs a mesh with faces";
    return hulls;
  }
  Vector<ConvexPart> parts;
  parts.append(std::move(root));
  convex_split_to_count(parts, math::clamp(max_hulls, 1, 256), false);

  for (const int i : parts.index_range()) {
    const Vector<float3> pts = convex_part_points(parts[i]);
    if (pts.size() < 4) {
      continue;
    }
    std::string err;
    Mesh *hull = cgal_convex_hull_3(pts, err);
    if (!hull || hull->faces_num == 0) {
      if (hull) {
        BKE_id_free(nullptr, hull);
      }
      continue;
    }
    tag_convex_piece(*hull, int(hulls.size()));
    hulls.append(hull);
  }
  if (hulls.is_empty()) {
    r_error = r_error.empty() ? "Convex Decomposition 3D produced no hulls" : r_error;
  }
  return hulls;
}

Vector<Mesh *> cgal_mesh_convex_hulls_2(const Mesh &mesh, int max_hulls, std::string &r_error)
{
  Vector<Mesh *> hulls;
  ConvexPart root = convex_part_from_mesh(mesh);
  if (root.tris.size() < 1) {
    r_error = "Convex Partition 2D needs a mesh with faces";
    return hulls;
  }
  float z = 0.0f;
  int zn = 0;
  for (const ConvexTri &t : root.tris) {
    z += t[0].z + t[1].z + t[2].z;
    zn += 3;
  }
  z = (zn > 0) ? z / float(zn) : 0.0f;

  Vector<ConvexPart> parts;
  parts.append(std::move(root));
  convex_split_to_count(parts, math::clamp(max_hulls, 1, 256), true);

  for (const int i : parts.index_range()) {
    const Vector<float3> pts = convex_part_points(parts[i]);
    Vector<float2> pts2;
    pts2.reserve(pts.size());
    for (const float3 &p : pts) {
      pts2.append(float2(p.x, p.y));
    }
    const Vector<float2> ring = convex_hull_2d_ring(pts2);
    Mesh *hull = convex_mesh_from_ring_xy(ring, z);
    if (!hull) {
      continue;
    }
    tag_convex_piece(*hull, int(hulls.size()));
    hulls.append(hull);
  }
  if (hulls.is_empty()) {
    r_error = "Convex Partition 2D produced no hulls";
  }
  return hulls;
}

}  // namespace blender::geometry





