/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_array.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include <cstdint>
#include <string>
#include <vector>

namespace blender {

struct Mesh;
struct PointCloud;

namespace geometry {

enum class CgalBooleanOperation { Union = 0, Difference = 1, Intersection = 2 };
enum class CgalSubdivisionMode { Loop = 0, CatmullClark = 1, DooSabin = 2, Sqrt3 = 3 };

Mesh *cgal_triangle_mesh_from_buffers(Span<float3> positions, Span<int> corner_verts);

/* Points */
Mesh *cgal_convex_hull_3(Span<float3> points, std::string &r_error);
Mesh *cgal_alpha_shape_3(Span<float3> points,
                         float alpha,
                         bool use_optimal_alpha,
                         int solid_components,
                         float &r_alpha_used,
                         std::string &r_error);
Mesh *cgal_alpha_wrap_points(Span<float3> points,
                             float alpha,
                             float offset,
                             std::string &r_error);
Mesh *cgal_advancing_front(Span<float3> points,
                           float radius_ratio,
                           float beta,
                           std::string &r_error);
/**
 * Tetrahedralization as triangle mesh. Transfers point attrs from \a src_pointcloud or mesh verts.
 * \a separate_tets: each tet uses its own 4 vertices (no sharing) vs welded shared verts.
 */
Mesh *cgal_delaunay_3d(Span<float3> points,
                       const PointCloud *src_pointcloud,
                       const Mesh *src_mesh,
                       bool separate_tets,
                       std::string &r_error);
Mesh *cgal_min_sphere(Span<float3> points, int segments, std::string &r_error);
Mesh *cgal_optimal_bbox(Span<float3> points, std::string &r_error);

/* Mesh */
Mesh *cgal_mesh_simplify(const Mesh &mesh, float keep_ratio, std::string &r_error);
Mesh *cgal_mesh_isotropic_remesh(const Mesh &mesh,
                                 float edge_length,
                                 int iterations,
                                 std::string &r_error);
Mesh *cgal_mesh_smooth_shape(const Mesh &mesh,
                             float time,
                             int iterations,
                             bool do_scale,
                             std::string &r_error);
Mesh *cgal_mesh_boolean(const Mesh &a,
                        const Mesh &b,
                        CgalBooleanOperation op,
                        std::string &r_error);
Mesh *cgal_mesh_hole_fill(const Mesh &mesh, std::string &r_error);
/**
 * Discrete k-harmonic fairing (Botsch/Kobbelt). Continuity k solves L^{k+1}x=0
 * (internally capped at 8). The original k-ring around pinned verts is kept
 * as a C^k Dirichlet handle (position, then tangent, then curvature, ...).
 */
Mesh *cgal_mesh_fair(const Mesh &mesh,
                     int continuity,
                     std::string &r_error,
                     Span<uint8_t> free_mask = {});
Mesh *cgal_mesh_refine(const Mesh &mesh, float density_factor, std::string &r_error);
Mesh *cgal_mesh_clip_plane(const Mesh &mesh,
                           const float3 &plane_point,
                           const float3 &plane_normal,
                           std::string &r_error);
Mesh *cgal_mesh_subdivision(const Mesh &mesh,
                            CgalSubdivisionMode mode,
                            int steps,
                            std::string &r_error);
Mesh *cgal_mesh_repair(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_keep_largest(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_alpha_wrap(const Mesh &mesh, float alpha, float offset, std::string &r_error);
/**
 * Corefine Mesh A with Mesh B (embed intersection into A).
 * If \a r_seam_edge_selection is non-null, filled with one bool per output edge
 * (true = intersection/seam edge). Size equals out->edges_num.
 */
Mesh *cgal_mesh_detect_features(const Mesh &mesh, float angle, std::string &r_error);
Mesh *cgal_mesh_angle_area_smooth(const Mesh &mesh, int iterations, bool safety, std::string &r_error);
Mesh *cgal_mesh_tangential_relaxation(const Mesh &mesh, int iterations, std::string &r_error);
Mesh *cgal_mesh_extrude(const Mesh &mesh,
                        float distance,
                        const float3 &offset,
                        bool along_normals,
                        std::string &r_error);
Mesh *cgal_mesh_remesh_planar_patches(const Mesh &mesh, float cos_angle, float max_dist, std::string &r_error);
Mesh *cgal_mesh_random_perturbation(const Mesh &mesh, float max_move, bool project, std::string &r_error);
Mesh *cgal_mesh_triangulate(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_orient_outward(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_repair_self_intersections(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_split_long_edges(const Mesh &mesh, float max_length, std::string &r_error);
Mesh *cgal_mesh_connected_component_keep(const Mesh &mesh, int index, std::string &r_error);
Mesh *cgal_mesh_merge_border_vertices(const Mesh &mesh, float distance, std::string &r_error);
bool cgal_mesh_does_self_intersect(const Mesh &mesh, std::string &r_error);
float cgal_mesh_volume(const Mesh &mesh, std::string &r_error);
float cgal_mesh_area(const Mesh &mesh, std::string &r_error);

Mesh *cgal_mesh_corefine(const Mesh &a,
                         const Mesh &b,
                         std::string &r_error,
                         Array<bool> *r_seam_edge_selection = nullptr);

/* Point set processing / reconstruction / mesh queries (batch). */
float cgal_points_average_spacing(Span<float3> points, int neighbors, std::string &r_error);
PointCloud *cgal_points_jet_smooth(Span<float3> points,
                                   int neighbors,
                                   int iterations,
                                   std::string &r_error);
PointCloud *cgal_points_bilateral_smooth(Span<float3> points,
                                         int neighbors,
                                         int iterations,
                                         float sharpness_deg,
                                         std::string &r_error);
PointCloud *cgal_points_remove_outliers(Span<float3> points,
                                        int neighbors,
                                        float percent,
                                        std::string &r_error);
PointCloud *cgal_points_grid_simplify(Span<float3> points,
                                      float cell_size,
                                      std::string &r_error);
PointCloud *cgal_points_random_simplify(Span<float3> points,
                                        float percent,
                                        std::string &r_error);
/**
 * Jet + MST normals. \a src_pc / \a src_mesh (optional) preserve prior point attributes
 * when size matches (pointcloud preferred). Output always has float3 "normal".
 */
PointCloud *cgal_points_estimate_normals(Span<float3> points,
                                         int neighbors,
                                         const PointCloud *src_pc,
                                         const Mesh *src_mesh,
                                         std::string &r_error);
/** WLOP: simplify + regularize. Transfers attrs from \a src_attrs by nearest point. */
PointCloud *cgal_points_wlop(Span<float3> points,
                             float select_percentage,
                             float neighbor_radius,
                             int iterations,
                             bool require_uniform,
                             const PointCloud *src_attrs,
                             std::string &r_error);
/** Hierarchy cluster simplify. Transfers attrs from \a src_attrs by nearest point. */
PointCloud *cgal_points_hierarchy_simplify(Span<float3> points,
                                           int cluster_size,
                                           float max_variation,
                                           const PointCloud *src_attrs,
                                           std::string &r_error);
/**
 * Edge-aware densify: output = original points + new samples (total aims at output_count).
 * Normals empty => jet estimate. Transfers attrs (originals by index, new by nearest).
 */
PointCloud *cgal_points_edge_aware_upsample(Span<float3> points,
                                            Span<float3> normals,
                                            int output_count,
                                            float sharpness_angle_deg,
                                            float edge_sensitivity,
                                            float neighbor_radius,
                                            int normal_neighbors,
                                            const PointCloud *src_pc,
                                            const Mesh *src_mesh,
                                            std::string &r_error);
/**
 * VCM normals (feature-aware). \a src_pc / \a src_mesh preserve prior attributes.
 * Differs from Estimate Normals (jet PCA): VCM uses Voronoi covariance measure.
 */
PointCloud *cgal_points_vcm_estimate_normals(Span<float3> points,
                                             float offset_radius,
                                             float convolution_radius,
                                             const PointCloud *src_pc,
                                             const Mesh *src_mesh,
                                             std::string &r_error);
/**
 * Poisson reconstruction.
 * \a normals empty => estimate with \a neighbors; otherwise must match points size.
 * \a spacing 0 => auto from average spacing / 2.
 */
Mesh *cgal_points_poisson(Span<float3> points,
                          Span<float3> normals,
                          float spacing,
                          int neighbors,
                          std::string &r_error);
/**
 * Query points with bool "Inside". Preserves attributes from \a src_pointcloud
 * (same count) or mesh vertex attributes when queries come from mesh verts.
 */
PointCloud *cgal_mesh_side_of(const Mesh &mesh,
                              Span<float3> query,
                              const PointCloud *src_pointcloud,
                              const Mesh *src_mesh,
                              std::string &r_error);
/** Fill \a r_inside (size == query.size()): true = ON_BOUNDED_SIDE. */
bool cgal_mesh_side_of_query(const Mesh &mesh,
                             Span<float3> query,
                             MutableSpan<bool> r_inside,
                             std::string &r_error);
/** Query points with float attribute "Distance". */
PointCloud *cgal_mesh_distance_to(const Mesh &mesh, Span<float3> query, std::string &r_error);
PointCloud *cgal_mesh_sample_points(const Mesh &mesh, int count, std::string &r_error);

/* Advanced mesh analysis / shape. */
/** Curve skeleton as wire mesh (verts + edges). Closed triangle mesh required. */
Mesh *cgal_mesh_skeleton(const Mesh &mesh, std::string &r_error);
/** Boundary loops as wire mesh (verts + edges). */
Mesh *cgal_mesh_extract_border(const Mesh &mesh, std::string &r_error);
/**
 * Heat-method geodesic distance from source points (snapped to nearest verts).
 * Returns a copy of the input mesh with vertex float attribute "Geodesic".
 */
Mesh *cgal_mesh_geodesic_distance(const Mesh &mesh,
                                  Span<float3> source_points,
                                  std::string &r_error);
/**
 * SDF segmentation. Output triangulated mesh with face attributes
 * "Segment" (int) and "SDF" (float).
 */
Mesh *cgal_mesh_segmentation(const Mesh &mesh,
                             int clusters,
                             float smoothing,
                             std::string &r_error);
/** Approximate min-volume ellipsoid as UV-sphere mesh. */
Mesh *cgal_min_ellipsoid(Span<float3> points, int segments, std::string &r_error);

/* Batch analysis / utility. */
bool cgal_mesh_is_closed(const Mesh &mesh, std::string &r_error);
bool cgal_mesh_centroid(const Mesh &mesh, float3 &r_centroid, std::string &r_error);
Mesh *cgal_mesh_autorefine(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_remove_degenerate(const Mesh &mesh, std::string &r_error);
/** Fill per-vertex mean curvature (Point domain). r_values size must equal verts_num. */
bool cgal_mesh_mean_curvature(const Mesh &mesh, MutableSpan<float> r_values, std::string &r_error);
/** Fill per-vertex Gaussian curvature (Point domain). r_values size must equal verts_num. */
bool cgal_mesh_gaussian_curvature(const Mesh &mesh,
                                  MutableSpan<float> r_values,
                                  std::string &r_error);
/** Scale-space surface reconstruction from an unorganized point set. */
Mesh *cgal_points_scale_space(Span<float3> points, int iterations, std::string &r_error);
/**
 * Scale-space with smoother/mesher choice.
 * \a smoother: 0=Weighted PCA, 1=Jet. \a mesher: 0=Alpha shape, 1=Advancing front.
 */
Mesh *cgal_points_scale_space_ex(Span<float3> points,
                                 int iterations,
                                 int smoother,
                                 int mesher,
                                 std::string &r_error);

/**
 * Principal curvatures + directions (Point domain).
 * r_dmin/r_dmax size == verts_num.
 */
bool cgal_mesh_principal_curvatures(const Mesh &mesh,
                                    MutableSpan<float> r_kmin,
                                    MutableSpan<float> r_kmax,
                                    MutableSpan<float3> r_dmin,
                                    MutableSpan<float3> r_dmax,
                                    std::string &r_error);
bool cgal_mesh_shape_diameter(const Mesh &mesh, MutableSpan<float> r_sdf, std::string &r_error);
/**
 * Per-face flags. \a r_intersect and \a r_inside must equal faces_num
 * (\a r_inside may be empty to skip the inside test).
 * Self Intersect: mutual interior crossing (or adjacent bow-tie).
 * Inside: face is in the self-intersection interior (winding >= 2). Intersection faces are false.
 */
bool cgal_mesh_mark_self_intersect(const Mesh &mesh,
                                   MutableSpan<bool> r_intersect,
                                   MutableSpan<bool> r_inside,
                                   std::string &r_error);

/* Batch 17. */
bool cgal_mesh_is_outward_oriented(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_reverse_orientation(const Mesh &mesh, std::string &r_error);
int cgal_mesh_hole_count(const Mesh &mesh, std::string &r_error);
bool cgal_mesh_border_edges(const Mesh &mesh, MutableSpan<bool> r_border, std::string &r_error);
bool cgal_mesh_dihedral_angles(const Mesh &mesh, MutableSpan<float> r_angle_rad, std::string &r_error);
Mesh *cgal_mesh_duplicate_non_manifold(const Mesh &mesh, std::string &r_error);

/* Batch 18 — 15 new nodes. */
PointCloud *cgal_points_pca_estimate_normals(Span<float3> points,
                                             int neighbors,
                                             const PointCloud *src_attrs,
                                             std::string &r_error);
PointCloud *cgal_points_mst_orient_normals(Span<float3> points,
                                           Span<float3> normals,
                                           int neighbors,
                                           const PointCloud *src_attrs,
                                           std::string &r_error);
PointCloud *cgal_points_radial_orient_normals(Span<float3> points,
                                              Span<float3> normals,
                                              const PointCloud *src_attrs,
                                              std::string &r_error);
PointCloud *cgal_points_cluster(Span<float3> points,
                                float neighbor_radius,
                                const PointCloud *src_attrs,
                                int &r_cluster_count,
                                std::string &r_error);
PointCloud *cgal_mesh_polyhedral_envelope(const Mesh &mesh,
                                          float epsilon,
                                          Span<float3> query,
                                          const PointCloud *src_pointcloud,
                                          const Mesh *src_mesh,
                                          std::string &r_error);
Mesh *cgal_mesh_vertex_normals(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_face_normals(const Mesh &mesh, std::string &r_error);
/** Mesh copy with Face float attribute "Aspect Ratio" (max/min edge). */
Mesh *cgal_mesh_face_aspect_ratio(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_vertex_valence(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_mean_edge_length(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_border_vertex(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_face_quality(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_region_growing(const Mesh &mesh,
                               float max_distance,
                               float max_angle_deg,
                               int min_region_size,
                               int &r_region_count,
                               std::string &r_error);
Mesh *cgal_mesh_surface_shortest_path(const Mesh &mesh,
                                      const float3 &source,
                                      const float3 &target,
                                      std::string &r_error);
PointCloud *cgal_mesh_locate(const Mesh &mesh,
                             Span<float3> query,
                             const PointCloud *src_pointcloud,
                             const Mesh *src_mesh,
                             std::string &r_error);

/* Batch 19 — 6 new nodes. */
/** Mesh copy with Edge float attribute "Edge Length". */
Mesh *cgal_mesh_edge_length(const Mesh &mesh, std::string &r_error);
/** Mesh copy with Face float attribute "Perimeter". */
Mesh *cgal_mesh_face_perimeter(const Mesh &mesh, std::string &r_error);
/** Arithmetic mean of point positions. */
bool cgal_points_centroid(Span<float3> points, float3 &r_centroid, std::string &r_error);
/**
 * Least-squares plane from points. Returns a single-quad mesh; also writes
 * center / unit normal / half-extent used for the quad size.
 */
Mesh *cgal_points_fit_plane(Span<float3> points,
                            float3 &r_center,
                            float3 &r_normal,
                            std::string &r_error);
/** CGAL estimate_global_k_neighbor_scale → recommended k-nearest neighborhood size. */
bool cgal_points_neighbor_scale(Span<float3> points, int &r_scale_k, std::string &r_error);
/**
 * Orient point normals along LiDAR-style scanlines (CGAL scanline_orient_normals).
 * Preserves prior point attributes when \a src_pc size matches.
 */
PointCloud *cgal_points_scanline_orient_normals(Span<float3> points,
                                                Span<float3> normals,
                                                const PointCloud *src_pc,
                                                const Mesh *src_mesh,
                                                std::string &r_error);

/* Batch 20 — 15-node goal (3 rewired existing + keep component + 11 new). */
PointCloud *cgal_points_local_neighbor_scales(Span<float3> points,
                                              const PointCloud *src_pc,
                                              const Mesh *src_mesh,
                                              std::string &r_error);
Mesh *cgal_mesh_remove_small_components(const Mesh &mesh, int min_faces, std::string &r_error);
Mesh *cgal_points_fit_line(Span<float3> points,
                           float3 &r_center,
                           float3 &r_direction,
                           std::string &r_error);
bool cgal_points_diameter(Span<float3> points, float &r_diameter, std::string &r_error);
Mesh *cgal_mesh_orient_polygon_soup(const Mesh &mesh, std::string &r_error);
bool cgal_mesh_self_intersection_count(const Mesh &mesh, int &r_count, std::string &r_error);
PointCloud *cgal_points_local_density(Span<float3> points,
                                      int neighbors,
                                      const PointCloud *src_pc,
                                      const Mesh *src_mesh,
                                      std::string &r_error);
bool cgal_mesh_compactness(const Mesh &mesh, float &r_compactness, std::string &r_error);
Mesh *cgal_mesh_component_size(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_face_planarity(const Mesh &mesh, std::string &r_error);
bool cgal_mesh_is_triangle_mesh(const Mesh &mesh, bool &r_is_triangle, std::string &r_error);

/* Batch 21 — catalog high-value (ARAP / LSCM / Efficient RANSAC). */
/**
 * ARAP surface deformation. \a roi_mask / \a control_mask size == verts_num.
 * \a target_xyz size verts_num (control verts only used).
 * \a algorithm: 0=ORIGINAL_ARAP, 1=SPOKES_AND_RIMS.
 */
Mesh *cgal_mesh_arap_deform(const Mesh &mesh,
                            Span<uint8_t> roi_mask,
                            Span<uint8_t> control_mask,
                            Span<float3> target_xyz,
                            int algorithm,
                            int iterations,
                            float tolerance,
                            std::string &r_error);
/**
 * UV parameterize per face corner (same order as mesh.corner_verts).
 * \a method: 0=LSCM, 1=ARAP, 2=Discrete Conformal, 3=Mean Value,
 *            4=Discrete Authalic, 5=Barycentric, 6=Iterative Authalic.
 * \a seam_per_edge size == edges_num (true = seam). Empty = no seams.
 * \a face_selection size == faces_num. Empty = all faces.
 * \a r_corner_uv size must equal corners_num; written as float2.
 */
bool cgal_mesh_parameterize_uv_corners(const Mesh &mesh,
                                       Span<bool> seam_per_edge,
                                       Span<bool> face_selection,
                                       bool normalize,
                                       int method,
                                       int energy_iterations,
                                       float lambda,
                                       MutableSpan<float2> r_corner_uv,
                                       std::string &r_error);
inline bool cgal_mesh_lscm_uv_corners(const Mesh &mesh,
                                      Span<bool> seam_per_edge,
                                      Span<bool> face_selection,
                                      bool normalize,
                                      MutableSpan<float2> r_corner_uv,
                                      std::string &r_error)
{
  return cgal_mesh_parameterize_uv_corners(
      mesh, seam_per_edge, face_selection, normalize, 0, 0, 1000.0f, r_corner_uv, r_error);
}

/**
 * Point-set planar region growing. Writes Point int regions in \a r_region (-1 unassigned).
 * Empty normals => jet estimate. \a r_region size must equal points size.
 */
bool cgal_points_region_growing_planes(Span<float3> points,
                                       Span<float3> normals,
                                       float neighbor_radius,
                                       float max_distance,
                                       float max_angle_deg,
                                       int min_region_size,
                                       MutableSpan<int> r_region,
                                       int &r_region_count,
                                       std::string &r_error);
/**
 * Efficient RANSAC. Writes Point int "Shape" (-1 unassigned) and "Shape Type"
 * (0=Plane,1=Sphere,2=Cylinder,3=Cone,4=Torus). Preserves attrs from src.
 * shape_flags: bit0 plane, bit1 sphere, bit2 cylinder, bit3 cone, bit4 torus.
 * epsilon/cluster_epsilon <= 0 use CGAL defaults.
 * \a random_seed: fixed seed for deterministic RANSAC (viewport-stable).
 */
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
                                         std::string &r_error);

/* Catalog batch 23. */
/** Almost-degenerate repair (needles/caps). Cap is cos(largest angle). */
Mesh *cgal_mesh_repair_degeneracies(const Mesh &mesh,
                                    float cap_threshold,
                                    float needle_threshold,
                                    float collapse_length,
                                    std::string &r_error);
/**
 * Mesh–mesh surface intersection as polylines (xyz packed per curve).
 * Returns true even if empty intersection (polylines empty).
 */
bool cgal_mesh_surface_intersection(const Mesh &a,
                                    const Mesh &b,
                                    Vector<Vector<float3>> &r_polylines,
                                    std::string &r_error);
/**
 * Structure point set: detect planes (RANSAC) then resample planes/edges/corners.
 * Output PointCloud with normals. Empty normals => jet estimate.
 */
PointCloud *cgal_points_structure(Span<float3> points,
                                  Span<float3> normals,
                                  float epsilon,
                                  float attraction_factor,
                                  float ransac_epsilon,
                                  float ransac_cluster_epsilon,
                                  int min_points,
                                  std::string &r_error);

/* Catalog batch 24. */
/**
 * Refine mesh where vertex scalar field crosses \a isovalue.
 * Optional \a r_isoline_edges (Edge domain) marks isoline edges when non-null.
 */
Mesh *cgal_mesh_refine_at_isolevel(const Mesh &mesh,
                                   Span<float> vertex_values,
                                   float isovalue,
                                   Array<bool> *r_isoline_edges,
                                   std::string &r_error);
/**
 * Exact multi-source geodesic distance per vertex (Surface_mesh_shortest_path).
 * \a source_mask size == verts_num (true = source). Unreachable = -1.
 */
bool cgal_mesh_exact_geodesic_distances(const Mesh &mesh,
                                        Span<bool> source_mask,
                                        MutableSpan<float> r_distance,
                                        std::string &r_error);
/**
 * Skin surface from point balls. Empty radii => constant \a default_radius.
 */
Mesh *cgal_points_skin_surface(Span<float3> points,
                               Span<float> radii,
                               float default_radius,
                               float shrink_factor,
                               int subdivisions,
                               std::string &r_error);

/* Catalog batch 25 — new only. */
/** Union of balls from centers + radii (no shrink). */
Mesh *cgal_points_union_of_balls(Span<float3> points,
                                 Span<float> radii,
                                 float default_radius,
                                 int subdivisions,
                                 std::string &r_error);
/** Min enclosing sphere of input balls; UV sphere mesh + center/radius in attributes if needed. */
Mesh *cgal_points_min_sphere_of_spheres(Span<float3> points,
                                        Span<float> radii,
                                        float default_radius,
                                        int segments,
                                        float3 &r_center,
                                        float &r_radius,
                                        std::string &r_error);

/* Catalog batch 26 — new only. */
/** Variational Shape Approximation simplified mesh; r_proxy_count filled when ok. */
Mesh *cgal_mesh_vsa_approximate(const Mesh &mesh,
                                int max_proxies,
                                int iterations,
                                int seeding_method,
                                int &r_proxy_count,
                                std::string &r_error);
/** Min enclosing circle on plane projection (0=XY,1=XZ,2=YZ). */
Mesh *cgal_points_min_circle_2(Span<float3> points,
                               int plane,
                               int segments,
                               float3 &r_center,
                               float &r_radius,
                               std::string &r_error);
/** Min-area oriented rectangle on plane projection. */
Mesh *cgal_points_min_rectangle_2(Span<float3> points,
                                  int plane,
                                  float3 &r_center,
                                  float &r_radius,
                                  std::string &r_error);

/* Catalog batch 27 — new only. */
Mesh *cgal_points_min_ellipse_2(Span<float3> points,
                                int plane,
                                int segments,
                                float3 &r_center,
                                float &r_radius,
                                std::string &r_error);
Mesh *cgal_points_min_parallelogram_2(Span<float3> points,
                                      int plane,
                                      float3 &r_center,
                                      float &r_radius,
                                      std::string &r_error);
Mesh *cgal_points_convex_hull_2(Span<float3> points, int plane, std::string &r_error);
bool cgal_mesh_plane_slice(const Mesh &mesh,
                           float3 origin,
                           float3 normal,
                           Vector<Vector<float3>> &r_polylines,
                           std::string &r_error);
/**
 * VCM feature edges. Writes Point float "Feature" (0/1), "Feature Ratio",
 * and ".selection" (same as Feature). Radii <= 0 auto from average spacing.
 * \a r_feature_count set to number of feature points.
 */
PointCloud *cgal_points_vcm_feature_edges(Span<float3> points,
                                          float offset_radius,
                                          float convolution_radius,
                                          float threshold,
                                          bool only_features,
                                          const PointCloud *src_pc,
                                          const Mesh *src_mesh,
                                          int &r_feature_count,
                                          std::string &r_error);
Mesh *cgal_points_delaunay_2(Span<float3> points, int plane, std::string &r_error);
Mesh *cgal_points_alpha_shape_2(Span<float3> points,
                                int plane,
                                float alpha,
                                bool use_optimal_alpha,
                                float &r_alpha_used,
                                std::string &r_error);
Mesh *cgal_points_voronoi_2(Span<float3> points,
                            int plane,
                            float clip_margin,
                            std::string &r_error);
Mesh *cgal_mesh_straight_skeleton_2(const Mesh &mesh, std::string &r_error);
/**
 * Faces: straight-skeleton solid offset (signed: +expand / −shrink).
 * Edges only (no faces): bilateral strip; cyclic wire → hollow frame both sides.
 * \a mode reserved (ignored).
 */
Mesh *cgal_mesh_polygon_offset_2(const Mesh &mesh,
                                 float offset_distance,
                                 int mode,
                                 std::string &r_error);
/** 2D medial-axis / centerline (interior straight-skeleton arcs) as wire mesh. */
Mesh *cgal_mesh_medial_axis_2(const Mesh &mesh, std::string &r_error);

/* Catalog batch 28 — new only. */
Mesh *cgal_points_min_annulus_2(Span<float3> points,
                                int plane,
                                int segments,
                                float3 &r_center,
                                float &r_outer_radius,
                                float &r_inner_radius,
                                std::string &r_error);
Mesh *cgal_mesh_convex_partition_2(const Mesh &mesh, int method, std::string &r_error);
Mesh *cgal_mesh_y_monotone_partition_2(const Mesh &mesh, std::string &r_error);
Mesh *cgal_points_max_area_k_gon_2(Span<float3> points, int plane, int k, std::string &r_error);
Mesh *cgal_mesh_polyline_simplify_2(const Mesh &mesh, float cost_threshold, std::string &r_error);
Mesh *cgal_points_width_3(Span<float3> points,
                          float plane_scale,
                          float &r_width,
                          float3 &r_center,
                          std::string &r_error);
Mesh *cgal_mesh_minkowski_sum_2(const Mesh &mesh_a, const Mesh &mesh_b, std::string &r_error);
Mesh *cgal_mesh_extrude_skeleton(const Mesh &mesh,
                                 float height,
                                 bool outward,
                                 std::string &r_error);
Mesh *cgal_mesh_polygon_fill_2(const Mesh &mesh, std::string &r_error);
bool cgal_mesh_do_intersect_2(const Mesh &mesh_a, const Mesh &mesh_b, std::string &r_error);

/* Catalog batch 29 — new only. */
Mesh *cgal_mesh_boolean_ops_2(const Mesh &mesh_a, const Mesh *mesh_b, int mode, std::string &r_error);
Mesh *cgal_points_largest_empty_iso_rectangle_2(Span<float3> points,
                                                int plane,
                                                float3 &r_center,
                                                float &r_area_sqrt,
                                                std::string &r_error);
Mesh *cgal_mesh_polygon_repair_2(const Mesh &mesh, std::string &r_error);
PointCloud *cgal_points_monge_jet_fit_pc(Span<float3> points,
                                         int knn,
                                         int degree_fitting,
                                         int degree_monge,
                                         const PointCloud *src_pc,
                                         const Mesh *src_mesh,
                                         std::string &r_error);

/* Catalog batch 30 — new only. */
Mesh *cgal_mesh_convex_decomposition_3(const Mesh &mesh, int &r_piece_count, std::string &r_error);
/** UE-style approximate convex hulls. Each mesh is one convex hull. */
Vector<Mesh *> cgal_mesh_convex_hulls_3(const Mesh &mesh, int max_hulls, std::string &r_error);
Vector<Mesh *> cgal_mesh_convex_hulls_2(const Mesh &mesh, int max_hulls, std::string &r_error);
Mesh *cgal_mesh_laplace_deform(const Mesh &mesh,
                               Span<uint8_t> roi_mask,
                               Span<uint8_t> control_mask,
                               Span<float3> target_xyz,
                               float weight,
                               std::string &r_error);
Mesh *cgal_mesh_surface_delaunay_remesh(const Mesh &mesh,
                                        float facet_size,
                                        float facet_angle,
                                        float facet_distance,
                                        float features_angle_bound,
                                        bool protect_constraints,
                                        std::string &r_error);

/* Catalog batch 31 — new only. */
Mesh *cgal_points_otr_reconstruct_2(Span<float3> points,
                                    int plane,
                                    float keep_percent,
                                    int relocation,
                                    std::string &r_error);
Mesh *cgal_points_regular_triangulation_2(Span<float3> points,
                                          Span<float> radii,
                                          int plane,
                                          std::string &r_error);
Mesh *cgal_points_power_diagram_2(Span<float3> points,
                                  Span<float> radii,
                                  int plane,
                                  float clip_margin,
                                  std::string &r_error);
Mesh *cgal_points_voronoi_3(Span<float3> points, float clip_margin, std::string &r_error);
Mesh *cgal_mesh_visibility_2(const Mesh &mesh, const float3 &query, std::string &r_error);
Mesh *cgal_mesh_refine_2(const Mesh &mesh,
                         float max_edge,
                         float shape_bound,
                         int lloyd_iters,
                         std::string &r_error);
Mesh *cgal_points_alpha_complex_3(Span<float3> points,
                                  float alpha,
                                  bool use_optimal_alpha,
                                  bool separate_tets,
                                  float &r_alpha_used,
                                  int &r_tet_count,
                                  std::string &r_error);
Mesh *cgal_mesh_minkowski_sum_3_convex(const Mesh &mesh_a,
                                       const Mesh &mesh_b,
                                       std::string &r_error);

/* Catalog batch 32 — new only. */
Mesh *cgal_points_regular_triangulation_3(Span<float3> points,
                                          Span<float> radii,
                                          bool separate_tets,
                                          int &r_tet_count,
                                          std::string &r_error);
Mesh *cgal_points_power_diagram_3(Span<float3> points,
                                  Span<float> radii,
                                  float clip_margin,
                                  std::string &r_error);
Mesh *cgal_points_largest_empty_circle_2(Span<float3> points,
                                         int plane,
                                         int segments,
                                         float3 &r_center,
                                         float &r_radius,
                                         std::string &r_error);
Mesh *cgal_mesh_largest_inscribed_circle_2(const Mesh &mesh,
                                           int segments,
                                           float3 &r_center,
                                           float &r_radius,
                                           std::string &r_error);
Mesh *cgal_points_min_width_2(Span<float3> points,
                              int plane,
                              float plane_scale,
                              float &r_width,
                              float3 &r_center,
                              std::string &r_error);
Mesh *cgal_points_periodic_delaunay_2(Span<float3> points,
                                      int plane,
                                      float domain_x,
                                      float domain_y,
                                      std::string &r_error);
Mesh *cgal_mesh_constrained_voronoi_2(const Mesh &mesh, std::string &r_error);

/* Catalog batch 33 — new only. */
Mesh *cgal_mesh_arrangement_2(const Mesh &mesh, std::string &r_error);
Mesh *cgal_points_delaunay_on_sphere(Span<float3> points, std::string &r_error);
Mesh *cgal_points_periodic_delaunay_3(Span<float3> points,
                                      float domain_x,
                                      float domain_y,
                                      float domain_z,
                                      bool separate_tets,
                                      int &r_tet_count,
                                      std::string &r_error);

/* Catalog batch 34 — new only. */
bool cgal_mesh_do_intersect_3(const Mesh &mesh_a, const Mesh &mesh_b, std::string &r_error);
Mesh *cgal_mesh_split_by_mesh(const Mesh &mesh, const Mesh &cutter, int &r_piece_count, std::string &r_error);
/**
 * Sibson natural-neighbor interpolator (XY Delaunay). Build once, query many times.
 * Queries are read-only and safe to run from multiple threads after #build.
 */
class CgalNaturalNeighbor2 {
 public:
  CgalNaturalNeighbor2() = default;
  ~CgalNaturalNeighbor2();
  CgalNaturalNeighbor2(CgalNaturalNeighbor2 &&other) noexcept;
  CgalNaturalNeighbor2 &operator=(CgalNaturalNeighbor2 &&other) noexcept;
  CgalNaturalNeighbor2(const CgalNaturalNeighbor2 &) = delete;
  CgalNaturalNeighbor2 &operator=(const CgalNaturalNeighbor2 &) = delete;

  bool build(Span<float3> sites, std::string &r_error);
  bool is_valid() const;
  int site_count() const;

  void query_many(Span<float3> query,
                  std::vector<int> &r_offsets,
                  std::vector<int> &r_site_index,
                  std::vector<float> &r_weight,
                  MutableSpan<bool> r_valid) const;

 private:
  void *impl_ = nullptr;
};
/**
 * Laplace natural-neighbor interpolator (3D Delaunay). Build once, query many times.
 * Queries are read-only and safe to run from multiple threads after #build.
 */
class CgalNaturalNeighbor3 {
 public:
  CgalNaturalNeighbor3() = default;
  ~CgalNaturalNeighbor3();
  CgalNaturalNeighbor3(CgalNaturalNeighbor3 &&other) noexcept;
  CgalNaturalNeighbor3 &operator=(CgalNaturalNeighbor3 &&other) noexcept;
  CgalNaturalNeighbor3(const CgalNaturalNeighbor3 &) = delete;
  CgalNaturalNeighbor3 &operator=(const CgalNaturalNeighbor3 &) = delete;

  bool build(Span<float3> sites, std::string &r_error);
  bool is_valid() const;
  int site_count() const;

  void query_many(Span<float3> query,
                  std::vector<int> &r_offsets,
                  std::vector<int> &r_site_index,
                  std::vector<float> &r_weight,
                  MutableSpan<bool> r_valid) const;

 private:
  void *impl_ = nullptr;
};
Mesh *cgal_points_proximity_graph_2(Span<float3> points, int mode, std::string &r_error);
Mesh *cgal_mesh_fill_polyline(const Mesh &mesh, std::string &r_error);
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
                                   std::string &r_error);

/* Catalog batch 35 — new only. */
Mesh *cgal_mesh_halfspace_intersection_3(const Mesh &mesh, std::string &r_error);
bool cgal_points_lloyd_relax(const Mesh &surface,
                             Span<float3> sites,
                             int max_iters,
                             float convergence,
                             MutableSpan<float3> r_positions,
                             std::string &r_error);
Mesh *cgal_mesh_conforming_delaunay_2(const Mesh &mesh, int mode, std::string &r_error);
Mesh *cgal_points_max_perimeter_k_gon_2(Span<float3> points, int k, std::string &r_error);
Mesh *cgal_mesh_constrained_delaunay_2(const Mesh &mesh, int fill_rule, std::string &r_error);

/* Catalog batch 36 — new only. */
Mesh *cgal_mesh_exterior_skeleton_2(const Mesh &mesh, float max_offset, std::string &r_error);
Mesh *cgal_points_voronoi_on_sphere(Span<float3> points, std::string &r_error);

/* Catalog batch 37 — new only. */
Mesh *cgal_points_radius_graph_3(Span<float3> points, float radius, std::string &r_error);
Mesh *cgal_points_knn_graph_3(Span<float3> points, int k, std::string &r_error);
bool cgal_points_natural_neighbor_3(Span<float3> sites,
                                    Span<float> site_values,
                                    Span<float3> queries,
                                    MutableSpan<float> r_values,
                                    std::string &r_error);
bool cgal_points_sibson_gradient_2(Span<float3> points,
                                   Span<float> values,
                                   MutableSpan<float3> r_gradients,
                                   std::string &r_error);

/* Catalog batch 38 — 10 new nodes. */
Mesh *cgal_points_octree_3(
    Span<float3> points, int max_depth, int bucket, bool solid, std::string &r_error);
Mesh *cgal_points_hilbert_path_3(Span<float3> points, std::string &r_error);
Mesh *cgal_mesh_shortest_cycle(const Mesh &mesh, std::string &r_error);
Mesh *cgal_points_periodic_voronoi_2(Span<float3> points,
                                     float domain_x,
                                     float domain_y,
                                     std::string &r_error);
Mesh *cgal_points_periodic_voronoi_3(Span<float3> points,
                                     float domain_x,
                                     float domain_y,
                                     float domain_z,
                                     std::string &r_error);
Mesh *cgal_points_gabriel_graph_3(Span<float3> points, std::string &r_error);
Mesh *cgal_points_euclidean_mst_3(Span<float3> points, std::string &r_error);
Mesh *cgal_mesh_euclidean_mst_3(const Mesh &mesh, std::string &r_error);
Mesh *cgal_points_beta_skeleton_2(Span<float3> points, float beta, std::string &r_error);
Mesh *cgal_points_convex_layers_2(Span<float3> points, std::string &r_error);

/* Catalog batch 39 — new only. */
Mesh *cgal_points_farthest_voronoi_2(Span<float3> points, float clip_margin, std::string &r_error);
Mesh *cgal_points_convex_layers_3(Span<float3> points, std::string &r_error);
Mesh *cgal_mesh_overlay_2(const Mesh &mesh_a, const Mesh &mesh_b, std::string &r_error);
Mesh *cgal_mesh_polygon_kernel_2(const Mesh &mesh, std::string &r_error);
bool cgal_mesh_self_intersection_curves(const Mesh &mesh,
                                        Vector<Vector<float3>> &r_polylines,
                                        std::string &r_error);
Mesh *cgal_points_crust_2(Span<float3> points, std::string &r_error);
Mesh *cgal_mesh_geodesic_voronoi(const Mesh &mesh, Span<bool> sources, std::string &r_error);
Mesh *cgal_points_isosurface_3(Span<float3> points,
                               Span<float> values,
                               float isolevel,
                               std::string &r_error);

/* Catalog batch 40 — new only. */
Mesh *cgal_mesh_line_arrangement_2(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_vertical_decomposition_2(const Mesh &mesh, std::string &r_error);
Mesh *cgal_points_circle_arrangement_2(Span<float3> points,
                                       Span<float> radii,
                                       int segments,
                                       std::string &r_error);
Mesh *cgal_points_crust_3(Span<float3> points, std::string &r_error);
Mesh *cgal_points_clipped_voronoi_3(Span<float3> points, float clip_margin, std::string &r_error);
Mesh *cgal_mesh_complement_2(const Mesh &mesh, float padding, std::string &r_error);
Mesh *cgal_points_simple_polygon_2(Span<float3> points, std::string &r_error);

/* Catalog batch 41 — new only. */
Mesh *cgal_mesh_split_crossings_2(const Mesh &mesh, std::string &r_error);
PointCloud *cgal_mesh_crossing_points_2(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_pullout_directions_2(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_ssab_partition_2(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_snap_borders(const Mesh &mesh, float tolerance, std::string &r_error);
Mesh *cgal_mesh_autorefine_clean(const Mesh &mesh, std::string &r_error);
Mesh *cgal_points_random_polygon_2(int count, float size, int seed, std::string &r_error);
Mesh *cgal_points_random_convex_set_2(int count, float size, int seed, std::string &r_error);

/* Catalog batch 42 — new only. */
Mesh *cgal_points_largest_empty_sphere_3(Span<float3> points,
                                         int segments,
                                         float3 &r_center,
                                         float &r_radius,
                                         std::string &r_error);
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
                                    std::string &r_error);
Mesh *cgal_points_voronoi_slice_3(Span<float3> points,
                                  const float3 &origin,
                                  const float3 &normal,
                                  float clip_margin,
                                  std::string &r_error);
Mesh *cgal_mesh_convex_offset_3(const Mesh &mesh, float offset, std::string &r_error);
Mesh *cgal_mesh_volume_components(const Mesh &mesh, int &r_volume_count, std::string &r_error);
Mesh *cgal_mesh_inscribed_sphere_3(const Mesh &mesh,
                                   int segments,
                                   float3 &r_center,
                                   float &r_radius,
                                   std::string &r_error);
Mesh *cgal_points_min_cylinder_3(Span<float3> points,
                                 int segments,
                                 float3 &r_center,
                                 float &r_radius,
                                 std::string &r_error);

Mesh *cgal_mesh_fair_hole_fill(const Mesh &mesh, int continuity, std::string &r_error);
Mesh *cgal_mesh_simplify_polyline_3(const Mesh &mesh,
                                    float max_distance,
                                    bool iterative,
                                    std::string &r_error);
bool cgal_simplify_polyline_xyz(Span<float3> points,
                                bool closed,
                                float max_distance,
                                bool iterative,
                                Vector<float3> &r_out,
                                Vector<int> &r_src,
                                std::string &r_error);
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
                                std::string &r_error);
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
                                        std::string &r_error);
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
                                          std::string &r_error);
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
                                        std::string &r_error);
Mesh *cgal_points_line_region_growing(Span<float3> points,
                                      Span<float3> normals,
                                      float neighbor_radius,
                                      float max_distance,
                                      int min_region_size,
                                      MutableSpan<int> out_region,
                                      int &r_count,
                                      std::string &r_error);

Mesh *cgal_points_min_annulus_3(Span<float3> points,
                                int segments,
                                float3 &r_center,
                                float &r_outer,
                                float &r_inner,
                                std::string &r_error);
Mesh *cgal_mesh_collapse_short_edges(const Mesh &mesh,
                                     float max_length,
                                     std::string &r_error);

/* Catalog batch 45 — new only. */
bool cgal_regularize_open_polyline_xy(Span<float3> points,
                                      bool closed,
                                      float max_offset,
                                      float min_length,
                                      Vector<float3> &r_out,
                                      std::string &r_error);
Mesh *cgal_mesh_constrained_simplify(const Mesh &mesh,
                                     float keep_ratio,
                                     bool keep_boundary,
                                     Span<uint8_t> preserve,
                                     std::string &r_error);

/* Catalog batch 47 — new only. */
bool cgal_mesh_cone_slice(const Mesh &mesh,
                          float3 apex,
                          float3 axis,
                          float radius,
                          float height,
                          int segments,
                          Vector<Vector<float3>> &r_polylines,
                          std::string &r_error);
Mesh *cgal_mesh_clip_box(const Mesh &mesh,
                         float3 center,
                         float3 size,
                         bool clip_volume,
                         std::string &r_error);
Mesh *cgal_mesh_clip_by_mesh(const Mesh &mesh,
                             const Mesh &clipper,
                             bool clip_volume,
                             std::string &r_error);

/* Catalog batch 48 — new only. */
Mesh *cgal_points_rectangular_p_center_2(Span<float3> points,
                                         int p,
                                         float &r_radius,
                                         std::string &r_error);
Mesh *cgal_mesh_polyline_hull_2(const Mesh &mesh, std::string &r_error);

/* Catalog batch 49 — new only. */
bool cgal_mesh_curve_intersect(const Mesh &mesh,
                               Span<float3> curve_xyz,
                               Span<int> curve_offsets,
                               Vector<Vector<float3>> &r_segments,
                               Vector<float3> &r_points,
                               std::string &r_error);
bool cgal_mesh_geodesic_isolines(const Mesh &mesh,
                                 Span<bool> sources,
                                 int count,
                                 float max_distance,
                                 Vector<Vector<float3>> &r_polylines,
                                 std::string &r_error);
Mesh *cgal_mesh_overlap_faces(const Mesh &mesh, const Mesh &other, std::string &r_error);
bool cgal_mesh_contour_stack(const Mesh &mesh,
                             float3 origin,
                             float3 direction,
                             int count,
                             float spacing,
                             Vector<Vector<float3>> &r_polylines,
                             std::string &r_error);

/* Catalog batch 50 — new only. */
Mesh *cgal_points_alpha_edges_3(Span<float3> points,
                                float alpha,
                                bool optimal,
                                int solid_components,
                                float &r_alpha_used,
                                std::string &r_error);
Mesh *cgal_points_delaunay_edges_3(Span<float3> points, std::string &r_error);
bool cgal_mesh_edge_path(const Mesh &mesh,
                         Span<bool> sources,
                         Span<bool> targets,
                         Vector<float3> &r_polyline,
                         std::string &r_error);
bool cgal_mesh_curvature_isolines(const Mesh &mesh,
                                  int mode,
                                  int count,
                                  Vector<Vector<float3>> &r_polylines,
                                  std::string &r_error);

/* Catalog batch 51 — new only. */
Mesh *cgal_mesh_offset_sdf(const Mesh &mesh,
                           float offset,
                           int resolution,
                           bool signed_distance,
                           std::string &r_error);
Mesh *cgal_mesh_cdt_hole_fill(const Mesh &mesh, std::string &r_error);
Mesh *cgal_points_bisector_surface(Span<float3> a, Span<float3> b, std::string &r_error);
Mesh *cgal_mesh_projected_outline(const Mesh &mesh, float3 direction, std::string &r_error);

/* Catalog batch 52 — new only. */
Mesh *cgal_mesh_visibility_graph_2(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_terrain_tin(const Mesh &mesh, int fill_rule, std::string &r_error);
Mesh *cgal_points_min_circle_3(Span<float3> points,
                               int segments,
                               float3 &r_center,
                               float3 &r_normal,
                               float &r_radius,
                               std::string &r_error);

/* Catalog batch 53 — new only. */
Mesh *cgal_mesh_interior_tets(const Mesh &mesh, std::string &r_error);
Mesh *cgal_points_surface_delaunay_graph(Span<float3> points, int knn, std::string &r_error);
Mesh *cgal_mesh_split_charts(const Mesh &mesh, float angle_deg, std::string &r_error);
Mesh *cgal_mesh_restricted_voronoi(const Mesh &mesh, Span<float3> sites, std::string &r_error);

/* Catalog batch 54 — new only. */
Mesh *cgal_points_walk_tets(Span<float3> points, float3 start, float3 end, std::string &r_error);
Mesh *cgal_mesh_skeleton_spokes(const Mesh &mesh, std::string &r_error);
bool cgal_mesh_radial_slices(const Mesh &mesh,
                             float3 origin,
                             float3 axis,
                             int count,
                             Vector<Vector<float3>> &r_polylines,
                             std::string &r_error);
Mesh *cgal_mesh_intersection_band(const Mesh &a, const Mesh &b, std::string &r_error);

/* Catalog batch 55. */
bool cgal_points_sphere_intersect(Span<float3> points,
                                  Span<float> radii,
                                  float uniform_radius,
                                  int segments,
                                  Vector<Vector<float3>> &r_polylines,
                                  std::string &r_error);
Mesh *cgal_mesh_sphere_arrangement(const Mesh &mesh, float3 origin, std::string &r_error);
Mesh *cgal_mesh_graphcut_segment(const Mesh &mesh,
                                 float angle_deg,
                                 int min_size,
                                 std::string &r_error);
PointCloud *cgal_points_classification_features(Span<float3> points,
                                                int knn,
                                                const PointCloud *src_pc,
                                                const Mesh *src_mesh,
                                                std::string &r_error);
PointCloud *cgal_points_register_icp(Span<float3> src,
                                     Span<float3> tgt,
                                     int iterations,
                                     const PointCloud *src_pc,
                                     const Mesh *src_mesh,
                                     std::string &r_error);
PointCloud *cgal_points_register_4pcs(Span<float3> src,
                                      Span<float3> tgt,
                                      int samples,
                                      int icp_iterations,
                                      const PointCloud *src_pc,
                                      const Mesh *src_mesh,
                                      std::string &r_error);
Mesh *cgal_points_alpha_wrap_2(Span<float3> points,
                               const Mesh *segments,
                               float alpha,
                               float offset,
                               std::string &r_error);
Mesh *cgal_mesh_cage_deform_3(const Mesh &cage_rest,
                              const Mesh &cage_pose,
                              const Mesh &interior,
                              std::string &r_error);
Mesh *cgal_dual_contour_grid(Span<float> sdf,
                             int nx,
                             int ny,
                             int nz,
                             float3 origin,
                             float3 voxel,
                             float isovalue,
                             std::string &r_error);
Mesh *cgal_mesh_tet_remesh(const Mesh &mesh, float target_edge, std::string &r_error);
Mesh *cgal_mesh_constrained_delaunay_3(const Mesh &mesh, std::string &r_error);
Mesh *cgal_mesh_approximate_convex_decomposition(const Mesh &mesh,
                                                 int count,
                                                 int resolution,
                                                 std::string &r_error);
Mesh *cgal_marching_cubes_grid(Span<float> sdf,
                               int nx,
                               int ny,
                               int nz,
                               float3 origin,
                               float3 voxel,
                               float isovalue,
                               bool tcmc,
                               std::string &r_error);
PointCloud *cgal_points_extreme_point_3(Span<float3> points,
                                        float3 direction,
                                        std::string &r_error);
PointCloud *cgal_mesh_barycentric_3(const Mesh &cage,
                                    Span<float3> query,
                                    int method,
                                    PointCloud **r_weights,
                                    std::string &r_error);

}  // namespace geometry
}  // namespace blender

