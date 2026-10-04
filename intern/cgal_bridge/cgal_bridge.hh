/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace blender::cgal_bridge {

/**
 * Polygon mesh buffers (triangles or n-gons).
 * face_offsets empty => every face is a triangle (corner_verts.size() == faces*3).
 * face_offsets size == faces_num+1 => corner_verts[face_offsets[i] .. face_offsets[i+1]).
 *
 * Topology maps for attribute interpolation (not nearest-surface sampling):
 * - face_src[i]: original face index (-1 unknown / generated).
 * - face_src_mesh[i]: 0 or 1 for boolean dual inputs (empty => all from mesh 0).
 * - vert_src0[i]: primary original vertex (-1 = generated).
 * - vert_src1[i]: secondary original vertex for edge lerp (-1 = pure copy of src0).
 * - vert_factor[i]: blend weight toward src1 (0 = src0, 1 = src1).
 */
struct MeshResult {
  std::vector<float> positions; /* xyz * n */
  std::vector<int> corner_verts;
  std::vector<int> face_offsets; /* optional; empty = all triangles */
  std::string error;
  bool ok = false;
  double alpha_used = 0.0;
  float center[3] = {0, 0, 0};
  float radius = 0.0f;

  /* Optional topology maps (size == faces_num / verts_num when present). */
  std::vector<int> face_src;
  std::vector<int8_t> face_src_mesh;
  std::vector<int> vert_src0;
  std::vector<int> vert_src1;
  std::vector<float> vert_factor;

  /**
   * When true, only positions are algorithm output; face/corner topology matches
   * the input mesh exactly (fair / smooth). Callers should copy the source mesh
   * and overwrite positions so all attributes are preserved.
   */
  bool positions_only = false;

  /**
   * Intersection / corefine seam edges as pairs of output vertex indices
   * (order independent). Used to build an Edge-domain selection after
   * Blender recalculates edges.
   */
  std::vector<int> seam_vert_a;
  std::vector<int> seam_vert_b;

  /** Optional per-face scalar (e.g. SDF); size == faces_num when used. */
  std::vector<float> face_scalar;
  /**
   * Optional per-face integer tag (e.g. SDF segment id).
   * Independent of face_src parent map so attributes can still transfer.
   */
  std::vector<int> face_tag;

  int verts_num() const
  {
    return int(positions.size() / 3);
  }
  int faces_num() const
  {
    if (!face_offsets.empty()) {
      return int(face_offsets.size()) - 1;
    }
    return int(corner_verts.size() / 3);
  }
  int corners_num() const
  {
    return int(corner_verts.size());
  }
  bool is_all_triangles() const
  {
    return face_offsets.empty() || (faces_num() > 0 && corners_num() == faces_num() * 3);
  }
  bool has_face_map() const
  {
    return int(face_src.size()) == faces_num() && faces_num() > 0;
  }
  bool has_vert_map() const
  {
    return int(vert_src0.size()) == verts_num() && verts_num() > 0;
  }
};

/** Alias kept for call-site churn reduction. */
using TriangleMeshResult = MeshResult;

struct MeshIn {
  const float *positions = nullptr;
  int verts_num = 0;
  const int *corner_verts = nullptr;
  int corners_num = 0;
  /** nullptr => treat as pure triangles (faces_num * 3 corners). */
  const int *face_offsets = nullptr;
  int faces_num = 0;
  /**
   * Optional loose edges (wire loops). Each edge is (edge_v0[i], edge_v1[i]).
   * Used when faces_num==0 (pure edge mesh) for 2D polygon algorithms.
   * Prefer owned_edge_* storage so MeshIn stays self-contained under copies.
   */
  const int *edge_v0 = nullptr;
  const int *edge_v1 = nullptr;
  int edges_num = 0;
  std::vector<int> owned_edge_v0;
  std::vector<int> owned_edge_v1;

  bool is_all_triangles() const
  {
    return face_offsets == nullptr || corners_num == faces_num * 3;
  }

  void set_owned_edges(std::vector<int> v0, std::vector<int> v1)
  {
    owned_edge_v0 = std::move(v0);
    owned_edge_v1 = std::move(v1);
    edges_num = int(owned_edge_v0.size());
    edge_v0 = owned_edge_v0.empty() ? nullptr : owned_edge_v0.data();
    edge_v1 = owned_edge_v1.empty() ? nullptr : owned_edge_v1.data();
  }
};

/* --- Points --- */
MeshResult convex_hull_3(const float *positions, int points_num);
MeshResult alpha_shape_3(const float *positions,
                         int points_num,
                         double alpha,
                         bool use_optimal_alpha,
                         int solid_components);
MeshResult alpha_wrap_points(const float *positions,
                             int points_num,
                             double alpha,
                             double offset);
MeshResult advancing_front_surface(const float *positions,
                                   int points_num,
                                   double radius_ratio_bound,
                                   double beta);
/**
 * 3D Delaunay tetrahedralization as triangle mesh faces.
 * \a separate_tets: if true, each tet gets its own 4 vertices (no shared verts),
 * so tets are independent shells; if false, verts are welded (shared indices).
 */
MeshResult delaunay_3d(const float *positions, int points_num, bool separate_tets = false);
MeshResult min_sphere(const float *positions, int points_num, int sphere_segments);
MeshResult optimal_bbox(const float *positions, int points_num);

/* --- Mesh (prefer n-gon I/O; algorithms that require tris triangulate internally) --- */
MeshResult mesh_simplify(const MeshIn &mesh, double stop_ratio);
MeshResult mesh_isotropic_remesh(const MeshIn &mesh, double target_edge_length, int iterations);
MeshResult mesh_smooth_shape(const MeshIn &mesh, double time, int iterations, bool do_scale);
MeshResult mesh_boolean(const MeshIn &a, const MeshIn &b, int op);
MeshResult mesh_hole_fill(const MeshIn &mesh);
MeshResult mesh_fair(const MeshIn &mesh, int continuity);
/**
 * Smooth densification via isotropic remeshing (target edge = mean_edge / density).
 * CGAL PMP::refine mid-edge split is intentionally not used (spikes on curved surfaces).
 */
MeshResult mesh_refine(const MeshIn &mesh, double density_factor);
MeshResult mesh_clip_plane(const MeshIn &mesh,
                           float px,
                           float py,
                           float pz,
                           float nx,
                           float ny,
                           float nz);
/** mode: 0=Loop, 1=CatmullClark, 2=DooSabin, 3=Sqrt3. Loop/Sqrt3 need triangles. */
MeshResult mesh_subdivision(const MeshIn &mesh, int mode, int steps);
MeshResult mesh_repair(const MeshIn &mesh);
MeshResult mesh_keep_largest_component(const MeshIn &mesh);
MeshResult mesh_alpha_wrap(const MeshIn &mesh, double alpha, double offset);
MeshResult mesh_corefine(const MeshIn &a, const MeshIn &b);

/* --- Batch mesh ops (second wave) --- */
MeshResult mesh_detect_features(const MeshIn &mesh, double angle_deg);
MeshResult mesh_angle_area_smooth(const MeshIn &mesh, int iterations, bool use_safety);
MeshResult mesh_tangential_relaxation(const MeshIn &mesh, int iterations);
/**
 * Extrude open mesh into a prism.
 * If along_normals: offset top along vertex normals by dist.
 * Else: constant direction (dx,dy,dz) scaled by dist (direction need not be unit).
 */
MeshResult mesh_extrude(const MeshIn &mesh,
                        float dist,
                        float dx,
                        float dy,
                        float dz,
                        bool along_normals);
MeshResult mesh_remesh_planar_patches(const MeshIn &mesh, double cos_angle, double max_dist);
MeshResult mesh_random_perturbation(const MeshIn &mesh, double max_move, bool project);
MeshResult mesh_triangulate(const MeshIn &mesh);
MeshResult mesh_orient_outward(const MeshIn &mesh);
bool mesh_does_self_intersect(const MeshIn &mesh, std::string &error);
MeshResult mesh_repair_self_intersections(const MeshIn &mesh);
MeshResult mesh_split_long_edges(const MeshIn &mesh, double max_length);
MeshResult mesh_connected_component_keep(const MeshIn &mesh, int component_index);
double mesh_volume(const MeshIn &mesh, std::string &error);
double mesh_area(const MeshIn &mesh, std::string &error);
MeshResult mesh_merge_border_vertices(const MeshIn &mesh, double dist);

/* --- Batch point ops --- */
double points_average_spacing(const float *positions, int n, int k, std::string &error);
bool points_jet_smooth(const float *in_xyz,
                       int n,
                       int k,
                       int iter,
                       float *out_xyz,
                       std::string &error);
bool points_bilateral_smooth(const float *in_xyz,
                             int n,
                             int k,
                             int iter,
                             double sharpness,
                             float *out_xyz,
                             std::string &error);
bool points_remove_outliers(const float *in_xyz,
                            int n,
                            int k,
                            double percent,
                            std::vector<float> &out_xyz,
                            std::string &error);
bool points_grid_simplify(const float *in_xyz,
                          int n,
                          double cell_size,
                          std::vector<float> &out_xyz,
                          std::string &error);
bool points_random_simplify(const float *in_xyz,
                            int n,
                            double percent,
                            std::vector<float> &out_xyz,
                            std::string &error);
bool points_estimate_normals(const float *in_xyz,
                             int n,
                             int k,
                             float *out_nxyz,
                             std::string &error);
MeshResult points_poisson_reconstruct(const float *positions,
                                      const float *normals,
                                      int n,
                                      double spacing);
MeshResult points_scale_space_reconstruct(const float *positions, int n, int iterations);
/** WLOP simplify + regularize; select_percentage in (0,100], radius 0 = auto. */
bool points_wlop(const float *in_xyz,
                 int n,
                 double select_percentage,
                 double neighbor_radius,
                 int iterations,
                 bool require_uniform,
                 std::vector<float> &out_xyz,
                 std::string &error);
/** Hierarchy simplify; cluster_size > 0, max_variation in (0, 1/3]. */
bool points_hierarchy_simplify(const float *in_xyz,
                               int n,
                               int cluster_size,
                               double max_variation,
                               std::vector<float> &out_xyz,
                               std::string &error);
/**
 * Edge-aware upsample. If normals null, estimates with jet (k neighbors).
 * number_of_output_points is total output count.
 */
bool points_edge_aware_upsample(const float *in_xyz,
                                const float *in_nxyz,
                                int n,
                                int number_of_output_points,
                                double sharpness_angle_deg,
                                double edge_sensitivity,
                                double neighbor_radius,
                                int normal_neighbors,
                                std::vector<float> &out_xyz,
                                std::vector<float> &out_nxyz,
                                std::string &error);
/** VCM normal estimation; offset_radius / convolution_radius > 0. */
bool points_vcm_estimate_normals(const float *in_xyz,
                                 int n,
                                 double offset_radius,
                                 double convolution_radius,
                                 float *out_nxyz,
                                 std::string &error);
/** For each query point: -1 outside, 0 boundary, 1 inside. */
bool mesh_side_of(const MeshIn &mesh,
                  const float *query_xyz,
                  int query_n,
                  std::vector<int8_t> &out_side,
                  std::string &error);
bool mesh_distance_to(const MeshIn &mesh,
                      const float *query_xyz,
                      int query_n,
                      std::vector<float> &out_dist,
                      std::string &error);
bool mesh_sample_points(const MeshIn &mesh,
                        int count,
                        std::vector<float> &out_xyz,
                        std::string &error);

/**
 * Wireframe result (skeleton / border): vertex positions + edge pairs.
 * Converted to a Blender mesh with verts + edges (no faces).
 */
struct WireResult {
  std::vector<float> positions;
  std::vector<int> edge_v0;
  std::vector<int> edge_v1;
  std::string error;
  bool ok = false;
};

/** Mean-curvature-flow curve skeleton (closed triangle mesh). */
WireResult mesh_skeleton(const MeshIn &mesh);
/** Extract boundary edges as a wire mesh. */
WireResult mesh_extract_border(const MeshIn &mesh);
/**
 * Heat-method geodesic distances from source vertex indices (MeshIn order).
 * out_dist size == mesh.verts_num.
 */
bool mesh_geodesic_distances(const MeshIn &mesh,
                             const std::vector<int> &source_verts,
                             std::vector<float> &out_dist,
                             std::string &error);
/**
 * SDF-based segmentation on closed triangle mesh.
 * Result mesh is triangulated; face_src[i] = segment id; face_scalar[i] = SDF.
 */
MeshResult mesh_segmentation(const MeshIn &mesh, int n_clusters, double smoothing_lambda);
/** Approximate minimum-volume enclosing ellipsoid as a UV-sphere mesh. */
MeshResult min_ellipsoid(const float *positions, int points_num, int sphere_segments);

/* --- Batch 15: analysis / utility --- */
bool mesh_is_closed(const MeshIn &mesh, std::string &error);
bool mesh_does_bound_volume(const MeshIn &mesh, std::string &error);
int mesh_component_count(const MeshIn &mesh, std::string &error);
bool mesh_centroid(const MeshIn &mesh, float out_xyz[3], std::string &error);
double mesh_border_length(const MeshIn &mesh, std::string &error);
double mesh_hausdorff_distance(const MeshIn &a, const MeshIn &b, std::string &error);
bool mesh_closest_points(const MeshIn &mesh,
                         const float *query_xyz,
                         int query_n,
                         std::vector<float> &out_xyz,
                         std::vector<float> &out_dist,
                         std::string &error);
MeshResult mesh_stitch_borders(const MeshIn &mesh);
MeshResult mesh_autorefine(const MeshIn &mesh);
MeshResult mesh_remove_degenerate(const MeshIn &mesh);
bool mesh_mean_curvature(const MeshIn &mesh, std::vector<float> &out_h, std::string &error);
bool mesh_gaussian_curvature(const MeshIn &mesh, std::vector<float> &out_k, std::string &error);
bool mesh_face_areas(const MeshIn &mesh, std::vector<float> &out_area, std::string &error);
MeshResult points_aabb(const float *positions, int n);
WireResult points_principal_axes(const float *positions, int n, float scale);
bool points_detect_planes(const float *positions,
                          int n,
                          int min_points,
                          double epsilon,
                          double normal_threshold,
                          std::vector<int> &out_plane_id,
                          std::string &error);

/* --- Batch 16: topology / fields (new nodes only) --- */
bool mesh_is_manifold(const MeshIn &mesh, std::string &error);
int mesh_euler_characteristic(const MeshIn &mesh, std::string &error);
/** Per original face: connected-component id (0-based). */
bool mesh_face_component_ids(const MeshIn &mesh, std::vector<int> &out_ids, std::string &error);
/**
 * Per-vertex principal curvatures and directions (CGAL
 * Principal_curvatures_and_directions):
 *   out_kmin/out_kmax: size verts_num
 *   out_dmin/out_dmax: xyz * verts_num (unit directions)
 */
bool mesh_principal_curvatures(const MeshIn &mesh,
                               std::vector<float> &out_kmin,
                               std::vector<float> &out_kmax,
                               std::vector<float> &out_dmin,
                               std::vector<float> &out_dmax,
                               std::string &error);
/** Per original face: shape-diameter function (closed mesh). */
bool mesh_shape_diameter(const MeshIn &mesh, std::vector<float> &out_sdf, std::string &error);
/** Per original face: 1 if any triangle of the face self-intersects. */
bool mesh_mark_self_intersect_faces(const MeshIn &mesh,
                                    std::vector<int8_t> &out_hit,
                                    std::string &error);

/* --- Batch 17: orientation / border / dihedral --- */
bool mesh_is_outward_oriented(const MeshIn &mesh, std::string &error);
MeshResult mesh_reverse_orientation(const MeshIn &mesh);
int mesh_hole_count(const MeshIn &mesh, std::string &error);
/** out_border[i] for Blender edge i given by edge_v0/v1. */
bool mesh_border_edge_flags(const MeshIn &mesh,
                            const int *edge_v0,
                            const int *edge_v1,
                            int edges_num,
                            std::vector<int8_t> &out_border,
                            std::string &error);
/** Signed dihedral angle in radians for each Blender edge (0 on border). */
bool mesh_dihedral_angles(const MeshIn &mesh,
                          const int *edge_v0,
                          const int *edge_v1,
                          int edges_num,
                          std::vector<float> &out_angle_rad,
                          std::string &error);
MeshResult mesh_duplicate_non_manifold_vertices(const MeshIn &mesh);

/* --- Batch 18: 15 new ops (points + mesh fields + path/locate) --- */
bool points_pca_estimate_normals(const float *in_xyz,
                                 int n,
                                 int k,
                                 float *out_nxyz,
                                 std::string &error);
bool points_mst_orient_normals(const float *in_xyz,
                               const float *in_nxyz,
                               int n,
                               int k,
                               float *out_nxyz,
                               std::string &error);
bool points_radial_orient_normals(const float *in_xyz,
                                  const float *in_nxyz,
                                  int n,
                                  float *out_nxyz,
                                  std::string &error);
bool points_cluster(const float *in_xyz,
                    int n,
                    double neighbor_radius,
                    std::vector<int> &out_cluster,
                    int &out_cluster_count,
                    std::string &error);
bool mesh_polyhedral_envelope_contains(const MeshIn &mesh,
                                       double epsilon,
                                       const float *query_xyz,
                                       int query_n,
                                       std::vector<int8_t> &out_inside,
                                       std::string &error);
bool mesh_vertex_normals(const MeshIn &mesh, std::vector<float> &out_nxyz, std::string &error);
bool mesh_face_normals(const MeshIn &mesh, std::vector<float> &out_nxyz, std::string &error);
/** Per-face aspect ratio = longest / shortest edge (triangles/n-gons). */
bool mesh_face_aspect_ratio(const MeshIn &mesh,
                            std::vector<float> &out_aspect,
                            std::string &error);
bool mesh_vertex_valence(const MeshIn &mesh, std::vector<int> &out_valence, std::string &error);
bool mesh_mean_edge_length(const MeshIn &mesh, std::vector<float> &out_len, std::string &error);
bool mesh_border_vertices(const MeshIn &mesh, std::vector<int8_t> &out_border, std::string &error);
bool mesh_face_quality(const MeshIn &mesh, std::vector<float> &out_quality, std::string &error);
bool mesh_region_growing(const MeshIn &mesh,
                         double max_distance,
                         double max_accepted_angle_deg,
                         int min_region_size,
                         std::vector<int> &out_region,
                         int &out_region_count,
                         std::string &error);
WireResult mesh_surface_shortest_path(const MeshIn &mesh,
                                      const float source_xyz[3],
                                      const float target_xyz[3]);
bool mesh_locate_points(const MeshIn &mesh,
                        const float *query_xyz,
                        int query_n,
                        std::vector<int> &out_face,
                        std::vector<float> &out_bary,
                        std::string &error);

/* --- Batch 19: 6 new ops (edge/face measures, points centroid/plane/scale/scanline) --- */
bool mesh_edge_lengths(const float *positions,
                       int verts_num,
                       const int *edge_verts,
                       int edges_num,
                       std::vector<float> &out_len,
                       std::string &error);
bool mesh_face_perimeters(const MeshIn &mesh, std::vector<float> &out_perim, std::string &error);
bool points_centroid(const float *in_xyz, int n, float out_xyz[3], std::string &error);
bool points_fit_plane(const float *in_xyz,
                      int n,
                      float out_center[3],
                      float out_normal[3],
                      float &out_half_extent,
                      std::string &error);
bool points_neighbor_scale(const float *in_xyz, int n, int &out_k, std::string &error);
bool points_scanline_orient_normals(const float *in_xyz,
                                    const float *in_nxyz,
                                    int n,
                                    float *out_nxyz,
                                    std::string &error);

/* --- Batch 20: 11 new ops for goal (local scales, components, fit line, etc.) --- */
bool points_local_neighbor_scales(const float *in_xyz,
                                  int n,
                                  std::vector<int> &out_k,
                                  std::string &error);
MeshResult mesh_remove_small_components(const MeshIn &mesh, int min_faces);
bool points_fit_line(const float *in_xyz,
                     int n,
                     float out_center[3],
                     float out_direction[3],
                     float &out_half_extent,
                     std::string &error);
bool points_diameter(const float *in_xyz, int n, float &out_diameter, std::string &error);
MeshResult mesh_orient_polygon_soup(const MeshIn &mesh);
bool mesh_self_intersection_count(const MeshIn &mesh, int &out_count, std::string &error);
bool points_local_density(const float *in_xyz,
                          int n,
                          int k,
                          std::vector<float> &out_density,
                          std::string &error);
bool mesh_compactness(const MeshIn &mesh, float &out_compactness, std::string &error);
bool mesh_face_component_sizes(const MeshIn &mesh,
                               std::vector<int> &out_size,
                               std::string &error);
bool mesh_face_planarity(const MeshIn &mesh, std::vector<float> &out_planarity, std::string &error);
bool mesh_is_triangle_mesh(const MeshIn &mesh, bool &out_is_triangle, std::string &error);


/* --- Batch 21: catalog high-value (ARAP deform, LSCM UV, Efficient RANSAC) --- */
/**
 * ARAP surface mesh deformation.
 * \a roi_mask / \a control_mask: per-vertex 0/1, size == verts_num.
 * \a target_xyz: per-vertex target positions (only control verts used), size verts_num*3.
 * \a algorithm: 0=ORIGINAL_ARAP, 1=SPOKES_AND_RIMS.
 * Output is positions_only (topology preserved).
 */
MeshResult mesh_arap_deform(const MeshIn &mesh,
                            const uint8_t *roi_mask,
                            const uint8_t *control_mask,
                            const float *target_xyz,
                            int algorithm,
                            int iterations,
                            double tolerance);
/**
 * UV parameterization with optional edge seams.
 * Writes per-corner UV (u,v) into \a out_corner_uv (size corners_num*2), matching
 * MeshIn corner order. Seams allow different UV at the same vertex across faces.
 *
 * \a method: 0=LSCM, 1=ARAP UV, 2=Discrete Conformal, 3=Mean Value,
 *            4=Discrete Authalic, 5=Barycentric, 6=Iterative Authalic.
 * \a energy_iterations: ARAP energy iterations (ignored by others; default 50 if <=0).
 * \a lambda: ARAP lambda (>=0; larger => more rigid). Ignored by non-ARAP methods.
 * \a edge_v0 / \a edge_v1: Blender edge endpoints (size edges_num); may be null if no seams.
 * \a seam_edge: 0/1 per edge; null = no seams (needs existing mesh boundary).
 * \a face_selected: 0/1 per face; null = all faces. Unselected corners get (0,0).
 * \a normalize: affinely map selected UV into approximately [0,1]^2.
 */
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
                                  std::string &error);
/** Alias kept for call sites. */
inline bool mesh_lscm_uv_corners(const MeshIn &mesh,
                                 const int *edge_v0,
                                 const int *edge_v1,
                                 int edges_num,
                                 const uint8_t *seam_edge,
                                 const uint8_t *face_selected,
                                 bool normalize,
                                 std::vector<float> &out_corner_uv,
                                 std::string &error)
{
  return mesh_parameterize_uv_corners(mesh,
                                      edge_v0,
                                      edge_v1,
                                      edges_num,
                                      seam_edge,
                                      face_selected,
                                      normalize,
                                      0,
                                      0,
                                      1000.0,
                                      out_corner_uv,
                                      error);
}

/**
 * Point-set region growing (least-squares planes).
 * \a out_region: per-point region id (-1 unassigned), size n.
 * \a neighbor_radius: sphere neighbor query radius (>0).
 * \a max_distance / \a max_angle_deg: plane fit tolerances.
 * \a min_region_size: drop smaller regions.
 */
bool points_region_growing_planes(const float *in_xyz,
                                  const float *in_nxyz,
                                  int n,
                                  double neighbor_radius,
                                  double max_distance,
                                  double max_angle_deg,
                                  int min_region_size,
                                  std::vector<int> &out_region,
                                  int &out_region_count,
                                  std::string &error);
/**
 * Efficient RANSAC shape detection on oriented points.
 * \a in_nxyz may be null (jet-estimate with k=24).
 * \a out_shape_id: per-point shape index (-1 unassigned), size n.
 * \a out_shape_type: per-point type 0=Plane,1=Sphere,2=Cylinder,3=Cone,4=Torus,-1 none.
 * Shape flags: bit0=plane, bit1=sphere, bit2=cylinder, bit3=cone, bit4=torus.
 * epsilon/cluster_epsilon <= 0 => CGAL defaults (1% bbox diagonal).
 * \a random_seed: CGAL global RNG seed. Same seed => deterministic detect
 * (required for Geometry Nodes viewport re-evaluation).
 */
/* --- Catalog batch 23 --- */
/**
 * Remove exact + almost-degenerate faces (needles/caps).
 * \a cap_threshold: cos of max angle (0 => CGAL default cos(160°)).
 * \a needle_threshold: longest/shortest edge ratio (0 => default 4).
 * \a collapse_length_threshold: max collapse edge length (0 => unlimited).
 */
MeshResult mesh_repair_degeneracies(const MeshIn &mesh,
                                    double cap_threshold,
                                    double needle_threshold,
                                    double collapse_length_threshold);
/**
 * Surface intersection polylines between two meshes.
 * Each polyline is xyz packed floats (size = n_pts * 3).
 */
bool mesh_surface_intersection_polylines(const MeshIn &mesh_a,
                                         const MeshIn &mesh_b,
                                         std::vector<std::vector<float>> &out_polylines,
                                         std::string &error);
/**
 * Structure point set (planes via Efficient RANSAC, then structure_point_set).
 * \a epsilon: adjacency + sampling size (required > 0).
 * Output points + normals.
 */
bool points_structure(const float *in_xyz,
                      const float *in_nxyz,
                      int n,
                      double epsilon,
                      double attraction_factor,
                      double ransac_epsilon,
                      double ransac_cluster_epsilon,
                      int min_points,
                      std::vector<float> &out_xyz,
                      std::vector<float> &out_nxyz,
                      std::string &error);
/**
 * Scale-space reconstruction with smoother/mesher choice.
 * \a smoother: 0=Weighted PCA, 1=Jet.
 * \a mesher: 0=Alpha shape, 1=Advancing front.
 */
MeshResult points_scale_space_reconstruct_ex(const float *positions,
                                             int n,
                                             int iterations,
                                             int smoother,
                                             int mesher);

/* --- Catalog batch 24 --- */
/**
 * Split edges where scalar crosses \a isovalue and insert isoline edges.
 * \a vertex_values size == verts_num. Isoline edges exported as seam pairs.
 */
MeshResult mesh_refine_at_isolevel(const MeshIn &mesh,
                                   const float *vertex_values,
                                   double isovalue);
/**
 * Exact multi-source geodesic distances (Surface_mesh_shortest_path).
 * \a source_verts: original vertex indices. Unreachable verts get -1.
 */
bool mesh_exact_geodesic_distances(const MeshIn &mesh,
                                   const std::vector<int> &source_verts,
                                   std::vector<float> &out_dist,
                                   std::string &error);
/**
 * Skin surface from balls. \a radii null => radius 1. shrink in (0,1).
 * subdivisions: Loop-like skin subdiv passes (0–4).
 */
MeshResult points_skin_surface(const float *positions,
                               const float *radii,
                               int n,
                               double shrink_factor,
                               int subdivisions);

/* Catalog batch 25 — new only (not previously deleted). */
/**
 * Union of balls mesh (no shrink). \a radii null => radius 1. subdivisions 0–2.
 */
MeshResult points_union_of_balls(const float *positions,
                                 const float *radii,
                                 int n,
                                 int subdivisions);
/**
 * Minimum enclosing sphere of spheres (center + radius), as UV sphere mesh.
 * \a radii null => radius 0 (same as Min Sphere of points).
 */
MeshResult min_sphere_of_spheres(const float *positions,
                                 const float *radii,
                                 int n,
                                 int sphere_segments);

/* Catalog batch 26 — new only (not previously deleted). */
/**
 * Variational Shape Approximation (plane proxies → simplified triangle mesh).
 * face_tag[0] = proxy count when size==1; radius also stores proxy count.
 */
MeshResult mesh_vsa_approximate(const MeshIn &mesh,
                                int max_proxies,
                                int iterations,
                                int seeding_method);
/**
 * Min enclosing circle of points projected to a plane.
 * plane: 0=XY, 1=XZ, 2=YZ. Disk mesh + center/radius.
 */
MeshResult points_min_circle_2(const float *positions, int n, int plane, int segments);
/**
 * Min-area oriented rectangle of points projected to a plane (quad mesh).
 * plane: 0=XY, 1=XZ, 2=YZ.
 */
MeshResult points_min_rectangle_2(const float *positions, int n, int plane);

/* Catalog batch 27 — new only (not previously deleted). */
/** Min enclosing ellipse on plane projection (disk-like fan mesh). */
MeshResult points_min_ellipse_2(const float *positions, int n, int plane, int segments);
/** Min-area oriented parallelogram on plane projection (quad). */
MeshResult points_min_parallelogram_2(const float *positions, int n, int plane);
/** 2D convex hull of projected points (fan mesh). Distinct from deleted 3D Convex Hull node. */
MeshResult points_convex_hull_2(const float *positions, int n, int plane);
/** Plane slice of a triangle mesh → polylines (xyz packed per curve). */
bool mesh_plane_slice(const MeshIn &mesh,
                      float px,
                      float py,
                      float pz,
                      float nx,
                      float ny,
                      float nz,
                      std::vector<std::vector<float>> &out_polylines,
                      std::string &error);
/**
 * VCM sharp-feature flags per point.
 * \a offset_radius / convolution_radius <= 0 auto from average spacing.
 * \a out_ratio: eigenvalue ratio per point; feature when ratio >= threshold.
 */
bool points_vcm_feature_edges(const float *in_xyz,
                              int n,
                              double offset_radius,
                              double convolution_radius,
                              double threshold,
                              std::vector<int8_t> &out_feature,
                              std::vector<float> &out_ratio,
                              int &out_feature_count,
                              std::string &error);
/** Delaunay triangulation of projected points as a triangle mesh. */
MeshResult points_delaunay_2(const float *positions, int n, int plane);
/**
 * Alpha shape of projected points as filled triangles.
 * \a use_optimal_alpha picks CGAL optimal solid component alpha when true.
 */
MeshResult points_alpha_shape_2(const float *positions,
                                int n,
                                int plane,
                                double alpha,
                                bool use_optimal_alpha);
/** Voronoi cells of projected points as isolated n-gons (clip box = site bbox + padding). */
MeshResult points_voronoi_2(const float *positions, int n, int plane, float clip_margin);
/**
 * Interior straight skeleton for all projected XY border islands (with holes).
 * Closed meshes fall back to convex hull.
 */
WireResult mesh_straight_skeleton_2(const MeshIn &mesh);
/**
 * Polygon offset of all projected XY border islands (with holes).
 * \a outward: false = interior (inward), true = exterior (outward).
 * \a mode: 0 = straight-skeleton offset (default), 1 = Euclidean (disk Minkowski /
 *          approximated inset). Distance must be > 0.
 * Output is a triangle mesh of the offset region (holes preserved empty).
 * Exterior skeleton path drops CGAL's artificial outer frame.
 * Accepts face borders or pure edge loops (wire mesh).
 */
/**
 * Signed \a offset_distance: >0 expand (outward), <0 shrink (inward).
 * \a mode: 0=skeleton, 1=Euclidean. Open polylines use |offset| as buffer radius.
 */
/**
 * Faces → skeleton solid offset; edge-only cyclic wires → bilateral strip frame.
 * \a mode ignored (ABI).
 */
MeshResult mesh_polygon_offset_2(const MeshIn &mesh, double offset_distance, int mode = 0);
/**
 * 2D medial-axis / centerline of XY polygons (interior straight-skeleton arcs).
 * Closest CGAL polygonal centerline; not a 3D MAT. Wire mesh output.
 */
WireResult mesh_medial_axis_2(const MeshIn &mesh);

/* Catalog batch 28 — new only (not previously deleted). */
/** Min enclosing annulus of projected points (ring mesh). radius=outer, alpha_used=inner. */
MeshResult points_min_annulus_2(const float *positions, int n, int plane, int segments);
/**
 * Convex partition of XY border polygons.
 * method: 0=approx, 1=greene, 2=optimal. Output n-gon pieces.
 */
MeshResult mesh_convex_partition_2(const MeshIn &mesh, int method);
/** Y-monotone partition of XY border polygons (n-gon pieces). */
MeshResult mesh_y_monotone_partition_2(const MeshIn &mesh);
/** Maximum-area inscribed k-gon of the projected convex hull. */
MeshResult points_max_area_k_gon_2(const float *positions, int n, int plane, int k);
/** Simplify border rings (CGAL polyline simplification, squared-distance cost). */
MeshResult mesh_polyline_simplify_2(const MeshIn &mesh, double cost_threshold);
/**
 * Minimal width of a 3D point set (parallel supporting planes).
 * radius/alpha_used = width; mesh = two plane quads; plane_scale sizes quads.
 */
MeshResult points_width_3(const float *positions, int n, float plane_scale);
/** Minkowski sum of first border polygons of two meshes (triangulated PWH). */
MeshResult mesh_minkowski_sum_2(const MeshIn &mesh_a, const MeshIn &mesh_b);
/**
 * Straight-skeleton roof extrusion of XY border islands.
 * \a outward: false=inward slope, true=outward. Height is max extrusion height.
 */
MeshResult mesh_extrude_skeleton(const MeshIn &mesh, double height, bool outward);
/** CDT fill of XY border islands with holes (triangle mesh). */
MeshResult mesh_polygon_fill_2(const MeshIn &mesh);
/** True if any XY border polygons of A and B intersect (edges or containment). */
bool mesh_do_intersect_2(const MeshIn &mesh_a, const MeshIn &mesh_b, std::string &error);

/* Catalog batch 29 — new only (not previously deleted). */
/**
 * 2D boolean of flattened meshes treated as 2D shapes (union of XY-area faces).
 * mode: 0=union, 1=intersection, 2=difference(A-B), 3=symmetric difference.
 * If B is empty, output is the silhouette outline of A (face_tag 0=outer 1=hole).
 */
MeshResult mesh_boolean_ops_2(const MeshIn &mesh_a, const MeshIn &mesh_b, int mode);
/** Largest empty axis-aligned rectangle inside points' padded bbox (quad mesh). */
MeshResult points_largest_empty_iso_rectangle_2(const float *positions, int n, int plane);
/** Repair self-intersecting / invalid XY border polygons (even-odd rule). */
MeshResult mesh_polygon_repair_2(const MeshIn &mesh);
/**
 * Per-point Monge form via jet fitting (k-NN).
 * out_k1/k2 size n; out_d1/d2/n size n*3 (xyz vectors).
 */
bool points_monge_jet_fit(const float *positions,
                          int n,
                          int knn,
                          int degree_fitting,
                          int degree_monge,
                          std::vector<float> &out_k1,
                          std::vector<float> &out_k2,
                          std::vector<float> &out_d1,
                          std::vector<float> &out_d2,
                          std::vector<float> &out_n,
                          std::string &error);


/* Catalog batch 30 — new only. */
/** 3D convex decomposition via Nef + convex_decomposition_3. face_tag = piece id. */
MeshResult mesh_convex_decomposition_3(const MeshIn &mesh);
/**
 * Cotangent Laplacian editing with ROI / control pins / targets.
 * Output is positions_only (topology preserved).
 */
MeshResult mesh_laplace_deform(const MeshIn &mesh,
                               const uint8_t *roi_mask,
                               const uint8_t *control_mask,
                               const float *target_xyz,
                               double weight);
/**
 * Heavy surface remesh via PMP::surface_Delaunay_remeshing (Mesh_3 criteria).
 */
MeshResult mesh_surface_delaunay_remesh(const MeshIn &mesh,
                                        double facet_size,
                                        double facet_angle,
                                        double facet_distance,
                                        double features_angle_bound,
                                        bool protect_constraints);

/* Catalog batch 31 — new only. */
/** Optimal transportation reconstruction of a 1-D shape (wire) from 2D points. */
WireResult points_otr_reconstruct_2(const float *positions,
                                    int n,
                                    int plane,
                                    double keep_percent,
                                    int relocation);
/** Weighted Delaunay (regular triangulation) of projected points. radii may be null. */
MeshResult points_regular_triangulation_2(const float *positions,
                                          const float *radii,
                                          int n,
                                          int plane);
/** Power cells (weighted Voronoi) as isolated n-gons. */
MeshResult points_power_diagram_2(const float *positions,
                                  const float *radii,
                                  int n,
                                  int plane,
                                  float clip_margin);
/** 3D Voronoi cells as isolated polyhedra (clip box = site bbox + padding). */
MeshResult points_voronoi_3(const float *positions, int n, float clip_margin);
/**
 * Visibility among XY walls from a query point.
 * Faces: only border cycles (solid obstacles / room). Loose edges: thin walls.
 * Query must lie inside the XY bounding box of the walls.
 */
MeshResult mesh_visibility_2(const MeshIn &mesh, float qx, float qy, float qz);
/**
 * Mesh_2 Delaunay refinement of XY polygons.
 * \a max_edge <= 0 means no size bound. \a shape_bound is CGAL B (default 0.125).
 */
MeshResult mesh_refine_2(const MeshIn &mesh, double max_edge, double shape_bound, int lloyd_iters);
/**
 * 3D alpha complex as tetrahedron shells (≠ Alpha Shape surface).
 * face_tag = tet id. radius/alpha_used store tet count / alpha.
 */
MeshResult points_alpha_complex_3(const float *positions,
                                  int n,
                                  double alpha,
                                  bool use_optimal_alpha,
                                  bool separate_tets);
/** Convex Minkowski sum: conv(verts(A)) ⊕ conv(verts(B)). */
MeshResult mesh_minkowski_sum_3_convex(const MeshIn &mesh_a, const MeshIn &mesh_b);

/* Catalog batch 32 — new only. */
/** Weighted Delaunay tetrahedra. radii may be null. face_tag = tet id. */
MeshResult points_regular_triangulation_3(const float *positions,
                                          const float *radii,
                                          int n,
                                          bool separate_tets);
/** 3D power cells as isolated polyhedra (clip box = site bbox + padding). */
MeshResult points_power_diagram_3(const float *positions,
                                  const float *radii,
                                  int n,
                                  float clip_margin);
/** Largest empty circle among projected points (Voronoi vertex in the hull). */
MeshResult points_largest_empty_circle_2(const float *positions, int n, int plane, int segments);
/** Largest inscribed circle of XY border polygons (disk mesh + center/radius). */
MeshResult mesh_largest_inscribed_circle_2(const MeshIn &mesh, int segments);
/**
 * Minimal caliper width of projected points.
 * radius/alpha_used = width; mesh = supporting-strip quad.
 */
MeshResult points_min_width_2(const float *positions, int n, int plane, float plane_scale);
/**
 * Periodic Delaunay of projected points.
 * domain_x/domain_y <= 0 → padded point bbox.
 */
MeshResult points_periodic_delaunay_2(const float *positions,
                                      int n,
                                      int plane,
                                      double domain_x,
                                      double domain_y);
/** Interior constrained Voronoi of XY border polygons (wire). */
WireResult mesh_constrained_voronoi_2(const MeshIn &mesh);

/* Catalog batch 33 — remaining (others reserved/deleted). */
/**
 * Find All Cells 2D: split crossings, emit every bounded loop as a
 * welded n-gon. Works from a pure edge/wire mesh. face_tag = cell id.
 */
MeshResult mesh_arrangement_2(const MeshIn &mesh);
/** Delaunay triangulation on the bounding sphere of 3D points. */
MeshResult points_delaunay_on_sphere(const float *positions, int n);
/**
 * Periodic Delaunay tetrahedra using offset-corrected vertex positions
 * so wrapping cells sit on the domain boundary.
 * domain_* <= 0 → padded point bbox. face_tag = tet id.
 */
MeshResult points_periodic_delaunay_3(const float *positions,
                                      int n,
                                      double domain_x,
                                      double domain_y,
                                      double domain_z,
                                      bool separate_tets);

/* Catalog batch 34 — new only. */
/** True if any triangles of A and B intersect. */
bool mesh_do_intersect_3(const MeshIn &mesh_a, const MeshIn &mesh_b, std::string &error);
/**
 * Corefine Mesh with Cutter and split Mesh along the intersection.
 * face_tag = connected-component id; radius = component count.
 */
MeshResult mesh_split_by_mesh(const MeshIn &mesh, const MeshIn &cutter);
/**
 * Sibson natural-neighbor interpolator on the XY Delaunay of sites.
 * Build once, then query many times (read-only, thread-safe).
 */
struct NaturalNeighbor2;

NaturalNeighbor2 *natural_neighbor_2_new(const float *sites_xyz, int sites_n, std::string &error);
void natural_neighbor_2_free(NaturalNeighbor2 *nn);
int natural_neighbor_2_site_count(const NaturalNeighbor2 *nn);

/**
 * Packed Sibson weights for a batch of queries (XY only).
 * r_offsets size == query_n + 1.
 * Query i uses r_site_index/r_weight in [r_offsets[i], r_offsets[i+1]).
 * Weights are normalized to sum to 1. Outside the hull: empty range, r_valid[i]=false.
 * r_valid may be null.
 */
void natural_neighbor_2_query_many(const NaturalNeighbor2 *nn,
                                   const float *query_xyz,
                                   int query_n,
                                   std::vector<int> &r_offsets,
                                   std::vector<int> &r_site_index,
                                   std::vector<float> &r_weight,
                                   bool *r_valid);

/**
 * Laplace natural-neighbor interpolator on the 3D Delaunay of sites.
 * Build once, then query many times (read-only after build).
 */
struct NaturalNeighbor3;

NaturalNeighbor3 *natural_neighbor_3_new(const float *sites_xyz, int sites_n, std::string &error);
void natural_neighbor_3_free(NaturalNeighbor3 *nn);
int natural_neighbor_3_site_count(const NaturalNeighbor3 *nn);

/**
 * Packed Laplace weights for a batch of 3D queries.
 * r_offsets size == query_n + 1.
 * Query i uses r_site_index/r_weight in [r_offsets[i], r_offsets[i+1]).
 * Weights are normalized to sum to 1. Outside the hull: empty range, r_valid[i]=false.
 * r_valid may be null.
 */
void natural_neighbor_3_query_many(const NaturalNeighbor3 *nn,
                                   const float *query_xyz,
                                   int query_n,
                                   std::vector<int> &r_offsets,
                                   std::vector<int> &r_site_index,
                                   std::vector<float> &r_weight,
                                   bool *r_valid);
/**
 * Proximity graph of XY Delaunay.
 * mode: 0=Gabriel, 1=Relative Neighborhood, 2=Euclidean MST.
 */
WireResult points_proximity_graph_2(const float *positions, int n, int mode);
/** Fill every closed 3D polyline (all wire cycles, else all n-gons) with triangles. */
MeshResult mesh_fill_polyline(const MeshIn &mesh);
/**
 * Snap points onto regularized least-squares planes (parallel / ortho / coplanar).
 * plane_id[i] < 0 is left unassigned. out_xyz size n*3.
 */
bool points_regularize_planes(const float *positions,
                              const int *plane_id,
                              int n,
                              bool parallelism,
                              bool orthogonality,
                              bool coplanarity,
                              bool axis_symmetry,
                              double angle_deg,
                              double coplanar_tol,
                              std::vector<float> &out_xyz,
                              int &out_plane_count,
                              std::string &error);

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
                             std::string &error);

/* Catalog batch 35 - new only (not previously deleted). */
/** Intersect inward half-spaces of every mesh face (convex kernel). */
MeshResult mesh_halfspace_intersection_3(const MeshIn &mesh);
/** Lloyd / CVT relax of a point cloud on a 2D or 3D surface. */
bool points_lloyd_relax(const MeshIn &surface,
                        const float *sites_xyz,
                        int sites_n,
                        int max_iters,
                        double convergence,
                        std::vector<float> &out_xyz,
                        std::string &error);
/** Insert Steiner points so every constraint is a Delaunay/Gabriel edge. mode: 0=Delaunay, 1=Gabriel. */
MeshResult mesh_conforming_delaunay_2(const MeshIn &mesh, int mode);
/** Maximum-perimeter k-gon inscribed in the XY convex hull. */
MeshResult points_max_perimeter_k_gon_2(const float *positions, int n, int k);
/** Constrained Delaunay. fill_rule: 0=even-odd, 1=nonzero winding. */
MeshResult mesh_constrained_delaunay_2(const MeshIn &mesh, int fill_rule);

/* Catalog batch 36 - new only (not previously deleted). */
/** Exterior straight skeleton. max_offset > 0. Frame vertices discarded. */
WireResult mesh_exterior_skeleton_2(const MeshIn &mesh, double max_offset);
/** Spherical Voronoi cells (dual of Delaunay on Sphere). Isolated n-gons. */
MeshResult points_voronoi_on_sphere(const float *positions, int n);

/* Catalog batch 37 - new only (not previously deleted). */
/** Connect 3D points whose Euclidean distance is <= radius. */
WireResult points_radius_graph_3(const float *positions, int n, double radius);
/** k-nearest-neighbor wire of 3D points. k>=1. */
WireResult points_knn_graph_3(const float *positions, int n, int k);
/** Laplace natural-neighbor interpolate site values at query points. */
bool points_natural_neighbor_3(const float *sites,
                               const float *site_values,
                               int n_sites,
                               const float *queries,
                               int n_queries,
                               std::vector<float> &out_values,
                               std::string &error);
/** Sibson gradient at each 2D site from its value. out_grad is gx,gy,0 per site. */
bool points_sibson_gradient_2(const float *positions,
                              const float *values,
                              int n,
                              std::vector<float> &out_grad_xyz,
                              std::string &error);

/* Catalog batch 38 - 10 new nodes. */
/** Octree of 3D points. Solid cubes, or wire boxes (seams). */
MeshResult points_octree_3(const float *positions, int n, int max_depth, int bucket, bool solid);
/** Polyline visiting points in Hilbert order. */
WireResult points_hilbert_path_3(const float *positions, int n);
/** Shortest non-contractible cycle on a closed surface (genus >= 1). */
WireResult mesh_shortest_cycle(const MeshIn &mesh);
/** Periodic Voronoi cells (dual of Periodic Delaunay 2D). Isolated n-gons. */
MeshResult points_periodic_voronoi_2(const float *positions, int n, double domain_x, double domain_y);
/** Periodic Voronoi cells as closed polyhedra (dual of Periodic Delaunay 3D). face_tag = cell. */
MeshResult points_periodic_voronoi_3(const float *positions,
                                     int n,
                                     double domain_x,
                                     double domain_y,
                                     double domain_z);
/** 3D Gabriel graph (diametral-ball-empty Delaunay edges). */
WireResult points_gabriel_graph_3(const float *positions, int n);
/** Euclidean MST of 3D points (Borůvka on the Delaunay graph). */
WireResult points_euclidean_mst_3(const float *positions, int n);
/** Graph MST on existing mesh edges (no Delaunay). Euclidean edge weights. */
WireResult mesh_euclidean_mst_3(const MeshIn &mesh);
/** Lune-based beta-skeleton of XY points (subgraph of Delaunay). */
WireResult points_beta_skeleton_2(const float *positions, int n, double beta);
/** Nested convex hulls (onion peeling) in XY. Isolated n-gons, face_tag = layer. */
MeshResult points_convex_layers_2(const float *positions, int n);

/* Catalog batch 39 — new only (not previously deleted). */
/**
 * Farthest-point Voronoi cells of XY sites (dual of the upper hull of
 * the paraboloid lift). Isolated n-gons, face_tag = hull-site index.
 * Unbounded cells clipped to the site bbox + padding.
 */
MeshResult points_farthest_voronoi_2(const float *positions, int n, float clip_margin);
/** Nested 3D convex hulls (onion peeling). Isolated hulls, face_tag = layer. */
MeshResult points_convex_layers_3(const float *positions, int n);
/**
 * Arrangement overlay of two XY edge sets.
 * face_tag: 1 = only A, 2 = only B, 3 = both. Bounded cells only.
 */
MeshResult mesh_overlay_2(const MeshIn &mesh_a, const MeshIn &mesh_b);
/** Kernel of XY polygons (intersection of inward edge half-planes). Isolated n-gons. */
MeshResult mesh_polygon_kernel_2(const MeshIn &mesh);
/** Self-intersection polylines of one triangle mesh (≠ two-mesh Mesh Intersection). */
bool mesh_self_intersection_polylines(const MeshIn &mesh,
                                      std::vector<std::vector<float>> &out_polylines,
                                      std::string &error);
/** Crust curve reconstruction of XY samples (Amenta–Bern–Eppstein). Wire. */
WireResult points_crust_2(const float *positions, int n);
/**
 * Geodesic Voronoi on a triangle mesh (MMP / Surface_mesh_shortest_path).
 * source_mask size == verts_num (nonzero = source). Faces are split along the
 * geodesic-equidistant bisector (not the mesh dual of vertex labels).
 * face_tag = source vertex index.
 */
MeshResult mesh_geodesic_voronoi(const MeshIn &mesh, const uint8_t *source_mask);
/**
 * Marching tetrahedra isosurface of a 3D point + scalar field
 * (Delaunay tet mesh of the samples).
 */
MeshResult points_isosurface_3(const float *positions, const float *values, int n, double isolevel);

/* Catalog batch 40 — new only (not previously deleted). */
/** Arrangement of infinite lines through XY edges. Bounded cells, face_tag = island. */
MeshResult mesh_line_arrangement_2(const MeshIn &mesh);
/** Vertical decomposition of an XY arrangement (trapezoids). face_tag = island. */
MeshResult mesh_vertical_decomposition_2(const MeshIn &mesh);
/**
 * Arrangement of discretized circles (center = point, radius = radii[i]).
 * face_tag = overlap count (how many disks contain the cell).
 */
MeshResult points_circle_arrangement_2(const float *positions,
                                       const float *radii,
                                       int n,
                                       int segments);
/** 3D crust: DT of samples ∪ Voronoi vertices, keep sample–sample triangles. */
MeshResult points_crust_3(const float *positions, int n);
/**
 * Voronoi cells of 3D sites clipped to the point bbox + padding.
 * Isolated polyhedra, face_tag = site index.
 */
MeshResult points_clipped_voronoi_3(const float *positions, int n, float clip_margin);
/**
 * Complement of XY faces inside the point bbox + padding.
 * face_tag = island. Empty rings ⇒ the padded bbox rectangle.
 */
MeshResult mesh_complement_2(const MeshIn &mesh, float padding);
/** Uncross XY points into one simple polygon (CGAL make_simple_polygon). */
MeshResult points_simple_polygon_2(const float *positions, int n);

/* Catalog batch 41 — new only (not previously deleted). */
/** Sweep-line split of XY edges at crossings. Wire mesh of x-monotone subsegments. */
WireResult mesh_split_crossings_2(const MeshIn &mesh);
/** Proper intersection points of XY edges (not endpoints). Positions only. */
MeshResult mesh_crossing_points_2(const MeshIn &mesh);
/**
 * Single-mold translational casting: input polygon + a wedge per top edge
 * showing the closed range of pullout directions. face_tag 0 = part, 1+ = wedges.
 */
MeshResult mesh_pullout_directions_2(const MeshIn &mesh);
/** Small-side angle-bisector convex decomposition. face_tag = piece id. */
MeshResult mesh_ssab_partition_2(const MeshIn &mesh);
/** Snap nearby border vertices within Tolerance (PMP experimental snap_borders). */
MeshResult mesh_snap_borders(const MeshIn &mesh, double tolerance);
/** Autorefine then delete leftover self-intersections. */
MeshResult mesh_autorefine_clean(const MeshIn &mesh);
/** Random simple n-gon in [-Size, Size]². */
MeshResult points_random_polygon_2(int count, double half_size, int seed);
/** Random convex n-gon in [-Size, Size]². */
MeshResult points_random_convex_set_2(int count, double half_size, int seed);

/* Catalog batch 42 — new only (not previously deleted). */
/** Largest empty sphere whose center lies inside the convex hull (DT3 circumcenter). */
MeshResult points_largest_empty_sphere_3(const float *positions, int n, int segments);
/**
 * Efficient RANSAC shapes as preview meshes (plane n-gon / sphere / cylinder / cone / torus).
 * face_tag = shape id. Same flags as Efficient RANSAC.
 */
MeshResult points_ransac_primitives(const float *in_xyz,
                                    const float *in_nxyz,
                                    int n,
                                    double epsilon,
                                    double cluster_epsilon,
                                    double normal_threshold,
                                    int min_points,
                                    double probability,
                                    int shape_flags,
                                    unsigned int random_seed,
                                    int segments);
/** 3D Voronoi ∩ plane (Voronoi_intersection_2_traits_3). Isolated n-gons, face_tag = site. */
MeshResult points_voronoi_slice_3(const float *positions,
                                  int n,
                                  float ox,
                                  float oy,
                                  float oz,
                                  float nx,
                                  float ny,
                                  float nz,
                                  float clip_margin);
/** Offset every face plane then halfspace-intersect (convex / convex kernel). */
MeshResult mesh_convex_offset_3(const MeshIn &mesh, double offset);
/**
 * Volume-connected components of a closed triangle mesh.
 * face_tag = volume id. alpha_used = volume count.
 */
MeshResult mesh_volume_components(const MeshIn &mesh);
/** Largest inscribed sphere of the convex kernel (Chebyshev center). */
MeshResult mesh_inscribed_sphere_3(const MeshIn &mesh, int segments);
/** PCA axis + min enclosing circle of the radial plane → finite cylinder. */
MeshResult points_min_cylinder_3(const float *positions, int n, int segments);

/* --- Catalog batch 43 (new only; do not restore deleted ids) --- */
/**
 * Fill every boundary loop with triangulate_refine_and_fair_hole.
 * \a continuity: 0/1/2 (C0/C1/C2 fairing of the patch). Input is triangulated
 * internally because the fairing API requires a triangle mesh.
 */
MeshResult mesh_fair_hole_fill(const MeshIn &mesh, int continuity);
/**
 * Douglas–Peucker (or iterative) 3D polyline simplification.
 * Input: wire edges, or verts in order when there are no edges.
 */
WireResult mesh_simplify_polyline_3(const MeshIn &mesh,
                                    double max_distance,
                                    bool iterative);
/**
 * Simplify one 3D polyline (Douglas–Peucker / iterative).
 * \a closed: treat as a loop (first point is appended internally).
 * Output points are a subset of the input (original samples kept).
 */
bool simplify_polyline_xyz(const float *xyz,
                           int n,
                           bool closed,
                           double max_distance,
                           bool iterative,
                           std::vector<float> &out_xyz,
                           std::vector<int> &out_src,
                           std::string &error);
/**
 * Unified point-shape region growing.
 * \a mode: 0 plane, 1 sphere, 2 cylinder, 3 circle (PCA plane), 4 line.
 * Fitted preview mesh in the result; optional per-point region ids.
 */
MeshResult points_shape_fitting(const float *in_xyz,
                                const float *in_nxyz,
                                int n,
                                int mode,
                                double neighbor_radius,
                                double max_distance,
                                double max_angle_deg,
                                int min_region_size,
                                double min_radius,
                                double max_radius,
                                int segments,
                                std::vector<int> *out_region);
/**
 * Least-squares sphere region growing. Fitted UV spheres, face_tag = region id.
 * \a out_region (optional): per-point id, size n, -1 unassigned.
 */
MeshResult points_sphere_region_growing(const float *in_xyz,
                                        const float *in_nxyz,
                                        int n,
                                        double neighbor_radius,
                                        double max_distance,
                                        double max_angle_deg,
                                        int min_region_size,
                                        double min_radius,
                                        double max_radius,
                                        int segments,
                                        std::vector<int> *out_region);
/** Least-squares cylinder region growing. Finite capped cylinders, face_tag = id. */
MeshResult points_cylinder_region_growing(const float *in_xyz,
                                          const float *in_nxyz,
                                          int n,
                                          double neighbor_radius,
                                          double max_distance,
                                          double max_angle_deg,
                                          int min_region_size,
                                          double min_radius,
                                          double max_radius,
                                          int segments,
                                          std::vector<int> *out_region);
/**
 * Least-squares circle region growing in the PCA plane of the cloud.
 * Disks in that plane, face_tag = id.
 */
MeshResult points_circle_region_growing(const float *in_xyz,
                                        const float *in_nxyz,
                                        int n,
                                        double neighbor_radius,
                                        double max_distance,
                                        double max_angle_deg,
                                        int min_region_size,
                                        double min_radius,
                                        double max_radius,
                                        int segments,
                                        std::vector<int> *out_region);
/**
 * 3D line region growing (PCA line + sphere neighbors). Wire segments.
 */
WireResult points_line_region_growing(const float *in_xyz,
                                      const float *in_nxyz,
                                      int n,
                                      double neighbor_radius,
                                      double max_distance,
                                      int min_region_size,
                                      std::vector<int> *out_region);

/* --- Catalog batch 44 (new only; do not restore deleted ids) --- */
/** Smallest spherical shell (two concentric spheres) enclosing the points. */
MeshResult points_min_annulus_3(const float *positions, int n, int segments);
/** Collapse edges shorter than max_length (shortest-first, midpoint placement). */
MeshResult mesh_collapse_short_edges(const MeshIn &mesh, double max_length);

/* --- Catalog batch 45 (new only; do not restore deleted ids) --- */
/**
 * Regularize one open (or closed) XY polyline. Output points replace the input
 * (same plane, mean Z kept).
 */
bool regularize_open_polyline_xy(const float *xyz,
                                 int n,
                                 bool closed,
                                 double max_offset,
                                 double min_length,
                                 std::vector<float> &out_xyz,
                                 std::string &error);
/**
 * QEM simplify that does not collapse constrained edges.
 * Border edges if \a keep_boundary; edges incident to preserve[i]!=0 vertices.
 */
MeshResult mesh_constrained_simplify(const MeshIn &mesh,
                                     double stop_ratio,
                                     bool keep_boundary,
                                     const uint8_t *preserve,
                                     int preserve_n);

/* --- Catalog batch 47 (new only; do not restore deleted ids) --- */
/** Mesh ∩ finite cone side (apex → axis*height, radius at base). Polylines as packed xyz. */
bool mesh_cone_slice(const MeshIn &mesh,
                     float ax,
                     float ay,
                     float az,
                     float dx,
                     float dy,
                     float dz,
                     float radius,
                     float height,
                     int segments,
                     std::vector<std::vector<float>> &out_polylines,
                     std::string &error);
/** Keep the part of \a mesh inside an AABB (center + full size). */
MeshResult mesh_clip_box(const MeshIn &mesh,
                         float cx,
                         float cy,
                         float cz,
                         float sx,
                         float sy,
                         float sz,
                         bool clip_volume);
/** Keep the part of \a mesh inside closed \a clipper (PMP clip). Distinct from Boolean. */
MeshResult mesh_clip_by_mesh(const MeshIn &mesh, const MeshIn &clipper, bool clip_volume);

/* Catalog batch 48 — new only (not previously deleted). */
/**
 * p axis-aligned squares of equal L_infty radius covering the XY points (p=2..4).
 * Isolated quads, face_tag = square index. radius = half-side.
 */
MeshResult points_rectangular_p_center_2(const float *positions, int n, int p);
/**
 * Melkman convex hull of each ordered XY ring (face loops, else wire polylines).
 * Isolated n-gons, face_tag = island. Distinct from deleted Convex Hull 2D (unordered points).
 */
MeshResult mesh_polyline_hull_2(const MeshIn &mesh);

/* Catalog batch 49 — new only (not previously deleted). */
/**
 * Intersect 3D polylines with a triangle mesh (AABB Segment_3 vs faces).
 * Overlapping coplanar pieces go to \a out_segments (packed xyz polylines);
 * isolated pierce points go to \a out_points (xyz * n). Distinct from Mesh
 * Intersection (surface-surface) and Mesh Slicer (infinite plane).
 */
bool mesh_curve_intersect(const MeshIn &mesh,
                          const float *curve_xyz,
                          int points_num,
                          const int *curve_offsets,
                          int curves_num,
                          std::vector<std::vector<float>> &out_segments,
                          std::vector<float> &out_points,
                          std::string &error);
/**
 * Isolines of multi-source MMP geodesic distance. \a count levels between 0
 * and max geodesic (or \a max_distance if > 0). Distinct from Exact Geodesic
 * (vertex field) and Refine at Isolevel (splits the mesh).
 */
bool mesh_geodesic_isolines(const MeshIn &mesh,
                            const uint8_t *source_mask,
                            int count,
                            float max_distance,
                            std::vector<std::vector<float>> &out_polylines,
                            std::string &error);
/**
 * Faces of \a mesh whose triangles intersect \a other (AABB + triangle test).
 * Compacted triangles, face_src = original face of \a mesh. Distinct from
 * Do Intersect 3D (boolean), Mesh Intersection (curves), Mark Self Intersect.
 */
MeshResult mesh_overlap_faces(const MeshIn &mesh, const MeshIn &other);
/**
 * Count parallel plane slices along Direction. Spacing <= 0 fits the bbox.
 * Distinct from Mesh Slicer (single plane).
 */
bool mesh_contour_stack(const MeshIn &mesh,
                        float ox,
                        float oy,
                        float oz,
                        float dx,
                        float dy,
                        float dz,
                        int count,
                        float spacing,
                        std::vector<std::vector<float>> &out_polylines,
                        std::string &error);

/* Catalog batch 50 — new only (not previously deleted). */
/**
 * 1-skeleton of Alpha_shape_3 (REGULAR + SINGULAR edges). Original verts kept.
 * Distinct from Alpha Shape (REGULARIZED triangles) and Alpha Complex 3D (tets).
 */
WireResult points_alpha_edges_3(const float *positions,
                                int n,
                                double alpha,
                                bool use_optimal_alpha,
                                int solid_components);
/**
 * Finite edges of Delaunay_triangulation_3. Original verts kept.
 * Distinct from Delaunay 3D (tet faces) and Voronoi 3D (dual).
 */
WireResult points_delaunay_edges_3(const float *positions, int n);
/**
 * Dijkstra on mesh edges from Source verts to the nearest Target vert.
 * Packed xyz polyline. Distinct from Surface Shortest Path (geodesic MMP).
 */
bool mesh_edge_path(const MeshIn &mesh,
                    const uint8_t *source_mask,
                    const uint8_t *target_mask,
                    std::vector<float> &out_xyz,
                    std::string &error);
/**
 * Isolines of interpolated-corrected mean (mode 0) or Gaussian (mode 1)
 * curvature. Distinct from Curvature (field) and Geodesic Isolines.
 */
bool mesh_curvature_isolines(const MeshIn &mesh,
                             int mode,
                             int count,
                             std::vector<std::vector<float>> &out_polylines,
                             std::string &error);

/* Catalog batch 51 — new only (deleted IDs stay reserved). */
/**
 * Signed (closed mesh) or unsigned distance isosurface.
 * Positive offset grows a closed solid; negative shrinks it.
 * Distinct from Alpha Wrap (envelope) and deleted Offset Convex 3D (planar offset).
 */
MeshResult mesh_offset_sdf(const MeshIn &mesh, double offset, int resolution, bool signed_distance);
/**
 * Fill every boundary loop with triangulate_hole (2D CDT in the fitted plane,
 * 3D DT fallback). No refine, no fair. Distinct from Fair Hole Fill.
 */
MeshResult mesh_cdt_hole_fill(const MeshIn &mesh);
/**
 * Voronoi faces dual to DT edges that connect set A to set B.
 * Isolated n-gons. Distinct from Voronoi 3D (all cells) and Mesh Intersection.
 */
MeshResult points_bisector_surface(const float *a_xyz, int a_n, const float *b_xyz, int b_n);
/**
 * Boolean union of triangles projected along Direction, filled in that plane.
 * Distinct from deleted Project Mesh 2D (flatten, keep overlaps) and Convex Hull 2D.
 */
MeshResult mesh_projected_outline(const MeshIn &mesh, float dx, float dy, float dz);

/* Catalog batch 52 — new only (deleted IDs stay reserved). */
/**
 * Overmars–Welzl visibility graph of XY obstacle segments (input edges /
 * face borders). Original walls plus non-crossing vertex-vertex chords.
 * Distinct from Visibility 2D (visibility polygon from a query point).
 */
WireResult mesh_visibility_graph_2(const MeshIn &mesh);
/**
 * XY constrained Delaunay with per-vertex Z kept (2.5D terrain TIN).
 * fill_rule: 0 = all finite faces (convex hull), 1 = even-odd domain.
 * Distinct from Delaunay 2D / Constrained Delaunay 2D (those flatten Z).
 */
MeshResult mesh_terrain_tin(const MeshIn &mesh, int fill_rule);
/**
 * Smallest enclosing circle in the least-squares plane of the points.
 * Disk mesh lies in that 3D plane (center / radius / normal on the result).
 * Distinct from Min Circle 2D (drops an axis: XY / XZ / YZ).
 */
MeshResult points_min_circle_3(const float *positions, int n, int segments);

/* Catalog batch 53 — new only (deleted IDs stay reserved). */
/**
 * Delaunay tetrahedra whose circumcenter lies inside a closed mesh.
 * Distinct from Delaunay 3D (all tets of the convex hull of a point set)
 * and from Mesh_3 (quality volume mesher, skipped).
 */
MeshResult mesh_interior_tets(const MeshIn &mesh);
/**
 * Tangent-plane Delaunay 1-ring of each point (kNN projected to PCA plane).
 * Distinct from KNN Graph 3D (fixed-k Euclidean) and Delaunay 3D (tets).
 */
WireResult points_surface_delaunay_graph(const float *positions, int n, int knn);
/**
 * Cut the mesh along edges whose adjacent-face normals differ by more than
 * angle_deg, then duplicate those edges so each chart is a separate island.
 * Distinct from deleted Detect Features (edge flags only) and from Region
 * Growing (plane-fit labels, no cut).
 */
MeshResult mesh_split_charts(const MeshIn &mesh, double angle_deg);
/**
 * Restricted Voronoi diagram: clip 3D Voronoi cells of the sites onto the
 * mesh surface. Distinct from Geodesic Voronoi (MMP geodesic distance) and
 * from Voronoi Slice 3D (clip to a plane).
 */
MeshResult mesh_restricted_voronoi(const MeshIn &mesh, const float *sites, int n_sites);

/* Catalog batch 54 — new only (deleted IDs stay reserved). */
/**
 * Finite Delaunay tetrahedra whose interior is stabbed by segment AB.
 * Distinct from Delaunay 3D (every tet of the convex hull).
 */
MeshResult points_walk_tets(const float *positions,
                            int n,
                            double ax,
                            double ay,
                            double az,
                            double bx,
                            double by,
                            double bz);
/**
 * Mean-curvature-flow skeleton plus a spoke from each node to the surface
 * vertices that contracted into it. Distinct from Skeleton (centerline only).
 */
WireResult mesh_skeleton_spokes(const MeshIn &mesh);
/**
 * Slice with Count planes that all contain Axis. Distinct from Mesh Slicer
 * (one plane) and Contour Stack (parallel planes).
 */
bool mesh_radial_slices(const MeshIn &mesh,
                        float ox,
                        float oy,
                        float oz,
                        float ax,
                        float ay,
                        float az,
                        int count,
                        std::vector<std::vector<float>> &out_polylines,
                        std::string &error);
/**
 * Faces of both meshes that touch the corefine intersection seam.
 * Distinct from Mesh Intersection (curves) and Corefine (full meshes).
 */
MeshResult mesh_intersection_band(const MeshIn &a, const MeshIn &b);

/* Catalog batch 55 — remaining official CGAL (6.0.1 + overlaid 6.1/6.2 headers). */
/** Sphere–sphere intersection circles. Radius from optional per-point array or uniform. */
bool points_sphere_intersect(const float *positions,
                             int n,
                             const float *radii,
                             float uniform_radius,
                             int segments,
                             std::vector<std::vector<float>> &out_polylines,
                             std::string &error);
/** Great-circle arrangement of face planes on the unit sphere around Origin. */
MeshResult mesh_sphere_arrangement(const MeshIn &mesh, float ox, float oy, float oz);
/** Feature-aware face chunks: region-grow by dihedral angle, then merge tiny patches.
 * face_tag = segment id. Original mesh is kept (positions_only). */
MeshResult mesh_graphcut_segment(const MeshIn &mesh, double angle_deg, int min_size);
/** Local eigen features (Classification). */
bool points_classification_features(const float *positions,
                                    int n,
                                    int knn,
                                    std::vector<float> &linearity,
                                    std::vector<float> &planarity,
                                    std::vector<float> &sphericity,
                                    std::vector<float> &omnivariance,
                                    std::vector<float> &anisotropy,
                                    std::vector<float> &verticality,
                                    std::string &error);
/**
 * ETHZ random-forest classify. train_label[i] < 0 means unlabeled (predicted).
 * Labels >= 0 are kept. If every point is labeled, output copies the input.
 */
bool points_classify_ethz(const float *positions,
                          int n,
                          const int *train_label,
                          int knn,
                          int n_trees,
                          std::vector<int> &out_label,
                          std::string &error);
/** ICP: rigid-align source onto target. Writes transformed source xyz. */
bool points_register_icp(const float *src,
                         int n_src,
                         const float *tgt,
                         int n_tgt,
                         int iterations,
                         std::vector<float> &out_xyz,
                         std::string &error);
/** Global 4-point congruent set + ICP refine. */
bool points_register_4pcs(const float *src,
                          int n_src,
                          const float *tgt,
                          int n_tgt,
                          int samples,
                          int icp_iterations,
                          std::vector<float> &out_xyz,
                          std::string &error);
/** 2D Alpha Wrap of XY points / segments. Isolated n-gons. */
MeshResult points_alpha_wrap_2(const float *positions,
                               int n,
                               const MeshIn *segments,
                               double alpha,
                               double offset);
/**
 * 3D mean-value cage deform. cage_rest / cage_pose share topology.
 * interior verts are re-expressed in rest-cage coordinates then posed.
 */
MeshResult mesh_cage_deform_3(const MeshIn &cage_rest,
                              const MeshIn &cage_pose,
                              const MeshIn &interior);
/** Dual Contouring / Surface Nets of a dense Cartesian SDF grid (nx*ny*nz samples). */
MeshResult mesh_dual_contour_grid(const float *sdf,
                                  int nx,
                                  int ny,
                                  int nz,
                                  double x0,
                                  double y0,
                                  double z0,
                                  double dx,
                                  double dy,
                                  double dz,
                                  double isovalue);
/** Tetrahedral isotropic remesh of the interior of a closed mesh. */
MeshResult mesh_tet_remesh(const MeshIn &mesh, double target_edge);
/** 3D conforming constrained Delaunay of a triangle mesh (PLC). */
MeshResult mesh_constrained_delaunay_3(const MeshIn &mesh);

/** Approximate convex volumes covering a closed mesh. face_tag = Part id.
 * resolution = cells along the longest bbox axis (other axes scale with extent). */
MeshResult mesh_approximate_convex_decomposition(const MeshIn &mesh, int count, int resolution);

/** Marching Cubes (optionally TCMC) of a dense Cartesian SDF grid. */
MeshResult mesh_marching_cubes_grid(const float *sdf,
                                    int nx,
                                    int ny,
                                    int nz,
                                    double x0,
                                    double y0,
                                    double z0,
                                    double dx,
                                    double dy,
                                    double dz,
                                    double isovalue,
                                    bool tcmc);
/** Convex-hull support vertex in Direction. */
bool points_extreme_point_3(const float *positions,
                            int n,
                            float dx,
                            float dy,
                            float dz,
                            float out_xyz[3],
                            std::string &error);
/** Generalized barycentric weights of query points wrt a closed convex cage.
 * method: 0 Wachspress, 1 Mean Value, 2 Discrete Harmonic.
 * weights size = n_query * n_cage, row-major per query. */
bool mesh_barycentric_3(const MeshIn &cage,
                        const float *query,
                        int n_query,
                        int method,
                        std::vector<float> &weights,
                        int &n_cage,
                        std::string &error);

}  // namespace blender::cgal_bridge


