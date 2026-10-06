# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  .
  intern
  ..
  include
  ../intern
  ../image/include
  ../../editors/include
  ../../compositor
  ../../makesrna
  ../../compositor/algorithms
  ../../compositor/cached_resources
  ../../compositor/derived_resources
  ../../compositor/utilities
  ../../draw
  ../../gpu/intern
  ../../modifiers
  ../../io/common
  ../../io/csv
  ../../io/ply
  ../../io/spz
  ../../io/stl
  ../../io/wavefront_obj
  # RNA_prototypes.hh
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
  # Spectra (header-only, Eigen-based partial eigensolvers).
  ${LIBDIR}/spectra/include
)

set(SRC
  nodes/node_geo_accumulate_field.cc
  nodes/node_geo_attribute_capture.cc
  nodes/node_geo_attribute_domain_size.cc
  nodes/node_geo_attribute_statistic.cc
  nodes/node_geo_attribute_transfer.cc
  nodes/node_geo_bake.cc
  nodes/node_geo_make_it_stand.cc
  nodes/node_geo_winding_number.cc
  nodes/node_geo_blur_attribute.cc
  nodes/node_geo_bone_info.cc
  nodes/node_geo_boolean.cc
  nodes/node_geo_bounding_box.cc
  nodes/node_geo_camera_info.cc
  nodes/node_geo_cgal_advancing_front.cc
  nodes/node_geo_cgal_alpha_shape.cc
  nodes/node_geo_cgal_alpha_wrap.cc
  nodes/node_geo_cgal_corefine.cc
  nodes/node_geo_cgal_delaunay_3d.cc
  nodes/node_geo_cgal_fair.cc
  nodes/node_geo_cgal_mesh_boolean.cc
  nodes/node_geo_cgal_min_sphere.cc
  nodes/node_geo_cgal_optimal_bbox.cc
  nodes/node_geo_cgal_repair.cc
  nodes/node_geo_cgal_periodic_delaunay_2.cc
  nodes/node_geo_cgal_periodic_delaunay_3.cc
  nodes/node_geo_cgal_periodic_voronoi_2.cc
  nodes/node_geo_cgal_periodic_voronoi_3.cc
  nodes/node_geo_cgal_smooth_shape.cc
  nodes/node_geo_cgal_orient_outward.cc
  nodes/node_geo_cgal_repair_self_intersect.cc
  nodes/node_geo_cgal_split_long_edges.cc
  nodes/node_geo_cgal_does_self_intersect.cc
  nodes/node_geo_cgal_mesh_volume.cc
  nodes/node_geo_cgal_estimate_normals.cc
  nodes/node_geo_cgal_poisson.cc
  nodes/node_geo_cgal_side_of_mesh.cc
  nodes/node_geo_cgal_skeleton.cc
  nodes/node_geo_cgal_segmentation.cc
  nodes/node_geo_cgal_min_ellipsoid.cc
  nodes/node_geo_cgal_is_closed.cc
  nodes/node_geo_cgal_centroid.cc
  nodes/node_geo_cgal_autorefine.cc
  nodes/node_geo_cgal_remove_degenerate.cc
  nodes/node_geo_cgal_curvature.cc
  nodes/node_geo_cgal_scale_space.cc
  nodes/node_geo_cgal_principal_curvature.cc
  nodes/node_geo_cgal_shape_diameter.cc
  nodes/node_geo_cgal_mark_self_intersect.cc
  nodes/node_geo_cgal_wlop.cc
  nodes/node_geo_cgal_hierarchy_simplify.cc
  nodes/node_geo_cgal_edge_aware_upsample.cc
  nodes/node_geo_cgal_vcm_estimate_normals.cc
  nodes/node_geo_cgal_pca_estimate_normals.cc
  nodes/node_geo_cgal_mst_orient_normals.cc
  nodes/node_geo_cgal_cluster_point_set.cc
  nodes/node_geo_cgal_region_growing.cc
  nodes/node_geo_cgal_surface_shortest_path.cc
  nodes/node_geo_cgal_jet_smooth.cc
  nodes/node_geo_cgal_bilateral_smooth.cc
  nodes/node_geo_cgal_remove_outliers.cc
  nodes/node_geo_cgal_arap_deform.cc
  nodes/node_geo_cgal_lscm_uv.cc
  nodes/node_geo_cgal_arap_uv.cc
  nodes/node_geo_cgal_alpha_shape_2.cc
  nodes/node_geo_cgal_alpha_complex_3.cc
  nodes/node_geo_cgal_arrangement_2.cc

  nodes/node_geo_cgal_boolean_ops_2.cc
  nodes/node_geo_cgal_convex_decomposition_3.cc
  nodes/node_geo_cgal_convex_partition_2.cc
  nodes/node_geo_cgal_delaunay_2.cc
  nodes/node_geo_cgal_delaunay_on_sphere.cc
  nodes/node_geo_cgal_do_intersect_2.cc
  nodes/node_geo_cgal_do_intersect_3.cc
  nodes/node_geo_cgal_natural_neighbor_2.cc
  nodes/node_geo_cgal_fill_polyline.cc
  nodes/node_geo_cgal_max_perimeter_k_gon_2.cc
  nodes/node_geo_cgal_constrained_delaunay_2.cc
  nodes/node_geo_cgal_exterior_skeleton_2.cc
  nodes/node_geo_cgal_voronoi_on_sphere.cc
  nodes/node_geo_cgal_knn_graph_3.cc
  nodes/node_geo_cgal_skeleton_spokes.cc
  nodes/node_geo_cgal_sphere_intersect.cc
  nodes/node_geo_cgal_sphere_arrangement.cc
  nodes/node_geo_cgal_tet_remesh.cc
  nodes/node_geo_cgal_constrained_dt3.cc
  nodes/node_geo_cgal_approx_convex_decomp.cc
  nodes/node_geo_cgal_dual_contour.cc
  nodes/node_geo_cgal_cage_deform_3.cc
  nodes/node_geo_cgal_graphcut_segment.cc
  nodes/node_geo_cgal_point_features.cc
  nodes/node_geo_cgal_register_icp.cc
  nodes/node_geo_cgal_optimal_transport.cc
  nodes/node_geo_cgal_super4pcs.cc
  nodes/node_geo_cgal_alpha_wrap_2.cc
  nodes/node_geo_cgal_natural_neighbor_3.cc
  nodes/node_geo_cgal_euclidean_mst_3.cc
  nodes/node_geo_cgal_convex_layers_2.cc
  nodes/node_geo_cgal_convex_layers_3.cc
  nodes/node_geo_cgal_self_intersection_curves.cc
  nodes/node_geo_cgal_crust_2.cc
  nodes/node_geo_cgal_geodesic_voronoi.cc
  nodes/node_geo_cgal_vertical_decomp_2.cc
  nodes/node_geo_cgal_simple_polygon_2.cc
  nodes/node_geo_cgal_ssab_partition_2.cc
  nodes/node_geo_cgal_snap_borders.cc
  nodes/node_geo_cgal_largest_empty_sphere_3.cc
  nodes/node_geo_cgal_inscribed_sphere_3.cc
  nodes/node_geo_cgal_min_cylinder_3.cc
  nodes/node_geo_cgal_fair_hole_fill.cc
  nodes/node_geo_cgal_simplify_polyline_3.cc
  nodes/node_geo_cgal_point_shape_fitting.cc
  nodes/node_geo_cgal_min_annulus_3.cc
  nodes/node_geo_cgal_collapse_short_edges.cc
  nodes/node_geo_cgal_regularize_open_contour_2.cc
  nodes/node_geo_cgal_constrained_simplify.cc
  nodes/node_geo_cgal_y_monotone_partition_2.cc
  nodes/node_geo_cgal_polyline_simplify_2.cc
  nodes/node_geo_cgal_geodesic_isolines.cc
  nodes/node_geo_cgal_contour_stack.cc
  nodes/node_geo_cgal_exact_geodesic.cc
  nodes/node_geo_cgal_extrude_skeleton.cc
  nodes/node_geo_cgal_laplace_deform.cc
  nodes/node_geo_cgal_largest_empty_circle_2.cc
  nodes/node_geo_cgal_largest_empty_iso_rectangle_2.cc
  nodes/node_geo_cgal_largest_inscribed_circle_2.cc
  nodes/node_geo_cgal_max_area_k_gon_2.cc
  nodes/node_geo_cgal_medial_axis_2.cc
  nodes/node_geo_cgal_mesh_intersection.cc
  nodes/node_geo_cgal_mesh_slicer.cc
  nodes/node_geo_cgal_min_annulus_2.cc
  nodes/node_geo_cgal_min_circle_2.cc
  nodes/node_geo_cgal_min_ellipse_2.cc
  nodes/node_geo_cgal_min_parallelogram_2.cc
  nodes/node_geo_cgal_min_rectangle_2.cc
  nodes/node_geo_cgal_min_sphere_of_spheres.cc
  nodes/node_geo_cgal_min_width_2.cc
  nodes/node_geo_cgal_minkowski_sum_2.cc
  nodes/node_geo_cgal_minkowski_sum_3.cc
  nodes/node_geo_cgal_monge_jet_fit.cc

  nodes/node_geo_cgal_polygon_fill_2.cc
  nodes/node_geo_cgal_polygon_offset_2.cc
  nodes/node_geo_cgal_polygon_repair_2.cc
  nodes/node_geo_cgal_refine_isolevel.cc
  nodes/node_geo_cgal_refine_mesh_2.cc
  nodes/node_geo_cgal_repair_degeneracies.cc
  nodes/node_geo_cgal_skin_surface.cc
  nodes/node_geo_cgal_straight_skeleton_2.cc
  nodes/node_geo_cgal_surface_delaunay_remesh.cc
  nodes/node_geo_cgal_union_of_balls.cc
  nodes/node_geo_cgal_uv_distortion.cc
  nodes/node_geo_cgal_vcm_feature_edges.cc
  nodes/node_geo_cgal_visibility_2.cc
  nodes/node_geo_cgal_voronoi_2.cc
  nodes/node_geo_cgal_voronoi_3.cc
  nodes/node_geo_cgal_vsa_approximate.cc
  nodes/node_geo_cgal_width_3.cc
  nodes/node_geo_closure.cc
  nodes/node_geo_closure_to_list.cc
  nodes/node_geo_cluster_by_connected.cc
  nodes/node_geo_cluster_by_distance.cc
  nodes/node_geo_collection_children.cc
  nodes/node_geo_collection_info.cc
  nodes/node_geo_combine_bundle.cc
  nodes/node_geo_combine_list.cc
  nodes/node_geo_common.cc
  nodes/node_geo_construct_curves.cc
  nodes/node_geo_convex_hull.cc
  nodes/node_geo_cube_grid_topology.cc
  nodes/node_geo_curve_endpoint_selection.cc
  nodes/node_geo_curve_fill.cc
  nodes/node_geo_curve_fillet.cc
  nodes/node_geo_curve_handle_type_selection.cc
  nodes/node_geo_curve_intersection.cc
  nodes/node_geo_curve_length.cc
  nodes/node_geo_curve_primitive_arc.cc
  nodes/node_geo_curve_primitive_bezier_segment.cc
  nodes/node_geo_curve_primitive_circle.cc
  nodes/node_geo_curve_primitive_line.cc
  nodes/node_geo_curve_primitive_quadratic_bezier.cc
  nodes/node_geo_curve_primitive_quadrilateral.cc
  nodes/node_geo_curve_primitive_spiral.cc
  nodes/node_geo_curve_primitive_star.cc
  nodes/node_geo_curve_resample.cc
  nodes/node_geo_curve_reverse.cc
  nodes/node_geo_curve_sample.cc
  nodes/node_geo_curve_set_handle_type.cc
  nodes/node_geo_curve_spline_parameter.cc
  nodes/node_geo_curve_spline_type.cc
  nodes/node_geo_curve_subdivide.cc
  nodes/node_geo_curve_to_mesh.cc
  nodes/node_geo_curve_to_points.cc
  nodes/node_geo_curve_topology_curve_of_point.cc
  nodes/node_geo_curve_topology_points_of_curve.cc
  nodes/node_geo_curve_trim.cc
  nodes/node_geo_curves_to_grease_pencil.cc
  nodes/node_geo_deform_curves_on_surface.cc
  nodes/node_geo_debug.cc
  nodes/node_geo_delete_geometry.cc
  nodes/node_geo_distribute_points_in_grid.cc
  nodes/node_geo_distribute_points_in_volume.cc
  nodes/node_geo_distribute_points_on_faces.cc
  nodes/node_geo_dual_mesh.cc
  nodes/node_geo_duplicate_elements.cc
  nodes/node_geo_edge_paths_to_curves.cc
  nodes/node_geo_edge_paths_to_selection.cc
  nodes/node_geo_edge_split.cc
  nodes/node_geo_edges_to_face_groups.cc
  nodes/node_geo_enable_input.cc
  nodes/node_geo_enable_output.cc
  nodes/node_geo_evaluate_at_index.cc
  nodes/node_geo_evaluate_closure.cc
  nodes/node_geo_evaluate_on_domain.cc
  nodes/node_geo_expression.cc
  nodes/node_geo_write_at_index.cc
  nodes/node_geo_extrude_mesh.cc
  nodes/node_geo_spom_prisms.cc
  nodes/node_geo_field_average.cc
  nodes/node_geo_field_min_and_max.cc
  nodes/node_geo_field_to_grid.cc
  nodes/node_geo_field_to_list.cc
  nodes/node_geo_field_variance.cc
  nodes/node_geo_filter_list.cc
  nodes/node_geo_flip_solver.cc
  nodes/node_geo_flip_faces.cc
  nodes/node_geo_foreach_geometry_element.cc
  nodes/node_geo_geometry_clip.cc
  nodes/node_geo_geometry_materials.cc
  nodes/node_geo_geometry_to_instance.cc
  nodes/node_geo_get_attribute_names.cc
  nodes/node_geo_get_bundle_item.cc
  nodes/node_geo_get_geometry_bundle.cc
  nodes/node_geo_get_geometry_component.cc
  nodes/node_geo_get_named_grid.cc
  nodes/node_geo_get_nested_bundle_paths.cc
  nodes/node_geo_gizmo_dial.cc
  nodes/node_geo_gizmo_linear.cc
  nodes/node_geo_gizmo_transform.cc
  nodes/node_geo_guide_geometry.cc
  nodes/node_geo_grease_pencil_color.cc
  nodes/node_geo_grease_pencil_draw_time.cc
  nodes/node_geo_grease_pencil_fill_id.cc
  nodes/node_geo_grease_pencil_opacity.cc
  nodes/node_geo_grease_pencil_stroke_softness.cc
  nodes/node_geo_grease_pencil_stroke_visibility.cc
  nodes/node_geo_grease_pencil_to_curves.cc
  nodes/node_geo_grid_advect.cc
  nodes/node_geo_grid_clip.cc
  nodes/node_geo_grid_curl.cc
  nodes/node_geo_grid_deactivate_voxels.cc
  nodes/node_geo_grid_dilate_erode.cc
  nodes/node_geo_grid_divergence.cc
  nodes/node_geo_grid_gradient.cc
  nodes/node_geo_grid_info.cc
  nodes/node_geo_grid_laplacian.cc
  nodes/node_geo_grid_mean.cc
  nodes/node_geo_grid_median.cc
  nodes/node_geo_grid_prune.cc
  nodes/node_geo_grid_solve_poisson.cc
  nodes/node_geo_grid_to_mesh.cc
  nodes/node_geo_grid_to_points.cc
  nodes/node_geo_grid_topology_boolean.cc
  nodes/node_geo_grid_voxelize.cc
  nodes/node_geo_image.cc
  nodes/node_geo_image_info.cc
  nodes/node_geo_image_texture.cc
  nodes/node_geo_import_csv.cc
  nodes/node_geo_import_obj.cc
  nodes/node_geo_import_ply.cc
  nodes/node_geo_import_spz.cc
  nodes/node_geo_import_stl.cc
  nodes/node_geo_import_text.cc
  nodes/node_geo_import_vdb.cc
  nodes/node_geo_index_of_nearest.cc
  nodes/node_geo_index_switch.cc
  nodes/node_geo_input_active_camera.cc
  nodes/node_geo_input_collection.cc
  nodes/node_geo_input_curve_handles.cc
  nodes/node_geo_input_curve_tilt.cc
  nodes/node_geo_input_edge_smooth.cc
  nodes/node_geo_input_face_smooth.cc
  nodes/node_geo_input_font.cc
  nodes/node_geo_input_id.cc
  nodes/node_geo_input_index.cc
  nodes/node_geo_input_instance_bounds.cc
  nodes/node_geo_input_instance_reference.cc
  nodes/node_geo_input_instance_rotation.cc
  nodes/node_geo_input_instance_scale.cc
  nodes/node_geo_input_instance_transform.cc
  nodes/node_geo_input_material.cc
  nodes/node_geo_input_material_index.cc
  nodes/node_geo_input_mesh_edge_angle.cc
  nodes/node_geo_input_mesh_edge_neighbors.cc
  nodes/node_geo_input_mesh_edge_vertices.cc
  nodes/node_geo_input_mesh_face_area.cc
  nodes/node_geo_input_mesh_face_is_planar.cc
  nodes/node_geo_input_mesh_face_neighbors.cc
  nodes/node_geo_input_mesh_island.cc
  nodes/node_geo_input_mesh_vertex_neighbors.cc
  nodes/node_geo_input_named_attribute.cc
  nodes/node_geo_input_named_layer_selection.cc
  nodes/node_geo_input_normal.cc
  nodes/node_geo_input_object.cc
  nodes/node_geo_input_position.cc
  nodes/node_geo_input_radius.cc
  nodes/node_geo_input_scene_time.cc
  nodes/node_geo_input_shortest_edge_paths.cc
  nodes/node_geo_input_spline_cyclic.cc
  nodes/node_geo_input_spline_length.cc
  nodes/node_geo_input_spline_resolution.cc
  nodes/node_geo_input_tangent.cc
  nodes/node_geo_input_voxel_index.cc
  nodes/node_geo_instance_on_points.cc
  nodes/node_geo_instances_to_points.cc
  nodes/node_geo_interpolate_curves.cc
  nodes/node_geo_is_viewport.cc
  nodes/node_geo_join_bundle.cc
  nodes/node_geo_join_geometry.cc
  nodes/node_geo_k_nearest.cc
  nodes/node_geo_rbf_interpolate.cc
  nodes/node_geo_linear_solver.cc
  intern/linear_solver_math.cc
  intern/linear_solver_math.hh
  intern/sparse_matrix_math.cc
  nodes/node_geo_heat_geodesic.cc
  intern/mesh_heat_geodesic.cc
  intern/mesh_heat_geodesic.hh
  nodes/node_geo_list_get_item.cc
  nodes/node_geo_list_length.cc
  nodes/node_geo_loop_subdivision.cc
  nodes/node_geo_instant_meshes.cc
  nodes/node_geo_triangle_remesh.cc
  nodes/node_geo_quadwild.cc
  nodes/node_geo_mesh_graph_coloring.cc
  nodes/node_geo_tangent_field.cc
  nodes/node_geo_mesh_divergence.cc
  nodes/node_geo_mesh_gradient.cc
  nodes/node_geo_mesh_laplacian.cc
  nodes/node_geo_material_replace.cc
  nodes/node_geo_material_selection.cc
  nodes/node_geo_menu_switch.cc
  nodes/node_geo_merge_by_distance.cc
  nodes/node_geo_merge_layers.cc
  nodes/node_geo_merge_points.cc
  nodes/node_geo_mesh_bevel.cc
  nodes/node_geo_mesh_face_group_boundaries.cc
  nodes/node_geo_mesh_primitive_circle.cc
  nodes/node_geo_mesh_primitive_cone.cc
  nodes/node_geo_mesh_primitive_cube.cc
  nodes/node_geo_mesh_primitive_cylinder.cc
  nodes/node_geo_mesh_primitive_grid.cc
  nodes/node_geo_mesh_primitive_ico_sphere.cc
  nodes/node_geo_mesh_primitive_line.cc
  nodes/node_geo_mesh_primitive_uv_sphere.cc
  nodes/node_geo_mesh_subdivide.cc
  nodes/node_geo_mesh_to_curve.cc
  nodes/node_geo_mesh_to_density_grid.cc
  nodes/node_geo_mesh_to_points.cc
  nodes/node_geo_mesh_to_sdf_grid.cc
  nodes/node_geo_mesh_to_volume.cc
  nodes/node_geo_mesh_topology_corners_of_edge.cc
  nodes/node_geo_mesh_topology_corners_of_face.cc
  nodes/node_geo_mesh_topology_corners_of_vertex.cc
  nodes/node_geo_mesh_topology_edges_of_corner.cc
  nodes/node_geo_mesh_topology_edges_of_vertex.cc
  nodes/node_geo_mesh_topology_face_of_corner.cc
  nodes/node_geo_mesh_topology_offset_corner_in_face.cc
  nodes/node_geo_mesh_topology_vertex_of_corner.cc
  nodes/node_geo_modal_event.cc
  nodes/node_geo_modal_timer.cc
  nodes/node_geo_mouse_position.cc
  nodes/node_geo_nurbs_order.cc
  nodes/node_geo_nurbs_weight.cc
  nodes/node_geo_object_info.cc
  nodes/node_geo_offset_point_in_curve.cc
  nodes/node_geo_points.cc
  nodes/node_geo_points_to_curves.cc
  nodes/node_geo_points_to_sdf_grid.cc
  nodes/node_geo_points_to_vertices.cc
  nodes/node_geo_points_to_volume.cc
  nodes/node_geo_proximity.cc
  nodes/node_geo_python.cc
  nodes/node_geo_rasterize_points.cc
  nodes/node_geo_raycast.cc
  nodes/node_geo_realize_instances.cc
  nodes/node_geo_remove_attribute.cc
  nodes/node_geo_rename_attribute.cc
  nodes/node_geo_repeat.cc
  nodes/node_geo_rotate_instances.cc
  nodes/node_geo_sample_grid.cc
  nodes/node_geo_sample_grid_index.cc
  nodes/node_geo_sample_index.cc
  nodes/node_geo_sample_nearest.cc
  nodes/node_geo_sample_nearest_surface.cc
  nodes/node_geo_sample_sound_frequencies.cc
  nodes/node_geo_sample_uv_surface.cc
  nodes/node_geo_scale_elements.cc
  nodes/node_geo_select_elements.cc
  nodes/node_geo_edit_elements.cc
  nodes/node_geo_scale_instances.cc
  nodes/node_geo_sdf_grid_boolean.cc
  nodes/node_geo_sdf_grid_fillet.cc
  nodes/node_geo_sdf_grid_laplacian.cc
  nodes/node_geo_sdf_grid_mean.cc
  nodes/node_geo_sdf_grid_mean_curvature.cc
  nodes/node_geo_sdf_grid_median.cc
  nodes/node_geo_sdf_grid_offset.cc
  nodes/node_geo_self_object.cc
  nodes/node_geo_separate_bundle.cc
  nodes/node_geo_separate_components.cc
  nodes/node_geo_separate_geometry.cc
  nodes/node_geo_set_closure_default.cc
  nodes/node_geo_set_group_input_default.cc
  nodes/node_geo_set_curve_handles.cc
  nodes/node_geo_set_curve_normal.cc
  nodes/node_geo_set_curve_radius.cc
  nodes/node_geo_set_curve_tilt.cc
  nodes/node_geo_set_geometry_bundle.cc
  nodes/node_geo_set_geometry_name.cc
  nodes/node_geo_set_grease_pencil_color.cc
  nodes/node_geo_set_grease_pencil_depth_mode.cc
  nodes/node_geo_set_grease_pencil_softness.cc
  nodes/node_geo_set_grid_background.cc
  nodes/node_geo_set_grid_transform.cc
  nodes/node_geo_set_id.cc
  nodes/node_geo_set_instance_transform.cc
  nodes/node_geo_set_material.cc
  nodes/node_geo_set_material_index.cc
  nodes/node_geo_set_mesh_normal.cc
  nodes/node_geo_set_nurbs_order.cc
  nodes/node_geo_set_nurbs_weight.cc
  nodes/node_geo_set_point_radius.cc
  nodes/node_geo_set_points_type.cc
  nodes/node_geo_set_position.cc
  nodes/node_geo_set_shade_smooth.cc
  nodes/node_geo_set_spline_cyclic.cc
  nodes/node_geo_set_spline_resolution.cc
  nodes/node_geo_simulation.cc
  nodes/node_geo_sort_elements.cc
  nodes/node_geo_sort_list.cc
  nodes/node_geo_sparse_matrix_math.cc
  nodes/node_geo_split_to_instances.cc
  nodes/node_geo_store_bundle_item.cc
  nodes/node_geo_store_named_attribute.cc
  nodes/node_geo_store_named_portal.cc
  nodes/node_geo_store_named_grid.cc
  nodes/node_geo_string_join.cc
  nodes/node_geo_string_to_curves.cc
  nodes/node_geo_subdivision_surface.cc
  nodes/node_geo_switch.cc
  nodes/node_geo_tag_filter.cc
  nodes/node_geo_tool_3d_cursor.cc
  nodes/node_geo_tool_active_element.cc
  nodes/node_geo_tool_face_set.cc
  nodes/node_geo_tool_selection.cc
  nodes/node_geo_tool_set_face_set.cc
  nodes/node_geo_tool_set_selection.cc
  nodes/node_geo_time_shift.cc
  nodes/node_geo_transfer_attributes.cc
  nodes/node_geo_transform_geometry.cc
  nodes/node_geo_translate_instances.cc
  nodes/node_geo_triangulate.cc
  nodes/node_geo_uv_pack_islands.cc
  nodes/node_geo_uv_tangent.cc
  nodes/node_geo_uv_unwrap.cc
  nodes/node_geo_viewer.cc
  nodes/node_geo_viewport_camera.cc
  nodes/node_geo_viewport_transform.cc
  nodes/node_geo_volume_cube.cc
  nodes/node_geo_volume_to_mesh.cc
  nodes/node_geo_voronoi_fracture.cc
  nodes/node_geo_warning.cc
  nodes/node_geo_wrangle.cc
  nodes/node_geo_xpbd_solver.cc
  nodes/node_geo_box_engine_solver.cc
  nodes/node_geo_box_engine_set_rigid_body.cc
  nodes/node_geo_box_engine_set_constraint.cc
  nodes/node_geo_jolt_solver.cc
  nodes/node_geo_jolt_set_rigid_body.cc
  nodes/node_geo_jolt_set_constraint.cc
  intern/deformable_solver_shared.hh

  include/NOD_geo_bake.hh
  include/NOD_geo_bundle.hh
  include/NOD_geo_capture_attribute.hh
  include/NOD_geo_closure.hh
  include/NOD_geo_closure_to_list.hh
  include/NOD_geo_combine_list.hh
  include/NOD_geo_field_to_grid.hh
  include/NOD_geo_field_to_list.hh
  include/NOD_geo_foreach_geometry_element.hh
  include/NOD_geo_index_switch.hh
  include/NOD_geo_menu_switch.hh
  include/NOD_geo_rasterize_points.hh
  include/NOD_geo_repeat.hh
  include/NOD_geo_simulation.hh
  include/NOD_geo_tag_filter.hh
  include/NOD_geo_viewer.hh

  node_geometry_tree.cc
  node_geometry_util.cc

  node_geometry_util.hh
)

set(LIB
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::blenloader
  PRIVATE bf::blentranslation
  PRIVATE bf::bmesh
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::functions
  PRIVATE bf::geometry
  PRIVATE bf::gpu
  PRIVATE bf::imbuf
  PRIVATE bf::imbuf::movie
  PRIVATE bf::intern::guardedalloc
  bf_nodes
  bf_compositor
  bf_io_common
  bf_io_csv
  PRIVATE bf::intern::atomic
  PRIVATE bf::extern::xxhash
  PRIVATE bf::render
  PRIVATE bf::windowmanager
  PRIVATE bf::dependencies::optional::gmp
  PRIVATE bf::dependencies::optional::bullet
  PRIVATE bf::dependencies::eigen
)

if(WITH_IO_SPZ)
  list(APPEND LIB
    PRIVATE bf_io_common
    PRIVATE bf_io_spz
  )
  add_definitions(-DWITH_IO_SPZ)
endif()

if(WITH_IO_STL)
  list(APPEND LIB
    PRIVATE bf_io_common
    PRIVATE bf_io_stl
  )
  add_definitions(-DWITH_IO_STL)
endif()

if(WITH_IO_PLY)
  list(APPEND LIB
    PRIVATE bf_io_common
    PRIVATE bf_io_ply
  )
  add_definitions(-DWITH_IO_PLY)
endif()

if(WITH_IO_WAVEFRONT_OBJ)
  list(APPEND LIB
    PRIVATE bf_io_common
    PRIVATE bf_io_wavefront_obj
  )
  add_definitions(-DWITH_IO_WAVEFRONT_OBJ)
endif()

if(WITH_BULLET)
  list(APPEND INC
    ../../../../intern/rigidbody
  )
endif()

if(WITH_OPENSUBDIV)
  add_definitions(-DWITH_OPENSUBDIV)
endif()

if(WITH_PYTHON)
  list(APPEND INC
    ../../python
  )
  add_definitions(-DWITH_PYTHON)
endif()

add_node_discovery(
  bf_nodes_geometry_generated
  "${SRC}"
  ${CMAKE_CURRENT_BINARY_DIR}/register_geometry_nodes.cc
  register_geometry_nodes
)

list(APPEND LIB
  bf_nodes_geometry_generated
)

# If compiling with MSVC clang we need to add the `D_LIBCPP_VERSION` define
# so we don't run into TBB errors when compiling with lib.
if(WITH_TBB AND MSVC_CLANG)
  string(APPEND CMAKE_CXX_FLAGS " /D_LIBCPP_VERSION")
endif()



if(WITH_VORO_PLUSPLUS)
  list(APPEND INC
    ../../../../extern/voro++/src
  )
  list(APPEND LIB
    PRIVATE bf::extern::voroplusplus
  )
  add_definitions(-DWITH_VORO_PLUSPLUS)
endif()

if(WITH_CGAL)
  list(APPEND INC
    ../../../../intern/cgal_bridge
  )
  list(APPEND LIB
    PRIVATE bf::intern::cgal_bridge
  )
  add_definitions(-DWITH_CGAL)
endif()

if(WITH_BOX2D)
  list(APPEND INC
    ../../../../extern/box2d/include
  )
  list(APPEND LIB
    PRIVATE bf::extern::box2d
  )
  add_definitions(-DWITH_BOX2D)
endif()

if(WITH_BOX3D)
  list(APPEND INC
    ../../../../extern/box3d/include
  )
  list(APPEND LIB
    PRIVATE bf::extern::box3d
  )
  add_definitions(-DWITH_BOX3D)
endif()

if(WITH_JOLT)
  list(APPEND INC
    ../../../../intern/jolt_bridge/include
  )
  list(APPEND LIB
    PRIVATE bf::intern::jolt_bridge
  )
  add_definitions(-DWITH_JOLT)
endif()

blender_add_lib(bf_nodes_geometry "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
target_link_libraries(bf_nodes_geometry_generated bf_nodes_geometry)

# Explicit names so MSVC incremental/unity builds cannot drop the cores.
if(WITH_BOX2D)
  target_link_libraries(bf_nodes_geometry PRIVATE extern_box2d)
endif()
if(WITH_BOX3D)
  target_link_libraries(bf_nodes_geometry PRIVATE extern_box3d)
endif()
if(WITH_JOLT)
  target_link_libraries(bf_nodes_geometry PRIVATE bf_intern_jolt_bridge)
endif()
blender_set_target_unity_build(bf_nodes_geometry 10)

# RNA_prototypes.hh
add_dependencies(bf_nodes_geometry bf_rna)
