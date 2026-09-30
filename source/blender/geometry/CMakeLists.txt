# SPDX-FileCopyrightText: 2006 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  PUBLIC .
  PUBLIC xpbd
  ../makesrna
  ../../../intern/eigen
  ../draw/intern
  ../gpu
)

set(INC_SYS
)

set(SRC
  intern/add_curves_on_mesh.cc
  intern/curve_constraints.cc
  intern/curves_remove_and_split.cc
  intern/extend_curves.cc
  intern/extract_elements.cc
  intern/fillet_curves.cc
  intern/fit_curves.cc
  intern/flip_solver.cc
  intern/flip_solver_collision.cc
  intern/foreach_geometry.cc
  intern/interpolate_curves.cc
  intern/join_geometries.cc
  intern/merge_curves.cc
  intern/merge_layers.cc
  intern/mesh_bevel.cc
  intern/mesh_boolean.cc
  intern/mesh_boolean_manifold.cc
  intern/mesh_clip_plane.cc
  intern/mesh_copy_selection.cc
  intern/mesh_laplacian.cc
  intern/mesh_instant_meshes.cc
  intern/mesh_triangle_remesh.cc
  intern/mesh_cgal.cc
  intern/mesh_make_it_stand.cc
  intern/mesh_winding_number.cc
  intern/mesh_quadwild.cc
  intern/mesh_graph_coloring.cc
  intern/mesh_tangent_field.cc
  intern/mesh_loop_subdivide.cc
  intern/mesh_merge_verts.cc
  intern/mesh_primitive_cuboid.cc
  intern/mesh_primitive_cylinder_cone.cc
  intern/mesh_primitive_grid.cc
  intern/mesh_primitive_line.cc
  intern/mesh_primitive_uv_sphere.cc
  intern/mesh_selection.cc
  intern/mesh_split_edges.cc
  intern/mesh_to_curve_convert.cc
  intern/mesh_to_volume.cc
  intern/mesh_triangulate.cc
  intern/mix_geometries.cc
  intern/point_merge.cc
  intern/points_to_volume.cc
  intern/randomize.cc
  intern/realize_instances.cc
  intern/reorder.cc
  intern/resample_curves.cc
  intern/reverse_uv_sampler.cc
  intern/separate_geometry.cc
  intern/set_curve_type.cc
  intern/simplify_curves.cc
  intern/smooth_curves.cc
  intern/subdivide_curves.cc
  intern/transform.cc
  intern/trim_curves.cc
  intern/uv_pack.cc
  intern/uv_parametrizer.cc
  intern/volume_grid_resample.cc

  xpbd/intern/xpbd_constraint_coloring.cc

  GEO_add_curves_on_mesh.hh
  GEO_curve_constraints.hh
  GEO_curves_remove_and_split.hh
  GEO_extend_curves.hh
  GEO_extract_elements.hh
  GEO_fillet_curves.hh
  GEO_fit_curves.hh
  GEO_flip_solver.hh
  intern/flip_solver_private.hh
  GEO_foreach_geometry.hh
  GEO_grid_samplers.hh
  GEO_interpolate_curves.hh
  GEO_join_geometries.hh
  GEO_merge_curves.hh
  GEO_merge_layers.hh
  GEO_mesh_bevel.hh
  GEO_mesh_boolean.hh
  GEO_mesh_clip_plane.hh
  GEO_mesh_copy_selection.hh
  GEO_mesh_laplacian.hh
  GEO_mesh_instant_meshes.hh
  GEO_mesh_triangle_remesh.hh
  GEO_cgal.hh
  GEO_make_it_stand.hh
  GEO_winding_number.hh
  GEO_mesh_quadwild.hh
  GEO_mesh_graph_coloring.hh
  GEO_mesh_tangent_field.hh
  GEO_mesh_loop_subdivide.hh
  GEO_mesh_merge_verts.hh
  GEO_mesh_primitive_cuboid.hh
  GEO_mesh_primitive_cylinder_cone.hh
  GEO_mesh_primitive_grid.hh
  GEO_mesh_primitive_line.hh
  GEO_mesh_primitive_uv_sphere.hh
  GEO_mesh_selection.hh
  GEO_mesh_split_edges.hh
  GEO_mesh_to_curve.hh
  GEO_mesh_to_volume.hh
  GEO_mesh_triangulate.hh
  GEO_mix_geometries.hh
  GEO_point_merge.hh
  GEO_points_to_volume.hh
  GEO_randomize.hh
  GEO_realize_instances.hh
  GEO_reorder.hh
  GEO_resample_curves.hh
  GEO_reverse_uv_sampler.hh
  GEO_separate_geometry.hh
  GEO_set_curve_type.hh
  GEO_simplify_curves.hh
  GEO_smooth_curves.hh
  GEO_subdivide_curves.hh
  GEO_transform.hh
  GEO_trim_curves.hh
  GEO_uv_pack.hh
  GEO_uv_parametrizer.hh
  GEO_volume_grid_resample.hh

  xpbd/GEO_xpbd_constraint_align_rotations.hh
  xpbd/GEO_xpbd_constraint_collision_edge.hh
  xpbd/GEO_xpbd_constraint_collision_face.hh
  xpbd/GEO_xpbd_constraint_coloring.hh
  xpbd/GEO_xpbd_constraint_coloring_utils.hh
  xpbd/GEO_xpbd_constraint_damping_angular.hh
  xpbd/GEO_xpbd_constraint_damping_linear.hh
  xpbd/GEO_xpbd_constraint_distance.hh
  xpbd/GEO_xpbd_constraint_friction_edge.hh
  xpbd/GEO_xpbd_constraint_friction_face.hh
  xpbd/GEO_xpbd_constraint_math.hh
  xpbd/GEO_xpbd_constraint_pin_position.hh
  xpbd/GEO_xpbd_constraint_pin_rotation.hh
  xpbd/GEO_xpbd_constraint_rod_bend_twist.hh
  xpbd/GEO_xpbd_constraint_rod_stretch_shear.hh
  xpbd/GEO_xpbd_constraint_set.hh
  xpbd/GEO_xpbd_constraint_set_params.hh
  xpbd/GEO_xpbd_constraint_set_templated.hh
  xpbd/GEO_xpbd_geometry_ref.hh
  xpbd/GEO_xpbd_updater_gauss_seidel.hh
  xpbd/GEO_xpbd_updater_velocity.hh

  intern/mesh_boolean_intern.hh
  intern/mesh_boolean_manifold.hh
)

set(LIB
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::blentranslation
  PRIVATE bf::bmesh
  PRIVATE bf::dna
  PRIVATE bf::functions
  PRIVATE bf::nodes
  PRIVATE bf::intern::atomic
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::intern::profile
  PRIVATE bf::extern::curve_fit_nd
  PRIVATE bf::dependencies::optional::manifold
  PRIVATE bf::dependencies::optional::gmp
)

if(WITH_INSTANT_MESHES)
  list(APPEND INC
    ../../../intern/instant_meshes
  )
  list(APPEND LIB
    bf_intern_instant_meshes
  )
  add_definitions(-DWITH_INSTANT_MESHES)
endif()

if(WITH_PMP_REMESH)
  list(APPEND INC
    ../../../intern/pmp_remesh
  )
  list(APPEND LIB
    bf_intern_pmp_remesh
  )
  add_definitions(-DWITH_PMP_REMESH)
endif()

if(WITH_CGAL)
  list(APPEND INC
    ../../../intern/cgal_bridge
    ../../../intern/cgal_volume_bridge
  )
  list(APPEND LIB
    bf_intern_cgal_bridge
    bf_intern_cgal_volume
  )
  add_definitions(-DWITH_CGAL)
endif()

if(WITH_DIRECTIONAL)
  list(APPEND LIB
    bf::extern::directional
  )
  add_definitions(-DWITH_DIRECTIONAL)
endif()

if(WITH_UV_SLIM)
  list(APPEND INC
    ../../../intern/slim
  )
  add_definitions(-DWITH_UV_SLIM)
endif()

if(WITH_MANIFOLD)
  add_definitions(-DWITH_MANIFOLD)
endif()

blender_add_lib(bf_geometry "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
add_library(bf::geometry ALIAS bf_geometry)

if(WITH_GTESTS)
  set(TEST_INC
  )
  set(TEST_SRC
    tests/GEO_interpolate_curves_test.cc
    tests/GEO_merge_curves_test.cc
    tests/GEO_mesh_laplacian_test.cc
    tests/GEO_realize_instances_test.cc
  )
  set(TEST_LIB
    PRIVATE bf::intern::clog
  )
  blender_add_test_suite_lib(bf_geometry_tests "${TEST_SRC}" "${INC};${TEST_INC}" "${INC_SYS}" "${LIB};${TEST_LIB}")
endif()
