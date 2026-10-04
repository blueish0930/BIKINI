# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# IMAGE_NODES_MVP: GPU image node tree module (Compositor/Copernicus-style).
# Remove this whole directory + CMake add_subdirectory/link when reverting.

set(INC
  .
  ..
  include
  ../intern
  ../geometry/include
  ../../blenloader
  ../../blenloader_core
  ../../editors/include
  ../../compositor
  ../../compositor/algorithms
  ../../compositor/cached_resources
  ../../compositor/derived_resources
  ../../compositor/utilities
  ../../draw
  ../../gpu/intern
  ../../makesrna
  ../../render/intern

  # RNA_prototypes.hh
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  nodes/node_image_brightness_contrast.cc
  nodes/node_image_bake_image.cc
  nodes/node_image_combine_bundle_com.cc
  nodes/node_image_flip.cc
  nodes/node_image_fft.cc
  nodes/node_image_fluid_sim.cc
  nodes/node_image_height_to_normal.cc
  nodes/node_image_histogram.cc
  nodes/node_image_import_points.cc
  nodes/node_image_import_geo.cc
  nodes/node_image_normal_to_height.cc
  nodes/node_image_point_stamp.cc
  nodes/node_image_rasterize_geometry.cc
  nodes/node_image_sdf_shape.cc
  nodes/node_image_geo_sdf.cc
  nodes/node_image_fractal_primitive.cc
  nodes/node_image_shadertoy.cc
  nodes/node_image_terrain_primitive.cc
  nodes/node_image_terrain_erosion.cc
  nodes/node_image_terrain_simulate.cc
  nodes/node_image_terrain_modify.cc
  nodes/node_image_terrain_derive.cc
  nodes/node_image_terrain_color.cc
  nodes/node_image_group.cc
  nodes/node_image_paint.cc
  nodes/node_image_gradient.cc
  nodes/node_image_island_uv.cc
  nodes/node_image_island_padding.cc
  nodes/node_image_divergence.cc
  nodes/node_image_sample_at_pixel.cc
  nodes/node_image_write_at_pixel.cc
  nodes/node_image_scene_frame.cc
  nodes/node_image_camera_view.cc
  nodes/node_image_render_material.cc
  nodes/node_image_viewer.cc
  nodes/node_image_file_output.cc

  image_nodes_evaluate.cc
  image_points.cc
  node_image_tree.cc
  node_image_util.cc

  include/NOD_image_bake.hh
  include/NOD_image_fluid.hh
  include/NOD_image_fluid_solver.hh
  include/NOD_image_points.hh
  include/NOD_image_paint.hh
  node_image_terrain_common.hh
  node_image_util.hh
)

set(LIB
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::functions
  PRIVATE bf::geometry
  PRIVATE bf::gpu
  PRIVATE bf::imbuf
  PRIVATE bf::intern::guardedalloc
  bf_compositor
  PRIVATE bf::render
  PRIVATE bf::windowmanager
  PRIVATE bf::dependencies::optional::fftw3
)

add_node_discovery(
  bf_nodes_image_generated
  "${SRC}"
  ${CMAKE_CURRENT_BINARY_DIR}/register_image_nodes.cc
  register_image_nodes
)

list(APPEND LIB
  bf_nodes_image_generated
)

blender_add_lib(bf_nodes_image "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
target_link_libraries(bf_nodes_image_generated bf_nodes_image)

# RNA_prototypes.hh
add_dependencies(bf_nodes_image bf_rna)
