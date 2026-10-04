# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  .
  ..
  ../intern
  ../../blenloader
  ../../editors/include
  ../../makesrna

  # RNA_prototypes.hh
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  nodes/node_object_create.cc
  nodes/node_object_delete.cc
  nodes/node_object_group.cc
  nodes/node_object_input.cc
  nodes/node_object_modifiers.cc
  nodes/node_object_output.cc
  nodes/node_object_parent.cc
  nodes/node_object_set_material_slots.cc
  nodes/node_object_set_transform.cc
  nodes/node_object_set_visibility.cc
  nodes/node_object_viewport_display.cc

  object_nodes_evaluate.cc
  node_object_tree.cc
  node_object_util.cc

  node_object_util.hh
)

set(LIB
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::blenloader
  PRIVATE bf::blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::functions
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::windowmanager
)

add_node_discovery(
  bf_nodes_object_generated
  "${SRC}"
  ${CMAKE_CURRENT_BINARY_DIR}/register_object_nodes.cc
  register_object_nodes
)

list(APPEND LIB
  bf_nodes_object_generated
)

blender_add_lib(bf_nodes_object "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
target_link_libraries(bf_nodes_object_generated bf_nodes_object)

# RNA_prototypes.hh
add_dependencies(bf_nodes_object bf_rna)
