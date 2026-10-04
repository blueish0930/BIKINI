# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  ../include
  ../../makesrna
  ../../nodes/geometry/include

  # RNA_prototypes.hh
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
)

set(INC_SYS
)

set(SRC
  geometry_edit.cc
  geometry_edit_draw.cc
  geometry_edit_ops.cc
  geometry_edit_select.cc

  geometry_edit_intern.hh
)

set(LIB
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::bmesh
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::gpu
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::nodes
  PRIVATE bf::windowmanager
)

blender_add_lib(bf_editor_geometry_edit "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

add_dependencies(bf_editor_geometry_edit bf_rna)
