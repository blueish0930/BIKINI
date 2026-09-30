# SPDX-FileCopyrightText: 2026 BIKINI Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Native LuxCore render engine (intern, like Cycles). Not a Python addon.
# The Apache-2.0 LuxCore/SLG/LuxRays sources are compiled as intern/luxcore/kernel.

add_subdirectory(kernel)

set(INC
  .
  blender
  kernel/include
  ../../source/blender/makesrna
  ${CMAKE_BINARY_DIR}/source/blender/makesrna
  ../../source/blender/render
  ../../source/blender/gpu
  ../../source/blender/gpu/intern
  ../../source/blender/nodes
  ../../source/blender/nodes/intern
  ../../source/blender/nodes/shader
  ../../source/blender/editors/include
  ../../source/blender/makesdna
  ../../source/blender/blenkernel
  ../../source/blender/windowmanager
)

set(INC_SYS
)

set(SRC
  blender/engine.cc
  blender/session.cc
  blender/sync.cc
  blender/nodes.cc
  blender/node_tree.cc

  LUX_api.h
  blender/session.hh
  blender/sync.hh
  blender/nodes.hh
)

set(LIB
  PRIVATE bf::blenkernel
  PRIVATE bf::blenlib
  PRIVATE bf::blentranslation
  PRIVATE bf::depsgraph
  PRIVATE bf::dna
  PRIVATE bf::gpu
  PRIVATE bf::imbuf
  PRIVATE bf::intern::clog
  PRIVATE bf::intern::guardedalloc
  PRIVATE bf::nodes
)

if(TARGET bf_intern_luxcore_kernel)
  add_definitions(-DWITH_LUXCORE_SDK)
  if(EXISTS "${LUXCORE_SOURCE_DIR}/include")
    list(APPEND INC_SYS
      ${LUXCORE_SOURCE_DIR}/include
      ${LUXCORE_DEPS_DIR}/boost/1.88.0/Release/x86_64/include
    )
  endif()
  list(APPEND LIB bf_intern_luxcore_kernel)
  message(STATUS "LuxCore intern: compiling full SLG/LuxRays kernel into blender.exe")
else()
  message(STATUS "LuxCore intern: kernel not available, engine registers as a stub")
endif()

blender_add_lib(bf_intern_luxcore "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
add_dependencies(bf_intern_luxcore bf_rna)

if(TARGET bf_intern_luxcore_kernel)
  add_dependencies(bf_intern_luxcore bf_intern_luxcore_kernel)
  set_property(TARGET bf_intern_luxcore PROPERTY CXX_STANDARD 20)
  if(EXISTS "${LIBDIR}/imath/lib/Imath.lib")
    target_link_libraries(bf_intern_luxcore PUBLIC "${LIBDIR}/imath/lib/Imath.lib")
  endif()
endif()
