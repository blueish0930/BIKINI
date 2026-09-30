# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Isolated shared library for Mesh_3 / Kinetic / Nef / CDT3.
# Do NOT enable WINDOWS_EXPORT_ALL_SYMBOLS — Mesh_3 instantiations explode the COFF export table.

set(INC
  .
  ../cgal_bridge
)

set(INC_SYS
  ../../extern/cgal/include
  ../../extern/boost
)

if(EXISTS ${LIBDIR}/eigen)
  list(APPEND INC_SYS ${LIBDIR}/eigen)
elseif(EXISTS ${CMAKE_SOURCE_DIR}/lib/windows_x64/eigen)
  list(APPEND INC_SYS ${CMAKE_SOURCE_DIR}/lib/windows_x64/eigen)
endif()

set(SRC
  cgal_volume.cc
  cgal_volume_bridge.hh
)

set(LIB
  PRIVATE bf::extern::cgal
  PRIVATE bf::intern::cgal_bridge
  PRIVATE bf::dependencies::eigen
)

add_library(bf_intern_cgal_volume SHARED "${SRC}")
blender_link_libraries(bf_intern_cgal_volume "${LIB}")
blender_source_group(bf_intern_cgal_volume "${SRC}")
blender_target_include_dirs(bf_intern_cgal_volume ${INC})
blender_target_include_dirs_sys(bf_intern_cgal_volume ${INC_SYS})
add_library(bf::intern::cgal_volume ALIAS bf_intern_cgal_volume)

if(WIN32)
  set_target_properties(bf_intern_cgal_volume PROPERTIES VS_GLOBAL_VcpkgEnabled "false")
  # Isolated from the giant cgal_bridge DLL so Mesh_3 exports stay under 65535.
  set_target_properties(bf_intern_cgal_volume PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

target_compile_definitions(bf_intern_cgal_volume PRIVATE CGAL_EIGEN3_ENABLED=1)

if(MSVC)
  target_compile_options(bf_intern_cgal_volume PRIVATE /bigobj)
  target_compile_options(bf_intern_cgal_volume PRIVATE
    $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>,$<CONFIG:MinSizeRel>>:/Os /Gy /Gw /Zc:inline>
  )
endif()

if(EXECUTABLE_OUTPUT_PATH)
  set_target_properties(bf_intern_cgal_volume PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}/$<CONFIG>"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${EXECUTABLE_OUTPUT_PATH}/Release"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${EXECUTABLE_OUTPUT_PATH}/RelWithDebInfo"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${EXECUTABLE_OUTPUT_PATH}/Debug"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${EXECUTABLE_OUTPUT_PATH}/MinSizeRel"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/$<CONFIG>"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/$<CONFIG>"
  )
endif()

if(WIN32)
  install(TARGETS bf_intern_cgal_volume
    RUNTIME DESTINATION .
    COMPONENT Blender
  )
endif()

set_property(TARGET bf_intern_cgal_volume PROPERTY UNITY_BUILD OFF)
