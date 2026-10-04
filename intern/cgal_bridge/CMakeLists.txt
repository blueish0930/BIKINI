# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

set(INC
  .
)

set(INC_SYS
  ../../extern/cgal/include
  ../../extern/boost
)

# Eigen (Blender lib) for PMP smooth_shape / sparse solvers.
if(EXISTS ${LIBDIR}/eigen)
  list(APPEND INC_SYS ${LIBDIR}/eigen)
elseif(EXISTS ${CMAKE_SOURCE_DIR}/lib/windows_x64/eigen)
  list(APPEND INC_SYS ${CMAKE_SOURCE_DIR}/lib/windows_x64/eigen)
endif()

set(SRC
  cgal_bridge.hh
  cgal_convex_clip3.hh
  cgal_fail_guard.hh
  cgal_types.hh
  cgal_mesh_io.hh
  cgal_mesh_io.cc
  ops/cgal_convex_hull.cc
  ops/cgal_mesh_ops.cc
  ops/cgal_mesh_extra.cc
  ops/cgal_mesh_batch.cc
  ops/cgal_alpha_shape.cc
  ops/cgal_points_extra.cc
  ops/cgal_points_batch.cc
  ops/cgal_mesh_advanced.cc
  ops/cgal_mesh_batch15.cc
  ops/cgal_mesh_batch16.cc
  ops/cgal_mesh_batch17.cc
  ops/cgal_batch18.cc
  ops/cgal_batch19.cc
  ops/cgal_batch20.cc
  ops/cgal_batch21.cc
  ops/cgal_batch22.cc
  ops/cgal_batch24.cc
  ops/cgal_batch25.cc
  ops/cgal_batch26.cc
  ops/cgal_batch27.cc
  ops/cgal_batch28.cc
  ops/cgal_batch29.cc
  ops/cgal_batch30.cc
  ops/cgal_batch31.cc
  ops/cgal_batch32.cc
  ops/cgal_batch33.cc
  ops/cgal_batch34.cc
  ops/cgal_batch35.cc
  ops/cgal_batch36.cc
  ops/cgal_batch37.cc
  ops/cgal_batch38.cc
  ops/cgal_batch39.cc
  ops/cgal_batch40.cc
  ops/cgal_batch41.cc
  ops/cgal_batch42.cc
  ops/cgal_batch43.cc
  ops/cgal_batch44.cc
  ops/cgal_batch45.cc
  ops/cgal_batch47.cc
  ops/cgal_batch48.cc
  ops/cgal_batch49.cc
  ops/cgal_batch50.cc
  ops/cgal_batch51.cc
  ops/cgal_batch52.cc
  ops/cgal_batch53.cc
  ops/cgal_batch54.cc
  ops/cgal_batch55.cc
  ops/cgal_batch56.cc
  ops/cgal_batch57.cc
  ops/cgal_overlay_2.cc

)

set(LIB
  PRIVATE bf::extern::cgal
  PRIVATE bf::dependencies::eigen
)

# Shared library so CGAL template instantiations do not bloat blender.exe
# (same pattern as draco_bridge / meshoptimizer_bridge).
add_library(bf_intern_cgal_bridge SHARED "${SRC}")
blender_link_libraries(bf_intern_cgal_bridge "${LIB}")
blender_source_group(bf_intern_cgal_bridge "${SRC}")
blender_target_include_dirs(bf_intern_cgal_bridge ${INC})
blender_target_include_dirs_sys(bf_intern_cgal_bridge ${INC_SYS})
add_library(bf::intern::cgal_bridge ALIAS bf_intern_cgal_bridge)

if(WIN32)
  set_target_properties(bf_intern_cgal_bridge PROPERTIES VS_GLOBAL_VcpkgEnabled "false")
  # Export all C++ symbols for consumers (geometry / nodes).
  set_target_properties(bf_intern_cgal_bridge PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

target_compile_definitions(bf_intern_cgal_bridge PRIVATE CGAL_EIGEN3_ENABLED=1)

# Public include for GEO / nodes that #include "cgal_bridge.hh".
target_include_directories(bf_intern_cgal_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

if(MSVC)
  target_compile_options(bf_intern_cgal_bridge PRIVATE /bigobj)
  # Prefer smaller code for heavily template-instantiated CGAL.
  target_compile_options(bf_intern_cgal_bridge PRIVATE
    $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>,$<CONFIG:MinSizeRel>>:/Os /Gy /Gw /Zc:inline>
  )
endif()

# Place DLL next to blender.exe (MSVC multi-config: bin/Release).
if(EXECUTABLE_OUTPUT_PATH)
  set_target_properties(bf_intern_cgal_bridge PROPERTIES
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

if(APPLE)
  set_target_properties(bf_intern_cgal_bridge PROPERTIES
    XCODE_ATTRIBUTE_SKIP_INSTALL "YES"
    XCODE_ATTRIBUTE_INSTALL_PATH ""
  )
elseif(UNIX AND PLATFORM_BUNDLED_LIBRARIES AND WITH_INSTALL_PORTABLE)
  set_target_properties(bf_intern_cgal_bridge PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
  )
endif()

set_property(TARGET bf_intern_cgal_bridge PROPERTY UNITY_BUILD OFF)

# Ensure the DLL is installed/copied with the app when using install targets.
if(WIN32)
  install(TARGETS bf_intern_cgal_bridge
    RUNTIME DESTINATION .
    COMPONENT Blender
  )
endif()
