# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Shared library so extern_pmp is not statically linked into blender.exe
# (same pattern as bf_intern_cgal_bridge / quadriflow).

set(INC
  .
)

set(INC_SYS
  ../../extern/pmp-library/src
)

set(SRC
  pmp_remesh_capi.cpp
  pmp_remesh_capi.hpp
)

set(LIB
  PRIVATE bf::dependencies::eigen
  extern_pmp
)

add_library(bf_intern_pmp_remesh SHARED "${SRC}")
blender_link_libraries(bf_intern_pmp_remesh "${LIB}")
blender_source_group(bf_intern_pmp_remesh "${SRC}")
blender_target_include_dirs(bf_intern_pmp_remesh ${INC})
blender_target_include_dirs_sys(bf_intern_pmp_remesh ${INC_SYS})
add_library(bf::intern::pmp_remesh ALIAS bf_intern_pmp_remesh)

if(WIN32)
  set_target_properties(bf_intern_pmp_remesh PROPERTIES VS_GLOBAL_VcpkgEnabled "false")
  set_target_properties(bf_intern_pmp_remesh PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

target_include_directories(bf_intern_pmp_remesh PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

if(MSVC)
  target_compile_options(bf_intern_pmp_remesh PRIVATE
    $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>,$<CONFIG:MinSizeRel>>:/Os /Gy /Gw /Zc:inline>
  )
endif()

if(EXECUTABLE_OUTPUT_PATH)
  set_target_properties(bf_intern_pmp_remesh PROPERTIES
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
  set_target_properties(bf_intern_pmp_remesh PROPERTIES
    XCODE_ATTRIBUTE_SKIP_INSTALL "YES"
    XCODE_ATTRIBUTE_INSTALL_PATH ""
  )
elseif(UNIX AND PLATFORM_BUNDLED_LIBRARIES AND WITH_INSTALL_PORTABLE)
  set_target_properties(bf_intern_pmp_remesh PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
  )
endif()

if(WIN32)
  install(TARGETS bf_intern_pmp_remesh
    RUNTIME DESTINATION .
    COMPONENT Blender
  )
endif()
