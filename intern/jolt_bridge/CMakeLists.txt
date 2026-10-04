# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Shared library wrapping Jolt so bf_nodes_geometry never compiles Jolt headers.

set(INC
  .
  include
)

set(SRC
  jolt.cc
  include/jolt_bridge.h
)

if(TARGET Jolt::Jolt)
  set(LIB Jolt::Jolt)
  set(INC_SYS)
else()
  set(LIB Jolt)
  set(INC_SYS ../../extern/jolt/JoltPhysics)
endif()

add_library(bf_intern_jolt_bridge SHARED "${SRC}")
blender_link_libraries(bf_intern_jolt_bridge "${LIB}")
blender_source_group(bf_intern_jolt_bridge "${SRC}")
blender_target_include_dirs(bf_intern_jolt_bridge ${INC})
if(INC_SYS)
  blender_target_include_dirs_sys(bf_intern_jolt_bridge ${INC_SYS})
endif()
add_library(bf::intern::jolt_bridge ALIAS bf_intern_jolt_bridge)

target_compile_definitions(bf_intern_jolt_bridge PRIVATE JOLT_BRIDGE_EXPORTS)

if(WIN32)
  set_target_properties(bf_intern_jolt_bridge PROPERTIES VS_GLOBAL_VcpkgEnabled "false")
  set_target_properties(bf_intern_jolt_bridge PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

set_target_properties(bf_intern_jolt_bridge PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  UNITY_BUILD OFF
)

target_include_directories(bf_intern_jolt_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if(MSVC)
  target_compile_options(bf_intern_jolt_bridge PRIVATE
    /W3 /wd4244 /wd4267 /wd4305 /wd4996 /wd4018 /bigobj
  )
endif()

if(EXECUTABLE_OUTPUT_PATH)
  set_target_properties(bf_intern_jolt_bridge PROPERTIES
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
  install(TARGETS bf_intern_jolt_bridge
    RUNTIME DESTINATION .
    COMPONENT Blender
  )
endif()
