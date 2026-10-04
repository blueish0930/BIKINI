# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: MIT
#
# Compact XPBD core for Geometry Nodes.

if((CMAKE_C_COMPILER_ID STREQUAL "GNU") OR (CMAKE_C_COMPILER_ID MATCHES "Clang"))
  remove_c_and_cxx_flag("-Wextra" "-Wall" "-Wunused")
endif()
if(WIN32)
  add_definitions(-D_USE_MATH_DEFINES)
  add_definitions(-DNOMINMAX)
endif()

set(INC include)
set(INC_SYS)
set(SRC
  src/pbd.cc
  include/pbd/pbd.h
)
set(LIB)
blender_add_lib(extern_pbd "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
add_library(bf::extern::pbd ALIAS extern_pbd)
if(MSVC)
  target_compile_options(extern_pbd PRIVATE /wd4244 /wd4267 /wd4305 /wd4018)
endif()
target_include_directories(extern_pbd PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
