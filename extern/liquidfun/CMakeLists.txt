# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: MIT
#
# Compact 2D particle fluid / elastic core (LiquidFun-style) for Geometry Nodes.

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
  src/liquidfun.cc
  include/liquidfun/liquidfun.h
)
set(LIB)
blender_add_lib(extern_liquidfun "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
add_library(bf::extern::liquidfun ALIAS extern_liquidfun)
if(MSVC)
  target_compile_options(extern_liquidfun PRIVATE /wd4244 /wd4267 /wd4305 /wd4018)
endif()
target_include_directories(extern_liquidfun PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
