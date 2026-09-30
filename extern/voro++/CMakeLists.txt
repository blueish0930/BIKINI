# SPDX-FileCopyrightText: 2008 The Regents of the University of California
#                         (Lawrence Berkeley National Laboratory)
# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Voro++ (Chris H. Rycroft) — 3D Voronoi cell library.
# Upstream: https://github.com/chr1shr/voro

# Quiet third-party warnings.
if((CMAKE_C_COMPILER_ID STREQUAL "GNU") OR (CMAKE_C_COMPILER_ID MATCHES "Clang"))
  remove_c_and_cxx_flag(
    "-Wextra"
    "-Wall"
    "-Wunused"
  )
endif()

# Silence Voro++ status prints in library builds.
add_definitions(-DVOROPP_VERBOSE=0)

if(WIN32)
  add_definitions(-D_USE_MATH_DEFINES)
  add_definitions(-DNOMINMAX)
endif()

set(INC
  src
)

set(INC_SYS
)

set(SRC
  src/cell.cc
  src/cell.hh
  src/common.cc
  src/common.hh
  src/config.hh
  src/container.cc
  src/container.hh
  src/container_prd.cc
  src/container_prd.hh
  src/c_loops.cc
  src/c_loops.hh
  src/pre_container.cc
  src/pre_container.hh
  src/rad_option.hh
  src/unitcell.cc
  src/unitcell.hh
  src/voro++.hh
  src/v_base.cc
  src/v_base.hh
  src/v_compute.cc
  src/v_compute.hh
  src/wall.cc
  src/wall.hh
  src/worklist.hh
  # NOTE: v_base_wl.cc is #include'd at the end of v_base.cc — do not compile it alone.
)

set(LIB
)

blender_add_lib(extern_voroplusplus "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
add_library(bf::extern::voroplusplus ALIAS extern_voroplusplus)

if(MSVC)
  target_compile_options(extern_voroplusplus PRIVATE /wd4244 /wd4267 /wd4305 /wd4018)
endif()

target_include_directories(extern_voroplusplus PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
