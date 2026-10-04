# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Instant Field-Aligned Meshes (Jakob et al., SIGGRAPH Asia 2015)
# Upstream: https://github.com/wjakob/instant-meshes  (BSD-3-Clause)
# Core library only — no NanoGUI / viewer / file I/O CLI.

# Avoid noisy warnings from third-party code.
if((CMAKE_C_COMPILER_ID STREQUAL "GNU") OR (CMAKE_C_COMPILER_ID MATCHES "Clang"))
  add_cxx_flag("-Wno-unused-result")
  remove_cxx_flag("-Wmissing-declarations")
endif()

if(WIN32)
  add_definitions(-D_USE_MATH_DEFINES)
  add_definitions(-D__TBB_NO_IMPLICIT_LINKAGE)
  add_definitions(-DNOMINMAX)
endif()

set(INC
  src
  ext/dset
  ext/pcg32
  ext/pss
  ext/rply
)

set(INC_SYS
)

set(SRC
  src/aabb.h
  src/adjacency.cpp
  src/adjacency.h
  src/bvh.cpp
  src/bvh.h
  src/cleanup.cpp
  src/cleanup.h
  src/common.h
  src/dedge.cpp
  src/dedge.h
  src/extract.cpp
  src/extract.h
  src/field.cpp
  src/field.h
  src/hierarchy.cpp
  src/hierarchy.h
  src/meshstats.cpp
  src/meshstats.h
  src/normal.cpp
  src/normal.h
  src/reorder.cpp
  src/reorder.h
  src/serializer.cpp
  src/serializer.h
  src/subdivide.cpp
  src/subdivide.h
  ext/rply/rply.c
  ext/rply/rply.h
  ext/dset/dset.h
  ext/pcg32/pcg32.h
  ext/pss/parallel_stable_sort.h
  ext/pss/pss_common.h
)

set(LIB
  PRIVATE bf::dependencies::eigen
  PRIVATE bf::dependencies::optional::tbb
)

blender_add_lib(extern_instant_meshes "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

if(MSVC)
  # Quench known noisy warnings in vendor sources.
  target_compile_options(extern_instant_meshes PRIVATE /wd4244 /wd4267 /wd4018 /wd4146 /wd4305)
  set_source_files_properties(ext/rply/rply.c PROPERTIES COMPILE_FLAGS "/wd4127")
endif()
