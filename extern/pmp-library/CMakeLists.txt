# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Polygon Mesh Processing Library (Sieger & Botsch)
# Upstream: https://github.com/pmp-library/pmp-library  (MIT)
# Core + remeshing algorithms only — no viewers / I/O / examples.

# Avoid noisy warnings from third-party code.
if((CMAKE_C_COMPILER_ID STREQUAL "GNU") OR (CMAKE_C_COMPILER_ID MATCHES "Clang"))
  add_cxx_flag("-Wno-unused-result")
  remove_cxx_flag("-Wmissing-declarations")
endif()

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
  src/pmp/surface_mesh.cpp
  src/pmp/surface_mesh.h
  src/pmp/types.h
  src/pmp/mat_vec.h
  src/pmp/properties.h
  src/pmp/exceptions.h
  src/pmp/bounding_box.h

  src/pmp/algorithms/remeshing.cpp
  src/pmp/algorithms/remeshing.h
  src/pmp/algorithms/normals.cpp
  src/pmp/algorithms/normals.h
  src/pmp/algorithms/differential_geometry.cpp
  src/pmp/algorithms/differential_geometry.h
  src/pmp/algorithms/distance_point_triangle.cpp
  src/pmp/algorithms/distance_point_triangle.h
  src/pmp/algorithms/barycentric_coordinates.h
  src/pmp/algorithms/curvature.cpp
  src/pmp/algorithms/curvature.h
  src/pmp/algorithms/laplace.cpp
  src/pmp/algorithms/laplace.h
  src/pmp/algorithms/numerics.cpp
  src/pmp/algorithms/numerics.h
  src/pmp/algorithms/features.cpp
  src/pmp/algorithms/features.h
)

set(LIB
  PRIVATE bf::dependencies::eigen
)

blender_add_lib(extern_pmp "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")

if(MSVC)
  # Quench known noisy warnings in vendor sources.
  target_compile_options(extern_pmp PRIVATE /wd4244 /wd4267 /wd4018 /wd4146 /wd4305 /wd4996)
endif()
