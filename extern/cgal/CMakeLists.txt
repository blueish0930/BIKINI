# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# CGAL (Computational Geometry Algorithms Library) — header-only vendor.
# Upstream: https://github.com/CGAL/cgal  (v6.2 library release)
# Licenses: GPL-3.0-or-later / LGPL / commercial (see LICENSE*)

# Interface library: consumers only need includes + optional GMP/MPFR.

set(INC
  include
)

set(INC_SYS
  # Boost headers (BSL) — required by CGAL config.hpp
  ${CMAKE_SOURCE_DIR}/extern/boost
)

# Dummy translation unit so the target is a real static lib (MSVC needs an
# object for some dependency edges) while remaining header-only for algorithms.
set(SRC
  blender_cgal_stub.cc
)

set(LIB
)

# EPICK (exact predicates, inexact constructions) does not require GMP/MPFR.
# Enabling CGAL_USE_GMP pulls Boost.Multiprecision GMP backends that currently
# fail to compile against Blender's vendored GMP + Boost 1.86 on MSVC.
# Full EPECK + MPFR can be re-enabled once that stack is sorted.

blender_add_lib(extern_cgal "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
add_library(bf::extern::cgal ALIAS extern_cgal)

target_include_directories(extern_cgal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/extern/boost>
)

target_compile_definitions(extern_cgal PUBLIC
  CGAL_HEADER_ONLY=1
  CGAL_DISABLE_GMP=1
)

if(WIN32)
  target_compile_definitions(extern_cgal PUBLIC
    NOMINMAX
    _USE_MATH_DEFINES
  )
endif()

if(MSVC)
  # CGAL templates routinely exceed default object section limits.
  target_compile_options(extern_cgal PUBLIC /bigobj /wd4244 /wd4267 /wd4018 /wd4146 /wd4305 /wd4996)
endif()
