# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Directional (avaxman/Directional) — header-only directional field library (MPL-2.0).

add_library(extern_directional INTERFACE)
add_library(bf::extern::directional ALIAS extern_directional)

target_include_directories(extern_directional
  SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    # Some Directional headers use #include "tree.h" (same-dir style).
    ${CMAKE_CURRENT_SOURCE_DIR}/include/directional
)

target_link_libraries(extern_directional
  INTERFACE
    bf::dependencies::eigen
)

# C++20 for std::numbers used by Directional headers.
target_compile_features(extern_directional INTERFACE cxx_std_20)
