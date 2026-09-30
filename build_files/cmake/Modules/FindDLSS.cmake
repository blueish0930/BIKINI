# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: BSD-3-Clause

# Find the DLSS SDK. This modules defines
#  DLSS_INCLUDE_DIR, where to find the NGX headers for DLSS
#  DLSS_FOUND, if the DLSS SDK is found.

if(DLSS_SDK_ROOT AND NOT EXISTS "${DLSS_SDK_ROOT}/include/nvsdk_ngx.h")
  unset(DLSS_SDK_ROOT CACHE)
endif()
if(DLSS_INCLUDE_DIR AND NOT EXISTS "${DLSS_INCLUDE_DIR}/nvsdk_ngx.h")
  unset(DLSS_INCLUDE_DIR CACHE)
endif()
if(NOT DLSS_SDK_ROOT)
  if(DEFINED LIBDIR AND EXISTS "${LIBDIR}/dlss/include/nvsdk_ngx.h")
    set(DLSS_SDK_ROOT "${LIBDIR}/dlss" CACHE PATH
        "Path to the NVIDIA DLSS SDK (directory containing include/nvsdk_ngx.h)")
  elseif(EXISTS "${CMAKE_SOURCE_DIR}/lib/windows_x64/dlss/include/nvsdk_ngx.h")
    set(DLSS_SDK_ROOT "${CMAKE_SOURCE_DIR}/lib/windows_x64/dlss" CACHE PATH
        "Path to the NVIDIA DLSS SDK (directory containing include/nvsdk_ngx.h)")
  endif()
endif()

find_path(DLSS_INCLUDE_DIR
    NAMES
        "nvsdk_ngx.h"
    PATHS
        "${DLSS_SDK_ROOT}/include"
        "$ENV{DLSS_SDK_ROOT}/include"
        "${LIBDIR}/dlss/include"
        "${CMAKE_SOURCE_DIR}/lib/windows_x64/dlss/include"
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(DLSS REQUIRED_VARS DLSS_INCLUDE_DIR)
