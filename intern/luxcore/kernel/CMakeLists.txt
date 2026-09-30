# SPDX-FileCopyrightText: 2026 BIKINI Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Compile the full LuxCore / SLG / LuxRays C++ engine as a Blender intern
# static library (same pattern as intern/cycles). Not a Python addon.

set(_lux_src_default "${CMAKE_SOURCE_DIR}/extern/luxcore")
set(_lux_deps_default "${_lux_src_default}/out/dependencies/full_deploy/host")
if(NOT LUXCORE_SOURCE_DIR OR NOT EXISTS "${LUXCORE_SOURCE_DIR}/src/luxcore/luxcore.cpp")
  set(LUXCORE_SOURCE_DIR "${_lux_src_default}" CACHE PATH "LuxCore engine sources" FORCE)
endif()
if(NOT LUXCORE_DEPS_DIR OR
   NOT EXISTS "${LUXCORE_DEPS_DIR}/boost/1.88.0/Release/x86_64/include/boost/version.hpp")
  set(LUXCORE_DEPS_DIR "${_lux_deps_default}" CACHE PATH
      "LuxCore extra deps (Boost, spdlog, nlohmann, robin_hood)" FORCE)
endif()

if(NOT EXISTS "${LUXCORE_SOURCE_DIR}/src/luxcore/luxcore.cpp")
  message(WARNING "LuxCore sources not found at ${LUXCORE_SOURCE_DIR}")
  return()
endif()

set(_LUX_BOOST_INC "${LUXCORE_DEPS_DIR}/boost/1.88.0/Release/x86_64/include")
set(_LUX_BOOST_LIB "${LUXCORE_DEPS_DIR}/boost/1.88.0/Release/x86_64/lib")
set(_LUX_SPDLOG_INC "${LUXCORE_DEPS_DIR}/spdlog/1.17.0/include")
set(_LUX_JSON_INC "${LUXCORE_DEPS_DIR}/nlohmann_json/3.12.0/include")
set(_LUX_ROBIN_INC "${LUXCORE_DEPS_DIR}/robin-hood-hashing/3.11.5/include")

if(NOT EXISTS "${_LUX_BOOST_INC}/boost/version.hpp")
  message(WARNING "LuxCore Boost 1.88 not found (${_LUX_BOOST_INC}). Run luxmake deps.")
  return()
endif()

set(_GEN "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(_GEN_INC "${_GEN}/include")
set(_GEN_LUXRAYS_K "${_GEN}/luxrays-kernels")
set(_GEN_SLG_K "${_GEN}/slg-kernels")
file(MAKE_DIRECTORY "${_GEN_INC}/luxcore" "${_GEN_INC}/luxrays")
file(MAKE_DIRECTORY "${_GEN_LUXRAYS_K}" "${_GEN_SLG_K}")

function(luxcore_preprocess_kernels dest_dir namespace cl_files out_var)
  set(_outs)
  foreach(kernel IN LISTS ${cl_files})
    get_filename_component(_fname "${kernel}" NAME)
    string(REGEX REPLACE "\\.cl$" "" _kname "${_fname}")
    set(_dst "${dest_dir}/${_kname}_kernel.cpp")
    add_custom_command(
      OUTPUT "${_dst}"
      COMMAND ${CMAKE_COMMAND}
        -DDST=${_dst} -DSRC=${kernel}
        -DNAMESPACE=${namespace} -DKERNEL=${_kname}
        -P "${LUXCORE_SOURCE_DIR}/build-system/cmake/Scripts/PreprocessKernel.cmake"
      MAIN_DEPENDENCY "${kernel}"
      DEPENDS
        "${LUXCORE_SOURCE_DIR}/build-system/cmake/Scripts/PreprocessKernel.cmake"
        "${LUXCORE_SOURCE_DIR}/build-system/cmake/Scripts/PreprocessedKernel.template"
      COMMENT "LuxCore preprocess ${_kname}.cl"
    )
    list(APPEND _outs "${_dst}")
  endforeach()
  set(${out_var} "${_outs}" PARENT_SCOPE)
endfunction()

file(GLOB_RECURSE _LUXRAYS_CL CONFIGURE_DEPENDS "${LUXCORE_SOURCE_DIR}/include/luxrays/*.cl")
file(GLOB_RECURSE _SLG_CL CONFIGURE_DEPENDS "${LUXCORE_SOURCE_DIR}/include/slg/*.cl")
luxcore_preprocess_kernels("${_GEN_LUXRAYS_K}" luxrays _LUXRAYS_CL _LUXRAYS_KCPP)
luxcore_preprocess_kernels("${_GEN_SLG_K}" slg _SLG_CL _SLG_KCPP)

file(GLOB_RECURSE _LUXRAYS_SRC CONFIGURE_DEPENDS "${LUXCORE_SOURCE_DIR}/src/luxrays/*.cpp")
file(GLOB_RECURSE _SLG_SRC CONFIGURE_DEPENDS "${LUXCORE_SOURCE_DIR}/src/slg/*.cpp")
file(GLOB _LUXCORE_SRC CONFIGURE_DEPENDS "${LUXCORE_SOURCE_DIR}/src/luxcore/*.cpp")
list(FILTER _LUXCORE_SRC EXCLUDE REGEX "luxcoreimpl\\.cpp$")
file(GLOB _BCD_SRC CONFIGURE_DEPENDS "${LUXCORE_SOURCE_DIR}/deps/bcd-1.1/src/core/*.cpp")

set(_KERNEL_SRC
  ${_LUXRAYS_SRC}
  ${_SLG_SRC}
  ${_LUXCORE_SRC}
  ${_BCD_SRC}
  ${_LUXRAYS_KCPP}
  ${_SLG_KCPP}
  "${LUXCORE_SOURCE_DIR}/deps/clew/src/clew.cpp"
  "${LUXCORE_SOURCE_DIR}/deps/cuew/src/cuew.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/parselxs_stub.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/luxcoreimpl_wrap.cc"
)

add_library(bf_intern_luxcore_kernel STATIC ${_KERNEL_SRC})
set_property(TARGET bf_intern_luxcore_kernel PROPERTY CXX_STANDARD 20)
set_property(TARGET bf_intern_luxcore_kernel PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET bf_intern_luxcore_kernel PROPERTY UNITY_BUILD OFF)
set_property(TARGET bf_intern_luxcore_kernel PROPERTY LINKER_LANGUAGE CXX)

target_include_directories(bf_intern_luxcore_kernel
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${_GEN_INC}
    ${LUXCORE_SOURCE_DIR}/include
  PRIVATE
    ${LUXCORE_SOURCE_DIR}/src/luxcore
    ${LUXCORE_SOURCE_DIR}/src/luxcore/luxparser
    ${LUXCORE_SOURCE_DIR}/deps/bcd-1.1/include
    ${LUXCORE_SOURCE_DIR}/deps/bcd-1.1/include/bcd/core
    ${LUXCORE_SOURCE_DIR}/deps/clew/include
    ${LUXCORE_SOURCE_DIR}/deps/cuew/include
    ${LUXCORE_SOURCE_DIR}/deps/optix-7.1.0/include
    ${LUXCORE_SOURCE_DIR}/deps/lut
    ${_LUX_BOOST_INC}
    ${_LUX_SPDLOG_INC}
    ${_LUX_JSON_INC}
    ${_LUX_ROBIN_INC}
    ${LIBDIR}/OpenImageIO/include
    ${LIBDIR}/openexr/include
    ${LIBDIR}/openexr/include/OpenEXR
    ${LIBDIR}/imath/include
    ${LIBDIR}/imath/include/Imath
    ${LIBDIR}/opencolorio/include
    ${LIBDIR}/openvdb/include
    ${LIBDIR}/opensubdiv/include
    ${LIBDIR}/tbb/include
    ${LIBDIR}/embree/include
    ${LIBDIR}/openimagedenoise/include
    ${LIBDIR}/png/include
    ${LIBDIR}/jpeg/include
    ${LIBDIR}/openjpeg/include
    ${LIBDIR}/openjph/include
    ${LIBDIR}/pugixml/include
    ${LIBDIR}/fmt/include
    ${LIBDIR}/zlib/include
    ${LIBDIR}/eigen/include
    ${LIBDIR}/eigen/include/eigen3
    ${TBB_INCLUDE_DIRS}
    ${EMBREE_INCLUDE_DIRS}
    ${OPENVDB_INCLUDE_DIRS}
    ${OPENSUBDIV_INCLUDE_DIRS}
    ${OPENIMAGEDENOISE_INCLUDE_DIRS}
)

target_compile_definitions(bf_intern_luxcore_kernel
  PRIVATE
    NOMINMAX
    _CRT_SECURE_NO_WARNINGS
    _SCL_SECURE_NO_WARNINGS
    _USE_MATH_DEFINES
    __SSE2__
    __SSE__
    __MMX__
    CPP_API_EXPORTS
    LUXRAYS_DISABLE_CUDA
    LUXRAYS_DISABLE_OPTIX
    BOOST_ALL_NO_LIB
    BOOST_CONTAINER_NO_LIB
    SPDLOG_HEADER_ONLY
    SPDLOG_FMT_EXTERNAL
    IMATH_DLL
    OPENEXR_DLL
)

if(MSVC)
  # luxcore.cpp includes OpenImageIO before any other fmt header. OIIO then
  # compiles fmt header-only into that one object. See bikini_fmt_compiled.h.
  set_source_files_properties("${LUXCORE_SOURCE_DIR}/src/luxcore/luxcore.cpp"
    PROPERTIES COMPILE_OPTIONS "/FIbikini_fmt_compiled.h")
  target_compile_options(bf_intern_luxcore_kernel PRIVATE)
    /bigobj
    /EHsc
    /arch:AVX
    /openmp
    /FIomp.h
    /wd4244
    /wd4267
    /wd4305
    /wd4996
    /wd4018
    /wd4146
    /wd4251
    /wd4275
    /wd4800
    /wd5030
    /wd4100
    /wd4099
    /wd4250
    /wd4005
    /wd5038
  )
endif()

file(GLOB _BOOST_LIBS "${_LUX_BOOST_LIB}/libboost_*.lib")

set(_LUX_KERNEL_LIBS
  ${_BOOST_LIBS}
  ${LIBDIR}/OpenImageIO/lib/OpenImageIO.lib
  ${LIBDIR}/OpenImageIO/lib/OpenImageIO_Util.lib
  ${LIBDIR}/openexr/lib/OpenEXR.lib
  ${LIBDIR}/openexr/lib/OpenEXRCore.lib
  ${LIBDIR}/openexr/lib/Iex.lib
  ${LIBDIR}/openexr/lib/IlmThread.lib
  ${LIBDIR}/imath/lib/Imath.lib
  ${LIBDIR}/opencolorio/lib/OpenColorIO.lib
  ${LIBDIR}/openvdb/lib/openvdb.lib
  ${LIBDIR}/opensubdiv/lib/osdCPU.lib
  ${LIBDIR}/tbb/lib/tbb.lib
  ${LIBDIR}/embree/lib/embree4.lib
  ${LIBDIR}/openimagedenoise/lib/OpenImageDenoise.lib
  ${LIBDIR}/png/lib/libpng.lib
  ${LIBDIR}/jpeg/lib/libjpeg.lib
  ${LIBDIR}/openjpeg/lib/openjp2.lib
  ${LIBDIR}/pugixml/lib/pugixml.lib
  ${LIBDIR}/zlib/lib/libz_st.lib
)

if(EXISTS "${LIBDIR}/openjph/lib/openjph.0.25.lib")
  list(APPEND _LUX_KERNEL_LIBS "${LIBDIR}/openjph/lib/openjph.0.25.lib")
endif()
if(EXISTS "${LIBDIR}/openimagedenoise/lib/OpenImageDenoise_core.lib")
  list(APPEND _LUX_KERNEL_LIBS "${LIBDIR}/openimagedenoise/lib/OpenImageDenoise_core.lib")
endif()

if(EMBREE_LIBRARIES)
  list(APPEND _LUX_KERNEL_LIBS ${EMBREE_LIBRARIES})
endif()
if(TBB_LIBRARIES)
  list(APPEND _LUX_KERNEL_LIBS ${TBB_LIBRARIES})
endif()
if(OPENVDB_LIBRARIES)
  list(APPEND _LUX_KERNEL_LIBS ${OPENVDB_LIBRARIES})
endif()
if(OPENIMAGEDENOISE_LIBRARIES)
  list(APPEND _LUX_KERNEL_LIBS ${OPENIMAGEDENOISE_LIBRARIES})
endif()

target_link_libraries(bf_intern_luxcore_kernel PUBLIC ${_LUX_KERNEL_LIBS})

message(STATUS "LuxCore intern kernel: ${LUXCORE_SOURCE_DIR}")
