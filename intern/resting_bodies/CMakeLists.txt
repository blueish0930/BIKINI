set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
add_compile_definitions(NOMINMAX _USE_MATH_DEFINES)
if(MSVC)
  add_compile_options(/utf-8 /bigobj)
endif()
set(GC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/deps/geometry-central")
set(QH_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/deps/qhull")
add_library(qhullstatic_r STATIC
  "${QH_ROOT}/src/libqhull_r/global_r.c"
  "${QH_ROOT}/src/libqhull_r/stat_r.c"
  "${QH_ROOT}/src/libqhull_r/geom2_r.c"
  "${QH_ROOT}/src/libqhull_r/poly2_r.c"
  "${QH_ROOT}/src/libqhull_r/merge_r.c"
  "${QH_ROOT}/src/libqhull_r/libqhull_r.c"
  "${QH_ROOT}/src/libqhull_r/geom_r.c"
  "${QH_ROOT}/src/libqhull_r/poly_r.c"
  "${QH_ROOT}/src/libqhull_r/qset_r.c"
  "${QH_ROOT}/src/libqhull_r/mem_r.c"
  "${QH_ROOT}/src/libqhull_r/random_r.c"
  "${QH_ROOT}/src/libqhull_r/usermem_r.c"
  "${QH_ROOT}/src/libqhull_r/userprintf_r.c"
  "${QH_ROOT}/src/libqhull_r/io_r.c"
  "${QH_ROOT}/src/libqhull_r/user_r.c"
  "${QH_ROOT}/src/libqhull_r/accessors_r.c"
  "${QH_ROOT}/src/libqhull_r/rboxlib_r.c"
  "${QH_ROOT}/src/libqhull_r/userprintf_rbox_r.c"
)
target_include_directories(qhullstatic_r PUBLIC "${QH_ROOT}/src")
add_library(qhullcpp STATIC
  "${QH_ROOT}/src/libqhullcpp/Coordinates.cpp"
  "${QH_ROOT}/src/libqhullcpp/PointCoordinates.cpp"
  "${QH_ROOT}/src/libqhullcpp/Qhull.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullFacet.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullFacetList.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullFacetSet.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullHyperplane.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullPoint.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullPointSet.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullPoints.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullQh.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullRidge.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullSet.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullStat.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullUser.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullVertex.cpp"
  "${QH_ROOT}/src/libqhullcpp/QhullVertexSet.cpp"
  "${QH_ROOT}/src/libqhullcpp/RboxPoints.cpp"
  "${QH_ROOT}/src/libqhullcpp/RoadError.cpp"
  "${QH_ROOT}/src/libqhullcpp/RoadLogEvent.cpp"
)
target_include_directories(qhullcpp PUBLIC "${QH_ROOT}/src")
set(GC_SOURCES
  surface/surface_mesh.cpp
  surface/manifold_surface_mesh.cpp
  surface/base_geometry_interface.cpp
  surface/intrinsic_geometry_interface.cpp
  surface/extrinsic_geometry_interface.cpp
  surface/embedded_geometry_interface.cpp
  surface/vertex_position_geometry.cpp
  surface/surface_point.cpp
  surface/halfedge_factories.cpp
  surface/mesh_graph_algorithms.cpp
  numerical/linear_algebra_utilities.cpp
  numerical/suitesparse_utilities.cpp
  numerical/linear_solvers.cpp
  numerical/qr_solvers.cpp
  numerical/square_solvers.cpp
  numerical/positive_definite_solvers.cpp
  utilities/utilities.cpp
  utilities/quaternion.cpp
  utilities/disjoint_sets.cpp
  utilities/elementary_geometry.cpp)
list(TRANSFORM GC_SOURCES PREPEND "${GC_ROOT}/src/")
add_library(resting_geometry STATIC ${GC_SOURCES})
target_include_directories(resting_geometry PUBLIC "${GC_ROOT}/include"
  "${LIBDIR}/eigen/include/eigen3")
add_library(resting_core STATIC src/utils.cpp src/arc_algebra.cpp
  src/forward3D.cpp src/geometry_utils.cpp src/boundary_tools.cpp
  src/convex_hull.cpp src/ambient_conversions.cpp)
target_include_directories(resting_core PUBLIC include "${QH_ROOT}/src")
target_link_libraries(resting_core PUBLIC resting_geometry qhullcpp qhullstatic_r)
add_library(bf_intern_resting_bridge SHARED resting_bridge.cpp)
target_compile_definitions(bf_intern_resting_bridge PRIVATE RESTING_BRIDGE_BUILD)
target_link_libraries(bf_intern_resting_bridge PRIVATE resting_core)

target_include_directories(bf_intern_resting_bridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(resting_geometry resting_core bf_intern_resting_bridge PROPERTIES UNITY_BUILD OFF)
if(MSVC)
  foreach(target resting_geometry resting_core bf_intern_resting_bridge qhullcpp qhullstatic_r)
    target_compile_options(${target} PRIVATE /EHsc /utf-8 /bigobj)
    set_target_properties(${target} PROPERTIES VS_GLOBAL_VcpkgEnabled false)
  endforeach()
endif()
set_target_properties(bf_intern_resting_bridge PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}/$<CONFIG>"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${EXECUTABLE_OUTPUT_PATH}/Release")
install(TARGETS bf_intern_resting_bridge RUNTIME DESTINATION . COMPONENT Blender)
