# SPDX-FileCopyrightText: 2006 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

add_subdirectory(atomic)
add_subdirectory(clog)
add_subdirectory(ghost)
add_subdirectory(guardedalloc)
add_subdirectory(libmv)
add_subdirectory(memutil)
add_subdirectory(opensubdiv)
add_subdirectory(profile)
add_subdirectory(mikktspace)
add_subdirectory(eigen)
add_subdirectory(sky)

add_subdirectory(openvdb)

if(WITH_MOD_REMESH)
  add_subdirectory(dualcon)
endif()

if(WITH_IK_SOLVER)
  add_subdirectory(iksolver)
endif()

if(WITH_IK_ITASC)
  add_subdirectory(itasc)
endif()

if(WITH_CYCLES)
  add_subdirectory(cycles)
endif()

# === BIKINI LuxCore Begin ===
if(WITH_LUXCORE)
  add_subdirectory(luxcore)
endif()
# === BIKINI LuxCore End ===

if(WITH_BULLET)
  add_subdirectory(rigidbody)
endif()


if(WIN32)
  # Only windows needs utf16 converter.
  add_subdirectory(utfconv)

  # Only used for Windows for now.
  add_subdirectory(uriconvert)
endif()

if(WITH_MOD_FLUID)
  add_subdirectory(mantaflow)
endif()

if(WITH_UV_SLIM)
  add_subdirectory(slim)
endif()

if(WITH_QUADRIFLOW)
  add_subdirectory(quadriflow)
endif()

if(WITH_INSTANT_MESHES)
  add_subdirectory(instant_meshes)
endif()

if(WITH_PMP_REMESH)
  add_subdirectory(pmp_remesh)
endif()

if(WITH_CGAL)
  add_subdirectory(cgal_bridge)
  add_subdirectory(cgal_volume_bridge)
endif()

if(WITH_JOLT)
  # Jolt library first so the bridge can link the `Jolt` target (intern is
  # processed before extern/).
  add_subdirectory(${CMAKE_SOURCE_DIR}/extern/jolt ${CMAKE_BINARY_DIR}/extern/jolt)
  add_subdirectory(jolt_bridge)
endif()

if(UNIX AND NOT APPLE)
  add_subdirectory(libc_compat)
endif()

if(WITH_RENDERDOC)
  add_subdirectory(renderdoc_dynload)
endif()

if(UNIX AND NOT APPLE)
  # Important this comes after "ghost" as it uses includes defined by GHOST's CMake.
  if(WITH_GHOST_WAYLAND AND WITH_GHOST_WAYLAND_DYNLOAD)
    add_subdirectory(wayland_dynload)
  endif()
  if(WITH_GHOST_DBUS AND WITH_GHOST_DBUS_DYNLOAD)
    add_subdirectory(dbus_dynload)
  endif()
endif()

if(WITH_DRACO)
  add_subdirectory(draco_bridge)
endif()

if(WITH_MESHOPTIMIZER)
  add_subdirectory(meshoptimizer_bridge)
endif()
