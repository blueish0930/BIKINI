QRemeshify package for Blender Geometry Nodes (QuadWild)
========================================================

This folder is the full QRemeshify runtime used by Geometry Nodes → QuadWild.

Upstream addon: https://github.com/ksami/QRemeshify
Libraries:      QuadWild-BiMDF (https://github.com/cgg-bern/quadwild-bimdf)
Paper:          Reliable Feature-Line Driven Quad-Remeshing (SIGGRAPH 2021)

Required layout (next to blender.exe):

  blender.exe
  quadwild/
    lib_quadwild.dll
    lib_quadpatches.dll
    config/
      main_config/
      prep_config/
      satsuma/

Pipeline (identical to QRemeshify Python bindings):
  1) remeshAndField2  → mesh_rem.obj + field
  2) trace2           → mesh_rem_p0.obj
  3) quadPatches      → mesh_rem_p0_0_quadrangulation[_smooth].obj

Optional: set environment variable QUADWILD_DIR to this package path.

CLI tools (quadwild.exe / quad_from_patches.exe) are optional leftovers and
are NOT used by the Geometry Node when the DLLs above are present.
