QuadWild-BiMDF package for Blender Geometry Nodes
==================================================

Source: https://github.com/cgg-bern/quadwild-bimdf (GPL-3)
Paper:  Reliable Feature-Line Driven Quad-Remeshing (Pietroni et al., SIGGRAPH 2021)
        https://github.com/nicopietroni/quadwild

This folder must sit next to blender.exe:

  blender.exe
  quadwild/
    quadwild.exe
    quad_from_patches.exe
    config/...

Geometry Nodes → Mesh → Operations → QuadWild

Optional: set environment variable QUADWILD_DIR to an alternate package path.

Config samples under config/prep_config and config/main_config match the
upstream release. The Geometry Node writes a temporary prep config from
socket parameters and runs:

  quadwild <mesh.obj> 2 <prep.txt>
  quad_from_patches <mesh_rem_p0.obj> 1 config/main_config/flow_noalign_lemon.txt
