QuadWild-BiMDF package for Blender Geometry Nodes
==================================================

Source: https://github.com/cgg-bern/quadwild-bimdf (GPL-3)
Paper:  Reliable Feature-Line Driven Quad-Remeshing (Pietroni et al., SIGGRAPH 2021)
        https://github.com/nicopietroni/quadwild
Addon reference (same algorithm + defaults):
        https://github.com/ksami/QRemeshify  (QRemeshify / Gumroad)

This folder must sit next to blender.exe:

  blender.exe
  quadwild/
    quadwild.exe
    quad_from_patches.exe
    config/...

Geometry Nodes → Mesh → Operations → QuadWild

Optional: set environment variable QUADWILD_DIR to an alternate package path.

Pipeline (QRemeshify-aligned)
-----------------------------
  1) Prep (fixed alpha=0.01, scaleFact=1; sharp from feature mode):
       quadwild <mesh.obj> 2 <prep.txt>
  2) Bi-MDF QR (Alpha/Scale sockets, align singularities, satsuma/default.json):
       quad_from_patches <mesh_rem_p0.obj> 0 <qr_main_config.txt>
     → input_rem_p0_0_quadrangulation[_smooth].obj

Config samples under config/prep_config and config/main_config match the
upstream release. The Geometry Node writes temporary prep + QR configs from
socket parameters (matching QRemeshify QRParameters) instead of using a static
flow_noalign_lemon.txt by default.
