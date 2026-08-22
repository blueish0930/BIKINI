QRemeshify package for Geometry Nodes (QuadWild node)
=====================================================

This folder is a **full copy of the QRemeshify addon** used by the Geometry
Nodes "QuadWild" node. Remesh is NOT reimplemented in C++.

Layout:
  quadwild/
    qremeshify_bridge.py     # thin CLI glue (calls addon code)
    QRemeshify/              # exact addon (lib/, util/, operator logic via imports)
      lib/lib_quadwild.dll
      lib/lib_quadpatches.dll
      lib/data.py
      util/exporter.py
      ...
    config/                  # optional leftover CLI configs
    README.txt

Pipeline (same as https://github.com/ksami/QRemeshify ):
  Geometry Node → dump mesh OBJ →
  blender -b --python qremeshify_bridge.py -- ...
    → QRemeshify.util.exporter (sharp + mesh)
    → QRemeshify.lib.Quadwild.remeshAndField / trace / quadrangulate
  → load result OBJ

Environment:
  QUADWILD_DIR  — alternate package path
  QUADWILD_BRIDGE_CHILD=1 — set internally to block nested remesh

Source: Gumroad / github.com/ksami/QRemeshify (GPL-3)
