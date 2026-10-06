# BIKINI

BIKINI is a source branch that tracks Blender's daily builds, focused on geometry nodes and
interaction. Feature notes, node documentation and the VEX Wrangle reference live on the
documentation site; this repository holds the source that can be compiled.

- Documentation: <https://blueish0930.github.io/BIKINI/>
- Source repository: <https://github.com/blueish0930/BIKINI>
- Building from source, and the third-party libraries you have to fetch: [BUILDING.txt](BUILDING.txt)
- Third-party license notes: [COPYING.md](COPYING.md)
- Upstream Blender build handbook: <https://developer.blender.org/docs/handbook/building_blender/>

## What the source tree contains

After cloning, these directories are already part of the Git history:

| Directory | Contents |
| --- | --- |
| `source/` | The Blender application itself: window manager, editors, geometry nodes, compositor, sequencer, Python API |
| `intern/` | Engines and bridges compiled with Blender, including Cycles, OpenVDB and Mantaflow, plus BIKINI's LuxCore integration (`intern/luxcore`) and the CGAL bridges (`intern/cgal_bridge`, `intern/cgal_volume_bridge`) |
| `extern/` | Third-party sources committed to this repository, for example Audaspace, Bullet, CGAL, Eigen, Jolt, OpenSubdiv and gtest. Each library keeps its own license files |
| `scripts/` | Startup scripts, bundled add-ons, presets and templates |
| `release/` | Runtime data. Full third-party license texts live in `release/license/` |
| `locale/` | Interface translations |
| `assets/` | Brush and node assets |
| `build_files/` | CMake configuration and the Windows build script (`make.bat`) |
| `tests/`, `tools/` | Tests and build helper tools |
| `patches/luxcore-bikini.patch` | Local modification applied on top of LuxCore `for_v2.11` (Cauchy B for Rough Glass) |

`extern/luxcore` and the platform libraries under `lib/` are Git submodules. A plain `git clone`
only records their commit ids and leaves the directories empty, so they have to be initialized
separately. See [BUILDING.txt](BUILDING.txt) for the exact commands.
