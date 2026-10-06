# Third-Party License Notes

BIKINI as a whole follows Blender's GNU General Public License v3.0 or later. Copyright belongs
to the Blender Foundation and its contributors; the modifications BIKINI adds on top are offered
under the same license.

- Full license text: [release/license/spdx/GPL-3.0-or-later.txt](release/license/spdx/GPL-3.0-or-later.txt)
- Short note in the repository root: [COPYING](COPYING)
- Project documentation: <https://blueish0930.github.io/BIKINI/>

Third-party libraries that are compiled or linked into the program keep their own licenses. When
distributing a binary, both the GPL and the conditions of the libraries listed below have to be
satisfied.

Upstream Blender already generates a library catalog with the license texts in
[release/license/license.md](release/license/license.md). That file groups the libraries by
license; each group lists the library name, version and copyright line first and appends the
license text. The SPDX texts live in [release/license/spdx/](release/license/spdx/). That catalog
covers Blender's original dependencies and does not yet include the additional CGAL, LuxCore,
Spectra, OptiX and DLSS references of BIKINI; those are described below.

## Third-party sources inside this repository

Everything under `extern/` except the `luxcore` submodule is distributed with the Git history.
Each directory carries its own license files. The entry that is directly related to geometry
nodes and is not part of the `license.md` catalog is:

| Library | Location | License |
| --- | --- | --- |
| CGAL 6.2 | `extern/cgal` | Most files are GPL-3.0-or-later or LGPL-3.0-or-later, a few are under the Boost Software License. See [extern/cgal/LICENSE](extern/cgal/LICENSE) and [extern/cgal/README.blender](extern/cgal/README.blender) |

For the remaining bundled libraries (Audaspace, Bullet, Eigen, Jolt, OpenSubdiv, gtest and
others) the `COPYING`, `LICENSE` or `README` file in each directory is authoritative; they are
already summarized in `release/license/license.md`.

## Libraries you have to fetch yourself

These libraries are not part of the ordinary files of this repository. How to fetch them and how
to build are described in [BUILDING.txt](BUILDING.txt).

| Library | Where it goes | License | Where the license text lives |
| --- | --- | --- | --- |
| Blender platform precompiled libraries | `lib/windows_x64` and the equivalents | The individual open-source licenses of the bundled libraries, matching the entries in `release/license/license.md` | `deps.md` in the precompiled library repository, plus `license.md` |
| LuxCore / SLG / LuxRays | `extern/luxcore` | Apache-2.0 | `extern/luxcore/COPYING.txt` after checkout |
| Spectra | `lib/<platform>/spectra` | Mozilla Public License 2.0 | `LICENSE` after checkout; SPDX text in [release/license/spdx/MPL-2.0.txt](release/license/spdx/MPL-2.0.txt) |
| NVIDIA OptiX headers | `lib/<platform>/optix` | NVIDIA Software Developer Kits, Samples and Tools License | `LICENSE.txt` inside the SDK the user downloads |
| NVIDIA DLSS / NGX SDK | `lib/<platform>/dlss` | NVIDIA RTX SDKs License | `LICENSE.txt` inside the SDK the user downloads |

`patches/luxcore-bikini.patch` is a local modification applied on top of LuxCore and does not
change LuxCore's Apache-2.0 licensing.

OptiX and DLSS are proprietary NVIDIA SDKs. This repository neither includes their code nor
redistributes their license texts; when a user obtains an SDK from NVIDIA or from the
corresponding public repository, the `LICENSE.txt` shipped with it is authoritative.
