# BIKINI source repository

This repository records the current BIKINI Blender source tree as a new root snapshot. The previous Blender and BIKINI commit graph is intentionally absent from this repository's `main` history. The official upstream remote can still be used for reference, but future daily updates need a separate upstream checkout or an explicit import workflow because the new root does not share Git ancestry with upstream Blender.

## Dependencies

- `lib/windows_x64` is Blender's Windows precompiled-library submodule. The local 5.3 daily library files belong at this path. They are stored in the dependency repository, not as 4 GB of ordinary source-repository blobs.
- `extern/luxcore` is a LuxCore submodule pinned to its `for_v2.11` branch commit. The local `out` build products are not part of this repository.
- After cloning, initialize the submodules with `git submodule update --init lib/windows_x64 extern/luxcore`. Git LFS may be required for the precompiled Windows libraries.
- Apply the local LuxCore rough-glass changes with `git -C extern/luxcore apply ../../patches/luxcore-bikini.patch`. The patch is part of this repository because the pinned upstream LuxCore commit does not contain these local changes. Its application makes the LuxCore submodule appear modified in Git status.
- The local Release configuration also uses files in `lib/windows_x64/dlss`, `lib/windows_x64/optix` and `lib/windows_x64/spectra` that are not tracked by the upstream Windows-library submodule. Provision these local SDK files separately when reproducing that configuration; they are not included in this source repository.

The verified local Release build is configured with `LIBDIR=E:/Blender_Source/lib/windows_x64`. The 2026-10-04 merge notes and demo validation evidence are stored separately in `E:\BIKINI_Project_Records\2026-10-04` on the workstation. Historical validation reports are evidence for that run, not a claim that all demos pass now.
