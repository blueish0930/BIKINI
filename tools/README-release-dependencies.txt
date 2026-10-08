BIKINI v4.0.1 build dependency setup
====================================

This archive contains a Windows PowerShell fetch script and the BIKINI LuxCore patch.
It does not contain complete Git submodules, Git LFS objects, or NVIDIA SDK copies.

Clone https://github.com/blueish0930/BIKINI.git, extract this archive, and run:

    powershell -NoProfile -ExecutionPolicy Bypass -File tools\setup_release_dependencies.ps1 -RepoRoot C:\path\to\BIKINI

The script initializes the LuxCore and Windows library submodules at the commits recorded by
your BIKINI checkout, applies the local LuxCore patch once, pulls Git LFS objects, and fetches
these additional source trees from their original repositories:

    Spectra  bdd707b9a872bb17622696ed3cca2a27b743a5b7
    OptiX    f60c1e44f18426f426a2ed948f28515b3cf67b8a (8.0.0 headers)
    DLSS     374959484e79a640feaba44c93ac8cfb0a03f5b5 (310.9.1 SDK)

Existing dependency directories are left in place. See BUILDING.txt in the source repository
for prerequisites, manual commands, the LuxCore Conan dependency step, and the final build.
