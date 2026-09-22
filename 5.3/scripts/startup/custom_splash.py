# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Use a local image as this build's welcome splash.

Blender reads ``BLENDER_CUSTOM_SPLASH`` when drawing the startup splash.
Setting it from this startup script keeps the override on this build
instead of a machine-wide environment variable. An existing value wins.

``sys.executable`` is the bundled Python, not ``blender.exe``, so the image
is resolved from this build's local resource directory.

Looked up, in order:

- ``<directory containing blender.exe>/custom_splash.png`` (or ``.jpg`` / ``.jpeg``)
- ``<version>/datafiles/custom_splash.png`` (or ``.jpg`` / ``.jpeg``)
"""

import os

import bpy

_SPLASH_NAMES = (
    "custom_splash.png",
    "custom_splash.jpg",
    "custom_splash.jpeg",
)


def _candidate_dirs():
    local = bpy.utils.resource_path("LOCAL")
    if not local:
        return ()
    # ``LOCAL`` is ``<blender.exe directory>/<version>``.
    return (
        os.path.dirname(local),
        os.path.join(local, "datafiles"),
    )


def _find_splash():
    for directory in _candidate_dirs():
        for name in _SPLASH_NAMES:
            path = os.path.join(directory, name)
            if os.path.isfile(path):
                return path
    return None


def _apply():
    if os.environ.get("BLENDER_CUSTOM_SPLASH"):
        return
    path = _find_splash()
    if path:
        os.environ["BLENDER_CUSTOM_SPLASH"] = path


_apply()


def register():
    _apply()


def unregister():
    pass
