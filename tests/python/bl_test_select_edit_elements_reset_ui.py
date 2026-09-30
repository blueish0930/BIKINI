# SPDX-FileCopyrightText: 2026 Blender Authors
# SPDX-License-Identifier: GPL-2.0-or-later
"""Structural check: Select/Edit Elements expose Bake-style full-width Reset in draw_buttons.

Runs offline against source files (no Blender GUI). Fail if the body Reset button
pattern is missing or a header-only Reset remains the sole control.
"""
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
SELECT = ROOT / "source/blender/nodes/geometry/nodes/node_geo_select_elements.cc"
EDIT = ROOT / "source/blender/nodes/geometry/nodes/node_geo_edit_elements.cc"
DRAW = ROOT / "source/blender/editors/space_node/node_draw.cc"

# Image-Bake style: row + scale_y + op(..., "Reset", ...)
BODY_RESET = re.compile(
    r'row\.scale_y_set\(1\.2f\);\s*'
    r'row\.op\("node\.(select|edit)_elements_reset",\s*IFACE_\("Reset"\)',
    re.MULTILINE,
)


def check_body(path: pathlib.Path, op_token: str) -> None:
    text = path.read_text(encoding="utf-8", errors="replace")
    if "draw_buttons" not in text and "node_layout" not in text:
        raise SystemExit(f"{path}: missing node layout")
    if "add_default_layout" not in text:
        raise SystemExit(
            f"{path}: missing add_default_layout() — Reset draw_buttons will not show "
            "with use_custom_socket_order"
        )
    if not BODY_RESET.search(text):
        # looser fallback
        if f'node.{op_token}_elements_reset' not in text or 'IFACE_("Reset")' not in text:
            raise SystemExit(f"{path}: missing full-width Reset op button")
        if "scale_y_set" not in text:
            raise SystemExit(f"{path}: Reset row missing scale_y (Bake-style)")
    print(f"OK body Reset: {path.name}")


def main() -> int:
    check_body(SELECT, "select")
    check_body(EDIT, "edit")
    draw = DRAW.read_text(encoding="utf-8", errors="replace")
    if "Reset stored selection / edit mesh" in draw:
        raise SystemExit("header-only Reset tip still present in node_draw.cc")
    if "Always show body buttons for Select/Edit Elements" not in draw:
        raise SystemExit("NODE_OPTIONS force for body buttons missing")
    print("OK node_draw: no header Reset; NODE_OPTIONS force present")
    print("PASS bl_test_select_edit_elements_reset_ui")
    return 0


if __name__ == "__main__":
    sys.exit(main())
