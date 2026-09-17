# SPDX-FileCopyrightText: 2026 BIKINI
# SPDX-License-Identifier: GPL-3.0-or-later
"""Mirror topology operations in mesh edit mode (not just vertex transforms).

Official X-Mirror only moves counterparts. This add-on expands the selection
to topological/spatial pairs before extrude, inset, bevel, delete, loop cut,
and other topology ops — the same approach as BIKINI's C++ edit-mesh mirror.
"""

bl_info = {
    "name": "Mesh Topology Mirror",
    "author": "BIKINI",
    "version": (1, 0, 0),
    "blender": (4, 2, 0),
    "location": "3D Viewport > Mesh Options / Sidebar > Tool",
    "description": "Mirror topology operations (extrude, inset, bevel, loop cut…) in edit mode",
    "category": "Mesh",
}

from bpy.props import BoolProperty, FloatProperty, FloatVectorProperty
from bpy.types import AddonPreferences

from . import keymap as keymap_mod
from . import operators
from . import overlay
from . import ui


class MeshTopologyMirrorPrefs(AddonPreferences):
    bl_idname = __package__

    enable_topology_ops: BoolProperty(
        name="Mirror Topology Operations",
        description="When mesh X/Y/Z Mirror is on, topology operators also affect the other side",
        default=True,
    )
    show_overlay: BoolProperty(
        name="Highlight Mirror Counterparts",
        description="Draw unselected mirrored verts/edges/faces in cyan",
        default=True,
    )
    overlay_color: FloatVectorProperty(
        name="Highlight Color",
        subtype='COLOR',
        size=4,
        min=0.0,
        max=1.0,
        default=(0.25, 0.85, 0.95, 0.90),
    )
    spatial_threshold: FloatProperty(
        name="Spatial Pair Threshold",
        description="Maximum distance when pairing verts by position (Topology Mirror uses connectivity instead)",
        default=0.00002,
        min=1e-7,
        max=0.1,
        precision=6,
        step=0.001,
    )

    def draw(self, _context):
        layout = self.layout
        layout.prop(self, "enable_topology_ops")
        layout.prop(self, "show_overlay")
        layout.prop(self, "overlay_color")
        layout.prop(self, "spatial_threshold")
        layout.label(text="Uses the mesh X/Y/Z Mirror toggles in Mesh Options.")
        layout.label(text="Enable Topology Mirror to pair by connectivity instead of position.")


_classes = (
    MeshTopologyMirrorPrefs,
    *operators.classes,
    *ui.classes,
)

_stolen_extrude = []


def _steal_builtin_extrude():
    """Replace the builtin Python extrude operators; restore them on disable."""
    import bpy
    from bpy.utils import unregister_class
    _stolen_extrude.clear()
    wanted = {
        "view3d.edit_mesh_extrude_move_normal",
        "view3d.edit_mesh_extrude_individual_move",
        "view3d.edit_mesh_extrude_move_shrink_fatten",
    }
    found = []
    for name in dir(bpy.types):
        cls = getattr(bpy.types, name, None)
        if cls is None:
            continue
        if getattr(cls, "bl_idname", None) in wanted:
            found.append(cls)
    for cls in found:
        _stolen_extrude.append(cls)
        try:
            unregister_class(cls)
        except Exception:
            pass


def _restore_builtin_extrude():
    from bpy.utils import register_class
    for cls in _stolen_extrude:
        try:
            register_class(cls)
        except Exception:
            pass
    _stolen_extrude.clear()


def register():
    from bpy.utils import register_class
    _steal_builtin_extrude()
    for cls in _classes:
        register_class(cls)
    overlay.register()
    keymap_mod.register()
    ui.register()


def unregister():
    from bpy.utils import unregister_class
    ui.unregister()
    keymap_mod.unregister()
    overlay.unregister()
    for cls in reversed(_classes):
        unregister_class(cls)
    _restore_builtin_extrude()
