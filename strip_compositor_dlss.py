# Mute/remove compositor Neural, Denoise, Viewer so a crashing .blend can be opened.
# Usage (file is already loaded):
#   blender.exe --factory-startup your.blend --python strip_compositor_dlss.py
# Or from Blender Python console after a successful open.

import bpy

REMOVE_TYPES = {
    "CompositorNodeNeural",
    "CompositorNodeDenoise",
    "CompositorNodeViewer",
}


def strip_tree(ntree):
    removed = []
    for node in list(ntree.nodes):
        if node.bl_idname in REMOVE_TYPES:
            removed.append(node.bl_idname + ":" + node.name)
            ntree.nodes.remove(node)
    return removed


removed_all = []
for scene in bpy.data.scenes:
    scene.render.use_compositing = False
    ng = getattr(scene, "compositing_node_group", None)
    if ng:
        removed_all.extend(strip_tree(ng))

for tree in bpy.data.node_groups:
    if getattr(tree, "bl_idname", "") == "CompositorNodeTree" or tree.type == "COMPOSITING":
        removed_all.extend(strip_tree(tree))

print("STRIPPED", removed_all)
if bpy.data.filepath:
    bpy.ops.wm.save_mainfile()
    print("SAVED", bpy.data.filepath)
