import bpy, bmesh, traceback
bpy.ops.wm.read_factory_settings(use_empty=True)
mesh = bpy.data.meshes.new("M")
bm = bmesh.new(); bmesh.ops.create_cube(bm, size=2.0); bm.to_mesh(mesh); bm.free()
obj = bpy.data.objects.new("O", mesh)
bpy.context.collection.objects.link(obj)
bpy.context.view_layer.objects.active = obj
mod = obj.modifiers.new("GN", "NODES")
nt = bpy.data.node_groups.new("G", "GeometryNodeTree")
mod.node_group = nt
nt.interface.new_socket("Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
nt.interface.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
nodes, links = nt.nodes, nt.links
n_in = nodes.new("NodeGroupInput")
n_out = nodes.new("NodeGroupOutput")
n_dist = nodes.new("GeometryNodeDistributePointsOnFaces")
if "Density" in n_dist.inputs:
    n_dist.inputs["Density"].default_value = 10.0
n_viewer = nodes.new("GeometryNodeViewer")
n_index = nodes.new("GeometryNodeInputIndex")
print("viewer_items.new doc:", n_viewer.viewer_items.new.__doc__)
# try various APIs
for args in [
    dict(name="Geometry", socket_type="GEOMETRY"),
    dict(name="Geometry"),
]:
    try:
        n_viewer.viewer_items.new(**args)
        print("new ok", args)
        break
    except Exception as e:
        print("new fail", args, e)

# if empty, dump
print("count", len(n_viewer.viewer_items))
if len(n_viewer.viewer_items)==0:
    # default might already have sockets
    pass
else:
    # add float value if only geometry
    types = [getattr(i, "socket_type", None) for i in n_viewer.viewer_items]
    print("types", types)
    if "VALUE" not in types and "FLOAT" not in types:
        try:
            n_viewer.viewer_items.new(name="Value", socket_type="VALUE")
        except Exception as e:
            print("value new fail", e)
            try:
                n_viewer.viewer_items.new(name="Value", socket_type="FLOAT")
            except Exception as e2:
                print("value new fail2", e2)

for i in n_viewer.viewer_items:
    print("item", repr(i.name), getattr(i, "socket_type", None))
for s in n_viewer.inputs:
    print("sock", repr(s.name), s.type, s.bl_idname)

links.new(n_in.outputs[0], n_dist.inputs[0])
geo = next((s for s in n_viewer.inputs if s.type=="GEOMETRY"), None)
val = next((s for s in n_viewer.inputs if s.type != "GEOMETRY"), None)
print("connect geo", geo, "val", val)
if geo:
    links.new(n_dist.outputs["Points"], geo)
if val:
    try:
        links.new(n_index.outputs["Index"], val)
    except Exception as e:
        print("link val fail", e)
links.new(n_in.outputs[0], n_out.inputs[0])

for area in bpy.context.screen.areas:
    if area.type == "VIEW_3D":
        sp = area.spaces.active
        sp.show_viewer = True
        sp.overlay.show_viewer_attribute = True
        sp.overlay.show_viewer_text = True
        sp.overlay.show_text = True

activated = False
for window in bpy.context.window_manager.windows:
    for area in window.screen.areas:
        if area.type != "NODE_EDITOR":
            continue
        space = area.spaces.active
        space.tree_type = "GeometryNodeTree"
        space.node_tree = nt
        region = next(r for r in area.regions if r.type=="WINDOW")
        with bpy.context.temp_override(window=window, area=area, region=region, space_data=space, active_object=obj, object=obj):
            nt.nodes.active = n_viewer
            for n in nt.nodes:
                n.select = (n is n_viewer)
            try:
                bpy.ops.node.activate_viewer()
                activated = True
                print("activate ok")
            except Exception as e:
                print("activate fail", e)

print("activated", activated)
depsgraph = bpy.context.evaluated_depsgraph_get()
depsgraph.update()
obj_eval = obj.evaluated_get(depsgraph)
print("obj_eval.type", obj_eval.type)
if hasattr(obj_eval.data, "attributes"):
    print("data attrs", list(obj_eval.data.attributes.keys()))

count = 0
for inst in depsgraph.object_instances:
    count += 1
    ob = inst.object
    keys = list(ob.data.attributes.keys()) if hasattr(ob.data, "attributes") else []
    if keys or ob.type != "MESH":
        print("inst", ob.name, "type", ob.type, "is_instance", inst.is_instance, "attrs", keys)
        if ".viewer" in keys:
            a = ob.data.attributes[".viewer"]
            print("  .viewer domain", a.domain, "data_type", a.data_type, "len", len(a.data))
print("instance_count", count)
print("DONE")
