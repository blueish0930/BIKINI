/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo_nodes
 *
 * Geometry Nodes Python node: multi-line stretchable code body.
 * On evaluate, injects the input GeometrySet + this node as Python locals,
 * runs the script (which may mutate / replace geometry), then outputs the result.
 */

#include "BLI_string.hh"

#include "BLO_read_write.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

#ifdef WITH_PYTHON
#  include "BPY_extern_geometry.hh"
#endif

namespace blender::nodes::node_geo_python_cc {

NODE_STORAGE_FUNCS(NodeGeometryPython)

/**
 * Default body: active path is pass-through; commented cases document the API.
 * Keep this as pure ASCII comments + pass so evaluation is a no-op by default.
 */
static const char *default_python_code()
{
  return R"DEFAULT(# =============================================================================
# Geometry Nodes — Python node
# -----------------------------------------------------------------------------
# Injected locals on every evaluation (also in globals for convenience):
#
#   geometry, geo : bpy.types.GeometrySet   # INPUT geometry (same object under both names)
#   node, self_node : this GeometryNodePython (bpy RNA pointer for *this* node)
#   tree            : node.id_data          # GeometryNodeTree owning this node
#
# Write-back rule:
#   - Mutate `geometry` in place (mesh/curves attributes, bmesh topology, …), OR
#   - Rebind `geometry = <another bpy.types.GeometrySet>`
#   After the script finishes, the GeometrySet bound to name `geometry`
#   (fallback: `geo`) is sent to the Geometry output socket.
#
# Evaluation context:
#   - Runs during Geometry Nodes / modifier evaluation (depsgraph).
#   - `node` identifies *this* evaluating node instance (name, location, code, …).
#   - Prefer `geometry` over bpy.context.active_object — context is often empty here.
# =============================================================================

# --- ref: current evaluating node (uncomment to inspect) ---
# print("node=", node, "name=", node.name, "bl_idname=", node.bl_idname)
# print("tree=", tree, "tree.name=", tree.name)
# print("node.location=", tuple(node.location), "code_len=", len(node.code))
# print("self_node is node:", self_node is node)
# print("geometry=", geometry, "geo is geometry:", geo is geometry)

# --- Case A: read mesh component & modify an existing attribute ---------------
# mesh = geometry.mesh  # or geometry.mesh_base (no subdivision wrapper)
# if mesh is not None:
#     # Positions are a point-domain vector attribute named "position"
#     pos = mesh.attributes.get("position")
#     if pos and pos.domain == 'POINT':
#         for i, item in enumerate(pos.data):
#             x, y, z = item.vector
#             item.vector = (x, y, z + 0.1)  # nudge up
#     mesh.update()

# --- Case B: add attributes on different domains ------------------------------
# mesh = geometry.mesh
# if mesh is not None:
#     # POINT float
#     if "weight" not in mesh.attributes:
#         attr = mesh.attributes.new(name="weight", type='FLOAT', domain='POINT')
#         for i, item in enumerate(attr.data):
#             item.value = float(i) / max(1, len(attr.data) - 1)
#     # CORNER (face corner) color
#     if "col" not in mesh.attributes:
#         attr = mesh.attributes.new(name="col", type='FLOAT_COLOR', domain='CORNER')
#         for item in attr.data:
#             item.color = (1.0, 0.2, 0.1, 1.0)
#     # EDGE int
#     if "edge_id" not in mesh.attributes:
#         attr = mesh.attributes.new(name="edge_id", type='INT', domain='EDGE')
#         for i, item in enumerate(attr.data):
#             item.value = i
#     # FACE boolean
#     if "keep" not in mesh.attributes:
#         attr = mesh.attributes.new(name="keep", type='BOOLEAN', domain='FACE')
#         for item in attr.data:
#             item.value = True
#     mesh.update()

# --- Case C: topology — add / delete verts, edges, faces via bmesh ------------
# import bmesh
# mesh = geometry.mesh
# if mesh is not None:
#     bm = bmesh.new()
#     bm.from_mesh(mesh)
#     # Extrude-ish: add a vertex and connect
#     if bm.verts:
#         v0 = bm.verts[0]
#         v_new = bm.verts.new((v0.co.x + 1.0, v0.co.y, v0.co.z))
#         bm.edges.new((v0, v_new))
#     # Delete loose verts example (none by default)
#     # bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_edges], context='VERTS')
#     bm.to_mesh(mesh)
#     bm.free()
#     mesh.update()

# --- Case D: point cloud — create / tweak points ------------------------------
# pc = geometry.pointcloud
# if pc is not None:
#     # Scale all positions
#     pos = pc.attributes.get("position")
#     if pos:
#         for item in pos.data:
#             x, y, z = item.vector
#             item.vector = (x * 1.1, y * 1.1, z * 1.1)

# --- Case E: curves — radius / position on control points ---------------------
# curves = geometry.curves
# if curves is not None:
#     # Curves use attribute API similar to mesh (point = control point)
#     pos = curves.attributes.get("position")
#     radius = curves.attributes.get("radius")
#     if radius is not None:
#         for item in radius.data:
#             item.value = max(item.value, 0.05)
#     if pos is not None:
#         for item in pos.data:
#             x, y, z = item.vector
#             item.vector = (x, y, z)

# --- Case F: use node settings while evaluating ------------------------------
# # Example: read this node's multi-line code length / name into an attribute
# mesh = geometry.mesh
# if mesh is not None:
#     attr_name = "from_" + node.name.replace(".", "_")
#     if attr_name not in mesh.attributes:
#         attr = mesh.attributes.new(name=attr_name, type='FLOAT', domain='POINT')
#         for item in attr.data:
#             item.value = float(len(node.code))
#         mesh.update()

# --- Case G: intentionally clear output (empty GeometrySet) -------------------
# # import bpy
# # geometry = bpy.types.GeometrySet()

# Default active path: leave `geometry` unchanged → pure pass-through.
pass
)DEFAULT";
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_default_layout();

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .description(
          "Input geometry exposed to the script as locals geometry/geo; "
          "script result is written to the output");
  b.add_output<decl::Geometry>("Geometry"_ustr).propagate_all_geometry().align_with_previous();
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  bNode &node = *static_cast<bNode *>(ptr->data);
  NodeGeometryPython &storage = node_storage(node);

  layout.alignment_set(ui::LayoutAlign::Expand);
  layout.textbox_with_state(ptr, "code", &storage.textbox_state, IFACE_("Python"));
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryPython *storage = MEM_new<NodeGeometryPython>(__func__);
  storage->textbox_state.visible_lines = 16;
  storage->code = BLI_strdup(default_python_code());
  node->storage = storage;
  node->width = 420.0f;
}

static void node_free_storage(bNode *node)
{
  NodeGeometryPython *storage = static_cast<NodeGeometryPython *>(node->storage);
  if (storage == nullptr) {
    return;
  }
  MEM_SAFE_DELETE(storage->code);
  MEM_delete(storage);
  node->storage = nullptr;
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryPython &src_storage = node_storage(*src_node);
  NodeGeometryPython *dst_storage = MEM_new<NodeGeometryPython>(__func__,
                                                                dna::shallow_copy(src_storage));
  dst_node->storage = dst_storage;

  if (src_storage.code) {
    dst_storage->code = BLI_strdup(src_storage.code);
  }
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeGeometryPython &storage = node_storage(node);
  writer.write_string(storage.code);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeGeometryPython &storage = node_storage(node);
  BLO_read_string(&reader, &storage.code);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry"_ustr);

  const NodeGeometryPython &storage = node_storage(params.node());
  const char *code = storage.code ? storage.code : "";

  if (code[0] != '\0') {
#ifdef WITH_PYTHON
    bNode &node = const_cast<bNode &>(params.node());
    bNodeTree &ntree = node.owner_tree();
    const bool ok = BPY_run_geometry_node_script(
        nullptr, code, geometry_set, &ntree, &node);
    if (!ok) {
      params.error_message_add(NodeWarningType::Error,
                               "Python script failed (see console for traceback)");
    }
#else
    params.error_message_add(NodeWarningType::Error,
                             "Python is not available in this Blender build");
#endif
  }

  params.set_output("Geometry"_ustr, std::move(geometry_set));
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodePython"_ustr, GEO_NODE_PYTHON);
  ntype.ui_name = "Python";
  ntype.ui_description =
      "Run a Python script on the input GeometrySet (locals: geometry/geo, node, tree), "
      "then output the resulting geometry for downstream nodes";
  ntype.enum_name_legacy = "PYTHON";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  bke::node_type_storage(ntype, "NodeGeometryPython", node_free_storage, node_copy_storage);
  /* Temporarily hide from link-drag search (Add menu also commented out). */
  ntype.gather_link_search_ops = nullptr;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_python_cc
