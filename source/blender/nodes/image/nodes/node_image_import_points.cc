/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: Import Points — nested Geometry Node tree → point geometry with rotation/size/id. */

#include "BLI_string.hh"
#include "BLI_string_ref.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector.hh"

#include "DNA_node_types.h"
#include "DNA_space_types.h"

#include "BKE_context.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "ED_node.hh"
#include "ED_screen.hh"

#include "NOD_image_points.hh"
#include "NOD_socket_declarations_geometry.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_prototypes.hh"

#include "BLT_translation.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_import_points_cc {

using namespace blender::compositor;
using namespace blender::nodes::image_points;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Geometry>("Points"_ustr)
      .description(
          "Point cloud from the nested Geometry Node tree. Double-click or Tab to edit. "
          "Group Output: Points, Rotation, Size (2D pixels), ID. Mesh verts / curve points / "
          "instances are converted to points automatically. Other attributes pass to Point Stamp");
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  /* Tree is created in initfunc_api (needs Main). Keep flag-ready defaults here. */
  UNUSED_VARS(node);
}

static void node_init_api(const bContext *C, PointerRNA *ptr)
{
  if (!C || !ptr) {
    return;
  }
  Main *bmain = CTX_data_main(C);
  bNode *node = static_cast<bNode *>(ptr->data);
  if (!bmain || !node) {
    return;
  }
  if (node->runtime && !node->runtime->owner_tree && ptr->owner_id &&
      GS(ptr->owner_id->name) == ID_NT)
  {
    node->runtime->owner_tree = reinterpret_cast<bNodeTree *>(ptr->owner_id);
  }
  ensure_import_points_geometry_tree(*bmain, *node);
}

static void node_draw_buttons(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  ui::Layout &col = layout.column(false);
  /* Switchable among COP-Import Point* groups only (RNA poll filters the search). */
  col.prop(ptr, "node_tree", UI_ITEM_NONE, IFACE_("Node Group"), ICON_NODETREE);
  col.op("node.image_import_points_enter", IFACE_("Edit Nested Geometry"), ICON_NODETREE);
  col.label(IFACE_("Group Output: Points · Rotation · Size(2D) · ID"), ICON_INFO);
  col.label(IFACE_("Mesh/Curve/Instances → points · unit [0,1]"), ICON_INFO);
}

static void node_update(bNodeTree * /*ntree*/, bNode *node)
{
  if (!node || !node->id || GS(node->id->name) != ID_NT) {
    return;
  }
  bNodeTree *geo = reinterpret_cast<bNodeTree *>(node->id);
  if (geo->type != NTREE_GEOMETRY) {
    return;
  }
  /* Keep nested tree locked to fixed Group Output only. */
  lock_import_points_geometry_tree(*geo);
  /* Do not evaluate here: tree updates fire mid-link while Capture Attribute sockets rebuild.
   * Evaluation runs during Image Process cook (and geometry_from_points_links when safe). */
}

class ImportPointsOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    Result &points_out = this->get_result("Points");
    if (!points_out.should_compute()) {
      return;
    }

    points_out.allocate_single_value();

    bke::GeometrySet geometry;
    if (this->node().id && GS(this->node().id->name) == ID_NT) {
      const bNodeTree &geo_tree = *reinterpret_cast<const bNodeTree *>(this->node().id);
      geometry = evaluate_import_points_tree(geo_tree);
    }

    /* Never tag Point Stamp from COM cook — that causes redeclare/recook feedback loops. */

    if (PointsGeometryCache *cache = active_cache()) {
      const std::string key = cache->store(std::move(geometry));
      if (points_out.type() == ResultType::String) {
        points_out.set_single_value(key);
      }
    }
    else {
      if (points_out.type() == ResultType::String) {
        points_out.set_single_value(std::string(points_result_type_name));
      }
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new ImportPointsOperation(context, node);
}

/* -------------------------------------------------------------------- */
/** \name Enter nested geometry tree (double-click / Edit Group)
 * \{ */

static bool import_points_enter_poll(bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || snode->edittree->type != NTREE_IMAGE) {
    return false;
  }
  bNode *node = bke::node_get_active(*snode->edittree);
  return node && node->is_type("ImageNodeImportPoints"_ustr);
}

static wmOperatorStatus import_points_enter_exec(bContext *C, wmOperator * /*op*/)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  Main *bmain = CTX_data_main(C);
  if (!snode || !region || !bmain || !snode->edittree) {
    return OPERATOR_CANCELLED;
  }

  /* Must stay inside Image Process editor (local nest, not switch tree type). */
  if (snode->edittree->type != NTREE_IMAGE &&
      !(snode->nodetree && snode->nodetree->type == NTREE_IMAGE))
  {
    return OPERATOR_CANCELLED;
  }

  bNodeTree *host = snode->edittree->type == NTREE_IMAGE ? snode->edittree : snode->nodetree;
  bNode *node = bke::node_get_active(*host);
  if (!node || !node->is_type("ImageNodeImportPoints"_ustr)) {
    /* When already nested, active is inside GN — use path parent name. */
    if (snode->treepath.last() && snode->treepath.last() != snode->treepath.first()) {
      return OPERATOR_CANCELLED;
    }
    return OPERATOR_CANCELLED;
  }

  bNodeTree *geo = ensure_import_points_geometry_tree(*bmain, *node);
  if (!geo || node->id != &geo->id) {
    return OPERATOR_CANCELLED;
  }

  /* Push path only — tree_idname stays ImageNodeTree. */
  ED_node_tree_push(region, snode, geo, node);
  WM_event_add_notifier(C, NC_SCENE | ND_NODES, nullptr);
  WM_event_add_notifier(C, NC_NODE | ND_NODE_GIZMO, nullptr);
  return OPERATOR_FINISHED;
}

static void NODE_OT_image_import_points_enter(wmOperatorType *ot)
{
  ot->name = "Edit Import Points";
  ot->idname = "NODE_OT_image_import_points_enter";
  ot->description = "Enter the nested Geometry Node tree of Import Points";
  ot->exec = import_points_enter_exec;
  ot->poll = import_points_enter_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static void node_operators()
{
  WM_operatortype_append(NODE_OT_image_import_points_enter);
}

/** \} */

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeImportPoints"_ustr);
  ntype.ui_name = "Import Points";
  ntype.ui_description =
      "Nested Geometry Nodes under GPU Texture Editor (groups named COP-Import Point[.N]; "
      "switchable among those groups only; no Group Input). Group Output: Points, Rotation, "
      "Size, ID. Position/Size use unit square [0,1]. Double-click to edit in-place";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.initfunc_api = node_init_api;
  ntype.updatefunc = node_update;
  /* Do not use labelfunc — keep node title "Import Points", group name is separate. */
  ntype.draw_buttons = node_draw_buttons;
  ntype.get_compositor_operation = get_compositor_operation;
  ntype.register_operators = node_operators;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_import_points_cc
