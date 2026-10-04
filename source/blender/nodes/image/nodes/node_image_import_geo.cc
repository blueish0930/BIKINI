/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: Import Geo — nested Geometry Node tree → geometry (mesh/curves/points). */

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

namespace blender::nodes::node_image_import_geo_cc {

using namespace blender::compositor;
using namespace blender::nodes::image_points;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .description(
          "Geometry from the nested Geometry Node tree. Double-click or Tab to edit. "
          "Group Output: Geometry only. Instances are realized. Connect to Rasterize Geometry");
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
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
  ensure_import_geo_geometry_tree(*bmain, *node);
}

static void node_draw_buttons(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  ui::Layout &col = layout.column(false);
  /* Switchable among COP-Import Geo* groups only (RNA poll filters the search). */
  col.prop(ptr, "node_tree", UI_ITEM_NONE, IFACE_("Node Group"), ICON_NODETREE);
  col.op("node.image_import_geo_enter", IFACE_("Edit Nested Geometry"), ICON_NODETREE);
  col.label(IFACE_("Group Output: Geometry"), ICON_INFO);
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
  lock_import_geo_geometry_tree(*geo);
}

class ImportGeoOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    Result &geo_out = this->get_result("Geometry");
    if (!geo_out.should_compute()) {
      return;
    }

    geo_out.allocate_single_value();

    bke::GeometrySet geometry;
    if (this->node().id && GS(this->node().id->name) == ID_NT) {
      const bNodeTree &geo_tree = *reinterpret_cast<const bNodeTree *>(this->node().id);
      geometry = evaluate_import_geo_tree(geo_tree);
    }

    if (PointsGeometryCache *cache = active_cache()) {
      const std::string key = cache->store(std::move(geometry));
      if (geo_out.type() == ResultType::String) {
        geo_out.set_single_value(key);
      }
    }
    else {
      if (geo_out.type() == ResultType::String) {
        geo_out.set_single_value(std::string(points_result_type_name));
      }
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new ImportGeoOperation(context, node);
}

/* -------------------------------------------------------------------- */
/** \name Enter nested geometry tree (double-click / Edit Group)
 * \{ */

static bool import_geo_enter_poll(bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || snode->edittree->type != NTREE_IMAGE) {
    return false;
  }
  bNode *node = bke::node_get_active(*snode->edittree);
  return node && node->is_type("ImageNodeImportGeo"_ustr);
}

static wmOperatorStatus import_geo_enter_exec(bContext *C, wmOperator * /*op*/)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  Main *bmain = CTX_data_main(C);
  if (!snode || !region || !bmain || !snode->edittree) {
    return OPERATOR_CANCELLED;
  }

  if (snode->edittree->type != NTREE_IMAGE &&
      !(snode->nodetree && snode->nodetree->type == NTREE_IMAGE))
  {
    return OPERATOR_CANCELLED;
  }

  bNodeTree *host = snode->edittree->type == NTREE_IMAGE ? snode->edittree : snode->nodetree;
  bNode *node = bke::node_get_active(*host);
  if (!node || !node->is_type("ImageNodeImportGeo"_ustr)) {
    return OPERATOR_CANCELLED;
  }

  bNodeTree *geo = ensure_import_geo_geometry_tree(*bmain, *node);
  if (!geo || node->id != &geo->id) {
    return OPERATOR_CANCELLED;
  }

  ED_node_tree_push(region, snode, geo, node);
  WM_event_add_notifier(C, NC_SCENE | ND_NODES, nullptr);
  WM_event_add_notifier(C, NC_NODE | ND_NODE_GIZMO, nullptr);
  return OPERATOR_FINISHED;
}

static void NODE_OT_image_import_geo_enter(wmOperatorType *ot)
{
  ot->name = "Edit Import Geo";
  ot->idname = "NODE_OT_image_import_geo_enter";
  ot->description = "Enter the nested Geometry Node tree of Import Geo";
  ot->exec = import_geo_enter_exec;
  ot->poll = import_geo_enter_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static void node_operators()
{
  WM_operatortype_append(NODE_OT_image_import_geo_enter);
}

/** \} */

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeImportGeo"_ustr);
  ntype.ui_name = "Import Geo";
  ntype.ui_description =
      "Nested Geometry Nodes under GPU Texture Editor (groups named COP-Import Geo[.N]; "
      "switchable among those groups only; no Group Input). Group Output: Geometry. "
      "Double-click to edit in-place. Connect to Rasterize Geometry";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.initfunc_api = node_init_api;
  ntype.updatefunc = node_update;
  ntype.draw_buttons = node_draw_buttons;
  ntype.get_compositor_operation = get_compositor_operation;
  ntype.register_operators = node_operators;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_import_geo_cc
