/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* IMAGE_NODES_MVP: port of Compositor Viewer (writes via Context::write_viewer). */

#include "DNA_node_types.h"
#include "DNA_space_types.h"

#include "BKE_context.hh"
#include "BKE_node.hh"

#include "RNA_types.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "COM_node_operation.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_viewer_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Image"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Image shown as the node-editor backdrop and as a thumbnail on this node");
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  /* Match the GTE Python add-on: on-node thumbnail, toggleable via the header preview icon. */
  node->flag |= NODE_PREVIEW;
}

static bool image_viewer_should_reserve_preview(const bContext *C, const bNode *node)
{
  if (node == nullptr || !(node->flag & NODE_PREVIEW)) {
    return false;
  }
  const SpaceNode *snode = CTX_wm_space_node(C);
  if (snode == nullptr) {
    return true;
  }
  if (!(snode->overlay.flag & SN_OVERLAY_SHOW_OVERLAYS) ||
      !(snode->overlay.flag & SN_OVERLAY_SHOW_PREVIEWS))
  {
    return false;
  }
  return true;
}

static void node_layout(ui::Layout &layout, bContext *C, PointerRNA *ptr)
{
  const bNode *node = static_cast<const bNode *>(ptr->data);
  if (!image_viewer_should_reserve_preview(C, node)) {
    return;
  }
  /* Same reserved body as GTENodeViewer (box.scale_y = 6.4 → ~128px thumbnail). */
  ui::Layout &box = layout.box();
  box.scale_y_set(6.4f);
  box.label("", ICON_NONE);
}

using namespace blender::compositor;

class ViewerOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    this->context().write_viewer(this->get_input("Image"));
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new ViewerOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeViewer"_ustr);
  ntype.ui_name = "Viewer";
  ntype.ui_description =
      "Visualize data from the image node graph as a node editor backdrop and as a thumbnail on "
      "the node";
  ntype.nclass = NODE_CLASS_OUTPUT;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.get_compositor_operation = get_compositor_operation;
  ntype.no_muting = true;
  ntype.flag |= NODE_PREVIEW;
  ntype.default_width = bke::NodeWidth::_180;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_viewer_cc
