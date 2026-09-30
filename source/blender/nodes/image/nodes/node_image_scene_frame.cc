/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: Scene Time — current scene time in seconds / frame number.
 * Matches Geometry Nodes / Compositor "Scene Time" naming and sockets. */

#include "BKE_node.hh"

#include "COM_node_operation.hh"

#include "node_image_util.hh"

#include "BKE_node_legacy_types.hh"

namespace blender::nodes::node_image_scene_time_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  /* Order matches CompositorNodeSceneTime / GeometryNodeInputSceneTime. */
  b.add_output<decl::Float>("Seconds"_ustr)
      .description("Current scene time in seconds (frame / fps)");
  b.add_output<decl::Float>("Frame"_ustr).description("Current scene frame number");
}

using namespace blender::compositor;

class SceneTimeOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    this->execute_seconds();
    this->execute_frame();
  }

 private:
  void execute_seconds()
  {
    Result &result = this->get_result("Seconds");
    if (!result.should_compute()) {
      return;
    }
    result.allocate_single_value();
    result.set_single_value(this->context().get_time());
  }

  void execute_frame()
  {
    Result &result = this->get_result("Frame");
    if (!result.should_compute()) {
      return;
    }
    result.allocate_single_value();
    result.set_single_value(float(this->context().get_frame_number()));
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new SceneTimeOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeSceneTime"_ustr, IMG_NODE_SCENE_FRAME);
  ntype.ui_name = "Scene Time";
  ntype.ui_description =
      "Input the current scene time in seconds or frames (follows the timeline)";
  ntype.enum_name_legacy = "SCENE_TIME";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
  /* Old idname from the temporary "Scene Frame" registration. */
  bke::node_register_alias(ntype, "ImageNodeSceneFrame"_ustr);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_scene_time_cc
