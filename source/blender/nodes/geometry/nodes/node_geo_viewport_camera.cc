/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_object_types.h"

#include "BLI_mutex.hh"

#include "node_geometry_util.hh"

namespace blender::nodes {

static Mutex viewport_camera_mutex;
static std::optional<GeoNodesViewportCameraData> last_viewport_camera;

void update_last_viewport_camera(const GeoNodesViewportCameraData &data)
{
  std::lock_guard lock(viewport_camera_mutex);
  last_viewport_camera = data;
}

std::optional<GeoNodesViewportCameraData> get_last_viewport_camera()
{
  std::lock_guard lock(viewport_camera_mutex);
  return last_viewport_camera;
}

namespace node_geo_viewport_camera_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Object>("Viewport Camera"_ustr)
      .description("Runtime-only camera representing the most recently navigated 3D viewport");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeoNodesCallData &call_data = *params.user_data()->call_data;
  Object *viewport_camera = call_data.operator_data ? call_data.operator_data->viewport_camera :
                                                      call_data.modifier_data->viewport_camera;
  if (!viewport_camera) {
    params.set_default_remaining_outputs();
    return;
  }

  params.set_output("Viewport Camera"_ustr, viewport_camera);
}

static void node_register()
{
  /* Disabled: keep implementation + discover symbol for the generated register table, but do not
   * register the node type. Re-enable by defining WITH_VIEWPORT_CAMERA_NODE and restoring the
   * Add-menu entry + RNA define. */
#if !defined(WITH_VIEWPORT_CAMERA_NODE)
  return;
#else
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeInputViewportCamera"_ustr, GEO_NODE_VIEWPORT_CAMERA);
  ntype.ui_name = "Viewport Camera";
  ntype.ui_description =
      "Retrieve a runtime-only camera representing the most recently navigated 3D viewport";
  ntype.enum_name_legacy = "INPUT_VIEWPORT_CAMERA";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
#endif
}
NOD_REGISTER_NODE(node_register)

}  // namespace node_geo_viewport_camera_cc
}  // namespace blender::nodes
