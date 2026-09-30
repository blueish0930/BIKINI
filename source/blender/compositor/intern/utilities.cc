/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <optional>

#include "BLI_assert.hh"
#include "BLI_function_ref.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"

#include "DNA_node_types.h"

#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "NOD_node_declaration.hh"

#include "GPU_compute.hh"
#include "GPU_shader.hh"

#include "COM_result.hh"
#include "COM_utilities.hh"

namespace blender::compositor {

bool is_socket_available(const bNodeSocket *socket)
{
  /* Topology-cache vectors can contain freed sentinels (0xFF..) after Python-side
   * ImageNodeTree.evaluate() without a full ntree update. */
  if (socket == nullptr || intptr_t(socket) < 4096) {
    return false;
  }
  return socket->is_available() && StringRef(socket->idname) != "NodeSocketVirtual";
}

const bNodeSocket *get_output_linked_to_input(const bNodeSocket &input)
{
  if (input.runtime && input.is_logically_linked()) {
    const bNodeSocket *output = input.logically_linked_sockets()[0];
    if (output != nullptr && intptr_t(output) >= 4096) {
      return output;
    }
  }
  /* DNA `bNodeLink` is valid immediately after Python `links.new`; topology vectors may not be. */
  if (input.link && input.link->fromsock != nullptr && intptr_t(input.link->fromsock) >= 4096) {
    return input.link->fromsock;
  }
  return nullptr;
}

void foreach_output_linked_to_input(const bNodeSocket &input,
                                    FunctionRef<void(const bNodeSocket &output)> fn)
{
  if (!input.is_logically_linked()) {
    return;
  }
  for (const bNodeSocket *output : input.logically_linked_sockets()) {
    if (output) {
      fn(*output);
    }
  }
}

ResultType socket_data_type_to_result_type(const eNodeSocketDatatype data_type,
                                           const std::optional<int> dimensions)
{
  switch (data_type) {
    case SOCK_FLOAT:
      return ResultType::Float;
    case SOCK_INT:
      return ResultType::Int;
    case SOCK_BOOLEAN:
      return ResultType::Bool;
    case SOCK_VECTOR:
      switch (dimensions.value_or(3)) {
        case 2:
          return ResultType::Float2;
        case 3:
          return ResultType::Float3;
        case 4:
          return ResultType::Float4;
        default:
          BLI_assert_unreachable();
          return ResultType::Float;
      }
    case SOCK_INT_VECTOR:
      switch (dimensions.value_or(2)) {
        case 2:
          return ResultType::Int2;
        case 3:
          return ResultType::Int3;
        default:
          BLI_assert_unreachable();
          return ResultType::Float;
      }
    case SOCK_RGBA:
      return ResultType::Color;
    case SOCK_MATRIX:
      return ResultType::Float4x4;
    case SOCK_MENU:
      return ResultType::Menu;
    case SOCK_STRING:
      return ResultType::String;
    case SOCK_ROTATION:
      return ResultType::Quaternion;
    case SOCK_OBJECT:
      return ResultType::Object;
    case SOCK_IMAGE:
      return ResultType::Image;
    case SOCK_FONT:
      return ResultType::Font;
    case SOCK_SCENE:
      return ResultType::Scene;
    case SOCK_TEXT_ID:
      return ResultType::Text;
    case SOCK_MASK:
      return ResultType::Mask;
    case SOCK_BUNDLE:
      return ResultType::Bundle;
    case SOCK_GEOMETRY:
      /* Image Process geometry pipeline (Import Points / Import Geo / Point Stamp /
       * Rasterize Geometry): Geometry sockets carry a String cache key into the Image
       * Process GeometrySet cache; not a GPU image type. */
      return ResultType::String;
    case SOCK_CLOSURE:
      /* Closure appears on zone nodes allowed in Image Process; treat as unused single-value
       * float so evaluation does not hit the unreachable default. */
      return ResultType::Float;
    case SOCK_COLOR_RAMP:
    case SOCK_CURVE:
      /* Edited on the socket, not a compositor image. Keep the group evaluating. */
      return ResultType::Float;
    default:
      BLI_assert_unreachable();
      return ResultType::Float;
  }
}

ResultType get_node_socket_result_type(const bNodeSocket *socket)
{
  if (!is_plausible_pointer(socket)) {
    return ResultType::Float;
  }
  /* Gracefully handle undefined sockets, falling back to a float. */
  if (socket->typeinfo == &bke::NodeSocketTypeUndefined) {
    return ResultType::Float;
  }

  const eNodeSocketDatatype socket_type = static_cast<eNodeSocketDatatype>(socket->type);
  if (socket_type == SOCK_VECTOR) {
    const bNodeSocketValueVector *value = socket->default_value_typed<bNodeSocketValueVector>();
    const int dimensions = is_plausible_pointer(value) ? value->dimensions : 3;
    return socket_data_type_to_result_type(socket_type, dimensions);
  }
  if (socket_type == SOCK_INT_VECTOR) {
    const bNodeSocketValueIntVector *value =
        socket->default_value_typed<bNodeSocketValueIntVector>();
    const int dimensions = is_plausible_pointer(value) ? value->dimensions : 3;
    return socket_data_type_to_result_type(socket_type, dimensions);
  }
  /* Image Process still uses String cache keys on Bundle sockets (Point Stamp / Rasterize).
   * Official compositor uses first-class Bundle results. */
  if (socket_type == SOCK_BUNDLE) {
    const bNode *owner_node = (socket->runtime != nullptr) ? socket->runtime->owner_node : nullptr;
    const bNodeTree *tree = (owner_node != nullptr && owner_node->runtime != nullptr) ?
                                owner_node->runtime->owner_tree :
                                nullptr;
    if (tree != nullptr && intptr_t(tree) >= 4096 && tree->type == NTREE_IMAGE) {
      return ResultType::String;
    }
  }

  return socket_data_type_to_result_type(socket_type);
}

ResultType get_node_interface_socket_result_type(const bNodeTreeInterfaceSocket &socket)
{
  /* Gracefully handle undefined interface sockets, falling back to a float. */
  if (socket.socket_typeinfo() == &bke::NodeSocketTypeUndefined) {
    return ResultType::Float;
  }

  const eNodeSocketDatatype socket_type = socket.socket_typeinfo()->type;
  if (socket_type == SOCK_VECTOR) {
    return socket_data_type_to_result_type(
        socket_type, static_cast<bNodeSocketValueVector *>(socket.socket_data)->dimensions);
  }
  if (socket_type == SOCK_INT_VECTOR) {
    return socket_data_type_to_result_type(
        socket_type, static_cast<bNodeSocketValueIntVector *>(socket.socket_data)->dimensions);
  }

  return socket_data_type_to_result_type(socket_type);
}

bool is_output_linked_to_input_conditioned(const bNodeSocket &output,
                                           FunctionRef<bool(const bNodeSocket &)> condition)
{
  for (const bNodeSocket *input : output.logically_linked_sockets()) {
    if (condition(*input)) {
      return true;
    }
  }
  return false;
}

bool is_pixel_node(const bNode &node)
{
  return node.typeinfo->build_multi_function;
}

static std::optional<ImplicitInputType> get_implicit_input(
    const NodeDefaultInputType node_default_input_type)
{
  switch (node_default_input_type) {
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_VALUE:
      return std::nullopt;
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_UNIFORM_IMAGE_COORDINATES:
      return ImplicitInputType::UniformImageCoordinates;
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_NORMALIZED_IMAGE_COORDINATES:
      return ImplicitInputType::NormalizedImageCoordinates;
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_SCENE_FRAME:
      return ImplicitInputType::SceneFrame;
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_INDEX_FIELD:
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_ID_INDEX_FIELD:
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_NORMAL_FIELD:
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_POSITION_FIELD:
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_INSTANCE_TRANSFORM_FIELD:
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_HANDLE_LEFT_FIELD:
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_HANDLE_RIGHT_FIELD:
    case NodeDefaultInputType::NODE_DEFAULT_INPUT_SELF_OBJECT:
      break;
  }

  BLI_assert_unreachable();
  return std::nullopt;
}

static int get_domain_priority(const bNodeSocket *input,
                               const nodes::SocketDeclaration *socket_declaration)
{
  /* Negative priority means no priority is set and we fall back to the index, that is, we
   * prioritize inputs according to their order. */
  if (socket_declaration->compositor_domain_priority() < 0) {
    return input->index();
  }
  return socket_declaration->compositor_domain_priority();
}

InputDescriptor input_descriptor_from_input_socket(const bNodeSocket *socket)
{
  InputDescriptor input_descriptor;
  input_descriptor.type = get_node_socket_result_type(socket);

  /* Default to the index of the input as its domain priority in case the node does not have a
   * declaration. */
  input_descriptor.domain_priority = socket->index();

  /* Not every node has a declaration, in which case we assume the default values for the rest of
   * the properties. */
  const nodes::NodeDeclaration *node_declaration = socket->owner_node().declaration();
  if (!node_declaration) {
    return input_descriptor;
  }
  /* Prefer the socket's own declaration pointer. Index lookup can go out of range in Release
   * when topology cache `index_in_node` is still -1, which then dereferences a garbage
   * SocketDeclaration * (ACCESS_VIOLATION at a near-null address). */
  const nodes::SocketDeclaration *socket_declaration = socket->runtime ?
                                                              socket->runtime->declaration :
                                                              nullptr;
  if (!socket_declaration) {
    const int index = socket->index();
    if (index >= 0 && index < node_declaration->inputs.size()) {
      socket_declaration = node_declaration->inputs[index];
    }
  }
  if (!socket_declaration) {
    return input_descriptor;
  }
  input_descriptor.domain_priority = get_domain_priority(socket, socket_declaration);
  input_descriptor.expects_single_value = socket_declaration->structure_type ==
                                          nodes::StructureType::Single;
  input_descriptor.realization_mode = static_cast<InputRealizationMode>(
      socket_declaration->compositor_realization_mode());
  input_descriptor.implicit_input = get_implicit_input(socket_declaration->default_input_type);

  return input_descriptor;
}

InputDescriptor input_descriptor_from_interface_input(const bNodeTree &node_group,
                                                      const bNodeTreeInterfaceSocket &socket)
{
  InputDescriptor input_descriptor;
  input_descriptor.type = get_node_interface_socket_result_type(socket);
  input_descriptor.domain_priority = node_group.interface_input_index(socket);
  input_descriptor.expects_single_value = socket.structure_type ==
                                          NodeSocketInterfaceStructureType::Single;
  input_descriptor.realization_mode = InputRealizationMode::None;
  input_descriptor.implicit_input = get_implicit_input(socket.default_input);

  return input_descriptor;
}

void compute_dispatch_threads_at_least(gpu::Shader *shader, int2 threads_range, int2 local_size)
{
  /* If the threads range is divisible by the local size, dispatch the number of needed groups,
   * which is their division. If it is not divisible, then dispatch an extra group to cover the
   * remaining invocations, which means the actual threads range of the dispatch will be a bit
   * larger than the given one. */
  const int2 groups_to_dispatch = math::divide_ceil(threads_range, local_size);
  GPU_compute_dispatch(shader, groups_to_dispatch.x, groups_to_dispatch.y, 1);
}

bool is_node_preview_needed(const bNode &node)
{
  if (!(node.flag & NODE_PREVIEW)) {
    return false;
  }

  if (node.flag & NODE_COLLAPSED) {
    return false;
  }

  return true;
}

const bNodeSocket *find_preview_output_socket(const bNode &node)
{
  if (!is_node_preview_needed(node)) {
    return nullptr;
  }

  for (const bNodeSocket *output : node.output_sockets()) {
    if (is_socket_available(output) && output->is_logically_linked()) {
      return output;
    }
  }

  return nullptr;
}

}  // namespace blender::compositor
