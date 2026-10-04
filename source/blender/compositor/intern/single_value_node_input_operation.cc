/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <string>

#include "BLI_assert.hh"
#include "BLI_math_euler.hh"
#include "BLI_math_rotation_types.hh"
#include "BLI_math_vector_types.hh"

#include "DNA_node_types.h"

#include "BKE_node.hh"
#include "BKE_node_enum.hh"
#include "BKE_node_runtime.hh"

#include "NOD_geometry_nodes_bundle.hh"

#include "COM_operation.hh"
#include "COM_result.hh"
#include "COM_single_value_node_input_operation.hh"
#include "COM_utilities.hh"

namespace blender::compositor {

const StringRef SingleValueNodeInputOperation::output_identifier_ = StringRef("Output");

SingleValueNodeInputOperation::SingleValueNodeInputOperation(Context &context,
                                                             const bNodeSocket &input_socket)
    : Operation(context), input_socket_(input_socket)
{
  this->populate_result(get_node_socket_result_type(&input_socket));
}

void SingleValueNodeInputOperation::execute()
{
  Result &result = this->get_result();
  result.allocate_single_value();

  switch (input_socket_.type) {
    case SOCK_FLOAT: {
      const bNodeSocketValueFloat *storage =
          input_socket_.default_value_typed<bNodeSocketValueFloat>();
      result.set_single_value(is_plausible_pointer(storage) ? storage->value : 0.0f);
      break;
    }
    case SOCK_INT: {
      const bNodeSocketValueInt *storage = input_socket_.default_value_typed<bNodeSocketValueInt>();
      result.set_single_value(is_plausible_pointer(storage) ? storage->value : 0);
      break;
    }
    case SOCK_BOOLEAN: {
      const bNodeSocketValueBoolean *storage =
          input_socket_.default_value_typed<bNodeSocketValueBoolean>();
      result.set_single_value(is_plausible_pointer(storage) ? bool(storage->value) : false);
      break;
    }
    case SOCK_VECTOR: {
      const bNodeSocketValueVector *storage =
          input_socket_.default_value_typed<bNodeSocketValueVector>();
      const int dimensions = is_plausible_pointer(storage) ? storage->dimensions : 3;
      switch (dimensions) {
        case 2: {
          result.set_single_value(is_plausible_pointer(storage) ? float2(storage->value) :
                                                                    float2(0.0f));
          break;
        }
        case 4: {
          result.set_single_value(is_plausible_pointer(storage) ? float4(storage->value) :
                                                                    float4(0.0f));
          break;
        }
        default: {
          result.set_single_value(is_plausible_pointer(storage) ? float3(storage->value) :
                                                                float3(0.0f));
          break;
        }
      }
      break;
    }
    case SOCK_INT_VECTOR: {
      const bNodeSocketValueIntVector *storage =
          input_socket_.default_value_typed<bNodeSocketValueIntVector>();
      const int dimensions = is_plausible_pointer(storage) ? storage->dimensions : 3;
      if (dimensions == 2) {
        result.set_single_value(is_plausible_pointer(storage) ? int2(storage->value) : int2(0));
      }
      else {
        result.set_single_value(is_plausible_pointer(storage) ? int3(storage->value) : int3(0));
      }
      break;
    }
    case SOCK_RGBA: {
      const bNodeSocketValueRGBA *storage = input_socket_.default_value_typed<bNodeSocketValueRGBA>();
      result.set_single_value(is_plausible_pointer(storage) ? Color(storage->value) :
                                                            Color(0.0f, 0.0f, 0.0f, 1.0f));
      break;
    }
    case SOCK_MATRIX: {
      result.set_single_value(float4x4::identity());
      break;
    }
    case SOCK_MENU: {
      const bNodeSocketValueMenu *storage = input_socket_.default_value_typed<bNodeSocketValueMenu>();
      const bool plausible = is_plausible_pointer(storage);
      const bool multi = plausible &&
                         (storage->runtime_flag & bke::NODE_MENU_MULTI_SELECTION) != 0;
      result.set_single_value(nodes::MenuValue(plausible ? storage->value : 0, multi));
      break;
    }
    case SOCK_STRING: {
      const bNodeSocketValueString *storage =
          input_socket_.default_value_typed<bNodeSocketValueString>();
      result.set_single_value(is_plausible_pointer(storage) ? std::string(storage->value) :
                                                              std::string());
      break;
    }
    case SOCK_ROTATION: {
      const bNodeSocketValueRotation *rotation =
          input_socket_.default_value_typed<bNodeSocketValueRotation>();
      if (is_plausible_pointer(rotation)) {
        const math::EulerXYZ euler(float3(rotation->value_euler));
        result.set_single_value(math::to_quaternion(euler));
      }
      else {
        result.set_single_value(math::Quaternion::identity());
      }
      break;
    }
    case SOCK_OBJECT: {
      const bNodeSocketValueObject *storage =
          input_socket_.default_value_typed<bNodeSocketValueObject>();
      result.set_single_value(is_plausible_pointer(storage) ? storage->value : nullptr);
      break;
    }
    case SOCK_IMAGE: {
      const bNodeSocketValueImage *storage =
          input_socket_.default_value_typed<bNodeSocketValueImage>();
      result.set_single_value(is_plausible_pointer(storage) ? storage->value : nullptr);
      break;
    }
    case SOCK_FONT: {
      const bNodeSocketValueFont *storage = input_socket_.default_value_typed<bNodeSocketValueFont>();
      result.set_single_value(is_plausible_pointer(storage) ? storage->value : nullptr);
      break;
    }
    case SOCK_SCENE: {
      const bNodeSocketValueScene *storage =
          input_socket_.default_value_typed<bNodeSocketValueScene>();
      result.set_single_value(is_plausible_pointer(storage) ? storage->value : nullptr);
      break;
    }
    case SOCK_TEXT_ID: {
      const bNodeSocketValueText *storage = input_socket_.default_value_typed<bNodeSocketValueText>();
      result.set_single_value(is_plausible_pointer(storage) ? storage->value : nullptr);
      break;
    }
    case SOCK_MASK: {
      const bNodeSocketValueMask *storage = input_socket_.default_value_typed<bNodeSocketValueMask>();
      result.set_single_value(is_plausible_pointer(storage) ? storage->value : nullptr);
      break;
    }
    case SOCK_BUNDLE: {
      if (this->context().is_image_process()) {
        /* Image Process maps Bundle sockets to ResultType::String cache keys. */
        result.set_single_value(std::string(""));
      }
      else {
        result.set_single_value(nodes::Bundle::create());
      }
      break;
    }
    case SOCK_CUSTOM:
      /* An undefined socket, its value does not matter. */
      break;
    case SOCK_GEOMETRY:
      /* Image Process maps Geometry sockets to ResultType::String (cache keys).
       * Unlinked defaults must be empty strings — not float — or get_single_value_default
       * <std::string> throws bad_variant_access (Point Stamp crash on Bake). */
      result.set_single_value(std::string(""));
      break;
    case SOCK_CLOSURE:
      /* Closures are unused float placeholders in Image Process COM. */
      result.set_single_value(0.0f);
      break;
    default:
      BLI_assert_unreachable();
      break;
  }
}

Result &SingleValueNodeInputOperation::get_result()
{
  return Operation::get_result(output_identifier_);
}

void SingleValueNodeInputOperation::populate_result(const ResultType type)
{
  Operation::populate_result(output_identifier_, type);
}

}  // namespace blender::compositor
