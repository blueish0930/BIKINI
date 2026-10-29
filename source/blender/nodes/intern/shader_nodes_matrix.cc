/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Cycles has scalar/vector shader registers. Lower the ordinary 4x4 socket
 * graph at the renderer boundary; the editable source tree and Geometry Nodes
 * attributes stay matrices. */
#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <variant>
#include <vector>

#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_shader_object_transform.hh"
#include "BLI_listbase.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_noise.hh"
#include "BLI_shader_matrix_attribute.hh"
#include "BLI_string.hh"
#include "DNA_node_types.h"
#include "NOD_shader_nodes_inline.hh"
#include "NOD_socket.hh"
#include "NOD_vex.hh"
#include "vex/vex_program.hh"

namespace blender::nodes {
namespace {
using Scalar = std::variant<float, bNodeSocket *>;
using Matrix = std::array<Scalar, 16>;
using Vector3 = std::array<Scalar, 3>;
using Quaternion = std::array<Scalar, 4>; /* w, x, y, z */

class MatrixLowering {
  bNodeTree &tree_;
  std::unordered_map<const bNodeSocket *, bNode *> owners_;
  std::unordered_map<const bNodeSocket *, bNodeSocket *> origins_;
  std::unordered_map<const bNodeSocket *, Matrix> matrices_;
  std::unordered_map<const bNodeSocket *, Scalar> scalars_;
  std::unordered_map<const bNodeSocket *, Vector3> vectors_;
  std::unordered_map<const bNodeSocket *, Quaternion> rotations_;
  std::unordered_map<const bNode *, Vector3> singular_values_;
  std::unordered_map<const bNode *, std::array<bNode *, 7>> object_attributes_;
  std::unordered_map<const bNode *, bNode *> attribute_nodes_;

  bNode &node(const UString type) {
    bNode &result = *bke::node_add_node(nullptr, tree_, type);
    if (result.is_type("ShaderNodeAttribute"_ustr)) {
      /* Renderer-local helper nodes deliberately use the stable legacy socket identifiers. */
      static_cast<NodeShaderAttribute *>(result.storage)->data_type = SHD_ATTRIBUTE_DATA_LEGACY;
      update_node_declaration_and_sockets(tree_, result);
    }
    for (bNodeSocket &socket : result.inputs) {
      owners_[&socket] = &result;
    }
    for (bNodeSocket &socket : result.outputs) {
      owners_[&socket] = &result;
    }
    return result;
  }
  bNode &owner(const bNodeSocket &socket) { return *owners_.at(&socket); }
  bNodeSocket &input(bNode &n, int i) {
    return *static_cast<bNodeSocket *>(BLI_findlink(&n.inputs, i));
  }
  bNodeSocket &output(bNode &n, int i) {
    return *static_cast<bNodeSocket *>(BLI_findlink(&n.outputs, i));
  }
  bNode &object_attribute(bNode &n, const int field) {
    bNode *&attribute = object_attributes_[&n][field];
    if (!attribute) {
      attribute = &node("ShaderNodeAttribute"_ustr);
      auto &storage = *static_cast<NodeShaderAttribute *>(attribute->storage);
      storage.type = SHD_ATTRIBUTE_OBJECT;
      const std::string name = bke::shader_object_transform_attribute_name(
          reinterpret_cast<Object *>(n.id), field);
      STRNCPY(storage.name, name.c_str());
    }
    return *attribute;
  }
  std::array<Scalar, 4> object_components(bNode &n, const int field) {
    bNode &attribute = object_attribute(n, field);
    const Vector3 xyz = vector(output(attribute, 1));
    return {xyz[0], xyz[1], xyz[2], &output(attribute, 3)};
  }
  void assign(bNodeSocket &socket, const Scalar &value) {
    if (const auto *constant = std::get_if<float>(&value)) {
      socket.default_value_typed<bNodeSocketValueFloat>()->value = *constant;
    } else {
      bNodeSocket *source = std::get<bNodeSocket *>(value);
      bke::node_add_link(tree_, owner(*source), *source, owner(socket), socket);
      origins_[&socket] = source;
    }
  }
  bNodeSocket *materialize(const Scalar &value) {
    if (const auto *socket = std::get_if<bNodeSocket *>(&value)) {
      return *socket;
    }
    bNode &n = node("ShaderNodeValue"_ustr);
    output(n, 0).default_value_typed<bNodeSocketValueFloat>()->value =
        std::get<float>(value);
    return &output(n, 0);
  }
  Scalar op(int operation, Scalar a, Scalar b = 0.0f, Scalar c = 0.0f) {
    if (const auto *x = std::get_if<float>(&a)) {
      if (const auto *y = std::get_if<float>(&b)) {
        switch (operation) {
        case NODE_MATH_ADD:
          return *x + *y;
        case NODE_MATH_SUBTRACT:
          return *x - *y;
        case NODE_MATH_MULTIPLY:
          return *x * *y;
        case NODE_MATH_DIVIDE:
          return *y == 0 ? 0.0f : *x / *y;
        case NODE_MATH_SQRT:
          return std::sqrt(std::max(*x, 0.0f));
        case NODE_MATH_ABSOLUTE:
          return std::abs(*x);
        case NODE_MATH_GREATER_THAN:
          return float(*x > *y);
        case NODE_MATH_LESS_THAN:
          return float(*x < *y);
        case NODE_MATH_TRUNC:
          return std::trunc(*x);
        case NODE_MATH_SINE:
          return std::sin(*x);
        case NODE_MATH_COSINE:
          return std::cos(*x);
        case NODE_MATH_MAXIMUM:
          return std::max(*x, *y);
        case NODE_MATH_MINIMUM:
          return std::min(*x, *y);
        case NODE_MATH_COMPARE:
          return float(*x == *y);
        case NODE_MATH_FLOOR:
          return std::floor(*x);
        case NODE_MATH_CEIL:
          return std::ceil(*x);
        case NODE_MATH_FRACTION:
          return *x - std::floor(*x);
        case NODE_MATH_ROUND:
          return std::floor(*x + 0.5f);
        case NODE_MATH_SIGN:
          return float((*x > 0) - (*x < 0));
        case NODE_MATH_POWER:
          return std::pow(*x, *y);
        case NODE_MATH_MODULO:
          return *y == 0.0f ? 0.0f : std::fmod(*x, *y);
        case NODE_MATH_TANGENT:
          return std::tan(*x);
        case NODE_MATH_ARCSINE:
          return std::asin(std::clamp(*x, -1.0f, 1.0f));
        case NODE_MATH_ARCCOSINE:
          return std::acos(std::clamp(*x, -1.0f, 1.0f));
        case NODE_MATH_ARCTANGENT:
          return std::atan(*x);
        case NODE_MATH_ARCTAN2:
          return std::atan2(*x, *y);
        case NODE_MATH_EXPONENT:
          return std::exp(*x);
        default:
          break;
        }
      }
    }
    bNode &n = node("ShaderNodeMath"_ustr);
    n.custom1 = operation;
    assign(input(n, 0), a);
    assign(input(n, 1), b);
    assign(input(n, 2), c);
    return &output(n, 0);
  }
  Scalar add(Scalar a, Scalar b) { return op(NODE_MATH_ADD, a, b); }
  Scalar sub(Scalar a, Scalar b) { return op(NODE_MATH_SUBTRACT, a, b); }
  Scalar mul(Scalar a, Scalar b) { return op(NODE_MATH_MULTIPLY, a, b); }
  Scalar div(Scalar a, Scalar b) { return op(NODE_MATH_DIVIDE, a, b); }
  Scalar select(Scalar test, Scalar yes, Scalar no) {
    return add(mul(test, yes), mul(sub(1.0f, test), no));
  }
  Scalar equal_index(Scalar a, int b) {
    return op(NODE_MATH_LESS_THAN,
              op(NODE_MATH_ABSOLUTE, sub(op(NODE_MATH_TRUNC, a), float(b))),
              0.5f);
  }
  bNodeSocket *origin(bNodeSocket &socket) {
    auto found = origins_.find(&socket);
    return found == origins_.end() ? nullptr : found->second;
  }
  Scalar scalar(bNodeSocket &socket) {
    if (socket.is_input()) {
      if (bNodeSocket *source = origin(socket)) {
        return scalar(*source);
      }
      if (socket.type == SOCK_INT) {
        return float(socket.default_value_typed<bNodeSocketValueInt>()->value);
      }
      if (socket.type == SOCK_BOOLEAN) {
        return float(
            socket.default_value_typed<bNodeSocketValueBoolean>()->value);
      }
      return socket.default_value_typed<bNodeSocketValueFloat>()->value;
    }
    if (auto found = scalars_.find(&socket); found != scalars_.end()) {
      return found->second;
    }
    bNode &n = owner(socket);
    Scalar result = &socket;
    if (n.is_type("ShaderNodeWrangle"_ustr)) {
      wrangle(n);
      return scalars_.at(&socket);
    }
    if (n.is_type("ShaderNodeObjectTransform"_ustr)) {
      result = &output(object_attribute(n, 6), 2);
    } else if (n.is_type("ShaderNodeAttribute"_ustr) && StringRef(socket.identifier) == "Attribute") {
      const auto &storage = *static_cast<const NodeShaderAttribute *>(n.storage);
      const Vector3 value = vector(output(legacy_attribute(n), 1));
      result = value[0];
      if (storage.data_type == SHD_ATTRIBUTE_DATA_INT) {
        result = op(NODE_MATH_TRUNC, result);
      }
      else if (storage.data_type == SHD_ATTRIBUTE_DATA_BOOLEAN) {
        result = op(NODE_MATH_GREATER_THAN, op(NODE_MATH_ABSOLUTE, result), 0.0f);
      }
    } else if (n.is_type("FunctionNodeSeparateMatrix"_ustr)) {
      result = matrix(input(n, 0))[BLI_findindex(&n.outputs, &socket)];
    } else if (n.is_type("FunctionNodeGetMatrixComponent"_ustr)) {
      const Matrix m = matrix(input(n, 0));
      result = 0.0f;
      const Scalar column = scalar(input(n, 1)), row = scalar(input(n, 2));
      for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
          result = add(result, mul(m[c * 4 + r], mul(equal_index(column, c),
                                                     equal_index(row, r))));
        }
      }
    } else if (n.is_type("FunctionNodeMatrixDeterminant"_ustr)) {
      result = determinant(matrix(input(n, 0)), {0, 1, 2, 3}, {0, 1, 2, 3}, 4);
    } else if (n.is_type("FunctionNodeInvertMatrix"_ustr)) {
      result =
          op(NODE_MATH_GREATER_THAN,
             op(NODE_MATH_ABSOLUTE, determinant(matrix(input(n, 0)),
                                                {0, 1, 2, 3}, {0, 1, 2, 3}, 4)),
             0.0f);
    } else if (n.is_type("FunctionNodeRotationToQuaternion"_ustr)) {
      result = rotation(input(n, 0))[BLI_findindex(&n.outputs, &socket)];
    } else if (n.is_type("FunctionNodeRotationToAxisAngle"_ustr)) {
      const Quaternion q = rotation(input(n, 0));
      const Scalar sine = op(NODE_MATH_SQRT, dot3({q[1], q[2], q[3]}, {q[1], q[2], q[3]}));
      result = select(op(NODE_MATH_LESS_THAN, sine, 0.0005f), 0.0f,
                      mul(2.0f, op(NODE_MATH_ARCTAN2, sine, q[0])));
    }
    scalars_[&socket] = result;
    return result;
  }
  Scalar determinant(const Matrix &m, std::array<int, 4> cols,
                     std::array<int, 4> rows, int size) {
    if (size == 1) {
      return m[cols[0] * 4 + rows[0]];
    }
    Scalar value = 0.0f;
    for (int i = 0; i < size; i++) {
      std::array<int, 4> subcols{}, subrows{};
      for (int j = 1; j < size; j++) {
        subrows[j - 1] = rows[j];
      }
      for (int j = 0, k = 0; j < size; j++) {
        if (j != i) {
          subcols[k++] = cols[j];
        }
      }
      Scalar term = mul(m[cols[i] * 4 + rows[0]],
                        determinant(m, subcols, subrows, size - 1));
      value = i % 2 ? sub(value, term) : add(value, term);
    }
    return value;
  }
  Matrix identity() {
    Matrix value;
    for (int c = 0; c < 4; c++) {
      for (int r = 0; r < 4; r++) {
        value[c * 4 + r] = float(c == r);
      }
    }
    return value;
  }
  Vector3 vector(bNodeSocket &socket) {
    if (socket.is_input()) {
      if (bNodeSocket *source = origin(socket)) {
        return vector(*source);
      }
      const auto *v = socket.default_value_typed<bNodeSocketValueVector>();
      return {v->value[0], v->value[1], v->value[2]};
    }
    if (auto found = vectors_.find(&socket); found != vectors_.end()) {
      return found->second;
    }
    bNode &n = owner(socket);
    Vector3 result;
    if (n.is_type("ShaderNodeWrangle"_ustr)) {
      wrangle(n);
      return vectors_.at(&socket);
    }
    if (n.is_type("ShaderNodeObjectTransform"_ustr)) {
      result = vector(output(object_attribute(n, StringRef(socket.identifier) == "Location" ? 3 : 5), 1));
    } else if (n.is_type("ShaderNodeAttribute"_ustr) && StringRef(socket.identifier) == "Attribute") {
      result = vector(output(legacy_attribute(n), 1));
    } else if (n.is_type("FunctionNodeRotateVector"_ustr)) {
      result = rotate_vector(rotation(input(n, 1)), vector(input(n, 0)));
    } else if (n.is_type("FunctionNodeRotationToAxisAngle"_ustr)) {
      const Quaternion q = rotation(input(n, 0));
      const Vector3 axis{q[1], q[2], q[3]};
      const Scalar sine = op(NODE_MATH_SQRT, dot3(axis, axis));
      result = choose3(op(NODE_MATH_LESS_THAN, sine, 0.0005f),
                       {0.0f, select(op(NODE_MATH_LESS_THAN, q[0], 0.0f), -1.0f, 1.0f), 0.0f},
                       normalize3(axis));
    } else if (n.is_type("FunctionNodeTransformPoint"_ustr) ||
        n.is_type("FunctionNodeTransformDirection"_ustr) ||
        n.is_type("FunctionNodeProjectPoint"_ustr)) {
      const Vector3 v = vector(input(n, 0));
      const Matrix m = matrix(input(n, 1));
      const bool direction = n.is_type("FunctionNodeTransformDirection"_ustr);
      for (int r = 0; r < 3; r++) {
        result[r] = direction ? Scalar(0.0f) : m[12 + r];
        for (int c = 0; c < 3; c++) {
          result[r] = add(result[r], mul(m[c * 4 + r], v[c]));
        }
      }
      if (n.is_type("FunctionNodeProjectPoint"_ustr)) {
        Scalar w = m[15];
        for (int c = 0; c < 3; c++) {
          w = add(w, mul(m[c * 4 + 3], v[c]));
        }
        w = op(NODE_MATH_ABSOLUTE, w);
        for (Scalar &component : result) {
          component = div(component, w);
        }
      }
    } else if (n.is_type("FunctionNodeSeparateTransform"_ustr)) {
      const Matrix m = matrix(input(n, 0));
      if (StringRef(socket.identifier) == "Translation") {
        result = {m[12], m[13], m[14]};
      } else {
        for (int c = 0; c < 3; c++) {
          Scalar norm = 0.0f;
          for (int r = 0; r < 3; r++) {
            norm = add(norm, mul(m[c * 4 + r], m[c * 4 + r]));
          }
          result[c] = op(NODE_MATH_SQRT, norm);
        }
      }
    } else if (n.is_type("FunctionNodeMatrixSVD"_ustr)) {
      svd(n);
      result = singular_values_.at(&n);
    } else if (n.is_type("FunctionNodeRotationToEuler"_ustr)) {
      const Quaternion q = rotation(input(n, 0));
      const Scalar w = q[0], x = q[1], y = q[2], z = q[3];
      /* Match Blender's normalized_to_eul2, including the minimum-angle branch and
       * the singular case. A single asin formula chooses a different Euler branch. */
      const Scalar m00 = sub(1.0f, mul(2.0f, add(mul(y, y), mul(z, z))));
      const Scalar m01 = mul(2.0f, add(mul(x, y), mul(w, z)));
      const Scalar m02 = mul(2.0f, sub(mul(x, z), mul(w, y)));
      const Scalar m12 = mul(2.0f, add(mul(y, z), mul(w, x)));
      const Scalar m22 = sub(1.0f, mul(2.0f, add(mul(x, x), mul(y, y))));
      const Scalar m21 = mul(2.0f, sub(mul(y, z), mul(w, x)));
      const Scalar m11 = sub(1.0f, mul(2.0f, add(mul(x, x), mul(z, z))));
      const Scalar cy = op(NODE_MATH_SQRT, add(mul(m00, m00), mul(m01, m01)));
      const Scalar minus_m02 = mul(-1.0f, m02);
      Vector3 first{op(NODE_MATH_ARCTAN2, m12, m22),
                    op(NODE_MATH_ARCTAN2, minus_m02, cy),
                    op(NODE_MATH_ARCTAN2, m01, m00)};
      const Vector3 second{op(NODE_MATH_ARCTAN2, mul(-1.0f, m12), mul(-1.0f, m22)),
                           op(NODE_MATH_ARCTAN2, minus_m02, mul(-1.0f, cy)),
                           op(NODE_MATH_ARCTAN2, mul(-1.0f, m01), mul(-1.0f, m00))};
      Scalar first_norm = 0.0f, second_norm = 0.0f;
      for (int i = 0; i < 3; i++) {
        first_norm = add(first_norm, op(NODE_MATH_ABSOLUTE, first[i]));
        second_norm = add(second_norm, op(NODE_MATH_ABSOLUTE, second[i]));
      }
      const Scalar use_second = op(NODE_MATH_GREATER_THAN, first_norm, second_norm);
      const Scalar nonsingular = op(NODE_MATH_GREATER_THAN, cy, 16.0f * FLT_EPSILON);
      const Vector3 singular{op(NODE_MATH_ARCTAN2, mul(-1.0f, m21), m11), first[1], 0.0f};
      for (int i = 0; i < 3; i++) {
        result[i] = select(nonsingular, select(use_second, second[i], first[i]), singular[i]);
      }
    } else {
      bNode &split = node("ShaderNodeSeparateXYZ"_ustr);
      bke::node_add_link(tree_, n, socket, split, input(split, 0));
      result = {&output(split, 0), &output(split, 1), &output(split, 2)};
    }
    vectors_[&socket] = result;
    return result;
  }
#include "shader_nodes_rotation.hh"

  Quaternion rotation(bNodeSocket &socket) {
    if (socket.is_input()) {
      if (bNodeSocket *source = origin(socket)) {
        return rotation(*source);
      }
      const auto *e = socket.default_value_typed<bNodeSocketValueRotation>();
      return euler({e->value_euler[0], e->value_euler[1], e->value_euler[2]});
    }
    if (auto found = rotations_.find(&socket); found != rotations_.end()) {
      return found->second;
    }
    bNode &n = owner(socket);
    Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};
    if (n.is_type("ShaderNodeObjectTransform"_ustr)) {
      q = object_components(n, 4);
    } else if (n.is_type("ShaderNodeAttribute"_ustr)) {
      bNode &attr = legacy_attribute(n);
      const Vector3 xyz = vector(output(attr, 1));
      q = normalize_rotation({xyz[0], xyz[1], xyz[2], &output(attr, 3)});
    } else if (n.is_type("FunctionNodeInputRotation"_ustr)) {
      const auto &storage = *static_cast<const NodeInputRotation *>(n.storage);
      q = euler({storage.rotation_euler[0], storage.rotation_euler[1], storage.rotation_euler[2]});
    } else if (n.is_type("FunctionNodeAxisAngleToRotation"_ustr)) {
      q = axis_angle_rotation(vector(input(n, 0)), scalar(input(n, 1)));
    } else if (n.is_type("FunctionNodeInvertRotation"_ustr)) {
      q = rotation(input(n, 0));
      for (int i = 1; i < 4; i++) {
        q[i] = mul(-1.0f, q[i]);
      }
    } else if (n.is_type("FunctionNodeRotateRotation"_ustr)) {
      const Quaternion a = rotation(input(n, 0)), b = rotation(input(n, 1));
      q = n.custom1 == 0 ? multiply_rotation(b, a) : multiply_rotation(a, b);
    } else if (n.is_type("FunctionNodeAlignRotationToVector"_ustr)) {
      q = align_rotation(n);
    } else if (n.is_type("FunctionNodeAxesToRotation"_ustr)) {
      q = axes_rotation(n);
    } else if (n.is_type("ShaderNodeMix"_ustr)) {
      q = mix_rotation(n);
    } else if (n.is_type("FunctionNodeQuaternionToRotation"_ustr)) {
      Scalar norm = 0.0f;
      for (int i = 0; i < 4; i++) {
        q[i] = scalar(input(n, i));
        norm = add(norm, mul(q[i], q[i]));
      }
      norm = op(NODE_MATH_SQRT, norm);
      Scalar nonzero = op(NODE_MATH_GREATER_THAN, norm, 0.0f);
      for (int i = 0; i < 4; i++) {
        q[i] = select(nonzero, div(q[i], norm), float(i == 0));
      }
    } else if (n.is_type("FunctionNodeEulerToRotation"_ustr)) {
      q = euler(vector(input(n, 0)));
    } else if (n.is_type("FunctionNodeSeparateTransform"_ustr)) {
      Matrix m = matrix(input(n, 0));
      for (int c = 0; c < 3; c++) {
        Scalar length = 0.0f;
        for (int r = 0; r < 3; r++) {
          length = add(length, mul(m[c * 4 + r], m[c * 4 + r]));
        }
        length = op(NODE_MATH_SQRT, length);
        for (int r = 0; r < 3; r++) {
          m[c * 4 + r] = div(m[c * 4 + r], length);
        }
      }
      const Scalar sign =
          select(op(NODE_MATH_LESS_THAN,
                    determinant(m, {0, 1, 2, 0}, {0, 1, 2, 0}, 3), 0.0f),
                 -1.0f, 1.0f);
      for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
          m[c * 4 + r] = mul(m[c * 4 + r], sign);
        }
      }
      q[0] = mul(
          0.5f, op(NODE_MATH_SQRT, op(NODE_MATH_MAXIMUM, 0.0f,
                                      add(1.0f, add(m[0], add(m[5], m[10]))))));
      for (int i = 0; i < 3; i++) {
        const int j = (i + 1) % 3, k = (i + 2) % 3;
        Scalar mag = op(
            NODE_MATH_SQRT,
            op(NODE_MATH_MAXIMUM, 0.0f,
               sub(add(1.0f, m[i * 4 + i]), add(m[j * 4 + j], m[k * 4 + k]))));
        const Scalar negative =
            op(NODE_MATH_LESS_THAN, sub(m[j * 4 + k], m[k * 4 + j]), 0.0f);
        q[i + 1] = mul(mul(0.5f, mag), select(negative, -1.0f, 1.0f));
      }
    }
    rotations_[&socket] = q;
    return q;
  }
  Quaternion euler(Vector3 v) {
    Scalar c[3], s[3];
    for (int i = 0; i < 3; i++) {
      c[i] = op(NODE_MATH_COSINE, mul(v[i], 0.5f));
      s[i] = op(NODE_MATH_SINE, mul(v[i], 0.5f));
    }
    return {add(mul(mul(c[0], c[1]), c[2]), mul(mul(s[0], s[1]), s[2])),
            sub(mul(mul(s[0], c[1]), c[2]), mul(mul(c[0], s[1]), s[2])),
            add(mul(mul(c[0], s[1]), c[2]), mul(mul(s[0], c[1]), s[2])),
            sub(mul(mul(c[0], c[1]), s[2]), mul(mul(s[0], s[1]), c[2]))};
  }
  Matrix matrix(bNodeSocket &socket) {
    if (socket.is_input()) {
      if (bNodeSocket *source = origin(socket)) {
        return matrix(*source);
      }
      return identity();
    }
    if (auto found = matrices_.find(&socket); found != matrices_.end()) {
      return found->second;
    }
    bNode &n = owner(socket);
    Matrix m = identity();
    if (n.is_type("ShaderNodeWrangle"_ustr)) {
      wrangle(n);
      return matrices_.at(&socket);
    }
    if (n.is_type("ShaderNodeObjectTransform"_ustr)) {
      for (int c = 0; c < 4; c++) {
        const auto column = object_components(n, c);
        for (int r = 0; r < 4; r++) {
          m[c * 4 + r] = column[r];
        }
      }
    } else if (n.is_type("FunctionNodeCombineMatrix"_ustr)) {
      for (int i = 0; i < 16; i++) {
        m[i] = scalar(input(n, i));
      }
    } else if (n.is_type("ShaderNodeAttribute"_ustr)) {
      const auto &storage =
          *static_cast<const NodeShaderAttribute *>(n.storage);
      if (storage.type != SHD_ATTRIBUTE_GEOMETRY) {
        m.fill(0.0f);
        matrices_[&socket] = m;
        return m;
      }
      /* Linked constant strings are folded into the socket by the common
       * inliner. */
      const char *attribute_name = storage.name;
      if (const bNodeSocket *name_socket =
              bke::node_find_socket(n, SOCK_IN, "Name"_ustr)) {
        if (name_socket->default_value) {
          const char *name =
              name_socket->default_value_typed<bNodeSocketValueString>()->value;
          if (name && name[0]) {
            attribute_name = name;
          }
        }
      }
      for (int c = 0; c < 4; c++) {
        bNode &attr = node("ShaderNodeAttribute"_ustr);
        auto &copy = *static_cast<NodeShaderAttribute *>(attr.storage);
        const std::string name =
            shader_matrix_attribute_column_name(attribute_name, c);
        STRNCPY(copy.name, name.c_str());
        copy.type = storage.type;
        bNode &split = node("ShaderNodeSeparateColor"_ustr);
        bke::node_add_link(tree_, attr, output(attr, 0), split,
                           input(split, 0));
        for (int r = 0; r < 3; r++) {
          m[c * 4 + r] = &output(split, r);
        }
        m[c * 4 + 3] = &output(attr, 3);
      }
    } else if (n.is_type("FunctionNodeTransposeMatrix"_ustr)) {
      const Matrix a = matrix(input(n, 0));
      for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
          m[c * 4 + r] = a[r * 4 + c];
        }
      }
    } else if (n.is_type("FunctionNodeMatrixMultiply"_ustr)) {
      const Matrix a = matrix(input(n, 0)), b = matrix(input(n, 1));
      for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
          m[c * 4 + r] = 0.0f;
          for (int k = 0; k < 4; k++) {
            m[c * 4 + r] = add(m[c * 4 + r], mul(a[k * 4 + r], b[c * 4 + k]));
          }
        }
      }
    } else if (n.is_type("FunctionNodeInvertMatrix"_ustr)) {
      const Matrix a = matrix(input(n, 0));
      const Scalar det = determinant(a, {0, 1, 2, 3}, {0, 1, 2, 3}, 4);
      const Scalar ok =
          op(NODE_MATH_GREATER_THAN, op(NODE_MATH_ABSOLUTE, det), 0.0f);
      for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
          std::array<int, 4> cols{}, rows{};
          for (int i = 0, j = 0, k = 0; i < 4; i++) {
            if (i != r) {
              cols[j++] = i;
            }
            if (i != c) {
              rows[k++] = i;
            }
          }
          Scalar cofactor = determinant(a, cols, rows, 3);
          if ((c + r) % 2) {
            cofactor = mul(cofactor, -1.0f);
          }
          m[c * 4 + r] = select(ok, div(cofactor, det), float(c == r));
        }
      }
    } else if (n.is_type("FunctionNodeSetMatrixComponent"_ustr)) {
      m = matrix(input(n, 0));
      const Scalar col = scalar(input(n, 1)), row = scalar(input(n, 2)),
                   value = scalar(input(n, 3));
      for (int c = 0; c < 4; c++) {
        for (int r = 0; r < 4; r++) {
          m[c * 4 + r] = select(mul(equal_index(col, c), equal_index(row, r)),
                                value, m[c * 4 + r]);
        }
      }
    } else if (n.is_type("FunctionNodeCombineTransform"_ustr)) {
      const Vector3 t = vector(input(n, 0)), scale = vector(input(n, 2));
      const Quaternion q = rotation(input(n, 1));
      const Scalar w = q[0], x = q[1], y = q[2], z = q[3];
      m[0] = sub(1.0f, mul(2.0f, add(mul(y, y), mul(z, z))));
      m[1] = mul(2.0f, add(mul(x, y), mul(w, z)));
      m[2] = mul(2.0f, sub(mul(x, z), mul(w, y)));
      m[4] = mul(2.0f, sub(mul(x, y), mul(w, z)));
      m[5] = sub(1.0f, mul(2.0f, add(mul(x, x), mul(z, z))));
      m[6] = mul(2.0f, add(mul(y, z), mul(w, x)));
      m[8] = mul(2.0f, add(mul(x, z), mul(w, y)));
      m[9] = mul(2.0f, sub(mul(y, z), mul(w, x)));
      m[10] = sub(1.0f, mul(2.0f, add(mul(x, x), mul(y, y))));
      for (int c = 0; c < 3; c++) {
        for (int r = 0; r < 3; r++) {
          m[c * 4 + r] = mul(m[c * 4 + r], scale[c]);
        }
        m[12 + c] = t[c];
      }
    } else if (n.is_type("FunctionNodeMatrixSVD"_ustr)) {
      svd(n);
      return matrices_.at(&socket);
    }
    matrices_[&socket] = m;
    return m;
  }
  void svd(bNode &n) {
    if (singular_values_.contains(&n)) {
      return;
    }
    Matrix a = matrix(input(n, 0)), v = identity();
    /* Match the existing Matrix SVD node's documented 3x3 decomposition,
     * carried in 4x4 sockets. */
    for (int sweep = 0; sweep < 8; sweep++) {
      for (const auto pair : {std::array<int, 2>{0, 1}, {0, 2}, {1, 2}}) {
        const int p = pair[0], q = pair[1];
        Scalar aa = 0.0f, bb = 0.0f, ab = 0.0f;
        for (int r = 0; r < 3; r++) {
          aa = add(aa, mul(a[p * 4 + r], a[p * 4 + r]));
          bb = add(bb, mul(a[q * 4 + r], a[q * 4 + r]));
          ab = add(ab, mul(a[p * 4 + r], a[q * 4 + r]));
        }
        const Scalar tau = div(sub(bb, aa), mul(2.0f, ab));
        Scalar t = div(select(op(NODE_MATH_LESS_THAN, tau, 0.0f), -1.0f, 1.0f),
                       add(op(NODE_MATH_ABSOLUTE, tau),
                           op(NODE_MATH_SQRT, add(1.0f, mul(tau, tau)))));
        t = select(op(NODE_MATH_GREATER_THAN, op(NODE_MATH_ABSOLUTE, ab),
                      mul(1e-7f, op(NODE_MATH_SQRT, mul(aa, bb)))),
                   t, 0.0f);
        const Scalar c = div(1.0f, op(NODE_MATH_SQRT, add(1.0f, mul(t, t)))),
                     s = mul(c, t);
        for (Matrix *m : {&a, &v}) {
          for (int r = 0; r < 3; r++) {
            const Scalar x = (*m)[p * 4 + r], y = (*m)[q * 4 + r];
            (*m)[p * 4 + r] = sub(mul(c, x), mul(s, y));
            (*m)[q * 4 + r] = add(mul(s, x), mul(c, y));
          }
        }
      }
    }
    Matrix u = identity();
    Vector3 values;
    for (int c = 0; c < 3; c++) {
      Scalar length = 0.0f;
      for (int r = 0; r < 3; r++) {
        length = add(length, mul(a[c * 4 + r], a[c * 4 + r]));
      }
      values[c] = op(NODE_MATH_SQRT, length);
      for (int r = 0; r < 3; r++) {
        u[c * 4 + r] = div(a[c * 4 + r], values[c]);
      }
    }
    for (const auto pair : {std::array<int, 2>{0, 1}, {1, 2}, {0, 1}}) {
      const int p = pair[0], q = pair[1];
      const Scalar swap = op(NODE_MATH_LESS_THAN, values[p], values[q]);
      const Scalar x = values[p], y = values[q];
      values[p] = select(swap, y, x);
      values[q] = select(swap, x, y);
      for (Matrix *m : {&u, &v}) {
        for (int r = 0; r < 3; r++) {
          const Scalar x = (*m)[p * 4 + r], y = (*m)[q * 4 + r];
          (*m)[p * 4 + r] = select(swap, y, x);
          (*m)[q * 4 + r] = select(swap, x, y);
        }
      }
    }
    matrices_[&output(n, 0)] = u;
    matrices_[&output(n, 2)] = v;
    singular_values_[&n] = values;
  }

#include "shader_nodes_wrangle_graph.hh"

public:
  explicit MatrixLowering(bNodeTree &tree) : tree_(tree) {
    for (bNode &n : tree.nodes) {
      for (bNodeSocket &s : n.inputs) {
        owners_[&s] = &n;
      }
      for (bNodeSocket &s : n.outputs) {
        owners_[&s] = &n;
      }
    }
    for (bNodeLink &link : tree.links) {
      origins_[link.tosock] = link.fromsock;
    }
  }
  void run() {
    Vector<bNode *> original;
    for (bNode &n : tree_.nodes) {
      original.append(&n);
    }
    std::unordered_map<const bNodeSocket *, bNodeSocket *> replacements;
    for (bNode *n : original) {
      if (n->is_type("ShaderNodeAttribute"_ustr) &&
          static_cast<const NodeShaderAttribute *>(n->storage)->data_type == SHD_ATTRIBUTE_DATA_LEGACY)
      {
        continue;
      }
      if (!STR_ELEM(
              n->idname, "FunctionNodeSeparateMatrix",
              "FunctionNodeGetMatrixComponent", "FunctionNodeMatrixDeterminant",
              "FunctionNodeInvertMatrix", "FunctionNodeTransformPoint",
              "FunctionNodeTransformDirection", "FunctionNodeProjectPoint",
              "FunctionNodeSeparateTransform",
              "FunctionNodeRotationToQuaternion", "FunctionNodeRotationToEuler",
              "FunctionNodeRotationToAxisAngle", "FunctionNodeRotateVector", "ShaderNodeAttribute",
              "FunctionNodeMatrixSVD", "ShaderNodeWrangle", "ShaderNodeObjectTransform")) {
        continue;
      }
      for (bNodeSocket &socket : n->outputs) {
        if (!socket.is_available()) {
          continue;
        }
        if (n->is_type("ShaderNodeAttribute"_ustr) && socket.type == SOCK_RGBA &&
            StringRef(socket.identifier) == "Attribute") {
          replacements[&socket] = &output(legacy_attribute(*n), 0);
        }
        if (socket.type == SOCK_FLOAT || socket.type == SOCK_BOOLEAN || socket.type == SOCK_INT) {
          replacements[&socket] = materialize(scalar(socket));
        } else if (socket.type == SOCK_VECTOR ||
                   (socket.type == SOCK_ROTATION && n->is_type("ShaderNodeObjectTransform"_ustr))) {
          Vector3 value;
          if (socket.type == SOCK_ROTATION) {
            /* Shader links cast rotations to Euler vectors. Keep quaternion consumers in
             * the symbolic lowering, and materialize Euler for ordinary shader inputs. */
            bNode &convert = node("FunctionNodeRotationToEuler"_ustr);
            origins_[&input(convert, 0)] = &socket;
            value = vector(output(convert, 0));
          } else {
            value = vector(socket);
          }
          bNode &combine = node("ShaderNodeCombineXYZ"_ustr);
          for (int i = 0; i < 3; i++) {
            assign(input(combine, i), value[i]);
          }
          replacements[&socket] = &output(combine, 0);
        }
      }
    }
    Vector<bNodeLink *> links;
    for (bNodeLink &link : tree_.links) {
      links.append(&link);
    }
    for (bNodeLink *link : links) {
      auto found = replacements.find(link->fromsock);
      if (found != replacements.end() && found->second != link->fromsock) {
        bNodeSocket *target = link->tosock;
        bNodeSocket *source = found->second;
        bke::node_remove_link(&tree_, *link);
        bke::node_add_link(tree_, owner(*source), *source, owner(*target),
                           *target);
      }
    }
  }
};
} // namespace

bool lower_shader_matrix_nodes(bNodeTree &tree, std::string &r_error) {
  try {
    MatrixLowering(tree).run();
    return true;
  } catch (const std::exception &error) {
    r_error = error.what();
    return false;
  }
}
} // namespace blender::nodes
