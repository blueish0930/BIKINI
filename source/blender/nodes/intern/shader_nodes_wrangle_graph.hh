/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Implementation members of MatrixLowering. Symbolically execute the existing
 * VEX bytecode into renderer-native shader operations. Geometry attributes
 * remain runtime inputs. */
struct WrValue {
  vex::Type type = vex::Type::Float;
  Matrix data{};
  std::string text;
  std::vector<WrValue> array;
};
std::unordered_map<const bNode *, std::unordered_map<std::string, WrValue>>
    wr_results_;

WrValue wr_scalar(Scalar x, vex::Type type = vex::Type::Float) {
  WrValue v;
  v.type = type;
  v.data.fill(0.0f);
  v.data[0] = x;
  return v;
}
int wr_size(const WrValue &v) {
  return vex::type_is_matrix(v.type)
             ? vex::type_linear_dim(v.type) * vex::type_linear_dim(v.type)
         : vex::type_has_xyzw(v.type) ? vex::type_linear_dim(v.type)
                                      : 1;
}
Scalar wr_component(const WrValue &v, int i) {
  return v.data[wr_size(v) == 1 ? 0 : i];
}
WrValue wr_socket(bNodeSocket &s) {
  if (s.type == SOCK_MATRIX) {
    WrValue v;
    v.type = vex::Type::Matrix;
    v.data = matrix(s);
    return v;
  }
  if (s.type == SOCK_VECTOR) {
    WrValue v;
    v.type = vex::Type::Vector;
    auto a = vector(s);
    for (int i = 0; i < 3; i++) {
      v.data[i] = a[i];
    }
    return v;
  }
  if (s.type == SOCK_ROTATION) {
    auto q = rotation(s);
    WrValue v;
    v.type = vex::Type::Rotation;
    v.data[0] = q[1];
    v.data[1] = q[2];
    v.data[2] = q[3];
    v.data[3] = q[0];
    return v;
  }
  if (s.type == SOCK_RGBA) {
    WrValue v;
    v.type = vex::Type::Color;
    if (s.is_input() && !origin(s)) {
      auto *c = s.default_value_typed<bNodeSocketValueRGBA>();
      for (int i = 0; i < 4; i++) {
        v.data[i] = c->value[i];
      }
    } else {
      bNodeSocket *src = s.is_input() ? origin(s) : &s;
      bNode &split = node("ShaderNodeSeparateColor"_ustr);
      bke::node_add_link(tree_, owner(*src), *src, split, input(split, 0));
      for (int i = 0; i < 3; i++) {
        v.data[i] = &output(split, i);
      }
      v.data[3] = 1.0f;
    }
    return v;
  }
  return wr_scalar(scalar(s), s.type == SOCK_INT       ? vex::Type::Int
                              : s.type == SOCK_BOOLEAN ? vex::Type::Bool
                                                       : vex::Type::Float);
}
WrValue wr_binary(int operation, const WrValue &a, const WrValue &b) {
  WrValue r;
  r.type = wr_size(a) >= wr_size(b) ? a.type : b.type;
  const bool am = vex::type_is_matrix(a.type), bm = vex::type_is_matrix(b.type);
  if (operation == NODE_MATH_MULTIPLY && am && bm) {
    r.type = vex::Type::Matrix;
    r.data.fill(0.0f);
    for (int c = 0; c < 4; c++) {
      for (int row = 0; row < 4; row++) {
        for (int k = 0; k < 4; k++) {
          r.data[c * 4 + row] = add(
              r.data[c * 4 + row], mul(a.data[k * 4 + row], b.data[c * 4 + k]));
        }
      }
    }
    return r;
  }
  if (operation == NODE_MATH_MULTIPLY && (am || bm) &&
      vex::type_has_xyzw(am ? b.type : a.type)) {
    r.type = am ? b.type : a.type;
    r.data.fill(0.0f);
    const WrValue &m = am ? a : b, &v = am ? b : a;
    const int size = wr_size(v);
    for (int i = 0; i < size; i++) {
      for (int k = 0; k < size; k++) {
        r.data[i] =
            add(r.data[i], mul(m.data[am ? k * 4 + i : i * 4 + k], v.data[k]));
      }
    }
    return r;
  }
  for (int i = 0; i < wr_size(r); i++) {
    r.data[i] = op(operation, wr_component(a, i), wr_component(b, i));
  }
  return r;
}
WrValue wr_select(Scalar condition, const WrValue &yes, const WrValue &no) {
  if (const float *c = std::get_if<float>(&condition)) {
    return *c != 0 ? yes : no;
  }
  WrValue r = yes;
  if (wr_size(no) > wr_size(r)) {
    r.type = no.type;
  }
  for (int i = 0; i < wr_size(r); i++) {
    r.data[i] = select(condition, wr_component(yes, i), wr_component(no, i));
  }
  return r;
}
WrValue wr_index(const WrValue &v, Scalar index) {
  WrValue r;
  if (vex::type_is_matrix(v.type)) {
    r.type = vex::Type::Vector4;
    for (int row = 0; row < 4; row++) {
      r.data[row] = 0.0f;
      for (int c = 0; c < 4; c++) {
        r.data[row] =
            add(r.data[row], mul(equal_index(index, c), v.data[c * 4 + row]));
      }
    }
  } else if (!v.array.empty()) {
    r = v.array.front();
    r.data.fill(0.0f);
    for (int i = 0; i < int(v.array.size()); i++) {
      r = wr_select(equal_index(index, i), v.array[i], r);
    }
  } else {
    r = wr_scalar(0.0f);
    for (int i = 0; i < wr_size(v); i++) {
      r.data[0] = add(r.data[0], mul(equal_index(index, i), v.data[i]));
    }
  }
  return r;
}
WrValue wr_builtin(vex::Builtin fn, const std::vector<WrValue> &a,
                   const std::unordered_map<std::string, WrValue> &channels) {
  using B = vex::Builtin;
  using T = vex::Type;
  auto arg = [&](int i) -> WrValue {
    return i < int(a.size()) ? a[i] : wr_scalar(0.0f);
  };
  auto unary = [&](int operation) {
    WrValue r = arg(0);
    for (int i = 0; i < wr_size(r); i++) {
      r.data[i] = op(operation, r.data[i]);
    }
    return r;
  };
  switch (fn) {
  case B::Sin:
    return unary(NODE_MATH_SINE);
  case B::Cos:
    return unary(NODE_MATH_COSINE);
  case B::Tan:
    return unary(NODE_MATH_TANGENT);
  case B::Asin:
    return unary(NODE_MATH_ARCSINE);
  case B::Acos:
    return unary(NODE_MATH_ARCCOSINE);
  case B::Atan:
    return unary(NODE_MATH_ARCTANGENT);
  case B::Abs:
    return unary(NODE_MATH_ABSOLUTE);
  case B::Floor:
    return unary(NODE_MATH_FLOOR);
  case B::Ceil:
    return unary(NODE_MATH_CEIL);
  case B::Round:
    return unary(NODE_MATH_ROUND);
  case B::Trunc:
    return unary(NODE_MATH_TRUNC);
  case B::Sqrt:
    return unary(NODE_MATH_SQRT);
  case B::Exp:
    return unary(NODE_MATH_EXPONENT);
  case B::Sign:
    return unary(NODE_MATH_SIGN);
  case B::Fract:
    return unary(NODE_MATH_FRACTION);
  case B::Log:
    return wr_binary(NODE_MATH_LOGARITHM, arg(0),
                     a.size() > 1 ? arg(1) : wr_scalar(float(M_E)));
  case B::Atan2:
    return wr_binary(NODE_MATH_ARCTAN2, arg(0), arg(1));
  case B::Min:
    return wr_binary(NODE_MATH_MINIMUM, arg(0), arg(1));
  case B::Max:
    return wr_binary(NODE_MATH_MAXIMUM, arg(0), arg(1));
  case B::Pow:
    return wr_binary(NODE_MATH_POWER, arg(0), arg(1));
  case B::Clamp:
    return wr_binary(NODE_MATH_MINIMUM,
                     wr_binary(NODE_MATH_MAXIMUM, arg(0), arg(1)), arg(2));
  case B::Radians:
    return wr_binary(NODE_MATH_MULTIPLY, arg(0),
                     wr_scalar(float(M_PI / 180.0)));
  case B::Degrees:
    return wr_binary(NODE_MATH_MULTIPLY, arg(0),
                     wr_scalar(float(180.0 / M_PI)));
  case B::FloatFn:
    return wr_scalar(arg(0).data[0]);
  case B::IntFn:
    return wr_scalar(op(NODE_MATH_TRUNC, arg(0).data[0]), T::Int);
  case B::BoolFn:
    return wr_scalar(op(NODE_MATH_GREATER_THAN,
                        op(NODE_MATH_ABSOLUTE, arg(0).data[0]), 0.0f),
                     T::Bool);
  case B::Chf:
  case B::Chi:
  case B::Chv:
  case B::Chb:
  case B::Chc:
  case B::Chm:
  case B::Chq:
  case B::Chu:
  case B::Chr:
  case B::Chs: {
    auto it = channels.find(arg(0).text);
    return it == channels.end() ? wr_scalar(0.0f) : it->second;
  }
  case B::Mix: {
    return wr_binary(NODE_MATH_ADD, arg(0),
                     wr_binary(NODE_MATH_MULTIPLY,
                               wr_binary(NODE_MATH_SUBTRACT, arg(1), arg(0)),
                               arg(2)));
  }
  case B::Smooth: {
    WrValue t = a.size() == 1
                    ? arg(0)
                    : wr_binary(NODE_MATH_DIVIDE,
                                wr_binary(NODE_MATH_SUBTRACT, arg(2), arg(0)),
                                wr_binary(NODE_MATH_SUBTRACT, arg(1), arg(0)));
    t = wr_binary(NODE_MATH_MINIMUM,
                  wr_binary(NODE_MATH_MAXIMUM, t, wr_scalar(0.0f)),
                  wr_scalar(1.0f));
    return wr_binary(
        NODE_MATH_MULTIPLY, wr_binary(NODE_MATH_MULTIPLY, t, t),
        wr_binary(NODE_MATH_SUBTRACT, wr_scalar(3.0f),
                  wr_binary(NODE_MATH_MULTIPLY, t, wr_scalar(2.0f))));
  }
  case B::Map: {
    WrValue factor = wr_binary(NODE_MATH_DIVIDE,
                               wr_binary(NODE_MATH_SUBTRACT, arg(0), arg(1)),
                               wr_binary(NODE_MATH_SUBTRACT, arg(2), arg(1)));
    return wr_binary(NODE_MATH_ADD, arg(3),
                     wr_binary(NODE_MATH_MULTIPLY, factor,
                               wr_binary(NODE_MATH_SUBTRACT, arg(4), arg(3))));
  }
  case B::Set:
  case B::Vec2Fn:
  case B::Vec3Fn:
  case B::Vec4Fn:
  case B::ColorFn: {
    std::vector<Scalar> c;
    for (const WrValue &v : a) {
      for (int i = 0; i < wr_size(v); i++) {
        c.push_back(v.data[i]);
      }
    }
    const int n = fn == B::Vec2Fn   ? 2
                  : fn == B::Vec3Fn ? 3
                  : (fn == B::Vec4Fn || fn == B::ColorFn)
                      ? 4
                      : std::min(4, int(c.size()));
    WrValue r;
    r.type = n == 2 ? T::Vector2 : n == 3 ? T::Vector : T::Vector4;
    for (int i = 0; i < n; i++) {
      r.data[i] = i < int(c.size())
                      ? c[i]
                      : (a.size() == 1 && c.size() == 1 ? c[0] : Scalar(0.0f));
    }
    return r;
  }
  case B::Identity:
  case B::MatrixFn: {
    WrValue r;
    r.type = T::Matrix;
    r.data = identity();
    if (fn == B::Identity || a.empty()) {
      return r;
    }
    if (a.size() == 1 && vex::type_is_matrix(a[0].type)) {
      return a[0];
    }
    if (a.size() == 1 && a[0].array.empty()) {
      for (int i = 0; i < 4; i++) {
        r.data[i * 5] = a[0].data[0];
      }
      return r;
    }
    std::vector<Scalar> c;
    for (const WrValue &v : a) {
      if (!v.array.empty()) {
        for (const WrValue &e : v.array) {
          c.push_back(e.data[0]);
        }
      } else {
        for (int i = 0; i < wr_size(v); i++) {
          c.push_back(v.data[i]);
        }
      }
    }
    for (int i = 0; i < std::min(16, int(c.size())); i++) {
      r.data[i] = c[i];
    }
    return r;
  }
  case B::Transpose: {
    WrValue r = arg(0);
    for (int c = 0; c < 4; c++) {
      for (int row = 0; row < 4; row++) {
        r.data[c * 4 + row] = a[0].data[row * 4 + c];
      }
    }
    return r;
  }
  case B::Determinant:
    return wr_scalar(determinant(arg(0).data, {0, 1, 2, 3}, {0, 1, 2, 3}, 4));
  case B::Invert: {
    bNode &combine = node("FunctionNodeCombineMatrix"_ustr);
    for (int i = 0; i < 16; i++) {
      assign(input(combine, i), arg(0).data[i]);
    }
    bNode &inv = node("FunctionNodeInvertMatrix"_ustr);
    bke::node_add_link(tree_, combine, output(combine, 0), inv, input(inv, 0));
    origins_[&input(inv, 0)] = &output(combine, 0);
    WrValue r;
    r.type = T::Matrix;
    r.data = matrix(output(inv, 0));
    return r;
  }
  case B::Dot:
  case B::Length:
  case B::Distance:
  case B::Normalize: {
    WrValue v = fn == B::Distance
                    ? wr_binary(NODE_MATH_SUBTRACT, arg(0), arg(1))
                    : arg(0);
    Scalar sum = 0.0f;
    for (int i = 0; i < wr_size(v); i++) {
      sum = add(sum, mul(v.data[i],
                         fn == B::Dot ? wr_component(arg(1), i) : v.data[i]));
    }
    if (fn == B::Dot) {
      return wr_scalar(sum);
    }
    const Scalar length = op(NODE_MATH_SQRT, sum);
    return fn == B::Normalize
               ? wr_binary(NODE_MATH_DIVIDE, v, wr_scalar(length))
               : wr_scalar(length);
  }
  case B::Cross: {
    WrValue r;
    r.type = T::Vector;
    for (int i = 0; i < 3; i++) {
      int j = (i + 1) % 3, k = (i + 2) % 3;
      r.data[i] = sub(mul(arg(0).data[j], arg(1).data[k]),
                      mul(arg(0).data[k], arg(1).data[j]));
    }
    return r;
  }
  case B::Noise:
  case B::Hash:
  case B::Rand: {
    if (fn == B::Rand && a.empty()) {
      throw std::runtime_error(
          "Cycles Wrangle: rand requires an explicit seed");
    }
    if (fn == B::Rand && ELEM(arg(0).type, T::Int, T::Bool)) {
      if (const float *seed = std::get_if<float>(&a[0].data[0])) {
        return wr_scalar(noise::hash_to_float(uint32_t(int(*seed))));
      }
      throw std::runtime_error(
          "Cycles Wrangle: dynamic integer rand seeds are unsupported");
    }
    WrValue v = arg(0);
    const int dim = fn == B::Rand ? std::min(wr_size(v), 4) : 3;
    bNode &noise = node("ShaderNodeTexWhiteNoise"_ustr);
    noise.custom1 = dim;
    if (dim == 1) {
      input(noise, 1).flag &= ~SOCK_UNAVAIL;
      assign(input(noise, 1), v.data[0]);
    } else {
      bNode &xyz = node("ShaderNodeCombineXYZ"_ustr);
      for (int i = 0; i < 3; i++) {
        assign(input(xyz, i), wr_component(v, i));
      }
      bke::node_add_link(tree_, xyz, output(xyz, 0), noise, input(noise, 0));
      if (dim == 4) {
        input(noise, 1).flag &= ~SOCK_UNAVAIL;
        assign(input(noise, 1), v.data[3]);
      }
    }
    return wr_scalar(&output(noise, 0));
  }
  case B::TransformPoint:
  case B::TransformDirection:
  case B::ProjectPoint: {
    WrValue v = arg(0);
    v.type = T::Vector4;
    v.data[3] = fn == B::TransformDirection ? 0.0f : 1.0f;
    WrValue r = wr_binary(NODE_MATH_MULTIPLY, arg(1), v);
    if (fn == B::ProjectPoint) {
      const Scalar w = r.data[3];
      for (int i = 0; i < 3; i++) {
        r.data[i] = div(r.data[i], w);
      }
    }
    r.type = T::Vector;
    return r;
  }
  case B::ArrayFn:
  case B::ArrayFloat:
  case B::ArrayInt:
  case B::ArrayVec:
  case B::ArrayMat: {
    WrValue r;
    r.type = T::FloatArray;
    r.array = a;
    return r;
  }
  case B::Len:
    return wr_scalar(float(arg(0).array.size()), T::Int);
  default:
    throw std::runtime_error("Cycles Wrangle: unsupported function opcode " +
                             std::to_string(int(fn)));
  }
}

void wrangle(bNode &n) {
  if (wr_results_.contains(&n)) {
    return;
  }
  const auto &storage = *static_cast<const NodeShaderWrangle *>(n.storage);
  const char *code =
      storage.code_run && storage.code_run[0] ? storage.code_run : storage.code;
  const auto compiled = vex::compile_shader_material(code ? code : "");
  if (!compiled.program) {
    throw std::runtime_error("Cycles Wrangle: " + compiled.error);
  }
  const vex::Program &p = *compiled.program;
  std::unordered_map<std::string, WrValue> channels, fields;
  for (const NodeShaderWrangleItem &item :
       Span(storage.input_items.items, storage.input_items.items_num)) {
    for (bNodeSocket &s : n.inputs) {
      if (item.name && StringRef(s.name) == item.name) {
        WrValue v = wr_socket(s);
        channels[item.name] = v;
        if (item.kind == SH_NODE_WRANGLE_ITEM_FIELD) {
          fields[item.name] = v;
        }
      }
    }
  }
  struct Frame {
    int pc;
    std::vector<WrValue> locals;
  };
  struct State {
    int pc = 0;
    int steps = 0;
    Scalar mask = 1.0f;
    std::vector<WrValue> stack, locals, attrs;
    std::vector<Frame> frames;
  };
  State first;
  first.locals.resize(p.local_count);
  first.attrs.resize(p.attrs.size());
  for (int i = 0; i < p.attrs.size(); i++) {
    const auto &a = p.attrs[i];
    WrValue v;
    v.type = a.type;
    if (auto it = fields.find(a.name); it != fields.end()) {
      v = it->second;
    } else if (a.read) {
      if (a.name == "position" || a.name == "normal") {
        bNode &geo = node("ShaderNodeNewGeometry"_ustr);
        v = wr_socket(output(geo, a.name == "position" ? 0 : 1));
      } else {
        bNode &attr = node("ShaderNodeAttribute"_ustr);
        auto &s = *static_cast<NodeShaderAttribute *>(attr.storage);
        STRNCPY(s.name, a.name.c_str());
        if (a.type == vex::Type::Matrix) {
          v.type = a.type;
          v.data = matrix(output(attr, 4));
        } else if (vex::type_has_xyzw(a.type)) {
          v = wr_socket(output(attr, 1));
          v.type = a.type;
          if (wr_size(v) == 4) {
            v.data[3] = &output(attr, 3);
          }
        } else {
          v = wr_scalar(&output(attr, 2), a.type);
        }
      }
    }
    first.attrs[i] = v;
  }
  std::vector<State> pending;
  pending.push_back(std::move(first));
  std::vector<WrValue> result(p.attrs.size());
  int paths = 0, total_steps = 0;
  while (!pending.empty()) {
    State s = std::move(pending.back());
    pending.pop_back();
    if (++paths > 256) {
      throw std::runtime_error("Cycles Wrangle: too many dynamic branches");
    }
    auto pop = [&]() -> WrValue {
      if (s.stack.empty()) {
        throw std::runtime_error("Cycles Wrangle: invalid stack");
      }
      WrValue v = std::move(s.stack.back());
      s.stack.pop_back();
      return v;
    };
    bool finished = false;
    while (s.pc < p.code.size() && !finished) {
      if (++total_steps > 100000 || ++s.steps > 16384) {
        throw std::runtime_error(
            "Cycles Wrangle: loop expansion limit exceeded");
      }
      const auto in = p.code[s.pc++];
      using O = vex::Op;
      using B = vex::Builtin;
      auto binary = [&](int operation) {
        WrValue b = pop(), a = pop();
        s.stack.push_back(wr_binary(operation, a, b));
      };
      auto builtin = [&](B fn, int count) {
        std::vector<WrValue> a(count);
        for (int i = count - 1; i >= 0; i--) {
          a[i] = pop();
        }
        s.stack.push_back(wr_builtin(fn, a, channels));
      };
      switch (in.op) {
      case O::Nop:
        break;
      case O::PushF:
        s.stack.push_back(wr_scalar(p.const_f[in.imm]));
        break;
      case O::PushI:
        s.stack.push_back(wr_scalar(float(p.const_i[in.imm]), vex::Type::Int));
        break;
      case O::PushB:
        s.stack.push_back(wr_scalar(float(in.imm != 0), vex::Type::Bool));
        break;
      case O::PushS: {
        WrValue v;
        v.type = vex::Type::String;
        v.text = p.const_s[in.imm];
        s.stack.push_back(v);
        break;
      }
      case O::PushV: {
        WrValue v;
        v.type = vex::Type::Vector;
        for (int i = 0; i < 3; i++) {
          v.data[i] = p.const_v[in.imm][i];
        }
        s.stack.push_back(v);
        break;
      }
      case O::PushLocal:
        s.stack.push_back(s.locals.at(in.imm));
        break;
      case O::PushLocalIndex: {
        const auto &load = p.local_index_loads[in.imm];
        Scalar index = load.index_is_const
                           ? Scalar(float(p.const_i[load.index]))
                           : s.locals.at(load.index).data[0];
        WrValue v = wr_index(s.locals.at(load.array_local), index);
        if (load.member >= 0) {
          v = wr_scalar(wr_component(v, load.member));
        }
        s.stack.push_back(v);
        break;
      }
      case O::StoreLocal:
        s.locals.at(in.imm) = pop();
        break;
      case O::PushAttr:
        s.stack.push_back(s.attrs.at(in.imm));
        break;
      case O::StoreAttr:
        s.attrs.at(in.imm) = pop();
        break;
      case O::AttrAdd:
        s.attrs.at(in.imm) =
            wr_binary(NODE_MATH_ADD, s.attrs.at(in.imm), pop());
        break;
      case O::AttrMulAdd: {
        WrValue count = pop(), value = pop();
        s.attrs.at(in.imm) =
            wr_binary(NODE_MATH_ADD, s.attrs.at(in.imm),
                      wr_binary(NODE_MATH_MULTIPLY, value, count));
        break;
      }
      case O::IncLocal:
      case O::DecLocal:
        s.locals.at(in.imm).data[0] = add(s.locals.at(in.imm).data[0],
                                          in.op == O::IncLocal ? 1.0f : -1.0f);
        break;
      case O::Dup:
        s.stack.push_back(s.stack.back());
        break;
      case O::Pop:
        pop();
        break;
      case O::Add:
      case O::AddFF:
      case O::AddVV:
        binary(NODE_MATH_ADD);
        break;
      case O::Sub:
      case O::SubFF:
      case O::SubVV:
        binary(NODE_MATH_SUBTRACT);
        break;
      case O::Mul:
      case O::MulFF:
      case O::MulFV:
      case O::MulVF:
      case O::MulVV:
        binary(NODE_MATH_MULTIPLY);
        break;
      case O::Div:
      case O::DivFF:
      case O::DivVF:
      case O::DivVV:
        binary(NODE_MATH_DIVIDE);
        break;
      case O::Mod:
        binary(NODE_MATH_MODULO);
        break;
      case O::Pow:
        binary(NODE_MATH_POWER);
        break;
      case O::Neg:
        s.stack.push_back(
            wr_binary(NODE_MATH_MULTIPLY, pop(), wr_scalar(-1.0f)));
        break;
      case O::Lt:
        binary(NODE_MATH_LESS_THAN);
        break;
      case O::Gt:
        binary(NODE_MATH_GREATER_THAN);
        break;
      case O::Eq:
        binary(NODE_MATH_COMPARE);
        break;
      case O::Le:
      case O::Ge:
      case O::Ne: {
        WrValue b = pop(), a = pop();
        int operation = in.op == O::Le   ? NODE_MATH_GREATER_THAN
                        : in.op == O::Ge ? NODE_MATH_LESS_THAN
                                         : NODE_MATH_COMPARE;
        s.stack.push_back(wr_scalar(
            sub(1.0f, op(operation, a.data[0], b.data[0])), vex::Type::Bool));
        break;
      }
      case O::And:
        binary(NODE_MATH_MULTIPLY);
        break;
      case O::Or:
        binary(NODE_MATH_MAXIMUM);
        break;
      case O::Not:
        s.stack.push_back(wr_scalar(op(NODE_MATH_COMPARE, pop().data[0], 0.0f),
                                    vex::Type::Bool));
        break;
      case O::Jmp:
        s.pc = in.imm;
        break;
      case O::JmpIfFalse:
      case O::JmpIfTrue: {
        Scalar test = op(NODE_MATH_GREATER_THAN,
                         op(NODE_MATH_ABSOLUTE, pop().data[0]), 0.0f);
        if (in.op == O::JmpIfFalse) {
          test = sub(1.0f, test);
        }
        if (const float *c = std::get_if<float>(&test)) {
          if (*c != 0) {
            s.pc = in.imm;
          }
        } else {
          State other = s;
          other.pc = in.imm;
          other.mask = mul(s.mask, test);
          pending.push_back(std::move(other));
          s.mask = mul(s.mask, sub(1.0f, test));
        }
        break;
      }
      case O::GetMember:
      case O::PushLocalMember: {
        WrValue a = in.op == O::GetMember ? pop() : s.locals.at(in.imm), r;
        int sw = in.op == O::GetMember ? in.imm : int(in.a);
        int count = vex::swizzle_count(sw);
        r.type = vex::swizzle_result_type(count);
        for (int i = 0; i < count; i++) {
          r.data[i] = wr_component(a, vex::swizzle_axis(sw, i));
        }
        s.stack.push_back(r);
        break;
      }
      case O::GetIndex: {
        WrValue index = pop(), a = pop();
        s.stack.push_back(wr_index(a, index.data[0]));
        break;
      }
      case O::SetMember: {
        WrValue a = pop(), value = pop();
        int count = vex::swizzle_count(in.imm);
        for (int i = 0; i < count; i++) {
          a.data[vex::swizzle_axis(in.imm, i)] = wr_component(value, i);
        }
        s.stack.push_back(a);
        break;
      }
      case O::SetIndex: {
        WrValue index = pop(), a = pop(), value = pop();
        int size = vex::type_is_matrix(a.type) ? 4 : wr_size(a);
        for (int i = 0; i < size; i++) {
          Scalar test = equal_index(index.data[0], i);
          if (vex::type_is_matrix(a.type)) {
            for (int row = 0; row < 4; row++) {
              a.data[i * 4 + row] =
                  select(test, value.data[row], a.data[i * 4 + row]);
            }
          } else {
            a.data[i] = select(test, value.data[0], a.data[i]);
          }
        }
        s.stack.push_back(a);
        break;
      }
      case O::MakeVec: {
        std::vector<WrValue> a(in.imm);
        for (int i = in.imm - 1; i >= 0; i--) {
          a[i] = pop();
        }
        s.stack.push_back(wr_builtin(B::Set, a, channels));
        break;
      }
      case O::LengthV3:
        builtin(B::Length, 1);
        break;
      case O::DotV3:
        builtin(B::Dot, 2);
        break;
      case O::CrossV3:
        builtin(B::Cross, 2);
        break;
      case O::SqrtF:
        builtin(B::Sqrt, 1);
        break;
      case O::LogF:
        builtin(B::Log, 1);
        break;
      case O::AbsF:
        builtin(B::Abs, 1);
        break;
      case O::ArrayLen:
        builtin(B::Len, 1);
        break;
      case O::Call:
        builtin(B(in.imm), int(in.a));
        break;
      case O::CallUser: {
        const auto &fn = p.user_fns[in.imm];
        std::vector<WrValue> args(in.a);
        for (int i = int(in.a) - 1; i >= 0; i--) {
          args[i] = pop();
        }
        if (s.frames.size() >= 32) {
          throw std::runtime_error(
              "Cycles Wrangle: function recursion limit exceeded");
        }
        s.frames.push_back({s.pc, std::move(s.locals)});
        s.locals.resize(std::max(fn.local_count, int(in.a)));
        for (int i = 0; i < int(in.a); i++) {
          s.locals[i] = args[i];
        }
        s.pc = fn.entry;
        break;
      }
      case O::Return:
        if (s.frames.empty()) {
          finished = true;
        } else {
          Frame f = std::move(s.frames.back());
          s.frames.pop_back();
          s.pc = f.pc;
          s.locals = std::move(f.locals);
        }
        break;
      default:
        throw std::runtime_error("Cycles Wrangle: unsupported instruction " +
                                 std::to_string(int(in.op)));
      }
    }
    for (int i = 0; i < p.attrs.size(); i++) {
      result[i] = wr_select(s.mask, s.attrs[i], result[i]);
    }
  }
  auto &cache = wr_results_[&n];
  for (int i = 0; i < p.attrs.size(); i++) {
    cache[p.attrs[i].name] = result[i];
  }
  for (bNodeSocket &s : n.outputs) {
    auto it = cache.find(s.name);
    WrValue v = it == cache.end() ? wr_scalar(0.0f) : it->second;
    if (s.type == SOCK_MATRIX) {
      matrices_[&s] = v.data;
    } else if (s.type == SOCK_VECTOR) {
      vectors_[&s] = {wr_component(v, 0), wr_component(v, 1),
                      wr_component(v, 2)};
    } else if (s.type == SOCK_FLOAT || s.type == SOCK_BOOLEAN ||
               s.type == SOCK_INT) {
      scalars_[&s] = v.data[0];
    }
  }
}
