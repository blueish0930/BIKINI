/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */
/* Shared VM/JIT operation semantics. No control flow is duplicated. */
case Op::Nop:
break;
case Op::PushF:
push(Value::from_float(program.const_f[in.imm]));
break;
case Op::PushI:
push(Value::from_int(program.const_i[in.imm]));
break;
case Op::PushV:
push(Value::from_vec(program.const_v[in.imm]));
break;
case Op::PushB:
push(Value::from_bool(in.imm != 0));
break;
case Op::PushS:
push(Value::from_str_i(in.imm));
break;
case Op::PushLocal:
push((in.imm >= 0 && in.imm < local_n) ? locals_s[in.imm]
                                       : Value::from_float(0.0f));
break;
case Op::PushLocalIndex:
push(load_local_index(program, in, locals_s, local_n, &local));
break;
case Op::PushLocalMember:
if (in.imm >= 0 && in.imm < local_n) {
  push(apply_get_member(locals_s[in.imm], int(in.a)));
} else {
  push(Value::from_float(0.0f));
}
break;
case Op::StoreLocal: {
  Value v = pop();
  if (in.imm >= 0 && in.imm < local_n) {
    locals_s[in.imm] = v;
  }
  break;
}
case Op::IncLocal:
if (in.imm >= 0 && in.imm < local_n) {
  locals_s[in.imm].type = Type::Int;
  locals_s[in.imm].i += 1;
  locals_s[in.imm].v.x = float(locals_s[in.imm].i);
}
break;
case Op::DecLocal:
if (in.imm >= 0 && in.imm < local_n) {
  locals_s[in.imm].type = Type::Int;
  locals_s[in.imm].i -= 1;
  locals_s[in.imm].v.x = float(locals_s[in.imm].i);
}
break;
case Op::Dup:
if (sp > 0 && sp < stack_cap) {
  stack[sp] = stack[sp - 1];
  sp++;
}
break;
case Op::Pop:
if (sp > 0) {
  sp--;
}
break;
case Op::PushAttr:
if (in.imm >= 0 && in.imm < local.attrs.size()) {
  push(load_attr_value(local.attrs[in.imm], local.index, local));
} else {
  push(Value::from_float(0.0f));
}
break;
case Op::StoreAttr: {
  Value v = pop();
  if (in.imm >= 0 && in.imm < local.attrs.size()) {
    store_attr_value(local.attrs[in.imm], local.index, v, local);
  }
  break;
}
case Op::AttrAdd: {
  Value v = pop();
  if (in.imm >= 0 && in.imm < local.attrs.size()) {
    attr_add_inplace(local.attrs[in.imm], local.index, v);
  }
  break;
}
case Op::AttrMulAdd: {
  Value k = pop();
  Value b = pop();
  if (in.imm >= 0 && in.imm < local.attrs.size()) {
    attr_add_inplace(local.attrs[in.imm], local.index,
                     scale_value(b, k.as_int()));
  }
  break;
}
case Op::PushIndex:
push(Value::from_int(local.index));
break;
case Op::PushNpoints:
push(Value::from_int(local.npoints));
break;
case Op::PushNedges:
push(Value::from_int(local.nedges));
break;
case Op::PushNfaces:
push(Value::from_int(local.nfaces));
break;
case Op::PushNcorners:
push(Value::from_int(local.ncorners));
break;
case Op::PushHitPos:
push(Value::from_vec(local.hit_pos));
break;
case Op::Add:
if (sp >= 2) {
  Value b = pop();
  add_inplace(stack[sp - 1], b);
}
break;
case Op::Sub: {
  Value b = pop();
  Value a = pop();
  push(numeric_sub(a, b));
  break;
}
case Op::Mul: {
  Value b = pop();
  Value a = pop();
  push(numeric_mul(a, b, nullptr));
  break;
}
case Op::Div: {
  Value b = pop();
  Value a = pop();
  push(numeric_div(a, b));
  break;
}
case Op::AddFF: {
  const Value b = pop(), a = pop();
  push(Value::from_float(a.v.x + b.v.x));
  break;
}
case Op::AddVV: {
  const Value b = pop(), a = pop();
  push(Value::from_vec(float3(a.v.x + b.v.x, a.v.y + b.v.y, a.v.z + b.v.z)));
  break;
}
case Op::SubFF: {
  const Value b = pop(), a = pop();
  push(Value::from_float(a.v.x - b.v.x));
  break;
}
case Op::SubVV: {
  const Value b = pop(), a = pop();
  push(Value::from_vec(float3(a.v.x - b.v.x, a.v.y - b.v.y, a.v.z - b.v.z)));
  break;
}
case Op::MulFF: {
  const Value b = pop(), a = pop();
  push(Value::from_float(a.v.x * b.v.x));
  break;
}
case Op::MulFV: {
  const Value b = pop(), a = pop();
  push(Value::from_vec(float3(a.v.x * b.v.x, a.v.x * b.v.y, a.v.x * b.v.z)));
  break;
}
case Op::MulVF: {
  const Value b = pop(), a = pop();
  push(Value::from_vec(float3(a.v.x * b.v.x, a.v.y * b.v.x, a.v.z * b.v.x)));
  break;
}
case Op::MulVV: {
  const Value b = pop(), a = pop();
  push(Value::from_vec(float3(a.v.x * b.v.x, a.v.y * b.v.y, a.v.z * b.v.z)));
  break;
}
case Op::DivFF: {
  const Value b = pop(), a = pop();
  push(Value::from_float(b.v.x != 0.0f ? a.v.x / b.v.x : 0.0f));
  break;
}
case Op::DivVF: {
  const Value b = pop(), a = pop();
  const float inv = b.v.x != 0.0f ? 1.0f / b.v.x : 0.0f;
  push(Value::from_vec(float3(a.v.x * inv, a.v.y * inv, a.v.z * inv)));
  break;
}
case Op::DivVV: {
  const Value b = pop(), a = pop();
  push(Value::from_vec(float3(a.v.x / b.v.x, a.v.y / b.v.y, a.v.z / b.v.z)));
  break;
}
case Op::Mod: {
  Value b = pop();
  Value a = pop();
  if (a.type == Type::Int && b.type == Type::Int) {
    const int d = b.as_int();
    push(Value::from_int(d != 0 ? a.as_int() % d : 0));
  } else {
    const float d = b.as_float();
    push(Value::from_float(d != 0.0f ? fmodf(a.as_float(), d) : 0.0f));
  }
  break;
}
case Op::Neg: {
  Value a = pop();
  if (a.type == Type::Vector4) {
    push(Value::from_vec4(-a.as_vec4(), Type::Vector4));
  } else if (a.type == Type::Vector) {
    push(Value::from_vec(-a.as_vec()));
  } else if (a.type == Type::Vector2) {
    push(Value::from_vec2(-a.as_vec2()));
  } else if (a.type == Type::Matrix) {
    push(Value::from_matrix(-value_as_matrix(a)));
  } else if (a.type == Type::Matrix3) {
    push(Value::from_matrix3(-value_as_matrix3(a)));
  } else if (a.type == Type::Matrix2) {
    push(Value::from_matrix2(-a.as_matrix2()));
  } else if (a.type == Type::Int) {
    push(Value::from_int(-a.as_int()));
  } else {
    push(Value::from_float(-a.as_float()));
  }
  break;
}
case Op::Lt: {
  Value b = pop();
  Value a = pop();
  push(Value::from_bool(a.as_float() < b.as_float()));
  break;
}
case Op::Le: {
  Value b = pop();
  Value a = pop();
  push(Value::from_bool(a.as_float() <= b.as_float()));
  break;
}
case Op::Gt: {
  Value b = pop();
  Value a = pop();
  push(Value::from_bool(a.as_float() > b.as_float()));
  break;
}
case Op::Ge: {
  Value b = pop();
  Value a = pop();
  push(Value::from_bool(a.as_float() >= b.as_float()));
  break;
}
case Op::Eq: {
  Value b = pop();
  Value a = pop();
  push(Value::from_bool(a.as_float() == b.as_float()));
  break;
}
case Op::Ne: {
  Value b = pop();
  Value a = pop();
  push(Value::from_bool(a.as_float() != b.as_float()));
  break;
}
case Op::And: {
  Value b = pop();
  Value a = pop();
  push(Value::from_bool(a.as_bool() && b.as_bool()));
  break;
}
case Op::Or: {
  Value b = pop();
  Value a = pop();
  push(Value::from_bool(a.as_bool() || b.as_bool()));
  break;
}
case Op::Not:
push(Value::from_bool(!pop().as_bool()));
break;
#ifndef VEX_JIT_OPERATION
case Op::Jmp:
if (++jumps > 100000000) {
  err = "Instruction limit exceeded (infinite loop?)";
  lane_ok = false;
  break;
}
pc = in.imm;
continue;
case Op::JmpIfFalse:
if (++jumps > 100000000) {
  err = "Instruction limit exceeded (infinite loop?)";
  lane_ok = false;
  break;
}
if (!pop().as_bool()) {
  pc = in.imm;
  continue;
}
break;
case Op::JmpIfTrue:
if (++jumps > 100000000) {
  err = "Instruction limit exceeded (infinite loop?)";
  lane_ok = false;
  break;
}
if (pop().as_bool()) {
  pc = in.imm;
  continue;
}
break;
#endif
case Op::LengthV3:
push(typed_length_v3(pop()));
break;
case Op::DotV3: {
  const Value b = pop();
  const Value a = pop();
  push(typed_dot_v3(a, b));
  break;
}
case Op::CrossV3: {
  const Value b = pop();
  const Value a = pop();
  push(typed_cross_v3(a, b));
  break;
}
case Op::SqrtF:
push(typed_sqrt_f(pop()));
break;
case Op::LogF:
push(typed_log_f(pop()));
break;
case Op::AbsF:
push(typed_abs_f(pop()));
break;
case Op::ArrayLen:
push(typed_array_len(pop()));
break;
case Op::Call: {
  const int na = std::min(int(in.a), 32);
  Value result;
  if (sp >= na) {
    /* The arguments are the top na stack values, in order: no copy. */
    sp -= na;
    result =
        call_builtin(Builtin(in.imm), Span<Value>(stack + sp, na), local, err);
  } else {
    Value args_buf[32];
    for (int a = na - 1; a >= 0; a--) {
      args_buf[a] = pop();
    }
    result =
        call_builtin(Builtin(in.imm), Span<Value>(args_buf, na), local, err);
  }
  if (!err.empty()) {
    lane_ok = false;
    break;
  }
  push(result);
  break;
}
case Op::SampleElem: {
  const int na = std::min(int(in.a & 0x1f), 32);
  Value result;
  if (sp >= na) {
    sp -= na;
    result = call_static_sample(in, Span<Value>(stack + sp, na), local, err);
  } else {
    Value args_buf[32];
    for (int a = na - 1; a >= 0; a--) {
      args_buf[a] = pop();
    }
    result = call_static_sample(in, Span<Value>(args_buf, na), local, err);
  }
  push(result);
  if (!err.empty()) {
    lane_ok = false;
  }
  break;
}
case Op::FaceCorners: {
  const int nargs = int(in.a);
  Value args_buf[2];
  const int na = std::min(nargs, 2);
  for (int a = na - 1; a >= 0; a--) {
    args_buf[a] = pop();
  }
  push(face_corners_direct(in, Span<Value>(args_buf, na), local));
  break;
}
case Op::GatherSamples:
if (!gather_samples_direct(program, in, locals_s, local_n, local, err)) {
  lane_ok = false;
}
break;
case Op::CallUser: {
  if (in.imm < 0 || in.imm >= int(program.user_fns.size())) {
    lane_ok = false;
    err = "Invalid function call";
    break;
  }
  const UserFn &fn = program.user_fns[in.imm];
  const int nargs = int(in.a);
  const int fn_n = std::max(fn.local_count, std::max(nargs, 1));
  Vector<Value> fn_locals(fn_n);
  for (int i = nargs - 1; i >= 0; i--) {
    fn_locals[i] = pop();
  }
  Value ret = Value::from_int(0);
  if (!vm_interp(program, local, err, &ret, fn.entry, fn_locals.data(), fn_n,
                 1)) {
    lane_ok = false;
    break;
  }
  push(ret);
  break;
}
case Op::Pow: {
  const Value b = pop();
  const Value a = pop();
  push(Value::from_float(powf(a.as_float(), b.as_float())));
  break;
}
case Op::GetMember: {
  push(apply_get_member(pop(), in.imm));
  break;
}
case Op::SetMember: {
  const Value obj = pop();
  const Value mem = pop();
  push(apply_set_member(obj, mem, in.imm));
  break;
}
case Op::GetIndex: {
  const Value ix = pop();
  const Value arr = pop();
  push(get_index_value(arr, ix.as_int()));
  break;
}
case Op::SetIndex: {
  const Value ix = pop();
  const Value arr = pop();
  const Value elem = pop();
  push(set_index_value(arr, ix.as_int(), elem, local));
  break;
}
case Op::MakeVec: {
  const int cn = in.imm;
  float w = 0.0f;
  float z = 0.0f;
  float y = 0.0f;
  float x = 0.0f;
  if (cn >= 4) {
    w = pop().as_float();
  }
  if (cn >= 3) {
    z = pop().as_float();
  }
  if (cn >= 2) {
    y = pop().as_float();
  }
  if (cn >= 1) {
    x = pop().as_float();
  }
  push(make_vec_value(cn, x, y, z, w));
  break;
}
case Op::WhileCmpAdd:
if (in.imm >= 0 && in.imm < program.while_adds.size() &&
    program.while_adds[in.imm].attr >= 0 &&
    program.while_adds[in.imm].attr < local.attrs.size()) {
  while_cmp_add_one(local.attrs, program.while_adds[in.imm], local.index);
}
break;
#ifndef VEX_JIT_OPERATION
case Op::Return:
pc = code_n;
continue;
#endif
