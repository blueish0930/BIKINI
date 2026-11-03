/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "vex_jit.hh"

#include "BKE_wrangle_array.hh"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <llvm/ExecutionEngine/Orc/LLJIT.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Passes/PassBuilder.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h>

#if defined(_WIN32) && defined(_M_X64)
extern "C" void __chkstk();
#endif

namespace blender::nodes::vex {
namespace {

/* Reject inconsistent stacks before binding or writing any output. */
static bool stack_effect(const Inst &in, int &pops, int &pushes) {
  pops = pushes = 0;
  switch (in.op) {
  case Op::Nop:
  case Op::IncLocal:
  case Op::DecLocal:
  case Op::Jmp:
  case Op::GatherSamples:
  case Op::WhileCmpAdd:
  case Op::Return:
    return true;
  case Op::PushF:
  case Op::PushI:
  case Op::PushV:
  case Op::PushB:
  case Op::PushS:
  case Op::PushLocal:
  case Op::PushLocalIndex:
  case Op::PushLocalMember:
  case Op::PushAttr:
  case Op::PushIndex:
  case Op::PushNpoints:
  case Op::PushNedges:
  case Op::PushNfaces:
  case Op::PushNcorners:
  case Op::PushHitPos:
    pushes = 1;
    return true;
  case Op::StoreLocal:
  case Op::StoreAttr:
  case Op::AttrAdd:
  case Op::Pop:
  case Op::JmpIfFalse:
  case Op::JmpIfTrue:
    pops = 1;
    return true;
  case Op::AttrMulAdd:
    pops = 2;
    return true;
  case Op::Dup:
    pops = 1;
    pushes = 2;
    return true;
  case Op::Neg:
  case Op::Not:
  case Op::LengthV3:
  case Op::SqrtF:
  case Op::LogF:
  case Op::AbsF:
  case Op::ArrayLen:
  case Op::GetMember:
    pops = pushes = 1;
    return true;
  case Op::Add:
  case Op::Sub:
  case Op::Mul:
  case Op::Div:
  case Op::AddFF:
  case Op::AddVV:
  case Op::SubFF:
  case Op::SubVV:
  case Op::MulFF:
  case Op::MulFV:
  case Op::MulVF:
  case Op::MulVV:
  case Op::DivFF:
  case Op::DivVF:
  case Op::DivVV:
  case Op::Mod:
  case Op::Pow:
  case Op::Lt:
  case Op::Le:
  case Op::Gt:
  case Op::Ge:
  case Op::Eq:
  case Op::Ne:
  case Op::And:
  case Op::Or:
  case Op::DotV3:
  case Op::CrossV3:
  case Op::SetMember:
  case Op::GetIndex:
    pops = 2;
    pushes = 1;
    return true;
  case Op::SetIndex:
    pops = 3;
    pushes = 1;
    return true;
  case Op::MakeVec:
    if (in.imm < 1 || in.imm > 4)
      return false;
    pops = in.imm;
    pushes = 1;
    return true;
  case Op::Call:
    pops = std::min(int(in.a), 32);
    pushes = 1;
    return true;
  case Op::CallUser:
    pops = int(in.a);
    pushes = 1;
    return true;
  case Op::SampleElem:
    pops = int(in.a & 0x1f);
    pushes = 1;
    return true;
  case Op::FaceCorners:
    pops = std::min(int(in.a), 2);
    pushes = 1;
    return true;
  default:
    return false;
  }
}

static bool analyze(const Program &program, std::vector<int> &depth,
                    int &capacity) {
  const int size = int(program.code.size());
  depth.assign(size + 1, -1);
  depth[0] = 0;
  std::deque<int> todo{0};
  capacity = 1;
  auto edge = [&](int pc, int value) {
    if (pc < 0 || pc > size)
      return false;
    if (depth[pc] >= 0)
      return depth[pc] == value;
    depth[pc] = value;
    todo.push_back(pc);
    return true;
  };
  while (!todo.empty()) {
    const int pc = todo.front();
    todo.pop_front();
    if (pc == size)
      continue;
    const Inst &in = program.code[pc];
    int pops, pushes;
    if (!stack_effect(in, pops, pushes) || depth[pc] < pops)
      return false;
    const int after = depth[pc] - pops + pushes;
    capacity = std::max(capacity, after + 1);
    if (in.op == Op::Return)
      continue;
    if (ELEM(in.op, Op::Jmp, Op::JmpIfFalse, Op::JmpIfTrue)) {
      if (!edge(in.imm, after))
        return false;
      if (in.op == Op::Jmp)
        continue;
    }
    if (!edge(pc + 1, after))
      return false;
  }
  return true;
}

/* BLENDER_VEX_JIT_COUNT=1: count executed helper calls (op, or 256 + builtin
 * for Call; 254 array view resolves, 255 sample fallbacks), printed at exit. */
static std::atomic<uint64_t> helper_counts[1024];
static bool count_helpers()
{
  static const bool enabled = std::getenv("BLENDER_VEX_JIT_COUNT") != nullptr;
  return enabled;
}
static void count_helper(const int key)
{
  helper_counts[key].fetch_add(1, std::memory_order_relaxed);
}
struct HelperCountReport {
  ~HelperCountReport() {
    for (int key = 0; key < 1024; key++)
      if (const uint64_t n = helper_counts[key].load())
        std::fprintf(stderr, "VEX_JIT_COUNT %s=%d calls=%llu\n",
                     key >= 256 ? "builtin" : "op", key >= 256 ? key - 256 : key,
                     static_cast<unsigned long long>(n));
  }
};
static HelperCountReport helper_count_report;

struct ModuleLease {
  llvm::orc::ResourceTrackerSP tracker;
  ~ModuleLease() {
    if (tracker)
      llvm::consumeError(tracker->remove());
  }
};

struct Backend {
  struct CacheEntry {
    std::shared_ptr<const JitKernel> kernel;
    uint64_t last_used = 0;
    std::string parameter_group;
    bool parameter_specialized = false;
  };
  std::mutex mutex;
  std::unique_ptr<llvm::orc::LLJIT> engine;
  std::unordered_map<std::string, CacheEntry> cache;
  uint64_t serial = 0;
  uint64_t use_tick = 0;
  Backend() {
    llvm::InitializeNativeTarget();
    llvm::InitializeNativeTargetAsmPrinter();
    auto created = llvm::orc::LLJITBuilder().create();
    if (created)
      engine = std::move(*created);
    else
      llvm::consumeError(created.takeError());
    if (engine) {
      /* Native kernels may spill private values or use memory intrinsics. These
       * CRT symbols are not necessarily exported by the Blender executable. */
      llvm::orc::SymbolMap symbols;
      auto add = [&](const char *name, auto address) {
        symbols[engine->mangleAndIntern(name)] = llvm::orc::ExecutorSymbolDef(
            llvm::orc::ExecutorAddr::fromPtr(address),
            llvm::JITSymbolFlags::Exported);
      };
      add("memcpy", &std::memcpy);
      add("memmove", &std::memmove);
      add("memset", &std::memset);
#if defined(_WIN32) && defined(_M_X64)
      add("__chkstk", &__chkstk);
#endif
      llvm::consumeError(engine->getMainJITDylib().define(
          llvm::orc::absoluteSymbols(std::move(symbols))));
    }
  }
};

static std::string cache_key(const Program &p) {
  /* Full structural key, not just a hash: collisions cannot reuse another
   * program. Helpers receive the current Program, so geometry, strings and
   * runtime bindings are never captured in generated code. */
  std::string key;
  key.push_back(std::getenv("BLENDER_VEX_JIT_LEGACY_STORAGE") ? 0 : 1);
  key.push_back(std::getenv("BLENDER_VEX_JIT_DISABLE_ARRAY_VIEWS") ? 0 : 1);
  auto append = [&](const auto &v) {
    key.append(reinterpret_cast<const char *>(&v), sizeof(v));
  };
  const int size = int(p.code.size());
  append(size);
  append(p.local_count);
  const int attributes = int(p.attrs.size());
  append(attributes);
  for (const auto &attribute : p.attrs)
    append(attribute.type);
  for (const Inst &in : p.code) {
    append(in.op);
    append(in.a);
    append(in.imm);
  }
  const int nf = int(p.const_f.size()), ni = int(p.const_i.size()),
            nv = int(p.const_v.size());
  append(nf);
  append(ni);
  append(nv);
  for (const float v : p.const_f)
    append(v);
  for (const int v : p.const_i)
    append(v);
  for (const float3 v : p.const_v) {
    append(v.x);
    append(v.y);
    append(v.z);
  }
  const int nl = int(p.local_index_loads.size());
  append(nl);
  for (const auto &load : p.local_index_loads) {
    append(load.array_local);
    append(load.index);
    append(load.index_is_const);
    append(load.member);
    append(load.sample_slot);
    append(load.sample_index_array_local);
  }
  const int ng = int(p.gather_samples.size());
  append(ng);
  for (const auto &gather : p.gather_samples) {
    append(gather.index_array_local);
    const int outputs = int(gather.output_locals.size());
    append(outputs);
    for (const int output : gather.output_locals)
      append(output);
  }
  return key;
}

class Lowering {
  const Program &program;
  llvm::LLVMContext &context;
  llvm::Module &module;
  llvm::IRBuilder<> b;
  llvm::Function *function;
  llvm::Value *frame, *stack, *locals, *bridge_stack, *bridge_locals;
  llvm::Value *branch_counter;
  bool private_storage;
  Span<JitUniformParm> uniform_parms;
  bool specialize_parms;
  std::vector<llvm::Value *> uniform_slots;
  llvm::Type *ptr, *f32, *i32;
  std::vector<llvm::BasicBlock *> blocks;
  llvm::BasicBlock *failed;
  const std::vector<int> &depth;
  const bool native_pools = jit_array_pools_native();

  llvm::Value *offset(llvm::Value *base, size_t bytes) {
    return b.CreateConstGEP1_64(b.getInt8Ty(), base, bytes);
  }
  llvm::Value *load(llvm::Value *base, size_t bytes, llvm::Type *type) {
    return b.CreateLoad(type, offset(base, bytes));
  }
  llvm::Value *slot(llvm::Value *base, int index) {
    return offset(base, sizeof(Value) * size_t(index));
  }
  void copy(llvm::Value *dst, llvm::Value *src) {
    /* Copy semantic fields rather than opaque bytes/padding. Private values
     * must never escape through memcpy, otherwise SROA retains stack objects.
     */
    auto *tag = load(src, offsetof(Value, type), b.getInt8Ty());
    auto *integer = load(src, offsetof(Value, i), i32);
    auto *vector = b.CreateAlignedLoad(llvm::FixedVectorType::get(f32, 4),
                                       offset(src, offsetof(Value, v)),
                                       llvm::Align(alignof(float)));
    b.CreateStore(tag, offset(dst, offsetof(Value, type)));
    b.CreateStore(integer, offset(dst, offsetof(Value, i)));
    b.CreateAlignedStore(vector, offset(dst, offsetof(Value, v)),
                         llvm::Align(alignof(float)));
  }
  llvm::Value *component(llvm::Value *value, int axis) {
    return load(value, offsetof(Value, v) + 4 * axis, f32);
  }
  void store_value(llvm::Value *dst, Type type, llvm::Value *x,
                   llvm::Value *y = nullptr, llvm::Value *z = nullptr,
                   llvm::Value *integer = nullptr, llvm::Value *w = nullptr) {
    b.CreateStore(b.getInt8(uint8_t(type)), offset(dst, offsetof(Value, type)));
    b.CreateStore(integer ? integer : b.getInt32(0),
                  offset(dst, offsetof(Value, i)));
    llvm::Value *vector =
        llvm::Constant::getNullValue(llvm::FixedVectorType::get(f32, 4));
    vector = b.CreateInsertElement(vector, x, b.getInt32(0));
    if (y)
      vector = b.CreateInsertElement(vector, y, b.getInt32(1));
    if (z)
      vector = b.CreateInsertElement(vector, z, b.getInt32(2));
    if (w)
      vector = b.CreateInsertElement(vector, w, b.getInt32(3));
    b.CreateAlignedStore(vector, offset(dst, offsetof(Value, v)),
                         llvm::Align(alignof(float)));
  }
  llvm::Value *truth(llvm::Value *value) {
    auto *type = load(value, offsetof(Value, type), b.getInt8Ty());
    auto *integer_type =
        b.CreateOr(b.CreateICmpEQ(type, b.getInt8(uint8_t(Type::Int))),
                   b.CreateICmpEQ(type, b.getInt8(uint8_t(Type::Bool))));
    return b.CreateSelect(
        integer_type,
        b.CreateICmpNE(load(value, offsetof(Value, i), i32), b.getInt32(0)),
        b.CreateFCmpUNE(component(value, 0), llvm::ConstantFP::get(f32, 0.0)));
  }
  llvm::Value *as_float(llvm::Value *value) {
    auto *type = load(value, offsetof(Value, type), b.getInt8Ty());
    auto *integer_type =
        b.CreateOr(b.CreateICmpEQ(type, b.getInt8(uint8_t(Type::Int))),
                   b.CreateICmpEQ(type, b.getInt8(uint8_t(Type::Bool))));
    return b.CreateSelect(
        integer_type, b.CreateSIToFP(load(value, offsetof(Value, i), i32), f32),
        component(value, 0));
  }
  llvm::Value *is_type(llvm::Value *tag, const Type type) {
    return b.CreateICmpEQ(tag, b.getInt8(uint8_t(type)));
  }
  /* `sqrtf(std::max(x, 0.0f))` of the VM: NaN and -0 pass through the clamp. */
  llvm::Value *sqrt_clamped(llvm::Value *x) {
    auto *zero = llvm::ConstantFP::get(f32, 0.0);
    return b.CreateUnaryIntrinsic(
        llvm::Intrinsic::sqrt, b.CreateSelect(b.CreateFCmpOLT(x, zero), zero, x));
  }
  llvm::Value *axis(llvm::Value *value, const int index) {
    auto *type = load(value, offsetof(Value, type), b.getInt8Ty());
    llvm::Value *vector_like = b.getFalse();
    for (const Type candidate : {Type::Vector2, Type::Vector, Type::Vector4,
                                 Type::Color, Type::Rotation, Type::Matrix2})
      vector_like = b.CreateOr(vector_like, is_type(type, candidate));
    return b.CreateSelect(vector_like, component(value, index),
                          as_float(value));
  }
  llvm::Value *as_vec_component(llvm::Value *value, const int index) {
    auto *type = load(value, offsetof(Value, type), b.getInt8Ty());
    auto *v2 = is_type(type, Type::Vector2);
    auto *vector_like =
        b.CreateOr(is_type(type, Type::Vector), is_type(type, Type::Vector4));
    vector_like = b.CreateOr(vector_like, is_type(type, Type::Color));
    vector_like = b.CreateOr(vector_like, is_type(type, Type::Rotation));
    auto *v =
        b.CreateSelect(vector_like, component(value, index), as_float(value));
    return b.CreateSelect(v2,
                          index == 2 ? llvm::ConstantFP::get(f32, 0)
                                     : component(value, index),
                          v);
  }
  llvm::Value *is_scalar(llvm::Value *tag) {
    return b.CreateOr(is_type(tag, Type::Float),
                      b.CreateOr(is_type(tag, Type::Int), is_type(tag, Type::Bool)));
  }
  /* Value::as_vec() axis of a scalar or 3D vector. */
  llvm::Value *scalar_or_vec(llvm::Value *value, llvm::Value *vector, int index) {
    return b.CreateSelect(vector, component(value, index), as_float(value));
  }
  void store_vec_or_float(llvm::Value *dst, llvm::Value *vector,
                          llvm::Value *const r[3]) {
    auto *zero = llvm::ConstantFP::get(f32, 0);
    store_value(dst, Type::Float, r[0], b.CreateSelect(vector, r[1], zero),
                b.CreateSelect(vector, r[2], zero));
    b.CreateStore(b.CreateSelect(vector, b.getInt8(uint8_t(Type::Vector)),
                                 b.getInt8(uint8_t(Type::Float))),
                  offset(dst, offsetof(Value, type)));
  }

  /* Generic + - * / of scalars and 3D vectors, exactly as add_inplace() and
   * numeric_sub/mul/div(); the int pair is handled before. Leaves the insert
   * point in the block for every other type. */
  void emit_numeric(const Inst &in, llvm::Value *a, llvm::Value *v,
                    llvm::BasicBlock *next) {
    auto *tag_a = load(a, offsetof(Value, type), b.getInt8Ty());
    auto *tag_b = load(v, offsetof(Value, type), b.getInt8Ty());
    auto *va = is_type(tag_a, Type::Vector), *vb = is_type(tag_b, Type::Vector);
    auto *native = llvm::BasicBlock::Create(context, "numeric", function);
    auto *other = llvm::BasicBlock::Create(context, "numeric_other", function);
    b.CreateCondBr(b.CreateAnd(b.CreateOr(is_scalar(tag_a), va),
                               b.CreateOr(is_scalar(tag_b), vb)),
                   native, other);
    b.SetInsertPoint(native);
    auto *zero = llvm::ConstantFP::get(f32, 0);
    auto *fb = as_float(v);
    auto *divisor = b.CreateFCmpUNE(fb, zero);
    llvm::Value *r[3];
    for (int axis = 0; axis < 3; axis++) {
      auto *x = scalar_or_vec(a, va, axis), *y = scalar_or_vec(v, vb, axis);
      switch (in.op) {
      case Op::Add:
        r[axis] = b.CreateFAdd(x, y);
        break;
      case Op::Sub:
        r[axis] = b.CreateFSub(x, y);
        break;
      case Op::Mul:
        /* A scalar times a vector is computed as vector * scalar. */
        r[axis] = b.CreateSelect(b.CreateAnd(vb, b.CreateNot(va)),
                                 b.CreateFMul(y, x), b.CreateFMul(x, y));
        break;
      default:
        /* Only a vector numerator gives a vector; a scalar divides by the
         * divisor's as_float(). Zero scalar divisors give zero. */
        r[axis] = b.CreateSelect(
            b.CreateAnd(va, vb), b.CreateFDiv(x, y),
            b.CreateSelect(divisor, b.CreateFDiv(x, fb), zero));
        break;
      }
    }
    auto *vector = in.op == Op::Div ? va : b.CreateOr(va, vb);
    if (in.op == Op::Add) {
      /* add_inplace keeps `i`, and a float result keeps v.yzw. */
      b.CreateStore(b.CreateSelect(vector, b.getInt8(uint8_t(Type::Vector)),
                                   b.getInt8(uint8_t(Type::Float))),
                    offset(a, offsetof(Value, type)));
      for (int axis = 0; axis < 4; axis++) {
        auto *old = component(a, axis);
        b.CreateStore(axis == 0   ? r[0]
                      : axis == 3 ? b.CreateSelect(vector, zero, old)
                                  : b.CreateSelect(vector, r[axis], old),
                      offset(a, offsetof(Value, v) + 4 * axis));
      }
    } else
      store_vec_or_float(a, vector, r);
    b.CreateBr(next);
    b.SetInsertPoint(other);
  }

  /* vector()/set() of three scalars or one scalar/vector, and normalize(),
   * as call_builtin(). Anything else uses the helper. */
  /* The #Vector of pool array `handle` (an IntArray, FloatArray or VecArray by `is_int` and
   * `is_float`): two loads for the pool range. Branches to `generic` outside the pool. */
  llvm::Value *pool_array(llvm::Value *is_int, llvm::Value *is_float, llvm::Value *handle,
                          llvm::BasicBlock *generic) {
    auto *i64 = b.getInt64Ty();
    auto *pools = offset(frame, offsetof(JitFrame, array_pools));
    auto *pool = b.CreateSelect(is_int, load(pools, 0, ptr),
                                b.CreateSelect(is_float, load(pools, sizeof(void *), ptr),
                                               load(pools, 2 * sizeof(void *), ptr)));
    auto *pool_begin = load(pool, 0, ptr);
    auto *pool_bytes = b.CreateSub(b.CreatePtrToInt(load(pool, sizeof(void *), ptr), i64),
                                   b.CreatePtrToInt(pool_begin, i64));
    auto *item_bytes = b.CreateSelect(
        is_int, b.getInt64(sizeof(Vector<int>)),
        b.CreateSelect(is_float, b.getInt64(sizeof(Vector<float>)),
                       b.getInt64(sizeof(Vector<float3>))));
    auto *item_offset = b.CreateMul(b.CreateZExt(handle, i64), item_bytes);
    auto *present = llvm::BasicBlock::Create(context, "array_present", function);
    b.CreateCondBr(b.CreateAnd(b.CreateICmpSGE(handle, b.getInt32(0)),
                               b.CreateICmpULT(item_offset, pool_bytes)),
                   present, generic);
    b.SetInsertPoint(present);
    return b.CreateGEP(b.getInt8Ty(), pool_begin, item_offset);
  }

  /* `append(a, x)` of one int, float or vector item, the case of call_builtin() that returns `a`
   * itself, within the capacity of the pool array. Returns true with the insert point in the
   * generic block, like emit_vector_builtin(). */
  bool emit_append(const Inst &in, const int sp, llvm::BasicBlock *next) {
    if (!native_pools || Builtin(in.imm) != Builtin::Append || in.a != 2)
      return false;
    auto *array = slot(stack, sp - 2);
    auto *item = slot(stack, sp - 1);
    auto *tag = load(array, offsetof(Value, type), b.getInt8Ty());
    auto *item_tag = load(item, offsetof(Value, type), b.getInt8Ty());
    auto *is_int = is_type(tag, Type::IntArray);
    auto *is_float = is_type(tag, Type::FloatArray);
    auto *is_vector = is_type(tag, Type::VecArray);
    auto *integer_item = b.CreateOr(is_type(item_tag, Type::Int), is_type(item_tag, Type::Bool));
    auto *scalar_item = is_scalar(item_tag);
    auto *eligible = b.CreateOr(
        b.CreateAnd(is_int, integer_item),
        b.CreateOr(b.CreateAnd(is_float, scalar_item),
                   b.CreateAnd(is_vector,
                               b.CreateOr(scalar_item, is_type(item_tag, Type::Vector)))));
    auto *native = llvm::BasicBlock::Create(context, "append", function);
    auto *generic = llvm::BasicBlock::Create(context, "generic_builtin", function);
    b.CreateCondBr(eligible, native, generic);
    b.SetInsertPoint(native);
    auto *i64 = b.getInt64Ty();
    auto *items = pool_array(is_int, is_float, load(array, offsetof(Value, i), i32), generic);
    auto *end = load(items, sizeof(void *), ptr);
    auto *stride = b.CreateSelect(is_vector, b.getInt64(12), b.getInt64(4));
    auto *new_end = b.CreateGEP(b.getInt8Ty(), end, stride);
    auto *fits = b.CreateICmpULE(b.CreatePtrToInt(new_end, i64),
                                 b.CreatePtrToInt(load(items, 2 * sizeof(void *), ptr), i64));
    auto *store = llvm::BasicBlock::Create(context, "append_store", function);
    b.CreateCondBr(fits, store, generic);
    b.SetInsertPoint(store);
    b.CreateStore(new_end, offset(items, sizeof(void *)));
    auto *int_case = llvm::BasicBlock::Create(context, "append_int", function);
    auto *float_case = llvm::BasicBlock::Create(context, "append_float", function);
    auto *vector_case = llvm::BasicBlock::Create(context, "append_vector", function);
    auto *other = llvm::BasicBlock::Create(context, "append_nonint", function);
    b.CreateCondBr(is_int, int_case, other);
    b.SetInsertPoint(other);
    b.CreateCondBr(is_float, float_case, vector_case);
    b.SetInsertPoint(int_case);
    b.CreateStore(load(item, offsetof(Value, i), i32), end);
    b.CreateBr(next);
    b.SetInsertPoint(float_case);
    b.CreateStore(as_float(item), end);
    b.CreateBr(next);
    b.SetInsertPoint(vector_case);
    for (int axis = 0; axis < 3; axis++)
      b.CreateStore(as_vec_component(item, axis), offset(end, 4 * axis));
    b.CreateBr(next);
    b.SetInsertPoint(generic);
    return true;
  }

  /* Takes the next array of pool `kind` (int, float, vector) like prepare_tls_array(): its size
   * becomes `bytes`, its capacity is kept. Branches to `generic` when the pool or the capacity
   * is too small, which the helpers grow. Returns the handle; `r_data` is the first item. */
  llvm::Value *take_pool_array(const int kind, llvm::Value *bytes, llvm::BasicBlock *generic,
                               llvm::Value *&r_data) {
    const size_t item_bytes = kind == 0   ? sizeof(Vector<int>)
                              : kind == 1 ? sizeof(Vector<float>)
                                          : sizeof(Vector<float3>);
    auto *i64 = b.getInt64Ty();
    auto *pool = load(frame, offsetof(JitFrame, array_pools) + kind * sizeof(void *), ptr);
    auto *used = load(frame, offsetof(JitFrame, array_pools_used) + kind * sizeof(void *), ptr);
    auto *handle = b.CreateLoad(i32, used);
    auto *pool_begin = load(pool, 0, ptr);
    auto *pool_bytes = b.CreateSub(b.CreatePtrToInt(load(pool, sizeof(void *), ptr), i64),
                                   b.CreatePtrToInt(pool_begin, i64));
    auto *item_offset = b.CreateMul(b.CreateZExt(handle, i64), b.getInt64(item_bytes));
    auto *present = llvm::BasicBlock::Create(context, "pool_array", function);
    b.CreateCondBr(b.CreateAnd(b.CreateICmpSGE(handle, b.getInt32(0)),
                               b.CreateICmpULT(item_offset, pool_bytes)),
                   present, generic);
    b.SetInsertPoint(present);
    auto *items = b.CreateGEP(b.getInt8Ty(), pool_begin, item_offset);
    r_data = load(items, 0, ptr);
    auto *capacity = b.CreateSub(b.CreatePtrToInt(load(items, 2 * sizeof(void *), ptr), i64),
                                 b.CreatePtrToInt(r_data, i64));
    auto *fits = llvm::BasicBlock::Create(context, "pool_array_fits", function);
    b.CreateCondBr(b.CreateICmpULE(bytes, capacity), fits, generic);
    b.SetInsertPoint(fits);
    b.CreateStore(b.CreateGEP(b.getInt8Ty(), r_data, bytes), offset(items, sizeof(void *)));
    b.CreateStore(b.CreateAdd(handle, b.getInt32(1)), used);
    return handle;
  }

  /* `facepoints(0, f)` and `facepoints(f)` of the mesh being wrangled: the corner vertices of
   * face `f` copied into an int pool array, an empty one outside the faces (geo_builtin_fn()).
   * Returns true with the insert point in the generic block, like emit_vector_builtin(). */
  bool emit_face_points(const Inst &in, const int sp, llvm::BasicBlock *next) {
    if (!native_pools || Builtin(in.imm) != Builtin::FacePoints || !ELEM(in.a, 1, 2))
      return false;
    auto *dst = slot(stack, sp - in.a);
    auto *face_arg = slot(stack, sp - 1);
    auto integer = [&](llvm::Value *value) {
      auto *tag = load(value, offsetof(Value, type), b.getInt8Ty());
      return b.CreateOr(is_type(tag, Type::Int), is_type(tag, Type::Bool));
    };
    auto *offsets = load(frame, offsetof(JitFrame, face_offsets), ptr);
    llvm::Value *eligible = b.CreateAnd(b.CreateIsNotNull(offsets), integer(face_arg));
    if (in.a == 2)
      eligible = b.CreateAnd(
          eligible, b.CreateAnd(integer(dst), b.CreateICmpEQ(load(dst, offsetof(Value, i), i32),
                                                              b.getInt32(0))));
    auto *native = llvm::BasicBlock::Create(context, "face_points", function);
    auto *generic = llvm::BasicBlock::Create(context, "generic_builtin", function);
    b.CreateCondBr(eligible, native, generic);
    b.SetInsertPoint(native);
    auto *i64 = b.getInt64Ty();
    auto *face = load(face_arg, offsetof(Value, i), i32);
    auto *inside = b.CreateICmpULT(face, load(frame, offsetof(JitFrame, faces_num), i32));
    auto *safe = b.CreateZExt(b.CreateSelect(inside, face, b.getInt32(0)), i64);
    auto *start = b.CreateSExt(
        b.CreateLoad(i32, b.CreateGEP(i32, offsets, safe)), i64);
    auto *end = b.CreateSExt(
        b.CreateLoad(i32, b.CreateGEP(i32, offsets, b.CreateAdd(safe, b.getInt64(1)))), i64);
    auto *bytes = b.CreateSelect(inside, b.CreateMul(b.CreateSub(end, start), b.getInt64(4)),
                                 b.getInt64(0));
    llvm::Value *data;
    auto *handle = take_pool_array(0, bytes, generic, data);
    auto *corner_verts = load(frame, offsetof(JitFrame, corner_verts), ptr);
    b.CreateMemCpy(data, llvm::MaybeAlign(4), b.CreateGEP(i32, corner_verts, start),
                   llvm::MaybeAlign(4), bytes);
    store_value(dst, Type::IntArray, llvm::ConstantFP::get(f32, 0), nullptr, nullptr, handle);
    b.CreateBr(next);
    b.SetInsertPoint(generic);
    return true;
  }

  /* `int a[];`, `float a[];` and `vector a[];`: store_tls_array() of an empty Vector. Returns
   * true with the insert point in the generic block, like emit_vector_builtin(). */
  bool emit_empty_array(const Inst &in, const int sp, llvm::BasicBlock *next) {
    const Builtin builtin = Builtin(in.imm);
    const int kind = builtin == Builtin::ArrayInt     ? 0
                     : builtin == Builtin::ArrayFloat ? 1
                     : builtin == Builtin::ArrayVec   ? 2
                                                      : -1;
    if (!native_pools || kind < 0 || in.a != 0)
      return false;
    const Type type = kind == 0 ? Type::IntArray : kind == 1 ? Type::FloatArray : Type::VecArray;
    auto *generic = llvm::BasicBlock::Create(context, "generic_builtin", function);
    llvm::Value *data;
    auto *handle = take_pool_array(kind, b.getInt64(0), generic, data);
    store_value(slot(stack, sp), type, llvm::ConstantFP::get(f32, 0), nullptr, nullptr, handle);
    b.CreateBr(next);
    b.SetInsertPoint(generic);
    return true;
  }

  bool emit_vector_builtin(const Inst &in, const int sp,
                           llvm::BasicBlock *next) {
    const Builtin builtin = Builtin(in.imm);
    const int count = in.a;
    const bool vec3 = builtin == Builtin::Vec3Fn && ELEM(count, 1, 3);
    const bool set = builtin == Builtin::Set && ELEM(count, 1, 3);
    const bool normalize = builtin == Builtin::Normalize && count == 1;
    if (!vec3 && !set && !normalize)
      return false;
    auto *dst = slot(stack, sp - count);
    auto *native = llvm::BasicBlock::Create(context, "vector_builtin", function);
    auto *generic = llvm::BasicBlock::Create(context, "generic_builtin", function);
    llvm::Value *r[3];
    if (count == 3) {
      llvm::Value *ok = b.getTrue();
      for (int i = 0; i < 3; i++)
        ok = b.CreateAnd(ok, is_scalar(load(slot(stack, sp - 3 + i),
                                             offsetof(Value, type), b.getInt8Ty())));
      b.CreateCondBr(ok, native, generic);
      b.SetInsertPoint(native);
      for (int i = 0; i < 3; i++)
        r[i] = as_float(slot(stack, sp - 3 + i));
    } else {
      auto *tag = load(dst, offsetof(Value, type), b.getInt8Ty());
      auto *vector = is_type(tag, Type::Vector);
      /* set(x) of one scalar is a 2D vector. */
      b.CreateCondBr(set ? vector : b.CreateOr(vector, is_scalar(tag)), native, generic);
      b.SetInsertPoint(native);
      for (int i = 0; i < 3; i++)
        r[i] = scalar_or_vec(dst, vector, i);
    }
    if (normalize) {
      /* math::normalize(): ((x*x + y*y) + z*z) > 1e-35, then divide by the length. */
      auto *length2 = b.CreateFAdd(
          b.CreateFAdd(b.CreateFMul(r[0], r[0]), b.CreateFMul(r[1], r[1])),
          b.CreateFMul(r[2], r[2]));
      auto *large = b.CreateFCmpOGT(length2, llvm::ConstantFP::get(f32, 1.0e-35f));
      auto *length = b.CreateUnaryIntrinsic(llvm::Intrinsic::sqrt, length2);
      for (int i = 0; i < 3; i++)
        r[i] = b.CreateSelect(large, b.CreateFDiv(r[i], length),
                              llvm::ConstantFP::get(f32, 0));
    }
    store_value(dst, Type::Vector, r[0], r[1], r[2]);
    b.CreateBr(next);
    b.SetInsertPoint(generic);
    return true;
  }

  llvm::Value *call_pointer(void *address, llvm::Type *result,
                            llvm::ArrayRef<llvm::Type *> types,
                            llvm::ArrayRef<llvm::Value *> args) {
    auto *target = llvm::ConstantExpr::getIntToPtr(
        b.getInt64(uintptr_t(address)), llvm::cast<llvm::PointerType>(ptr));
    return b.CreateCall(llvm::FunctionType::get(result, types, false), target,
                        args);
  }

  void bridge_before(const Inst &in, const int sp) {
    if (!private_storage)
      return;
    int pops, pushes;
    stack_effect(in, pops, pushes);
    for (int i = sp - pops; i < sp; i++)
      copy(slot(bridge_stack, i), slot(stack, i));
    auto local = [&](const int index) {
      if (index >= 0 && index < program.local_count)
        copy(slot(bridge_locals, index), slot(locals, index));
    };
    if (in.op == Op::PushLocalMember)
      local(in.imm);
    if (in.op == Op::PushLocalIndex && in.imm >= 0 &&
        in.imm < program.local_index_loads.size()) {
      const auto &load = program.local_index_loads[in.imm];
      local(load.array_local);
      if (!load.index_is_const)
        local(load.index);
      local(load.sample_index_array_local);
    }
    if (in.op == Op::GatherSamples && in.imm >= 0 &&
        in.imm < program.gather_samples.size()) {
      const auto &gather = program.gather_samples[in.imm];
      local(gather.index_array_local);
      /* A direct gather may deliberately leave an output local unchanged. */
      for (const int output : gather.output_locals)
        local(output);
    }
  }

  void bridge_after(const Inst &in, const int sp) {
    if (!private_storage)
      return;
    int pops, pushes;
    stack_effect(in, pops, pushes);
    for (int i = sp - pops; i < sp - pops + pushes; i++)
      copy(slot(stack, i), slot(bridge_stack, i));
    if (in.op == Op::GatherSamples && in.imm >= 0 &&
        in.imm < program.gather_samples.size()) {
      for (const int output : program.gather_samples[in.imm].output_locals)
        if (output >= 0 && output < program.local_count)
          copy(slot(locals, output), slot(bridge_locals, output));
    }
  }

  void invalidate_array_views() {
    if (native_pools)
      return;
    auto *cache = load(frame, offsetof(JitFrame, array_cache), ptr);
    auto *generation = offset(cache, offsetof(JitArrayCache, generation));
    b.CreateStore(
        b.CreateAdd(b.CreateLoad(b.getInt64Ty(), generation), b.getInt64(1)),
        generation);
  }

  void emit_count(const int key) {
    if (count_helpers())
      call_pointer(reinterpret_cast<void *>(&count_helper), b.getVoidTy(),
                   {i32}, {b.getInt32(std::clamp(key, 0, 1023))});
  }

  bool emit_helper(const Inst &in, const int sp, llvm::BasicBlock *next) {
    void *address = jit_operation_address(in.op);
    if (!address)
      return false;
    helper_ops++;
    emit_count(in.op == Op::Call ? 256 + in.imm : int(in.op));
    bool mutates_arrays =
        !ELEM(in.op, Op::GetIndex, Op::ArrayLen, Op::StoreAttr, Op::GetMember,
              Op::SetMember, Op::Neg);
    if (in.op == Op::PushLocalIndex && in.imm >= 0 &&
        in.imm < program.local_index_loads.size())
      mutates_arrays = program.local_index_loads[in.imm].sample_slot >= 0;
    if (mutates_arrays)
      invalidate_array_views();
    bridge_before(in, sp);
    auto *ok = call_pointer(
        address, b.getInt1Ty(), {ptr, i32, i32, i32},
        {frame, b.getInt32(in.imm), b.getInt32(in.a), b.getInt32(sp)});
    if (private_storage) {
      auto *success = llvm::BasicBlock::Create(context, "helper_ok", function);
      b.CreateCondBr(ok, success, failed);
      b.SetInsertPoint(success);
      bridge_after(in, sp);
      b.CreateBr(next);
    } else
      b.CreateCondBr(ok, next, failed);
    return true;
  }

  void finish_index_read(const Inst &in, llvm::Value *dst,
                         llvm::BasicBlock *next) {
    if (in.op == Op::PushLocalIndex &&
        program.local_index_loads[in.imm].member >= 0) {
      const int packed = program.local_index_loads[in.imm].member;
      const int count = swizzle_count(packed);
      llvm::Value *parts[4]{};
      for (int i = 0; i < count; i++)
        parts[i] = axis(dst, swizzle_axis(packed, i));
      store_value(dst,
                  count == 1   ? Type::Float
                  : count == 2 ? Type::Vector2
                  : count == 3 ? Type::Vector
                               : Type::Vector4,
                  parts[0], parts[1], parts[2], nullptr, parts[3]);
    }
    b.CreateBr(next);
  }

  bool emit_array(const Inst &in, const int sp, llvm::Value *array,
                  llvm::Value *index_value, llvm::Value *dst,
                  llvm::BasicBlock *next,
                  llvm::Value *constant_index = nullptr) {
    if (std::getenv("BLENDER_VEX_JIT_DISABLE_ARRAY_VIEWS"))
      return emit_helper(in, sp, next);
    const bool length = in.op == Op::ArrayLen;
    const bool write = in.op == Op::SetIndex;
    auto *tag = load(array, offsetof(Value, type), b.getInt8Ty());
    auto *handle = load(array, offsetof(Value, i), i32);
    auto *is_int = is_type(tag, Type::IntArray);
    auto *is_float = is_type(tag, Type::FloatArray);
    auto *is_vector = is_type(tag, Type::VecArray);
    llvm::Value *eligible =
        b.CreateAnd(b.CreateOr(is_int, b.CreateOr(is_float, is_vector)),
                    b.CreateICmpSGE(handle, b.getInt32(0)));
    llvm::Value *raw = b.getInt32(0);
    llvm::Value *integer_index = b.getTrue();
    if (constant_index)
      raw = constant_index;
    else if (!length) {
      auto *index_tag = load(index_value, offsetof(Value, type), b.getInt8Ty());
      integer_index = b.CreateOr(is_type(index_tag, Type::Int),
                                 is_type(index_tag, Type::Bool));
      eligible =
          b.CreateAnd(eligible, b.CreateOr(is_type(index_tag, Type::Int),
                                           is_type(index_tag, Type::Bool)));
      raw = load(index_value, offsetof(Value, i), i32);
    }
    auto *element = write ? slot(stack, sp - 3) : nullptr;
    if (write) {
      auto *element_tag = load(element, offsetof(Value, type), b.getInt8Ty());
      eligible = b.CreateAnd(
          eligible, b.CreateOr(b.CreateNot(is_int),
                               b.CreateOr(is_type(element_tag, Type::Int),
                                          is_type(element_tag, Type::Bool))));
    }
    if (!length) {
      auto *component_path =
          llvm::BasicBlock::Create(context, "component_index", function);
      auto *array_path =
          llvm::BasicBlock::Create(context, "array_index", function);
      auto *vector_like = b.CreateOr(
          is_type(tag, Type::Vector2),
          b.CreateOr(is_type(tag, Type::Vector),
                     b.CreateOr(is_type(tag, Type::Vector4),
                                b.CreateOr(is_type(tag, Type::Color),
                                           is_type(tag, Type::Rotation)))));
      b.CreateCondBr(b.CreateAnd(vector_like, integer_index), component_path,
                     array_path);
      b.SetInsertPoint(component_path);
      auto *dimension =
          b.CreateSelect(is_type(tag, Type::Vector2), b.getInt32(2),
                         b.CreateSelect(is_type(tag, Type::Vector),
                                        b.getInt32(3), b.getInt32(4)));
      auto *index = b.CreateSelect(b.CreateICmpSLT(raw, b.getInt32(0)),
                                   b.CreateAdd(raw, dimension), raw);
      auto *valid = b.CreateICmpULT(index, dimension);
      auto *safe = b.CreateSelect(valid, index, b.getInt32(0));
      auto *values = b.CreateAlignedLoad(llvm::FixedVectorType::get(f32, 4),
                                         offset(array, offsetof(Value, v)),
                                         llvm::Align(alignof(float)));
      if (write) {
        auto *updated = b.CreateInsertElement(values, as_float(element), safe);
        copy(dst, array);
        b.CreateAlignedStore(b.CreateSelect(valid, updated, values),
                             offset(dst, offsetof(Value, v)),
                             llvm::Align(alignof(float)));
        b.CreateBr(next);
      } else {
        store_value(dst, Type::Float,
                    b.CreateSelect(valid, b.CreateExtractElement(values, safe),
                                   llvm::ConstantFP::get(f32, 0)));
        finish_index_read(in, dst, next);
      }
      b.SetInsertPoint(array_path);
    }
    auto *lookup = llvm::BasicBlock::Create(context, "array_lookup", function);
    auto *fallback =
        llvm::BasicBlock::Create(context, "array_generic", function);
    auto *ready = llvm::BasicBlock::Create(context, "array_ready", function);
    b.CreateCondBr(eligible, lookup, fallback);
    b.SetInsertPoint(lookup);
    /* `items` is the #Vector object of a pool array, only on the native path. */
    llvm::Value *size = nullptr, *data = nullptr, *items = nullptr;
    if (native_pools) {
      auto *i64 = b.getInt64Ty();
      items = pool_array(is_int, is_float, handle, fallback);
      data = load(items, 0, ptr);
      auto *bytes = b.CreateSub(b.CreatePtrToInt(load(items, sizeof(void *), ptr), i64),
                                b.CreatePtrToInt(data, i64));
      size = b.CreateTrunc(b.CreateSelect(is_vector, b.CreateUDiv(bytes, b.getInt64(12)),
                                          b.CreateLShr(bytes, b.getInt64(2))),
                           i32);
      b.CreateBr(ready);
    } else {
      auto *resolve =
          llvm::BasicBlock::Create(context, "array_resolve", function);
      auto *resolved =
          llvm::BasicBlock::Create(context, "array_resolved", function);
      auto *cache = load(frame, offsetof(JitFrame, array_cache), ptr);
      auto *generation =
          load(cache, offsetof(JitArrayCache, generation), b.getInt64Ty());
      auto *tag32 = b.CreateZExt(tag, i32);
      auto *hash =
          b.CreateAnd(b.CreateAdd(b.CreateMul(tag32, b.getInt32(17)), handle),
                      b.getInt32(63));
      auto *view = b.CreateGEP(b.getInt8Ty(),
                               offset(cache, offsetof(JitArrayCache, views)),
                               b.CreateMul(b.CreateZExt(hash, b.getInt64Ty()),
                                           b.getInt64(sizeof(JitArrayView))));
      auto *hit = b.CreateAnd(
          b.CreateICmpEQ(
              generation,
              load(view, offsetof(JitArrayView, generation), b.getInt64Ty())),
          b.CreateAnd(
              b.CreateICmpEQ(tag32,
                             load(view, offsetof(JitArrayView, type), i32)),
              b.CreateICmpEQ(handle,
                             load(view, offsetof(JitArrayView, handle), i32))));
      b.CreateCondBr(hit, ready, resolve);
      b.SetInsertPoint(resolve);
      emit_count(254);
      auto *ok =
          call_pointer(reinterpret_cast<void *>(&jit_array_view), b.getInt1Ty(),
                       {i32, i32, ptr}, {tag32, handle, view});
      b.CreateCondBr(ok, resolved, fallback);
      b.SetInsertPoint(resolved);
      b.CreateStore(tag32, offset(view, offsetof(JitArrayView, type)));
      b.CreateStore(handle, offset(view, offsetof(JitArrayView, handle)));
      b.CreateStore(generation, offset(view, offsetof(JitArrayView, generation)));
      b.CreateBr(ready);
      b.SetInsertPoint(ready);
      size = load(view, offsetof(JitArrayView, size), i32);
      data = load(view, offsetof(JitArrayView, data), ptr);
    }
    b.SetInsertPoint(ready);
    if (length) {
      store_value(dst, Type::Int, b.CreateSIToFP(size, f32), nullptr, nullptr,
                  size);
      b.CreateBr(next);
    } else {
      auto *index = b.CreateSelect(b.CreateICmpSLT(raw, b.getInt32(0)),
                                   b.CreateAdd(raw, size), raw);
      auto *valid = b.CreateICmpULT(index, size);
      auto *access =
          llvm::BasicBlock::Create(context, "array_access", function);
      auto *missing =
          write && items ? llvm::BasicBlock::Create(context, "array_grow", function)
          : write        ? fallback
                         : llvm::BasicBlock::Create(context, "array_zero", function);
      b.CreateCondBr(valid, access, missing);
      if (write && items) {
        /* `a[i] = x` past the end zero-fills the gap like array_write_index(). Within the
         * capacity the pool array only moves its end, otherwise the helper reallocates. */
        b.SetInsertPoint(missing);
        auto *i64 = b.getInt64Ty();
        auto *stride = b.CreateSelect(is_vector, b.getInt64(12), b.getInt64(4));
        auto *end = b.CreateMul(b.CreateZExt(b.CreateAdd(raw, b.getInt32(1)), i64), stride);
        auto *capacity = b.CreateSub(
            b.CreatePtrToInt(load(items, 2 * sizeof(void *), ptr), i64),
            b.CreatePtrToInt(data, i64));
        auto *fits = b.CreateAnd(
            b.CreateICmpSGE(raw, b.getInt32(0)),
            b.CreateAnd(b.CreateICmpSLT(raw, b.getInt32(k_jit_array_grow_max)),
                        b.CreateICmpULE(end, capacity)));
        auto *grow = llvm::BasicBlock::Create(context, "array_grow_fill", function);
        b.CreateCondBr(fits, grow, fallback);
        b.SetInsertPoint(grow);
        auto *filled = b.CreateMul(b.CreateZExt(size, i64), stride);
        b.CreateMemSet(b.CreateGEP(b.getInt8Ty(), data, filled), b.getInt8(0),
                       b.CreateSub(end, filled), llvm::MaybeAlign(4));
        b.CreateStore(b.CreateGEP(b.getInt8Ty(), data, end), offset(items, sizeof(void *)));
        b.CreateBr(access);
      }
      if (!write) {
        b.SetInsertPoint(missing);
        store_value(dst, Type::Float, llvm::ConstantFP::get(f32, 0));
        b.CreateStore(
            b.CreateSelect(is_int, b.getInt8(uint8_t(Type::Int)),
                           b.CreateSelect(is_vector,
                                          b.getInt8(uint8_t(Type::Vector)),
                                          b.getInt8(uint8_t(Type::Float)))),
            offset(dst, offsetof(Value, type)));
        finish_index_read(in, dst, next);
      }
      b.SetInsertPoint(access);
      auto *stride = b.CreateSelect(is_vector, b.getInt64(12), b.getInt64(4));
      auto *item =
          b.CreateGEP(b.getInt8Ty(), data,
                      b.CreateMul(b.CreateSExt(index, b.getInt64Ty()), stride));
      auto *int_case = llvm::BasicBlock::Create(context, "array_int", function);
      auto *nonint_case =
          llvm::BasicBlock::Create(context, "array_nonint", function);
      auto *float_case =
          llvm::BasicBlock::Create(context, "array_float", function);
      auto *vector_case =
          llvm::BasicBlock::Create(context, "array_vector", function);
      b.CreateCondBr(is_int, int_case, nonint_case);
      b.SetInsertPoint(int_case);
      if (write) {
        b.CreateStore(load(element, offsetof(Value, i), i32), item);
        copy(dst, array);
      } else {
        auto *integer = b.CreateLoad(i32, item);
        store_value(dst, Type::Int, b.CreateSIToFP(integer, f32), nullptr,
                    nullptr, integer);
      }
      if (write)
        b.CreateBr(next);
      else
        finish_index_read(in, dst, next);
      b.SetInsertPoint(nonint_case);
      b.CreateCondBr(is_float, float_case, vector_case);
      b.SetInsertPoint(float_case);
      if (write) {
        b.CreateStore(as_float(element), item);
        copy(dst, array);
      } else
        store_value(dst, Type::Float, b.CreateLoad(f32, item));
      if (write)
        b.CreateBr(next);
      else
        finish_index_read(in, dst, next);
      b.SetInsertPoint(vector_case);
      if (write) {
        for (int axis = 0; axis < 3; axis++)
          b.CreateStore(as_vec_component(element, axis),
                        offset(item, 4 * axis));
        copy(dst, array);
      } else {
        llvm::Value *x = b.CreateLoad(f32, item),
                    *y = b.CreateLoad(f32, offset(item, 4)),
                    *z = b.CreateLoad(f32, offset(item, 8));
        store_value(dst, Type::Vector, x, y, z);
      }
      if (write)
        b.CreateBr(next);
      else
        finish_index_read(in, dst, next);
    }
    b.SetInsertPoint(fallback);
    return emit_helper(in, sp, next);
  }

  /* `i[]@a`, `f[]@a` and `v[]@a` stored as the same kind and not continued in chunk attributes:
   * load_attr_value() copies the packed items into a pool array (packed_to_value()). */
  bool emit_array_attribute(const Inst &in, const int sp, llvm::BasicBlock *next,
                            const Type type) {
    using bke::WrangleArrayKind;
    using bke::WrangleArrayValue;
    const int kind = type == Type::IntArray ? 0 : type == Type::FloatArray ? 1 : 2;
    const WrangleArrayKind stored = kind == 0   ? WrangleArrayKind::Int
                                    : kind == 1 ? WrangleArrayKind::Float
                                                : WrangleArrayKind::Float3;
    auto *available = llvm::BasicBlock::Create(context, "array_attribute", function);
    auto *packed = llvm::BasicBlock::Create(context, "array_attribute_packed", function);
    auto *fallback = llvm::BasicBlock::Create(context, "attribute_generic", function);
    auto *count = load(frame, offsetof(JitFrame, attrs_num), i32);
    b.CreateCondBr(b.CreateICmpSGT(count, b.getInt32(in.imm)), available, fallback);
    b.SetInsertPoint(available);
    auto *attr = offset(load(frame, offsetof(JitFrame, attrs), ptr), sizeof(AttrRT) * in.imm);
    auto *index = load(load(frame, offsetof(JitFrame, env), ptr), offsetof(VMEnv, index), i32);
    auto *written = load(attr, offsetof(AttrRT, warr), ptr);
    auto *heads = b.CreateSelect(b.CreateIsNotNull(written), written,
                                 load(attr, offsetof(AttrRT, rarr), ptr));
    b.CreateCondBr(
        b.CreateAnd(b.CreateAnd(is_type(load(attr, offsetof(AttrRT, type), b.getInt8Ty()), type),
                                b.CreateIsNotNull(heads)),
                    b.CreateICmpULT(index, load(attr, offsetof(AttrRT, size), i32))),
        packed, fallback);
    b.SetInsertPoint(packed);
    auto *head = b.CreateGEP(
        b.getInt8Ty(), heads,
        b.CreateMul(b.CreateSExt(index, b.getInt64Ty()), b.getInt64(sizeof(WrangleArrayValue))));
    auto *same_kind = b.CreateAnd(
        b.CreateICmpEQ(load(head, offsetof(WrangleArrayValue, kind), b.getInt8Ty()),
                       b.getInt8(uint8_t(stored))),
        b.CreateICmpNE(load(head, offsetof(WrangleArrayValue, truncated), b.getInt8Ty()),
                       b.getInt8(WrangleArrayValue::continued)));
    auto *copy_block = llvm::BasicBlock::Create(context, "array_attribute_copy", function);
    b.CreateCondBr(same_kind, copy_block, fallback);
    b.SetInsertPoint(copy_block);
    copy_packed(head, kind, slot(stack, sp), fallback, next);
    b.SetInsertPoint(fallback);
    return emit_helper(in, sp, next);
  }

  /* `i[]@a = ints`, `f[]@a = floats` and `v[]@a = vectors` that fit one packed value: the
   * set_ints(), set_floats() and set_float3s() of store_attr_value(). An earlier long array of
   * this evaluation (ArrayOverflow::any) and every other case take the helper. */
  bool emit_array_store(const Inst &in, const int sp, llvm::BasicBlock *next, const Type type) {
    using bke::WrangleArrayKind;
    using bke::WrangleArrayValue;
    const int kind = type == Type::IntArray ? 0 : type == Type::FloatArray ? 1 : 2;
    const WrangleArrayKind stored = kind == 0   ? WrangleArrayKind::Int
                                    : kind == 1 ? WrangleArrayKind::Float
                                                : WrangleArrayKind::Float3;
    const int max_items = kind == 2 ? WrangleArrayValue::max_values / 3
                                    : WrangleArrayValue::max_values;
    auto *value = slot(stack, sp - 1);
    auto *available = llvm::BasicBlock::Create(context, "array_store", function);
    auto *bound = llvm::BasicBlock::Create(context, "array_store_bound", function);
    auto *overflow_check = llvm::BasicBlock::Create(context, "array_store_overflow", function);
    auto *fallback = llvm::BasicBlock::Create(context, "attribute_generic", function);
    auto *count = load(frame, offsetof(JitFrame, attrs_num), i32);
    b.CreateCondBr(
        b.CreateAnd(b.CreateICmpSGT(count, b.getInt32(in.imm)),
                    is_type(load(value, offsetof(Value, type), b.getInt8Ty()), type)),
        available, fallback);
    b.SetInsertPoint(available);
    auto *attr = offset(load(frame, offsetof(JitFrame, attrs), ptr), sizeof(AttrRT) * in.imm);
    auto *index = load(load(frame, offsetof(JitFrame, env), ptr), offsetof(VMEnv, index), i32);
    auto *heads = load(attr, offsetof(AttrRT, warr), ptr);
    b.CreateCondBr(
        b.CreateAnd(b.CreateAnd(is_type(load(attr, offsetof(AttrRT, type), b.getInt8Ty()), type),
                                b.CreateIsNotNull(heads)),
                    b.CreateICmpULT(index, load(attr, offsetof(AttrRT, size), i32))),
        bound, fallback);
    b.SetInsertPoint(bound);
    auto *overflow = load(attr, offsetof(AttrRT, array_overflow), ptr);
    auto *no_overflow = llvm::BasicBlock::Create(context, "array_store_pool", function);
    b.CreateCondBr(b.CreateIsNull(overflow), no_overflow, overflow_check);
    b.SetInsertPoint(overflow_check);
    b.CreateCondBr(b.CreateICmpEQ(load(overflow, jit_array_overflow_any_offset(), b.getInt8Ty()),
                                  b.getInt8(0)),
                   no_overflow, fallback);
    b.SetInsertPoint(no_overflow);
    auto *i64 = b.getInt64Ty();
    auto *items = pool_array(b.getInt1(kind == 0), b.getInt1(kind == 1),
                             load(value, offsetof(Value, i), i32), fallback);
    auto *data = load(items, 0, ptr);
    auto *bytes = b.CreateSub(b.CreatePtrToInt(load(items, sizeof(void *), ptr), i64),
                              b.CreatePtrToInt(data, i64));
    auto *size = b.CreateUDiv(bytes, b.getInt64(kind == 2 ? 12 : 4));
    auto *write = llvm::BasicBlock::Create(context, "array_store_write", function);
    b.CreateCondBr(b.CreateICmpULE(size, b.getInt64(max_items)), write, fallback);
    b.SetInsertPoint(write);
    auto *head = b.CreateGEP(
        b.getInt8Ty(), heads,
        b.CreateMul(b.CreateSExt(index, i64), b.getInt64(sizeof(WrangleArrayValue))));
    b.CreateMemSet(head, b.getInt8(0), sizeof(WrangleArrayValue), llvm::MaybeAlign(4));
    auto *count16 = b.CreateTrunc(size, b.getInt16Ty());
    b.CreateStore(count16, offset(head, offsetof(WrangleArrayValue, count)));
    b.CreateStore(count16, offset(head, offsetof(WrangleArrayValue, total)));
    b.CreateStore(b.getInt8(uint8_t(stored)), offset(head, offsetof(WrangleArrayValue, kind)));
    b.CreateMemCpy(offset(head, offsetof(WrangleArrayValue, d)), llvm::MaybeAlign(4), data,
                   llvm::MaybeAlign(4), bytes);
    b.CreateBr(next);
    b.SetInsertPoint(fallback);
    return emit_helper(in, sp, next);
  }

  /* packed_to_value() of a packed Int, Float or Float3 array (`kind` 0, 1, 2) into `dst`. */
  void copy_packed(llvm::Value *head, const int kind, llvm::Value *dst,
                   llvm::BasicBlock *generic, llvm::BasicBlock *next) {
    using bke::WrangleArrayValue;
    auto *items = b.CreateZExt(load(head, offsetof(WrangleArrayValue, count), b.getInt16Ty()),
                               b.getInt64Ty());
    auto *bytes = b.CreateMul(items, b.getInt64(kind == 2 ? 12 : 4));
    llvm::Value *data;
    auto *handle = take_pool_array(kind, bytes, generic, data);
    b.CreateMemCpy(data, llvm::MaybeAlign(4), offset(head, offsetof(WrangleArrayValue, d)),
                   llvm::MaybeAlign(4), bytes);
    store_value(dst,
                kind == 0   ? Type::IntArray
                : kind == 1 ? Type::FloatArray
                            : Type::VecArray,
                llvm::ConstantFP::get(f32, 0), nullptr, nullptr, handle);
    b.CreateBr(next);
  }

  bool emit_attribute(const Inst &in, const int sp, llvm::BasicBlock *next) {
    if (in.imm < 0 || in.imm >= program.attrs.size())
      return emit_helper(in, sp, next);
    const Type type = program.attrs[in.imm].type;
    if (in.op == Op::PushAttr && native_pools &&
        ELEM(type, Type::IntArray, Type::FloatArray, Type::VecArray))
      return emit_array_attribute(in, sp, next, type);
    if (in.op == Op::StoreAttr && native_pools &&
        ELEM(type, Type::IntArray, Type::FloatArray, Type::VecArray))
      return emit_array_store(in, sp, next, type);
    /* AttrAdd is attr_add_inplace(): `+=` into the writable array only. */
    const bool add = in.op == Op::AttrAdd;
    const bool write = in.op == Op::StoreAttr || add;
    if (add && type == Type::Bool)
      return emit_helper(in, sp, next);
    size_t writable, readable;
    int width = 1;
    switch (type) {
    case Type::Float:
      writable = offsetof(AttrRT, wf);
      readable = offsetof(AttrRT, rf);
      break;
    case Type::Int:
      writable = offsetof(AttrRT, wi);
      readable = offsetof(AttrRT, ri);
      break;
    case Type::Bool:
      writable = offsetof(AttrRT, wb);
      readable = offsetof(AttrRT, rb);
      break;
    case Type::Vector:
      writable = offsetof(AttrRT, wv);
      readable = offsetof(AttrRT, rv);
      width = 3;
      break;
    default:
      return emit_helper(in, sp, next);
    }
    auto *available = llvm::BasicBlock::Create(context, "attribute", function);
    auto *direct =
        llvm::BasicBlock::Create(context, "attribute_direct", function);
    auto *fallback =
        llvm::BasicBlock::Create(context, "attribute_generic", function);
    auto *attrs = load(frame, offsetof(JitFrame, attrs), ptr);
    auto *count = load(frame, offsetof(JitFrame, attrs_num), i32);
    b.CreateCondBr(b.CreateICmpSGT(count, b.getInt32(in.imm)), available,
                   fallback);
    b.SetInsertPoint(available);
    auto *attr = offset(attrs, sizeof(AttrRT) * size_t(in.imm));
    auto *env = load(frame, offsetof(JitFrame, env), ptr);
    auto *index = load(env, offsetof(VMEnv, index), i32);
    auto *data = load(attr, writable, ptr);
    if (!write)
      data = b.CreateSelect(b.CreateIsNotNull(data), data,
                            load(attr, readable, ptr));
    llvm::Value *valid = b.CreateAnd(
        is_type(load(attr, offsetof(AttrRT, type), b.getInt8Ty()), type),
        b.CreateIsNotNull(data));
    valid = b.CreateAnd(
        valid, b.CreateICmpULT(index, load(attr, offsetof(AttrRT, size), i32)));
    auto *value = slot(stack, sp - (write ? 1 : 0));
    if (write && type == Type::Int) {
      auto *tag = load(value, offsetof(Value, type), b.getInt8Ty());
      valid = b.CreateAnd(
          valid, b.CreateOr(is_type(tag, Type::Int), is_type(tag, Type::Bool)));
    }
    b.CreateCondBr(valid, direct, fallback);
    b.SetInsertPoint(direct);
    auto *element = b.CreateGEP(
        b.getInt8Ty(), data,
        b.CreateMul(b.CreateSExt(index, b.getInt64Ty()),
                    b.getInt64(type == Type::Bool ? 1 : width * 4)));
    if (write) {
      if (type == Type::Int) {
        llvm::Value *integer = load(value, offsetof(Value, i), i32);
        if (add)
          integer = b.CreateAdd(b.CreateLoad(i32, element), integer);
        b.CreateStore(integer, element);
      }
      else if (type == Type::Bool)
        b.CreateStore(b.CreateZExt(truth(value), b.getInt8Ty()), element);
      else if (type == Type::Float) {
        llvm::Value *x = as_float(value);
        if (add)
          x = b.CreateFAdd(b.CreateLoad(f32, element), x);
        b.CreateStore(x, element);
      }
      else {
        for (int axis = 0; axis < 3; axis++) {
          llvm::Value *x = as_vec_component(value, axis);
          if (add)
            x = b.CreateFAdd(b.CreateLoad(f32, offset(element, 4 * axis)), x);
          b.CreateStore(x, offset(element, 4 * axis));
        }
      }
    } else if (type == Type::Int || type == Type::Bool) {
      auto *integer =
          type == Type::Bool
              ? b.CreateZExt(b.CreateLoad(b.getInt8Ty(), element), i32)
              : b.CreateLoad(i32, element);
      store_value(value, type, b.CreateSIToFP(integer, f32), nullptr, nullptr,
                  integer);
    } else {
      llvm::Value *parts[3]{};
      for (int axis = 0; axis < width; axis++)
        parts[axis] = b.CreateLoad(f32, offset(element, 4 * axis));
      store_value(value, type, parts[0], parts[1], parts[2]);
    }
    b.CreateBr(next);
    b.SetInsertPoint(fallback);
    return emit_helper(in, sp, next);
  }

public:
  int native_ops = 0, helper_ops = 0;
  Lowering(const Program &p, llvm::LLVMContext &c, llvm::Module &m,
           const std::vector<int> &d, const std::string &name,
           const int capacity, const bool use_private_storage,
           const Span<JitUniformParm> parms, const bool specialize)
      : program(p), context(c), module(m), b(c),
        private_storage(use_private_storage), uniform_parms(parms),
        specialize_parms(specialize), depth(d) {
    ptr = b.getPtrTy();
    f32 = b.getFloatTy();
    i32 = b.getInt32Ty();
    function = llvm::Function::Create(
        llvm::FunctionType::get(b.getInt1Ty(), {ptr}, false),
        llvm::Function::ExternalLinkage, name, module);
    auto *entry = llvm::BasicBlock::Create(context, "entry", function);
    b.SetInsertPoint(entry);
    frame = function->getArg(0);
    llvm::Value *parm_values = nullptr;
    if (!uniform_parms.is_empty() && !specialize_parms) {
      parm_values = load(frame, offsetof(JitFrame, uniform_parms), ptr);
    }
    for (const int index : uniform_parms.index_range()) {
      const Value &value = uniform_parms[index].value;
      auto *dst = b.CreateAlloca(b.getInt8Ty(), b.getInt64(sizeof(Value)));
      dst->setAlignment(llvm::Align(alignof(Value)));
      llvm::Value *components[4];
      for (int axis = 0; axis < 4; axis++) {
        components[axis] = specialize_parms ?
            static_cast<llvm::Value *>(llvm::ConstantFP::get(f32, value.v[axis])) :
            component(slot(parm_values, index), axis);
      }
      llvm::Value *integer = specialize_parms ?
          static_cast<llvm::Value *>(b.getInt32(value.i)) :
          load(slot(parm_values, index), offsetof(Value, i), i32);
      store_value(dst, value.type, components[0], components[1], components[2], integer,
                  components[3]);
      uniform_slots.push_back(dst);
    }
    /* Runtime operation helpers do not consume the main kernel's branch count.
     * Keep it private so mem2reg can retain it across their calls. Synchronize
     * at return/error boundaries and before reporting the instruction limit. */
    branch_counter = b.CreateAlloca(b.getInt64Ty());
    b.CreateStore(load(frame, offsetof(JitFrame, jumps), b.getInt64Ty()),
                  branch_counter);
    bridge_stack = load(frame, offsetof(JitFrame, stack), ptr);
    bridge_locals = load(frame, offsetof(JitFrame, locals), ptr);
    stack = bridge_stack;
    locals = bridge_locals;
    if (private_storage) {
      auto *private_stack = b.CreateAlloca(
          llvm::ArrayType::get(b.getInt8Ty(), sizeof(Value) * capacity));
      auto *private_locals = b.CreateAlloca(llvm::ArrayType::get(
          b.getInt8Ty(), sizeof(Value) * std::max(p.local_count, 1)));
      private_stack->setAlignment(llvm::Align(alignof(Value)));
      private_locals->setAlignment(llvm::Align(alignof(Value)));
      stack = private_stack;
      locals = private_locals;
      /* These addresses never escape to a runtime helper. SROA/mem2reg can
       * retain local values across calls and promote loop state to SSA. */
      b.CreateMemSet(locals, b.getInt8(0),
                     sizeof(Value) * std::max(p.local_count, 1),
                     llvm::MaybeAlign(alignof(Value)));
      for (int i = 0; i < p.local_count; i++)
        b.CreateStore(b.getInt8(uint8_t(Type::Float)),
                      offset(slot(locals, i), offsetof(Value, type)));
    }
    for (int pc = 0; pc <= int(p.code.size()); pc++) {
      blocks.push_back(depth[pc] >= 0
                           ? llvm::BasicBlock::Create(context, "pc", function)
                           : nullptr);
    }
    failed = llvm::BasicBlock::Create(context, "error", function);
    b.CreateBr(blocks[0]);
  }

  bool emit() {
    const int size = int(program.code.size());
    for (int pc = 0; pc <= size; pc++) {
      if (!blocks[pc])
        continue;
      b.SetInsertPoint(blocks[pc]);
      const int sp = depth[pc];
      if (pc == size || program.code[pc].op == Op::Return) {
        b.CreateStore(b.CreateLoad(b.getInt64Ty(), branch_counter),
                      offset(frame, offsetof(JitFrame, jumps)));
        if (sp > 0)
          copy(offset(frame, offsetof(JitFrame, return_value)),
               slot(stack, sp - 1));
        b.CreateRet(b.getTrue());
        continue;
      }
      /* The element loop polls cancellation before a kernel starts. */
      if (pc > 0 && (pc & 1023) == 0) {
        auto *allowed =
            call_pointer(reinterpret_cast<void *>(&jit_cancel_guard),
                         b.getInt1Ty(), {ptr}, {frame});
        auto *body = llvm::BasicBlock::Create(context, "body", function);
        b.CreateCondBr(allowed, body, failed);
        b.SetInsertPoint(body);
      }
      const Inst &in = program.code[pc];
      auto *next = blocks[pc + 1];
      native_ops++;
      switch (in.op) {
      case Op::Nop:
        break;
      case Op::PushAttr:
      case Op::StoreAttr:
      case Op::AttrAdd:
        if (!emit_attribute(in, sp, next))
          return false;
        continue;
      case Op::PushIndex:
      case Op::PushNpoints:
      case Op::PushNedges:
      case Op::PushNfaces:
      case Op::PushNcorners: {
        const size_t member =
            in.op == Op::PushIndex     ? offsetof(VMEnv, index)
            : in.op == Op::PushNpoints ? offsetof(VMEnv, npoints)
            : in.op == Op::PushNedges  ? offsetof(VMEnv, nedges)
            : in.op == Op::PushNfaces  ? offsetof(VMEnv, nfaces)
                                       : offsetof(VMEnv, ncorners);
        auto *integer =
            load(load(frame, offsetof(JitFrame, env), ptr), member, i32);
        store_value(slot(stack, sp), Type::Int, b.CreateSIToFP(integer, f32),
                    nullptr, nullptr, integer);
        break;
      }
      case Op::PushF:
        store_value(slot(stack, sp), Type::Float,
                    llvm::ConstantFP::get(f32, program.const_f[in.imm]));
        break;
      case Op::PushI:
      case Op::PushB:
      case Op::PushS: {
        int value = in.op == Op::PushI ? program.const_i[in.imm] : in.imm;
        if (in.op == Op::PushB)
          value = value != 0;
        const Type type = in.op == Op::PushI   ? Type::Int
                          : in.op == Op::PushB ? Type::Bool
                                               : Type::String;
        store_value(
            slot(stack, sp), type,
            llvm::ConstantFP::get(f32, in.op == Op::PushS ? 0 : float(value)),
            nullptr, nullptr, b.getInt32(value));
        break;
      }
      case Op::PushV: {
        const float3 v = program.const_v[in.imm];
        store_value(
            slot(stack, sp), Type::Vector, llvm::ConstantFP::get(f32, v.x),
            llvm::ConstantFP::get(f32, v.y), llvm::ConstantFP::get(f32, v.z));
        break;
      }
      case Op::PushLocal:
        if (in.imm < 0 || in.imm >= program.local_count)
          return false;
        copy(slot(stack, sp), slot(locals, in.imm));
        break;
      case Op::GetIndex:
      case Op::SetIndex:
      case Op::ArrayLen: {
        const bool length = in.op == Op::ArrayLen;
        auto *array = slot(stack, sp - (length ? 1 : 2));
        auto *index = length ? nullptr : slot(stack, sp - 1);
        auto *dst = slot(stack, sp - (in.op == Op::SetIndex ? 3
                                      : length              ? 1
                                                            : 2));
        if (!emit_array(in, sp, array, index, dst, next))
          return false;
        continue;
      }
      case Op::PushLocalIndex: {
        if (in.imm < 0 || in.imm >= program.local_index_loads.size())
          return false;
        const auto &read = program.local_index_loads[in.imm];
        if (read.sample_slot >= 0 || read.array_local < 0 ||
            read.array_local >= program.local_count || read.index < 0 ||
            (read.index_is_const ? read.index >= program.const_i.size()
                                 : read.index >= program.local_count)) {
          native_ops--;
          if (!emit_helper(in, sp, next))
            return false;
        } else if (!emit_array(in, sp, slot(locals, read.array_local),
                               read.index_is_const ? nullptr
                                                   : slot(locals, read.index),
                               slot(stack, sp), next,
                               read.index_is_const
                                   ? b.getInt32(program.const_i[read.index])
                                   : nullptr))
          return false;
        continue;
      }
      case Op::StoreLocal:
        if (in.imm < 0 || in.imm >= program.local_count)
          return false;
        copy(slot(locals, in.imm), slot(stack, sp - 1));
        break;
      case Op::Dup:
        copy(slot(stack, sp), slot(stack, sp - 1));
        break;
      case Op::Pop:
        break;
      case Op::GetMember:
      case Op::PushLocalMember: {
        const bool local_member = in.op == Op::PushLocalMember;
        if (local_member && (in.imm < 0 || in.imm >= program.local_count))
          return false;
        auto *src = local_member ? slot(locals, in.imm) : slot(stack, sp - 1);
        const int packed = local_member ? int(in.a) : in.imm;
        const int count = swizzle_count(packed);
        llvm::Value *values[4]{};
        for (int i = 0; i < count; i++)
          values[i] = axis(src, swizzle_axis(packed, i));
        store_value(slot(stack, local_member ? sp : sp - 1),
                    count == 1   ? Type::Float
                    : count == 2 ? Type::Vector2
                    : count == 3 ? Type::Vector
                                 : Type::Vector4,
                    values[0], values[1], values[2], nullptr, values[3]);
        break;
      }
      case Op::MakeVec: {
        llvm::Value *values[4]{};
        for (int i = 0; i < in.imm; i++)
          values[i] = as_float(slot(stack, sp - in.imm + i));
        if (in.imm == 1)
          values[1] = values[2] = values[0];
        store_value(slot(stack, sp - in.imm),
                    in.imm == 2   ? Type::Vector2
                    : in.imm == 4 ? Type::Vector4
                                  : Type::Vector,
                    values[0], values[1], values[2], nullptr, values[3]);
        break;
      }
      case Op::SqrtF:
      case Op::LogF:
      case Op::AbsF: {
        auto *input = as_float(slot(stack, sp - 1));
        auto *result =
            in.op == Op::SqrtF  ? sqrt_clamped(input)
            : in.op == Op::AbsF ? b.CreateUnaryIntrinsic(llvm::Intrinsic::fabs, input)
                                : call_pointer(jit_scalar_math_address(in.op), f32, {f32},
                                               {input});
        store_value(slot(stack, sp - 1), Type::Float, result);
        break;
      }
      case Op::LengthV3:
      case Op::DotV3:
      case Op::CrossV3: {
        const bool unary = in.op == Op::LengthV3;
        auto *src_a = slot(stack, sp - (unary ? 1 : 2));
        auto *src_b = unary ? src_a : slot(stack, sp - 1);
        llvm::Value *a[3], *v[3];
        for (int i = 0; i < 3; i++) {
          a[i] = as_vec_component(src_a, i);
          v[i] = as_vec_component(src_b, i);
        }
        if (in.op == Op::CrossV3) {
          llvm::Value *result[3];
          for (int i = 0; i < 3; i++) {
            const int j = (i + 1) % 3, k = (i + 2) % 3;
            result[i] = b.CreateFSub(b.CreateFMul(a[j], v[k]),
                                     b.CreateFMul(a[k], v[j]));
          }
          store_value(src_a, Type::Vector, result[0], result[1], result[2]);
        } else {
          auto *result = b.CreateFAdd(
              b.CreateFAdd(b.CreateFMul(a[0], v[0]), b.CreateFMul(a[1], v[1])),
              b.CreateFMul(a[2], v[2]));
          if (unary)
            result = sqrt_clamped(result);
          store_value(src_a, Type::Float, result);
        }
        break;
      }
      case Op::IncLocal:
      case Op::DecLocal: {
        if (in.imm < 0 || in.imm >= program.local_count)
          return false;
        auto *dst = slot(locals, in.imm);
        auto *value = b.CreateAdd(load(dst, offsetof(Value, i), i32),
                                  b.getInt32(in.op == Op::IncLocal ? 1 : -1));
        b.CreateStore(b.getInt8(uint8_t(Type::Int)),
                      offset(dst, offsetof(Value, type)));
        b.CreateStore(value, offset(dst, offsetof(Value, i)));
        b.CreateStore(b.CreateSIToFP(value, f32),
                      offset(dst, offsetof(Value, v)));
        break;
      }
      case Op::Lt:
      case Op::Le:
      case Op::Gt:
      case Op::Ge:
      case Op::Eq:
      case Op::Ne:
      case Op::And:
      case Op::Or:
      case Op::Not: {
        auto *dst = slot(stack, sp - (in.op == Op::Not ? 1 : 2));
        llvm::Value *result;
        if (in.op == Op::Not)
          result = b.CreateNot(truth(dst));
        else if (in.op == Op::And || in.op == Op::Or) {
          auto *x = truth(dst), *y = truth(slot(stack, sp - 1));
          result = in.op == Op::And ? b.CreateAnd(x, y) : b.CreateOr(x, y);
        } else {
          /* The language compares as_float(), even for two integers. Do not
           * replace this with integer comparison: large ints round to float.
           * != and truth tests must also remain true for NaN. */
          auto *x = as_float(dst), *y = as_float(slot(stack, sp - 1));
          const auto predicate = in.op == Op::Lt   ? llvm::CmpInst::FCMP_OLT
                                 : in.op == Op::Le ? llvm::CmpInst::FCMP_OLE
                                 : in.op == Op::Gt ? llvm::CmpInst::FCMP_OGT
                                 : in.op == Op::Ge ? llvm::CmpInst::FCMP_OGE
                                 : in.op == Op::Eq ? llvm::CmpInst::FCMP_OEQ
                                                   : llvm::CmpInst::FCMP_UNE;
          result = b.CreateFCmp(predicate, x, y);
        }
        auto *integer = b.CreateZExt(result, i32);
        store_value(dst, Type::Bool, b.CreateUIToFP(integer, f32), nullptr,
                    nullptr, integer);
        break;
      }
      case Op::Add:
      case Op::Sub:
      case Op::Mul:
      case Op::Div:
      case Op::Mod: {
        auto *a = slot(stack, sp - 2), *v = slot(stack, sp - 1);
        auto *integer_pair = b.CreateAnd(
            is_type(load(a, offsetof(Value, type), b.getInt8Ty()), Type::Int),
            is_type(load(v, offsetof(Value, type), b.getInt8Ty()), Type::Int));
        auto *fast = llvm::BasicBlock::Create(context, "integer", function);
        auto *slow = llvm::BasicBlock::Create(context, "generic", function);
        auto *x = load(a, offsetof(Value, i), i32);
        auto *y = load(v, offsetof(Value, i), i32);
        if (in.op == Op::Div || in.op == Op::Mod) {
          /* Keep the existing helper for the signed-overflow corner case;
           * never introduce LLVM sdiv/srem poison into another value path. */
          auto *overflow =
              b.CreateAnd(b.CreateICmpEQ(x, b.getInt32(0x80000000u)),
                          b.CreateICmpEQ(y, b.getInt32(-1)));
          integer_pair = b.CreateAnd(integer_pair, b.CreateNot(overflow));
        }
        b.CreateCondBr(integer_pair, fast, slow);
        b.SetInsertPoint(fast);
        llvm::Value *result;
        if (in.op == Op::Add)
          result = b.CreateAdd(x, y);
        else if (in.op == Op::Sub)
          result = b.CreateSub(x, y);
        else if (in.op == Op::Mul)
          result = b.CreateMul(x, y);
        else {
          auto *nonzero = b.CreateICmpNE(y, b.getInt32(0));
          auto *divisor = b.CreateSelect(nonzero, y, b.getInt32(1));
          auto *quotient = in.op == Op::Div ? b.CreateSDiv(x, divisor)
                                            : b.CreateSRem(x, divisor);
          result = b.CreateSelect(nonzero, quotient, b.getInt32(0));
        }
        if (in.op == Op::Add) {
          /* add_inplace retains the other semantic fields. */
          b.CreateStore(result, offset(a, offsetof(Value, i)));
          b.CreateStore(b.CreateSIToFP(result, f32),
                        offset(a, offsetof(Value, v)));
        } else
          store_value(a, Type::Int, b.CreateSIToFP(result, f32), nullptr,
                      nullptr, result);
        b.CreateBr(next);
        b.SetInsertPoint(slow);
        if (in.op != Op::Mod)
          emit_numeric(in, a, v, next);
        if (!emit_helper(in, sp, next))
          return false;
        continue;
      }
      case Op::SampleElem: {
        const int sample_count = in.a & 0x1f;
        const int sample_index_arg = (in.a >> 5) - 1;
        const bool explicit_index =
            sample_index_arg >= 0 && sample_index_arg < sample_count;
        auto *source_index =
            explicit_index ? slot(stack, sp - sample_count + sample_index_arg)
                           : nullptr;
        auto *direct_sample =
            llvm::BasicBlock::Create(context, "sample_direct", function);
        auto *bound_sample =
            llvm::BasicBlock::Create(context, "sample_bound", function);
        auto *fallback_sample =
            llvm::BasicBlock::Create(context, "sample_fallback", function);
        llvm::Value *can_sample = b.CreateICmpUGT(
            load(frame, offsetof(JitFrame, static_samples_num), i32),
            b.getInt32(in.imm));
        if (explicit_index) {
          auto *tag = load(source_index, offsetof(Value, type), b.getInt8Ty());
          can_sample =
              b.CreateAnd(can_sample, b.CreateOr(is_type(tag, Type::Int),
                                                 is_type(tag, Type::Bool)));
        }
        b.CreateCondBr(can_sample, bound_sample, fallback_sample);
        b.SetInsertPoint(bound_sample);
        auto *samples = load(frame, offsetof(JitFrame, static_samples), ptr);
        auto *sample_attr = offset(samples, sizeof(AttrRT) * size_t(in.imm));
        auto *sample_tag =
            load(sample_attr, offsetof(AttrRT, type), b.getInt8Ty());
        auto *sample_index =
            explicit_index ? load(source_index, offsetof(Value, i), i32)
                           : load(load(frame, offsetof(JitFrame, env), ptr),
                                  offsetof(VMEnv, index), i32);
        auto *in_bounds = b.CreateICmpULT(
            sample_index, load(sample_attr, offsetof(AttrRT, size), i32));
        b.CreateCondBr(in_bounds, direct_sample, fallback_sample);
        b.SetInsertPoint(direct_sample);
        auto *switch_type = b.CreateSwitch(sample_tag, fallback_sample, 4);
        for (const Type type :
             {Type::Float, Type::Int, Type::Bool, Type::Vector}) {
          auto *typed =
              llvm::BasicBlock::Create(context, "sample_typed", function);
          auto *loaded =
              llvm::BasicBlock::Create(context, "sample_loaded", function);
          switch_type->addCase(b.getInt8(uint8_t(type)), typed);
          b.SetInsertPoint(typed);
          const size_t writable = type == Type::Float  ? offsetof(AttrRT, wf)
                                  : type == Type::Int  ? offsetof(AttrRT, wi)
                                  : type == Type::Bool ? offsetof(AttrRT, wb)
                                                       : offsetof(AttrRT, wv);
          const size_t readable = type == Type::Float  ? offsetof(AttrRT, rf)
                                  : type == Type::Int  ? offsetof(AttrRT, ri)
                                  : type == Type::Bool ? offsetof(AttrRT, rb)
                                                       : offsetof(AttrRT, rv);
          auto *write_data = load(sample_attr, writable, ptr);
          auto *data = b.CreateSelect(b.CreateIsNotNull(write_data), write_data,
                                      load(sample_attr, readable, ptr));
          b.CreateCondBr(b.CreateIsNotNull(data), loaded, fallback_sample);
          b.SetInsertPoint(loaded);
          const int stride = type == Type::Vector ? 12
                             : type == Type::Bool ? 1
                                                  : 4;
          auto *item = b.CreateGEP(
              b.getInt8Ty(), data,
              b.CreateMul(b.CreateSExt(sample_index, b.getInt64Ty()),
                          b.getInt64(stride)));
          auto *dst = slot(stack, sp - sample_count);
          if (type == Type::Int || type == Type::Bool) {
            auto *integer =
                type == Type::Int
                    ? b.CreateLoad(i32, item)
                    : b.CreateZExt(b.CreateLoad(b.getInt8Ty(), item), i32);
            store_value(dst, type, b.CreateSIToFP(integer, f32), nullptr,
                        nullptr, integer);
          } else {
            llvm::Value *x = b.CreateLoad(f32, item), *y = nullptr,
                        *z = nullptr;
            if (type == Type::Vector) {
              y = b.CreateLoad(f32, offset(item, 4));
              z = b.CreateLoad(f32, offset(item, 8));
            }
            store_value(dst, type, x, y, z);
          }
          b.CreateBr(next);
        }
        if (native_pools) {
          /* A packed array sample (`rarr`, see jit_sample()): Int, Float and Float3 arrays are
           * copied like packed_to_value(), other kinds and continued arrays take the helper. */
          using bke::WrangleArrayKind;
          using bke::WrangleArrayValue;
          auto *packed = llvm::BasicBlock::Create(context, "sample_packed", function);
          auto *head_block = llvm::BasicBlock::Create(context, "sample_packed_head", function);
          switch_type->addCase(b.getInt8(uint8_t(Type::Void)), packed);
          b.SetInsertPoint(packed);
          auto *heads = load(sample_attr, offsetof(AttrRT, rarr), ptr);
          b.CreateCondBr(b.CreateIsNotNull(heads), head_block, fallback_sample);
          b.SetInsertPoint(head_block);
          auto *head = b.CreateGEP(b.getInt8Ty(), heads,
                                   b.CreateMul(b.CreateSExt(sample_index, b.getInt64Ty()),
                                               b.getInt64(sizeof(WrangleArrayValue))));
          auto *whole = llvm::BasicBlock::Create(context, "sample_packed_whole", function);
          b.CreateCondBr(
              b.CreateICmpNE(load(head, offsetof(WrangleArrayValue, truncated), b.getInt8Ty()),
                             b.getInt8(WrangleArrayValue::continued)),
              whole, fallback_sample);
          b.SetInsertPoint(whole);
          auto *switch_kind = b.CreateSwitch(
              load(head, offsetof(WrangleArrayValue, kind), b.getInt8Ty()), fallback_sample, 3);
          for (const int kind : {0, 1, 2}) {
            const WrangleArrayKind stored = kind == 0   ? WrangleArrayKind::Int
                                            : kind == 1 ? WrangleArrayKind::Float
                                                        : WrangleArrayKind::Float3;
            auto *copy_block = llvm::BasicBlock::Create(context, "sample_packed_copy", function);
            switch_kind->addCase(b.getInt8(uint8_t(stored)), copy_block);
            b.SetInsertPoint(copy_block);
            copy_packed(head, kind, slot(stack, sp - sample_count), fallback_sample, next);
          }
        }
        b.SetInsertPoint(fallback_sample);
        if (in.imm >= 0 && in.imm < program.element_samples.size()) {
          auto *flags =
              load(frame, offsetof(JitFrame, sample_may_allocate), ptr);
          auto *allocates = b.CreateICmpNE(
              load(flags, size_t(in.imm), b.getInt8Ty()), b.getInt8(0));
          auto *invalidate =
              llvm::BasicBlock::Create(context, "sample_invalidate", function);
          auto *sample = llvm::BasicBlock::Create(context, "sample", function);
          b.CreateCondBr(allocates, invalidate, sample);
          b.SetInsertPoint(invalidate);
          invalidate_array_views();
          b.CreateBr(sample);
          b.SetInsertPoint(sample);
        } else
          invalidate_array_views();
        const int count = in.a & 0x1f;
        const int index_arg = (in.a >> 5) - 1;
        llvm::Value *index = llvm::ConstantPointerNull::get(b.getPtrTy());
        if (index_arg >= 0 && index_arg < count) {
          index = slot(bridge_stack, sp - count + index_arg);
          if (private_storage)
            copy(index, slot(stack, sp - count + index_arg));
        }
        auto *dst = slot(bridge_stack, sp - count);
        emit_count(255);
        auto *ok = call_pointer(reinterpret_cast<void *>(&jit_sample),
                                b.getInt1Ty(), {ptr, i32, ptr, ptr},
                                {frame, b.getInt32(in.imm), index, dst});
        helper_ops++;
        auto *success =
            llvm::BasicBlock::Create(context, "sample_ok", function);
        b.CreateCondBr(ok, success, failed);
        b.SetInsertPoint(success);
        if (private_storage)
          copy(slot(stack, sp - count), dst);
        break;
      }
      case Op::Call: {
        int uniform_index = -1;
        for (const int index : uniform_parms.index_range()) {
          if (uniform_parms[index].pc == pc) {
            uniform_index = index;
            break;
          }
        }
        if (uniform_index >= 0) {
          copy(slot(stack, sp - 1), uniform_slots[uniform_index]);
          break;
        }
        if (emit_empty_array(in, sp, next) || emit_append(in, sp, next) ||
            emit_face_points(in, sp, next) ||
            emit_vector_builtin(in, sp, next)) {
          if (!emit_helper(in, sp, next))
            return false;
          continue;
        }
        const Builtin builtin = Builtin(in.imm);
        void *address = jit_scalar_builtin_address(builtin);
        const bool minmax = builtin == Builtin::Min || builtin == Builtin::Max;
        const bool binary =
            minmax || builtin == Builtin::Atan2 || builtin == Builtin::Pow;
        const int count = binary ? 2 : 1;
        if ((!address && !minmax) || in.a != count) {
          native_ops--;
          if (!emit_helper(in, sp, next))
            return false;
          continue;
        }
        auto *dst = slot(stack, sp - count);
        auto *rhs = binary ? slot(stack, sp - 1) : dst;
        auto *tag_a = load(dst, offsetof(Value, type), b.getInt8Ty());
        auto *tag_b = load(rhs, offsetof(Value, type), b.getInt8Ty());
        auto scalar = [&](llvm::Value *tag) {
          return b.CreateOr(
              is_type(tag, Type::Float),
              b.CreateOr(is_type(tag, Type::Int), is_type(tag, Type::Bool)));
        };
        auto *fast =
            llvm::BasicBlock::Create(context, "scalar_builtin", function);
        auto *slow =
            llvm::BasicBlock::Create(context, "generic_builtin", function);
        b.CreateCondBr(b.CreateAnd(scalar(tag_a), scalar(tag_b)), fast, slow);
        b.SetInsertPoint(fast);
        auto *x = as_float(dst), *y = as_float(rhs);
        if (minmax) {
          auto *ints =
              b.CreateAnd(is_type(tag_a, Type::Int), is_type(tag_b, Type::Int));
          auto *ix = load(dst, offsetof(Value, i), i32),
               *iy = load(rhs, offsetof(Value, i), i32);
          auto *ir =
              b.CreateSelect(builtin == Builtin::Min ? b.CreateICmpSLT(iy, ix)
                                                     : b.CreateICmpSLT(ix, iy),
                             iy, ix);
          auto *fr =
              b.CreateSelect(builtin == Builtin::Min ? b.CreateFCmpOLT(y, x)
                                                     : b.CreateFCmpOLT(x, y),
                             y, x);
          store_value(dst, Type::Float,
                      b.CreateSelect(ints, b.CreateSIToFP(ir, f32), fr),
                      nullptr, nullptr,
                      b.CreateSelect(ints, ir, b.getInt32(0)));
          b.CreateStore(b.CreateSelect(ints, b.getInt8(uint8_t(Type::Int)),
                                       b.getInt8(uint8_t(Type::Float))),
                        offset(dst, offsetof(Value, type)));
        } else
          store_value(dst, Type::Float,
                      call_pointer(address, f32, {f32, f32}, {x, y}));
        b.CreateBr(next);
        b.SetInsertPoint(slow);
        if (!emit_helper(in, sp, next))
          return false;
        continue;
      }
      case Op::Neg: {
        auto *dst = slot(stack, sp - 1);
        auto *tag = load(dst, offsetof(Value, type), b.getInt8Ty());
        auto *matrix = b.CreateOr(is_type(tag, Type::Matrix),
                                  b.CreateOr(is_type(tag, Type::Matrix2),
                                             is_type(tag, Type::Matrix3)));
        auto *fallback =
            llvm::BasicBlock::Create(context, "neg_matrix", function);
        auto *direct =
            llvm::BasicBlock::Create(context, "neg_native", function);
        auto *integer =
            llvm::BasicBlock::Create(context, "neg_integer", function);
        auto *other = llvm::BasicBlock::Create(context, "neg_other", function);
        b.CreateCondBr(matrix, fallback, direct);
        b.SetInsertPoint(direct);
        b.CreateCondBr(is_type(tag, Type::Int), integer, other);
        b.SetInsertPoint(integer);
        auto *iv = b.CreateNeg(load(dst, offsetof(Value, i), i32));
        store_value(dst, Type::Int, b.CreateSIToFP(iv, f32), nullptr, nullptr,
                    iv);
        b.CreateBr(next);
        b.SetInsertPoint(other);
        auto *scalar =
            llvm::BasicBlock::Create(context, "neg_scalar", function);
        auto *switch_type = b.CreateSwitch(tag, scalar, 3);
        for (const Type type : {Type::Vector2, Type::Vector, Type::Vector4}) {
          auto *vector =
              llvm::BasicBlock::Create(context, "neg_vector", function);
          switch_type->addCase(b.getInt8(uint8_t(type)), vector);
          b.SetInsertPoint(vector);
          auto *x = b.CreateFNeg(component(dst, 0)),
               *y = b.CreateFNeg(component(dst, 1));
          llvm::Value *z =
              type == Type::Vector2 ? nullptr : b.CreateFNeg(component(dst, 2));
          llvm::Value *w =
              type == Type::Vector4 ? b.CreateFNeg(component(dst, 3)) : nullptr;
          store_value(dst, type, x, y, z, nullptr, w);
          b.CreateBr(next);
        }
        b.SetInsertPoint(scalar);
        store_value(dst, Type::Float, b.CreateFNeg(as_float(dst)));
        b.CreateBr(next);
        b.SetInsertPoint(fallback);
        if (!emit_helper(in, sp, next))
          return false;
        continue;
      }
      case Op::AddFF:
      case Op::SubFF:
      case Op::MulFF:
      case Op::DivFF:
      case Op::AddVV:
      case Op::SubVV:
      case Op::MulFV:
      case Op::MulVF:
      case Op::MulVV:
      case Op::DivVF:
      case Op::DivVV: {
        auto *a = slot(stack, sp - 2), *v = slot(stack, sp - 1);
        const bool scalar =
            ELEM(in.op, Op::AddFF, Op::SubFF, Op::MulFF, Op::DivFF);
        llvm::Value *result[3]{};
        /* Match the VM's reciprocal-then-multiply vector/scalar division
         * exactly. */
        llvm::Value *inv = nullptr;
        if (in.op == Op::DivVF) {
          auto *den = component(v, 0);
          inv = b.CreateSelect(
              b.CreateFCmpUNE(den, llvm::ConstantFP::get(f32, 0)),
              b.CreateFDiv(llvm::ConstantFP::get(f32, 1), den),
              llvm::ConstantFP::get(f32, 0));
        }
        for (int axis = 0; axis < (scalar ? 1 : 3); axis++) {
          auto *x = component(a, in.op == Op::MulFV ? 0 : axis);
          auto *y = component(v, ELEM(in.op, Op::MulVF, Op::DivVF) ? 0 : axis);
          if (ELEM(in.op, Op::AddFF, Op::AddVV))
            result[axis] = b.CreateFAdd(x, y);
          else if (ELEM(in.op, Op::SubFF, Op::SubVV))
            result[axis] = b.CreateFSub(x, y);
          else if (in.op == Op::DivFF)
            result[axis] = b.CreateSelect(
                b.CreateFCmpUNE(y, llvm::ConstantFP::get(f32, 0)),
                b.CreateFDiv(x, y), llvm::ConstantFP::get(f32, 0));
          else if (in.op == Op::DivVV)
            result[axis] = b.CreateFDiv(x, y);
          else
            result[axis] = b.CreateFMul(x, inv ? inv : y);
        }
        store_value(a, scalar ? Type::Float : Type::Vector, result[0],
                    result[1], result[2]);
        break;
      }
      case Op::Jmp:
      case Op::JmpIfFalse:
      case Op::JmpIfTrue: {
        /* Count every branch exactly, but poll cooperative cancellation every
         * 256 branches. Keeping ordinary branch bookkeeping native avoids a
         * C++/virtual callback at each loop test. Entry/periodic PC checks
         * stay.
         */
        auto *counter = branch_counter;
        auto *jumps = b.CreateLoad(b.getInt64Ty(), counter);
        auto *limit =
            llvm::BasicBlock::Create(context, "branch_limit", function);
        auto *counted =
            llvm::BasicBlock::Create(context, "branch_count", function);
        b.CreateCondBr(b.CreateICmpUGE(jumps, b.getInt64(100000000)), limit,
                       counted);
        b.SetInsertPoint(limit);
        b.CreateStore(jumps, offset(frame, offsetof(JitFrame, jumps)));
        call_pointer(reinterpret_cast<void *>(&jit_branch_guard), b.getInt1Ty(),
                     {ptr}, {frame});
        b.CreateBr(failed);
        b.SetInsertPoint(counted);
        auto *incremented = b.CreateAdd(jumps, b.getInt64(1));
        b.CreateStore(incremented, counter);
        auto *poll = llvm::BasicBlock::Create(context, "cancel_poll", function);
        auto *branch = llvm::BasicBlock::Create(context, "branch", function);
        b.CreateCondBr(b.CreateICmpEQ(b.CreateAnd(incremented, b.getInt64(255)),
                                      b.getInt64(0)),
                       poll, branch);
        b.SetInsertPoint(poll);
        auto *allowed =
            call_pointer(reinterpret_cast<void *>(&jit_cancel_guard),
                         b.getInt1Ty(), {ptr}, {frame});
        b.CreateCondBr(allowed, branch, failed);
        b.SetInsertPoint(branch);
        if (in.op == Op::Jmp)
          b.CreateBr(blocks[in.imm]);
        else {
          auto *condition = truth(slot(stack, sp - 1));
          b.CreateCondBr(condition,
                         in.op == Op::JmpIfTrue ? blocks[in.imm] : next,
                         in.op == Op::JmpIfTrue ? next : blocks[in.imm]);
        }
        continue;
      }
      default: {
        native_ops--;
        if (!emit_helper(in, sp, next))
          return false;
        continue;
      }
      }
      b.CreateBr(next);
    }
    b.SetInsertPoint(failed);
    /* A limit error has already incremented the public counter once. */
    auto *private_jumps = b.CreateLoad(b.getInt64Ty(), branch_counter);
    auto *public_jumps = load(frame, offsetof(JitFrame, jumps), b.getInt64Ty());
    b.CreateStore(b.CreateSelect(b.CreateICmpUGT(public_jumps, private_jumps),
                                 public_jumps, private_jumps),
                  offset(frame, offsetof(JitFrame, jumps)));
    b.CreateRet(b.getFalse());
    return !llvm::verifyFunction(*function);
  }
};
} // namespace

std::shared_ptr<const JitKernel> jit_kernel_get(const Program &program,
                                              const Span<JitUniformParm> uniform_parms) {
  if (std::getenv("BLENDER_VEX_DISABLE_JIT"))
    return nullptr;
  static Backend backend;
  std::lock_guard lock(backend.mutex);
  if (!backend.engine)
    return nullptr;
  std::string group = cache_key(program);
  auto append_group = [&](const auto &value) {
    group.append(reinterpret_cast<const char *>(&value), sizeof(value));
  };
  const int uniform_count = int(uniform_parms.size());
  append_group(uniform_count);
  for (const JitUniformParm &parm : uniform_parms) {
    append_group(parm.pc);
    append_group(parm.value.type);
  }
  bool specialize = !uniform_parms.is_empty() &&
                    !std::getenv("BLENDER_VEX_JIT_NO_PARAMETER_SPECIALIZATION");
  auto make_key = [&](const bool use_constants) {
    std::string key = group;
    key.push_back(use_constants ? 1 : 0);
    if (use_constants) {
      for (const JitUniformParm &parm : uniform_parms) {
        /* Semantic fields only: preserve integer width, negative zero and float bits. */
        key.append(reinterpret_cast<const char *>(&parm.value.i), sizeof(parm.value.i));
        key.append(reinterpret_cast<const char *>(&parm.value.v), sizeof(parm.value.v));
      }
    }
    return key;
  };
  std::string key = make_key(specialize);
  if (auto found = backend.cache.find(key); found != backend.cache.end()) {
    found->second.last_used = ++backend.use_tick;
    if (uniform_count && std::getenv("BLENDER_VEX_JIT_LOG")) {
      std::fprintf(stderr, "VEX_JIT_PARMS cache=hit uniforms=%d specialized=%d\n",
                   uniform_count, int(found->second.parameter_specialized));
    }
    return found->second.kernel;
  }
  /* Continuous sliders/animation must not trigger unlimited compilation. Keep at most
   * eight constant variants per program shape, then share a native uniform-load kernel.
   * Existing constant variants remain reusable when the user switches back. */
  int variants = 0;
  for (const auto &entry : backend.cache) {
    variants += entry.second.parameter_specialized && entry.second.parameter_group == group;
  }
  if (specialize && variants >= 8) {
    specialize = false;
    key = make_key(false);
    if (auto found = backend.cache.find(key); found != backend.cache.end()) {
      found->second.last_used = ++backend.use_tick;
      if (std::getenv("BLENDER_VEX_JIT_LOG")) {
        std::fprintf(stderr, "VEX_JIT_PARMS cache=hit uniforms=%d specialized=0\n", uniform_count);
      }
      return found->second.kernel;
    }
  }
  const auto start = std::chrono::steady_clock::now();
  std::shared_ptr<JitKernel> kernel;
  std::vector<int> depth;
  int capacity;
  if (analyze(program, depth, capacity)) {
    auto context = std::make_unique<llvm::LLVMContext>();
    const std::string name = "vex_jit_" + std::to_string(++backend.serial);
    auto module = std::make_unique<llvm::Module>(name, *context);
    module->setDataLayout(backend.engine->getDataLayout());
    module->setTargetTriple(backend.engine->getTargetTriple());
    const bool private_storage =
        std::getenv("BLENDER_VEX_JIT_LEGACY_STORAGE") == nullptr;
    Lowering lowering(program, *context, *module, depth, name, capacity,
                      private_storage, uniform_parms, specialize);
    if (lowering.emit()) {
      /* No fast-math flags or floating-point contraction are enabled. */
      llvm::LoopAnalysisManager lam;
      llvm::FunctionAnalysisManager fam;
      llvm::CGSCCAnalysisManager cgam;
      llvm::ModuleAnalysisManager mam;
      llvm::PassBuilder passes;
      passes.registerModuleAnalyses(mam);
      passes.registerCGSCCAnalyses(cgam);
      passes.registerFunctionAnalyses(fam);
      passes.registerLoopAnalyses(lam);
      passes.crossRegisterProxies(lam, fam, cgam, mam);
      auto pipeline =
          passes.buildPerModuleDefaultPipeline(llvm::OptimizationLevel::O2);
      pipeline.run(*module, mam);
      /* BLENDER_VEX_JIT_DUMP=<directory>: the optimized IR of every kernel. */
      if (const char *dump = std::getenv("BLENDER_VEX_JIT_DUMP")) {
        std::error_code error;
        llvm::raw_fd_ostream file(std::string(dump) + "/" + name + ".ll", error);
        if (!error)
          module->print(file, nullptr);
      }
      if (std::getenv("BLENDER_VEX_JIT_LOG") ||
          std::getenv("BLENDER_VEX_PROFILE")) {
        int allocas = 0, phis = 0, loads = 0, stores = 0, branches = 0;
        for (const auto &fn : *module)
          for (const auto &block : fn)
            for (const auto &in : block) {
              allocas += in.getOpcode() == llvm::Instruction::Alloca;
              phis += in.getOpcode() == llvm::Instruction::PHI;
              loads += in.getOpcode() == llvm::Instruction::Load;
              stores += in.getOpcode() == llvm::Instruction::Store;
              if (const auto *branch = llvm::dyn_cast<llvm::BranchInst>(&in)) {
                branches += branch->isConditional();
              }
            }
        std::fprintf(stderr,
                     "VEX_JIT_STORAGE private=%d allocas=%d phis=%d loads=%d "
                     "stores=%d conditional=%d uniforms=%d specialized=%d\n",
                     int(private_storage), allocas, phis, loads, stores, branches,
                     uniform_count, int(specialize));
      }
      auto lease = std::make_shared<ModuleLease>();
      lease->tracker =
          backend.engine->getMainJITDylib().createResourceTracker();
      if (auto error = backend.engine->addIRModule(
              lease->tracker, llvm::orc::ThreadSafeModule(
                                  std::move(module), std::move(context)))) {
        llvm::consumeError(std::move(error));
      } else {
        auto address = backend.engine->lookup(name);
        if (address) {
          kernel = std::make_shared<JitKernel>();
          kernel->entry = address->toPtr<JitKernel::Entry>();
          kernel->stack_capacity = capacity;
          kernel->native_ops = lowering.native_ops;
          kernel->helper_ops = lowering.helper_ops;
          kernel->private_storage = private_storage;
          kernel->uniform_parms = uniform_count;
          kernel->specialized_parms = specialize;
          kernel->lifetime = lease;
        } else
          llvm::consumeError(address.takeError());
      }
    }
  }
  /* Cache both successful compilation and preflight rejection. Active callers
   * keep their module alive while old cache entries are removed. */
  if (backend.cache.size() >= 128) {
    auto oldest =
        std::min_element(backend.cache.begin(), backend.cache.end(),
                         [](const auto &a, const auto &b) {
                           return a.second.last_used < b.second.last_used;
                         });
    backend.cache.erase(oldest);
  }
  backend.cache.emplace(key, Backend::CacheEntry{kernel, ++backend.use_tick, group, specialize});
  if (uniform_count && std::getenv("BLENDER_VEX_JIT_LOG")) {
    std::fprintf(stderr, "VEX_JIT_PARMS cache=build uniforms=%d specialized=%d variants=%d\n",
                 uniform_count, int(specialize), variants);
  }
  if (std::getenv("BLENDER_VEX_PROFILE") ||
      std::getenv("BLENDER_VEX_JIT_LOG")) {
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    std::fprintf(stderr,
                 "VEX_JIT status=%s backend=LLVM ops=%lld native=%d helpers=%d "
                 "compile_ms=%.3f\n",
                 kernel ? "compiled" : "fallback",
                 static_cast<long long>(program.code.size()),
                 kernel ? kernel->native_ops : 0,
                 kernel ? kernel->helper_ops : 0, ms);
  }
  return kernel;
}
} // namespace blender::nodes::vex
