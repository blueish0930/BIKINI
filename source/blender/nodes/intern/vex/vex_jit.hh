/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include "vex_program.hh"
#include <atomic>
#include <cstdint>
#include <memory>

namespace blender::nodes::vex {

/* Cached only until a helper may allocate, grow, or replace an array pool.
 * Views are worker-local and never survive an element evaluation. */
struct JitArrayView {
  const void *data = nullptr;
  int size = 0;
  int handle = -1;
  int type = 0;
  uint64_t generation = 0;
};
struct JitArrayCache {
  uint64_t generation = 1;
  JitArrayView views[64];
};

/* The existing VM helpers own all array, string, geometry and function
 * semantics. Native code operates on the same values and calls those helpers
 * directly. */
struct JitFrame {
  const Program *program;
  VMEnv *env;
  Value *stack;
  Value *locals;
  std::string *error;
  int local_count;
  int stack_capacity;
  int64_t jumps = 0;
  Value return_value;
  AttrRT *attrs = nullptr;
  int attrs_num = 0;
  JitArrayCache *array_cache = nullptr;
  const uint8_t *sample_may_allocate = nullptr;
  const AttrRT *static_samples = nullptr;
  int static_samples_num = 0;
  const Value *uniform_parms = nullptr;
};

struct JitUniformParm {
  int pc;
  Value value;
};

struct JitKernel {
  using Entry = bool (*)(JitFrame *);
  Entry entry = nullptr;
  int stack_capacity = 0;
  int native_ops = 0;
  int helper_ops = 0;
  bool private_storage = false;
  int uniform_parms = 0;
  bool specialized_parms = false;
  std::shared_ptr<void> lifetime;
  /* Measured CPU nanoseconds per element of earlier runs (0 before the first). Sizes the
   * parallel tasks of later runs; it never changes what a run computes. */
  mutable std::atomic<float> ns_per_element{0.0f};
};

/* A null kernel means preflight rejected the program; no user code has run. */
std::shared_ptr<const JitKernel> jit_kernel_get(const Program &program,
                                              Span<JitUniformParm> uniform_parms = {});
void *jit_operation_address(Op op);
void *jit_scalar_math_address(Op op);
/* Static samples already bind geometry/name/domain. Only the selected index
 * remains a runtime input; nullptr uses the current element index. */
bool jit_sample(JitFrame *frame, int slot, const Value *index, Value *result);
void *jit_scalar_builtin_address(Builtin builtin);
bool jit_array_view(int type, int handle, JitArrayView *view);
bool jit_branch_guard(JitFrame *frame);
bool jit_cancel_guard(JitFrame *frame);

} // namespace blender::nodes::vex
