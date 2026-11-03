# Geometry Wrangle CPU JIT

The existing VEX frontend, source syntax, attribute prefixes, input sockets, and
saved node storage are unchanged. Eligible CPU programs can now execute through
LLVM ORC native code. No `.blend` migration or rewiring is required.

The initial JIT lowers main-program branches, loops, constant/local loads and
statically typed float/vector arithmetic to native instructions. Other operations
call the same implementations used by the scalar VM, through
`intern/vex/vex_lane_ops.inc`. Array/string storage, geometry callbacks, parameter
lookup, user-function argument rules and recursion retain their existing runtime
semantics. User-function bodies currently retain the existing VM call mechanism;
this is not a claim that every function has been independently native-compiled.

The subsequent optimization uses private local/operand storage, with explicit
semantic-field transfers at runtime helper boundaries. These private addresses
never escape to callbacks, so LLVM can promote values and loop state to SSA.
Unassigned locals retain the original zero/Float initialization. Fused gathers
transfer their input and output locals, including outputs a direct gather may
leave unchanged. The native-code cache key includes the captured local-index and
gather metadata; equal opcode streams alone do not identify those programs.

Scalar/swizzle reads, vector construction, dot/cross/length and the existing
sqrt/log/abs scalar operations have direct lowering or a compact scalar ABI.
Scalar math wrappers reuse the established clamps/functions, and dot products
retain their original component order. Common packed numeric-array reads now
decode directly into owned thread-local buffers, preserving capacity and avoiding
a temporary vector plus a second copy. Flavor conversion, mutable-array
independence, overflow chunks and serialization formats retain their semantics.

No fast-math or FP contraction flags are enabled. In particular vector/scalar
division preserves the existing reciprocal-then-multiply sequence. Existing GPU
and native array execution paths retain priority. The JIT is attempted on the CPU
fallback path when the estimated work is at least 65,536 instruction-elements.
Programs rejected by stack/control-flow preflight use the VM before executing any
user instructions. Errors after native execution starts are returned normally;
partially executed code is never replayed in the VM.

Compiled programs use a bounded LRU cache. Each active evaluation owns its code,
so cache eviction cannot free instructions still running on another worker.
Per-worker VM environments, snapshots, array/string pools, cancellation and loop
limits remain active. Float precision remains the existing single precision.

## Build

Configure with `-DWITH_VEX_LLVM_JIT=ON`. Windows uses the LLVM development bundle
under `${LIBDIR}/llvm`, independently of the existing global `WITH_LLVM` option.
The bundled LLVM tools are not required. The current integration has been built
and exercised on Windows x64 Release with LLVM 22.1.7. Other build configurations
require compatible LLVM libraries and have not been accepted by this validation.

## Runtime controls

- `BLENDER_VEX_DISABLE_JIT=1`: use the previous CPU execution paths.
- `BLENDER_VEX_JIT_LOG=1`: report native compilation, helper counts and compilation
  time, or preflight fallback.
- `BLENDER_VEX_PROFILE=1`: additionally reports `VEX_VM_EXEC path=jit`.
- `BLENDER_VEX_JIT_FORCE=1`: test eligible programs even below the work threshold
  and before the native array path. This is for backend testing.
- `BLENDER_VEX_JIT_LEGACY_STORAGE=1`: diagnostic comparison using external
  operand/local storage; other optimizations remain enabled.

The GPU path is not disabled by these JIT controls. Set `BLENDER_VEX_NO_GPU=1`
when specifically comparing CPU implementations.

## Regression validation

`tests/python/geometry_nodes_wrangle_jit.py` evaluates the source corpus in
`wrangle_jit_cases.json`. Run it with the old executable, the new executable with
JIT disabled, and the new executable with `BLENDER_VEX_JIT_FORCE=1`. It records
coordinates, topology, attributes, diagnostics and exceptions for exact comparison.
The corpus contains existing frontend/function regression snippets and targeted
JIT cases, including loops, grouped declarations/arguments, recursion, 40-argument
functions, vectors, matrices, arrays, strings, geometry reads/writes and snapshots.
It deliberately includes invalid-source cases to check unchanged diagnostics.

Example:

```powershell
$env:BLENDER_VEX_JIT_FORCE = '1'
& 'E:\build_windows_Full_x64_vc17_Release\bin\Release\blender.exe' -b --factory-startup --python-exit-code 1 --python 'E:\Blender_Source\tests\python\geometry_nodes_wrangle_jit.py' -- --output-dir 'E:\Files\TestGeo\WrangleJIT_20261101' --label jit
```

The harness isolates the CPU path. It snapshots matrix rows immediately, before
releasing the evaluated mesh. Unassigned attributes on newly created elements
are not compared as defined output values; their explicitly assigned slots,
positions and connectivity are checked.

Final acceptance reports, same-executable VM/JIT timings, the developable-surface
long-run comparisons and read-only existing-project checks are under
`E:\Files\TestGeo\WrangleJIT_20261101`. Speed measurements must distinguish first
compilation from cached execution and use matching inputs and settings. Enabling
JIT does not remove the remaining generic Value/array/helper or node-graph costs.

The second optimization's source snapshots, 303-case comparison (including
cross-node numeric array roundtrips), 27 paper cases, existing-project checks,
profiles and controlled measurements are under
`E:\Files\TestGeo\WrangleJIT_20261101_Phase2`. A storage-only experiment and the
final combined optimization have separate reports; an IR stack-allocation count
is not itself a runtime speed measurement.

The third optimization emits generic integer add/subtract/multiply/divide/modulo
directly when both runtime tags are Int. Other types retain the shared VM
helpers. Division by zero, signed remainders, and add-in-place's retained fields
keep the existing semantics. Comparisons still convert to float, including two
large integers; boolean tests and unordered NaN inequality are unchanged.
Static geometry samples use a thin bridge carrying only the selected index and
result. Geometry/domain/name binding, snapshot lookup, arrays, conversions and
error handling continue through the existing sample implementation.

Source snapshots, 317-case three-path compatibility checks, paper validation and
interleaved same-input benchmarks for this change are under
`E:\Files\TestGeo\WrangleJIT_20261101_Phase3`. Complete-evaluation timings include
the actual solver graph and any first-use JIT compilation reached by that solve;
they are not isolated kernel or compile-time measurements. Use contemporary
baseline executable timings when reporting improvement, because host load varies.

The fourth optimization lowers common scalar/vector attribute reads and writes,
scalar mathematical builtins, unary negation and bound numeric geometry samples
to native code. Existing conversions and helper fallbacks remain available for
unsupported types, invalid indices and computed/snapshot bindings. Branch limits
still count every branch; cooperative cancellation is polled at entry, periodic
PC checks and every 256 branches instead of crossing the C++/virtual boundary at
every branch.

Numeric Int/Float/Vector array access can reuse worker-local pointer/length views.
Views are invalidated for every new element and before helpers that may allocate,
resize or replace array storage. In-range reads/writes and lengths are lowered
directly, including negative-index wrapping. Growth, lazy ranges, nonnumeric
arrays and unsupported conversions use the original helpers. Cached views hold
no contents and do not change alias semantics. Bound scalar samples do not
invalidate views; array/computed sample paths conservatively do.
`BLENDER_VEX_JIT_DISABLE_ARRAY_VIEWS=1` disables this optimization for diagnostics.

The matching editable developability graph caches immutable topology lookups,
selects a constant energy kernel, preallocates scratch arrays, and uses fixed
scalar/vector history slots internally. Original history arrays are reconstructed
on the final output. The eight L-BFGS pairs retain their order and reductions;
local dot fields feed the existing native sum reduction without an intermediate
attribute-writing Wrangle. These graph optimizations apply to the delivered
graph, while the backend changes also apply to existing graphs.

Fourth-round measurements and checks live under
`E:\Files\TestGeo\WrangleJIT_20261101_Phase4`. The sub-three-second acceptance
target is the user's saved Bunny preset: Energy 0, Method 1, 250 accepted steps,
3301 vertices and 6598 triangles. Reports distinguish first-use compilation from
cached recomputation. Other optimizer/energy/mesh settings have their own timing
results; they are not covered by that performance threshold. Numerical validation
compares all six energies and also reads back every final history-array element.

The fifth optimization adds direct constant-index and fused element-swizzle
loads, and native indexed reads/writes for vector components. Index wrapping,
out-of-range zero reads/unchanged vector writes, scalar swizzle replication and
helper fallbacks retain their previous behavior. The main kernel branch counter
is now private SSA state across helper calls; it is synchronized on return/error
and before invoking the existing instruction-limit error handler. Cancellation
polling frequency and the 100000000-branch limit remain unchanged.

The CPU fallback's uniform-store prepass now rejects array read/write aliases:
hoisting a whole-attribute write ahead of an earlier captured array read can
overwrite the local's snapshot, including differently declared array flavors
sharing one attribute name. Newly added regressions exposed this preexisting
VM/JIT difference; the fallback now agrees with the existing JIT result.

Read-only packed copy-on-write views and two face-index caching layouts were
tested and rejected for the current workload. Their source/timing experiments
are retained under `E:\Files\TestGeo\WrangleJIT_20261101_Phase5`; they are not
enabled in the delivered backend or graph. This round keeps the existing graph
and packed-array ABI. Performance reports distinguish the retained changes from
experimental measurements and continue separating first-use and cached solves.

Bound numeric channel parameters
-------------------------------

Geometry Wrangle CPU JIT recognizes literal-name calls to `chi`, `chf`, `chb`,
`chv`, `chu`, `chq`, `chc` and `chr` when their bound input is a single numeric
value. No source syntax or parameter socket changes are required. The geometry
evaluation snapshots the value once, using the same channel conversion as the
VM, including full-width integers, vector/color/rotation conversions and signed
zero. Missing-name attribute fallback and varying fields are never sampled as
uniform parameters.

The first eight cached parameter combinations per program/binding shape are
compiled with constant values. LLVM can propagate them through locals and remove
unreachable energy branches. Subsequent combinations use a shared native kernel
that loads current parameter snapshots at entry; changes and animation remain
live without an unlimited sequence of parameter-specific compilations. Existing
specializations can still be reused. These entries share the existing global
128-entry LRU kernel cache. A first-use specialization includes compilation time.

Computed names, list/string/matrix/ramp inputs and calls inside helper-executed
user functions retain their existing paths. This change does not add Shader
Wrangle GPU specialization. Nonfinite and unsafe integer-conversion inputs are
left on the runtime path, so examining an unexecuted call cannot introduce a new
invalid conversion. No fast-math or precision changes are enabled.

`BLENDER_VEX_JIT_DISABLE_UNIFORM_PARAMETERS=1` restores helper-based parameter
loads. `BLENDER_VEX_JIT_NO_PARAMETER_SPECIALIZATION=1` retains native uniform
loads while disabling value specialization. With `BLENDER_VEX_JIT_LOG=1`,
`VEX_JIT_PARMS` reports cache build/hit, uniform count and specialized state;
`VEX_JIT_STORAGE` includes the post-optimization conditional-branch count.

`tests/python/geometry_nodes_wrangle_uniform_jit.py` exercises parameter changes,
multiple instances, animation, numeric types, wide integers, signed zero,
missing-name varying fallback, renamed/computed names, list semantics and
unexecuted nonfinite inputs. Run separately with `--label jit`, `vm`, `uniform`
and `helpers`, then compare the result JSONs exactly. The existing full Wrangle
corpus is a separate compatibility check.

Generic arithmetic, samples and scheduling
------------------------------------------

Generic `+ - * /` (the forms the compiler emits when an operand type is only
known at run time, for example an array element) are lowered natively when both
operands are Float/Int/Bool scalars or 3D vectors. The results follow
add_inplace() and numeric_sub/mul/div() exactly: a scalar times a vector is
computed as vector * scalar, only a vector numerator gives a vector quotient,
scalar divisors of zero give zero, and `+` keeps the fields add_inplace keeps.
`vector()` and `set()` of three scalars or one scalar/vector (set of one scalar
stays a 2D vector through the helper) and `normalize()` of a scalar or vector
are native too, with math::normalize()'s 1e-35 threshold and division order.
These helper calls also invalidated the cached array views; avoiding them
removes most of the view re-resolution.

Same-domain samples (`point(0, "P", i)` in a point Wrangle) used to call the
geometry callback on every read. The executor now gives native code the typed
array that callback would read for an in-range index: the Jacobi snapshot of a
written attribute, otherwise the prebound sample (`VMEnv::jit_samples`). Array
attribute samples bind their packed span, and jit_sample() decodes the element
directly, as value_from_varray() does; arrays continued in chunk attributes,
out-of-range indices and other types keep the callback. The interpreter and the
GPU path still use `static_samples` unchanged.

Attributes that only name a sample of another domain (`face(0, "x", f)` in a
point Wrangle) are listed in `Program::attrs` for the GLSL emitter. When no
instruction references them and the program has no GPU/spatial/Jacobi path,
they are no longer bound, which interpolated them to the node's domain on every
evaluation (0.2 ms per attribute on the Bunny).

The VM's thread-local pools are reached through a constant-initialized TLS
pointer instead of a guarded function-local `thread_local`, and the per-element
reset takes it once. The JIT element loop reuses one frame per task and polls
cancellation every 32 elements (kernels still poll every 1024 instructions and
256 branches). Each kernel records its measured CPU time per element; later runs
size parallel tasks to about 40 microseconds, so small kernels on small domains
are no longer dominated by task scheduling. `BLENDER_VEX_GRAIN` still overrides.

Diagnostics: `BLENDER_VEX_JIT_COUNT=1` counts executed helper calls per op or
builtin (also array-view resolves and sample fallbacks) and prints them at exit;
`BLENDER_VEX_JIT_DUMP=<directory>` writes the optimized IR of every kernel.

Measured on the Bunny preset (250 steps, 3301 vertices), same machine,
interleaved runs with a baseline build of the previous sources: 2.30 s to 1.40 s
per cached solve with default settings (VM-only path 6.6 s to 6.2 s). Results
are bit-identical: 36 energy/method/model hashes of positions and attributes,
the 373-case corpus, the 44-case parameter test, and new JIT-versus-VM cases for
mixed-type arithmetic, NaN/inf, vector builtins and same/cross-domain samples.

Array pools read in native code
-------------------------------

Array values are handles into the thread's `Vector<Vector<T>>` pools. Native
code now reads the pool vectors directly (`JitFrame::array_pools`) instead of
caching views that every allocating helper invalidated: `begin_`, `end_` and
`capacity_end_` lead a `Vector`, which jit_array_pools_native() checks once at
runtime on real vectors (without it the view cache is kept). With the pools
readable, the common array helpers run natively when the pool array has room,
and fall back to the helpers unchanged when it does not (a new pool entry, a
reallocation, a continued array, another kind):

- `int a[]; float a[]; vector a[];` take the next pool array (cleared, capacity
  kept), like store_tls_array() of an empty Vector.
- `a[i] = x` past the end zero-fills the gap within the capacity, like
  array_write_index(); `append(a, x)` of one int, float or vector item.
- `i[]@a`, `f[]@a`, `v[]@a` stored as the same kind and not continued are copied
  into a pool array; stores that fit one packed value write it like
  set_ints()/set_floats()/set_float3s(), unless ArrayOverflow::any is set.
- Packed array samples (`point(0, "a", i)` of an array attribute) copy Int,
  Float and Float3 arrays like packed_to_value().
- `facepoints(0, f)` / `facepoints(f)` of the wrangled mesh copy its corner
  vertices (`VMEnv::corner_verts`, bound beside `face_offsets`).

`AttrAdd` of float, vector and int attributes, `sqrt` (as `sqrtf(max(x, 0))`),
`abs` and `length` are native as well. On the Bunny preset these remove about
110 million helper calls per solve; the energy kernel no longer calls a helper
in its inner loops.

Measured as before (Bunny, 250 steps, interleaved with a build of the previous
commit): 1.40 s to 1.20 s per cached solve. Results are bit-identical: the 36
energy/method/model hashes, the corpus (now 391 cases, JIT against VM), and the
parameter test. The new `native array pools` cases cover growth past the
capacity, negative and huge indices, declarations in loops, every append form,
long (continued) array attributes and samples, array stores that overflow, and
`facepoints` outside the faces and of other geometry.

Samples of attributes the program replaces
------------------------------------------

A same-domain sample of an attribute the program also writes reads the Jacobi
snapshot, which holds the values in the type the program writes. When the input
attribute has another type (`point(0, "x", i)` of an `int[]` while writing
`f@x`), that read returned the converted values: empty arrays, truncated floats,
averaged vectors, in the interpreter and the JIT alike. The executor now drops
such a snapshot, so the sample reads the input values as it does when the name is
not written. bind_attrs() replaces the attribute in that case (and when the
written attribute moves to another domain), so the prebound sample keeps a
sharing reference to the input data; before, a cross-domain sample of such an
attribute could read freed memory. Same-type writes keep the snapshot and take no
reference, which would copy the attribute on write. The cases are in
`wrangle_jit_cases.json` under "same-name samples of attributes the program
replaces".
