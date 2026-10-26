# Native Bijective UV node

Built and tested on 2026-10-22 after the user authorized compilation. Both
`bf_nodes_geometry` and the final `blender` target completed, followed by a
successful explicit-prefix runtime installation. The exact tested executable is
`E:/build_windows_Full_x64_vc17_Release/bin/Release/blender.exe`, SHA-256
`51ec77a54aed340da731dd5e905c6b2b94e41b4dfe68dc71bcf5820f94d8b250`.
The final link emitted LNK4098 (default C runtime library conflict); the focused
runtime, save/reopen, and render checks below passed despite that warning.

The node is registered as `GeometryNodeBijectiveUV`, displayed as **Bijective UV**
under **Mesh > UV**. Its Method menu selects Standard L-BFGS, Preconditioned
L-BFGS, or Assembled Linear System. The supplied reference file
`E:/Bijective UV Free Boundary - Standard L-BFGS Fixed.blend` remains unchanged.

## Shared objective and safeguards

Smith and Schaefer, *Bijective Parameterization with Free Boundaries*, SIGGRAPH
2015, https://people.engr.tamu.edu/schaefer/research/bijective.pdf.

The kernel follows the supplied graph's area-weighted symmetric Dirichlet
objective `ED = sum A ||J||² (1 + det(J)^-2)` and compact boundary barrier
`EB = weight * sum max(epsilon / distance(vertex, segment) - 1, 0)^2`.
The minimum distortion is 4. Barrier Distance zero uses one quarter of the
mean reference boundary edge length, matching the graph's default convention.
Each chart has its own support distance unless an explicit distance is supplied.

All three methods use the same analytic energy/gradient, positive determinant
checks, continuous triangle and boundary collision limits, safety factor 0.8,
Armijo c1=1e-4 and strong Wolfe c2=0.9 search. A decreasing Armijo step at the
collision cap may be accepted without satisfying Wolfe, as in the reference
graph. Search expands up to its certified limit and otherwise brackets/bisects.
Boundary collinearity with identically zero collision polynomial is treated
through endpoint contacts.

The methods generate different directions:

- **Standard L-BFGS:** ring of curvature pairs and scalar `H0 = gamma I`,
  `gamma = sᵀy / yᵀy`; no artificial lower clamp on gamma.
- **Preconditioned L-BFGS:** factor `P = 8 K_FEM + 1e-4 I` once per chart;
  apply `P^-1` between the two loops, with
  `gamma = sᵀy / (yᵀ P^-1 y)`.
- **Assembled Linear System:** assemble the analytic triangle Hessian after
  local positive-semidefinite projection, add a positive radial-curvature
  approximation of the boundary barrier and a `1e-4 I` regularizer, then
  solve `H p = -g`. This is an alternative projected Newton direction model,
  not a claim that the paper specified all three methods or that this is the
  exact unmodified Hessian of EB.

Double-precision arrays replace repeated geometry-node copies, field reductions
and interpreted two-loop updates. Reference triangle bases are computed once;
curvature history uses circular storage. A sorted boundary sweep index limits
energy and continuous-collision candidates. It replaces the paper's spatial
hash while preserving conservative bounds. Final boundary validation currently
uses pairwise segment checks; include that cost in end-to-end benchmarks.

## Input and output contract

Use oriented manifold charts whose vertices are already split along seams.
Polygons use Blender's existing internal triangulation. Disconnected charts
are optimized independently and are **not packed**; follow this node with Pack
UV Islands if atlas packing is needed. Per-chart bijectivity does not imply
that disconnected chart UV bounding boxes are disjoint.

Automatic uniform Tutte initialization supports disk charts with one boundary.
Charts with holes require Use Initial UV and a valid, positive, continuous
initial map. Discontinuous corner UVs on an unsplit vertex are rejected rather
than averaged. Closed charts, degenerate reference triangles, non-manifold
boundaries, flipped initial UVs and boundary intersections report errors.

UV is an anonymous corner-domain field on the Mesh output. Store it with Store
Named Attribute as FLOAT2 on the corner domain to create a persistent UV map.
Original mesh positions and topology are retained. Internal double results are
rounded to the actual float attribute representation and checked again before
publication; a loss of bijectivity at that precision is reported as an error.

Status values: 0 iteration budget, 1 all charts reached the stopping criterion,
2 at least one chart stalled in line search, 3 invalid input/numerical failure.
Five consecutive accepted steps below Tolerance stop a chart. Tolerance zero
disables that relative-energy rule; a numerically stationary gradient may still
stop it. Status 1 is a stopping criterion, not a global-optimum certificate.
Iterations Used is the maximum accepted count across charts; trial counts are
summed. Distortion is the area-weighted mean on the emitted float UVs.

## Verification

`tests/python/bijective_uv_kernel_test.cc` is a standalone C++ numerical driver
for finite-difference gradient comparison, all three objective-decreasing
solvers, Tutte initialization, invalid-input rejection and collision limiting.
Compile it with the Eigen include path and the geometry `nodes` directory;
keep assertions enabled. It was compiled with MSVC /O2 and assertions enabled.
All assertions passed; the measured energy-gradient finite-difference relative
error was 2.45014e-6, including active boundary barrier terms.

`tests/python/geometry_nodes_bijective_uv.py` builds temporary test objects and
compares all three methods against an independent NumPy distortion and boundary
intersection implementation. It saves JSON measurements and per-method NPZ UVs.
It fails explicitly if the native node is absent. It passed on the exact Release
for both the small curved grid and the Horse chart. The menu socket's Python API
uses display labels (for example `Preconditioned L-BFGS`) as its enum values;
`MENU_LABELS` translates the report identifiers to those values.

Reproduce the Horse comparison:

```powershell
& 'E:/build_windows_Full_x64_vc17_Release/bin/Release/blender.exe' `
  -b --factory-startup -t 4 --python `
  'E:/Blender_Source/tests/python/geometry_nodes_bijective_uv.py' -- `
  --asset 'E:/Agent/Codex/lbfgs_compare/initial_horse.npz' `
  --iterations 4000 --tolerance 1e-8 `
  --output 'E:/Agent/Codex/native_bijective/horse_comparison.json'
```

Use the same asset, float-rounded initial UV, barrier distance/weight, thread
count, budget and tolerance for all methods. Record initialization, solve and
wall time, actual iterations, termination reason, SD mean/max, minimum
determinant, boundary crossings, and barrier energy. Retain the original graph
as a separate baseline. Compare fixed-budget quality and time to a common
quality threshold separately. No speedup over the old interpreted graph is
claimed here: that graph was not rerun as a matched baseline in this session.
There is no claim of coordinate equality between the methods or with the graph.

## Measured results

All three Horse runs used 20,636 vertices and 39,698 triangles, the same
float-rounded initial UV, epsilon 0.003407061332836747, barrier weight
0.0003441694425418973, history 8, budget 4000 and tolerance 1e-8. The initial
area-weighted SD was 1427.34483046, with zero flips and boundary crossings.

- Standard L-BFGS: 21.412670 s, 4000 accepted iterations, SD mean 4.0950277473,
  SD max 27.4649190. Termination: budget, not convergence.
- Preconditioned L-BFGS: 8.947618 s, 761 accepted iterations, SD mean 4.0943308107,
  SD max 27.6083689. Termination: five small relative energy decreases.
- Assembled system: 19.507524 s, 82 accepted iterations, SD mean 4.0942642184,
  SD max 27.5465299. Termination: five small relative energy decreases.

Times include native preparation, solving and float-output validation. All
outputs had zero nonpositive determinants and zero boundary intersections in
the independent NumPy checks. These are single-run measurements on this host.
Different methods reach slightly different nonconvex solutions.

An additional fixed-100-iteration run disabled the relative-energy stop:

- Standard: 0.354095 s; SD mean 91.08538244.
- Preconditioned: 0.758995 s; SD mean 6.05224445.
- Assembled: 20.253105 s; SD mean 4.09426413.

All accepted exactly 100 iterations; all outputs remained free of flips and
boundary intersections. The assembled step is much more expensive, while making
larger early improvements. Preconditioned L-BFGS is the fastest default in the
tested end-to-end Horse run, not a universal performance guarantee.

Regression coverage includes automatic Tutte initialization, supplied UVs,
distorted annuli with supplied UVs, disconnected charts, and rejection of
flipped/degenerate inputs and unsupported automatic annulus initialization:
21 method/case combinations plus an explicit zero-iteration identity check.
Disconnected charts are checked separately: inter-chart overlap is expected
until a packing operation is applied.

Known numerical behavior: on the small curved-grid fixture, assembled Newton
can exhaust line-search progress at essentially the same SD as the other
methods before accumulating five small accepted decreases. It reports status 2,
not convergence. This did not occur in the tolerance-based Horse run.

Logs, JSON results, per-method NPZs, and comparison PNGs are in
`E:/Agent/Codex/native_bijective/`. `horse_comparison.json` is the stopping-rule
comparison; `horse_fixed100.json` is the fixed-iteration comparison.

`E:/Bijective UV Native - Three Solvers.blend` contains three live editable
native-node graphs and a reference surface. It was rendered, visually inspected,
saved, and reopened with numerical checks. Switching the menu on an already
evaluated node was also verified. Desktop startup was observed; an attempted
interactive menu inspection was stopped after user input and window closure,
so an interactive dropdown screenshot is not part of the acceptance evidence.
