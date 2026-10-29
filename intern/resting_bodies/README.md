# Resting Bodies

Native forward-model integration of **Putting Rigid Bodies to Rest**, Baktash,
Sharp, Zhou, Crane and Jacobson, SIGGRAPH 2025.

- Paper: https://www.cs.cmu.edu/~kmcrane/Projects/RestingBodies/PuttingRigidBodiesToRest.pdf
- Author implementation: https://github.com/hbaktash/resting-rigid-bodies
- Author code retains its MIT license in `LICENSE`.
- Geometry Central revision: `095124d31ace21fcda4c06df4b4cb046de84d30a`;
  license in `deps/geometry-central/LICENSE`.
- Qhull release tag: `v8.1-alpha6`; license in `deps/qhull/COPYING.txt`.

The forward core classifies hull contacts, traces great-circle gradient paths,
and constructs attraction-basin boundaries to compute spherical areas divided
by `4*pi`. It does not estimate probabilities by random sampling. Probabilities
assume uniformly distributed initial orientations and negligible momentum.
There are no bouncing, colliding piles, arbitrary supporting surfaces, or
inverse-design optimization in this node.

The wrapper normalizes geometry relative to the supplied mass center, merges
identical stable normals, validates area totals, limits contact-transition
loops, and constructs contact-preserving rigid transformations. Original
surface geometry is retained; only the support computation uses its convex hull.
The node computes uniform solid mass from the original closed oriented surface.
The center must be strictly inside the hull. The bridge API also accepts a supplied
mass center for standalone/reference validation.
The support plane is local `Z=0`; scene object transforms are applied afterward.

The node is named **Putting Rigid Bodies to Rest** and matches the reference group:
inputs `Mesh`, `Gravity`, `Progress`; outputs `Transform`, `Trajectory`,
`Separatrix`, `Stable Poses`, in that order. Gravity defaults to `(0, 0, -1)`
and Progress defaults to `1`. The RNA type and Gravity's legacy socket identifier
remain unchanged so existing group links to the initial direction are retained.
The default node width is 200.
Use Transform Geometry to apply the matrix to the original mesh.

`Stable Poses` is a point cloud on the unit Gauss sphere. Its attributes are
`rest_transform` (4x4 matrix), `rest_normal` (vector), `rest_height` (float),
and `rest_probability` (float).
Normals point downward in the input object's coordinate system. Matrices
transform original input coordinates to grounded output coordinates.
`Progress` follows accumulated rotation angle, not physical time.

`Trajectory` is one noncyclic Poly curve connecting the ordered rolling-direction
samples on the unit Gauss sphere. It has no custom attributes: only position and
the built-in curve representation are stored. It always describes the complete
path; Progress selects the Transform along that path. An initially stable
orientation produces a one-point curve, not an invented segment or closing edge.

`Separatrix` exports the ascending manifolds of saddle contacts from the same
boundary graph used for probability integration. Coplanar triangles belonging to
one merged stable pose do not introduce basin seams. Each arc is an independent,
noncyclic cubic Bezier curve with exactly two control points at its unit-sphere
endpoints. Aligned handles follow the great-circle tangent at each endpoint:
for arc-plane normal `n = normalize(a cross b)`, the forward tangents are
`n cross a` and `n cross b`, with handle length `(4/3) tan(angle/4)`.
All four handles are set explicitly. Curve evaluation resolution is 32; it does
not add control points. There are no connections between unrelated arcs.
An ordinary cubic Bezier approximates the interior of a circular arc; endpoints
and tangent directions are exact up to floating-point output precision.
Its point attribute `rest_normal` equals the endpoint direction. Curve-domain
attributes `rest_pose_a` and `rest_pose_b` give the adjacent sorted stable-pose
indices, and `rest_arc` identifies the analytic arc. These attributes formerly
belonged to wire-mesh edges. Pose indices are zero-based indices into the
`Stable Poses` output, whose points are ordered by decreasing probability;
they are not persistent IDs when the shape changes. The endpoint `rest_normal`
duplicates Position, and `rest_arc` currently duplicates the curve index.
These are descriptive metadata, not inputs required to generate or render the
Bezier geometry. Uniformly scaling/translating the input does not
change this sphere geometry. Gravity and Progress affect the selected transform
and trajectory, not the attraction-basin boundaries.

The simplified node fixes trajectory sampling at 0.002 radians, matching the
reference group's internal setting; advanced controls from the original prototype
are no longer sockets on this node.

Eight geometry/center precomputations are retained in a bounded process cache.
Each solver also caches its last initial-direction/angle-step trajectory.
The native C interface contains exceptions and separates geometry dependencies
from Blender node compilation. Node access is serialized to protect mutable
trace state; the author boundary counter is thread-local.

Changes to author sources: own temporary hull buffers/meshes, clear trace logs,
free boundary normals, make pairwise distance tables optional, and reject
singular or nonterminating contact traces. Author probability and gradient
formulas are retained. Exact unstable equilibria with ambiguous rolling
direction produce an error rather than silently perturbing the input.
