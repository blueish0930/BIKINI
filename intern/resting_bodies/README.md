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
The node computes uniform solid mass from the original closed oriented surface,
or accepts a custom mass center. The center must be strictly inside the hull.
The support plane is local `Z=0`; scene object transforms are applied afterward.

`Stable Poses` and `Trajectory` are point clouds on the unit Gauss sphere.
Their attributes are `rest_transform` (4x4 matrix), `rest_normal` (vector),
`rest_height` (float), and `rest_probability` (float, stable poses only).
Normals point downward in the input object's coordinate system. Matrices
transform original input coordinates to grounded output coordinates.
`Progress` follows accumulated rotation angle, not physical time.

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
