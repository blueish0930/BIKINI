/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Tag geometry for Box Engine Solver + emit collision-shape preview.
 *
 * Mesh is recentered into an Instance so transforms match collision.
 * Collision output matches the collider the solver will use:
 *   Box / Sphere / Capsule — Size primitives
 *   Bounding Box — mesh AABB
 *   Convex Hull — convex hull of vertices
 *   Compound — one convex hull per mesh island (or Proxy)
 *   Triangle Mesh — the triangle mesh itself
 */

#include "BKE_attribute.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_instances.hh"
#include "BKE_material.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "BLI_array.hh"
#include "BLI_bounds.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "GEO_mesh_primitive_cuboid.hh"
#include "GEO_mesh_primitive_uv_sphere.hh"

#include "box_engine_shared.hh"
#include "node_geometry_util.hh"

#include <fmt/format.h>
#include <string>

#ifdef WITH_BULLET
#  include "RBI_hull_api.h"
#endif

namespace blender::nodes::node_geo_box_engine_set_rigid_body_cc {

enum class BodyType {
  Static = 0,
  Kinematic = 1,
  Dynamic = 2,
};

/** Keep integer values in sync with GeometryNodeBoxEngineSolver. */
enum class ShapeType {
  Sphere = 0,
  Box = 1,
  Capsule = 2,
  ConvexHull = 3,
  BoundingBox = 4,
  TriangleMesh = 5,
  Compound = 6,
};

static const EnumPropertyItem body_type_items[] = {
    {int(BodyType::Static),
     "STATIC",
     0,
     N_("Static"),
     N_("Does not move (ground, walls). If this object moves, use Kinematic")},
    {int(BodyType::Kinematic),
     "KINEMATIC",
     0,
     N_("Kinematic"),
     N_("Animated collider (turntable, hands, rain). Sweeps; required for anything that moves")},
    {int(BodyType::Dynamic), "DYNAMIC", 0, N_("Dynamic"), N_("Simulated under gravity/forces")},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem shape_type_items[] = {
    {int(ShapeType::Box), "BOX", 0, N_("Box"), N_("Manual box; Size = full extents (XYZ)")},
    {int(ShapeType::Sphere),
     "SPHERE",
     0,
     N_("Sphere"),
     N_("Manual sphere/circle; Size.X = diameter")},
    {int(ShapeType::Capsule),
     "CAPSULE",
     0,
     N_("Capsule"),
     N_("Manual capsule; Size.X = diameter, Size.Y = total height")},
    {int(ShapeType::BoundingBox),
     "BOUNDING_BOX",
     0,
     N_("Bounding Box"),
     N_("Box from mesh AABB (no Size)")},
    {int(ShapeType::ConvexHull),
     "CONVEX_HULL",
     0,
     N_("Convex Hull"),
     N_("Convex hull of mesh vertices (no Size)")},
    {int(ShapeType::Compound),
     "COMPOUND",
     0,
     N_("Compound"),
     N_("One rigid body: convex hull per mesh island (and per instance). "
        "All hulls are the collision proxy. Split loose parts or instance pieces. No Size")},
    {int(ShapeType::TriangleMesh),
     "TRIANGLE_MESH",
     0,
     N_("Triangle Mesh"),
     N_("Non-convex triangle mesh. Box3D Static: mesh contacts. Box2D: one convex piece "
        "per triangle. Dynamic Box3D uses per-island hulls. No Size")},
    {0, nullptr, 0, nullptr, nullptr},
};

namespace attr {
constexpr StringRefNull body_type = "body_type";
constexpr StringRefNull shape_type = "shape_type";
constexpr StringRefNull shape_size = "shape_size";
constexpr StringRefNull density = "density";
constexpr StringRefNull friction = "friction";
constexpr StringRefNull restitution = "restitution";
constexpr StringRefNull radius = "radius";
}  // namespace attr

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .description("Body geometry: Mesh (→ centered instance), Points, or Instances");
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Original visual geometry + rigid-body attributes (not the collider mesh)");

  b.add_output<decl::Geometry>("Collision"_ustr)
      .description(
          "Collision-shape preview (matches the solver collider: hull / islands / "
          "triangle mesh / primitive). Connect to Viewer — not for rendering");

  b.add_input<decl::Bool>("Selection"_ustr)
      .default_value(true)
      .hide_value()
      .evaluated_geometry_field()
      .description("Which points/instances receive a rigid body");

  b.add_input<decl::String>("Name Prefix"_ustr)
      .default_value("Body")
      .description(
          "body_name prefix. First free name is the prefix itself; duplicates get "
          ".001, .002, … Existing non-empty body_name is not overwritten");

  b.add_input<decl::Menu>("Body Type"_ustr)
      .static_items(body_type_items)
      .default_value(BodyType::Dynamic)
      .optional_label()
      .description("Static, kinematic, or dynamic rigid body");
  b.add_input<decl::Menu>("Shape"_ustr)
      .static_items(shape_type_items)
      .default_value(ShapeType::BoundingBox)
      .optional_label()
      .description("Collision shape used by the solver");

  b.add_input<decl::Geometry>("Proxy"_ustr)
      .description(
          "Compound only: optional collision mesh. Each mesh island (and each instance) "
          "becomes one convex hull on a single body. Empty = use Geometry")
      .usage_by_menu("Shape"_ustr, int(ShapeType::Compound));

  b.add_input<decl::Vector>("Size"_ustr)
      .default_value(float3(1.0f, 1.0f, 1.0f))
      .subtype(PROP_XYZ)
      .evaluated_geometry_field()
      .description("Box / Sphere / Capsule only — full extents (not half)")
      .usage_by_menu("Shape"_ustr,
                     {int(ShapeType::Box), int(ShapeType::Sphere), int(ShapeType::Capsule)});

  b.add_input<decl::Float>("Density"_ustr)
      .default_value(1.0f)
      .min(1e-4f)
      .evaluated_geometry_field()
      .description("Mass density for Dynamic bodies only")
      .usage_by_menu("Body Type"_ustr, int(BodyType::Dynamic));

  b.add_input<decl::Float>("Friction"_ustr)
      .default_value(0.4f)
      .min(0.0f)
      .evaluated_geometry_field()
      .description("Coulomb friction coefficient");
  b.add_input<decl::Float>("Restitution"_ustr)
      .default_value(0.05f)
      .min(0.0f)
      .max(1.0f)
      .evaluated_geometry_field()
      .description("Bounce coefficient (0 = no bounce, 1 = elastic)");
}

static GeometrySet mesh_to_centered_instance(GeometrySet geometry, float3 &r_half_extents)
{
  r_half_extents = float3(0.5f);
  if (geometry.has_instances() || !geometry.has_mesh()) {
    if (const Mesh *mesh = geometry.get_mesh()) {
      if (const std::optional<Bounds<float3>> bounds = mesh->bounds_min_max()) {
        r_half_extents = math::max(bounds->size() * 0.5f, float3(1e-5f));
      }
    }
    return geometry;
  }

  Mesh *mesh = geometry.get_mesh_for_write();
  if (!mesh || mesh->verts_num == 0) {
    return geometry;
  }

  const std::optional<Bounds<float3>> bounds = mesh->bounds_min_max();
  float3 center(0.0f);
  if (bounds) {
    center = bounds->center();
    r_half_extents = math::max(bounds->size() * 0.5f, float3(1e-5f));
    MutableSpan<float3> positions = mesh->vert_positions_for_write();
    for (float3 &p : positions) {
      p -= center;
    }
    mesh->tag_positions_changed();
  }

  GeometrySet reference = geometry;
  std::unique_ptr<bke::Instances> instances = std::make_unique<bke::Instances>(1);
  const int handle = instances->add_reference(bke::InstanceReference{std::move(reference)});
  instances->reference_handles_for_write().first() = handle;
  instances->transforms_for_write().first() = math::from_location<float4x4>(center);

  GeometrySet out;
  out.replace_instances(instances.release());
  return out;
}

/** Half-extents from Size socket: Size is full extents (user-facing). */
static float3 half_from_size_socket(const float3 full_size, const ShapeType shape)
{
  if (shape == ShapeType::Sphere) {
    const float r = math::max(full_size.x * 0.5f, 1e-5f);
    return float3(r);
  }
  if (shape == ShapeType::Capsule) {
    return float3(math::max(full_size.x * 0.5f, 1e-5f),
                  math::max(full_size.y * 0.5f, 1e-5f),
                  math::max(full_size.x * 0.5f, 1e-5f));
  }
  return math::max(full_size * 0.5f, float3(1e-5f));
}

static void write_body_attrs(bke::MutableAttributeAccessor attributes,
                             const bke::AttrDomain domain,
                             const int size,
                             const fn::FieldContext &context,
                             const Field<bool> &selection_field,
                             const int body_type,
                             const int shape_type,
                             const Field<float3> &size_field,
                             const Field<float> &density_field,
                             const Field<float> &friction_field,
                             const Field<float> &restitution_field,
                             const float3 mesh_half_extents)
{
  if (size <= 0) {
    return;
  }
  fn::FieldEvaluator evaluator{context, size};
  evaluator.set_selection(selection_field);
  evaluator.add(size_field);
  evaluator.add(density_field);
  evaluator.add(friction_field);
  evaluator.add(restitution_field);
  evaluator.evaluate();

  const IndexMask selection = evaluator.get_evaluated_selection_as_mask();

  /* Always stamp body/shape type on every element. Selection must not drop Compound,
   * or the solver silently falls back to a single bounding box. */
  {
    bke::SpanAttributeWriter<int> body_all = attributes.lookup_or_add_for_write_span<int>(
        attr::body_type, domain);
    bke::SpanAttributeWriter<int> shape_all = attributes.lookup_or_add_for_write_span<int>(
        attr::shape_type, domain);
    if (body_all) {
      body_all.span.fill(body_type);
      body_all.finish();
    }
    if (shape_all) {
      shape_all.span.fill(shape_type);
      shape_all.finish();
    }
  }

  if (selection.is_empty()) {
    return;
  }

  const VArray<float3> sizes_in = evaluator.get_evaluated<float3>(0);
  const VArray<float> densities = evaluator.get_evaluated<float>(1);
  const VArray<float> frictions = evaluator.get_evaluated<float>(2);
  const VArray<float> restitutions = evaluator.get_evaluated<float>(3);

  bke::SpanAttributeWriter<int> body_w = attributes.lookup_or_add_for_write_span<int>(
      attr::body_type, domain);
  bke::SpanAttributeWriter<int> shape_w = attributes.lookup_or_add_for_write_span<int>(
      attr::shape_type, domain);
  bke::SpanAttributeWriter<float3> size_w = attributes.lookup_or_add_for_write_span<float3>(
      attr::shape_size, domain);
  bke::SpanAttributeWriter<float> dens_w = attributes.lookup_or_add_for_write_span<float>(
      attr::density, domain);
  bke::SpanAttributeWriter<float> fric_w = attributes.lookup_or_add_for_write_span<float>(
      attr::friction, domain);
  bke::SpanAttributeWriter<float> rest_w = attributes.lookup_or_add_for_write_span<float>(
      attr::restitution, domain);
  bke::SpanAttributeWriter<float> rad_w = attributes.lookup_or_add_for_write_span<float>(
      attr::radius, domain);

  const bool mesh_driven = ELEM(shape_type,
                                int(ShapeType::BoundingBox),
                                int(ShapeType::ConvexHull),
                                int(ShapeType::TriangleMesh),
                                int(ShapeType::Compound));

  selection.foreach_index([&](const int i) {
    if (body_w) {
      body_w.span[i] = body_type;
    }
    if (shape_w) {
      shape_w.span[i] = shape_type;
    }
    float3 half = mesh_half_extents;
    if (!mesh_driven) {
      half = half_from_size_socket(sizes_in[i], ShapeType(shape_type));
    }
    half = math::max(half, float3(1e-5f));
    if (size_w) {
      size_w.span[i] = half; /* always store half-extents for solver */
    }
    if (dens_w) {
      dens_w.span[i] = (body_type == int(BodyType::Dynamic)) ? math::max(densities[i], 1e-3f) :
                                                               math::max(densities[i], 0.0f);
    }
    if (fric_w) {
      fric_w.span[i] = frictions[i];
    }
    if (rest_w) {
      rest_w.span[i] = restitutions[i];
    }
    if (rad_w) {
      rad_w.span[i] = math::max(half.x, 1e-5f);
    }
  });

  if (body_w) {
    body_w.finish();
  }
  if (shape_w) {
    shape_w.finish();
  }
  if (size_w) {
    size_w.finish();
  }
  if (dens_w) {
    dens_w.finish();
  }
  if (fric_w) {
    fric_w.finish();
  }
  if (rest_w) {
    rest_w.finish();
  }
  if (rad_w) {
    rad_w.finish();
  }
}

static Mesh *preview_box_mesh(const float3 half)
{
  return geometry::create_cuboid_mesh(
      float3(half.x * 2.0f, half.y * 2.0f, half.z * 2.0f), 2, 2, 2);
}

static Mesh *preview_sphere_mesh(const float radius)
{
  /* Low-res UV sphere for preview. */
  return geometry::create_uv_sphere_mesh(radius, 8, 12, {});
}

static Mesh *preview_box_from_verts(const Span<float3> verts)
{
  if (verts.is_empty()) {
    return preview_box_mesh(float3(1e-5f));
  }
  const std::optional<Bounds<float3>> bounds = bounds::min_max(verts);
  if (!bounds) {
    return preview_box_mesh(float3(1e-5f));
  }
  Mesh *box = preview_box_mesh(math::max(bounds->size() * 0.5f, float3(1e-5f)));
  const float3 center = bounds->center();
  if (math::length_squared(center) > 1e-16f) {
    MutableSpan<float3> positions = box->vert_positions_for_write();
    for (float3 &p : positions) {
      p += center;
    }
    box->tag_positions_changed();
  }
  return box;
}

#ifdef WITH_BULLET
static Mesh *hull_mesh_from_bullet(const Span<float3> coords)
{
  if (coords.size() < 3) {
    return nullptr;
  }
  plConvexHull hull = plConvexHullCompute((float (*)[3])coords.data(), int(coords.size()));
  if (!hull) {
    return nullptr;
  }

  const int verts_num = plConvexHullNumVertices(hull);
  const int faces_num = verts_num <= 2 ? 0 : plConvexHullNumFaces(hull);
  const int loops_num = verts_num <= 2 ? 0 : plConvexHullNumLoops(hull);
  const int edges_num = verts_num == 2 ? 1 : verts_num < 2 ? 0 : loops_num / 2;
  if (verts_num < 3 || faces_num < 1 || loops_num < 3) {
    plConvexHullDelete(hull);
    return nullptr;
  }

  Mesh *result = BKE_mesh_new_nomain(verts_num, edges_num, faces_num, loops_num);
  BKE_id_material_eval_ensure_default_slot(&result->id);
  bke::mesh_smooth_set(*result, false);

  MutableSpan<float3> dst_positions = result->vert_positions_for_write();
  for (const int i : IndexRange(verts_num)) {
    float3 dummy_co;
    int original_index;
    plConvexHullGetVertex(hull, i, dummy_co, &original_index);
    if (!coords.index_range().contains(original_index)) {
      dst_positions[i] = float3(0);
      continue;
    }
    dst_positions[i] = coords[original_index];
  }

  Array<int> corner_verts(loops_num);
  Array<int> corner_edges(loops_num);
  uint edge_index = 0;
  MutableSpan<int2> edges = result->edges_for_write();
  for (const int i : IndexRange(loops_num)) {
    int v_from;
    int v_to;
    plConvexHullGetLoop(hull, i, &v_from, &v_to);
    corner_verts[i] = v_from;
    if (v_from < v_to) {
      edges[edge_index] = int2(v_from, v_to);
      const int reverse_index = plConvexHullGetReversedLoopIndex(hull, i);
      corner_edges[i] = int(edge_index);
      if (IndexRange(loops_num).contains(reverse_index)) {
        corner_edges[reverse_index] = int(edge_index);
      }
      edge_index++;
    }
  }

  Array<int> loops;
  int j = 0;
  MutableSpan<int> face_offsets = result->face_offsets_for_write();
  MutableSpan<int> mesh_corner_verts = result->corner_verts_for_write();
  MutableSpan<int> mesh_corner_edges = result->corner_edges_for_write();
  int dst_corner = 0;
  for (const int i : IndexRange(faces_num)) {
    const int len = plConvexHullGetFaceSize(hull, i);
    loops.reinitialize(len);
    plConvexHullGetFaceLoops(hull, i, loops.data());
    face_offsets[i] = j;
    for (const int k : IndexRange(len)) {
      mesh_corner_verts[dst_corner] = corner_verts[loops[k]];
      mesh_corner_edges[dst_corner] = corner_edges[loops[k]];
      dst_corner++;
    }
    j += len;
  }
  face_offsets[faces_num] = j;

  plConvexHullDelete(hull);
  return result;
}
#endif

/** Convex hull of verts in the same space as the source mesh. Falls back to island AABB. */
static Mesh *preview_hull_mesh(const Span<float3> verts)
{
#ifdef WITH_BULLET
  if (Mesh *hull = hull_mesh_from_bullet(verts)) {
    return hull;
  }
#endif
  return preview_box_from_verts(verts);
}

/** One hull instance per island. Island verts are already in output space. */
static GeometrySet preview_compound_islands(const Span<Vector<float3>> islands)
{
  GeometrySet preview;
  if (islands.is_empty()) {
    return preview;
  }
  std::unique_ptr<bke::Instances> out_inst = std::make_unique<bke::Instances>();
  Vector<int> out_handles;
  Vector<float4x4> out_transforms;
  out_handles.reserve(islands.size());
  out_transforms.reserve(islands.size());
  for (const Vector<float3> &island : islands) {
    Mesh *hull = preview_hull_mesh(island);
    if (!hull) {
      continue;
    }
    GeometrySet ref;
    ref.replace_mesh(hull);
    out_handles.append(out_inst->add_reference(bke::InstanceReference{std::move(ref)}));
    out_transforms.append(float4x4::identity());
  }
  if (!out_handles.is_empty()) {
    out_inst->resize(out_handles.size());
    out_inst->reference_handles_for_write().copy_from(out_handles);
    out_inst->transforms_for_write().copy_from(out_transforms);
    preview.replace_instances(out_inst.release());
  }
  return preview;
}

static GeometrySet build_collision_preview(const GeometrySet &geometry,
                                           const ShapeType shape_type,
                                           const float3 fallback_half)
{
  GeometrySet preview;
  if (!geometry.has_instances()) {
    /* Points: one shape at each point using fallback_half / attrs. */
    if (const PointCloud *points = geometry.get_pointcloud()) {
      if (points->totpoint == 0) {
        return preview;
      }
      const Span<float3> positions = points->positions();
      const bke::AttributeAccessor attributes = points->attributes();
      const VArray<float3> sizes = *attributes.lookup_or_default<float3>(
          attr::shape_size, bke::AttrDomain::Point, fallback_half);
      const VArray<int> shapes = *attributes.lookup_or_default<int>(
          attr::shape_type, bke::AttrDomain::Point, int(shape_type));

      std::unique_ptr<bke::Instances> instances = std::make_unique<bke::Instances>();
      Vector<int> handles;
      Vector<float4x4> transforms;
      handles.reserve(points->totpoint);
      transforms.reserve(points->totpoint);

      for (const int i : IndexRange(points->totpoint)) {
        const float3 half = math::max(sizes[i], float3(1e-5f));
        Mesh *shape_mesh = nullptr;
        const int st = shapes[i];
        if (st == int(ShapeType::Sphere)) {
          shape_mesh = preview_sphere_mesh(half.x);
        }
        else {
          shape_mesh = preview_box_mesh(half);
        }
        GeometrySet ref;
        ref.replace_mesh(shape_mesh);
        const int h = instances->add_reference(bke::InstanceReference{std::move(ref)});
        handles.append(h);
        transforms.append(math::from_location<float4x4>(positions[i]));
      }
      instances->resize(handles.size());
      instances->reference_handles_for_write().copy_from(handles);
      instances->transforms_for_write().copy_from(transforms);
      preview.replace_instances(instances.release());
    }
    return preview;
  }

  const bke::Instances &src = *geometry.get_instances();
  const Span<float4x4> transforms = src.transforms();
  const Span<int> ref_handles = src.reference_handles();
  const Span<bke::InstanceReference> refs = src.references();
  const bke::AttributeAccessor attributes = src.attributes();
  const VArray<float3> sizes = *attributes.lookup_or_default<float3>(
      attr::shape_size, bke::AttrDomain::Instance, fallback_half);
  const VArray<int> shapes = *attributes.lookup_or_default<int>(
      attr::shape_type, bke::AttrDomain::Instance, int(shape_type));

  std::unique_ptr<bke::Instances> out_inst = std::make_unique<bke::Instances>();
  Vector<int> out_handles;
  Vector<float4x4> out_transforms;

  for (const int i : transforms.index_range()) {
    const int st = shapes[i];
    const float3 half = math::max(sizes[i], float3(1e-5f));
    const int rh = ref_handles[i];
    GeometrySet extracted;
    const Mesh *src_mesh = nullptr;
    if (rh >= 0 && rh < refs.size()) {
      extracted = box_engine::collision_geometry_from_reference(refs[rh]);
      src_mesh = extracted.get_mesh();
    }

    auto add_preview_mesh = [&](Mesh *mesh, const float4x4 &tf) {
      if (!mesh) {
        return;
      }
      GeometrySet ref;
      ref.replace_mesh(mesh);
      const int h = out_inst->add_reference(bke::InstanceReference{std::move(ref)});
      out_handles.append(h);
      out_transforms.append(tf);
    };

    /* Collision socket is preview only — never mutate Geometry. */
    if (st == int(ShapeType::TriangleMesh) && src_mesh) {
      add_preview_mesh(BKE_mesh_copy_for_eval(*src_mesh), transforms[i]);
    }
    else if (st == int(ShapeType::ConvexHull) && src_mesh) {
      add_preview_mesh(preview_hull_mesh(src_mesh->vert_positions()), transforms[i]);
    }
    else if (st == int(ShapeType::Compound) && src_mesh) {
      Vector<Vector<float3>> islands;
      box_engine::fill_islands_from_mesh(*src_mesh, islands);
      for (const Vector<float3> &island : islands) {
        add_preview_mesh(preview_hull_mesh(island), transforms[i]);
      }
    }
    else if (st == int(ShapeType::Sphere)) {
      add_preview_mesh(preview_sphere_mesh(half.x), transforms[i]);
    }
    else if (st == int(ShapeType::Capsule)) {
      /* Approximate capsule with a tall box for preview. */
      add_preview_mesh(preview_box_mesh(float3(half.x, half.y + half.x, half.x)), transforms[i]);
    }
    else if (st == int(ShapeType::Box)) {
      /* Manual Size — do not replace with the visual mesh AABB. */
      add_preview_mesh(preview_box_mesh(half), transforms[i]);
    }
    else {
      /* Bounding Box (or unknown): mesh AABB, else stored half-extents. */
      float3 use_half = half;
      float4x4 tf = transforms[i];
      if (src_mesh) {
        if (const std::optional<Bounds<float3>> bounds = src_mesh->bounds_min_max()) {
          use_half = math::max(bounds->size() * 0.5f, float3(1e-5f));
          if (math::length(bounds->center()) > 1e-4f) {
            tf = transforms[i] * math::from_location<float4x4>(bounds->center());
          }
        }
      }
      add_preview_mesh(preview_box_mesh(use_half), tf);
    }
  }

  if (!out_handles.is_empty()) {
    out_inst->resize(out_handles.size());
    out_inst->reference_handles_for_write().copy_from(out_handles);
    out_inst->transforms_for_write().copy_from(out_transforms);
    preview.replace_instances(out_inst.release());
  }
  return preview;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry"_ustr);
  GeometrySet proxy_geo = params.extract_input<GeometrySet>("Proxy"_ustr);
  float3 mesh_half_extents(0.5f);
  geometry = mesh_to_centered_instance(std::move(geometry), mesh_half_extents);

  const int body_type = int(params.extract_input<BodyType>("Body Type"_ustr));
  const ShapeType shape_type = params.extract_input<ShapeType>("Shape"_ustr);
  const int shape_type_i = int(shape_type);
  const Field<bool> selection = params.extract_input<Field<bool>>("Selection"_ustr);
  const std::string name_prefix = params.extract_input<std::string>("Name Prefix"_ustr);

  /* Always extract; usage_by_menu only hides sockets in the UI. */
  const Field<float3> size_field = params.extract_input<Field<float3>>("Size"_ustr);
  const Field<float> density_field = params.extract_input<Field<float>>("Density"_ustr);
  const Field<float> friction = params.extract_input<Field<float>>("Friction"_ustr);
  const Field<float> restitution = params.extract_input<Field<float>>("Restitution"_ustr);

  /* Compound: bake one convex-hull source island list onto a sibling Mesh so the solver
   * attaches every island hull to a single body (instances stay visual). */
  Vector<Vector<float3>> compound_islands;
  Mesh *compound_proxy_mesh = nullptr;
  if (shape_type == ShapeType::Compound) {
    if (proxy_geo.has_mesh() || proxy_geo.has_instances()) {
      float3 proxy_half(0.5f);
      GeometrySet proxy_centered = mesh_to_centered_instance(std::move(proxy_geo), proxy_half);
      box_engine::collect_islands_from_geometry(proxy_centered, compound_islands);
      mesh_half_extents = proxy_half;
    }
    else {
      box_engine::collect_islands_from_geometry(geometry, compound_islands);
    }
    compound_proxy_mesh = box_engine::bake_compound_islands(compound_islands);
    if (compound_islands.size() <= 1) {
      params.error_message_add(
          NodeWarningType::Warning,
          TIP_("Compound found fewer than 2 islands — collision is a single hull. "
               "Separate loose parts (Split to Instances / disconnected mesh) "
               "so each piece gets its own convex hull"));
    }
    else {
      const std::string msg = fmt::format(
          fmt::runtime(TIP_("Compound: {} convex hulls on one body")), compound_islands.size());
      params.error_message_add(NodeWarningType::Info, msg);
    }
  }

  if (shape_type == ShapeType::TriangleMesh && body_type != int(BodyType::Static)) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("Triangle Mesh: Box3D mesh contacts are Static-only. "
             "Dynamic/Kinematic uses per-island convex hulls"));
  }

  bool wrote = false;
  if (bke::Instances *instances = geometry.get_instances_for_write()) {
    float3 half = mesh_half_extents;
    if (instances->instances_num() > 0 && !instances->references().is_empty()) {
      const bke::InstanceReference &ref = instances->references()[0];
      if (ref.type() == bke::InstanceReference::Type::GeometrySet) {
        if (const Mesh *mesh = ref.geometry_set().get_mesh()) {
          if (const std::optional<Bounds<float3>> bounds = mesh->bounds_min_max()) {
            half = math::max(bounds->size() * 0.5f, float3(1e-5f));
          }
        }
      }
    }
    const bke::InstancesFieldContext context{*instances};
    write_body_attrs(instances->attributes_for_write(),
                     bke::AttrDomain::Instance,
                     instances->instances_num(),
                     context,
                     selection,
                     body_type,
                     shape_type_i,
                     size_field,
                     density_field,
                     friction,
                     restitution,
                     half);
    {
      const VArray<bool> skip_hull = *instances->attributes().lookup_or_default<bool>(
          box_engine::attr_compound_hull, bke::AttrDomain::Instance, false);
      box_engine::ensure_unique_body_names(instances->attributes_for_write(),
                                           bke::AttrDomain::Instance,
                                           instances->instances_num(),
                                           name_prefix,
                                           &skip_hull);
    }
    wrote = instances->instances_num() > 0;
  }
  if (PointCloud *points = geometry.get_pointcloud_for_write()) {
    const bke::PointCloudFieldContext context{*points};
    write_body_attrs(points->attributes_for_write(),
                     bke::AttrDomain::Point,
                     points->totpoint,
                     context,
                     selection,
                     body_type,
                     shape_type_i,
                     size_field,
                     density_field,
                     friction,
                     restitution,
                     mesh_half_extents);
    box_engine::ensure_unique_body_names(points->attributes_for_write(),
                                         bke::AttrDomain::Point,
                                         points->totpoint,
                                         name_prefix);
    wrote = wrote || points->totpoint > 0;
  }

  if (!wrote) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Set Rigid Body needs Mesh, Points, or Instances"));
  }

  /* Append one convex-hull instance per island. The solver reads these as collision
   * pieces — same meshes as the Collision preview — so physics cannot collapse to
   * an overall hull. */
  if (shape_type == ShapeType::Compound && !compound_islands.is_empty()) {
    if (bke::Instances *inst = geometry.get_instances_for_write()) {
      const int old_n = inst->instances_num();
      const int add = compound_islands.size();
      inst->resize(old_n + add);

      bke::MutableAttributeAccessor attrs = inst->attributes_for_write();
      bke::SpanAttributeWriter<bool> hull_w = attrs.lookup_or_add_for_write_span<bool>(
          box_engine::attr_compound_hull, bke::AttrDomain::Instance);
      bke::SpanAttributeWriter<int> shape_w = attrs.lookup_or_add_for_write_span<int>(
          attr::shape_type, bke::AttrDomain::Instance);
      bke::SpanAttributeWriter<int> body_w = attrs.lookup_or_add_for_write_span<int>(
          attr::body_type, bke::AttrDomain::Instance);

      MutableSpan<int> handles = inst->reference_handles_for_write();
      MutableSpan<float4x4> tfs = inst->transforms_for_write();
      for (const int i : IndexRange(old_n)) {
        if (hull_w) {
          hull_w.span[i] = false;
        }
      }
      for (const int k : IndexRange(add)) {
        Mesh *hull = preview_hull_mesh(compound_islands[k]);
        GeometrySet ref;
        ref.replace_mesh(hull);
        handles[old_n + k] = inst->add_reference(bke::InstanceReference{std::move(ref)});
        tfs[old_n + k] = float4x4::identity();
        if (hull_w) {
          hull_w.span[old_n + k] = true;
        }
        if (shape_w) {
          shape_w.span[old_n + k] = -1;
        }
        if (body_w) {
          body_w.span[old_n + k] = body_type;
        }
      }
      if (hull_w) {
        hull_w.finish();
      }
      if (shape_w) {
        shape_w.finish();
      }
      if (body_w) {
        body_w.finish();
      }
    }
  }

  if (params.output_is_required("Collision"_ustr)) {
    if (shape_type == ShapeType::Compound) {
      params.set_output("Collision"_ustr, preview_compound_islands(compound_islands));
    }
    else {
      params.set_output("Collision"_ustr,
                        build_collision_preview(geometry, shape_type, mesh_half_extents));
    }
  }

  /* Solver side-channel: Mesh + Point Cloud carry baked islands; instances stay visual. */
  if (compound_proxy_mesh) {
    geometry.replace_mesh(compound_proxy_mesh);
    compound_proxy_mesh = nullptr;
  }
  if (shape_type == ShapeType::Compound && !compound_islands.is_empty()) {
    if (PointCloud *pc = box_engine::bake_compound_points(compound_islands)) {
      geometry.replace_pointcloud(pc);
    }
  }

  params.set_output("Geometry"_ustr, std::move(geometry));
}

static void register_box_set_rigid_body()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeBoxEngineSetRigidBody"_ustr, GEO_NODE_BOX_ENGINE_SET_RIGID_BODY);
  ntype.ui_name = "Box Set Rigid Body";
  ntype.ui_description =
      "Body type + collision shape for Box Engine. Mesh-based modes need no Size. "
      "Collision output shows the actual collider (hull / island hulls / triangle mesh)";
  ntype.enum_name_legacy = "BOX_ENGINE_SET_RIGID_BODY";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(register_box_set_rigid_body)

}  // namespace blender::nodes::node_geo_box_engine_set_rigid_body_cc
