/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Tag geometry for Jolt Solver + emit collision-shape preview.
 *
 * Mesh is recentered into an Instance so transforms match collision.
 * Shape integers 0–6 match Box Engine so tagged geometry can be reused.
 */

#include "BKE_attribute.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_instances.hh"
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
#include "GEO_mesh_primitive_cylinder_cone.hh"
#include "GEO_mesh_primitive_uv_sphere.hh"

#include "box_engine_shared.hh"
#include "node_geometry_util.hh"

#include <string>

namespace blender::nodes::node_geo_jolt_set_rigid_body_cc {

enum class BodyType {
  Static = 0,
  Kinematic = 1,
  Dynamic = 2,
};

enum class ShapeType {
  Sphere = 0,
  Box = 1,
  Capsule = 2,
  ConvexHull = 3,
  BoundingBox = 4,
  TriangleMesh = 5,
  Compound = 6,
  TaperedCapsule = 7,
  Cylinder = 8,
  TaperedCylinder = 9,
  Plane = 10,
  HeightField = 11,
  Empty = 12,
};

enum class DOFType {
  All = 0,
  PlaneXY = 1,
  PlaneXZ = 2,
  PlaneYZ = 3,
  Translation = 4,
  Rotation = 5,
};

static const EnumPropertyItem body_type_items[] = {
    {int(BodyType::Static), "STATIC", 0, N_("Static"), N_("Immovable collider")},
    {int(BodyType::Kinematic),
     "KINEMATIC",
     0,
     N_("Kinematic"),
     N_("Moved by the node graph; pushes dynamics")},
    {int(BodyType::Dynamic), "DYNAMIC", 0, N_("Dynamic"), N_("Simulated under gravity / forces")},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem shape_type_items[] = {
    {int(ShapeType::Box), "BOX", 0, N_("Box"), N_("Manual box. Size = full XYZ extents")},
    {int(ShapeType::Sphere), "SPHERE", 0, N_("Sphere"), N_("Size.X = diameter")},
    {int(ShapeType::Capsule),
     "CAPSULE",
     0,
     N_("Capsule"),
     N_("Z-up. Size.X = diameter, Size.Y = total height")},
    {int(ShapeType::Cylinder),
     "CYLINDER",
     0,
     N_("Cylinder"),
     N_("Z-up. Size.X = diameter, Size.Y = height")},
    {int(ShapeType::BoundingBox),
     "BOUNDING_BOX",
     0,
     N_("Bounding Box"),
     N_("Box from mesh AABB")},
    {int(ShapeType::ConvexHull),
     "CONVEX_HULL",
     0,
     N_("Convex Hull"),
     N_("Convex hull of mesh vertices")},
    {int(ShapeType::Compound),
     "COMPOUND",
     0,
     N_("Compound"),
     N_("One body: convex hull per mesh island")},
    {int(ShapeType::TriangleMesh),
     "TRIANGLE_MESH",
     0,
     N_("Triangle Mesh"),
     N_("Static collider from mesh triangles. Dynamic uses a convex hull instead")},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem dof_items[] = {
    {int(DOFType::All), "ALL", 0, N_("All"), N_("6 DOF")},
    {int(DOFType::PlaneXY), "PLANE_XY", 0, N_("Plane XY"), N_("2D: X/Y + rot Z")},
    {int(DOFType::PlaneXZ), "PLANE_XZ", 0, N_("Plane XZ"), N_("X/Z + rot Y")},
    {int(DOFType::PlaneYZ), "PLANE_YZ", 0, N_("Plane YZ"), N_("Y/Z + rot X")},
    {int(DOFType::Translation), "TRANSLATION", 0, N_("Translation"), N_("No rotation")},
    {int(DOFType::Rotation), "ROTATION", 0, N_("Rotation"), N_("No translation")},
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
constexpr StringRefNull linear_damping = "linear_damping";
constexpr StringRefNull angular_damping = "angular_damping";
constexpr StringRefNull sensor = "sensor";
constexpr StringRefNull allowed_dofs = "allowed_dofs";
constexpr StringRefNull convex_radius = "convex_radius";
constexpr StringRefNull radius_top = "radius_top";
constexpr StringRefNull ccd = "ccd";
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
      .description("Original visual geometry + rigid-body attributes");
  b.add_output<decl::Geometry>("Collision"_ustr)
      .description("Collision-shape preview. Connect to Viewer");

  b.add_input<decl::Bool>("Selection"_ustr)
      .default_value(true)
      .hide_value()
      .evaluated_geometry_field()
      .description("Which points/instances receive a rigid body");
  b.add_input<decl::String>("Name Prefix"_ustr)
      .default_value("Body")
      .description("body_name prefix. Duplicates get .001, .002, …");

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
  b.add_input<decl::Menu>("DOFs"_ustr)
      .static_items(dof_items)
      .default_value(DOFType::All)
      .optional_label()
      .description("Allowed translation/rotation axes");

  b.add_input<decl::Geometry>("Proxy"_ustr)
      .description("Compound only: optional collision mesh")
      .usage_by_menu("Shape"_ustr, int(ShapeType::Compound));

  b.add_input<decl::Vector>("Size"_ustr)
      .default_value(float3(1.0f, 1.0f, 1.0f))
      .subtype(PROP_XYZ)
      .evaluated_geometry_field()
      .description("Box / Sphere / Capsule / Cylinder extents")
      .usage_by_menu("Shape"_ustr,
                     {int(ShapeType::Box),
                      int(ShapeType::Sphere),
                      int(ShapeType::Capsule),
                      int(ShapeType::Cylinder)});

  b.add_input<decl::Float>("Density"_ustr)
      .default_value(1.0f)
      .min(1e-4f)
      .evaluated_geometry_field()
      .description("Mass density for Dynamic bodies")
      .usage_by_menu("Body Type"_ustr, int(BodyType::Dynamic));
  b.add_input<decl::Float>("Friction"_ustr)
      .default_value(0.5f)
      .min(0.0f)
      .evaluated_geometry_field()
      .description("Coulomb friction coefficient");
  b.add_input<decl::Float>("Restitution"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .max(1.0f)
      .evaluated_geometry_field()
      .description("Bounce coefficient (0 = no bounce, 1 = elastic)");
  b.add_input<decl::Float>("Linear Damping"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .description("Linear velocity damping");
  b.add_input<decl::Float>("Angular Damping"_ustr)
      .default_value(0.25f)
      .min(0.0f)
      .description("Higher values stop residual spin so stacks can rest");
  b.add_input<decl::Float>("Convex Radius"_ustr)
      .default_value(0.05f)
      .min(0.0f)
      .description("Jolt convex radius (rounded edges). 0 = sharp");
  b.add_input<decl::Bool>("Sensor"_ustr)
      .default_value(false)
      .description("Report contacts without resolving them");
  b.add_input<decl::Bool>("CCD"_ustr).default_value(false).description("LinearCast motion quality");
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

static float3 half_from_size(const float3 full, const ShapeType shape)
{
  if (shape == ShapeType::Sphere) {
    const float r = math::max(full.x * 0.5f, 1e-5f);
    return float3(r);
  }
  if (ELEM(shape, ShapeType::Capsule, ShapeType::Cylinder, ShapeType::TaperedCapsule,
           ShapeType::TaperedCylinder))
  {
    /* x = radius, y = half-height (Z-up after the solver rotates Jolt's Y-axis). */
    return float3(math::max(full.x * 0.5f, 1e-5f),
                  math::max(full.y * 0.5f, 1e-5f),
                  math::max(full.x * 0.5f, 1e-5f));
  }
  return math::max(full * 0.5f, float3(1e-5f));
}

static Mesh *preview_box_mesh(const float3 half)
{
  return geometry::create_cuboid_mesh(
      float3(half.x * 2.0f, half.y * 2.0f, half.z * 2.0f), 2, 2, 2);
}

static Mesh *preview_sphere_mesh(const float radius)
{
  return geometry::create_uv_sphere_mesh(radius, 8, 12, {});
}

static Mesh *preview_cylinder_mesh(const float radius, const float half_height)
{
  geometry::ConeAttributeOutputs dummy;
  return geometry::create_cylinder_or_cone_mesh(radius,
                                                radius,
                                                math::max(half_height * 2.0f, 1e-4f),
                                                12,
                                                1,
                                                1,
                                                geometry::ConeFillType::NGon,
                                                dummy);
}

static void write_attrs(bke::MutableAttributeAccessor attributes,
                        const bke::AttrDomain domain,
                        const int size,
                        const fn::FieldContext &context,
                        const Field<bool> &selection_field,
                        const int body_type,
                        const int shape_type,
                        const int dofs,
                        const Field<float3> &size_field,
                        const Field<float> &density_field,
                        const Field<float> &friction_field,
                        const Field<float> &restitution_field,
                        const float3 mesh_half,
                        const float lin_damp,
                        const float ang_damp,
                        const float convex_r,
                        const bool sensor,
                        const bool ccd)
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
  if (selection.is_empty()) {
    return;
  }
  const VArray<float3> sizes_in = evaluator.get_evaluated<float3>(0);
  const VArray<float> densities = evaluator.get_evaluated<float>(1);
  const VArray<float> frictions = evaluator.get_evaluated<float>(2);
  const VArray<float> restitutions = evaluator.get_evaluated<float>(3);

  auto span_i = [&](StringRef name) {
    return attributes.lookup_or_add_for_write_span<int>(name, domain);
  };
  auto span_f = [&](StringRef name) {
    return attributes.lookup_or_add_for_write_span<float>(name, domain);
  };
  auto span_v = [&](StringRef name) {
    return attributes.lookup_or_add_for_write_span<float3>(name, domain);
  };
  auto span_b = [&](StringRef name) {
    return attributes.lookup_or_add_for_write_span<bool>(name, domain);
  };

  bke::SpanAttributeWriter<float3> size_w = span_v(attr::shape_size);
  bke::SpanAttributeWriter<float> dens_w = span_f(attr::density);
  bke::SpanAttributeWriter<float> fric_w = span_f(attr::friction);
  bke::SpanAttributeWriter<float> rest_w = span_f(attr::restitution);
  bke::SpanAttributeWriter<float> rad_w = span_f(attr::radius);
  bke::SpanAttributeWriter<float> ld_w = span_f(attr::linear_damping);
  bke::SpanAttributeWriter<float> ad_w = span_f(attr::angular_damping);
  bke::SpanAttributeWriter<float> cr_w = span_f(attr::convex_radius);
  bke::SpanAttributeWriter<float> rt_w = span_f(attr::radius_top);
  bke::SpanAttributeWriter<bool> sensor_w = span_b(attr::sensor);
  bke::SpanAttributeWriter<bool> ccd_w = span_b(attr::ccd);
  bke::SpanAttributeWriter<int> dof_w = span_i(attr::allowed_dofs);
  bke::SpanAttributeWriter<int> body_w = span_i(attr::body_type);
  bke::SpanAttributeWriter<int> shape_w = span_i(attr::shape_type);

  const bool mesh_driven = ELEM(shape_type,
                                int(ShapeType::BoundingBox),
                                int(ShapeType::ConvexHull),
                                int(ShapeType::TriangleMesh),
                                int(ShapeType::Compound),
                                int(ShapeType::HeightField));

  selection.foreach_index([&](const int i) {
    if (body_w) {
      body_w.span[i] = body_type;
    }
    if (shape_w) {
      shape_w.span[i] = shape_type;
    }
    float3 half = mesh_half;
    if (!mesh_driven) {
      half = half_from_size(sizes_in[i], ShapeType(shape_type));
    }
    half = math::max(half, float3(1e-5f));
    if (size_w) {
      size_w.span[i] = half;
    }
    if (dens_w) {
      dens_w.span[i] = math::max(densities[i], (body_type == int(BodyType::Dynamic)) ? 1e-3f : 0.0f);
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
    if (ld_w) {
      ld_w.span[i] = lin_damp;
    }
    if (ad_w) {
      ad_w.span[i] = ang_damp;
    }
    if (cr_w) {
      cr_w.span[i] = convex_r;
    }
    if (rt_w) {
      rt_w.span[i] = half.z;
    }
    if (sensor_w) {
      sensor_w.span[i] = sensor;
    }
    if (ccd_w) {
      ccd_w.span[i] = ccd;
    }
    if (dof_w) {
      dof_w.span[i] = dofs;
    }
  });
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
  if (ld_w) {
    ld_w.finish();
  }
  if (ad_w) {
    ad_w.finish();
  }
  if (cr_w) {
    cr_w.finish();
  }
  if (rt_w) {
    rt_w.finish();
  }
  if (sensor_w) {
    sensor_w.finish();
  }
  if (ccd_w) {
    ccd_w.finish();
  }
  if (dof_w) {
    dof_w.finish();
  }
  if (body_w) {
    body_w.finish();
  }
  if (shape_w) {
    shape_w.finish();
  }
}

static GeometrySet simple_preview(const GeometrySet &geometry, const ShapeType shape, const float3 half)
{
  GeometrySet preview;
  auto mesh_for = [&]() -> Mesh * {
    if (shape == ShapeType::Sphere) {
      return preview_sphere_mesh(half.x);
    }
    if (shape == ShapeType::Capsule) {
      return preview_cylinder_mesh(half.x, math::max(half.y, half.x));
    }
    if (shape == ShapeType::Cylinder || shape == ShapeType::TaperedCylinder ||
        shape == ShapeType::TaperedCapsule)
    {
      return preview_cylinder_mesh(half.x, half.y);
    }
    return preview_box_mesh(half);
  };
  if (const bke::Instances *src = geometry.get_instances()) {
    std::unique_ptr<bke::Instances> out = std::make_unique<bke::Instances>();
    GeometrySet ref;
    ref.replace_mesh(mesh_for());
    const int h = out->add_reference(bke::InstanceReference{std::move(ref)});
    const int n = src->instances_num();
    out->resize(n);
    MutableSpan<int> handles = out->reference_handles_for_write();
    MutableSpan<float4x4> tfs = out->transforms_for_write();
    const Span<float4x4> src_tf = src->transforms();
    for (int i = 0; i < n; i++) {
      handles[i] = h;
      tfs[i] = src_tf[i];
    }
    preview.replace_instances(out.release());
    return preview;
  }
  preview.replace_mesh(mesh_for());
  return preview;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry"_ustr);
  GeometrySet proxy_geo = params.extract_input<GeometrySet>("Proxy"_ustr);
  float3 mesh_half(0.5f);
  geometry = mesh_to_centered_instance(std::move(geometry), mesh_half);

  const int body_type = int(params.extract_input<BodyType>("Body Type"_ustr));
  const ShapeType shape_type = params.extract_input<ShapeType>("Shape"_ustr);
  const int dofs = int(params.extract_input<DOFType>("DOFs"_ustr));
  const Field<bool> selection = params.extract_input<Field<bool>>("Selection"_ustr);
  const std::string name_prefix = params.extract_input<std::string>("Name Prefix"_ustr);
  const Field<float3> size_field = params.extract_input<Field<float3>>("Size"_ustr);
  const Field<float> density_field = params.extract_input<Field<float>>("Density"_ustr);
  const Field<float> friction = params.extract_input<Field<float>>("Friction"_ustr);
  const Field<float> restitution = params.extract_input<Field<float>>("Restitution"_ustr);
  const float lin_damp = params.extract_input<float>("Linear Damping"_ustr);
  const float ang_damp = params.extract_input<float>("Angular Damping"_ustr);
  const float convex_r = params.extract_input<float>("Convex Radius"_ustr);
  const bool sensor = params.extract_input<bool>("Sensor"_ustr);
  const bool ccd = params.extract_input<bool>("CCD"_ustr);

  Vector<Vector<float3>> compound_islands;
  Mesh *compound_proxy_mesh = nullptr;
  if (shape_type == ShapeType::Compound) {
    if (proxy_geo.has_mesh() || proxy_geo.has_instances()) {
      float3 proxy_half(0.5f);
      GeometrySet proxy_centered = mesh_to_centered_instance(std::move(proxy_geo), proxy_half);
      box_engine::collect_islands_from_geometry(proxy_centered, compound_islands);
      mesh_half = proxy_half;
    }
    else {
      box_engine::collect_islands_from_geometry(geometry, compound_islands);
    }
    compound_proxy_mesh = box_engine::bake_compound_islands(compound_islands);
  }

  if (bke::Instances *instances = geometry.get_instances_for_write()) {
    const int n = instances->instances_num();
    bke::InstancesFieldContext context(*instances);
    write_attrs(instances->attributes_for_write(),
                bke::AttrDomain::Instance,
                n,
                context,
                selection,
                body_type,
                int(shape_type),
                dofs,
                size_field,
                density_field,
                friction,
                restitution,
                mesh_half,
                lin_damp,
                ang_damp,
                convex_r,
                sensor,
                ccd);
    box_engine::ensure_unique_body_names(
        instances->attributes_for_write(), bke::AttrDomain::Instance, n, name_prefix);
  }
  else if (PointCloud *points = geometry.get_pointcloud_for_write()) {
    bke::PointCloudFieldContext context(*points);
    write_attrs(points->attributes_for_write(),
                bke::AttrDomain::Point,
                points->totpoint,
                context,
                selection,
                body_type,
                int(shape_type),
                dofs,
                size_field,
                density_field,
                friction,
                restitution,
                mesh_half,
                lin_damp,
                ang_damp,
                convex_r,
                sensor,
                ccd);
    box_engine::ensure_unique_body_names(
        points->attributes_for_write(), bke::AttrDomain::Point, points->totpoint, name_prefix);
  }

  if (params.output_is_required("Collision"_ustr)) {
    if (shape_type == ShapeType::Compound && !compound_islands.is_empty()) {
      GeometrySet prev;
      prev.replace_mesh(box_engine::bake_compound_islands(compound_islands));
      params.set_output("Collision"_ustr, std::move(prev));
    }
    else {
      params.set_output("Collision"_ustr, simple_preview(geometry, shape_type, mesh_half));
    }
  }
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

static void register_jolt_set_rigid_body()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeJoltSetRigidBody"_ustr, GEO_NODE_JOLT_SET_RIGID_BODY);
  ntype.ui_name = "Jolt Set Rigid Body";
  ntype.ui_description =
      "Body type + collision shape. Static bodies are colliders in Jolt Solver. "
      "Capsule/Cylinder are Z-up. Triangle Mesh is static (Dynamic uses a hull)";
  ntype.enum_name_legacy = "JOLT_SET_RIGID_BODY";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(register_jolt_set_rigid_body)

}  // namespace blender::nodes::node_geo_jolt_set_rigid_body_cc
