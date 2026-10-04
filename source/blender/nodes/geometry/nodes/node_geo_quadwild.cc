/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * QuadWild Geometry Node — three Mode menu stages (not N-panel):
 *   Remesh — full QRemeshify quadrangulation
 *   Field  — 4-RoSy cross-field attributes only
 *   Layout — patch layout mesh + seam / boundary attributes
 *
 * Runs bundled QRemeshify via qremeshify_bridge.py.
 */

#include "BLI_array.hh"
#include "BLI_math_base.hh"
#include "BLI_math_constants.hh"
#include "BLI_math_matrix.hh"
#include "BLI_vector.hh"

#include "FN_field.hh"

#include "BKE_geometry_fields.hh"

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_quadwild.hh"

#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"

#include "DEG_depsgraph_query.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_quadwild_cc {

enum class Mode {
  Remesh = 0,
  Field = 1,
  Layout = 2,
};

static const EnumPropertyItem mode_items[] = {
    {int(Mode::Remesh),
     "REMESH",
     0,
     N_("Remesh"),
     N_("Full feature-aligned pure-quad remesh (QRemeshify pipeline)")},
    {int(Mode::Field),
     "FIELD",
     0,
     N_("Field"),
     N_("Compute feature-aligned 4-RoSy cross-field only (no layout / remesh)")},
    {int(Mode::Layout),
     "LAYOUT",
     0,
     N_("Layout"),
     N_("Edge-only separatrix from patch IDs (.patch); seam + boundary attributes")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Mesh"_ustr)
      .supported_type(GeometryComponent::Type::Mesh)
      .is_default_link_socket()
      .description("Input surface (triangulated as needed inside QRemeshify prep)");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Remesh: pure quads. Field: mesh with 4-RoSy face attributes. "
          "Layout: patch mesh with seam / boundary attributes");

  b.add_input<decl::Menu>("Mode"_ustr)
      .static_items(mode_items)
      .default_value(Mode::Remesh)
      .optional_label()
      .description("Pipeline stage: full remesh, field only, or patch layout");

  /* Shared. */
  b.add_input<decl::Float>("Sharp Angle"_ustr)
      .default_value(DEG2RADF(35.0f))
      .min(-1.0f)
      .max(DEG2RADF(180.0f))
      .subtype(PROP_ANGLE)
      .description(
          "Dihedral angle for automatic sharp features. Set negative to disable "
          "(organic / no features)");
  b.add_input<decl::Bool>("Remesh Input"_ustr)
      .default_value(true)
      .description("QRemeshify preprocess remesh before field computation");
  b.add_input<decl::Bool>("Hard Edges"_ustr)
      .default_value(false)
      .hide_value()
      .evaluated_geometry_field()
      .description(
          "Extra feature edges (in addition to Sharp Angle). Written as QuadWild sharp features");

  /* Remesh-only. */
  b.add_input<decl::Float>("Scale"_ustr)
      .default_value(1.0f)
      .min(0.01f)
      .max(100.0f)
      .description("Output quad size scale (scaleFact)")
      .usage_by_menu("Mode"_ustr, int(Mode::Remesh));
  b.add_input<decl::Float>("Alpha"_ustr)
      .default_value(0.005f)
      .min(0.0f)
      .max(0.999f)
      .description("QR regularity vs isometry (QRemeshify default 0.005)")
      .usage_by_menu("Mode"_ustr, int(Mode::Remesh));
  b.add_input<decl::Bool>("Smooth Output"_ustr)
      .default_value(true)
      .description("Use smoothed quadrangulation when available")
      .usage_by_menu("Mode"_ustr, int(Mode::Remesh));

  /* Field-only. */
  b.add_input<decl::String>("Attribute Prefix"_ustr)
      .default_value("qw_dir")
      .description("Face attributes prefix for 4 directions: {prefix}0 … {prefix}3")
      .usage_by_menu("Mode"_ustr, int(Mode::Field));

  /* Layout-only. */
  b.add_input<decl::Bool>("Mark Seams"_ustr)
      .default_value(true)
      .description("Mark patch-boundary edges as UV seams")
      .usage_by_menu("Mode"_ustr, int(Mode::Layout));
  b.add_input<decl::String>("Boundary Attribute"_ustr)
      .default_value("qw_layout_boundary")
      .description("Bool edge attribute for layout boundaries (empty string skips)")
      .usage_by_menu("Mode"_ustr, int(Mode::Layout));
}

static void node_init(bNodeTree * /*tree*/, bNode * /*node*/) {}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mode mode = params.extract_input<Mode>("Mode"_ustr);
  const float sharp_angle = params.extract_input<float>("Sharp Angle"_ustr);
  const bool do_remesh = params.extract_input<bool>("Remesh Input"_ustr);
  const Field<bool> hard_edges_field = params.extract_input<Field<bool>>("Hard Edges"_ustr);

  geometry::QuadWildOptions options;
  switch (mode) {
    case Mode::Field:
      options.mode = geometry::QuadWildMode::Field;
      break;
    case Mode::Layout:
      options.mode = geometry::QuadWildMode::Layout;
      break;
    case Mode::Remesh:
    default:
      options.mode = geometry::QuadWildMode::Remesh;
      break;
  }

  if (sharp_angle >= 0.0f) {
    options.sharp_feature_threshold_deg = sharp_angle * (180.0f / float(M_PI));
  }
  else {
    options.sharp_feature_threshold_deg = -1.0f;
  }
  options.do_remesh = do_remesh;
  options.align_singularities = true;
  options.satsuma_config.clear();
  options.flow_config.clear();

  if (mode == Mode::Remesh) {
    options.scale_fact = math::max(params.extract_input<float>("Scale"_ustr), 0.01f);
    options.alpha = math::clamp(params.extract_input<float>("Alpha"_ustr), 0.0f, 0.999f);
    options.smooth_output = params.extract_input<bool>("Smooth Output"_ustr);
  }
  if (mode == Mode::Field) {
    options.field_attribute_prefix = params.extract_input<std::string>("Attribute Prefix"_ustr);
    if (options.field_attribute_prefix.empty()) {
      options.field_attribute_prefix = "qw_dir";
    }
  }
  if (mode == Mode::Layout) {
    options.mark_seams = params.extract_input<bool>("Mark Seams"_ustr);
    options.layout_boundary_attribute = params.extract_input<std::string>(
        "Boundary Attribute"_ustr);
  }

  if (const Object *self_object = params.self_object()) {
    if (DEG_object_transform_is_evaluated(*self_object)) {
      float4x4 mat = self_object->object_to_world();
      mat.location() = float3(0.0f);
      options.rot_scale = mat;
      options.apply_rot_scale = true;
    }
  }

  std::atomic<bool> found_mesh = false;
  std::atomic<bool> empty_result = false;
  std::string last_error;

  if (geometry::quadwild_package_dir_find().empty()) {
    params.error_message_add(
        NodeWarningType::Error,
        TIP_("QRemeshify package not found. Need quadwild/qremeshify_bridge.py "
             "and quadwild/QRemeshify/ next to blender.exe (or QUADWILD_DIR)"));
    params.set_output("Mesh"_ustr, std::move(geometry_set));
    return;
  }

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry) {
    const Mesh *src_mesh = geometry.get_mesh();
    if (!src_mesh) {
      return;
    }
    found_mesh.store(true, std::memory_order_relaxed);

    geometry::QuadWildOptions local = options;
    {
      const bke::MeshFieldContext context(*src_mesh, bke::AttrDomain::Edge);
      fn::FieldEvaluator evaluator(context, src_mesh->edges_num);
      Array<bool> sel(src_mesh->edges_num);
      evaluator.add_with_destination(hard_edges_field, sel.as_mutable_span());
      evaluator.evaluate();
      const Span<int2> edges = src_mesh->edges();
      for (const int e : edges.index_range()) {
        if (sel[e]) {
          local.hard_edges.append(edges[e]);
        }
      }
    }

    geometry::QuadWildResultInfo info;
    Mesh *result = geometry::mesh_quadwild(*src_mesh, local, &info);
    const bool has_expected_output = mode == Mode::Layout ? result->edges_num > 0 :
                                                           result->faces_num > 0;
    if (!info.success || !has_expected_output) {
      empty_result.store(true, std::memory_order_relaxed);
      if (!info.message.empty()) {
        last_error = info.message;
      }
    }
    geometry.replace_mesh(result);
  });

  if (!found_mesh.load(std::memory_order_relaxed)) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Input geometry does not contain a mesh"));
  }
  if (empty_result.load(std::memory_order_relaxed)) {
    if (!last_error.empty()) {
      params.error_message_add(NodeWarningType::Error, last_error);
    }
    else {
      params.error_message_add(NodeWarningType::Warning,
                               TIP_("QuadWild produced no faces for the selected Mode"));
    }
  }

  params.set_output("Mesh"_ustr, std::move(geometry_set));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeQuadWild"_ustr, GEO_NODE_QUADWILD);
  ntype.ui_name = "QuadWild";
  ntype.ui_description =
      "QRemeshify / QuadWild: Remesh (pure quads), Field (4-RoSy), or Layout "
      "(edge-only separatrix from .patch). Mode menu on the node — not the sidebar";
  ntype.enum_name_legacy = "QUADWILD";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  /* No draw_buttons — Mode is a Menu socket, not N-panel. */
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_quadwild_cc
