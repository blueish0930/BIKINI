/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "DNA_node_types.h"
#include "FN_field.hh"
#include "GEO_cgal.hh"
#include "NOD_rna_define.hh"
#include "RNA_define.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"
#include "node_geometry_util.hh"

#include <algorithm>

namespace blender::nodes::node_geo_cgal_fair_cc {

/** Hard safety cap; the N-panel value (default 3) is the user-facing clamp. */
static constexpr int FAIR_CONTINUITY_HARD_MAX = 8;
static constexpr int FAIR_CONTINUITY_DEFAULT_MAX = 3;

static int fair_max_continuity(const bNode &node)
{
  return std::clamp(int(node.custom1), 0, FAIR_CONTINUITY_HARD_MAX);
}

static void node_declare(NodeDeclarationBuilder &b)
{
  int max_continuity = FAIR_CONTINUITY_DEFAULT_MAX;
  if (const bNode *node = b.node_or_null()) {
    max_continuity = fair_max_continuity(*node);
  }

  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Input mesh (topology unchanged; only free verts move).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Faired mesh (same topology, smoothed free verts).");
  b.add_input<decl::Bool>("Selection"_ustr)
      .default_value(true)
      .hide_value()
      .evaluated_geometry_field()
      .description(
          "Vertices to fair. Unselected stay fixed. Boundary loops are always pinned.");
  b.add_input<decl::Int>("Continuity"_ustr)
      .default_value(1)
      .min(0)
      .max(max_continuity)
      .description(
          "0 = keep boundary positions (Laplace). "
          "1 = also keep the original 1-ring (tangent). "
          "2 = also keep the original 2-ring (curvature). "
          "Raise the N-panel max to allow higher orders.");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = FAIR_CONTINUITY_DEFAULT_MAX;
}

static void node_layout_ex(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "max_continuity", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const int max_continuity = fair_max_continuity(params.node());
  const int continuity = std::clamp(
      params.extract_input<int>("Continuity"_ustr), 0, max_continuity);
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  Array<uint8_t> free_mask(mesh_in->verts_num, 1);
  {
    const bke::MeshFieldContext context(*mesh_in, bke::AttrDomain::Point);
    fn::FieldEvaluator evaluator(context, mesh_in->verts_num);
    Array<bool> sel(mesh_in->verts_num);
    evaluator.add_with_destination(selection_field, sel.as_mutable_span());
    evaluator.evaluate();
    for (const int i : IndexRange(mesh_in->verts_num)) {
      free_mask[i] = sel[i] ? uint8_t(1) : uint8_t(0);
    }
  }

  std::string error;
  Mesh *mesh = geometry::cgal_mesh_fair(*mesh_in, continuity, error, free_mask.as_span());
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(
        NodeWarningType::Warning, error.empty() ? TIP_("Fair failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static int rna_fair_max_continuity_get(PointerRNA *ptr, PropertyRNA * /*prop*/)
{
  const bNode &node = *static_cast<const bNode *>(ptr->data);
  return fair_max_continuity(node);
}

static void rna_fair_max_continuity_set(PointerRNA *ptr, PropertyRNA * /*prop*/, const int value)
{
  bNode &node = *static_cast<bNode *>(ptr->data);
  node.custom1 = int16_t(std::clamp(value, 0, FAIR_CONTINUITY_HARD_MAX));
}

static void node_rna(StructRNA *srna)
{
  PropertyRNA *prop = RNA_def_property(srna, "max_continuity", PROP_INT, PROP_NONE);
  RNA_def_property_int_funcs_runtime(
      prop, rna_fair_max_continuity_get, rna_fair_max_continuity_set, nullptr, nullptr, nullptr);
  RNA_def_property_range(prop, 0, FAIR_CONTINUITY_HARD_MAX);
  RNA_def_property_ui_range(prop, 0, FAIR_CONTINUITY_HARD_MAX, 1, -1);
  RNA_def_property_int_default(prop, FAIR_CONTINUITY_DEFAULT_MAX);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(
      prop,
      "Max Continuity",
      "Highest Continuity order the node socket accepts. "
      "k pins the original k-ring at the selection/port boundary");
  RNA_def_property_update_runtime(prop, rna_Node_socket_update);
  RNA_def_property_update_notifier(prop, NC_NODE | NA_EDITED);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalFair"_ustr, GEO_NODE_CGAL_FAIR);
  ntype.ui_name = "Fair";
  ntype.ui_description =
      "Discrete k-harmonic fairing (Botsch/Kobbelt). "
      "Continuity k pins the original k-ring, then solves L^{k+1}x=0.";
  ntype.enum_name_legacy = "CGAL_FAIR";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
  if (ntype.rna_ext.srna) {
    node_rna(ntype.rna_ext.srna);
  }
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_fair_cc
