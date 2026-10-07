/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include <fmt/format.h>

#include "BLI_array_utils.hh"

#include "GEO_mesh_laplacian.hh"
#include "GEO_mesh_triangulate.hh"

#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_wrangle_array.hh"

#include "DNA_mesh_types.h"
#include "DNA_node_types.h"

#include "NOD_geometry_nodes_bundle.hh"
#include "NOD_geometry_nodes_bundle_signature.hh"
#include "NOD_geometry_nodes_list.hh"
#include "NOD_rna_define.hh"
#include "NOD_socket_usage_inference.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_mesh_laplacian_cc {

enum class WeightMode {
  Uniform = 0,
  Cotangent = 1,
  Custom = 2,
};

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();

  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description(
          "Triangle mesh used to build the discrete Laplacian; other faces are triangulated");
  auto &weight_attribute =
      b.add_input<decl::String>("Weight Attribute"_ustr)
          .default_value("weight")
          .is_attribute_name()
          .optional_label()
          .description(
              "Float attribute, or float array written by a Wrangle node (f[]@), on the point or "
              "face corner domain with the values of the matrix entries. The row of an entry is "
              "the vertex of its point or face corner");
  auto &column_attribute =
      b.add_input<decl::String>("Column Index Attribute"_ustr)
          .default_value("col")
          .is_attribute_name()
          .optional_label()
          .description(
              "Integer attribute, or integer array written by a Wrangle node (i[]@), on the same "
              "domain as the weights with the vertex indices that are the columns of the matrix "
              "entries");
  if (node != nullptr) {
    const bool is_custom = WeightMode(node->custom1) == WeightMode::Custom;
    weight_attribute.available(is_custom);
    column_attribute.available(is_custom);
  }
  b.add_input<decl::Bool>("Diffusion Matrix"_ustr)
      .default_value(false)
      .description("Output I+tL, or M+tML when Use Mass is enabled");
  b.add_input<decl::Float>("Diffusion Coefficient"_ustr)
      .default_value(1.0f)
      .description("Scalar t used when building a diffusion matrix")
      .usage_inference([](const socket_usage_inference::SocketUsageParams &params) {
        /* Hide when Diffusion Matrix is off. */
        return params.bool_input_may_be("Diffusion Matrix"_ustr, true);
      });
  b.add_input<decl::Bool>("Use Mass"_ustr)
      .default_value(false)
      .description(
          "Include lumped mass M (face area/3). Without diffusion yields ML; with diffusion "
          "yields M+tML");
  b.add_output<decl::Bundle>("Matrix"_ustr)
      .create_signature([](const bNode &) { return BundleSignature::sparse_coo(); })
      .structure_type(StructureType::Single)
      .description(
          "Sparse COO matrix as a bundle of equal-length lists: weight (float), row (int), col "
          "(int)");
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

static geometry::MeshLaplacianWeightMode to_geometry_mode(const WeightMode mode)
{
  switch (mode) {
    case WeightMode::Uniform:
      return geometry::MeshLaplacianWeightMode::Uniform;
    case WeightMode::Cotangent:
      return geometry::MeshLaplacianWeightMode::Cotangent;
    case WeightMode::Custom:
      return geometry::MeshLaplacianWeightMode::Custom;
  }
  return geometry::MeshLaplacianWeightMode::Cotangent;
}

static BundlePtr empty_matrix_bundle()
{
  BundlePtr bundle_ptr = Bundle::create();
  Bundle &bundle = bundle_ptr.ensure_mutable_inplace();
  const bke::bNodeSocketType *float_type = bke::node_socket_type_find_static(SOCK_FLOAT);
  const bke::bNodeSocketType *int_type = bke::node_socket_type_find_static(SOCK_INT);
  bundle.add(
      *BundleKey::from_str("weight"),
      BundleItemSocketValue{float_type,
                            bke::SocketValueVariant::From(GList::from_container(Array<float>()))});
  bundle.add(
      *BundleKey::from_str("row"),
      BundleItemSocketValue{int_type,
                            bke::SocketValueVariant::From(GList::from_container(Array<int>()))});
  bundle.add(
      *BundleKey::from_str("col"),
      BundleItemSocketValue{int_type,
                            bke::SocketValueVariant::From(GList::from_container(Array<int>()))});
  return bundle_ptr;
}

static BundlePtr coo_to_bundle(geometry::MeshLaplacianCOO &&coo)
{
  BundlePtr bundle_ptr = Bundle::create();
  Bundle &bundle = bundle_ptr.ensure_mutable_inplace();
  const bke::bNodeSocketType *float_type = bke::node_socket_type_find_static(SOCK_FLOAT);
  const bke::bNodeSocketType *int_type = bke::node_socket_type_find_static(SOCK_INT);

  Array<float> weights(coo.weights.as_span());
  Array<int> rows(coo.rows.as_span());
  Array<int> cols(coo.cols.as_span());

  bundle.add(*BundleKey::from_str("weight"),
             BundleItemSocketValue{
                 float_type,
                 bke::SocketValueVariant::From(GList::from_container(std::move(weights)))});
  bundle.add(
      *BundleKey::from_str("row"),
      BundleItemSocketValue{int_type,
                            bke::SocketValueVariant::From(GList::from_container(std::move(rows)))});
  bundle.add(
      *BundleKey::from_str("col"),
      BundleItemSocketValue{int_type,
                            bke::SocketValueVariant::From(GList::from_container(std::move(cols)))});
  return bundle_ptr;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh"_ustr);
  const bool build_diffusion = params.extract_input<bool>("Diffusion Matrix"_ustr);
  const float diffusion_t = params.extract_input<float>("Diffusion Coefficient"_ustr);
  const bool use_mass = params.extract_input<bool>("Use Mass"_ustr);
  const WeightMode mode = WeightMode(params.node().custom1);
  std::string weight_name;
  std::string column_name;
  if (mode == WeightMode::Custom) {
    weight_name = params.extract_input<std::string>("Weight Attribute"_ustr);
    column_name = params.extract_input<std::string>("Column Index Attribute"_ustr);
  }

  const Mesh *src_mesh = geometry_set.get_mesh();
  if (!src_mesh || src_mesh->verts_num == 0) {
    if (!src_mesh) {
      params.error_message_add(NodeWarningType::Warning,
                               TIP_("Input geometry does not contain a mesh"));
    }
    params.set_output("Matrix"_ustr, empty_matrix_bundle());
    return;
  }

  /* Custom weights are read from the original mesh, triangulation keeps the vertices. */
  VArraySpan<float> custom_weights;
  VArraySpan<int> custom_cols;
  Span<int> custom_rows;
  Array<int> point_rows;
  Vector<float> array_weights;
  Vector<int> array_cols;
  Vector<int> array_rows;
  if (mode == WeightMode::Custom) {
    const bke::AttributeAccessor attributes = src_mesh->attributes();
    const std::optional<bke::AttributeMetaData> weight_meta = attributes.lookup_meta_data(
        weight_name);
    const std::optional<bke::AttributeMetaData> column_meta = attributes.lookup_meta_data(
        column_name);
    if (!weight_meta || !column_meta) {
      params.error_message_add(NodeWarningType::Error,
                               fmt::format(fmt::runtime(TIP_("Attribute does not exist: \"{}\"")),
                                           weight_meta ? column_name : weight_name));
      params.set_output("Matrix"_ustr, empty_matrix_bundle());
      return;
    }
    const bke::AttrDomain domain = weight_meta->domain;
    if (!ELEM(domain, bke::AttrDomain::Point, bke::AttrDomain::Corner)) {
      params.error_message_add(
          NodeWarningType::Error,
          TIP_("The weight attribute must be on the point or face corner domain"));
      params.set_output("Matrix"_ustr, empty_matrix_bundle());
      return;
    }
    if (column_meta->domain != domain) {
      params.error_message_add(
          NodeWarningType::Error,
          TIP_("The weight and column index attributes must be on the same domain"));
      params.set_output("Matrix"_ustr, empty_matrix_bundle());
      return;
    }
    const bool weight_is_array = weight_meta->data_type == bke::AttrType::WrangleArray;
    const bool column_is_array = column_meta->data_type == bke::AttrType::WrangleArray;
    if (weight_is_array != column_is_array) {
      params.error_message_add(
          NodeWarningType::Error,
          TIP_("The weight and column index attributes must both be arrays or both be single "
               "values"));
      params.set_output("Matrix"_ustr, empty_matrix_bundle());
      return;
    }
    const Span<int> corner_verts = src_mesh->corner_verts();
    if (weight_is_array) {
      /* Arrays written by a Wrangle node (`f[]@weight`, `i[]@col`): one entry per array item. */
      const VArraySpan<bke::WrangleArrayValue> weights =
          attributes.lookup(weight_name).varray.typed<bke::WrangleArrayValue>();
      const VArraySpan<bke::WrangleArrayValue> cols =
          attributes.lookup(column_name).varray.typed<bke::WrangleArrayValue>();
      const auto is_number_list = [](const bke::WrangleArrayValue &value) {
        return value.count == 0 ||
               ELEM(value.kind, bke::WrangleArrayKind::Int, bke::WrangleArrayKind::Float);
      };
      bool wrong_kind = false;
      bool different_lengths = false;
      for (const int i : weights.index_range()) {
        const bke::WrangleArrayValue &weight = weights[i];
        const bke::WrangleArrayValue &col = cols[i];
        if (!is_number_list(weight) || !is_number_list(col)) {
          wrong_kind = true;
          continue;
        }
        different_lengths |= weight.count != col.count;
        const int row = domain == bke::AttrDomain::Point ? i : corner_verts[i];
        for (const int k : IndexRange(std::min(weight.count, col.count))) {
          array_weights.append(weight.kind == bke::WrangleArrayKind::Float ? weight.d.f[k] :
                                                                              float(weight.d.i[k]));
          array_cols.append(col.kind == bke::WrangleArrayKind::Int ? col.d.i[k] :
                                                                     int(col.d.f[k]));
          array_rows.append(row);
        }
      }
      if (wrong_kind) {
        params.error_message_add(
            NodeWarningType::Warning,
            TIP_("Array attributes must be float or integer arrays, other arrays were ignored"));
      }
      if (different_lengths) {
        params.error_message_add(
            NodeWarningType::Warning,
            TIP_("The weight and column index arrays have different lengths on some elements, "
                 "the extra items were ignored"));
      }
      custom_weights = VArray<float>::from_span(array_weights.as_span());
      custom_cols = VArray<int>::from_span(array_cols.as_span());
      custom_rows = array_rows;
    }
    else {
      const bke::AttributeReader<float> weights = attributes.lookup<float>(weight_name, domain);
      const bke::AttributeReader<int> cols = attributes.lookup<int>(column_name, domain);
      if (!weights || !cols) {
        params.error_message_add(
            NodeWarningType::Error,
            TIP_("The weight and column index attributes must be convertible to float and "
                 "integer"));
        params.set_output("Matrix"_ustr, empty_matrix_bundle());
        return;
      }
      custom_weights = *weights;
      custom_cols = *cols;
      if (domain == bke::AttrDomain::Point) {
        point_rows.reinitialize(src_mesh->verts_num);
        array_utils::fill_index_range<int>(point_rows);
        custom_rows = point_rows;
      }
      else {
        custom_rows = corner_verts;
      }
    }
    const IndexRange verts_range(src_mesh->verts_num);
    if (std::any_of(custom_cols.begin(), custom_cols.end(), [&](const int col) {
          return !verts_range.contains(col);
        }))
    {
      params.error_message_add(
          NodeWarningType::Warning,
          TIP_("Entries with a column index outside of the vertex range were ignored"));
    }
  }

  /* With custom weights the triangles are only needed for the mass. */
  const bool needs_triangles = mode != WeightMode::Custom || use_mass;

  const Mesh *triangle_mesh = src_mesh;
  Mesh *owned_triangle_mesh = nullptr;
  if (needs_triangles && src_mesh->corners_num != src_mesh->faces_num * 3) {
    const IndexMask selection(src_mesh->faces_num);
    if (std::optional<Mesh *> result = geometry::mesh_triangulate(
            *src_mesh,
            selection,
            geometry::TriangulateNGonMode::Beauty,
            geometry::TriangulateQuadMode::ShortEdge,
            bke::AttributeFilter{}))
    {
      owned_triangle_mesh = *result;
      triangle_mesh = owned_triangle_mesh;
      params.error_message_add(
          NodeWarningType::Info,
          TIP_("Non-triangle faces were triangulated before building the Laplacian"));
    }
    else {
      params.error_message_add(NodeWarningType::Error, TIP_("Failed to triangulate mesh"));
      params.set_output("Matrix"_ustr, empty_matrix_bundle());
      return;
    }
  }

  geometry::MeshLaplacianOptions options;
  options.mode = to_geometry_mode(mode);
  options.build_diffusion = build_diffusion;
  options.diffusion_t = diffusion_t;
  options.use_mass = use_mass;
  options.custom_weights = custom_weights;
  options.custom_rows = custom_rows;
  options.custom_cols = custom_cols;

  geometry::MeshLaplacianCOO coo = geometry::mesh_laplacian_from_mesh(*triangle_mesh, options);

  if (owned_triangle_mesh) {
    BKE_id_free(nullptr, owned_triangle_mesh);
  }

  params.set_output("Matrix"_ustr, coo_to_bundle(std::move(coo)));
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem mode_items[] = {
      {int(WeightMode::Uniform),
       "UNIFORM",
       0,
       "Uniform",
       "Graph Laplacian with equal weight −1 on each unique edge"},
      {int(WeightMode::Cotangent),
       "COTANGENT",
       0,
       "Cotangent",
       "Cotangent Laplacian using opposite-angle cot weights on triangle edges"},
      {int(WeightMode::Custom),
       "CUSTOM",
       0,
       "Custom",
       "Matrix from attributes, used exactly as given without changing the sign or adding a "
       "diagonal: every point or face corner writes its values to the row of its vertex and the "
       "columns stored in the column index attribute. Both attributes are either single values "
       "or Wrangle arrays"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "Discrete Laplacian weight formula",
                    mode_items,
                    NOD_inline_enum_accessors(custom1),
                    int(WeightMode::Cotangent));
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeMeshLaplacian"_ustr, GEO_NODE_MESH_LAPLACIAN);
  ntype.ui_name = "Mesh Laplacian";
  ntype.ui_description =
      "Build a sparse discrete Laplacian (or diffusion) matrix for a triangle mesh as "
      "weight/row/col lists in a bundle";
  ntype.enum_name_legacy = "MESH_LAPLACIAN";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_mesh_laplacian_cc
