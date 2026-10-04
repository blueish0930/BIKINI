/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_mesh_laplacian.hh"
#include "GEO_mesh_triangulate.hh"

#include "BKE_lib_id.hh"

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
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description(
          "Triangle mesh used to build the discrete Laplacian; other faces are triangulated");
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

  const Mesh *src_mesh = geometry_set.get_mesh();
  if (!src_mesh || src_mesh->verts_num == 0) {
    if (!src_mesh) {
      params.error_message_add(NodeWarningType::Warning,
                               TIP_("Input geometry does not contain a mesh"));
    }
    params.set_output("Matrix"_ustr, empty_matrix_bundle());
    return;
  }

  const Mesh *triangle_mesh = src_mesh;
  Mesh *owned_triangle_mesh = nullptr;
  if (src_mesh->corners_num != src_mesh->faces_num * 3) {
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
