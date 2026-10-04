/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "GEO_winding_number.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_winding_number_cc {

enum class Method {
  Fast = 0,
  Exact = 1,
};

static const EnumPropertyItem method_items[] = {
    {int(Method::Fast),
     "FAST",
     0,
     N_("Fast"),
     N_("Hierarchical dipole / quadrupole (Barill et al. 2018). Faster on large meshes.")},
    {int(Method::Exact),
     "EXACT",
     0,
     N_("Exact"),
     N_("Sum the solid angle of every triangle. Slow on large meshes.")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description(
          "Triangle mesh used as the solid. May be open, self-intersecting, or non-manifold.");
  auto &winding = b.add_output<decl::Float>("Winding"_ustr)
                      .align_with_previous()
                      .description(
                          "Generalized winding number at Sample Position. Closed oriented solids "
                          "are ~1 inside and ~0 outside.");
  b.add_input<decl::Menu>("Method"_ustr)
      .static_items(method_items)
      .default_value(Method::Fast)
      .optional_label()
      .description("Fast hierarchical approximation, or exact solid-angle sum.");
  b.add_input<decl::Float>("Accuracy"_ustr)
      .default_value(2.0f)
      .min(1.0f)
      .max(8.0f)
      .usage_by_menu("Method"_ustr, int(Method::Fast))
      .description(
          "How far a cluster must be before it is approximated. Larger is more accurate and "
          "slower. 2 is recommended.");
  auto &sample_position = b.add_input<decl::Vector>("Sample Position"_ustr)
                              .default_input_type(NODE_DEFAULT_INPUT_POSITION_FIELD)
                              .structure_type(StructureType::Dynamic)
                              .description("Where to evaluate. Defaults to the context Position.");
  const std::array<int, 1> dynamic_inputs = {sample_position.index()};
  winding.inferred_structure_type(dynamic_inputs).propagate_references(dynamic_inputs);
}

class WindingNumberFunction : public mf::MultiFunction {
 private:
  GeometrySet target_;
  geometry::WindingNumberMethod method_;
  float accuracy_;
  mf::Signature signature_;

  mutable CacheMutex mutex_;
  mutable geometry::MeshWindingNumber evaluator_;
  mutable bool ok_ = false;

 public:
  WindingNumberFunction(GeometrySet target,
                        const geometry::WindingNumberMethod method,
                        const float accuracy)
      : target_(std::move(target)), method_(method), accuracy_(accuracy)
  {
    target_.ensure_owns_direct_data();
    mf::SignatureBuilder builder{"Winding Number", signature_};
    builder.single_input<float3>("Sample Position");
    builder.single_output<float>("Winding");
    this->set_signature(&signature_);
  }

  void prepare_for_execution() const override
  {
    mutex_.ensure([&]() {
      const Mesh *mesh = target_.get_mesh();
      if (!mesh) {
        return;
      }
      std::string error;
      ok_ = evaluator_.build(*mesh, accuracy_, method_, error);
    });
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float3> &positions = params.readonly_single_input<float3>(0,
                                                                              "Sample Position");
    MutableSpan<float> winding = params.uninitialized_single_output<float>(1, "Winding");
    if (!ok_) {
      index_mask::masked_fill(winding, 0.0f, mask);
      return;
    }
    mask.foreach_index([&](const int i) { winding[i] = evaluator_.eval(positions[i]); });
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    hints.min_grain_size = method_ == geometry::WindingNumberMethod::Exact ? 4 : 64;
    return hints;
  }

  void hash_unique(UniqueHashBytes &hash) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
    hash.add(target_.get_mesh());
    hash.add(method_);
    hash.add(accuracy_);
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet mesh_geo = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Method method = params.extract_input<Method>("Method"_ustr);
  const float accuracy = params.extract_input<float>("Accuracy"_ustr);
  auto sample_position = params.extract_input<bke::SocketValueVariant>("Sample Position"_ustr);

  if (!mesh_geo.get_mesh()) {
    params.set_default_remaining_outputs();
    return;
  }
  if (method == Method::Exact && mesh_geo.get_mesh()->faces_num > 8000) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("Exact sums every triangle at every sample and is very slow on large meshes. Use Fast."));
  }

  const geometry::WindingNumberMethod gmethod = method == Method::Exact ?
                                                    geometry::WindingNumberMethod::Exact :
                                                    geometry::WindingNumberMethod::Fast;

  std::string error_message;
  bke::SocketValueVariant winding;
  if (!execute_multi_function_on_value_variant(
          std::make_shared<WindingNumberFunction>(std::move(mesh_geo), gmethod, accuracy),
          {&sample_position},
          {&winding},
          params.user_data(),
          error_message))
  {
    params.set_default_remaining_outputs();
    params.error_message_add(NodeWarningType::Error, std::move(error_message));
    return;
  }

  params.set_output("Winding"_ustr, std::move(winding));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeWindingNumber"_ustr, GEO_NODE_WINDING_NUMBER);
  ntype.ui_name = "Winding Number";
  ntype.ui_description =
      "Generalized winding number of a triangle mesh at the context position. Works on open, "
      "self-intersecting, and non-manifold meshes.";
  ntype.enum_name_legacy = "WINDING_NUMBER";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_winding_number_cc
