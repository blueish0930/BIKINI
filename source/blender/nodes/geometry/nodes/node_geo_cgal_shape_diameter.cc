/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Shape Diameter Function (SDF) as a Face-domain field (context evaluation).
 * Closed mesh required. Distinct from Segmentation (which clusters by SDF).
 */

#include "BKE_mesh.hh"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_shape_diameter_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Float>("Diameter"_ustr)
      .structure_type(StructureType::Field)
      .description("Approximate local diameter / thickness at each face.");
}

static VArray<float> construct_varray(const Mesh &mesh, const AttrDomain domain)
{
  Array<float> values(mesh.faces_num, 0.0f);
  if (mesh.faces_num > 0) {
    std::string error;
    if (!geometry::cgal_mesh_shape_diameter(mesh, values, error)) {
      values.fill(0.0f);
    }
  }
  return mesh.attributes().adapt_domain<float>(
      VArray<float>::from_container(std::move(values)), AttrDomain::Face, domain);
}

class ShapeDiameterFieldInput final : public bke::MeshFieldInput {
 public:
  ShapeDiameterFieldInput()
      : bke::MeshFieldInput(CPPType::get<float>(), "CGAL Shape Diameter Field")
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    return construct_varray(mesh, domain);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep & /*deep_hash_cache*/) const override
  {
    static constexpr int8_t id = 1;
    hash.add(&id);
  }

  std::optional<AttrDomain> preferred_domain(const Mesh & /*mesh*/) const override
  {
    return AttrDomain::Face;
  }

  bke::NativeFieldDomain native_domain_info(const Mesh & /*mesh*/) const override
  {
    return bke::NativeFieldDomain::Domain{AttrDomain::Face};
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  params.set_output("Diameter"_ustr, Field<float>::from_input<ShapeDiameterFieldInput>());
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalShapeDiameter"_ustr, GEO_NODE_CGAL_SHAPE_DIAMETER);
  ntype.ui_name = "Shape Diameter";
  ntype.ui_description =
      "Field node: Shape Diameter Function (local thickness) per face of a closed mesh. Larger values mean thicker regions.";
  ntype.enum_name_legacy = "CGAL_SHAPE_DIAMETER";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_shape_diameter_cc
