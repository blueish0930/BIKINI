/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL Principal_curvatures_and_directions as Point-domain fields:
 *   Min / Max          — principal curvature scalars (k_min, k_max)
 *   Min Direction /
 *   Max Direction      — unit direction vectors
 */

#include "BKE_mesh.hh"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_principal_curvature_cc {

enum class Channel {
  KMin = 0,
  KMax = 1,
  DMin = 2,
  DMax = 3,
};

struct PrincipalData {
  Array<float> kmin;
  Array<float> kmax;
  Array<float3> dmin;
  Array<float3> dmax;

  static PrincipalData compute(const Mesh &mesh)
  {
    PrincipalData d;
    d.kmin = Array<float>(mesh.verts_num, 0.0f);
    d.kmax = Array<float>(mesh.verts_num, 0.0f);
    d.dmin = Array<float3>(mesh.verts_num, float3(0.0f));
    d.dmax = Array<float3>(mesh.verts_num, float3(0.0f));
    if (mesh.verts_num > 0 && mesh.faces_num > 0) {
      std::string error;
      if (!geometry::cgal_mesh_principal_curvatures(mesh, d.kmin, d.kmax, d.dmin, d.dmax, error)) {
        d.kmin.fill(0.0f);
        d.kmax.fill(0.0f);
        d.dmin.fill(float3(0.0f));
        d.dmax.fill(float3(0.0f));
      }
    }
    return d;
  }
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Float>("Min"_ustr)
      .structure_type(StructureType::Field)
      .description("Smaller principal curvature k_min.");
  b.add_output<decl::Float>("Max"_ustr)
      .structure_type(StructureType::Field)
      .description("Larger principal curvature k_max.");
  b.add_output<decl::Vector>("Min Direction"_ustr)
      .structure_type(StructureType::Field)
      .description("Unit tangent direction of k_min.");
  b.add_output<decl::Vector>("Max Direction"_ustr)
      .structure_type(StructureType::Field)
      .description("Unit tangent direction of k_max.");
}

class PrincipalScalarFieldInput final : public bke::MeshFieldInput {
 private:
  Channel channel_;

 public:
  explicit PrincipalScalarFieldInput(const Channel channel)
      : bke::MeshFieldInput(CPPType::get<float>(), "CGAL Principal Curvature Scalar"),
        channel_(channel)
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    PrincipalData d = PrincipalData::compute(mesh);
    Array<float> values = (channel_ == Channel::KMin) ? std::move(d.kmin) : std::move(d.kmax);
    return mesh.attributes().adapt_domain<float>(
        VArray<float>::from_container(std::move(values)), AttrDomain::Point, domain);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep & /*deep_hash_cache*/) const override
  {
    static constexpr int8_t kmin_id = 1;
    static constexpr int8_t kmax_id = 2;
    hash.add(channel_ == Channel::KMin ? &kmin_id : &kmax_id);
  }

  std::optional<AttrDomain> preferred_domain(const Mesh & /*mesh*/) const override
  {
    return AttrDomain::Point;
  }

  bke::NativeFieldDomain native_domain_info(const Mesh & /*mesh*/) const override
  {
    return bke::NativeFieldDomain::Domain{AttrDomain::Point};
  }
};

class PrincipalDirectionFieldInput final : public bke::MeshFieldInput {
 private:
  Channel channel_;

 public:
  explicit PrincipalDirectionFieldInput(const Channel channel)
      : bke::MeshFieldInput(CPPType::get<float3>(), "CGAL Principal Curvature Direction"),
        channel_(channel)
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    PrincipalData d = PrincipalData::compute(mesh);
    Array<float3> values = (channel_ == Channel::DMin) ? std::move(d.dmin) : std::move(d.dmax);
    return mesh.attributes().adapt_domain<float3>(
        VArray<float3>::from_container(std::move(values)), AttrDomain::Point, domain);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep & /*deep_hash_cache*/) const override
  {
    static constexpr int8_t dmin_id = 3;
    static constexpr int8_t dmax_id = 4;
    hash.add(channel_ == Channel::DMin ? &dmin_id : &dmax_id);
  }

  std::optional<AttrDomain> preferred_domain(const Mesh & /*mesh*/) const override
  {
    return AttrDomain::Point;
  }

  bke::NativeFieldDomain native_domain_info(const Mesh & /*mesh*/) const override
  {
    return bke::NativeFieldDomain::Domain{AttrDomain::Point};
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  params.set_output("Min"_ustr,
                    Field<float>::from_input<PrincipalScalarFieldInput>(Channel::KMin));
  params.set_output("Max"_ustr,
                    Field<float>::from_input<PrincipalScalarFieldInput>(Channel::KMax));
  params.set_output("Min Direction"_ustr,
                    Field<float3>::from_input<PrincipalDirectionFieldInput>(Channel::DMin));
  params.set_output("Max Direction"_ustr,
                    Field<float3>::from_input<PrincipalDirectionFieldInput>(Channel::DMax));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalPrincipalCurvature"_ustr, GEO_NODE_CGAL_PRINCIPAL_CURVATURE);
  ntype.ui_name = "Principal Curvature";
  ntype.ui_description =
      "Field node: principal curvatures k_min/k_max and their tangent directions on the context mesh.";
  ntype.enum_name_legacy = "CGAL_PRINCIPAL_CURVATURE";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_principal_curvature_cc
