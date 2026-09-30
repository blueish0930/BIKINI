/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "BLI_math_vector.hh"

#include <algorithm>
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_dual_contour_cc {

enum class Method {
  DualContour = 0,
  MarchingCubes = 1,
};

static const EnumPropertyItem method_items[] = {
    {int(Method::DualContour),
     "DUAL_CONTOUR",
     0,
     N_("Dual Contour"),
     N_("Surface Nets / dual contouring. One vertex per occupied cell.")},
    {int(Method::MarchingCubes),
     "MARCHING_CUBES",
     0,
     N_("Marching Cubes"),
     N_("CGAL topologically-correct marching cubes (TCMC).")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Menu>("Method"_ustr)
      .static_items(method_items)
      .default_value(MenuValue(Method::DualContour))
      .optional_label()
      .description("Dual Contour (Surface Nets) or Marching Cubes (TCMC).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .description("Triangle mesh of the SDF = Isovalue surface.");
  b.add_input<decl::Vector>("Min"_ustr)
      .default_value(float3(-1.0f))
      .description("Minimum corner of the SDF sampling box.");
  b.add_input<decl::Vector>("Max"_ustr)
      .default_value(float3(1.0f))
      .description("Maximum corner of the SDF sampling box.");
  b.add_input<decl::Float>("SDF"_ustr)
      .default_value(0.0f)
      .structure_type(StructureType::Field)
      .description(
          "Signed scalar field. Connect Position, a texture, Geometry Proximity distance, etc. "
          "The isosurface is where SDF equals Isovalue.");
  b.add_input<decl::Float>("Isovalue"_ustr)
      .default_value(0.0f)
      .description("Extract the level set SDF = Isovalue (0 = zero set).");
  b.add_input<decl::Int>("Resolution"_ustr)
      .default_value(32)
      .min(8)
      .max(96)
      .description("Grid samples per axis inside Min–Max.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const Method method = params.extract_input<Method>("Method"_ustr);
  float3 bmin = params.extract_input<float3>("Min"_ustr);
  float3 bmax = params.extract_input<float3>("Max"_ustr);
  Field<float> sdf_field = params.extract_input<Field<float>>("SDF"_ustr);
  const float isovalue = params.extract_input<float>("Isovalue"_ustr);
  const int res = std::max(8, std::min(96, params.extract_input<int>("Resolution"_ustr)));
  if (bmin.x > bmax.x) {
    std::swap(bmin.x, bmax.x);
  }
  if (bmin.y > bmax.y) {
    std::swap(bmin.y, bmax.y);
  }
  if (bmin.z > bmax.z) {
    std::swap(bmin.z, bmax.z);
  }
  const float3 size = math::max(bmax - bmin, float3(1e-6f));
  const int n = res + 1;
  const float3 voxel = size / float(res);
  const int tot = n * n * n;
  PointCloud *samples = BKE_pointcloud_new_nomain(PointCloudType::Points, tot);
  MutableSpan<float3> sample_pos = samples->positions_for_write();
  int index = 0;
  for (int k = 0; k < n; k++) {
    for (int j = 0; j < n; j++) {
      for (int i = 0; i < n; i++) {
        sample_pos[index++] = bmin + float3(voxel.x * i, voxel.y * j, voxel.z * k);
      }
    }
  }
  Array<float> sdf(tot);
  {
    const bke::PointCloudFieldContext ctx(*samples);
    fn::FieldEvaluator ev(ctx, tot);
    ev.add_with_destination(sdf_field, sdf.as_mutable_span());
    ev.evaluate();
  }
  BKE_id_free(nullptr, samples);

  std::string error;
  Mesh *out = nullptr;
  if (method == Method::MarchingCubes) {
    out = geometry::cgal_marching_cubes_grid(
        sdf.as_span(), n, n, n, bmin, voxel, isovalue, true, error);
  }
  else {
    out = geometry::cgal_dual_contour_grid(
        sdf.as_span(), n, n, n, bmin, voxel, isovalue, error);
  }
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Implicit Surface: SDF never crosses Isovalue") :
                                             error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalDualContour"_ustr, GEO_NODE_CGAL_DUAL_CONTOUR);
  ntype.ui_name = "Implicit Surface";
  ntype.ui_description =
      "Extract a triangle isosurface from a scalar SDF field on a Cartesian grid between Min and "
      "Max. Method: Dual Contour (Surface Nets) or Marching Cubes (CGAL TCMC). Distinct from "
      "Isosurface 3D (marching tetrahedra on input samples).";
  ntype.enum_name_legacy = "CGAL_DUAL_CONTOUR";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_dual_contour_cc
