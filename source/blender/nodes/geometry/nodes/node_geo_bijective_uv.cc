/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"

#include "node_geometry_util.hh"

#include "bijective_uv_solver.hh"

#include <chrono>
#include <numeric>

namespace blender::nodes::node_geo_bijective_uv_cc {

static const EnumPropertyItem method_items[] = {
    {0, "STANDARD_LBFGS", 0, N_("Standard L-BFGS"),
     N_("Limited-memory BFGS with a scalar initial inverse Hessian")},
    {1, "PRECONDITIONED_LBFGS", 0, N_("Preconditioned L-BFGS"),
     N_("L-BFGS with a reference FEM metric factored once per chart")},
    {2, "ASSEMBLED_SYSTEM", 0, N_("Assembled Linear System"),
     N_("Assemble a projected distortion Hessian and positive barrier "
        "curvature each iteration")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .supported_type(GeometryComponent::Type::Mesh)
      .only_realized_data()
      .description("Oriented charts with edges already split along seams; "
                   "polygons are triangulated internally");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Menu>("Method"_ustr).static_items(method_items).default_value(MenuValue(1));
  b.add_input<decl::Bool>("Use Initial UV"_ustr)
      .default_value(false)
      .description("Use a valid non-overlapping initial map instead of "
                   "circular Tutte initialization");
  b.add_input<decl::Vector>("Initial UV"_ustr)
      .evaluated_geometry_field()
      .hide_value()
      .description("XY coordinates; UV must be continuous at each chart vertex "
                   "and have positive triangle orientation");
  b.add_input<decl::Int>("Iterations"_ustr).default_value(500).min(0).max(100000);
  b.add_input<decl::Int>("History"_ustr)
      .default_value(8)
      .min(1)
      .max(32)
      .description("Number of curvature pairs retained by either L-BFGS method");
  b.add_input<decl::Float>("Tolerance"_ustr)
      .default_value(1e-8f)
      .min(0)
      .max(1)
      .description("Stop after five accepted steps below this relative energy "
                   "decrease; zero disables this criterion");
  b.add_input<decl::Float>("Barrier Distance"_ustr)
      .default_value(0)
      .min(0)
      .description("Boundary barrier support radius; zero uses one quarter of "
                   "the mean 3D boundary edge length");
  b.add_input<decl::Float>("Barrier Weight"_ustr)
      .default_value(1)
      .min(0)
      .description("Weight of the boundary barrier; continuous collision "
                   "limiting remains active at zero weight");
  b.add_output<decl::Vector>("UV"_ustr).anonymous_attribute_output().description(
      "Unpacked UV coordinates; each disconnected chart is "
      "optimized independently");
  b.add_output<decl::Int>("Iterations Used"_ustr)
      .description("Maximum accepted iteration count across charts");
  b.add_output<decl::Int>("Status"_ustr)
      .description("0: iteration budget reached; 1: tolerance reached in every chart; "
                   "2: line search stalled; 3: invalid input or numerical failure");
  b.add_output<decl::Float>("Distortion"_ustr)
      .description("Area-weighted symmetric Dirichlet distortion; an isometry "
                   "has value 4");
  b.add_output<decl::Float>("Barrier Energy"_ustr);
  b.add_output<decl::Int>("Line Search Trials"_ustr);
  b.add_output<decl::Float>("Solve Time"_ustr)
      .description("Seconds spent preparing and solving all charts");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  namespace uv = geometry::bijective_uv;
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.set_default_remaining_outputs();
    return;
  }
  const auto start = std::chrono::steady_clock::now();
  uv::Options options;
  options.method = params.extract_input<MenuValue>("Method"_ustr).value;
  options.iterations = std::clamp(params.extract_input<int>("Iterations"_ustr), 0, 100000);
  options.history = std::clamp(params.extract_input<int>("History"_ustr), 1, 32);
  options.tolerance = params.extract_input<float>("Tolerance"_ustr);
  options.epsilon = params.extract_input<float>("Barrier Distance"_ustr);
  options.weight = params.extract_input<float>("Barrier Weight"_ustr);
  const bool use_initial = params.extract_input<bool>("Use Initial UV"_ustr);
  const Field<float3> initial_field = params.extract_input<Field<float3>>("Initial UV"_ustr);
  const std::optional<std::string> uv_id =
      params.get_output_anonymous_attribute_id_if_needed("UV"_ustr);

  try {
    const Span<float3> positions = mesh->vert_positions();
    const Span<int> corners = mesh->corner_verts();
    const Span<int3> corner_tris = mesh->corner_tris();
    std::vector<int> parent(mesh->verts_num);
    std::iota(parent.begin(), parent.end(), 0);
    auto root = [&](int v) {
      while (parent[v] != v) {
        parent[v] = parent[parent[v]];
        v = parent[v];
      }
      return v;
    };
    for (const int3 &t : corner_tris) {
      const int a = root(corners[t[0]]);
      parent[root(corners[t[1]])] = a;
      parent[root(corners[t[2]])] = a;
    }
    std::map<int, std::vector<int>> chart_vertices;
    std::map<int, std::vector<std::array<int, 3>>> chart_faces;
    for (const int3 &t : corner_tris) {
      chart_faces[root(corners[t[0]])].push_back({corners[t[0]], corners[t[1]], corners[t[2]]});
    }
    for (const int v : positions.index_range()) {
      if (chart_faces.count(root(v))) {
        chart_vertices[root(v)].push_back(v);
      }
    }
    Array<float2> initial(mesh->verts_num, float2(0));
    if (use_initial) {
      const bke::MeshFieldContext context{*mesh, AttrDomain::Corner};
      FieldEvaluator evaluator{context, mesh->corners_num};
      evaluator.add(initial_field);
      evaluator.evaluate();
      const VArray<float3> values = evaluator.get_evaluated<float3>(0);
      Array<bool> seen(mesh->verts_num, false);
      for (const int c : corners.index_range()) {
        const int v = corners[c];
        const float3 value = values[c];
        const float2 xy(value.x, value.y);
        if (seen[v] && initial[v] != xy) {
          throw std::runtime_error("Initial UV has a seam at an unsplit "
                                   "vertex; use Split Edges along seams first");
        }
        initial[v] = xy;
        seen[v] = true;
      }
    }
    Array<float3> output(mesh->verts_num, float3(0));
    std::vector<int> local(mesh->verts_num, -1);
    int iterations = 0, trials = 0, status = 1;
    double area = 0, distortion = 0, barrier = 0;
    for (auto &[chart, vertices] : chart_vertices) {
      std::vector<uv::V3> p;
      p.reserve(vertices.size());
      for (int i = 0; i < int(vertices.size()); i++) {
        const int v = vertices[i];
        local[v] = i;
        p.emplace_back(positions[v].x, positions[v].y, positions[v].z);
      }
      auto faces = chart_faces.at(chart);
      for (auto &t : faces) {
        for (int &v : t) {
          v = local[v];
        }
      }
      uv::Solver solver(p, faces, options);
      uv::Vec u;
      if (use_initial) {
        u.resize(2 * vertices.size());
        for (int i = 0; i < int(vertices.size()); i++) {
          u.segment<2>(2 * i) = uv::V2(initial[vertices[i]].x, initial[vertices[i]].y);
        }
      } else {
        u = solver.initialize();
      }
      const uv::Result result = solver.solve(std::move(u));
      /* Verify the actual float output, not only the internal double solution.
       */
      uv::Vec rounded(result.uv.size());
      for (int i = 0; i < int(vertices.size()); i++) {
        const float x = float(result.uv[2 * i]), y = float(result.uv[2 * i + 1]);
        output[vertices[i]] = float3(x, y, 0);
        rounded.segment<2>(2 * i) = uv::V2(x, y);
      }
      uv::Vec gradient;
      double ed, eb;
      if (!std::isfinite(solver.evaluate(rounded, gradient, ed, eb)) ||
          !solver.boundary_valid(rounded)) {
        throw std::runtime_error("UV loses bijectivity at float attribute precision; rescale the "
                                 "chart or improve initialization");
      }
      area += solver.area;
      distortion += ed;
      barrier += eb;
      iterations = std::max(iterations, result.iterations);
      trials += result.trials;
      if (result.status == 2) {
        status = 2;
      } else if (result.status == 0 && status != 2) {
        status = 0;
      }
    }
    if (uv_id) {
      Mesh *output_mesh = geometry.get_mesh_for_write();
      bke::SpanAttributeWriter<float3> attribute =
          output_mesh->attributes_for_write().lookup_or_add_for_write_only_span<float3>(
              *uv_id, AttrDomain::Corner);
      const Span<int> output_corners = output_mesh->corner_verts();
      for (const int c : output_corners.index_range()) {
        attribute.span[c] = output[output_corners[c]];
      }
      attribute.finish();
    }
    params.set_output("Mesh"_ustr, std::move(geometry));
    params.set_output("Iterations Used"_ustr, iterations);
    params.set_output("Status"_ustr, status);
    params.set_output("Distortion"_ustr, float(distortion / area));
    params.set_output("Barrier Energy"_ustr, float(barrier));
    params.set_output("Line Search Trials"_ustr, trials);
    params.set_output(
        "Solve Time"_ustr,
        float(std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count()));
    if (status == 2) {
      params.error_message_add(NodeWarningType::Warning,
                               "Line search stalled before reaching tolerance");
    }
  } catch (const std::exception &error) {
    params.error_message_add(NodeWarningType::Error, error.what());
    params.set_output("Mesh"_ustr, std::move(geometry));
    params.set_output("Status"_ustr, 3);
    params.set_default_remaining_outputs();
  }
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeBijectiveUV"_ustr, GEO_NODE_BIJECTIVE_UV);
  ntype.ui_name = "Bijective UV";
  ntype.ui_description = "Free-boundary bijective chart parameterization with "
                         "three native optimization methods";
  ntype.enum_name_legacy = "BIJECTIVE_UV";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

} // namespace blender::nodes::node_geo_bijective_uv_cc
