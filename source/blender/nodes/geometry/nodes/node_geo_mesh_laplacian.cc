/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include <fmt/format.h>

#include "BLI_array_utils.hh"
#include "BLI_offset_indices.hh"

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

/** #bNode::custom2. */
enum {
  WRITE_ATTRIBUTE = (1 << 0),
};

/** Custom mode reads its matrix from attributes, so there it has nothing to write. */
static bool node_writes_attributes(const bNode &node)
{
  return WeightMode(node.custom1) != WeightMode::Custom && (node.custom2 & WRITE_ATTRIBUTE) != 0;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  const bool is_custom = node != nullptr && WeightMode(node->custom1) == WeightMode::Custom;
  const bool is_writing = node != nullptr && node_writes_attributes(*node);

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
  /* Read in custom mode, written with Write Attribute in the other modes. */
  auto &weight_attribute =
      b.add_input<decl::String>("Weight Attribute"_ustr)
          .default_value("weight")
          .is_attribute_name()
          .optional_label()
          .description(
              "Name of the weight attribute. Custom mode reads it: a float attribute, or float "
              "array written by a Wrangle node (f[]@), on the point or face corner domain with "
              "the values of the matrix entries, where the row of an entry is the vertex of its "
              "point or face corner. In the other modes it is the float array that Write "
              "Attribute stores on the points");
  auto &column_attribute =
      b.add_input<decl::String>("Column Index Attribute"_ustr)
          .default_value("col")
          .is_attribute_name()
          .optional_label()
          .description(
              "Name of the column index attribute. Custom mode reads it: an integer attribute, or "
              "integer array written by a Wrangle node (i[]@), on the same domain as the weights "
              "with the vertex indices that are the columns of the matrix entries. In the other "
              "modes it is the integer array that Write Attribute stores on the points");
  auto &mesh_output =
      b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().description(
          "The input mesh with the matrix stored in the two array attributes");
  auto &matrix_output =
      b.add_output<decl::Bundle>("Matrix"_ustr)
          .create_signature([](const bNode &) { return BundleSignature::sparse_coo(); })
          .structure_type(StructureType::Single)
          .description("Sparse COO matrix as a bundle of equal-length lists: weight (float), row "
                       "(int), col (int)");
  if (node != nullptr) {
    /* Write Attribute switches between the two outputs. */
    weight_attribute.available(is_custom || is_writing);
    column_attribute.available(is_custom || is_writing);
    mesh_output.available(is_writing);
    matrix_output.available(!is_writing);
  }
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  const bNode &node = *static_cast<const bNode *>(ptr->data);
  layout.prop(ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
  if (WeightMode(node.custom1) != WeightMode::Custom) {
    layout.prop(ptr, "write_attribute", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  }
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

using ArraySpan = VArraySpan<bke::WrangleArrayValue>;

/** The attributes that hold the rest of arrays too long for one value, see #continued. */
static Vector<ArraySpan> lookup_array_chunks(const bke::AttributeAccessor &attributes,
                                             const StringRef name,
                                             const bke::AttrDomain domain)
{
  Vector<ArraySpan> chunks;
  for (int chunk = 1;; chunk++) {
    const bke::AttributeReader<bke::WrangleArrayValue> reader =
        attributes.lookup<bke::WrangleArrayValue>(
            bke::WrangleArrayValue::chunk_attribute_name(name, chunk), domain);
    if (!reader) {
      break;
    }
    chunks.append(*reader);
  }
  return chunks;
}

/** All 32-bit words of the number array of one element, including its chunks. */
static void array_words(const bke::WrangleArrayValue &head,
                        const Span<ArraySpan> chunks,
                        const int index,
                        Vector<int> &r_words)
{
  r_words.clear();
  r_words.extend(Span<int>(head.d.i, head.count));
  if (head.truncated != bke::WrangleArrayValue::continued) {
    return;
  }
  for (const ArraySpan &chunk : chunks) {
    const bke::WrangleArrayValue &part = chunk[index];
    if (part.count == 0) {
      break;
    }
    r_words.extend(
        Span<int>(part.d.i, std::min(int(part.count), int(bke::WrangleArrayValue::max_values))));
  }
}

/**
 * Store one number array per point, \a words[offsets[i]] are the 32-bit words of point i. Arrays
 * that do not fit in one value continue in chunk attributes like the ones of the Wrangle node.
 */
static bool write_array_attribute(bke::MutableAttributeAccessor attributes,
                                  const StringRef name,
                                  const bke::WrangleArrayKind kind,
                                  const OffsetIndices<int> offsets,
                                  const Span<int> words)
{
  constexpr int chunk_words = bke::WrangleArrayValue::max_values;
  const std::optional<bke::AttributeMetaData> meta_data = attributes.lookup_meta_data(name);
  if (meta_data && (meta_data->domain != bke::AttrDomain::Point ||
                    meta_data->data_type != bke::AttrType::WrangleArray))
  {
    attributes.remove(name);
  }
  bke::SpanAttributeWriter<bke::WrangleArrayValue> head_writer =
      attributes.lookup_or_add_for_write_only_span<bke::WrangleArrayValue>(
          name, bke::AttrDomain::Point);
  if (!head_writer) {
    return false;
  }
  int chunks_num = 0;
  for (const int i : offsets.index_range()) {
    const Span<int> point_words = words.slice(offsets[i]);
    bke::WrangleArrayValue &head = head_writer.span[i];
    head.clear(kind);
    head.count = uint16_t(std::min(int(point_words.size()), chunk_words));
    head.set_total_items(int(point_words.size()));
    std::copy_n(point_words.data(), head.count, head.d.i);
    if (point_words.size() > chunk_words) {
      head.truncated = bke::WrangleArrayValue::continued;
      chunks_num = std::max(chunks_num,
                            int(point_words.size() - 1) / chunk_words);
    }
  }
  head_writer.finish();

  for (int chunk = 1; chunk <= chunks_num; chunk++) {
    bke::SpanAttributeWriter<bke::WrangleArrayValue> writer =
        attributes.lookup_or_add_for_write_only_span<bke::WrangleArrayValue>(
            bke::WrangleArrayValue::chunk_attribute_name(name, chunk), bke::AttrDomain::Point);
    if (!writer) {
      return false;
    }
    for (const int i : offsets.index_range()) {
      const Span<int> point_words = words.slice(offsets[i]);
      const int count = std::clamp(int(point_words.size()) - chunk * chunk_words, 0, chunk_words);
      bke::WrangleArrayValue &part = writer.span[i];
      part.clear(kind);
      part.count = uint16_t(count);
      part.total = uint16_t(count);
      if (count > 0) {
        std::copy_n(point_words.data() + chunk * chunk_words, count, part.d.i);
      }
    }
    writer.finish();
  }
  /* Chunks of longer arrays that were stored under this name before. */
  for (int chunk = chunks_num + 1;; chunk++) {
    if (!attributes.remove(bke::WrangleArrayValue::chunk_attribute_name(name, chunk))) {
      break;
    }
  }
  return true;
}

/** Store the rows of the matrix as the point array attributes that the custom mode reads. */
static bool write_matrix_attributes(Mesh &mesh,
                                    const geometry::MeshLaplacianCOO &coo,
                                    const StringRef weight_name,
                                    const StringRef column_name)
{
  /* The entries are sorted by row, so the entries of a point are one range. */
  Array<int> offset_data(mesh.verts_num + 1, 0);
  for (const int row : coo.rows) {
    offset_data[row]++;
  }
  const OffsetIndices<int> offsets = offset_indices::accumulate_counts_to_offsets(offset_data);
  const Span<float> weights = coo.weights;
  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  return write_array_attribute(attributes,
                               weight_name,
                               bke::WrangleArrayKind::Float,
                               offsets,
                               weights.cast<int>()) &&
         write_array_attribute(
             attributes, column_name, bke::WrangleArrayKind::Int, offsets, coo.cols.as_span());
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh"_ustr);
  const bool build_diffusion = params.extract_input<bool>("Diffusion Matrix"_ustr);
  const float diffusion_t = params.extract_input<float>("Diffusion Coefficient"_ustr);
  const bool use_mass = params.extract_input<bool>("Use Mass"_ustr);
  const WeightMode mode = WeightMode(params.node().custom1);
  const bool write_attribute = node_writes_attributes(params.node());
  /* The name inputs only exist when attributes are read or written. */
  std::string weight_name;
  std::string column_name;
  if (mode == WeightMode::Custom || write_attribute) {
    weight_name = params.extract_input<std::string>("Weight Attribute"_ustr);
    column_name = params.extract_input<std::string>("Column Index Attribute"_ustr);
  }
  /* Only one of the outputs exists: the mesh when writing attributes, otherwise the matrix. */
  const auto set_outputs = [&](BundlePtr matrix) {
    if (write_attribute) {
      params.set_output("Mesh"_ustr, std::move(geometry_set));
    }
    else {
      params.set_output("Matrix"_ustr, std::move(matrix));
    }
  };

  if (write_attribute) {
    /* Checked before anything is computed. */
    std::string name_error;
    if (weight_name.empty() || column_name.empty()) {
      name_error = TIP_("Attribute name must not be empty");
    }
    else if (weight_name == column_name) {
      name_error = TIP_("The weight and column index attributes must have different names");
    }
    else if (const Mesh *mesh = geometry_set.get_mesh()) {
      const bke::AttributeAccessor attributes = mesh->attributes();
      for (const std::string &name : {weight_name, column_name}) {
        if (attributes.is_builtin(name)) {
          name_error = fmt::format(
              fmt::runtime(TIP_("Cannot write the matrix to the built-in attribute \"{}\"")),
              name);
        }
      }
    }
    if (!name_error.empty()) {
      params.error_message_add(NodeWarningType::Error, name_error);
      set_outputs(empty_matrix_bundle());
      return;
    }
  }

  const Mesh *src_mesh = geometry_set.get_mesh();
  if (!src_mesh || src_mesh->verts_num == 0) {
    if (!src_mesh) {
      params.error_message_add(NodeWarningType::Warning,
                               TIP_("Input geometry does not contain a mesh"));
    }
    set_outputs(empty_matrix_bundle());
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
      set_outputs(empty_matrix_bundle());
      return;
    }
    const bke::AttrDomain domain = weight_meta->domain;
    if (!ELEM(domain, bke::AttrDomain::Point, bke::AttrDomain::Corner)) {
      params.error_message_add(
          NodeWarningType::Error,
          TIP_("The weight attribute must be on the point or face corner domain"));
      set_outputs(empty_matrix_bundle());
      return;
    }
    if (column_meta->domain != domain) {
      params.error_message_add(
          NodeWarningType::Error,
          TIP_("The weight and column index attributes must be on the same domain"));
      set_outputs(empty_matrix_bundle());
      return;
    }
    const bool weight_is_array = weight_meta->data_type == bke::AttrType::WrangleArray;
    const bool column_is_array = column_meta->data_type == bke::AttrType::WrangleArray;
    if (weight_is_array != column_is_array) {
      params.error_message_add(
          NodeWarningType::Error,
          TIP_("The weight and column index attributes must both be arrays or both be single "
               "values"));
      set_outputs(empty_matrix_bundle());
      return;
    }
    const Span<int> corner_verts = src_mesh->corner_verts();
    if (weight_is_array) {
      /* Arrays written by a Wrangle node (`f[]@weight`, `i[]@col`): one entry per array item. */
      const ArraySpan weights =
          attributes.lookup(weight_name).varray.typed<bke::WrangleArrayValue>();
      const ArraySpan cols = attributes.lookup(column_name).varray.typed<bke::WrangleArrayValue>();
      const Vector<ArraySpan> weight_chunks = lookup_array_chunks(attributes, weight_name, domain);
      const Vector<ArraySpan> col_chunks = lookup_array_chunks(attributes, column_name, domain);
      Vector<int> weight_words;
      Vector<int> col_words;
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
        array_words(weight, weight_chunks, i, weight_words);
        array_words(col, col_chunks, i, col_words);
        different_lengths |= weight_words.size() != col_words.size();
        const int row = domain == bke::AttrDomain::Point ? i : corner_verts[i];
        const Span<float> weight_floats = weight_words.as_span().cast<float>();
        const Span<float> col_floats = col_words.as_span().cast<float>();
        for (const int k : IndexRange(std::min(weight_words.size(), col_words.size()))) {
          array_weights.append(weight.kind == bke::WrangleArrayKind::Float ?
                                   weight_floats[k] :
                                   float(weight_words[k]));
          array_cols.append(col.kind == bke::WrangleArrayKind::Int ? col_words[k] :
                                                                     int(col_floats[k]));
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
        set_outputs(empty_matrix_bundle());
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
      set_outputs(empty_matrix_bundle());
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

  if (write_attribute) {
    /* This may copy the mesh, the pointers to the input mesh are not used from here on. */
    if (!write_matrix_attributes(
            *geometry_set.get_mesh_for_write(), coo, weight_name, column_name))
    {
      params.error_message_add(NodeWarningType::Error,
                               TIP_("Failed to write the matrix attributes"));
    }
    set_outputs(empty_matrix_bundle());
    return;
  }

  set_outputs(coo_to_bundle(std::move(coo)));
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
  RNA_def_node_boolean(
      srna,
      "write_attribute",
      "Write Attribute",
      "Output the mesh with the matrix stored on it instead of the matrix bundle: every point "
      "gets a float array attribute and an integer array attribute, named by the two inputs, "
      "with the entries of its row, in the form that the Custom mode reads",
      NOD_inline_boolean_accessors(custom2, WRITE_ATTRIBUTE));
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
