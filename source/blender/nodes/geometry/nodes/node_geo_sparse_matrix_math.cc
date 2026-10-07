/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_geometry_nodes_bundle.hh"
#include "NOD_geometry_nodes_bundle_signature.hh"
#include "NOD_geometry_nodes_list.hh"
#include "NOD_geometry_nodes_sparse_matrix.hh"
#include "NOD_rna_define.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "BLI_array.hh"

#include <cstring>

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_sparse_matrix_math_cc {

using sparse_matrix::COOMatrix;
using sparse_matrix::Operation;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_default_layout();

  const bNode *node = b.node_or_null();
  const Operation operation = node ? Operation(node->custom1) : Operation::Add;
  const bool is_scale = operation == Operation::Scale;
  const bool is_transpose = operation == Operation::Transpose;

  b.add_input<decl::Bundle>("A"_ustr)
      .create_signature([](const bNode &) { return BundleSignature::sparse_coo(); })
      .description("Left sparse COO matrix bundle (weight/row/col lists)");
  b.add_output<decl::Bundle>("Result"_ustr)
      .create_signature([](const bNode &) { return BundleSignature::sparse_coo(); })
      .description("Sparse COO result (weight/row/col, zeros omitted)")
      .align_with_previous()
      .propagate_all();
  auto &b_in = b.add_input<decl::Bundle>("B"_ustr)
                   .create_signature([](const bNode &) { return BundleSignature::sparse_coo(); })
                   .description("Right sparse COO matrix bundle (weight/row/col lists)");
  auto &scale = b.add_input<decl::Float>("Scale"_ustr)
                    .default_value(1.0f)
                    .description("Scalar multiplied with every stored value of A");

  b_in.available(!is_scale && !is_transpose);
  scale.available(is_scale);
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "operation", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = int16_t(Operation::Add);
}

static bool extract_list_float(const Bundle &bundle,
                               const char *key_name,
                               Vector<float> &r_values,
                               std::string &r_error)
{
  const std::optional<BundleKey> key = BundleKey::from_str(key_name);
  if (!key) {
    r_error = "Invalid bundle key";
    return false;
  }
  const std::optional<GListPtr> list_opt = bundle.lookup<GListPtr>(*key);
  if (!list_opt || !*list_opt) {
    /* Missing item means empty list (empty sparse matrix component). */
    r_values.clear();
    return true;
  }
  const GListPtr &list = *list_opt;
  if (!list->cpp_type().is<float>()) {
    r_error = std::string("Bundle item '") + key_name + "' must be a float list";
    return false;
  }
  const VArray<float> values = list->typed<float>().varray();
  r_values.resize(values.size());
  values.materialize(r_values.as_mutable_span());
  return true;
}

static bool extract_list_int(const Bundle &bundle,
                             const char *key_name,
                             Vector<int> &r_values,
                             std::string &r_error)
{
  const std::optional<BundleKey> key = BundleKey::from_str(key_name);
  if (!key) {
    r_error = "Invalid bundle key";
    return false;
  }
  const std::optional<GListPtr> list_opt = bundle.lookup<GListPtr>(*key);
  if (!list_opt || !*list_opt) {
    r_values.clear();
    return true;
  }
  const GListPtr &list = *list_opt;
  if (!list->cpp_type().is<int>()) {
    r_error = std::string("Bundle item '") + key_name + "' must be an int list";
    return false;
  }
  const VArray<int> values = list->typed<int>().varray();
  r_values.resize(values.size());
  values.materialize(r_values.as_mutable_span());
  return true;
}

/**
 * Read weight/row/col from a matrix Bundle. Missing or empty bundle → empty COO.
 * \return false on hard validation errors (type mismatch / unequal lengths).
 */
static bool extract_coo_from_bundle(const BundlePtr &bundle_ptr,
                                    Vector<float> &r_weight,
                                    Vector<int> &r_row,
                                    Vector<int> &r_col,
                                    std::string &r_error)
{
  r_weight.clear();
  r_row.clear();
  r_col.clear();
  if (!bundle_ptr) {
    return true;
  }
  const Bundle &bundle = *bundle_ptr;
  if (!extract_list_float(bundle, "weight", r_weight, r_error)) {
    return false;
  }
  if (!extract_list_int(bundle, "row", r_row, r_error)) {
    return false;
  }
  if (!extract_list_int(bundle, "col", r_col, r_error)) {
    return false;
  }
  if (!sparse_matrix::lengths_equal(
          int64_t(r_weight.size()), int64_t(r_row.size()), int64_t(r_col.size())))
  {
    r_error = "weight, row, and col lists must have equal length";
    return false;
  }
  return true;
}

static BundlePtr make_coo_bundle(const COOMatrix &matrix)
{
  BundlePtr bundle_ptr = Bundle::create();
  Bundle &bundle = bundle_ptr.ensure_mutable_inplace();

  const bke::bNodeSocketType *float_stype = bke::node_socket_type_find_static(SOCK_FLOAT);
  const bke::bNodeSocketType *int_stype = bke::node_socket_type_find_static(SOCK_INT);
  BLI_assert(float_stype && int_stype);

  Array<float> weights(matrix.weight.size());
  Array<int> rows(matrix.row.size());
  Array<int> cols(matrix.col.size());
  if (!matrix.weight.empty()) {
    memcpy(weights.data(), matrix.weight.data(), matrix.weight.size() * sizeof(float));
    memcpy(rows.data(), matrix.row.data(), matrix.row.size() * sizeof(int));
    memcpy(cols.data(), matrix.col.data(), matrix.col.size() * sizeof(int));
  }

  bundle.add_new(*BundleKey::from_str("weight"),
                 BundleItemSocketValue{float_stype,
                                       bke::SocketValueVariant::From(
                                           GList::from_container(std::move(weights)))});
  bundle.add_new(*BundleKey::from_str("row"),
                 BundleItemSocketValue{int_stype,
                                       bke::SocketValueVariant::From(
                                           GList::from_container(std::move(rows)))});
  bundle.add_new(*BundleKey::from_str("col"),
                 BundleItemSocketValue{int_stype,
                                       bke::SocketValueVariant::From(
                                           GList::from_container(std::move(cols)))});
  return bundle_ptr;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const Operation operation = Operation(params.node().custom1);

  BundlePtr a_bundle = params.extract_input<BundlePtr>("A"_ustr);

  Vector<float> a_w;
  Vector<int> a_r, a_c;
  std::string error;

  if (!extract_coo_from_bundle(a_bundle, a_w, a_r, a_c, error)) {
    params.error_message_add(NodeWarningType::Error, error);
    params.set_output("Result"_ustr, make_coo_bundle({}));
    return;
  }

  COOMatrix result;
  if (operation == Operation::Scale) {
    const float scale = params.extract_input<float>("Scale"_ustr);
    result = sparse_matrix::sparse_matrix_scale(
        a_w.data(), a_r.data(), a_c.data(), int64_t(a_w.size()), scale);
  }
  else if (operation == Operation::Transpose) {
    result = sparse_matrix::sparse_matrix_transpose(
        a_w.data(), a_r.data(), a_c.data(), int64_t(a_w.size()));
  }
  else {
    BundlePtr b_bundle = params.extract_input<BundlePtr>("B"_ustr);
    Vector<float> b_w;
    Vector<int> b_r, b_c;
    if (!extract_coo_from_bundle(b_bundle, b_w, b_r, b_c, error)) {
      params.error_message_add(NodeWarningType::Error, error);
      params.set_output("Result"_ustr, make_coo_bundle({}));
      return;
    }
    result = sparse_matrix::sparse_matrix_apply(operation,
                                                a_w.data(),
                                                a_r.data(),
                                                a_c.data(),
                                                int64_t(a_w.size()),
                                                b_w.data(),
                                                b_r.data(),
                                                b_c.data(),
                                                int64_t(b_w.size()));
  }

  params.set_output("Result"_ustr, make_coo_bundle(result));
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem operation_items[] = {
      {int(Operation::Add), "ADD", 0, "Add", "Sparse matrix addition (A + B)"},
      {int(Operation::Subtract), "SUBTRACT", 0, "Subtract", "Sparse matrix subtraction (A - B)"},
      {int(Operation::Multiply),
       "MULTIPLY",
       0,
       "Multiply",
       "Sparse matrix–matrix multiplication (A × B)"},
      {int(Operation::Scale),
       "SCALE",
       0,
       "Scale",
       "Multiply every stored value of A by a float constant"},
      {int(Operation::Transpose),
       "TRANSPOSE",
       0,
       "Transpose",
       "Swap the rows and columns of A"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "operation",
                    "Operation",
                    "Sparse matrix operation to apply to the input matrices",
                    operation_items,
                    NOD_inline_enum_accessors(custom1),
                    int(Operation::Add));
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeSparseMatrixMath"_ustr, GEO_NODE_SPARSE_MATRIX_MATH);
  ntype.ui_name = "Sparse Matrix Math";
  ntype.ui_description =
      "Add, subtract, multiply, scale, or transpose large sparse matrices stored as weight/row/col "
      "bundles";
  ntype.enum_name_legacy = "SPARSE_MATRIX_MATH";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_sparse_matrix_math_cc
