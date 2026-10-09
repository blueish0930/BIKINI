/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_geometry_nodes_bundle.hh"
#include "NOD_geometry_nodes_bundle_signature.hh"
#include "NOD_geometry_nodes_list.hh"
#include "NOD_rna_define.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include <algorithm>
#include <cmath>
#include <string>

#include "BLI_math_vector_types.hh"

#include "MEM_guardedalloc.h"

#include "node_geometry_util.hh"

#include "linear_solver_math.hh"

namespace blender::nodes::node_geo_linear_solver_cc {

namespace ls = blender::geometry::linear_solver;

enum class Mode : int8_t {
  Solve = 0,
  Decompose = 1,
  SolveWithDecomposition = 2,
  Multiply = 3,
};

enum class DecomposeSubMode : int8_t {
  MatrixFactor = 0,
  Eigen = 1,
};

enum class DataType : int8_t {
  Float = 0,
  Vector = 1,
};

/*
 * Packing in bNode DNA fields (no extra DNA struct):
 *   custom1: mode (low 8) | data_type (bits 8-15)
 *   custom2: solver_class (0-1) | direct (2-5) | iterative (6-9) | precond (10-13) | eigen_kind (14)
 *            eigen uses remaining: store eigen_kind in high of custom3
 *   custom3: float as int: decompose_sub (low 4) | eigen_solver (4-7)
 *   custom4: shift (float)
 */

static Mode get_mode(const bNode &node)
{
  return Mode(node.custom1 & 0xff);
}

static DataType get_data_type(const bNode &node)
{
  return DataType((node.custom1 >> 8) & 0xff);
}

static ls::SolverClass get_solver_class(const bNode &node)
{
  return ls::SolverClass(node.custom2 & 0x3);
}

static ls::DirectMethod get_direct(const bNode &node)
{
  return ls::DirectMethod((node.custom2 >> 2) & 0xf);
}

static ls::IterativeMethod get_iterative(const bNode &node)
{
  return ls::IterativeMethod((node.custom2 >> 6) & 0xf);
}

static ls::Preconditioner get_precond(const bNode &node)
{
  return ls::Preconditioner((node.custom2 >> 10) & 0xf);
}

static DecomposeSubMode get_decompose_sub(const bNode &node)
{
  return DecomposeSubMode(int(node.custom3) & 0xf);
}

static ls::EigenSolverKind get_eigen_kind(const bNode &node)
{
  return ls::EigenSolverKind((int(node.custom3) >> 4) & 0xf);
}

static bool is_shift_eigen_kind(const ls::EigenSolverKind kind)
{
  return kind == ls::EigenSolverKind::GenEigsRealShiftSolver ||
         kind == ls::EigenSolverKind::SymEigsShiftSolver;
}

class LinearSolverDecompositionItem : public BundleItemInternalValueMixin {
 public:
  ls::Decomposition data;

  StringRefNull type_name() const override
  {
    return "Linear Solver Decomposition";
  }

  void delete_self() override
  {
    MEM_delete(this);
  }
};

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  const DataType dtype = node ? get_data_type(*node) : DataType::Float;
  /* Single typed sockets (not if/else branches) so float↔vector rebuilds cleanly. */
  const eNodeSocketDatatype data_sock = (dtype == DataType::Vector) ? SOCK_VECTOR : SOCK_FLOAT;

  auto &a_in = b.add_input<decl::Bundle>("A"_ustr)
                   .create_signature([](const bNode &) { return BundleSignature::sparse_coo(); })
                   .structure_type(StructureType::Single)
                   .description("Sparse COO matrix bundle: weight (float list), row (int list), "
                                "col (int list)");

  auto &b_in = b.add_input(data_sock, "b"_ustr)
                   .structure_type(StructureType::List)
                   .description("Right-hand side list (float or vector per Data Type)");
  /* Input x is only for Multiply (y = A*x). Solve modes have b, not x-in. Do not use updatefunc
   * + input_by_identifier: float↔vector rebuild leaves sockets available until topology cache
   * exists, so a ghost x input appears. Prefer .available() on declare (same as Debug / KNearest). */
  auto &x_in = b.add_input(data_sock, "x"_ustr)
                   .structure_type(StructureType::List)
                   .description("Unknown / multiply vector list (float or vector per Data Type)");
  auto &x_out = b.add_output(data_sock, "x"_ustr)
                    .structure_type(StructureType::List)
                    .description("Solution or matrix-vector product (same Data Type as inputs)");

  auto &pin = b.add_input<decl::Bool>("Pin"_ustr)
                  .structure_type(StructureType::List)
                  .description("Boolean pin mask; pinned DOFs are fixed to the corresponding b "
                               "values. For a matrix factor the mask is part of the factorization, "
                               "and Solve with Decomposition takes the pinned values from its b");
  auto &decomp_in = b.add_input<decl::Bundle>("Decomposition"_ustr)
                        .description("Reusable matrix factorization from Decompose mode. It "
                                     "replaces the matrix A, which is not needed again");

  auto &num_eigen = b.add_input<decl::Int>("Num Eigenpairs"_ustr)
                        .default_value(0)
                        .min(0)
                        .description(
                            "How many eigenpairs to compute. "
                            "Sym/Gen shift solvers: the k eigenvalues closest to Shift "
                            "(shift-invert); ordered by increasing |λ−σ|. Increasing k only "
                            "appends farther pairs. 0 = a few pairs (or full spectrum if n is "
                            "small)");
  auto &shift = b.add_input<decl::Float>("Shift"_ustr)
                    .default_value(0.0f)
                    .description("Shift σ for shift / shift-invert eigen solvers only");
  auto &max_iter = b.add_input<decl::Int>("Max Iterations"_ustr)
                       .default_value(1000)
                       .min(1)
                       .description("Maximum iterations for iterative solvers");
  auto &tol = b.add_input<decl::Float>("Tolerance"_ustr)
                  .default_value(1e-10f)
                  .min(0.0f)
                  .description("Relative residual tolerance for iterative solvers");

  auto &decomp_out = b.add_output<decl::Bundle>("Decomposition"_ustr)
                         .description("Matrix factorization for Solve with Decomposition");
  auto &eigenvectors = b.add_output<decl::Float>("Eigenvectors"_ustr)
                           .structure_type(StructureType::List)
                           .description(
                               "List of eigenvectors: outer list length = Num Eigenpairs; "
                               "each item is a float list of length n (one eigenvector). "
                               "Use Get List Item to pick vector j");
  auto &eigenvalues = b.add_output<decl::Float>("Eigenvalues"_ustr)
                          .structure_type(StructureType::List)
                          .description("Eigenvalues list (length k), same order as Eigenvectors");

  if (node != nullptr) {
    const Mode mode = get_mode(*node);
    const DecomposeSubMode sub = get_decompose_sub(*node);
    const ls::EigenSolverKind ekind = get_eigen_kind(*node);

    const bool need_b = mode == Mode::Solve || mode == Mode::SolveWithDecomposition;
    const bool need_x_in = mode == Mode::Multiply;
    /* The factorization already contains the matrix and the pin mask. */
    const bool need_a = mode != Mode::SolveWithDecomposition;
    const bool need_pin = mode == Mode::Solve || mode == Mode::Decompose;
    const bool need_decomp_in = mode == Mode::SolveWithDecomposition;
    const bool need_eigen_params = mode == Mode::Decompose && sub == DecomposeSubMode::Eigen;
    const bool need_shift = need_eigen_params && is_shift_eigen_kind(ekind);
    const bool need_iter_params = mode == Mode::Solve &&
                                  get_solver_class(*node) == ls::SolverClass::Iterative;
    const bool out_x = mode == Mode::Solve || mode == Mode::SolveWithDecomposition ||
                       mode == Mode::Multiply;
    const bool out_decomp = mode == Mode::Decompose && sub == DecomposeSubMode::MatrixFactor;
    const bool out_eigen = mode == Mode::Decompose && sub == DecomposeSubMode::Eigen;

    a_in.available(need_a);
    b_in.available(need_b);
    x_in.available(need_x_in);
    pin.available(need_pin);
    decomp_in.available(need_decomp_in);
    num_eigen.available(need_eigen_params);
    shift.available(need_shift);
    max_iter.available(need_iter_params);
    tol.available(need_iter_params);
    x_out.available(out_x);
    decomp_out.available(out_decomp);
    eigenvectors.available(out_eigen);
    eigenvalues.available(out_eigen);
  }
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  /* Full-width dropdowns like Nearest Neighbors (no property_split strip). */
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);

  PropertyRNA *mode_prop = RNA_struct_find_property(ptr, "mode");
  const Mode mode = Mode(mode_prop ? RNA_property_enum_get(ptr, mode_prop) : 0);

  /* Decompose only needs matrix A — hide data type. */
  if (mode != Mode::Decompose) {
    layout.prop(ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  }

  if (mode == Mode::Decompose) {
    layout.prop(ptr, "decompose_sub_mode", UI_ITEM_NONE, "", ICON_NONE);
  }

  PropertyRNA *sub_prop = RNA_struct_find_property(ptr, "decompose_sub_mode");
  const DecomposeSubMode sub = DecomposeSubMode(
      sub_prop ? RNA_property_enum_get(ptr, sub_prop) : 0);

  /* Solve with Decomposition: only apply stored factor — no Direct/Iterative UI. */
  if (mode == Mode::Solve) {
    layout.prop(ptr, "solver_class", UI_ITEM_NONE, "", ICON_NONE);
    PropertyRNA *sc_prop = RNA_struct_find_property(ptr, "solver_class");
    const ls::SolverClass sc = ls::SolverClass(
        sc_prop ? RNA_property_enum_get(ptr, sc_prop) : 0);
    if (sc == ls::SolverClass::Direct) {
      layout.prop(ptr, "direct_method", UI_ITEM_NONE, "", ICON_NONE);
    }
    else {
      layout.prop(ptr, "iterative_method", UI_ITEM_NONE, "", ICON_NONE);
      layout.prop(ptr, "preconditioner", UI_ITEM_NONE, "", ICON_NONE);
    }
  }
  else if (mode == Mode::Decompose && sub == DecomposeSubMode::MatrixFactor) {
    /* Factorization always uses a direct method. */
    layout.prop(ptr, "direct_method", UI_ITEM_NONE, "", ICON_NONE);
  }
  else if (mode == Mode::Decompose && sub == DecomposeSubMode::Eigen) {
    layout.prop(ptr, "eigen_solver", UI_ITEM_NONE, "", ICON_NONE);
  }
}

static void list_to_doubles_float(const GListPtr &list, std::vector<double> &out)
{
  out.clear();
  if (!list || !list->cpp_type().is<float>()) {
    return;
  }
  const VArray<float> values = list->typed<float>().varray();
  out.resize(size_t(values.size()));
  for (const int64_t i : values.index_range()) {
    out[size_t(i)] = double(values[i]);
  }
}

/** Planar layout: all X, then all Y, then all Z. */
static void list_to_doubles_vector_planar(const GListPtr &list, std::vector<double> &out, int &n)
{
  out.clear();
  n = 0;
  if (!list || !list->cpp_type().is<float3>()) {
    return;
  }
  const VArray<float3> values = list->typed<float3>().varray();
  n = int(values.size());
  out.resize(size_t(n) * 3);
  for (int i = 0; i < n; i++) {
    const float3 v = values[i];
    out[size_t(i)] = double(v.x);
    out[size_t(n) + size_t(i)] = double(v.y);
    out[size_t(2 * n) + size_t(i)] = double(v.z);
  }
}

static void list_to_bool_mask(const GListPtr &list, std::vector<char> &out)
{
  out.clear();
  if (!list) {
    return;
  }
  if (list->cpp_type().is<bool>()) {
    const VArray<bool> values = list->typed<bool>().varray();
    out.resize(size_t(values.size()));
    for (const int64_t i : values.index_range()) {
      out[size_t(i)] = values[i] ? 1 : 0;
    }
    return;
  }
  /* Accept float 0/1 as fallback. */
  if (list->cpp_type().is<float>()) {
    const VArray<float> values = list->typed<float>().varray();
    out.resize(size_t(values.size()));
    for (const int64_t i : values.index_range()) {
      out[size_t(i)] = (values[i] > 0.5f) ? 1 : 0;
    }
  }
}

static void list_to_ints(const GListPtr &list, std::vector<int> &out)
{
  out.clear();
  if (!list) {
    return;
  }
  if (list->cpp_type().is<int>()) {
    const VArray<int> values = list->typed<int>().varray();
    out.resize(size_t(values.size()));
    for (const int64_t i : values.index_range()) {
      out[size_t(i)] = values[i];
    }
    return;
  }
  if (list->cpp_type().is<float>()) {
    const VArray<float> values = list->typed<float>().varray();
    out.resize(size_t(values.size()));
    for (const int64_t i : values.index_range()) {
      out[size_t(i)] = int(std::lround(double(values[i])));
    }
  }
}

static GListPtr doubles_to_float_list(const std::vector<double> &values)
{
  Array<float> arr(values.size());
  for (size_t i = 0; i < values.size(); i++) {
    arr[i] = float(values[i]);
  }
  return GList::from_container(std::move(arr));
}

/**
 * Nested list: outer length k, each element is a float list of length n.
 * Stored as List of SocketValueVariant (each holding a float GList), same pattern
 * as Closure to List when values are themselves lists.
 */
static GListPtr eigenvecs_to_nested_float_lists(const std::vector<double> &flat,
                                                const int n,
                                                const int k)
{
  if (n <= 0 || k <= 0) {
    return GList::from_container(Array<bke::SocketValueVariant>());
  }
  Array<bke::SocketValueVariant> outer(k);
  for (int j = 0; j < k; j++) {
    Array<float> vec(n);
    for (int i = 0; i < n; i++) {
      const size_t idx = size_t(j) * size_t(n) + size_t(i);
      vec[i] = (idx < flat.size()) ? float(flat[idx]) : 0.0f;
    }
    outer[j] = bke::SocketValueVariant::From(GList::from_container(std::move(vec)));
  }
  return GList::from_container(std::move(outer));
}

static GListPtr planar_to_vector_list(const std::vector<double> &planar, const int n)
{
  Array<float3> arr(n);
  for (int i = 0; i < n; i++) {
    arr[i] = float3(float(planar[size_t(i)]),
                    float(planar[size_t(n) + size_t(i)]),
                    float(planar[size_t(2 * n) + size_t(i)]));
  }
  return GList::from_container(std::move(arr));
}

static bool extract_coo_from_bundle(const BundlePtr &bundle,
                                    ls::CooMatrix &coo,
                                    std::string &error)
{
  if (!bundle) {
    error = "Matrix bundle A is empty";
    return false;
  }
  const std::optional<BundleKey> wkey = BundleKey::from_str("weight");
  const std::optional<BundleKey> rkey = BundleKey::from_str("row");
  const std::optional<BundleKey> ckey = BundleKey::from_str("col");
  if (!wkey || !rkey || !ckey) {
    error = "Invalid bundle key";
    return false;
  }
  const std::optional<GListPtr> weight_list = bundle->lookup<GListPtr>(*wkey);
  const std::optional<GListPtr> row_list = bundle->lookup<GListPtr>(*rkey);
  const std::optional<GListPtr> col_list = bundle->lookup<GListPtr>(*ckey);
  if (!weight_list || !row_list || !col_list || !*weight_list || !*row_list || !*col_list) {
    error = "Bundle A must contain lists weight, row, and col";
    return false;
  }
  list_to_doubles_float(*weight_list, coo.weight);
  list_to_ints(*row_list, coo.row);
  list_to_ints(*col_list, coo.col);
  const size_t n = std::min({coo.weight.size(), coo.row.size(), coo.col.size()});
  if (n == 0) {
    error = "COO lists are empty";
    return false;
  }
  coo.weight.resize(n);
  coo.row.resize(n);
  coo.col.resize(n);
  return true;
}

static ls::SolveOptions make_solve_options(const bNode &node, GeoNodeExecParams &params)
{
  ls::SolveOptions opt;
  opt.solver_class = get_solver_class(node);
  opt.direct = get_direct(node);
  opt.iterative = get_iterative(node);
  opt.preconditioner = get_precond(node);
  /* Force direct for decompose / solve-with-decomp. */
  if (get_mode(node) != Mode::Solve) {
    opt.solver_class = ls::SolverClass::Direct;
  }
  else if (opt.solver_class == ls::SolverClass::Iterative) {
    opt.max_iterations = std::max(1, params.extract_input<int>("Max Iterations"_ustr));
    opt.tolerance = std::max(0.0, double(params.extract_input<float>("Tolerance"_ustr)));
  }
  return opt;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const bNode &bnode = params.node();
  const Mode mode = get_mode(bnode);
  const DataType dtype = get_data_type(bnode);
  const DecomposeSubMode sub = get_decompose_sub(bnode);
  const int components = (dtype == DataType::Vector) ? 3 : 1;

  ls::CooMatrix coo;
  std::string error;
  auto fail = [&](const std::string &msg) {
    params.error_message_add(NodeWarningType::Error, msg);
    params.set_default_remaining_outputs();
  };

  auto extract_b = [&](std::vector<double> &b_out, int &n_pts) -> bool {
    GListPtr list = params.extract_input<GListPtr>("b"_ustr);
    if (dtype == DataType::Vector) {
      list_to_doubles_vector_planar(list, b_out, n_pts);
      return !b_out.empty();
    }
    list_to_doubles_float(list, b_out);
    n_pts = int(b_out.size());
    return !b_out.empty();
  };

  auto extract_pin = [&]() -> std::vector<char> {
    std::vector<char> mask;
    GListPtr list = params.extract_input<GListPtr>("Pin"_ustr);
    list_to_bool_mask(list, mask);
    return mask;
  };

  auto set_solution = [&](const ls::SolveResult &r, const int n_pts) {
    if (dtype == DataType::Vector) {
      params.set_output("x"_ustr, planar_to_vector_list(r.x, n_pts));
    }
    else {
      params.set_output("x"_ustr, doubles_to_float_list(r.x));
    }
  };

  if (mode == Mode::Solve) {
    BundlePtr A = params.extract_input<BundlePtr>("A"_ustr);
    if (!extract_coo_from_bundle(A, coo, error)) {
      fail(error);
      return;
    }
    std::vector<double> b;
    int n_pts = 0;
    extract_b(b, n_pts);
    const std::vector<char> pin = extract_pin();
    const ls::SolveOptions opt = make_solve_options(bnode, params);
    ls::SolveResult result;
    if (components == 3) {
      result = ls::solve_system_components(coo, b, 3, pin, opt);
      n_pts = ls::infer_dimension(coo, n_pts);
    }
    else {
      result = ls::solve_system(coo, b, pin, opt);
      n_pts = int(result.x.size());
    }
    if (!result.success) {
      fail(result.message.empty() ? "Linear solve failed" : result.message);
      return;
    }
    set_solution(result, n_pts);
    return;
  }

  if (mode == Mode::Decompose) {
    BundlePtr A = params.extract_input<BundlePtr>("A"_ustr);
    if (!extract_coo_from_bundle(A, coo, error)) {
      fail(error);
      return;
    }
    const std::vector<char> pin = extract_pin();
    /* Pin values default 0 when only mask is available at decompose time. */
    std::vector<double> pin_vals;

    if (sub == DecomposeSubMode::MatrixFactor) {
      ls::DirectMethod dm = get_direct(bnode);
      ls::Decomposition decomp = ls::decompose_system(coo, pin, pin_vals, dm);
      if (!decomp.valid) {
        fail(decomp.message.empty() ? "Decomposition failed" : decomp.message);
        return;
      }
      BundlePtr out = Bundle::create();
      auto *item = MEM_new<LinearSolverDecompositionItem>(__func__);
      item->data = std::move(decomp);
      ImplicitSharingPtr<LinearSolverDecompositionItem> ptr(item);
      out.ensure_mutable_inplace().add_new(*BundleKey::from_str("factor"), std::move(ptr));
      params.set_output("Decomposition"_ustr, std::move(out));
      return;
    }

    ls::EigenOptions eopt;
    eopt.kind = get_eigen_kind(bnode);
    eopt.num_eigenpairs = params.extract_input<int>("Num Eigenpairs"_ustr);
    if (is_shift_eigen_kind(eopt.kind)) {
      eopt.shift = double(params.extract_input<float>("Shift"_ustr));
      eopt.shift_invert = true;
    }
    else {
      eopt.shift = 0.0;
      eopt.shift_invert = false;
    }
    const ls::EigenResult er = ls::compute_eigenpairs(coo, pin, pin_vals, eopt);
    if (!er.success) {
      fail(er.message.empty() ? "Eigen decomposition failed" : er.message);
      return;
    }
    const int k = int(er.eigenvalues.size());
    const int n_eig = (k > 0) ? int(er.eigenvectors_flat.size()) / k : 0;
    params.set_output("Eigenvectors"_ustr,
                      eigenvecs_to_nested_float_lists(er.eigenvectors_flat, n_eig, k));
    params.set_output("Eigenvalues"_ustr, doubles_to_float_list(er.eigenvalues));
    return;
  }

  if (mode == Mode::SolveWithDecomposition) {
    std::vector<double> b;
    int n_pts = 0;
    extract_b(b, n_pts);
    BundlePtr decomp_bundle = params.extract_input<BundlePtr>("Decomposition"_ustr);
    if (!decomp_bundle) {
      fail("Decomposition input is empty");
      return;
    }
    const std::optional<ImplicitSharingPtr<LinearSolverDecompositionItem>> item =
        decomp_bundle->lookup_path<ImplicitSharingPtr<LinearSolverDecompositionItem>>("factor");
    if (!item || !*item) {
      fail("Decomposition bundle does not contain a valid factor");
      return;
    }
    if (components == 3) {
      /* Solve each component with the same factor. */
      const int n = (*item)->data.n;
      std::vector<double> x_planar(size_t(n) * 3, 0.0);
      for (int c = 0; c < 3; c++) {
        std::vector<double> bc(n, 0.0);
        /* b is planar with its own list length as stride, which may differ from n. */
        for (int i = 0; i < n && i < n_pts; i++) {
          bc[size_t(i)] = b[size_t(c) * size_t(n_pts) + size_t(i)];
        }
        const ls::SolveResult one = ls::solve_with_decomposition((*item)->data, bc);
        if (!one.success) {
          fail(one.message.empty() ? "Solve with decomposition failed" : one.message);
          return;
        }
        for (int i = 0; i < n && i < int(one.x.size()); i++) {
          x_planar[size_t(c) * size_t(n) + size_t(i)] = one.x[size_t(i)];
        }
      }
      params.set_output("x"_ustr, planar_to_vector_list(x_planar, n));
      return;
    }
    const ls::SolveResult result = ls::solve_with_decomposition((*item)->data, b);
    if (!result.success) {
      fail(result.message.empty() ? "Solve with decomposition failed" : result.message);
      return;
    }
    params.set_output("x"_ustr, doubles_to_float_list(result.x));
    return;
  }

  if (mode == Mode::Multiply) {
    BundlePtr A = params.extract_input<BundlePtr>("A"_ustr);
    if (!extract_coo_from_bundle(A, coo, error)) {
      fail(error);
      return;
    }
    GListPtr xlist = params.extract_input<GListPtr>("x"_ustr);
    if (dtype == DataType::Vector) {
      std::vector<double> x_planar;
      int n_pts = 0;
      list_to_doubles_vector_planar(xlist, x_planar, n_pts);
      const ls::MultiplyResult mr = ls::multiply_components(coo, x_planar, 3);
      if (!mr.success) {
        fail(mr.message.empty() ? "Multiply failed" : mr.message);
        return;
      }
      const int n = ls::infer_dimension(coo, n_pts);
      params.set_output("x"_ustr, planar_to_vector_list(mr.y, n));
      return;
    }
    std::vector<double> x;
    list_to_doubles_float(xlist, x);
    const ls::MultiplyResult mr = ls::multiply(coo, x);
    if (!mr.success) {
      fail(mr.message.empty() ? "Multiply failed" : mr.message);
      return;
    }
    params.set_output("x"_ustr, doubles_to_float_list(mr.y));
    return;
  }

  fail("Unknown mode");
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem mode_items[] = {
      {int(Mode::Solve), "SOLVE", 0, "Linear System Solve", "Solve sparse Ax=b"},
      {int(Mode::Decompose), "DECOMPOSE", 0, "Decompose", "Matrix factor or eigenpairs"},
      {int(Mode::SolveWithDecomposition),
       "SOLVE_WITH_DECOMPOSITION",
       0,
       "Solve with Decomposition",
       "Solve Ax=b by applying a stored matrix factorization (no direct/iterative choice)"},
      {int(Mode::Multiply), "MULTIPLY", 0, "Multiply", "y = A*x"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  static const EnumPropertyItem data_type_items[] = {
      {int(DataType::Float), "FLOAT", 0, "Float", "Scalar float lists for b/x"},
      {int(DataType::Vector), "VECTOR", 0, "Vector", "Vector lists; A applied per component"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  static const EnumPropertyItem solver_class_items[] = {
      {int(ls::SolverClass::Direct), "DIRECT", 0, "Direct", "Direct factorization solvers"},
      {int(ls::SolverClass::Iterative),
       "ITERATIVE",
       0,
       "Iterative",
       "Krylov iterative solvers with preconditioner"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  static const EnumPropertyItem direct_items[] = {
      {int(ls::DirectMethod::LLT), "LLT", 0, "LLT", "Cholesky LLT (SPD)"},
      {int(ls::DirectMethod::LDLT), "LDLT", 0, "LDLT", "LDLT (symmetric)"},
      {int(ls::DirectMethod::LU), "LU", 0, "LU", "Partial pivoting LU"},
      {int(ls::DirectMethod::QR), "QR", 0, "QR", "Column-pivoted QR"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  static const EnumPropertyItem iterative_items[] = {
      {int(ls::IterativeMethod::GMRES), "GMRES", 0, "GMRES", "Generalized Minimal Residual"},
      {int(ls::IterativeMethod::DGMRES), "DGMRES", 0, "DGMRES", "Deflated GMRES"},
      {int(ls::IterativeMethod::MINRES), "MINRES", 0, "MINRES", "Minimal Residual (symmetric)"},
      {int(ls::IterativeMethod::CG), "CG", 0, "CG", "Conjugate Gradient (SPD)"},
      {int(ls::IterativeMethod::LSCG), "LSCG", 0, "LSCG", "Least-Squares Conjugate Gradient"},
      {int(ls::IterativeMethod::BiCGSTAB), "BICGSTAB", 0, "BiCGSTAB", "BiCGSTAB"},
      {int(ls::IterativeMethod::IDRS), "IDRS", 0, "IDRS", "Induced Dimension Reduction"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  static const EnumPropertyItem precond_items[] = {
      {int(ls::Preconditioner::Identity), "IDENTITY", 0, "Identity", "No preconditioning"},
      {int(ls::Preconditioner::Diagonal), "DIAGONAL", 0, "Diagonal", "Jacobi / diagonal"},
      {int(ls::Preconditioner::IncompleteLUT),
       "INCOMPLETE_LUT",
       0,
       "IncompleteLUT",
       "Incomplete LU threshold"},
      {int(ls::Preconditioner::IncompleteCholesky),
       "INCOMPLETE_CHOLESKY",
       0,
       "IncompleteCholesky",
       "Incomplete Cholesky (SPD)"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  static const EnumPropertyItem decompose_sub_items[] = {
      {int(DecomposeSubMode::MatrixFactor),
       "MATRIX_FACTOR",
       0,
       "Matrix Factor",
       "Direct factorization for later solve"},
      {int(DecomposeSubMode::Eigen), "EIGEN", 0, "Eigen", "Eigenvalues and eigenvectors"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  static const EnumPropertyItem eigen_solver_items[] = {
      {int(ls::EigenSolverKind::GenEigsSolver),
       "GEN_EIGS",
       0,
       "GenEigsSolver",
       "General (non-symmetric) eigenproblem"},
      {int(ls::EigenSolverKind::SymEigsSolver),
       "SYM_EIGS",
       0,
       "SymEigsSolver",
       "Symmetric / self-adjoint eigenproblem"},
      {int(ls::EigenSolverKind::GenEigsRealShiftSolver),
       "GEN_EIGS_REAL_SHIFT",
       0,
       "GenEigsRealShiftSolver",
       "General eigenproblem with real shift (shift-invert)"},
      {int(ls::EigenSolverKind::SymEigsShiftSolver),
       "SYM_EIGS_SHIFT",
       0,
       "SymEigsShiftSolver",
       "Symmetric eigenproblem with shift (shift-invert)"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(
      srna,
      "mode",
      "Mode",
      "Operation",
      mode_items,
      EnumRNAAccessors(
          [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
            return int(get_mode(*static_cast<const bNode *>(ptr->data)));
          },
          [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
            bNode &node = *static_cast<bNode *>(ptr->data);
            node.custom1 = int16_t((node.custom1 & ~0xff) | (value & 0xff));
          }),
      int(Mode::Solve));

  RNA_def_node_enum(
      srna,
      "data_type",
      "Data Type",
      "Float or vector lists for b and x",
      data_type_items,
      EnumRNAAccessors(
          [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
            return int(get_data_type(*static_cast<const bNode *>(ptr->data)));
          },
          [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
            bNode &node = *static_cast<bNode *>(ptr->data);
            node.custom1 = int16_t((node.custom1 & ~(0xff << 8)) | ((value & 0xff) << 8));
          }),
      int(DataType::Float));

  RNA_def_node_enum(
      srna,
      "solver_class",
      "Solver Class",
      "Direct or iterative linear solver",
      solver_class_items,
      EnumRNAAccessors(
          [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
            return int(get_solver_class(*static_cast<const bNode *>(ptr->data)));
          },
          [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
            bNode &node = *static_cast<bNode *>(ptr->data);
            node.custom2 = int16_t((node.custom2 & ~0x3) | (value & 0x3));
          }),
      int(ls::SolverClass::Direct));

  RNA_def_node_enum(
      srna,
      "direct_method",
      "Direct Method",
      "Direct factorization method",
      direct_items,
      EnumRNAAccessors(
          [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
            return int(get_direct(*static_cast<const bNode *>(ptr->data)));
          },
          [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
            bNode &node = *static_cast<bNode *>(ptr->data);
            node.custom2 = int16_t((node.custom2 & ~(0xf << 2)) | ((value & 0xf) << 2));
          }),
      int(ls::DirectMethod::LLT));

  RNA_def_node_enum(
      srna,
      "iterative_method",
      "Iterative Method",
      "Krylov iterative method",
      iterative_items,
      EnumRNAAccessors(
          [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
            return int(get_iterative(*static_cast<const bNode *>(ptr->data)));
          },
          [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
            bNode &node = *static_cast<bNode *>(ptr->data);
            node.custom2 = int16_t((node.custom2 & ~(0xf << 6)) | ((value & 0xf) << 6));
          }),
      int(ls::IterativeMethod::CG));

  RNA_def_node_enum(
      srna,
      "preconditioner",
      "Preconditioner",
      "Preconditioner for iterative methods",
      precond_items,
      EnumRNAAccessors(
          [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
            return int(get_precond(*static_cast<const bNode *>(ptr->data)));
          },
          [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
            bNode &node = *static_cast<bNode *>(ptr->data);
            node.custom2 = int16_t((node.custom2 & ~(0xf << 10)) | ((value & 0xf) << 10));
          }),
      int(ls::Preconditioner::Diagonal));

  RNA_def_node_enum(
      srna,
      "decompose_sub_mode",
      "Decompose Mode",
      "Matrix factorization or eigen decomposition",
      decompose_sub_items,
      EnumRNAAccessors(
          [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
            return int(get_decompose_sub(*static_cast<const bNode *>(ptr->data)));
          },
          [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
            bNode &node = *static_cast<bNode *>(ptr->data);
            const int eigen = (int(node.custom3) >> 4) & 0xf;
            node.custom3 = float((value & 0xf) | (eigen << 4));
          }),
      int(DecomposeSubMode::MatrixFactor));

  RNA_def_node_enum(
      srna,
      "eigen_solver",
      "Eigen Solver",
      "Eigenvalue solver style (Spectra-like)",
      eigen_solver_items,
      EnumRNAAccessors(
          [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
            return int(get_eigen_kind(*static_cast<const bNode *>(ptr->data)));
          },
          [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
            bNode &node = *static_cast<bNode *>(ptr->data);
            const int sub = int(node.custom3) & 0xf;
            node.custom3 = float(sub | ((value & 0xf) << 4));
          }),
      int(ls::EigenSolverKind::SymEigsSolver));
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = int16_t(Mode::Solve);
  node->custom2 = int16_t(int(ls::SolverClass::Direct) | (int(ls::DirectMethod::LLT) << 2) |
                          (int(ls::IterativeMethod::CG) << 6) |
                          (int(ls::Preconditioner::Diagonal) << 10));
  node->custom3 = float(int(DecomposeSubMode::MatrixFactor) |
                        (int(ls::EigenSolverKind::SymEigsSolver) << 4));
  node->custom4 = 0.0f;
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeLinearSolver"_ustr, GEO_NODE_LINEAR_SOLVER);
  ntype.ui_name = "Linear Solver";
  ntype.ui_description =
      "Sparse linear solver (Eigen + Spectra): solve, factor, eigen, multiply; "
      "COO bundle weight/row/col";
  ntype.enum_name_legacy = "LINEAR_SOLVER";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  /* Availability is set in declare via .available(); do not use updatefunc +
   * input_by_identifier (breaks on float↔vector rebuild / dirty topology). */
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_linear_solver_cc
