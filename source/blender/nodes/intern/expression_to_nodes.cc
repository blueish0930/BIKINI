/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>
#include <sstream>

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <fast_float.h>

#include "BLI_listbase.hh"
#include "BLI_resource_scope.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLT_translation.hh"

#include "NOD_expression_parse.hh"
#include "NOD_expression_to_nodes.hh"
#include "NOD_fn_format_string.hh"
#include "NOD_socket.hh"
#include "NOD_socket_items.hh"

#include "BKE_lib_id.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_dot_export.hh"

#include "DNA_node_types.h"

namespace blender::nodes::expression {

class AstToNodeGroupBuilder;

struct NodeAndSocket {
  bNode *node = nullptr;
  bNodeSocket *socket = nullptr;

  operator bool() const
  {
    return this->socket != nullptr;
  }
};

/** A value type does not necessarily have to correspond to a single socket. */
enum class ValueType {
  Float,
  Vec2,
  Vec3,
  String,
  Rgb,
  Rgba,
  Boolean,
  Integer,
  /** 4x4 transform / matrix (Geometry/Compositor sockets). */
  Matrix,
  /** Quaternion rotation (Geometry/Compositor sockets). */
  Rotation,
};

static std::optional<ValueType> socket_to_value_type(const bke::bNodeTreeType &tree_type,
                                                     const bNodeSocket &socket)
{
  switch (socket.type) {
    case SOCK_FLOAT:
      return ValueType::Float;
    case SOCK_VECTOR:
      /* TODO: Use dimensions? */
      return ValueType::Vec3;
    case SOCK_STRING:
      return ValueType::String;
    case SOCK_RGBA:
      return tree_type.type == NTREE_SHADER ? ValueType::Rgb : ValueType::Rgba;
    case SOCK_BOOLEAN:
      return ValueType::Boolean;
    case SOCK_INT:
      return ValueType::Integer;
    case SOCK_MATRIX:
      return ValueType::Matrix;
    case SOCK_ROTATION:
      return ValueType::Rotation;
    default:
      return std::nullopt;
  }
}

static std::optional<eNodeSocketDatatype> value_to_closest_socket_type(const ValueType type)
{
  switch (type) {
    case ValueType::Float:
      return SOCK_FLOAT;
    case ValueType::Vec2:
      return SOCK_VECTOR;
    case ValueType::Vec3:
      return SOCK_VECTOR;
    case ValueType::String:
      return SOCK_STRING;
    case ValueType::Rgb:
      return SOCK_RGBA;
    case ValueType::Rgba:
      return SOCK_RGBA;
    case ValueType::Boolean:
      return SOCK_BOOLEAN;
    case ValueType::Integer:
      return SOCK_INT;
    case ValueType::Matrix:
      return SOCK_MATRIX;
    case ValueType::Rotation:
      return SOCK_ROTATION;
  }
  return std::nullopt;
}

static StringRefNull get_value_type_name(const ValueType type)
{
  switch (type) {
    case ValueType::Float:
      return "float";
    case ValueType::Vec2:
      return "vec2";
    case ValueType::Vec3:
      return "vec3";
    case ValueType::String:
      return "string";
    case ValueType::Rgb:
      return "rgb";
    case ValueType::Rgba:
      return "rgba";
    case ValueType::Boolean:
      return "bool";
    case ValueType::Integer:
      return "int";
    case ValueType::Matrix:
      return "matrix";
    case ValueType::Rotation:
      return "rotation";
  }
  return "";
}

class Value {
 public:
  ValueType type;
  /**
   * Usually, a single value corresponds to a single socket. However, in general, this is not
   * necessarily the case. For example, a 4d vector might be represented by a 3d vector and a float
   * socket. This allows supporting intermediate types in expressions that are not supported by the
   * underlying node tree type.
   */
  Vector<NodeAndSocket, 1> sockets;

  Value(ValueType type, bNode &node, bNodeSocket &socket) : type(type), sockets({{&node, &socket}})
  {
  }

  Value(ValueType type, Vector<NodeAndSocket> sockets) : type(type), sockets(std::move(sockets)) {}
};

struct TypeCheckCallParams {
  const bke::bNodeTreeType &tree_type;
  Vector<const bke::bNodeSocketType *> input_types;
};

static bNodeSocket *find_available_socket_by_index(ListBaseT<bNodeSocket> &sockets,
                                                   const int index)
{
  int remaining = index;
  for (bNodeSocket &socket : sockets) {
    if (!socket.is_available()) {
      continue;
    }
    if (remaining == 0) {
      return &socket;
    }
    remaining--;
  }
  return nullptr;
}

struct InsertCallParams {
  AstToNodeGroupBuilder &builder;
  const bNodeTree &tree;

  Vector<Value> inputs;
  std::optional<Value> output;

  bNode &add_node(const UString idname);
  void update_node_sockets(bNode &node);

  void add_input(Value value)
  {
    this->inputs.append(std::move(value));
  }

  void add_input(bNode &node, bNodeSocket &socket, const ValueType type)
  {
    this->add_input(Value(type, node, socket));
  }

  void add_input(bNode &node, bNodeSocket &socket)
  {
    this->add_input(node, socket, *socket_to_value_type(*tree.typeinfo, socket));
  }

  void add_input(bNode &node, const int index, const ValueType type)
  {
    bNodeSocket *socket = find_available_socket_by_index(node.inputs, index);
    BLI_assert(socket);
    this->add_input(node, *socket, type);
  }

  void add_input(bNode &node, const int index)
  {
    bNodeSocket *socket = find_available_socket_by_index(node.inputs, index);
    BLI_assert(socket);
    this->add_input(node, *socket);
  }

  void set_output(Value value)
  {
    /* Should only be set once. */
    BLI_assert(!this->output.has_value());
    this->output = std::move(value);
  }

  void set_output(bNode &node, bNodeSocket &socket, const ValueType type)
  {
    this->set_output(Value(type, node, socket));
  }

  void set_output(bNode &node, bNodeSocket &socket)
  {
    this->set_output(node, socket, *socket_to_value_type(*tree.typeinfo, socket));
  }

  void set_output(bNode &node, const int index, const ValueType type)
  {
    bNodeSocket *socket = find_available_socket_by_index(node.outputs, index);
    BLI_assert(socket);
    this->set_output(node, *socket, type);
  }

  void set_output(bNode &node, const int index)
  {
    bNodeSocket *socket = find_available_socket_by_index(node.outputs, index);
    BLI_assert(socket);
    this->set_output(node, *socket);
  }

  void use_node_sockets(bNode &node)
  {
    this->use_node_inputs(node);
    this->use_node_output(node);
  }

  void use_node_inputs(bNode &node)
  {
    for (bNodeSocket &socket : node.inputs) {
      if (socket.is_available()) {
        this->add_input(node, socket);
      }
    }
  }

  void use_node_output(bNode &node)
  {
    for (bNodeSocket &socket : node.outputs) {
      if (socket.is_available()) {
        this->set_output(node, socket);
      }
    }
  }
};

using InsertCallFn = std::function<void(InsertCallParams &params)>;
using TypeCheckCallFn = std::function<bool(TypeCheckCallParams &params)>;

struct FunctionSymbolParam {
  ValueType type;
  bool needs_exact = false;
};

class FunctionSymbol {
 public:
  std::string name;
  Vector<FunctionSymbolParam> params;
  std::optional<Vector<const bke::bNodeTreeType *>> allowed_tree_types;
  InsertCallFn insert;

  FunctionSymbol(std::string name,
                 Vector<FunctionSymbolParam> params,
                 InsertCallFn insert,
                 std::optional<Vector<const bke::bNodeTreeType *>> allowed_tree_types = {})
      : name(std::move(name)),
        params(std::move(params)),
        allowed_tree_types(allowed_tree_types),
        insert(std::move(insert))
  {
  }
};

using ImplicitConversionFn = std::function<Value(const Value &value, ValueType to_type)>;
using FinalizeValueFn = std::function<NodeAndSocket(
    bNodeTree &tree, const Value &value, const eNodeSocketDatatype to_type)>;

class SymbolTable {
 private:
  MultiValueMap<std::string, FunctionSymbol> function_;
  Map<std::pair<ValueType, ValueType>, ImplicitConversionFn> implicit_conversions_;
  Map<std::pair<ValueType, eNodeSocketDatatype>, FinalizeValueFn> finalize_value_fns_;

  struct MatchResult {
    Vector<bool> param_needs_conversion;

    bool is_better_than(const MatchResult &other) const
    {
      int self_conversion_num = 0;
      int other_conversion_num = 0;
      for (const int i : this->param_needs_conversion.index_range()) {
        const bool self_needs_conversion = this->param_needs_conversion[i];
        const bool other_needs_conversion = other.param_needs_conversion[i];
        if (self_needs_conversion && !other_needs_conversion) {
          return false;
        }
        self_conversion_num += self_needs_conversion;
        other_conversion_num += other_needs_conversion;
      }
      return self_conversion_num < other_conversion_num;
    }
  };

 public:
  void add(FunctionSymbol function_symbol)
  {
    function_.add(function_symbol.name, std::move(function_symbol));
  }

  void add_implicit_conversion(const ValueType from_type, const ValueType to_type)
  {
    this->add_custom_implicit_conversion(
        from_type, to_type, [to_type](const Value &value, ValueType /*to_type*/) {
          return Value(to_type, value.sockets);
        });
  }

  void add_custom_implicit_conversion(const ValueType from_type,
                                      const ValueType to_type,
                                      ImplicitConversionFn fn)
  {
    implicit_conversions_.add_new(std::pair(from_type, to_type), std::move(fn));
  }

  void add_finalize_value_fn(const ValueType from_type,
                             const eNodeSocketDatatype to_type,
                             FinalizeValueFn fn)
  {
    finalize_value_fns_.add_new(std::pair(from_type, to_type), std::move(fn));
  }

  const ImplicitConversionFn *lookup_implicit_conversion(const ValueType from_type,
                                                         const ValueType to_type) const
  {
    return implicit_conversions_.lookup_ptr(std::pair(from_type, to_type));
  }

  const FinalizeValueFn *lookup_finalize_value_fn(const ValueType from_type,
                                                  const eNodeSocketDatatype to_type) const
  {
    return finalize_value_fns_.lookup_ptr(std::pair(from_type, to_type));
  }

  const FunctionSymbol *lookup_function(const StringRef name,
                                        const bke::bNodeTreeType &tree_type,
                                        const Span<ValueType> input_types,
                                        std::string &r_error) const
  {
    const Span<FunctionSymbol> candidates = function_.lookup(name);
    if (candidates.is_empty()) {
      r_error = fmt::format("{}: '{}'", TIP_("Unknown function"), name);
      return nullptr;
    }
    std::optional<MatchResult> best_match;
    Vector<const FunctionSymbol *> best_matching_functions;
    for (const FunctionSymbol &function : candidates) {
      const std::optional<MatchResult> match = this->compute_match(
          function, tree_type, input_types);
      if (!match) {
        continue;
      }
      if (!best_match) {
        best_match = std::move(*match);
        best_matching_functions.append(&function);
        continue;
      }
      if (match->is_better_than(*best_match)) {
        best_match = std::move(*match);
        best_matching_functions.clear();
        best_matching_functions.append(&function);
        continue;
      }
      if (best_match->is_better_than(*match)) {
        continue;
      }
      /* Both functions are equally good matches. */
      best_matching_functions.append(&function);
    }

    if (best_matching_functions.is_empty()) {
      Vector<StringRef> param_types;
      for (const ValueType input_type : input_types) {
        param_types.append(get_value_type_name(input_type));
      }
      r_error = fmt::format(
          "{}: {}({})", TIP_("No matching function"), name, fmt::join(param_types, ", "));
      return nullptr;
    }
    if (best_matching_functions.size() >= 2) {
      r_error = fmt::format("{}: '{}'", TIP_("Ambiguous function call"), name);
      return nullptr;
    }
    const FunctionSymbol *selected_function = best_matching_functions.first();
    return selected_function;
  }

  std::optional<MatchResult> compute_match(const FunctionSymbol &function,
                                           const bke::bNodeTreeType & /*tree_type*/,
                                           const Span<ValueType> input_types) const
  {
    if (function.params.size() != input_types.size()) {
      return std::nullopt;
    }
    Vector<bool> param_needs_conversion(input_types.size(), false);
    for (const int i : IndexRange(input_types.size())) {
      const ValueType input_type = input_types[i];
      const FunctionSymbolParam &param = function.params[i];
      if (input_type == param.type) {
        continue;
      }
      if (param.needs_exact) {
        return std::nullopt;
      }
      if (!this->lookup_implicit_conversion(input_type, param.type)) {
        return std::nullopt;
      }
      param_needs_conversion[i] = true;
    }
    return MatchResult{std::move(param_needs_conversion)};
  }
};

struct BuildOptions {};

class AstToNodeGroupBuilder {
 private:
  /**
   * Optional bmain, only non-null when building a node group that is added to the main database.
   */
  Main *bmain_;
  const NodeExpression &bnode_storage_;
  const Span<ast::Expr *> root_exprs_;
  const Span<int> expr_indices_;
  const SymbolTable &symbol_table_;
  [[maybe_unused]] const BuildOptions &options_;

  bNodeTree &r_tree_;
  std::string &r_error_;

  Map<StringRef, Value> inputs_;

  friend InsertCallParams;

 public:
  AstToNodeGroupBuilder(Main *bmain,
                        const bNode &expr_bnode,
                        const Span<ast::Expr *> root_exprs,
                        const Span<int> expr_indices,
                        const SymbolTable &symbol_table,
                        const BuildOptions &options,
                        bNodeTree &r_tree,
                        std::string &r_error)
      : bmain_(bmain),
        bnode_storage_(*static_cast<const NodeExpression *>(expr_bnode.storage)),
        root_exprs_(root_exprs),
        expr_indices_(expr_indices),
        symbol_table_(symbol_table),
        options_(options),
        r_tree_(r_tree),
        r_error_(r_error)
  {
  }

  void build()
  {
    this->add_interface_inputs();
    this->add_interface_outputs();

    bNode &group_input_node = this->add_node("NodeGroupInput"_ustr);
    bNode &group_output_node = this->add_node("NodeGroupOutput"_ustr);

    {
      bNodeSocket *group_input_socket = static_cast<bNodeSocket *>(group_input_node.outputs.first());
      for ([[maybe_unused]] const int i : IndexRange(bnode_storage_.input_items.items_num)) {
        const NodeExpressionInputItem &item = bnode_storage_.input_items.items[i];
        const std::optional<ValueType> input_type = socket_to_value_type(*r_tree_.typeinfo,
                                                                         *group_input_socket);
        if (!input_type) {
          r_error_ = TIP_("Unsupported socket_type for input");
          return;
        }
        inputs_.add(item.name, Value(*input_type, group_input_node, *group_input_socket));
        group_input_socket = group_input_socket->next;
      }
    }

    {
      bNodeSocket *group_output_socket = static_cast<bNodeSocket *>(group_output_node.inputs.first());
      for (const int i : expr_indices_.index_range()) {
        std::optional<Value> expr_result = this->build_expr(*root_exprs_[i]);
        if (!expr_result) {
          return;
        }
        NodeAndSocket eval_value_socket;
        if (const FinalizeValueFn *finalize_fn = symbol_table_.lookup_finalize_value_fn(
                expr_result->type, group_output_socket->type))
        {
          eval_value_socket = (*finalize_fn)(r_tree_, *expr_result, group_output_socket->type);
        }
        else {
          BLI_assert(!expr_result->sockets.is_empty());
          eval_value_socket = expr_result->sockets[0];
        }
        bke::node_add_link(r_tree_,
                           *eval_value_socket.node,
                           *eval_value_socket.socket,
                           group_output_node,
                           *group_output_socket);
        group_output_socket = group_output_socket->next;
      }
    }

    if (bmain_) {
      BKE_ntree_update(*bmain_);
    }
    else {
      BKE_ntree_update_without_main(r_tree_);
    }

    if (bmain_) {
      this->position_nodes_in_output_tree();
    }
  }

 private:
  void add_interface_inputs()
  {
    for (const int i : IndexRange(bnode_storage_.input_items.items_num)) {
      const NodeExpressionInputItem &item = bnode_storage_.input_items.items[i];
      const bke::bNodeSocketType *stype = bke::node_socket_type_find_static(item.socket_type);
      bNodeTreeInterfaceSocket *iosock = r_tree_.tree_interface.add_socket(
          item.name, "", stype->idname.ref(), NODE_INTERFACE_SOCKET_INPUT, nullptr);
      /* Accept single values and fields from the parent Expression node. */
      if (iosock) {
        iosock->structure_type = NodeSocketInterfaceStructureType::Dynamic;
      }
    }
  }

  void add_interface_outputs()
  {
    for (const int i : expr_indices_.index_range()) {
      const NodeExpressionItem &expr_item =
          bnode_storage_.expression_items.items[expr_indices_[i]];
      const bke::bNodeSocketType *output_stype = bke::node_socket_type_find_static(
          expr_item.socket_type);
      bNodeTreeInterfaceSocket *iosock = r_tree_.tree_interface.add_socket(
          expr_item.name, "", output_stype->idname.ref(), NODE_INTERFACE_SOCKET_OUTPUT, nullptr);
      if (iosock) {
        iosock->structure_type = NodeSocketInterfaceStructureType::Dynamic;
      }
    }
  }

  std::optional<Value> build_expr(const ast::Expr &expr)
  {
    return std::visit([&](const auto &ast_node) { return this->build_expr(ast_node); }, expr.expr);
  }

  std::optional<Value> build_expr(const ast::NumberLiteral &ast_node)
  {
    /* TODO: Handle floats vs. integers. */
    float value;
    fast_float::from_chars_result result = fast_float::from_chars(
        ast_node.value.begin(), ast_node.value.end(), value);
    if (result.ec != std::errc()) {
      r_error_ = fmt::format("{}: {}", TIP_("Invalid number"), ast_node.value);
      return {};
    }
    bNode &node = this->add_node("ShaderNodeValue"_ustr);
    bNodeSocket *socket = static_cast<bNodeSocket *>(node.outputs.first());
    socket->default_value_typed<bNodeSocketValueFloat>()->value = value;
    return Value(ValueType::Float, node, *socket);
  }

  std::optional<Value> build_expr(const ast::StringLiteral &ast_node)
  {
    bNode &node = this->add_node("FunctionNodeInputString"_ustr);
    auto &storage = *static_cast<NodeInputString *>(node.storage);
    const StringRef str = ast_node.value.drop_known_prefix("\"").drop_known_suffix("\"");
    storage.string = BLI_strdupn(str.data(), str.size());
    return Value(ValueType::String, node, *static_cast<bNodeSocket *>(node.outputs.first()));
  }

  std::optional<Value> build_expr(const ast::Identifier &ast_node)
  {
    std::optional<Value> input = inputs_.lookup_try(ast_node.identifier);
    if (!input) {
      r_error_ = fmt::format("{}: {}", TIP_("Unknown variable"), ast_node.identifier);
      return {};
    }
    return input;
  }

  std::optional<Value> build_expr(const ast::BinaryOp &ast_node)
  {
    return this->build_generic_call(ast_node.op, {ast_node.a, ast_node.b});
  }

  std::optional<Value> build_expr(const ast::UnaryOp &ast_node)
  {
    return this->build_generic_call(ast_node.op, {ast_node.expr});
  }

  std::optional<Value> build_expr(const ast::ConditionalOp &ast_node)
  {
    return this->build_generic_call("?:",
                                    {ast_node.condition, ast_node.true_expr, ast_node.false_expr});
  }

  std::optional<Value> build_expr(const ast::MemberAccess &ast_node)
  {
    return this->build_generic_call("." + ast_node.identifier, {ast_node.expr});
  }

  std::optional<Value> build_expr(const ast::Call &ast_node)
  {
    if (const ast::Identifier *identifier = std::get_if<ast::Identifier>(&ast_node.function->expr))
    {
      return this->build_generic_call(identifier->identifier, ast_node.args);
    }
    r_error_ = TIP_("Unexpected function call");
    return {};
  }

  std::optional<Value> build_generic_call(const StringRef name, const Span<const ast::Expr *> args)
  {
    Vector<Value> arg_values;
    Vector<ValueType> arg_types(args.size());
    for (const int i : args.index_range()) {
      std::optional<Value> arg_value = this->build_expr(*args[i]);
      if (!arg_value) {
        /* There is an error in the argument. */
        return {};
      }
      arg_types[i] = arg_value->type;
      arg_values.append(std::move(*arg_value));
    }

    const FunctionSymbol *function = symbol_table_.lookup_function(
        name, *r_tree_.typeinfo, arg_types, r_error_);
    if (!function) {
      BLI_assert(!r_error_.empty());
      return {};
    }

    for (const int i : args.index_range()) {
      const ValueType arg_type = arg_types[i];
      const ValueType param_type = function->params[i].type;
      if (arg_type != param_type) {
        std::optional<Value> converted_value = this->implicitly_convert(arg_values[i], param_type);
        /* Should always work, otherwise the function lookup should have failed already. */
        BLI_assert(converted_value);
        arg_values[i] = std::move(*converted_value);
      }
    }

    InsertCallParams insert_params{*this, r_tree_};
    function->insert(insert_params);
    BLI_assert(insert_params.inputs.size() == args.size());
    BLI_assert(insert_params.output.has_value());

    for (const int i : args.index_range()) {
      this->link_values(arg_values[i], insert_params.inputs[i]);
    }
    return insert_params.output;
  }

  bNode &add_node(const UString idname)
  {
    return *bke::node_add_node(nullptr, r_tree_, idname);
  }

  void link_values(const Value &from, const Value &to)
  {
    BLI_assert(from.type == to.type);
    BLI_assert(from.sockets.size() == to.sockets.size());
    for (const int i : from.sockets.index_range()) {
      const NodeAndSocket &from_socket = from.sockets[i];
      const NodeAndSocket &to_socket = to.sockets[i];
      bke::node_add_link(
          r_tree_, *from_socket.node, *from_socket.socket, *to_socket.node, *to_socket.socket);
    }
  }

  std::optional<Value> implicitly_convert(const Value &value, const ValueType to_type)
  {
    const ValueType from_type = value.type;
    if (from_type == to_type) {
      return value;
    }
    const ImplicitConversionFn *fn = symbol_table_.lookup_implicit_conversion(from_type, to_type);
    if (!fn) {
      r_error_ = fmt::format("{}: {} -> {}",
                             TIP_("No implicit conversion"),
                             get_value_type_name(from_type),
                             get_value_type_name(to_type));
      return std::nullopt;
    }
    std::optional<Value> converted_value = (*fn)(value, to_type);
    return converted_value;
  }

  void position_nodes_in_output_tree()
  {
    bNodeTree &tree = r_tree_;
    tree.ensure_topology_cache();

    Map<int, int> num_by_depth;
    Map<bNode *, int> depth_by_node;

    /* Simple algorithm that does a very rough layout of the generated tree. This does not produce
     * great results generally, but is usually good enough when debugging smaller node trees. */
    for (bNode *node : tree.toposort_right_to_left()) {
      int depth = 0;
      for (bNodeSocket *socket : node->output_sockets()) {
        for (bNodeSocket *target : socket->directly_linked_sockets()) {
          depth = std::max(depth, depth_by_node.lookup(&target->owner_node()) + 1);
        }
      }
      depth_by_node.add_new(node, depth);
      const int index_at_depth = num_by_depth.lookup_or_add(depth, 0)++;
      node->location[0] = 200 - depth * 200;
      node->location[1] = -index_at_depth * 300;
    }
  }
};

static FunctionSymbol float_math_function(const StringRef name,
                                          const NodeMathOperation op,
                                          const int inputs_num)
{
  Vector<FunctionSymbolParam> param_types(inputs_num, {ValueType::Float});
  return FunctionSymbol(name, std::move(param_types), [op](InsertCallParams &params) {
    bNode &math_node = params.add_node("ShaderNodeMath"_ustr);
    math_node.custom1 = op;
    params.update_node_sockets(math_node);
    params.use_node_sockets(math_node);
  });
}

static FunctionSymbol vector_math_element_wise(const StringRef name,
                                               const NodeVectorMathOperation op,
                                               const ValueType vector_type,
                                               const Span<ValueType> input_types)
{
  Vector<FunctionSymbolParam> param_types(input_types.size());
  for (const int i : input_types.index_range()) {
    param_types[i] = FunctionSymbolParam{input_types[i]};
  }
  return FunctionSymbol(
      name,
      std::move(param_types),
      [op, vector_type, input_types = Vector<ValueType>(input_types)](InsertCallParams &params) {
        bNode &math_node = params.add_node("ShaderNodeVectorMath"_ustr);
        math_node.custom1 = op;
        for (const int i : input_types.index_range()) {
          params.add_input(math_node, i, input_types[i]);
        }
        params.set_output(math_node, 0, vector_type);
      }

  );
}

static FunctionSymbol negate_float_function()
{
  return FunctionSymbol("-", {{ValueType::Float}}, [](InsertCallParams &params) {
    bNode &math_node = params.add_node("ShaderNodeMath"_ustr);
    math_node.custom1 = NODE_MATH_SUBTRACT;
    params.update_node_sockets(math_node);
    static_cast<bNodeSocket *>(math_node.inputs.first())
        ->default_value_typed<bNodeSocketValueFloat>()
        ->value = 0.0f;
    params.add_input(math_node, 1);
    params.use_node_output(math_node);
  });
}

static FunctionSymbol vector_member_access(const ValueType type, const int index)
{
  BLI_assert(index >= 0 && index <= 2);
  return FunctionSymbol(fmt::format(".{}", char('x' + index)),
                        {{type, true}},
                        [type, index](InsertCallParams &params) {
                          bNode &node = params.add_node("ShaderNodeSeparateXYZ"_ustr);
                          params.add_input(node, 0, type);
                          params.set_output(node, index);
                        });
}

static FunctionSymbol string_concatenation()
{
  return FunctionSymbol(
      "+", {{ValueType::String}, {ValueType::String}}, [](InsertCallParams &params) {
        bNode &node = params.add_node("FunctionNodeFormatString"_ustr);
        auto &storage = *static_cast<NodeFunctionFormatString *>(node.storage);
        storage.items = MEM_new_array<NodeFunctionFormatStringItem>(2, "string_concatenation");
        NodeFunctionFormatStringItem &item0 = storage.items[0];
        NodeFunctionFormatStringItem &item1 = storage.items[1];
        item0.identifier = storage.next_identifier++;
        item1.identifier = storage.next_identifier++;
        item0.name = BLI_strdup("a");
        item1.name = BLI_strdup("b");
        item0.socket_type = SOCK_STRING;
        item1.socket_type = SOCK_STRING;
        storage.items_num = 2;
        params.update_node_sockets(node);
        STRNCPY(static_cast<bNodeSocket *>(node.inputs.first())
                    ->default_value_typed<bNodeSocketValueString>()
                    ->value,
                "{a}{b}");
        params.add_input(node, 1);
        params.add_input(node, 2);
        params.use_node_output(node);
      });
}

static FunctionSymbol ternary_conditional_operator(const ValueType type)
{
  return FunctionSymbol(
      "?:", {{ValueType::Boolean}, {type}, {type}}, [type](InsertCallParams &params) {
        const eNodeSocketDatatype socket_type = *value_to_closest_socket_type(type);
        if (ELEM(params.tree.type, NTREE_GEOMETRY, NTREE_COMPOSIT)) {
          bNode &node = params.add_node("GeometryNodeSwitch"_ustr);
          auto &storage = *static_cast<NodeSwitch *>(node.storage);
          storage.input_type = socket_type;
          params.update_node_sockets(node);
          params.add_input(node, 0);
          params.add_input(node, 2, type);
          params.add_input(node, 1, type);
          params.set_output(node, 0, type);
          return;
        }
        bNode &node = params.add_node("ShaderNodeMix"_ustr);
        NodeShaderMix &storage = *static_cast<NodeShaderMix *>(node.storage);
        storage.clamp_factor = false;
        if (ELEM(socket_type, SOCK_FLOAT, SOCK_INT, SOCK_BOOLEAN)) {
          storage.data_type = socket_type;
        }
        else if (socket_type == SOCK_VECTOR) {
          storage.data_type = SOCK_VECTOR;
        }
        else {
          storage.data_type = SOCK_RGBA;
        }
        params.update_node_sockets(node);
        params.add_input(node, 0);
        params.add_input(node, 1);
        params.add_input(node, 2);
        params.set_output(node, 0, type);
      });
}

static FunctionSymbol vec2_from_scalars()
{
  return FunctionSymbol(
      "vec2", {{ValueType::Float}, {ValueType::Float}}, [](InsertCallParams &params) {
        bNode &node = params.add_node("ShaderNodeCombineXYZ"_ustr);
        params.add_input(node, 0);
        params.add_input(node, 1);
        params.set_output(node, 0, ValueType::Vec2);
      });
}

static FunctionSymbol vec3_from_scalars()
{
  return FunctionSymbol("vec3",
                        {{ValueType::Float}, {ValueType::Float}, {ValueType::Float}},
                        [](InsertCallParams &params) {
                          bNode &node = params.add_node("ShaderNodeCombineXYZ"_ustr);
                          params.use_node_sockets(node);
                        });
}

static FunctionSymbol vec2_from_vec3()
{
  return FunctionSymbol("vec2", {{ValueType::Vec3}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("ShaderNodeVectorMath"_ustr);
    node.custom1 = NODE_VECTOR_MATH_MULTIPLY;
    params.add_input(node, 0, ValueType::Vec3);
    bNodeSocket *mul_socket = static_cast<bNodeSocket *>(node.inputs.first())->next;
    float *value = mul_socket->default_value_typed<bNodeSocketValueVector>()->value;
    value[0] = 1.0f;
    value[1] = 1.0f;
    value[2] = 0.0f;
    value[3] = 0.0f;
    params.set_output(node, 0, ValueType::Vec2);
  });
}

static FunctionSymbol implicit_conversion_node_function(
    const StringRef name,
    const ValueType input_type,
    const ValueType output_type,
    const eNodeSocketDatatype output_socket_type)
{
  return FunctionSymbol(name,
                        {{input_type}},
                        [input_type, output_type, output_socket_type](InsertCallParams &params) {
                          const bke::bNodeSocketType *stype = bke::node_socket_type_find_static(
                              output_socket_type);
                          bNode &node = params.add_node("NodeImplicitConversion"_ustr);
                          auto &storage = *static_cast<NodeImplicitConversion *>(node.storage);
                          STRNCPY_UTF8(storage.type_idname, stype->idname.c_str());
                          params.add_input(node, 0, input_type);
                          params.set_output(node, 0, output_type);
                        });
}

static FunctionSymbol vec3_from_vec2()
{
  return FunctionSymbol("vec3", {{ValueType::Vec2}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("NodeReroute"_ustr);
    params.add_input(node, 0, ValueType::Vec2);
    params.set_output(node, 0, ValueType::Vec3);
  });
}

static UString get_combine_color_node_idname(const int tree_type)
{
  switch (tree_type) {
    case NTREE_GEOMETRY:
      return "FunctionNodeCombineColor"_ustr;
    case NTREE_COMPOSIT:
      return "CompositorNodeCombineColor"_ustr;
    case NTREE_SHADER:
      return "ShaderNodeCombineColor"_ustr;
  }
  BLI_assert_unreachable();
  return {};
}

static FunctionSymbol rgb_from_scalars()
{
  return FunctionSymbol("rgb",
                        {{ValueType::Float}, {ValueType::Float}, {ValueType::Float}},
                        [](InsertCallParams &params) {
                          const UString idname = get_combine_color_node_idname(params.tree.type);
                          bNode &node = params.add_node(idname);
                          params.add_input(node, 0);
                          params.add_input(node, 1);
                          params.add_input(node, 2);
                          params.use_node_output(node);
                        });
}

static FunctionSymbol rgba_from_scalars()
{
  return FunctionSymbol(
      "rgba",
      {{ValueType::Float}, {ValueType::Float}, {ValueType::Float}, {ValueType::Float}},
      [](InsertCallParams &params) {
        const UString idname = get_combine_color_node_idname(params.tree.type);
        bNode &node = params.add_node(idname);
        params.use_node_sockets(node);
      });
}

static FunctionSymbol vec3_length()
{
  return FunctionSymbol("length", {{ValueType::Vec3}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("ShaderNodeVectorMath"_ustr);
    node.custom1 = NODE_VECTOR_MATH_LENGTH;
    params.use_node_sockets(node);
  });
}

static UString get_separate_color_node_idname(const int tree_type)
{
  switch (tree_type) {
    case NTREE_GEOMETRY:
      return "FunctionNodeSeparateColor"_ustr;
    case NTREE_COMPOSIT:
      return "CompositorNodeSeparateColor"_ustr;
    case NTREE_SHADER:
      return "ShaderNodeSeparateColor"_ustr;
  }
  BLI_assert_unreachable();
  return {};
}

static FunctionSymbol create_color_member_access(const ValueType type, const int index)
{
  BLI_assert(index >= 0 && index < 4);
  return FunctionSymbol(
      fmt::format(".{}", char("rgba"[index])), {{type, true}}, [index](InsertCallParams &params) {
        const UString node_idname = get_separate_color_node_idname(params.tree.type);
        bNode &node = params.add_node(node_idname);
        params.use_node_inputs(node);
        params.set_output(node, index);
      });
}

static FunctionSymbol int_function(const ValueType type)
{
  return implicit_conversion_node_function("int", type, ValueType::Integer, SOCK_INT);
}

static FunctionSymbol float_function(const ValueType type)
{
  return implicit_conversion_node_function("float", type, ValueType::Float, SOCK_FLOAT);
}

static FunctionSymbol float_less_or_greater_than(const StringRef symbol,
                                                 const NodeMathOperation mode)
{
  return FunctionSymbol(
      symbol, {{ValueType::Float}, {ValueType::Float}}, [mode](InsertCallParams &params) {
        bNode &node = params.add_node("ShaderNodeMath"_ustr);
        node.custom1 = mode;
        params.add_input(node, 0, ValueType::Float);
        params.add_input(node, 1, ValueType::Float);
        params.set_output(node, 0, ValueType::Boolean);
      });
}

/* -------------------------------------------------------------------- */
/** \name Matrix / Rotation operations
 *
 * Official PR accepts Matrix/Rotation sockets but has no ValueType or operators for them.
 * Map all existing Function matrix/rotation nodes so expressions can use the full set.
 * \{ */

static FunctionSymbol matrix_multiply_matrices()
{
  return FunctionSymbol(
      "*", {{ValueType::Matrix}, {ValueType::Matrix}}, [](InsertCallParams &params) {
        bNode &node = params.add_node("FunctionNodeMatrixMultiply"_ustr);
        params.add_input(node, 0, ValueType::Matrix);
        params.add_input(node, 1, ValueType::Matrix);
        params.set_output(node, 0, ValueType::Matrix);
      });
}

/** `m * v` — transform point (includes translation). */
static FunctionSymbol matrix_multiply_vector_as_point()
{
  return FunctionSymbol(
      "*", {{ValueType::Matrix}, {ValueType::Vec3}}, [](InsertCallParams &params) {
        bNode &node = params.add_node("FunctionNodeTransformPoint"_ustr);
        /* Node sockets: Vector (0), Transform (1). Expression order is matrix * vector. */
        params.add_input(node, 1, ValueType::Matrix);
        params.add_input(node, 0, ValueType::Vec3);
        params.set_output(node, 0, ValueType::Vec3);
      });
}

/** `v * m` — same as transform point with reversed operand order. */
static FunctionSymbol vector_multiply_matrix_as_point()
{
  return FunctionSymbol(
      "*", {{ValueType::Vec3}, {ValueType::Matrix}}, [](InsertCallParams &params) {
        bNode &node = params.add_node("FunctionNodeTransformPoint"_ustr);
        params.add_input(node, 0, ValueType::Vec3);
        params.add_input(node, 1, ValueType::Matrix);
        params.set_output(node, 0, ValueType::Vec3);
      });
}

static FunctionSymbol matrix_invert()
{
  return FunctionSymbol("invert", {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeInvertMatrix"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 0, ValueType::Matrix);
  });
}

/** Alias: inv(m) */
static FunctionSymbol matrix_inv_alias()
{
  return FunctionSymbol("inv", {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeInvertMatrix"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 0, ValueType::Matrix);
  });
}

static FunctionSymbol matrix_transpose()
{
  return FunctionSymbol("transpose", {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeTransposeMatrix"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 0, ValueType::Matrix);
  });
}

static FunctionSymbol matrix_determinant()
{
  return FunctionSymbol("determinant", {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeMatrixDeterminant"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 0, ValueType::Float);
  });
}

/** Alias: det(m) */
static FunctionSymbol matrix_det_alias()
{
  return FunctionSymbol("det", {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeMatrixDeterminant"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 0, ValueType::Float);
  });
}

/** transform_point(vector, matrix) */
static FunctionSymbol transform_point_fn()
{
  return FunctionSymbol(
      "transform_point", {{ValueType::Vec3}, {ValueType::Matrix}}, [](InsertCallParams &params) {
        bNode &node = params.add_node("FunctionNodeTransformPoint"_ustr);
        params.add_input(node, 0, ValueType::Vec3);
        params.add_input(node, 1, ValueType::Matrix);
        params.set_output(node, 0, ValueType::Vec3);
      });
}

/** transform_direction(vector, matrix) — rotation/scale only, no translation. */
static FunctionSymbol transform_direction_fn()
{
  return FunctionSymbol("transform_direction",
                        {{ValueType::Vec3}, {ValueType::Matrix}},
                        [](InsertCallParams &params) {
                          bNode &node = params.add_node("FunctionNodeTransformDirection"_ustr);
                          params.add_input(node, 0, ValueType::Vec3);
                          params.add_input(node, 1, ValueType::Matrix);
                          params.set_output(node, 0, ValueType::Vec3);
                        });
}

/** project_point(vector, matrix) */
static FunctionSymbol project_point_fn()
{
  return FunctionSymbol(
      "project_point", {{ValueType::Vec3}, {ValueType::Matrix}}, [](InsertCallParams &params) {
        bNode &node = params.add_node("FunctionNodeProjectPoint"_ustr);
        params.add_input(node, 0, ValueType::Vec3);
        params.add_input(node, 1, ValueType::Matrix);
        params.set_output(node, 0, ValueType::Vec3);
      });
}

/** combine_transform(translation, rotation, scale) → matrix */
static FunctionSymbol combine_transform_fn()
{
  return FunctionSymbol(
      "combine_transform",
      {{ValueType::Vec3}, {ValueType::Rotation}, {ValueType::Vec3}},
      [](InsertCallParams &params) {
        bNode &node = params.add_node("FunctionNodeCombineTransform"_ustr);
        params.add_input(node, 0, ValueType::Vec3);
        params.add_input(node, 1, ValueType::Rotation);
        params.add_input(node, 2, ValueType::Vec3);
        params.set_output(node, 0, ValueType::Matrix);
      });
}

/** translation(matrix) / .translation */
static FunctionSymbol matrix_translation_component(const StringRef name)
{
  return FunctionSymbol(name, {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeSeparateTransform"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 0, ValueType::Vec3);
  });
}

/** rotation(matrix) / .rotation */
static FunctionSymbol matrix_rotation_component(const StringRef name)
{
  return FunctionSymbol(name, {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeSeparateTransform"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 1, ValueType::Rotation);
  });
}

/** scale(matrix) / .scale */
static FunctionSymbol matrix_scale_component(const StringRef name)
{
  return FunctionSymbol(name, {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeSeparateTransform"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 2, ValueType::Vec3);
  });
}

/** invert(rotation) */
static FunctionSymbol rotation_invert()
{
  return FunctionSymbol("invert", {{ValueType::Rotation}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeInvertRotation"_ustr);
    params.add_input(node, 0, ValueType::Rotation);
    params.set_output(node, 0, ValueType::Rotation);
  });
}

static FunctionSymbol rotation_inv_alias()
{
  return FunctionSymbol("inv", {{ValueType::Rotation}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeInvertRotation"_ustr);
    params.add_input(node, 0, ValueType::Rotation);
    params.set_output(node, 0, ValueType::Rotation);
  });
}

/** rotate_vector(vector, rotation) */
static FunctionSymbol rotate_vector_fn()
{
  return FunctionSymbol(
      "rotate_vector", {{ValueType::Vec3}, {ValueType::Rotation}}, [](InsertCallParams &params) {
        bNode &node = params.add_node("FunctionNodeRotateVector"_ustr);
        params.add_input(node, 0, ValueType::Vec3);
        params.add_input(node, 1, ValueType::Rotation);
        params.set_output(node, 0, ValueType::Vec3);
      });
}

/** rotate_rotation(rotation, rotate_by) */
static FunctionSymbol rotate_rotation_fn()
{
  return FunctionSymbol("rotate_rotation",
                        {{ValueType::Rotation}, {ValueType::Rotation}},
                        [](InsertCallParams &params) {
                          bNode &node = params.add_node("FunctionNodeRotateRotation"_ustr);
                          params.add_input(node, 0, ValueType::Rotation);
                          params.add_input(node, 1, ValueType::Rotation);
                          params.set_output(node, 0, ValueType::Rotation);
                        });
}

/** matrix(column0..3 as vec3) via Combine Matrix from 12 floats — use combine of columns.
 *  Simpler API: matrix from transform components already covered; also expose matrix() as
 *  identity-ish via combine_transform with zero translation identity scale later if needed.
 *
 *  combine_matrix takes 16 floats; expose as combine_matrix(c0r0..c3r3). Too heavy for exprs —
 *  prefer separate_matrix element access .m00 style via Separate Matrix + index.
 */

/** Element access m.m00 .. m.m33 via Separate Matrix (column-major, col then row). */
static FunctionSymbol matrix_element_access(const int col, const int row)
{
  BLI_assert(col >= 0 && col < 4 && row >= 0 && row < 4);
  const int output_index = col * 4 + row;
  return FunctionSymbol(
      fmt::format(".m{}{}", col, row),
      {{ValueType::Matrix, true}},
      [output_index](InsertCallParams &params) {
        bNode &node = params.add_node("FunctionNodeSeparateMatrix"_ustr);
        params.add_input(node, 0, ValueType::Matrix);
        params.set_output(node, output_index, ValueType::Float);
      });
}

/** svd_u / svd_s / svd_v helpers — Matrix SVD returns three outputs; expose as named functions. */
static FunctionSymbol matrix_svd_u()
{
  return FunctionSymbol("svd_u", {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeMatrixSVD"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 0, ValueType::Matrix);
  });
}

static FunctionSymbol matrix_svd_s()
{
  return FunctionSymbol("svd_s", {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeMatrixSVD"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 1, ValueType::Vec3);
  });
}

static FunctionSymbol matrix_svd_v()
{
  return FunctionSymbol("svd_v", {{ValueType::Matrix}}, [](InsertCallParams &params) {
    bNode &node = params.add_node("FunctionNodeMatrixSVD"_ustr);
    params.add_input(node, 0, ValueType::Matrix);
    params.set_output(node, 2, ValueType::Matrix);
  });
}

/** \} */

static void init_symbol_table(SymbolTable &symbols)
{
  symbols.add(float_math_function("+", NODE_MATH_ADD, 2));
  symbols.add(float_math_function("-", NODE_MATH_SUBTRACT, 2));
  symbols.add(float_math_function("*", NODE_MATH_MULTIPLY, 2));
  symbols.add(float_math_function("/", NODE_MATH_DIVIDE, 2));

  symbols.add(float_math_function("min", NODE_MATH_MINIMUM, 2));
  symbols.add(float_math_function("max", NODE_MATH_MAXIMUM, 2));
  symbols.add(float_math_function("radians", NODE_MATH_RADIANS, 1));
  symbols.add(float_math_function("degrees", NODE_MATH_DEGREES, 1));
  symbols.add(float_math_function("abs", NODE_MATH_ABSOLUTE, 1));
  symbols.add(float_math_function("floor", NODE_MATH_FLOOR, 1));
  symbols.add(float_math_function("ceil", NODE_MATH_CEIL, 1));
  symbols.add(float_math_function("trunc", NODE_MATH_TRUNC, 1));
  symbols.add(float_math_function("round", NODE_MATH_ROUND, 1));
  symbols.add(float_math_function("sin", NODE_MATH_SINE, 1));
  symbols.add(float_math_function("cos", NODE_MATH_COSINE, 1));
  symbols.add(float_math_function("tan", NODE_MATH_TANGENT, 1));
  symbols.add(float_math_function("asin", NODE_MATH_ARCSINE, 1));
  symbols.add(float_math_function("acos", NODE_MATH_ARCCOSINE, 1));
  symbols.add(float_math_function("atan", NODE_MATH_ARCTANGENT, 1));
  symbols.add(float_math_function("atan2", NODE_MATH_ARCTAN2, 2));
  symbols.add(float_math_function("exp", NODE_MATH_EXPONENT, 1));
  symbols.add(float_math_function("log", NODE_MATH_LOGARITHM, 1));
  symbols.add(float_math_function("sqrt", NODE_MATH_SQRT, 1));
  symbols.add(float_math_function("pow", NODE_MATH_POWER, 2));

  symbols.add(negate_float_function());

  symbols.add(float_less_or_greater_than("<", NODE_MATH_LESS_THAN));
  symbols.add(float_less_or_greater_than(">", NODE_MATH_GREATER_THAN));

  for (const ValueType type : {ValueType::Vec2, ValueType::Vec3}) {
    for (const std::array<ValueType, 2> input_types : Span<std::array<ValueType, 2>>({
             {type, type},
             {type, ValueType::Float},
             {ValueType::Float, type},
         }))
    {
      symbols.add(vector_math_element_wise("+", NODE_VECTOR_MATH_ADD, type, input_types));
      symbols.add(vector_math_element_wise("-", NODE_VECTOR_MATH_SUBTRACT, type, input_types));
      symbols.add(vector_math_element_wise("*", NODE_VECTOR_MATH_MULTIPLY, type, input_types));
      symbols.add(vector_math_element_wise("/", NODE_VECTOR_MATH_DIVIDE, type, input_types));
    }
  }

  symbols.add(vec2_from_scalars());
  symbols.add(vec2_from_vec3());
  symbols.add(vec3_from_scalars());
  symbols.add(vec3_from_vec2());
  symbols.add(rgb_from_scalars());
  symbols.add(rgba_from_scalars());

  symbols.add(vec3_length());

  symbols.add(int_function(ValueType::Float));
  symbols.add(int_function(ValueType::Integer));
  symbols.add(int_function(ValueType::Boolean));

  symbols.add(float_function(ValueType::Integer));
  symbols.add(float_function(ValueType::Boolean));
  symbols.add(float_function(ValueType::Float));

  for (const ValueType type : {ValueType::Vec2, ValueType::Vec3, ValueType::Rgb, ValueType::Rgba})
  {
    symbols.add(vector_member_access(type, 0));
    symbols.add(vector_member_access(type, 1));
    symbols.add(create_color_member_access(type, 0));
    symbols.add(create_color_member_access(type, 1));
  }
  for (const ValueType type : {ValueType::Vec3, ValueType::Rgb, ValueType::Rgba}) {
    symbols.add(vector_member_access(type, 2));
    symbols.add(create_color_member_access(type, 2));
  }
  for (const ValueType type : {ValueType::Rgba}) {
    symbols.add(create_color_member_access(type, 3));
  }

  symbols.add(string_concatenation());

  /* Matrix arithmetic and functions (Geometry/Compositor Function nodes). */
  symbols.add(matrix_multiply_matrices());
  symbols.add(matrix_multiply_vector_as_point());
  symbols.add(vector_multiply_matrix_as_point());
  symbols.add(matrix_invert());
  symbols.add(matrix_inv_alias());
  symbols.add(matrix_transpose());
  symbols.add(matrix_determinant());
  symbols.add(matrix_det_alias());
  symbols.add(transform_point_fn());
  symbols.add(transform_direction_fn());
  symbols.add(project_point_fn());
  symbols.add(combine_transform_fn());
  symbols.add(matrix_translation_component("translation"));
  symbols.add(matrix_rotation_component("rotation"));
  symbols.add(matrix_scale_component("scale"));
  symbols.add(matrix_translation_component(".translation"));
  symbols.add(matrix_rotation_component(".rotation"));
  symbols.add(matrix_scale_component(".scale"));
  for (const int col : IndexRange(4)) {
    for (const int row : IndexRange(4)) {
      symbols.add(matrix_element_access(col, row));
    }
  }
  symbols.add(matrix_svd_u());
  symbols.add(matrix_svd_s());
  symbols.add(matrix_svd_v());

  /* Rotation functions. */
  symbols.add(rotation_invert());
  symbols.add(rotation_inv_alias());
  symbols.add(rotate_vector_fn());
  symbols.add(rotate_rotation_fn());

  for (const ValueType type : {ValueType::Float,
                               ValueType::Vec2,
                               ValueType::Vec3,
                               ValueType::String,
                               ValueType::Rgb,
                               ValueType::Rgba,
                               ValueType::Boolean,
                               ValueType::Integer,
                               ValueType::Matrix,
                               ValueType::Rotation})
  {
    symbols.add(ternary_conditional_operator(type));
  }

  symbols.add_implicit_conversion(ValueType::Integer, ValueType::Float);
  symbols.add_implicit_conversion(ValueType::Boolean, ValueType::Float);
  symbols.add_implicit_conversion(ValueType::Boolean, ValueType::Integer);

  for (const eNodeSocketDatatype type : {SOCK_VECTOR, SOCK_RGBA, SOCK_ROTATION, SOCK_INT_VECTOR}) {
    symbols.add_finalize_value_fn(
        ValueType::Vec2,
        type,
        [](bNodeTree &tree, const Value &value, const eNodeSocketDatatype /*type*/) {
          bNode &node = *bke::node_add_node(nullptr, tree, "ShaderNodeVectorMath"_ustr);
          node.custom1 = NODE_VECTOR_MATH_MULTIPLY;
          bNodeSocket *input0 = static_cast<bNodeSocket *>(node.inputs.first());
          bNodeSocket *input1 = input0->next;
          float *mul_value = input1->default_value_typed<bNodeSocketValueVector>()->value;
          mul_value[0] = 1.0f;
          mul_value[1] = 1.0f;
          mul_value[2] = 0.0f;
          mul_value[3] = 0.0f;
          bke::node_add_link(
              tree, *value.sockets[0].node, *value.sockets[0].socket, node, *input0);
          return NodeAndSocket{&node, static_cast<bNodeSocket *>(node.outputs.first())};
        });
  }

  for (const eNodeSocketDatatype type : {SOCK_FLOAT, SOCK_INT, SOCK_BOOLEAN}) {
    symbols.add_finalize_value_fn(
        ValueType::Vec2,
        type,
        [](bNodeTree &tree, const Value &value, const eNodeSocketDatatype /*type*/) {
          bNode &separate_node = *bke::node_add_node(nullptr, tree, "ShaderNodeSeparateXYZ"_ustr);
          bNode &mix_node = *bke::node_add_node(nullptr, tree, "ShaderNodeMix"_ustr);

          bke::node_add_link(tree,
                             *value.sockets[0].node,
                             *value.sockets[0].socket,
                             separate_node,
                             *static_cast<bNodeSocket *>(separate_node.inputs.first()));
          bNodeSocket *x_socket = static_cast<bNodeSocket *>(separate_node.outputs.first());
          bNodeSocket *y_socket = x_socket->next;

          bNodeSocket *mix_factor = bke::node_find_socket(mix_node, SOCK_IN, "Factor_Float"_ustr);
          bNodeSocket *mix_a = bke::node_find_socket(mix_node, SOCK_IN, "A_Float"_ustr);
          bNodeSocket *mix_b = bke::node_find_socket(mix_node, SOCK_IN, "B_Float"_ustr);

          mix_factor->default_value_typed<bNodeSocketValueFloat>()->value = 0.5f;

          bke::node_add_link(tree, separate_node, *x_socket, mix_node, *mix_a);
          bke::node_add_link(tree, separate_node, *y_socket, mix_node, *mix_b);

          return NodeAndSocket{
              &mix_node,
              bke::node_find_socket(mix_node, SOCK_OUT, "Result_Float"_ustr),
          };
        });
  }
}

static SymbolTable &get_symbol_table()
{
  static SymbolTable symbol_table = []() {
    SymbolTable symbols;
    init_symbol_table(symbols);
    return symbols;
  }();
  return symbol_table;
}

std::shared_ptr<ExpressionNodeGroup> expression_node_to_group(Main *bmain,
                                                              const bNode &node,
                                                              const StringRef tree_idname,
                                                              const Span<StringRef> expressions,
                                                              const Span<int> expr_indices)
{
  BLI_assert(expressions.size() == expr_indices.size());
  auto output = std::make_shared<ExpressionNodeGroup>();

  ResourceScope parse_scope;
  Vector<ast::Expr *> expr_asts;
  for (const int i : expressions.index_range()) {
    ParseResult parse_result = expression::parse(parse_scope, expressions[i]);
    if (const std::string *error = std::get_if<std::string>(&parse_result)) {
      output->error = *error;
      return output;
    }
    expr_asts.append(std::get<ast::Expr *>(parse_result));
  }

  const SymbolTable &symbols = get_symbol_table();

  bNodeTree *tree = bke::node_tree_add_tree(bmain, node.name, tree_idname);

  output->tree = tree;
  BuildOptions options;
  AstToNodeGroupBuilder builder(
      bmain, node, expr_asts, expr_indices, symbols, options, *tree, output->error);
  builder.build();
  if (!output->error.empty()) {
    BKE_id_free(nullptr, &tree->id);
    output->tree = nullptr;
  }
  return output;
}

ExpressionNodeGroup::~ExpressionNodeGroup()
{
  if (this->tree) {
    if (this->tree->id.tag & ID_TAG_NO_MAIN) {
      BKE_id_free(nullptr, &this->tree->id);
    }
  }
}

bNode &InsertCallParams::add_node(const UString idname)
{
  return this->builder.add_node(idname);
}

void InsertCallParams::update_node_sockets(bNode &node)
{
  update_node_declaration_and_sockets(this->builder.r_tree_, node);
  if (node.typeinfo->updatefunc) {
    /* Ensure socket availability is up to date. */
    node.typeinfo->updatefunc(&this->builder.r_tree_, &node);
  }
}

}  // namespace blender::nodes::expression
