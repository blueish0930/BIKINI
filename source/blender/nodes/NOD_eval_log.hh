/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 *
 * Many geometry nodes related UI features need access to data produced during evaluation. Not only
 * is the final output required but also the intermediate results. Those features include attribute
 * search, node warnings, socket inspection and the viewer node.
 *
 * This file provides the system for logging data during evaluation and accessing the data after
 * evaluation. At the root of the logging data is a #NodesEvalLog which is created by the code that
 * invokes Geometry Nodes (e.g. the Geometry Nodes modifier).
 *
 * The system makes a distinction between "loggers" and the "log":
 * - Logger (#NodeTreeLogger): Is used during geometry nodes evaluation. Each thread logs data
 *   independently to avoid communication between threads. Logging should generally be fast.
 *   Generally, the logged data is just dumped into simple containers. Any processing of the data
 *   happens later if necessary. This is important for performance, because in practice, most of
 *   the logged data is never used again. So any processing of the data is likely to be a waste of
 *   resources.
 * - Log (#NodeTreeLog, #NodeLog): Those are used when accessing logged data in UI code. They
 *   contain and cache preprocessed data produced during logging. The log combines data from all
 *   thread-local loggers to provide simple access. Importantly, the (preprocessed) log is only
 *   created when it is actually used by UI code.
 */

#pragma once

#include <chrono>
#include <memory>

#include "BLI_cache_mutex.hh"
#include "BLI_compute_context.hh"
#include "BLI_function_ref.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_enum_flags.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_generic_pointer.hh"
#include "BLI_linear_allocator_chunked_list.hh"
#include "BLI_ustring.hh"

#include "BKE_compute_context_cache_fwd.hh"
#include "BKE_geometry_set.hh"
#include "BKE_node.hh"
#include "BKE_node_socket_value.hh"
#include "BKE_node_tree_zones.hh"
#include "BKE_volume_grid_fwd.hh"

#include "NOD_geometry_nodes_closure_location.hh"
#include "NOD_geometry_nodes_list_fwd.hh"
#include "NOD_warning.hh"

#include "DNA_node_types.h"

namespace blender {

struct SpaceNode;
struct NodesModifierData;
struct Report;
struct ImBuf;
struct Object;
struct wmWindowManager;

namespace fn {
class GField;
}

namespace nodes::eval_log {

using fn::GField;

enum class NamedAttributeUsage {
  None = 0,
  Read = 1 << 0,
  Write = 1 << 1,
  Remove = 1 << 2,
};
ENUM_OPERATORS(NamedAttributeUsage);

/**
 * Values of different types are logged differently. This is necessary because some types are so
 * simple that we can log them entirely (e.g. `int`), while we don't want to log all intermediate
 * geometries in their entirety.
 *
 * #ValueLog is a base class for the different ways we log values.
 */
class ValueLog {
 public:
  virtual ~ValueLog() = default;
};

/**
 * Simplest logger. It just stores a copy of the entire value. This is used for most simple types
 * like `int`.
 */
class GenericValueLog : public ValueLog {
 public:
  /**
   * This is owning the value, but not the memory.
   */
  GMutablePointer value;

  GenericValueLog(const GMutablePointer value) : value(value) {}

  ~GenericValueLog() override;
};

/**
 * Fields are not logged entirely, because they might contain arbitrarily large data (e.g.
 * geometries that are sampled). Instead, only the data needed for UI features is logged.
 */
class FieldInfoLog : public ValueLog {
 public:
  const CPPType &type;
  Vector<std::string> input_tooltips;

  FieldInfoLog(const GField &field);
};

struct StringLog : public ValueLog {
  StringRef value;
  bool truncated;
  StringLog(StringRef string, LinearAllocator<> &allocator);
};

struct GeometryAttributeInfo {
  std::string name;
  /** Can be empty when #name does not actually exist on a geometry yet. */
  std::optional<bke::AttrDomain> domain;
  std::optional<bke::AttrType> data_type;
};

struct VolumeGridInfo {
  std::string name;
  VolumeGridType grid_type;
};

/**
 * Geometries are not logged entirely, because that would result in a lot of time and memory
 * overhead. Instead, only the data needed for UI features is logged.
 */
class GeometryInfoLog : public ValueLog {
 public:
  std::string name;
  Vector<GeometryAttributeInfo> attributes;
  Vector<bke::GeometryComponent::Type> component_types;

  struct MeshInfo {
    int verts_num, edges_num, faces_num;
  };
  struct CurveInfo {
    int points_num;
    int splines_num;
  };
  struct PointCloudInfo {
    int points_num;
  };
  struct GreasePencilInfo {
    int layers_num;
    Vector<std::string> layer_names;
  };
  struct InstancesInfo {
    int instances_num;
  };
  struct EditDataInfo {
    bool has_deformed_positions = false;
    bool has_deform_matrices = false;
    int gizmo_transforms_num = 0;
  };
  struct VolumeInfo {
    Vector<VolumeGridInfo> grids;
  };

  std::optional<MeshInfo> mesh_info;
  std::optional<CurveInfo> curve_info;
  std::optional<PointCloudInfo> pointcloud_info;
  std::optional<GreasePencilInfo> grease_pencil_info;
  std::optional<InstancesInfo> instances_info;
  std::optional<EditDataInfo> edit_data_info;
  std::optional<VolumeInfo> volume_info;

  GeometryInfoLog(const bke::GeometrySet &geometry_set);
};

class GridInfoLog : public ValueLog {
 public:
  bool is_empty = false;

  GridInfoLog(const bke::GVolumeGrid &grid);
};

class BundleValueLog : public ValueLog {
 public:
  struct Item {
    UString key;
    std::variant<const bke::bNodeSocketType *, StringRefNull> type;
  };

  Vector<Item> items;

  BundleValueLog(Vector<Item> items);
};

class ClosureValueLog : public ValueLog {
 public:
  struct Item {
    std::string key;
    const bke::bNodeSocketType *type;
  };

  /**
   * Similar to #ClosureSourceLocation but does not keep pointer references to potentially
   * temporary data.
   */
  struct Source {
    uint32_t orig_node_tree_session_uid;
    int closure_output_node_id;
    ComputeContextHash compute_context_hash;
  };

  Vector<Item> inputs;
  Vector<Item> outputs;
  std::optional<Source> source;
  std::shared_ptr<ClosureEvalLog> eval_log;

  ClosureValueLog(Vector<Item> inputs,
                  Vector<Item> outputs,
                  const std::optional<ClosureSourceLocation> &source_location,
                  std::shared_ptr<ClosureEvalLog> eval_log);
};

class ListInfoLog : public ValueLog {
 public:
  int64_t size;

  ListInfoLog(const GListPtr &list);
};

/**
 * Data logged by a viewer node when it is executed.
 */
class ViewerNodeLog {
  mutable CacheMutex main_geometry_cache_mutex_;
  mutable std::optional<bke::GeometrySet> main_geometry_cache_;

 public:
  struct Item {
    int identifier;
    std::string name;
    bke::SocketValueVariant value;
  };

  struct ItemIdentifierGetter {
    int operator()(const Item &item) const
    {
      return item.identifier;
    }
  };

  CustomIDVectorSet<Item, ItemIdentifierGetter> items;

  const bke::GeometrySet *main_geometry() const;
};

/**
 * Data logged by a Guide Geometry output node. This is viewport overlay only: it must never feed
 * the Spreadsheet or the rendered modifier output.
 *
 * Stores owned draw buffers instead of a #GeometrySet. The evaluation result's mesh/attribute
 * arrays are implicitly shared; keeping a GeometrySet here would share those #ImplicitSharingPtr
 * values. When the modifier result is released, decrementing the log's copy crashes on a
 * dangling sharing_info.
 */
class GuideGeometryNodeLog {
 public:
  /* Used by overlay to reject logs that were never snapshotted (or were freed). */
  static constexpr uint32_t copied_magic = 0x47554446; /* 'GUDF' overlay v5 */
  static constexpr int max_elems = 2'000'000;
  uint32_t magic = copied_magic;
  uint32_t seal = 0;
  Vector<float3> mesh_positions;
  Vector<int2> mesh_edges;
  Vector<int3> mesh_tris;
  Vector<float3> curve_positions;
  Vector<int> curve_offsets;
  Vector<uint8_t> curve_cyclic;
  Vector<float3> point_positions;
  float4 color = float4(1.0f, 0.55f, 0.15f, 0.85f);
  int8_t display_mode = 0;
  bool xray = true;

  uint32_t compute_seal() const
  {
    auto mix = [](uint32_t a, const uint64_t b) -> uint32_t {
      return a ^ uint32_t(b) ^ uint32_t(b >> 32);
    };
    uint32_t s = copied_magic;
    s = mix(s, mesh_positions.size());
    s = mix(s, mesh_edges.size());
    s = mix(s, mesh_tris.size());
    s = mix(s, curve_positions.size());
    s = mix(s, curve_offsets.size());
    s = mix(s, curve_cyclic.size());
    s = mix(s, point_positions.size());
    return s;
  }

  void mark_ready()
  {
    magic = copied_magic;
    seal = compute_seal();
  }

  bool is_ready() const
  {
    if (magic != copied_magic) {
      return false;
    }
    if (int(mesh_positions.size()) > max_elems || int(mesh_edges.size()) > max_elems ||
        int(mesh_tris.size()) > max_elems || int(curve_positions.size()) > max_elems ||
        int(point_positions.size()) > max_elems)
    {
      return false;
    }
    if (curve_offsets.size() > 1 &&
        (curve_offsets.first() != 0 || curve_offsets.last() > int(curve_positions.size())))
    {
      return false;
    }
    return seal == compute_seal();
  }

  bool is_empty() const
  {
    if (!is_ready()) {
      return true;
    }
    return mesh_positions.is_empty() && curve_positions.is_empty() && point_positions.is_empty();
  }
};

/* Compositor image result. */
class ImageInfoLog : public ValueLog {
 public:
  /* Stores compositor::Domain information. */
  const int2 data_size;
  const int2 display_size;
  const int2 data_offset;
  const float3x3 transformation;

  /* Stores compositor::RealizationOptions information in a textual representation. */
  const StringRefNull interpolation;
  const StringRefNull extension_x;
  const StringRefNull extension_y;

  /* Stores compositor::Result.precision in a textual representation. */
  const StringRefNull precision;

  ImageInfoLog(int2 data_size,
               int2 display_size,
               int2 data_offset,
               float3x3 transformation,
               StringRefNull interpolation,
               StringRefNull extension_x,
               StringRefNull extension_y,
               StringRefNull precision);
};

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

/**
 * Logs all data for a specific geometry node tree in a specific context. When the same node group
 * is used in multiple times each instantiation will have a separate logger.
 */
class NodeTreeLogger {
 public:
  std::optional<ComputeContextHash> parent_hash;
  std::optional<int32_t> parent_node_id;
  Vector<ComputeContextHash> children_hashes;
  /**
   * The #ID.session_uid of the tree that this logger is for. It's an optional value because under
   * some circumstances it's not possible to know this exactly currently (e.g. for closures).
   */
  std::optional<uint32_t> tree_orig_session_uid;
  /** The time spend in the compute context that this logger corresponds to. */
  std::chrono::nanoseconds execution_time{};

  LinearAllocator<> *allocator = nullptr;

  struct WarningWithNode {
    int32_t node_id;
    NodeWarning warning;
  };
  struct SocketValueLog {
    int32_t node_id;
    int socket_index;
    destruct_ptr<ValueLog> value;
  };
  struct NodeExecutionTime {
    int32_t node_id;
    TimePoint start;
    TimePoint end;
  };
  struct ViewerNodeLogWithNode {
    int32_t node_id;
    destruct_ptr<ViewerNodeLog> viewer_log;
  };
  struct GuideGeometryNodeLogWithNode {
    int32_t node_id;
    /* Heap-owned. Do not put this in a LinearAllocator ChunkedList: the log
     * contains Vector<> heap buffers, and destroying those through allocator
     * segments during eval_log replacement has been crashing (MEM_freeN). */
    std::unique_ptr<GuideGeometryNodeLog> guide_log;
  };
  struct AttributeUsageWithNode {
    int32_t node_id;
    StringRefNull attribute_name;
    NamedAttributeUsage usage;
  };
  struct DebugMessage {
    int32_t node_id;
    StringRefNull message;
  };
  struct EvaluatedGizmoNode {
    int32_t node_id;
  };
  struct NodeImagePreview {
    int32_t node_id;
    /** An image preview of the node. Owned by the logger and should be freed when destructed. */
    ImBuf *image_preview = nullptr;
  };

  linear_allocator::ChunkedList<WarningWithNode> node_warnings;
  linear_allocator::ChunkedList<SocketValueLog, 16> input_socket_values;
  linear_allocator::ChunkedList<SocketValueLog, 16> output_socket_values;
  linear_allocator::ChunkedList<NodeExecutionTime, 16> node_execution_times;
  linear_allocator::ChunkedList<NodeImagePreview, 16> node_image_previews;
  linear_allocator::ChunkedList<ViewerNodeLogWithNode> viewer_node_logs;
  Vector<GuideGeometryNodeLogWithNode> guide_geometry_node_logs;
  linear_allocator::ChunkedList<AttributeUsageWithNode> used_named_attributes;
  linear_allocator::ChunkedList<DebugMessage> debug_messages;
  /** Keeps track of which gizmo nodes have been tracked by this evaluation. */
  linear_allocator::ChunkedList<EvaluatedGizmoNode> evaluated_gizmo_nodes;
  /** Appended at the end so older TUs keep the original field offsets. */
  linear_allocator::ChunkedList<int32_t, 16> cache_hit_node_ids;
  /**
   * First-complete-eval duration per node. Applied after summing live samples so Repeat/Group
   * overlays keep ~900ms instead of a later execute-only 50–60ms.
   */
  linear_allocator::ChunkedList<NodeExecutionTime, 16> frozen_overlay_times;

  NodeTreeLogger();
  ~NodeTreeLogger();

  void log_value(const bNode &node, const bNodeSocket &socket, GPointer value);
};

/**
 * Contains data that has been logged for a specific node in a context. So when the node is in a
 * node group that is used multiple times, there will be a different #NodeLog for every
 * instance.
 *
 * By default, not all of the info below is valid. A #NodeTreeLog::ensure_* method has to be called
 * first.
 */
class NodeLog {
 public:
  /** Warnings generated for that node. */
  VectorSet<NodeWarning> warnings;
  /** Time spent in this node. */
  std::chrono::nanoseconds execution_time{0};
  /** Maps from socket indices to their values. */
  Map<int, ValueLog *> input_values_;
  Map<int, ValueLog *> output_values_;
  /** Maps from attribute name to their usage flags. */
  Map<StringRefNull, NamedAttributeUsage> used_named_attributes;
  /** Messages that are used for debugging purposes during development. */
  Vector<StringRefNull> debug_messages;
  /** An image preview of the node. Owned by the log and should be freed when destructed. */
  ImBuf *image_preview = nullptr;
  /** True when this node's displayed time is from a cache hit. Keep last for ABI. */
  bool from_cache = false;

  NodeLog();
  ~NodeLog();
};

class NodesEvalLog;

/**
 * Contains data that has been logged for a specific node group in a context. If the same node
 * group is used multiple times, there will be a different #NodeTreeLog for every instance.
 *
 * This contains lazily evaluated data. Call the corresponding `ensure_*` methods before accessing
 * data.
 */
class NodeTreeLog {
 private:
  LinearAllocator<> allocator_;
  NodesEvalLog *root_log_;
  Vector<NodeTreeLogger *> tree_loggers_;
  VectorSet<ComputeContextHash> children_hashes_;
  bool reduced_node_warnings_ = false;
  bool reduced_execution_times_ = false;
  bool reduced_socket_values_ = false;
  bool reduced_viewer_node_logs_ = false;
  bool reduced_guide_geometry_node_logs_ = false;
  bool reduced_existing_attributes_ = false;
  bool reduced_used_named_attributes_ = false;
  bool reduced_debug_messages_ = false;
  bool reduced_evaluated_gizmo_nodes_ = false;
  bool reduced_layer_names_ = false;
  bool reduced_node_image_previews_ = false;

 public:
  Map<int32_t, destruct_ptr<NodeLog>> nodes;
  Map<int32_t, ViewerNodeLog *, 0> viewer_node_logs;
  Map<int32_t, GuideGeometryNodeLog *, 0> guide_geometry_node_logs;
  VectorSet<NodeWarning> all_warnings;
  std::chrono::nanoseconds execution_time{0};
  Vector<const GeometryAttributeInfo *> existing_attributes;
  Map<StringRefNull, NamedAttributeUsage> used_named_attributes;
  Set<int> evaluated_gizmo_nodes;
  Vector<std::string> all_layer_names;

  NodeTreeLog(NodesEvalLog *root_log, Vector<NodeTreeLogger *> tree_loggers);
  ~NodeTreeLog();

  /**
   * Propagate node warnings. This needs access to the node group pointers, because propagation
   * settings are stored on the nodes. However, the log can only store weak pointers (in the form
   * of e.g. session ids) to original data to avoid dangling pointers.
   */
  void ensure_node_warnings(const NodesModifierData &nmd);
  void ensure_node_warnings(const Main &bmain);
  void ensure_node_warnings(const Map<uint32_t, const bNodeTree *> &orig_tree_by_session_uid);

  void ensure_execution_times();
  void ensure_socket_values();
  void ensure_viewer_node_logs();
  void ensure_guide_geometry_node_logs();
  void ensure_existing_attributes();
  void ensure_used_named_attributes();
  void ensure_debug_messages();
  void ensure_evaluated_gizmo_nodes();
  void ensure_layer_names();
  void ensure_node_image_previews();

  NodeLog *find_node_log(int32_t identifier) const;
  NodeLog &lookup_or_add_node_log(const int32_t identifier);

  /** Group or zone node that entered this compute context, when logging recorded one. */
  std::optional<int32_t> caller_node_id() const;
  /** Original node-tree session uid this log belongs to, when known. */
  std::optional<uint32_t> tree_session_uid() const;
  void foreach_child_context_hash(FunctionRef<void(ComputeContextHash)> fn) const;

  ValueLog *find_socket_value_log(const bNodeSocket &query_socket);
  [[nodiscard]] bool try_convert_primitive_socket_value(const GenericValueLog &value_log,
                                                        const CPPType &dst_type,
                                                        void *dst);

  template<typename T>
  std::optional<T> find_primitive_socket_value(const bNodeSocket &query_socket)
  {
    if (auto *value_log = dynamic_cast<GenericValueLog *>(
            this->find_socket_value_log(query_socket)))
    {
      T value;
      if (this->try_convert_primitive_socket_value(*value_log, CPPType::get<T>(), &value)) {
        return value;
      }
    }
    return std::nullopt;
  }
};

class ContextualNodeTreeLogs {
 private:
  Map<const bke::bNodeTreeZone *, NodeTreeLog *> tree_logs_by_zone_;

 public:
  ContextualNodeTreeLogs(Map<const bke::bNodeTreeZone *, NodeTreeLog *> tree_logs_by_zone = {});

  /**
   * Get a tree log for the given zone/node/socket if available.
   */
  NodeTreeLog *get_main_tree_log(const bke::bNodeTreeZone *zone) const;
  NodeTreeLog *get_main_tree_log(const bNode &node) const;
  NodeTreeLog *get_main_tree_log(const bNodeSocket &socket) const;

  /**
   * Runs a callback for each tree log that may be returned above.
   */
  void foreach_tree_log(FunctionRef<void(NodeTreeLog &)> callback) const;
};

/**
 * This contains all the loggers that are used during evaluation as well as the preprocessed logs
 * that are used by UI code.
 */
class NodesEvalLog {
 private:
  /** Data that is stored for each thread. */
  struct LocalData {
    /** Each thread has its own allocator. */
    LinearAllocator<> allocator;
    /**
     * Store a separate #NodeTreeLogger for each instance of the corresponding node group (e.g.
     * when the same node group is used multiple times).
     */
    Map<ComputeContextHash, destruct_ptr<NodeTreeLogger>> tree_logger_by_context;
  };

  /** Container for all thread-local data. */
  threading::EnumerableThreadSpecific<LocalData> data_per_thread_;
  /**
   * A #NodeTreeLog for every compute context. Those are created lazily when requested by UI code.
   */
  Map<ComputeContextHash, std::unique_ptr<NodeTreeLog>> tree_logs_;

 public:
  NodesEvalLog();
  ~NodesEvalLog();

  /**
   * Get a thread-local logger for the current node tree.
   */
  NodeTreeLogger &get_local_tree_logger(const ComputeContext &compute_context);

  /**
   * Get a log a specific node tree instance.
   */
  NodeTreeLog &get_tree_log(const ComputeContextHash &compute_context_hash);

  /**
   * Utility accessor to logged data.
   */
  static Map<const bke::bNodeTreeZone *, ComputeContextHash>
  get_context_hash_by_zone_for_node_editor(const SpaceNode &snode,
                                           bke::ComputeContextCache &compute_context_cache);

  static ContextualNodeTreeLogs get_contextual_tree_logs(const SpaceNode &snode);
  /** Evaluation log for the node editor's current geometry/compositor/image context. */
  static NodesEvalLog *from_space_node(const SpaceNode &snode);
  static const ViewerNodeLog *find_viewer_node_log_for_path(const ViewerPath &viewer_path);

  /** Calls \a fn for every guide-geometry log produced by this evaluation. */
  void foreach_guide_geometry_node_log(FunctionRef<void(const GuideGeometryNodeLog &log)> fn);

  /**
   * Copy execution times for nodes that did not run in this evaluation (cache skip / lazy unused)
   * from the previous log, and mark them as cache hits so the overlay keeps the frozen time.
   */
  void inherit_missing_execution_times(NodesEvalLog &src);

  /** Sum of already-logged cook samples for \a node_id in \a context, across all threads. */
  int64_t logged_node_time_ns(const ComputeContextHash &context_hash, int32_t node_id);
};

/**
 * Calls \a fn for every Guide Geometry log that should currently be drawn on \a object_eval.
 * Shown while the object is selected (or active), independent of the node editor.
 */
void foreach_visible_guide_geometry(const Object &object_eval,
                                    const wmWindowManager &wm,
                                    FunctionRef<void(const GuideGeometryNodeLog &log)> fn);

}  // namespace nodes::eval_log

}  // namespace blender
