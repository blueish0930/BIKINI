/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/**
 * Image Process geometry pipeline helpers.
 *
 * Import Points embeds a *locked* GeometryNodeTree:
 *   - No Group Input
 *   - Fixed Group Output: Points (Geometry), Rotation, Size (Vector 2D), ID (Int)
 *   - All other point attributes on the geometry pass through to Point Stamp's Bundle
 *   - After eval, mesh verts / curve points / instances are converted to a point cloud
 *
 * Import Geo embeds a *locked* GeometryNodeTree:
 *   - No Group Input
 *   - Fixed Group Output: Geometry only (mesh / curves / points / realized instances)
 *   - Geometry is passed Import Geo → Rasterize Geometry via the same COM String cache key
 *
 * Point/geo data is passed via a COM String cache key (SOCK_GEOMETRY maps to ResultType::String).
 */

#include <string>

#include "BLI_array.hh"
#include "BLI_function_ref.hh"
#include "BLI_implicit_sharing_ptr.hh"
#include "BLI_map.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"

#include "BKE_geometry_set.hh"

#include "GPU_texture.hh"

namespace blender {
struct Main;
struct bNode;
struct bNodeTree;
}

namespace blender::nodes::image_points {

/** Type marker stored in the COM String result for points sockets. */
constexpr StringRefNull points_result_type_name = "Blender.Image.Points";

/** Attribute names written onto Import Points geometry from the fixed Group Outputs. */
constexpr StringRefNull attr_rotation = "rotation";
constexpr StringRefNull attr_size = "size";
constexpr StringRefNull attr_id = "id";
/** Point Stamp Bundle: coverage mask (1 on valid stamp texels, 0 outside). Not a point attr. */
constexpr StringRefNull attr_alpha = "alpha";

/** IDProperty name on locked nested GeometryNodeTree (int 1). */
constexpr StringRefNull import_points_locked_prop = "import_points_locked";
constexpr StringRefNull import_geo_locked_prop = "import_geo_locked";

/**
 * Datablock name for nested COP Import Points trees.
 * Unique copies become "COP-Import Point.001", etc. Filterable in the node group picker.
 */
constexpr StringRefNull import_points_tree_name_prefix = "COP-Import Point";

/**
 * Datablock name for nested COP Import Geo trees.
 * Unique copies become "COP-Import Geo.001", etc. Filterable in the node group picker.
 */
constexpr StringRefNull import_geo_tree_name_prefix = "COP-Import Geo";

/** Session cache of GeometrySets for the current Image Process cook. */
class PointsGeometryCache {
 private:
  Map<std::string, bke::GeometrySet> geometry_by_key_;

 public:
  void clear();
  std::string store(bke::GeometrySet geometry);
  const bke::GeometrySet *lookup(StringRef key) const;
};

/** Active cache for the ongoing Image Process evaluation (null outside cook). */
PointsGeometryCache *active_cache();
void set_active_cache(PointsGeometryCache *cache);

/**
 * Point Stamp attribute Color maps (premultiplied RGBA, transparent outside stamp).
 * Separate Bundle COM reads these when Bundle is linked (no need for visible multi-outputs).
 */
struct StampAttrMaps {
  int2 size = int2(0);
  /** Premultiplied scene-linear RGBA per pixel, row-major. */
  Map<std::string, Array<float4>> maps;
};

class StampAttrMapsCache {
 private:
  Map<std::string, StampAttrMaps> maps_by_node_key_;

 public:
  void clear();
  void store(StringRef node_key, StampAttrMaps maps);
  const StampAttrMaps *lookup(StringRef node_key) const;
  /** If exactly one Point Stamp published maps this cook, return it (key-agnostic fallback). */
  const StampAttrMaps *lookup_single_if_unique() const;
  int size() const;
  /** Call fn(key, maps) for every entry published this cook. */
  void foreach_maps(FunctionRef<void(StringRef key, const StampAttrMaps &maps)> fn) const;
};

StampAttrMapsCache *active_stamp_attr_cache();
void set_active_stamp_attr_cache(StampAttrMapsCache *cache);

/**
 * GPU Color textures for Rasterize/Stamp attrs (same cook as #StampAttrMapsCache).
 * Separate Bundle prefers this to avoid GPU→CPU download + re-upload (~3–4× Position cost).
 * #sharing_info keeps the texture alive until clear() at end of cook.
 */
struct StampAttrGpuTexture {
  gpu::Texture *texture = nullptr;
  ImplicitSharingPtr<> sharing_info;
  int2 size = int2(0);
};

struct StampAttrGpuMaps {
  int2 size = int2(0);
  Map<std::string, StampAttrGpuTexture> maps;
};

class StampAttrGpuCache {
 private:
  Map<std::string, StampAttrGpuMaps> maps_by_node_key_;

 public:
  void clear();
  void store(StringRef node_key, StampAttrGpuMaps maps);
  const StampAttrGpuMaps *lookup(StringRef node_key) const;
};

StampAttrGpuCache *active_stamp_attr_gpu_cache();
void set_active_stamp_attr_gpu_cache(StampAttrGpuCache *cache);

/** Stable key for a Point Stamp node within a cook. */
std::string stamp_node_cache_key(const bNode &node);

/**
 * Ensure the Import Points node has a locked GeometryNodeTree with fixed Group Output:
 * Points / Rotation / Size / ID. No Group Input. Creates on first call; stores on node.id.
 */
bNodeTree *ensure_import_points_geometry_tree(::blender::Main &bmain, bNode &import_points_node);

/** True if this tree is a locked Import Points nested GeometryNodeTree. */
bool is_import_points_geometry_tree(const bNodeTree &ntree);

/** True if name is a COP Import Points group (prefix "COP-Import Point"). */
bool is_import_points_tree_name(StringRef name);

/**
 * Force fixed interface + strip Group Input. Safe to call from tree update whenever the
 * tree is marked as Import Points locked.
 */
void lock_import_points_geometry_tree(bNodeTree &ntree);

/**
 * Evaluate the nested Geometry Node tree and write rotation/size/id field outputs onto the
 * resulting point geometry as named attributes. All other attributes already on the points
 * geometry are kept for Point Stamp Bundle discovery.
 *
 * Safe for COM cook only. Must not be called from node declare/update (tree mid-mutation + no
 * depsgraph self-object → crash, e.g. Object Info + Store Named Attribute).
 */
bke::GeometrySet evaluate_import_points_tree(const bNodeTree &geometry_tree);

/**
 * Last successfully evaluated Import Points geometry for a nested tree (session-local).
 * Used by Point Stamp declare/update for Bundle attribute signatures without re-evaluating GN.
 */
const bke::GeometrySet *lookup_last_import_points_geometry(const bNodeTree &geometry_tree);

/**
 * After a successful Import Points cook, tag linked Point Stamp nodes so their Bundle/socket
 * declaration rebuilds from the new attributes (Store Named Attribute, radius, …).
 */
void tag_point_stamps_for_import_geometry(bNodeTree &host_tree, const bNode &import_points_node);

/**
 * Ensure the Import Geo node has a locked GeometryNodeTree with a single Group Output:
 * Geometry. No Group Input. Creates on first call; stores on node.id.
 */
bNodeTree *ensure_import_geo_geometry_tree(::blender::Main &bmain, bNode &import_geo_node);

/** True if this tree is a locked Import Geo nested GeometryNodeTree. */
bool is_import_geo_geometry_tree(const bNodeTree &ntree);

/** True if name is a COP Import Geo group (prefix "COP-Import Geo"). */
bool is_import_geo_tree_name(StringRef name);

/** True if this tree is a locked Import Points or Import Geo nested GeometryNodeTree. */
bool is_image_process_nested_geometry_tree(const bNodeTree &ntree);

/**
 * Force fixed Geometry-only interface + strip Group Input. Safe to call from tree update
 * whenever the tree is marked as Import Geo locked.
 */
void lock_import_geo_geometry_tree(bNodeTree &ntree);

/**
 * Evaluate the nested Geometry Node tree and return the Geometry Group Output as-is
 * (instances are realized). Does not convert to a point cloud.
 *
 * Safe for COM cook only. Must not be called from node declare/update.
 */
bke::GeometrySet evaluate_import_geo_tree(const bNodeTree &geometry_tree);

/** Last successfully evaluated Import Geo geometry for a nested tree (session-local). */
const bke::GeometrySet *lookup_last_import_geo_geometry(const bNodeTree &geometry_tree);

/**
 * Destroy cached Import Points/Geo #GeometrySet values.
 * Must run while the GPU context is still alive — Mesh batch caches inside the sets
 * crash if they are freed after #GPU_exit (static destructor / atexit).
 */
void clear_session_geometry_caches();

/** Image Process nodes that nest a locked GeometryNodeTree on node.id. */
bool is_image_import_nested_host(const bNode &node);

/** Ensure the nested GeometryNodeTree for Import Points or Import Geo. */
bNodeTree *ensure_import_nested_geometry_tree(::blender::Main &bmain, bNode &node);

}  // namespace blender::nodes::image_points
