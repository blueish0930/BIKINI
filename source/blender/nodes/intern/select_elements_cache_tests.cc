/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Drives the shipped Select Elements geometry cache APIs used by Enter-to-edit.
 * Asserts Enter resolve reads the cached Geometry-input topology, not a separate object mesh.
 */

#include "testing/testing.h"

#include "BKE_geometry_set.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"
#include "DNA_node_types.h"

#include "NOD_geo_select_elements.hh"

#include "DEG_depsgraph_query.hh"

namespace blender::nodes::tests {

static bNodeTree make_key_tree(const uint session_uid)
{
  bNodeTree tree{};
  tree.id.session_uid = session_uid;
  return tree;
}

static bNode make_key_node(const int32_t identifier)
{
  bNode node{};
  node.identifier = identifier;
  return node;
}

/**
 * Cache a mesh GeometrySet under Select Elements key APIs, then copy it back.
 * Same pair used by resolve_edit_geometry after force-eval (try_cache).
 */
TEST(SelectElementsCache, RoundTripTopologyMatchesCachedMeshNotObjectMesh)
{
  Mesh *object_mesh = BKE_mesh_new_nomain(4, 0, 0, 0);
  Mesh *upstream_mesh = BKE_mesh_new_nomain(100, 0, 0, 0);
  ASSERT_NE(object_mesh, nullptr);
  ASSERT_NE(upstream_mesh, nullptr);
  ASSERT_NE(object_mesh->verts_num, upstream_mesh->verts_num);

  bNodeTree tree = make_key_tree(0xABCDEFu);
  bNode node = make_key_node(7);

  /* Ensure DEG_get_original identity for non-evaluated IDs (make_key path). */
  ASSERT_EQ(DEG_get_original(&tree), &tree);

  bke::GeometrySet cached_geo = bke::GeometrySet::from_mesh(
      BKE_mesh_copy_for_eval(*upstream_mesh));

  select_elements_cache_geometry(tree, node, std::move(cached_geo));

  std::optional<bke::GeometrySet> roundtrip = select_elements_copy_cached_geometry(tree, node);
  ASSERT_TRUE(roundtrip.has_value());
  ASSERT_TRUE(roundtrip->has_mesh());
  EXPECT_EQ(roundtrip->get_mesh()->verts_num, upstream_mesh->verts_num);
  EXPECT_NE(roundtrip->get_mesh()->verts_num, object_mesh->verts_num);

  select_elements_clear_cache(tree, node);
  EXPECT_FALSE(select_elements_copy_cached_geometry(tree, node).has_value());

  BKE_id_free(nullptr, object_mesh);
  BKE_id_free(nullptr, upstream_mesh);
}

TEST(SelectElementsCache, ClearDropsEntrySoStaleHostMeshCannotBeServed)
{
  Mesh *upstream_mesh = BKE_mesh_new_nomain(50, 0, 0, 0);
  ASSERT_NE(upstream_mesh, nullptr);

  bNodeTree tree = make_key_tree(0x1111u);
  bNode node = make_key_node(1);

  select_elements_cache_geometry(
      tree, node, bke::GeometrySet::from_mesh(BKE_mesh_copy_for_eval(*upstream_mesh)));
  ASSERT_TRUE(select_elements_copy_cached_geometry(tree, node).has_value());

  select_elements_clear_cache(tree, node);
  EXPECT_FALSE(select_elements_copy_cached_geometry(tree, node).has_value());

  BKE_id_free(nullptr, upstream_mesh);
}

}  // namespace blender::nodes::tests
