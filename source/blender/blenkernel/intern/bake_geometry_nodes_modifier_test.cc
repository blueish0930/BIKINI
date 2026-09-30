/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "testing/testing.h"

#include "BKE_bake_geometry_nodes_modifier.hh"

namespace blender::bke::bake::tests {

static std::unique_ptr<FrameCache> make_frame_cache(const int frame,
                                                    const int64_t payload_bytes = 0)
{
  auto frame_cache = std::make_unique<FrameCache>();
  frame_cache->frame = SubFrame{frame};
  if (payload_bytes > 0) {
    Map<int, BakeValues::Item> items;
    items.add_new(0,
                  BakeValues::Item{
                      SocketValueVariant(std::string(payload_bytes, 'x')), std::nullopt});
    frame_cache->values = BakeValues(std::move(items));
  }
  return frame_cache;
}

TEST(node_bake_cache, TrimFrames)
{
  NodeBakeCache cache;
  for (const int frame : IndexRange(1, 5)) {
    cache.frames.append(make_frame_cache(frame));
  }

  cache.trim_frames(3);

  ASSERT_EQ(cache.frames.size(), 3);
  EXPECT_EQ(cache.frames[0]->frame.frame(), 3);
  EXPECT_EQ(cache.frames[1]->frame.frame(), 4);
  EXPECT_EQ(cache.frames[2]->frame.frame(), 5);
}

TEST(node_bake_cache, TrimFramesKeepsAtLeastOneFrame)
{
  NodeBakeCache cache;
  for (const int frame : IndexRange(1, 3)) {
    cache.frames.append(make_frame_cache(frame));
  }

  cache.trim_frames(0);

  ASSERT_EQ(cache.frames.size(), 1);
  EXPECT_EQ(cache.frames[0]->frame.frame(), 3);
}

TEST(node_bake_cache, TrimFramesDoesNotGrowCache)
{
  NodeBakeCache cache;
  cache.frames.append(make_frame_cache(10));
  cache.frames.append(make_frame_cache(11));

  cache.trim_frames(5);

  ASSERT_EQ(cache.frames.size(), 2);
  EXPECT_EQ(cache.frames[0]->frame.frame(), 10);
  EXPECT_EQ(cache.frames[1]->frame.frame(), 11);
}

TEST(node_bake_cache, TrimMemory)
{
  NodeBakeCache cache;
  cache.frames.append(make_frame_cache(1, 1024));
  cache.frames.append(make_frame_cache(2, 1024));
  cache.frames.append(make_frame_cache(3, 1024));

  NodeBakeCache one_frame_cache;
  one_frame_cache.frames.append(make_frame_cache(0, 1024));
  const int64_t one_frame_memory = one_frame_cache.memory_usage();

  cache.trim_memory(one_frame_memory * 2);

  ASSERT_EQ(cache.frames.size(), 2);
  EXPECT_EQ(cache.frames[0]->frame.frame(), 2);
  EXPECT_EQ(cache.frames[1]->frame.frame(), 3);
  EXPECT_LE(cache.memory_usage(), one_frame_memory * 2);
}

TEST(node_bake_cache, TrimMemoryKeepsOversizedNewestFrame)
{
  NodeBakeCache cache;
  cache.frames.append(make_frame_cache(1, 1024));
  cache.frames.append(make_frame_cache(2, 1024));

  cache.trim_memory(1);

  ASSERT_EQ(cache.frames.size(), 1);
  EXPECT_EQ(cache.frames[0]->frame.frame(), 2);
}

}  // namespace blender::bke::bake::tests
