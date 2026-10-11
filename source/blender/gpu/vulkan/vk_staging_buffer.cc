/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_staging_buffer.hh"
#include "vk_context.hh"

namespace blender::gpu {

VKStagingBuffer::VKStagingBuffer(const VKBuffer &device_buffer,
                                 Direction direction,
                                 VkDeviceSize device_buffer_offset,
                                 VkDeviceSize region_size)
    : device_buffer_(device_buffer),
      device_buffer_offset_(device_buffer_offset),
      region_size_(region_size == UINT64_MAX ? device_buffer.size_in_bytes() : region_size)
{
  VkBufferUsageFlags usage;
  VmaMemoryUsage memory_usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  VmaAllocationCreateFlags allocation_flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;
  switch (direction) {
    case Direction::HostToDevice:
      usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
      allocation_flags |= VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT;
      break;
    case Direction::DeviceToHost:
      usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
      /* The host reads this buffer. Sequential-write memory preferred on the device is
       * write-combined video memory: filling it is fast, but reading it back was measured at
       * about 20 MB/s. Ask for cached system memory instead. */
      memory_usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
      allocation_flags |= VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT;
      break;
  }

  host_buffer_.create(
      region_size_, usage, memory_usage, allocation_flags, 0.4f, false, "StagingBuffer");
}

void VKStagingBuffer::copy_to_device(VKContext &context)
{
  BLI_assert(host_buffer_.is_allocated() && host_buffer_.is_mapped());
  render_graph::VKCopyBufferNode::CreateInfo copy_buffer = {};
  copy_buffer.src_buffer = host_buffer_.resource();
  copy_buffer.dst_buffer = device_buffer_.resource();
  copy_buffer.region.dstOffset = device_buffer_offset_;
  copy_buffer.region.size = region_size_;

  context.render_graph().add_node(copy_buffer);
}

void VKStagingBuffer::copy_from_device(VKContext &context)
{
  BLI_assert(host_buffer_.is_allocated() && host_buffer_.is_mapped());
  render_graph::VKCopyBufferNode::CreateInfo copy_buffer = {};
  copy_buffer.src_buffer = device_buffer_.resource();
  copy_buffer.dst_buffer = host_buffer_.resource();
  copy_buffer.region.srcOffset = device_buffer_offset_;
  copy_buffer.region.size = region_size_;

  context.render_graph().add_node(copy_buffer);
}

void VKStagingBuffer::free()
{
  host_buffer_.free();
}

}  // namespace blender::gpu
