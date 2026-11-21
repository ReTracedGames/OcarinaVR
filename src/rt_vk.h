// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// The Vulkan functions the core calls itself, loaded from the host's device (VK_NO_PROTOTYPES), and small helpers.
#pragma once
#include "rt_api.h"

#include <cstddef>
#include <cstdint>

namespace rt::vk {

#define RT_VK_DEVICE_FUNCS(X)                                                                                         \
    X(vkCreateBuffer) X(vkDestroyBuffer) X(vkGetBufferMemoryRequirements) X(vkAllocateMemory) X(vkFreeMemory)          \
    X(vkBindBufferMemory) X(vkMapMemory) X(vkGetBufferDeviceAddress) X(vkCreateCommandPool) X(vkAllocateCommandBuffers) \
    X(vkResetCommandPool) X(vkBeginCommandBuffer) X(vkEndCommandBuffer) X(vkCreateFence) X(vkWaitForFences)            \
    X(vkResetFences) X(vkGetFenceStatus) X(vkCmdPipelineBarrier2) X(vkCmdCopyBuffer) X(vkCmdFillBuffer)                \
    X(vkCreateAccelerationStructureKHR) X(vkDestroyAccelerationStructureKHR) X(vkGetAccelerationStructureBuildSizesKHR) \
    X(vkCmdBuildAccelerationStructuresKHR) X(vkGetAccelerationStructureDeviceAddressKHR) X(vkCreateShaderModule)        \
    X(vkCreateDescriptorSetLayout) X(vkCreatePipelineLayout) X(vkCreateComputePipelines) X(vkCreateDescriptorPool)      \
    X(vkAllocateDescriptorSets) X(vkUpdateDescriptorSets) X(vkCmdBindPipeline) X(vkCmdBindDescriptorSets)              \
    X(vkCmdPushConstants) X(vkCmdDispatch) X(vkCreateImageView) X(vkDestroyImageView) X(vkCreateSampler)                \
    X(vkCreateQueryPool) X(vkCmdResetQueryPool) X(vkCmdWriteTimestamp2) X(vkGetQueryPoolResults)                    \
    X(vkCmdUpdateBuffer) X(vkCmdPushDescriptorSetKHR) X(vkCreateImage) X(vkGetImageMemoryRequirements) X(vkBindImageMemory) X(vkCmdCopyImageToBuffer) X(vkCmdClearColorImage) X(vkGetBufferOpaqueCaptureAddress) X(vkGetDeviceMemoryOpaqueCaptureAddress) X(vkCmdCopyImage) X(vkCmdBlitImage) X(vkDestroyImage)

#define RT_VK_DECLARE(f) extern PFN_##f f;
RT_VK_DEVICE_FUNCS(RT_VK_DECLARE)
#undef RT_VK_DECLARE

extern VkDevice device;
extern VkPhysicalDevice physical;
extern VkQueue queue;
extern uint32_t queue_family;
extern float timestamp_period;   // (ns per GPU timestamp tick)

// Loads every function; false (and the name of the first missing one in *missing) when one is absent.
bool load(const RtDevice* d, const char** missing);
// The same without the ray functions (the VR mode asks for no ray extension: those pointers stay null).
bool load_plain(const RtDevice* d, const char** missing);

// A buffer with its own memory. host: mappable and coherent (mapped stays valid); else device-local.
struct Buffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkDeviceSize size = 0;
    VkDeviceAddress address = 0;
    void* mapped = nullptr;
};
// read_back: a host buffer the processor reads much of (pictures copied back): in memory the processor reads fast
// (cached), where the driver has some; the plain host memory of a graphics card can be a hundred times slower to read.
bool make_buffer(Buffer& b, VkDeviceSize size, VkBufferUsageFlags usage, bool host, bool read_back = false);
// The device feature bufferDeviceAddressCaptureReplay is there (the host enables every feature its query returns).
extern bool capture_replay;
// A device-local buffer whose device address should be the same at every run (a program holds it as a constant).
// want: the two values a previous run got (buffer, memory), or zeros; got: this run's, to save for the next. When the
// address asked for cannot be had, or the feature is missing, the buffer is made anyway, wherever the driver puts it.
bool make_buffer_fixed(Buffer& b, VkDeviceSize size, VkBufferUsageFlags usage, const uint64_t want[2], uint64_t got[2]);
// A compute pass of ours: one set of push descriptors, one push-constant block.
bool make_compute(const uint32_t* code, size_t bytes, const VkDescriptorSetLayoutBinding* b, uint32_t nb, uint32_t push_bytes,
                  VkDescriptorSetLayout& dsl, VkPipelineLayout& layout, VkPipeline& pipe);

// A 2D image with its own memory and one view, for our compute passes (storage, read and written in the GENERAL layout).
struct Image {
    VkImage image = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkImageView view = VK_NULL_HANDLE;
    uint32_t w = 0, h = 0;
    bool fresh = false;   // (made, not yet taken to the GENERAL layout)
};
bool make_image(Image& im, uint32_t w, uint32_t h, VkFormat format,
                VkImageUsageFlags usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
// The same, in three dimensions (our cloud noise).
bool make_image3d(Image& im, uint32_t w, uint32_t h, uint32_t d, VkFormat format,
                  VkImageUsageFlags usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
// Takes a fresh image to the GENERAL layout (its content is undefined before).
void image_ready(VkCommandBuffer cmd, Image& im);

// One memory barrier, everything before against everything after (our passes are few and short).
void full_barrier(VkCommandBuffer cmd);

} // namespace rt::vk
