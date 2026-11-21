// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
#include "rt_vk.h"
#include "rt_diag.h"

#include <cstring>

namespace rt::vk {

#define RT_VK_DEFINE(f) PFN_##f f = nullptr;
RT_VK_DEVICE_FUNCS(RT_VK_DEFINE)
#undef RT_VK_DEFINE

VkDevice device = VK_NULL_HANDLE;
VkPhysicalDevice physical = VK_NULL_HANDLE;
VkQueue queue = VK_NULL_HANDLE;
uint32_t queue_family = 0;
float timestamp_period = 1.0f;
bool capture_replay = false;

namespace {
VkPhysicalDeviceMemoryProperties mem_props{};

uint32_t memory_type(uint32_t bits, VkMemoryPropertyFlags want) {
    for (uint32_t i = 0; i < mem_props.memoryTypeCount; ++i)
        if ((bits & (1u << i)) && (mem_props.memoryTypes[i].propertyFlags & want) == want) return i;
    return UINT32_MAX;
}
} // namespace

namespace {
bool g_plain = false;
bool ray_name(const char* n) { return std::strstr(n, "AccelerationStructure") != nullptr; }
} // namespace

bool load_plain(const RtDevice* d, const char** missing) {
    g_plain = true;
    const bool ok = load(d, missing);
    g_plain = false;
    return ok;
}

bool load(const RtDevice* d, const char** missing) {
    device = d->device;
    physical = d->physical;
    queue = d->queue;
    queue_family = d->queue_family;
#define RT_VK_LOAD(f)                                                                                                 \
    f = (PFN_##f)d->get_device_proc(device, #f);                                                                      \
    if (!f && !(g_plain && ray_name(#f))) {                                                                                                         \
        *missing = #f;                                                                                                \
        return false;                                                                                                 \
    }
    RT_VK_DEVICE_FUNCS(RT_VK_LOAD)
#undef RT_VK_LOAD
    auto props = (PFN_vkGetPhysicalDeviceMemoryProperties)d->get_instance_proc(d->instance, "vkGetPhysicalDeviceMemoryProperties");
    if (!props) {
        *missing = "vkGetPhysicalDeviceMemoryProperties";
        return false;
    }
    props(physical, &mem_props);
    if (auto dp = (PFN_vkGetPhysicalDeviceProperties)d->get_instance_proc(d->instance, "vkGetPhysicalDeviceProperties")) {
        VkPhysicalDeviceProperties p{};
        dp(physical, &p);
        timestamp_period = p.limits.timestampPeriod;
    }
    if (auto f2 = (PFN_vkGetPhysicalDeviceFeatures2)d->get_instance_proc(d->instance, "vkGetPhysicalDeviceFeatures2")) {
        VkPhysicalDeviceVulkan12Features f12{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES};
        VkPhysicalDeviceFeatures2 f{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2};
        f.pNext = &f12;
        f2(physical, &f);
        capture_replay = f12.bufferDeviceAddressCaptureReplay != VK_FALSE;
    }
    return true;
}

bool make_buffer(Buffer& b, VkDeviceSize size, VkBufferUsageFlags usage, bool host, bool read_back) {
    VkBufferCreateInfo ci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    ci.size = size;
    ci.usage = usage | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device, &ci, nullptr, &b.buffer) != VK_SUCCESS) return false;
    VkMemoryRequirements r;
    vkGetBufferMemoryRequirements(device, b.buffer, &r);
    uint32_t t = host ? memory_type(r.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)
                      : memory_type(r.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (host && read_back) {
        const uint32_t c = memory_type(r.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT);
        if (c != UINT32_MAX) t = c;
    }
    if (t == UINT32_MAX) return false;
    VkMemoryAllocateFlagsInfo fi{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO};
    fi.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;
    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.pNext = &fi;
    ai.allocationSize = r.size;
    ai.memoryTypeIndex = t;
    if (vkAllocateMemory(device, &ai, nullptr, &b.memory) != VK_SUCCESS) return false;
    if (vkBindBufferMemory(device, b.buffer, b.memory, 0) != VK_SUCCESS) return false;
    if (host && vkMapMemory(device, b.memory, 0, VK_WHOLE_SIZE, 0, &b.mapped) != VK_SUCCESS) return false;
    VkBufferDeviceAddressInfo ad{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
    ad.buffer = b.buffer;
    b.address = vkGetBufferDeviceAddress(device, &ad);
    b.size = size;
    return true;
}

bool make_buffer_fixed(Buffer& b, VkDeviceSize size, VkBufferUsageFlags usage, const uint64_t want[2], uint64_t got[2]) {
    got[0] = got[1] = 0;
    if (!capture_replay) return make_buffer(b, size, usage, false);
    for (int attempt = 0; attempt < 2; ++attempt) {   // (first at the address asked for, then anywhere)
        const bool ask = attempt == 0;
        if (ask && !want[1]) continue;   // (the memory carries the address; a driver may give 0 for the buffer itself)
        VkBufferOpaqueCaptureAddressCreateInfo oa{VK_STRUCTURE_TYPE_BUFFER_OPAQUE_CAPTURE_ADDRESS_CREATE_INFO};
        oa.opaqueCaptureAddress = ask ? want[0] : 0;
        VkBufferCreateInfo ci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        ci.pNext = &oa;
        ci.flags = VK_BUFFER_CREATE_DEVICE_ADDRESS_CAPTURE_REPLAY_BIT;
        ci.size = size;
        ci.usage = usage | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
        ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VkBuffer buf = VK_NULL_HANDLE;
        if (vkCreateBuffer(device, &ci, nullptr, &buf) != VK_SUCCESS) continue;
        VkMemoryRequirements r;
        vkGetBufferMemoryRequirements(device, buf, &r);
        const uint32_t t = memory_type(r.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VkMemoryOpaqueCaptureAddressAllocateInfo ma{VK_STRUCTURE_TYPE_MEMORY_OPAQUE_CAPTURE_ADDRESS_ALLOCATE_INFO};
        ma.opaqueCaptureAddress = ask ? want[1] : 0;
        VkMemoryAllocateFlagsInfo fi{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO};
        fi.pNext = &ma;
        fi.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT | VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_CAPTURE_REPLAY_BIT;
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.pNext = &fi;
        ai.allocationSize = r.size;
        ai.memoryTypeIndex = t;
        VkDeviceMemory mem = VK_NULL_HANDLE;
        if (t == UINT32_MAX || vkAllocateMemory(device, &ai, nullptr, &mem) != VK_SUCCESS) {
            vkDestroyBuffer(device, buf, nullptr);
            continue;
        }
        if (vkBindBufferMemory(device, buf, mem, 0) != VK_SUCCESS) {
            vkDestroyBuffer(device, buf, nullptr);
            vkFreeMemory(device, mem, nullptr);
            continue;
        }
        b.buffer = buf, b.memory = mem, b.size = size;
        VkBufferDeviceAddressInfo ad{VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO};
        ad.buffer = buf;
        b.address = vkGetBufferDeviceAddress(device, &ad);
        got[0] = vkGetBufferOpaqueCaptureAddress(device, &ad);
        VkDeviceMemoryOpaqueCaptureAddressInfo mi{VK_STRUCTURE_TYPE_DEVICE_MEMORY_OPAQUE_CAPTURE_ADDRESS_INFO};
        mi.memory = mem;
        got[1] = vkGetDeviceMemoryOpaqueCaptureAddress(device, &mi);
        return true;
    }
    return false;
}

bool make_compute(const uint32_t* code, size_t bytes, const VkDescriptorSetLayoutBinding* b, uint32_t nb, uint32_t push_bytes,
                  VkDescriptorSetLayout& dsl, VkPipelineLayout& layout, VkPipeline& pipe) {
    VkDescriptorSetLayoutCreateInfo lci{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    lci.flags = VK_DESCRIPTOR_SET_LAYOUT_CREATE_PUSH_DESCRIPTOR_BIT_KHR;
    lci.bindingCount = nb;
    lci.pBindings = b;
    if (vkCreateDescriptorSetLayout(device, &lci, nullptr, &dsl) != VK_SUCCESS) return false;
    VkPushConstantRange pr{VK_SHADER_STAGE_COMPUTE_BIT, 0, push_bytes};
    VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pli.setLayoutCount = 1;
    pli.pSetLayouts = &dsl;
    pli.pushConstantRangeCount = 1;
    pli.pPushConstantRanges = &pr;
    if (vkCreatePipelineLayout(device, &pli, nullptr, &layout) != VK_SUCCESS) return false;
    VkShaderModuleCreateInfo sci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    sci.codeSize = bytes;
    sci.pCode = code;
    VkShaderModule sm;
    if (vkCreateShaderModule(device, &sci, nullptr, &sm) != VK_SUCCESS) return false;
    VkComputePipelineCreateInfo cci{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
    cci.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_COMPUTE_BIT, sm, "main", nullptr};
    cci.layout = layout;
    return vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &cci, nullptr, &pipe) == VK_SUCCESS;
}

bool make_image(Image& im, uint32_t w, uint32_t h, VkFormat format, VkImageUsageFlags usage) {
    VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    ci.imageType = VK_IMAGE_TYPE_2D;
    ci.format = format;
    ci.extent = {w, h, 1};
    ci.mipLevels = 1, ci.arrayLayers = 1;
    ci.samples = VK_SAMPLE_COUNT_1_BIT;
    ci.tiling = VK_IMAGE_TILING_OPTIMAL;
    ci.usage = usage;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (vkCreateImage(device, &ci, nullptr, &im.image) != VK_SUCCESS) return false;
    VkMemoryRequirements r;
    vkGetImageMemoryRequirements(device, im.image, &r);
    const uint32_t t = memory_type(r.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (t == UINT32_MAX) return false;
    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.allocationSize = r.size;
    ai.memoryTypeIndex = t;
    if (vkAllocateMemory(device, &ai, nullptr, &im.memory) != VK_SUCCESS) return false;
    if (vkBindImageMemory(device, im.image, im.memory, 0) != VK_SUCCESS) return false;
    VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    vi.image = im.image;
    vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vi.format = format;
    vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    if (vkCreateImageView(device, &vi, nullptr, &im.view) != VK_SUCCESS) return false;
    im.w = w, im.h = h, im.fresh = true;
    return true;
}

bool make_image3d(Image& im, uint32_t w, uint32_t h, uint32_t d, VkFormat format, VkImageUsageFlags usage) {
    VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    ci.imageType = VK_IMAGE_TYPE_3D;
    ci.format = format;
    ci.extent = {w, h, d};
    ci.mipLevels = 1, ci.arrayLayers = 1;
    ci.samples = VK_SAMPLE_COUNT_1_BIT;
    ci.tiling = VK_IMAGE_TILING_OPTIMAL;
    ci.usage = usage;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (vkCreateImage(device, &ci, nullptr, &im.image) != VK_SUCCESS) return false;
    VkMemoryRequirements r;
    vkGetImageMemoryRequirements(device, im.image, &r);
    const uint32_t t = memory_type(r.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (t == UINT32_MAX) return false;
    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.allocationSize = r.size;
    ai.memoryTypeIndex = t;
    if (vkAllocateMemory(device, &ai, nullptr, &im.memory) != VK_SUCCESS) return false;
    if (vkBindImageMemory(device, im.image, im.memory, 0) != VK_SUCCESS) return false;
    VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    vi.image = im.image;
    vi.viewType = VK_IMAGE_VIEW_TYPE_3D;
    vi.format = format;
    vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    if (vkCreateImageView(device, &vi, nullptr, &im.view) != VK_SUCCESS) return false;
    im.w = w, im.h = h, im.fresh = true;
    return true;
}

void image_ready(VkCommandBuffer cmd, Image& im) {
    if (!im.fresh) return;
    VkImageMemoryBarrier2 b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2};
    b.srcStageMask = VK_PIPELINE_STAGE_2_NONE;
    b.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    b.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
    b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    b.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.image = im.image;
    b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    di.imageMemoryBarrierCount = 1;
    di.pImageMemoryBarriers = &b;
    vkCmdPipelineBarrier2(cmd, &di);
    im.fresh = false;
}

void full_barrier(VkCommandBuffer cmd) {
    VkMemoryBarrier2 m{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
    m.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    m.srcAccessMask = VK_ACCESS_2_MEMORY_WRITE_BIT;
    m.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    m.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    VkDependencyInfo di{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
    di.memoryBarrierCount = 1;
    di.pMemoryBarriers = &m;
    vkCmdPipelineBarrier2(cmd, &di);
}

} // namespace rt::vk
