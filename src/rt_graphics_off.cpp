// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// OcarinaVR is the VR mode alone. The core it shares with the ReTraced project's graphics work (rt_core.cpp) still asks
// three modules what they want: the world as rays see it, the per-pixel images, the grass. They are not part of this
// release; here each of them answers "nothing", so that the core builds and runs as it is.
#include "rt_gbuf.h"
#include "rt_veg.h"
#include "rt_world.h"

#include <atomic>

namespace rt::world {

namespace {
std::atomic<uint64_t> frames{0};
std::atomic<uint32_t> dump_seq{0};
} // namespace

bool wanted() { return false; }
uint32_t device_extensions(const char* const*, uint32_t, const char**, uint32_t) { return 0; }
void device_features(void***) {}
void device_created(const RtHost*, const RtDevice*) {}
bool ready() { return false; }
int draw(const RtDraw*) { return 0; }
void inject(VkCommandBuffer) {}
bool sun_wanted() { return false; }
bool air_wanted() { return false; }
bool gi_wanted() { return false; }
uint32_t blas_extra() { return 0; }
int cut_class(uint64_t) { return 0; }
bool is_water_program(uint64_t) { return false; }
uint64_t frame_number() { return frames.load(std::memory_order_relaxed); }
uint32_t dump_seq_next() { return dump_seq++; }
int live_get(int) { return 0; }
void live_set(int, int) {}
const char* status_text() { return ""; }
bool refl_wanted() { return false; }
bool is_ctx_addr(uint64_t) { return false; }
void set_sun(const float[3]) {}
void set_effects(bool (*)()) {}
void set_env(const uint8_t*, uint32_t) {}
bool xfb_patching() { return false; }
void xfb_patch_failed() {}
void xfb_begin(VkCommandBuffer) {}
void xfb_end(VkCommandBuffer) {}
void frame() { frames++; }
void recording() {}
std::string report_json() { return "{\"on\": false}"; }

} // namespace rt::world

namespace rt::gbuf {

bool wanted() { return false; }
void device_created() {}
uint32_t extra_targets(const RtTargets*, VkImageView*, VkFormat*) { return 0; }
void inject(VkCommandBuffer, uint32_t, uint32_t) {}
bool dump_wanted() { return false; }
void dump_now(VkCommandBuffer, VkImage, VkFormat, VkImageUsageFlags, VkImage, VkFormat, VkImageUsageFlags) {}
VkImageView view(int) { return VK_NULL_HANDLE; }
void frame() {}
std::string report_json() { return "{\"on\": false}"; }

} // namespace rt::gbuf

namespace rt::veg {

bool wanted() { return false; }
void device_created() {}
bool on() { return false; }
bool addresses(uint64_t*, uint64_t*, uint32_t*) { return false; }
void note_module(int) {}
void set_flag(VkCommandBuffer, bool) {}
void process(VkCommandBuffer, const float*) {}
VkDeviceAddress bottom_level() { return 0; }
std::string report_json() { return "{\"on\": false}"; }

} // namespace rt::veg
