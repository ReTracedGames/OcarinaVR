// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project

#pragma once
#include "rt_api.h"

#include <string>

namespace rt::gbuf {

bool wanted();
void device_created();

uint32_t extra_targets(const RtTargets* in, VkImageView* views, VkFormat* formats);

void inject(VkCommandBuffer cmd, uint32_t scene_w, uint32_t scene_h);

bool dump_wanted();
void dump_now(VkCommandBuffer cmd, VkImage depth, VkFormat depth_format, VkImageUsageFlags depth_usage, VkImage lit,
              VkFormat lit_format, VkImageUsageFlags lit_usage);

VkImageView view(int i);
void frame();
std::string report_json();

}
