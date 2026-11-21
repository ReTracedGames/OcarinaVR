// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project

#pragma once
#include "rt_api.h"

#include <string>

namespace rt::veg {

bool wanted();
void device_created();
bool on();

bool addresses(uint64_t* words, uint64_t* clip, uint32_t* slots);
void note_module(int result);

void set_flag(VkCommandBuffer cmd, bool write);

void process(VkCommandBuffer cmd, const float* inv_vp);
VkDeviceAddress bottom_level();
std::string report_json();

}
