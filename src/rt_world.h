// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project

#pragma once
#include "rt_api.h"

#include <string>

namespace rt::world {

bool wanted();
uint32_t device_extensions(const char* const* avail, uint32_t n, const char** out, uint32_t cap);
void device_features(void*** tail);
void device_created(const RtHost* host, const RtDevice* d);
bool ready();
int draw(const RtDraw* d);
void inject(VkCommandBuffer cmd);
bool sun_wanted();
bool air_wanted();
bool gi_wanted();
uint32_t blas_extra();
int cut_class(uint64_t key);
bool is_water_program(uint64_t key);
uint64_t frame_number();
uint32_t dump_seq_next();
int live_get(int id);
void live_set(int id, int value);
const char* status_text();
bool refl_wanted();

bool is_ctx_addr(uint64_t guest_addr);
void set_sun(const float travel_dir[3]);
void set_effects(bool (*on)());

void set_env(const uint8_t* env, uint32_t size);
bool xfb_patching();
void xfb_patch_failed();
void xfb_begin(VkCommandBuffer cmd);
void xfb_end(VkCommandBuffer cmd);
void frame();
void recording();
std::string report_json();

}
