// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// The headset: everything OpenXR. The only file that includes openxr.h; the
// official loader (openxr_loader.dll, Khronos, Apache-2.0 OR MIT) is opened at need: without it, or without a headset,
// the game starts as it does today.
//
// The headset has a thread of its own: one turn per picture it shows, with the last picture the game finished
// (rt_vr.cpp makes the two eyes and the interface). The game never waits for it and keeps its own 60 pictures a second.
#pragma once
#include "rt_api.h"

#include <string>

namespace rt::vr {
struct Pose;
}

namespace rt::xr {

// At boot, before the game's graphics: the loader, an instance, the headset. False when there is none (the log says why).
bool boot();
bool on();
// The size the headset advises for one eye's picture (0 when unknown).
void recommended(uint32_t* w, uint32_t* h);
uint32_t instance_extensions(const char* const* avail, uint32_t n, const char** out, uint32_t cap);
uint32_t device_extensions(const char* const* avail, uint32_t n, const char** out, uint32_t cap);
void device_created(const RtHost* host, const RtDevice* d);
// The headset's controllers as a gamepad (hook V4): buttons by position (bit numbers below), sticks as XInput's.
// Returns 0 when no controller is held.
// The head's pose now, a little ahead (for the game's thread when it builds its view); false: none (no headset on a head).
bool locate_head(rt::vr::Pose* out, int64_t* time);
int pad(uint64_t* buttons, int32_t stick[4]);
int exit_hard();             // 1: a headset's runtime was loaded, the process must end without unloading its libraries
int shutdown(int wait_ms);   // the game exits: the link with the headset closed first (1), or not in time (0)
// The game's vibration for one hand (0 left, 1 right), as the host maps it to a gamepad's heavy and light motors.
void rumble(int side, float heavy, float light);
std::string report_json();

} // namespace rt::xr
