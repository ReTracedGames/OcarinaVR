// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// The VR mode. This file: the mode's state (options, the head's pose, the camera the head
// drives at the source, the frame's pictures for each eye) and the "fake headset" (vr = sim) that proves all of it
// without a headset. OpenXR lives in rt_xr.cpp, which is the only file that knows a real headset.
//
// Two functions are called by the host itself, by name, from the game's own threads (hooks V1 and V3 in
// host/hos_main.c): rt_vr_boot (the picture's size, before the game sets its graphics up) and rt_vr_view (the game
// just built its gameplay camera: the head is laid over it).
#pragma once
#include "rt_api.h"

#include <string>

namespace rt::vr {

// A pose in the tracking space (x right, y up, -z forward, metres): what OpenXR gives, what the fake headset makes.
struct Pose {
    double q[4] = {0, 0, 0, 1};   // orientation, x y z w
    double p[3] = {0, 0, 0};      // position
};

// One eye as the headset gives it, against the head: where it is, how it is turned, and its field. The pictures we hand
// the headset are made in exactly this field and orientation (see rt_vr_warp.comp).
struct EyeGeom {
    double q[4] = {0, 0, 0, 1};   // orientation against the head, x y z w
    double p[3] = {0, 0, 0};      // place against the head, metres
    float fov[4] = {0, 0, 0, 0};  // field, angles in radians: left, right, up, down (left and down are negative)
};
void set_eyes(const EyeGeom eyes[2]);   // from the headset, whenever it tells them
double relief();                         // 0 to 1: how much of the eyes' gap the pictures are made with (0: the same picture for both)
void set_relief(double r);

// The pictures of one finished frame of the game, as the headset's module takes them (our images, in the GENERAL layout).
struct Pair {
    uint64_t serial = 0;          // 0: none yet
    int kind = 0;                 // 0 a flat picture, 1 the big screen with relief, 2 the head's own view
    Pose head;                    // kind 2: the head's pose the frame was drawn with
    float tan_x = 1, tan_y = 1;   // kind 2: the half field of the game's own picture
    EyeGeom eg[2];                // kind 2: the field and orientation each eye's picture was made in (to be declared as they are)
    uint32_t w = 0, h = 0;
    VkImage eye[2] = {}, ui = VK_NULL_HANDLE;
    bool ui_separate = false;     // kind 2: the interface is on its own image (for a panel)
};
bool pair_now(Pair* out);         // the last finished one whose commands are all submitted; false: none yet
void set_ipd(double metres);      // the gap between the eyes, as the headset tells it
double ipd_m();
int mode();                       // 0 "ecran", 1 "libre"
void set_mode(int m);
int person();                     // 1: seen through Link's eyes; 3: the game's own camera behind him
void set_person(int p);
bool film();                      // the camera view for filming, in the game's window on the desk
void set_film(bool on);
// The Insert menu's VR rows (RtExports::live_get / live_set): 13 first person (0/1), 14 big screen (0/1), 15 the right
// stick turns by steps (0/1).
int live_get(int id);
void live_set(int id, int value);
// Hook V4 (host/nn_hid.c): the gamepads' inputs, by position, before the host binds them to the game's buttons. The
// headset's controllers are added, and in the first person the sticks are turned to the way the player faces.
void pad_hook(uint64_t* buttons, int32_t stick[4]);

bool wanted();                    // option vr = 1 | sim
bool sim();                       // vr = sim: the fake headset (poses from options, pictures to files)
bool effects();                   // vr_effects = 1: our graphics effects run in the VR mode too
// The scene's size as the VR mode knows it (false: not known yet). With our effects running in the VR mode, they take
// the scene's depth target by this size, never as "the largest one": a square shadow map is larger than a headset's
// picture (found: our per-pixel images were remade at 4096 x 4096 every other moment).
bool scene_size(uint32_t* w, uint32_t* h);
// Reads the options once. With the VR mode on, our graphics effects are switched off whatever the shortcut says (they
// are another project).
void configure();
void device_created(const RtHost* host, const RtDevice* d);
int draw(const RtDraw* d);        // bit 0: the core wants to record its own commands now; bit 2: see draw_indices
const uint32_t* draw_indices(uint32_t* count);   // (RtExports::draw_indices)
// A uniform block of a draw the core asked to see (draw() returned bit 3): may be replaced by a copy of the same size.
uint32_t uniform_seen(const RtUniform* u, void* out, uint32_t cap);
bool inject(VkCommandBuffer cmd); // true when the stop was the VR mode's
void recording();
// Hook V6 (the game's interface on an image of ours): see RtExports::ui_target and ui_view.
bool wants_ui();
int ui_target(VkImage image, VkFormat format, VkImageUsageFlags usage, uint32_t w, uint32_t h, int has_depth, VkImageView* view,
              VkFormat* view_format);
int ui_view(VkImageView view);
void frame(uint64_t number);
// The head's pose, given by the headset's module once per headset frame (null: no pose known, the camera is the game's).
void set_head(const Pose* head, int64_t display_time);
std::string report_json();

} // namespace rt::vr
