// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// VR mode: the player's hands. What the headset knows of each hand (a controller held, or the bare hand with its
// fingers), Link's skeleton, and the pose of his arms that puts his hands where the player's are. The aim: Link's
// hands follow the controllers or the bare hands and move as freely as the player's own, as if the player's body
// were in the game.
//
// Nothing here talks to the headset or to the card: positions in, matrices out (rt_xr.cpp feeds the hands, rt_vr.cpp
// writes the matrices into the game's own pose block). The fake headset has fake hands (vr_sim_hands).
#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

namespace rt::hands {

// One hand of the player, in the play space: metres, x to the right, y up, -z forward, the space the head's pose is
// given in (its origin: where the head was when the view was last centred).
struct Hand {
    bool tracked = false;        // a pose is known now
    bool fingers = false;        // the fingers themselves are followed (a bare hand): curl[] is measured, not made up
    // the grip pose, as OpenXR defines it: its origin in the palm; +x out of the left palm (into the right one), -z from
    // the little finger to the thumb, -y along the hand towards the knuckles
    double q[4] = {0, 0, 0, 1};
    double p[3] = {};
    double aim_q[4] = {0, 0, 0, 1}, aim_p[3] = {};   // where the hand points (-z), for aiming
    double vel[3] = {};          // metres a second
    double ang[3] = {};          // radians a second
    float trigger = 0, squeeze = 0;   // 0..1 (bare hand: a pinch of thumb and forefinger, a closed fist)
    float curl[5][3] = {};       // how far each joint of each finger is bent, radians (thumb, fore, middle, ring, little)
    int64_t time = 0;            // our clock when it was read
};

void configure();                              // options (once)
void set_hand(int side, const Hand& h);        // 0 left, 1 right: the headset's thread
bool hand_now(int side, Hand* out);            // the newest (false: none these last 0.3 s)
void latch();                                  // start of a frame's drawing: the hands this frame is drawn with
bool hand_latched(int side, Hand* out);

bool load(FILE* f, uint32_t n_models);         // Link's skeletons, from the file of tools/vr-corps.py
uint32_t matrices(int model);                  // how many 3 x 4 matrices the model's pose block holds (0: unknown)

// How the play space lies in the game's world for one frame: world = eye + base^T (p * units).
struct Frame {
    double base[3][3] = {};      // rows: the world's directions of the play space's x, y, z
    double eye[3] = {};
    double units = 1.0;          // game units to a metre
    double head_q[4] = {0, 0, 0, 1}, head_p[3] = {};
};

// Link's pose block with his arms where the player's hands are. `in`: the game's block (n matrices of 3 rows of 4
// floats), `out`: the copy that is changed; `offset`: what the game takes off world positions in this block.
// body: 1 his arms alone, at the player's own size, hands exactly where the player's are; 0 his whole body is drawn:
// the arms stay on his shoulders and reach in the same direction as the player's.
// Returns false when nothing was changed (no hand tracked, model unknown).
bool pose(int model, const Frame& fr, const float offset[3], const uint8_t* in, uint8_t* out, uint32_t n, int body);

// What Link holds or wears: a rigid thing drawn near him in a first person's frame. `id` names the thing, `world` is
// its own place (3 x 4 rows), `in` the game's pose block of this frame. Returns 0: leave it as the game draws it;
// 1: draw it at `out` (it is held: it follows the player's hand, at the hand's size); 2: do not draw it (it is fastened
// to the body, which is not drawn). A thing is taken as held or worn once it has kept its place against a bone of
// Link's while that bone moved: a pot on the ground beside him never does.
int attached(int model, uint64_t id, const double world[12], const Frame& fr, const float offset[3], const uint8_t* in, uint32_t n,
             int body, double out[12]);
// The same, for a thing of one piece (an arrow on a bow): when the thing in the same hand is drawn out between the
// player's two hands, this one is laid along the same line (see attached_block).
int attached_rigid(int model, uint64_t id, const double world[12], const Frame& fr, const float offset[3], const uint8_t* in, uint32_t n,
                   int body, double out[12]);
// The same for a thing that has bones of its own (the game draws Link's shield as a one-bone figure): `in` is ITS pose
// block of n matrices, `thing_offset` what the game takes off in it. On 1, `out` is the block moved as one piece to
// the player's hand; on 2, a block that draws nothing.
int attached_block(int model, uint64_t id, const Frame& fr, const float offset[3], const uint8_t* link_in, uint32_t link_n, int body,
                   const float thing_offset[3], const uint8_t* in, uint8_t* out, uint32_t n);
bool carrying();                               // something follows one of the player's hands now
bool two_handed();                             // a thing in one hand has a part in the other now (a string, a pouch)
bool held(uint64_t id);                        // this thing is in one of the player's hands (it was at one of the last frames)

// The player's movements as the game's own commands (called where the game reads its pad). The head's pose, whether
// the head drives the view, and, when the game's own camera sits at Link's eyes (he aims a thing), how far the hand
// points from where the game aims (radians, to the left and upwards).
struct Moment {
    double head_q[4] = {0, 0, 0, 1}, head_p[3] = {};
    bool first_person = false, game_aims = false;
    double aim_yaw = 0, aim_pitch = 0;
    double pull = 0;             // while he aims: how far the hand that draws is behind the hand that holds (metres; 0: one hand aims)
    float shake[2] = {};         // out: what each hand should feel (0 to 1)
    bool sticks_taken = false;   // out: the left stick steers the game's aim (it must not be turned to where the head looks)
};
void gestures(Moment& m, uint64_t* buttons, int32_t stick[4]);
bool gestures_on();                            // (the grips are then ours: the shield and the sword's hold)
int main_hand();                               // the hand that strikes and aims (0 left, 1 right)
void set_gestures(bool on);
bool arms_on();
void set_arms(bool on);
int tilt_degrees();
void set_tilt(int degrees);
int arm_cm();
double link_arm(int model);   // Link's own arm (0 the child, 1 the adult), game units; 0 when his skeleton is not read
void set_arm_cm(int cm);

std::string report_json();

} // namespace rt::hands
