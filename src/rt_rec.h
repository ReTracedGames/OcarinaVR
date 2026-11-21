// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// The video recorder: what the game's window shows on the desk, and the game's own sound, written to an MP4 file by
// Windows' own encoders (Media Foundation), without any outside recording tool. A tool
// of ours, not part of the published VR mode.
//
// Options: vr_rec = 1 (records from the first picture to the game's exit), vr_key_rec (the key that starts and stops:
// N), vr_rec_height (1080), vr_rec_mbps (30), vr_rec_sound (1), vr_rec_dir (the folder: "videos" beside our outputs'
// folder), vr_rec_seconds (0: no limit; for trials).
#pragma once
#include "rt_api.h"

#include <string>

namespace rt::rec {

bool available();                 // false in a build without the recorder
void configure();
bool wanted();                    // a recording is asked for (from the start, or by the key or the controllers since)
bool on();                        // a file is being written
void toggle();                    // asks for the start, or for the end
int key();                        // the keyboard key that toggles (0: none)
// Each frame, on the replay thread, with the picture the game's window is about to show (linear light, laid over the
// whole of its image): the picture is made ready for the encoder and its copy to memory is ordered. Starts and ends
// the file when asked.
void frame(VkCommandBuffer cmd, VkImageView picture);
void shutdown();                  // the game exits: the file is closed properly first
std::string report_json();

} // namespace rt::rec
