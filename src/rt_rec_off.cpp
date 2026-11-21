// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// The core asks a video recorder that is not part of this release: here it answers "nothing".
#include "rt_rec.h"

namespace rt::rec {

bool available() { return false; }
void configure() {}
bool wanted() { return false; }
bool on() { return false; }
void toggle() {}
int key() { return 0; }
void frame(VkCommandBuffer, VkImageView) {}
void shutdown() {}
std::string report_json() { return "{}"; }

} // namespace rt::rec
