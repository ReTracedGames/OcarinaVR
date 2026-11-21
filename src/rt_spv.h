// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
#pragma once
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <vector>

namespace rt {

// 1: out holds the module with gl_Position as a transform feedback output (buffer 0, stride 16); 0: not a vertex
// module, or one that already uses transform feedback (left as it is); -1: a vertex module that could not be edited.
int spv_add_position_xfb(const uint32_t* words, size_t n, std::vector<uint32_t>& out);

// Chantier 3, grass and foliage. A mesh-stage module that outputs triangles also writes them into two buffers of
// ours, reached by their device addresses (words: uints, clip: floats; slots: how many vertex / triangle slots they
// hold). 1: out holds the edited module; 0: not such a module (left as it is); -1: a mesh module of another form.
int spv_add_mesh_capture(const uint32_t* words, size_t n, uint64_t addr_words, uint64_t addr_clip, uint32_t slots,
                         std::vector<uint32_t>& out);

} // namespace rt
