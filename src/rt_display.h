// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
#pragma once
#include <string>
#include <vector>

namespace rt {

struct DisplayInfo {
    std::string name;
    bool primary = false, hdr_on = false;
    int width = 0, height = 0;
    unsigned bits = 0;
    double peak_nits = 0, full_frame_nits = 0, min_nits = 0, paper_white_nits = 0;
};

std::vector<DisplayInfo> displays();
std::string displays_json(const std::vector<DisplayInfo>& v);

} // namespace rt
