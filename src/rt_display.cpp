// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// What Windows says about each display: HDR on or off, peak and full-frame luminance
// (DXGI IDXGIOutput6::GetDesc1), the SDR white level ("paper white", DisplayConfig), the desktop size. Written from the
// public Windows documentation; DXGI is loaded at run time, so the core does not depend on it.
#include "rt_display.h"
#include "rt_diag.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <dxgi1_6.h>

#include <cstdio>
#include <cwchar>
#include <vector>

namespace rt {
namespace {

// SDR white level of the display whose GDI name is gdi (\\.\DISPLAY1), in nits; 0 when unknown.
double sdr_white_nits(const wchar_t* gdi) {
    UINT32 np = 0, nm = 0;
    if (GetDisplayConfigBufferSizes(QDC_ONLY_ACTIVE_PATHS, &np, &nm) != ERROR_SUCCESS) return 0;
    std::vector<DISPLAYCONFIG_PATH_INFO> paths(np);
    std::vector<DISPLAYCONFIG_MODE_INFO> modes(nm);
    if (QueryDisplayConfig(QDC_ONLY_ACTIVE_PATHS, &np, paths.data(), &nm, modes.data(), nullptr) != ERROR_SUCCESS) return 0;
    for (UINT32 i = 0; i < np; ++i) {
        DISPLAYCONFIG_SOURCE_DEVICE_NAME src{};
        src.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_SOURCE_NAME;
        src.header.size = sizeof src;
        src.header.adapterId = paths[i].sourceInfo.adapterId;
        src.header.id = paths[i].sourceInfo.id;
        if (DisplayConfigGetDeviceInfo(&src.header) != ERROR_SUCCESS || std::wcscmp(src.viewGdiDeviceName, gdi) != 0) continue;
        DISPLAYCONFIG_SDR_WHITE_LEVEL w{};
        w.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_SDR_WHITE_LEVEL;
        w.header.size = sizeof w;
        w.header.adapterId = paths[i].targetInfo.adapterId;
        w.header.id = paths[i].targetInfo.id;
        if (DisplayConfigGetDeviceInfo(&w.header) == ERROR_SUCCESS) return w.SDRWhiteLevel / 1000.0 * 80.0;   // (documented unit)
    }
    return 0;
}

} // namespace

std::vector<DisplayInfo> displays() {
    std::vector<DisplayInfo> out;
    HMODULE dx = LoadLibraryA("dxgi.dll");
    if (!dx) return out;
    auto create = (HRESULT(WINAPI*)(REFIID, void**))(void*)GetProcAddress(dx, "CreateDXGIFactory1");
    IDXGIFactory1* f = nullptr;
    if (!create || FAILED(create(__uuidof(IDXGIFactory1), (void**)&f)) || !f) return out;
    IDXGIAdapter1* a = nullptr;
    for (UINT ai = 0; f->EnumAdapters1(ai, &a) != DXGI_ERROR_NOT_FOUND; ++ai) {
        IDXGIOutput* o = nullptr;
        for (UINT oi = 0; a->EnumOutputs(oi, &o) != DXGI_ERROR_NOT_FOUND; ++oi) {
            IDXGIOutput6* o6 = nullptr;
            if (SUCCEEDED(o->QueryInterface(__uuidof(IDXGIOutput6), (void**)&o6)) && o6) {
                DXGI_OUTPUT_DESC1 d{};
                if (SUCCEEDED(o6->GetDesc1(&d))) {
                    DisplayInfo di;
                    char name[64];
                    std::snprintf(name, sizeof name, "%ls", d.DeviceName);
                    di.name = name;
                    di.width = d.DesktopCoordinates.right - d.DesktopCoordinates.left;
                    di.height = d.DesktopCoordinates.bottom - d.DesktopCoordinates.top;
                    di.hdr_on = d.ColorSpace == DXGI_COLOR_SPACE_RGB_FULL_G2084_NONE_P2020;
                    di.bits = d.BitsPerColor;
                    di.peak_nits = d.MaxLuminance;
                    di.full_frame_nits = d.MaxFullFrameLuminance;
                    di.min_nits = d.MinLuminance;
                    di.paper_white_nits = sdr_white_nits(d.DeviceName);
                    di.primary = d.DesktopCoordinates.left == 0 && d.DesktopCoordinates.top == 0;
                    out.push_back(di);
                }
                o6->Release();
            }
            o->Release();
        }
        a->Release();
    }
    f->Release();
    return out;
}

std::string displays_json(const std::vector<DisplayInfo>& v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); ++i) {
        char b[400];
        const auto& d = v[i];
        std::snprintf(b, sizeof b,
                      "%s{\"name\": \"%s\", \"primary\": %s, \"desktop\": \"%dx%d\", \"hdr_on\": %s, \"bits\": %u, "
                      "\"peak_nits\": %.1f, \"full_frame_nits\": %.1f, \"min_nits\": %.4f, \"paper_white_nits\": %.1f}",
                      i ? ", " : "", json_escape(d.name).c_str(), d.primary ? "true" : "false", d.width, d.height,
                      d.hdr_on ? "true" : "false", d.bits, d.peak_nits, d.full_frame_nits, d.min_nits, d.paper_white_nits);
        s += b;
    }
    return s + "]";
}

} // namespace rt
