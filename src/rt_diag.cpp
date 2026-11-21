// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
#include "rt_diag.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>

namespace rt {

extern void (*host_log_fn)(const char*);   // rt_core.cpp

namespace {
std::mutex log_mu;
FILE* log_file;
std::mutex opt_mu;
std::map<std::string, std::string> ini;     // from retraced.ini
std::map<std::string, std::string> used;    // what was asked, with the value given, for the report
std::map<std::string, std::string> forced;  // (force_option: over the environment and the files; the VR mode's switch-offs)

std::string module_dir() {
    HMODULE self = nullptr;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       (LPCSTR)&module_dir, &self);
    char p[MAX_PATH * 2] = {};
    GetModuleFileNameA(self, p, sizeof p);
    std::string s(p);
    size_t k = s.find_last_of("\\/");
    return k == std::string::npos ? std::string(".") : s.substr(0, k);
}

void vlog(bool host, const char* fmt, va_list a) {
    char b[2048];
    std::vsnprintf(b, sizeof b, fmt, a);
    std::lock_guard lk(log_mu);
    if (!log_file) {
        CreateDirectoryA(out_dir().c_str(), nullptr);
        log_file = std::fopen((out_dir() + "\\retraced.log").c_str(), "w");
    }
    if (log_file) {
        SYSTEMTIME t;
        GetLocalTime(&t);
        std::fprintf(log_file, "[%02d:%02d:%02d.%03d] %s\n", t.wHour, t.wMinute, t.wSecond, t.wMilliseconds, b);
        std::fflush(log_file);
    }
    if (host && host_log_fn) host_log_fn(b);
}

std::string trim(const std::string& s) {
    size_t a = 0, z = s.size();
    while (a < z && std::isspace((unsigned char)s[a])) ++a;
    while (z > a && std::isspace((unsigned char)s[z - 1])) --z;
    return s.substr(a, z - a);
}
} // namespace

const std::string& dll_dir() {
    static const std::string d = module_dir();
    return d;
}

const std::string& out_dir() {
    static const std::string d = [] {
        const char* e = std::getenv("RETRACED_OUT");
        return (e && *e) ? std::string(e) : dll_dir() + "\\retraced-out";
    }();
    return d;
}

void log(const char* fmt, ...) {
    va_list a;
    va_start(a, fmt);
    vlog(false, fmt, a);
    va_end(a);
}

void log_host(const char* fmt, ...) {
    va_list a;
    va_start(a, fmt);
    vlog(true, fmt, a);
    va_end(a);
}

void read_ini(const std::string& path) {   // (opt_mu held)
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return;
    char line[1024];
    while (std::fgets(line, sizeof line, f)) {
        std::string s = trim(line);
        if (s.empty() || s[0] == ';' || s[0] == '#' || s[0] == '[') continue;
        size_t eq = s.find('=');
        if (eq == std::string::npos) continue;
        std::string k = trim(s.substr(0, eq)), v = trim(s.substr(eq + 1));
        size_t c = v.find_first_of(";#");   // comment after the value
        if (c != std::string::npos) v = trim(v.substr(0, c));
        for (auto& ch : k) ch = (char)std::tolower((unsigned char)ch);
        ini[k] = v;
    }
    std::fclose(f);
}

void load_options() {
    std::lock_guard lk(opt_mu);
    ini.clear();
    read_ini(dll_dir() + "\\retraced.ini");
    read_ini(dll_dir() + "\\retraced_menu.ini");   // (the Insert menu's choices: over retraced.ini, under the environment)
}

void save_menu_option(const char* key, int64_t value) {
    std::lock_guard lk(opt_mu);
    const std::string path = dll_dir() + "\\retraced_menu.ini";
    std::map<std::string, std::string> kept;
    if (FILE* f = std::fopen(path.c_str(), "r")) {
        char line[512];
        while (std::fgets(line, sizeof line, f)) {
            std::string s = trim(line);
            size_t eq = s.find('=');
            if (s.empty() || s[0] == ';' || eq == std::string::npos) continue;
            kept[trim(s.substr(0, eq))] = trim(s.substr(eq + 1));
        }
        std::fclose(f);
    }
    kept[key] = std::to_string((long long)value);
    if (FILE* f = std::fopen(path.c_str(), "w")) {
        std::fputs("; Choices made in the game's Insert menu (Graphics tab). Read after retraced.ini;\n"
                   "; an environment variable RETRACED_<KEY> (the .bat shortcuts) always wins.\n", f);
        for (const auto& [k, v] : kept) std::fprintf(f, "%s = %s\n", k.c_str(), v.c_str());
        std::fclose(f);
    }
}

std::string opt(const char* key, const char* fallback) {
    std::string k(key), env = "RETRACED_";
    for (auto& ch : k) ch = (char)std::tolower((unsigned char)ch);
    for (char ch : k) env += (char)std::toupper((unsigned char)ch);
    std::string v = fallback ? fallback : "";
    std::lock_guard lk(opt_mu);
    if (auto f = forced.find(k); f != forced.end()) v = f->second;
    else if (const char* e = std::getenv(env.c_str()); e && *e) v = e;
    else if (auto it = ini.find(k); it != ini.end()) v = it->second;
    used[k] = v;
    return v;
}

void force_option(const char* key, const char* value) {
    std::string k(key);
    for (auto& ch : k) ch = (char)std::tolower((unsigned char)ch);
    std::lock_guard lk(opt_mu);
    forced[k] = value;
}

int64_t opt_int(const char* key, int64_t fallback) {
    std::string v = opt(key, std::to_string(fallback).c_str());
    char* end = nullptr;
    long long n = std::strtoll(v.c_str(), &end, 0);
    return (end && end != v.c_str()) ? (int64_t)n : fallback;
}

std::string options_json() {
    std::lock_guard lk(opt_mu);
    std::string s = "{";
    bool first = true;
    for (auto& [k, v] : used) {
        s += (first ? "" : ", ") + std::string("\"") + json_escape(k) + "\": \"" + json_escape(v) + "\"";
        first = false;
    }
    return s + "}";
}

int64_t now_ns() {
    static const int64_t freq = [] {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        return (int64_t)f.QuadPart;
    }();
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return (int64_t)((__int128)c.QuadPart * 1000000000 / freq);
}

bool write_file_atomic(const std::string& path, const std::string& text) {
    std::string tmp = path + ".tmp";
    FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) return false;
    bool ok = std::fwrite(text.data(), 1, text.size(), f) == text.size();
    ok = (std::fclose(f) == 0) && ok;
    // (no MOVEFILE_WRITE_THROUGH: flushing to disk cost 1.8 ms on the presentation thread)
    return ok && MoveFileExA(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING);
}

std::string json_escape(const std::string& s) {
    std::string o;
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') o += '\\', o += (char)c;
        else if (c < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); o += b; }
        else o += (char)c;
    }
    return o;
}

} // namespace rt
