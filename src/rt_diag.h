// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// Diagnostics of the core: our own log file, options (retraced.ini + RETRACED_* variables), clock, report.
#pragma once
#include <cstdint>
#include <string>

namespace rt {

// Folder of our outputs: RETRACED_OUT, else "retraced-out" beside retraced.dll.
const std::string& out_dir();
// Folder of retraced.dll itself.
const std::string& dll_dir();

void log(const char* fmt, ...);        // our log only (retraced.log)
void log_host(const char* fmt, ...);   // our log and the host's log

// Options: RETRACED_<KEY> (upper case) wins over "key=value" in retraced.ini beside the DLL.
std::string opt(const char* key, const char* fallback);
int64_t opt_int(const char* key, int64_t fallback);
void load_options();
// A value that wins over the environment and the files from now on (the VR mode switches our graphics effects off).
void force_option(const char* key, const char* value);
void save_menu_option(const char* key, int64_t value);   // (jeu\retraced_menu.ini: the Insert menu's choices)
std::string options_json();            // every option read, for the report

int64_t now_ns();                      // monotonic clock

// Writes text to path atomically (temporary file then rename), so a reader never sees half a file.
bool write_file_atomic(const std::string& path, const std::string& text);

std::string json_escape(const std::string& s);

} // namespace rt
