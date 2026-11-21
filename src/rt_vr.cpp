// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
#include "rt_vr.h"

#include "rt_diag.h"
#include "rt_hands.h"
#include "rt_rec.h"
#include "rt_vk.h"
#include "rt_xr.h"

#include "gen/rt_vr_film_comp.h"
#include "gen/rt_vr_ui_comp.h"
#include "gen/rt_vr_near_comp.h"
#include "gen/rt_vr_warp_comp.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace rt::vr {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr uint64_t kTonemap = 0xab3681946f93594aull;     // (step 222, the game's SDR tone curve)
constexpr uint64_t kTonemapGlare = 0xa00726fca31a4039ull;   // (the same step with the sun in the view: its glare)
constexpr uint64_t kFinalCopy = 0xb7914ae65792d0c8ull;   // (step 229, the copy into the picture the game presents)
constexpr VkFormat kPicFormat = VK_FORMAT_B10G11R11_UFLOAT_PACK32;   // (the game's finished picture, linear light)
constexpr VkFormat kUiFormat = VK_FORMAT_R16G16B16A16_SFLOAT;

// ---- options ----
int g_vr = 0;                    // 0 off, 1 a headset (OpenXR), 2 the fake headset
std::atomic<int> g_mode{1};      // 0 "ecran" (the game's own camera, on a big screen), 1 "libre" (the head drives the camera)
bool g_level = true;             // the horizon stays level whatever the game's camera pitch is
double g_tan_x = 1.376, g_tan_y = 1.428;   // half field of the picture in "libre" (Quest 3: 54 degrees outward, 55 down)
double g_units_per_m = 1.0;      // game units in one metre when the size is fixed (vr_scale: the world at that many % of its size)
bool g_units_auto = true;        // vr_scale = 0 or not given: from Link's own size, see units()
double g_ipd_m = 0.064;
double g_max_px = 64;            // the farthest a point of the picture is moved to make an eye
double g_screen_relief = 1.0, g_screen_conv = 6.0;   // the big screen's relief: its strength, the distance that stays on the screen
bool g_ui_redirect = true, g_show_holes = false, g_predict = true;
// The eyes as the headset gives them. Each eye's picture is made in that eye's own field and orientation, not in a
// field of ours: with the real headset, pictures declared with our own field (the same each side, 54 degrees)
// were shown as if they had the headset's (54 one side, 40 the other): squeezed, and each eye some 15 degrees off, the
// other way for the other eye. vr_eye_field = 0 brings the old way back (for the proof in tools/vr-casque.py).
std::mutex eyes_mu;
EyeGeom g_eyes[2];
bool g_eye_field = true;
std::atomic<double> g_relief{1.0};
double g_field_margin = 0;
// The first person, the VR mode's own view: the view is taken at Link's eyes,
// the way the player faces is ours (the right stick turns it), and the left stick is turned to it.
std::atomic<int> g_person{1};    // 1 first person, 3 the game's own camera
// The height of Link's eyes above his feet (game units, about metres). Not read from the game's camera: its aim is
// 0.946 above the feet at rest in the forest and the plain (the child), but 1.6 in the castle town, and it rises when
// he runs. Two fixed values, the child's and the adult's (the host tells which), and vr_eye_height over both.
double g_eye_h = 0, g_eye_child = 1.0, g_eye_adult = 1.55, g_eye_up = 0, g_fp_forward = 0.14;
bool player_child = true;
// How large the world is to the player: game units in one of the player's metres. With one unit a metre (at
// first) the player stood in the world at his own size while Link the child is a small boy: for Link's hands to
// be where the player's are, his arms were drawn 1.9 times as large, and his hand was as large as another child's
// head: every size was wrong. Now the player has Link's size: one of his metres is as many game units as Link's arm is to his own, so
// the arms are drawn at their true size, the hands fall where the player's are, and the head and the eyes move by
// the same measure (the world is seen as a child sees it; as an adult for Link the adult).
double units() {
    if (!g_units_auto) return g_units_per_m;
    const double arm = hands::link_arm(player_child ? 0 : 1), mine = hands::arm_cm() * 0.01;
    return arm > 0 && mine > 0 ? std::clamp(arm / mine, 0.30, 1.50) : g_units_per_m;
}
// In the first person the view sits in Link's head: his hair, his hat and his face are a few centimetres from the eyes
// (seen in the fake headset: at some head angles his hair fills the picture, and so near, the two eyes made
// from one picture cannot agree). Nothing nearer than this is drawn then (the game camera's own "near" distance).
double g_fp_near = 0.30;
double g_turn_snap = 30, g_turn_speed = 120;             // the right stick: turns by steps (degrees; 0: smoothly, degrees a second)
bool g_move_head = true;         // the left stick's "forward" is where the head looks (false: where the body faces)
// What the game takes the left stick against: 1 the view we write (measured: head turned 90 degrees to the
// left in the third person, stick forward, Link ran 75 degrees to the left of the game's camera), 0 its own camera.
int g_stick_frame = 1;
uint32_t g_w = 0, g_h = 0;       // the picture the game is asked to draw (rt_vr_boot)
bool g_configured = false;
// the fake headset's pose: fixed angles (degrees), a position (cm), a spin (degrees a second) and a sway (cm, 4 s)
double sim_yaw = 0, sim_pitch = 0, sim_roll = 0, sim_pos[3] = {0, 0, 0}, sim_spin = 0, sim_sway = 0;
int64_t sim_sleep_ms = 0;        // (vr_sim_sleep_ms: every frame held this long: the game under 60 real frames)
int64_t g_dump_every = 0, g_dump_from = 0, g_ui_ab = 0;
int64_t t_start = 0;

const RtHost* H = nullptr;
std::mutex mu;

// ---- small matrices (rows; a view is world -> view, 3 rows of 4) ----
void quat_rows(const double q[4], double r[3][3]) {   // r[i][j]: row i, column j of the rotation matrix of q
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    r[0][0] = 1 - 2 * (y * y + z * z), r[0][1] = 2 * (x * y - z * w), r[0][2] = 2 * (x * z + y * w);
    r[1][0] = 2 * (x * y + z * w), r[1][1] = 1 - 2 * (x * x + z * z), r[1][2] = 2 * (y * z - x * w);
    r[2][0] = 2 * (x * z - y * w), r[2][1] = 2 * (y * z + x * w), r[2][2] = 1 - 2 * (x * x + y * y);
}
void quat_mul(const double a[4], const double b[4], double o[4]) {
    o[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    o[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    o[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    o[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}
void quat_axis(double ax, double ay, double az, double degrees, double o[4]) {
    const double h = degrees * kPi / 360.0, s = std::sin(h);
    o[0] = ax * s, o[1] = ay * s, o[2] = az * s, o[3] = std::cos(h);
}

// ---- the head ----
std::mutex head_mu;
Pose g_head;                     // (the last pose the headset's module gave, already recentred)
bool g_head_ok = false;
int64_t g_head_time = 0;

Pose sim_pose() {
    const double t = (now_ns() - t_start) / 1e9;
    Pose p;
    double qy[4], qp[4], qr[4], a[4];
    quat_axis(0, 1, 0, sim_yaw + sim_spin * t, qy);   // (turning the head to the left is positive)
    quat_axis(1, 0, 0, sim_pitch, qp);                // (looking up is positive)
    quat_axis(0, 0, -1, sim_roll, qr);                // (leaning the head to the right is positive)
    quat_mul(qy, qp, a);
    quat_mul(a, qr, p.q);
    p.p[0] = sim_pos[0] * 0.01 + sim_sway * 0.01 * std::sin(t * kPi * 0.5);
    p.p[1] = sim_pos[1] * 0.01, p.p[2] = sim_pos[2] * 0.01;
    return p;
}

bool head_now(Pose& p, int64_t& time) {
    if (g_vr == 2) {
        p = sim_pose(), time = now_ns();
        return true;
    }
    // (asked of the headset now, for the moment this frame will be seen: less for the headset to correct afterwards)
    if (g_predict && xr::locate_head(&p, &time)) return true;
    std::lock_guard lk(head_mu);
    p = g_head, time = g_head_time;
    return g_head_ok;
}

std::mutex fp_mu;                // (the first person's state, the player, the times below)
int64_t libre_time = 0;          // when the head last drove a view

// ---- the views the game was given, kept to know which pose a drawn frame was made with ----
struct Given {
    float m[12] = {};            // the view matrix the picture is drawn with
    float cull11 = 0;            // what the game was given at [11] (the same camera, moved back: see g_cull_back)
    Pose head;
    double base[3][3] = {}, eye[3] = {};   // the level frame the head was laid on, and the game camera's position
    int64_t time = 0;            // the headset's time of the pose
    int64_t built = 0;           // our clock when the game built the view
    uint64_t serial = 0;
    bool fp = false;             // the first person's view (Link's body is then left out of it)
};
constexpr int kRing = 64;        // (the drawn frame's view was found 12 builds old in a trial: 16 was too close)
Given ring[kRing];
uint64_t given_serial = 0;
std::atomic<uint64_t> st_views{0}, st_views_cam{0}, st_matched{0}, st_unmatched{0}, st_prepass_frames{0};
float cam_seen[6] = {};          // (the game camera's own near, far, field, ratio, offset: before we write)
double eye_seen[3] = {};
// the game camera's speed (units a second), for "does the game slow its time under 60 real frames"
double speed_from[3] = {}, speed_now = 0, speed_max = 0;
int64_t speed_t0 = 0;

// ---- Link, and the first person's own heading ----
double player_pos[3] = {}, player_from[3] = {};
int64_t player_time = 0, player_t0 = 0;
double move_yaw = 0, move_speed = 0, move_vs_cam = 0, move_vs_view = 0;   // (measured: which way Link runs)
double fp_yaw = 0, eye_h_auto = 0, cam_yaw_now = 0, view_yaw_now = 0, target_dy = 0, target_dxz = 0;
double written_yaw = 0, want_yaw = 0;   // the heading of the view we wrote; the heading "stick forward" must run along
bool fp_now = false;
bool fp_init = false, turn_latch = false;
int64_t fp_time = 0, pad_time = 0;
std::atomic<uint64_t> st_fp_views{0}, st_pad_turns{0};
// The game's own scenes (a conversation, a scene of the story: the host's "event playing"): the game frames them with
// its own camera, so they are shown on the big screen, as the design asks (vr_events = tete to keep the head's view).
bool g_events_screen = true;
int64_t event_since = 0, event_last = 0;
std::atomic<uint64_t> st_event_views{0}, st_event_starts{0};

// ---- the frame being replayed ----
struct Cur {
    bool ctx_ok = false, matched = false, tonemap = false, done = false, ui_redirected = false;
    bool ours_by_field = false;  // a camera block with the headset's own field was met: the frame is the head's, whatever its view says
    uint64_t tested[12] = {};    // the camera blocks tried so far (their addresses)
    int n_tested = 0, n_read = 0, match_order = 0;
    double best_err = 1e30;
    uint8_t ctx[0x130] = {};
    Given given;
    VkImage depth = VK_NULL_HANDLE;
    VkFormat depth_format = VK_FORMAT_UNDEFINED;
    VkImageUsageFlags depth_usage = 0;
    uint32_t w = 0, h = 0;       // of the scene's depth
    uint32_t ui_sets = 0;
    // (diagnosis: why a frame drawn with the head's view is not shown as one) the large depth targets met, and the
    // frame's last draws without depth (the end-of-frame passes)
    struct DepthSeen {
        VkImage image;
        int format;
        uint32_t usage, w, h, writes, draws;
        uint64_t first_key;
    } depths[6] = {};
    int n_depths = 0;
    struct TailDraw {
        uint64_t key;
        int format;
        uint32_t w, h, count;
    } tail[12] = {};
    int n_tail = 0;
    // the fairy nearest the eyes in this frame (her glow takes her depth in the eyes' pictures): where, how far (squared)
    bool halo_ok = false;
    float halo[3] = {};
    double halo_d2 = 0;
} cur;
uint64_t frames = 0;
float last_proj[16] = {}, last_view[12] = {};
uint32_t last_scene[2] = {};
bool last_matched = false;
double last_mismatch = -1;

// ---- the end of the frame on the card ----
struct Pipe {
    VkDescriptorSetLayout dsl = VK_NULL_HANDLE;
    VkPipelineLayout layout = VK_NULL_HANDLE;
    VkPipeline pipe = VK_NULL_HANDLE;
};
bool g_gpu = false;              // (our two passes are made)
Pipe p_warp, p_ui, p_near, p_film;
vk::Image near_map;              // (rt_vr_near.comp: one texel a 16 x 16 tile of the picture)
VkSampler samp_lin = VK_NULL_HANDLE, samp_near = VK_NULL_HANDLE;
// The interface as drawn (linear); then, in three sets used in turn, the panel (sRGB) and the two eyes (sRGB): the
// headset's thread copies a finished set while the game writes the next one (found with the real headset:
// one set alone could be copied half old, half new).
constexpr int kSets = 3;
vk::Image ui_rt, ui8[kSets], eye[kSets][2];
// The camera view for filming (rt_vr_film.comp): what the game's window shows on the desk while the headset is
// worn. vr_film = 1 or the key C (vr_key_film); vr_film_fov: its field sideways, degrees; vr_film_smooth: how
// long it takes to follow the head, seconds; vr_film_level = 1: its horizon stays level; vr_film_ui = 1: the
// interface is laid over it.
vk::Image film_img;
std::atomic<bool> g_film{false};
double g_film_fov = 90.0, g_film_smooth = 0.25;
bool g_film_level = true, g_film_ui = true;
double film_fwd[3] = {0, 0, -1}, film_up[3] = {0, 1, 0};
int64_t film_time = 0;
uint64_t st_film_frames = 0, st_film_held = 0, st_cam_pitch_held = 0;
std::atomic<bool> ui_ready{false};
std::atomic<VkImageView> ui_view_now{VK_NULL_HANDLE};
uint32_t fin_w = 0, fin_h = 0;   // the picture the game presents (learnt from its final copy)
// Two traps found at the castle town and in the plain, which the forest does not have:
//  - the game draws with its final-copy program into smaller 8-bit targets earlier in the frame (1051 x 1091 at the
//    town): the picture it presents is the largest such target ever seen;
//  - a shadow map can be a 32-bit depth target larger than the scene (4096 x 4096): the scene's depth is the one at the
//    size of the tone curve's target (the scene's size), nothing else.
uint32_t big_w = 0, big_h = 0;   // the largest target of the final-copy program so far
uint32_t scene_w = 0, scene_h = 0;   // the scene's size (its depth target's)
uint32_t tone_w = 0, tone_h = 0;     // the size of a known tone curve's target, once one was seen
uint64_t st_small_copies = 0, st_other_depths = 0;
EyeGeom frame_eyes[2];           // (the eyes this frame's pictures were made for)
uint64_t view_lag = 1;           // how many view builds behind the newest the drawn frame's view is (learnt from sure frames)
float good_proj[16] = {};        // the projection of the last frame surely drawn with the head's view
uint64_t st_guess_field = 0, st_guess_blind = 0, st_unmatched_head = 0, st_matched_later = 0;
uint64_t lag_max = 0, lag_min = 1000000, age_n = 0;
double age_sum = 0, age_max = 0, age_min = 1e9, age_win = 0, age_recent = 0;   // (milliseconds from the view built to its frame finished)
double worst_err = -1;           // (the largest "nearest view" difference of a frame the head should have driven and did not)
VkImage pic_image = VK_NULL_HANDLE;   // the game's finished picture before that copy
VkImageUsageFlags pic_usage = 0;
std::unordered_map<VkImage, VkImageView> views_c, views_d;
bool inject_now = false;
Pair pair_done, pair_out;         // made by the frame's last stop; given out once its commands are submitted
bool pair_pending = false;
uint64_t st_injects = 0, st_ui_frames = 0, st_ui_sets = 0, st_head_frames = 0, st_screen_frames = 0, st_flat_frames = 0, st_dumps = 0;
int last_kind = -1;
uint64_t st_why[6] = {}, st_why_frames = 0, st_why_logs = 0;   // (head frames not shown as such, by cause)
uint64_t st_other_tone = 0;      // (frames with a scene and none of the tone curve programs we know: still shown as scenes)
// (diagnosis: a whole frame's draws, run by run, written to the log: the frame after the first head frames shown flat,
// and one ordinary head frame to compare with)
struct TraceRun {
    uint64_t key;
    int cfmt, dfmt;
    uint32_t w, h, n, colors, writes, verts;
};
std::vector<TraceRun> trace_runs;
int trace_ask = 0, trace_asked_flat = 0;
bool trace_now = false, trace_ordinary_done = false;
bool g_trace_ordinary = false;
uint32_t why_last = 0;
// the fake headset's pictures: copied now and then into a buffer the processor reads, then written to a file
vk::Buffer dump;
uint64_t dump_at = 0, dump_frame = 0;
struct DumpHead {
    uint32_t magic, w, h, kind;          // kind: 0 flat, 1 the big screen with relief, 2 the head's own view
    uint32_t frame, ui_separate, depth_w, depth_h;
    float tan_x, tan_y, shift_px, conv;
    float proj[4];                        // P22, P23, P32, P33
    float view[12];
    double head_q[4], head_p[3];
    // (second version of the file: the field and orientation each eye's picture was made in and declared with, and the
    // headset's own for each eye, for tools/vr-casque.py, which shows what the headset's two screens would show)
    float decl_fov[2][4];
    double decl_q[2][4], decl_p[2][3];
    float true_fov[2][4];
    double true_q[2][4], true_p[2][3];
    uint32_t eye_field, pad;
} dump_head;

// The game's own picture must hold both eyes' fields, turned as the eyes are: its half field, from the eyes'. (eyes_mu held)
void fit_field() {
    double nx = 0, ny = 0;
    for (int e = 0; e < 2; ++e) {
        double r[3][3];
        quat_rows(g_eyes[e].q, r);
        const double tl = std::tan(g_eyes[e].fov[0]), tr = std::tan(g_eyes[e].fov[1]), tu = std::tan(g_eyes[e].fov[2]), td = std::tan(g_eyes[e].fov[3]);
        for (double tx : {tl, tr, 0.0})
            for (double ty : {tu, td, 0.0}) {
                const double d[3] = {r[0][0] * tx + r[0][1] * ty - r[0][2], r[1][0] * tx + r[1][1] * ty - r[1][2], r[2][0] * tx + r[2][1] * ty - r[2][2]};
                if (-d[2] < 0.05) continue;
                nx = std::max(nx, std::fabs(d[0] / -d[2])), ny = std::max(ny, std::fabs(d[1] / -d[2]));
            }
    }
    if (nx < 0.2 || ny < 0.2) return;
    nx = std::min(std::tan(std::atan(nx * 1.01) + g_field_margin), 3.7), ny = std::min(std::tan(std::atan(ny * 1.01) + g_field_margin), 3.7);
    if (std::fabs(nx - g_tan_x) > 0.004 * g_tan_x) g_tan_x = nx;
    if (std::fabs(ny - g_tan_y) > 0.004 * g_tan_y) g_tan_y = ny;
}

double opt_f(const char* key, double fallback) {
    const std::string v = opt(key, "");
    if (v.empty()) return fallback;
    char* end = nullptr;
    const double d = std::strtod(v.c_str(), &end);
    return end && end != v.c_str() ? d : fallback;
}

VkImageView view_of(std::unordered_map<VkImage, VkImageView>& cache, VkImage img, VkFormat format, VkImageAspectFlags aspect) {
    if (auto it = cache.find(img); it != cache.end()) return it->second;
    VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    vi.image = img;
    vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vi.format = format;
    vi.subresourceRange = {aspect, 0, 1, 0, 1};
    VkImageView v = VK_NULL_HANDLE;
    if (vk::vkCreateImageView(vk::device, &vi, nullptr, &v) != VK_SUCCESS) v = VK_NULL_HANDLE;
    cache[img] = v;   // (the host's targets live for the whole run; a failed one stays null)
    return v;
}

struct Binds {
    VkWriteDescriptorSet w[8]{};
    VkDescriptorImageInfo ii[8]{};
    uint32_t n = 0;
    Binds& tex(VkSampler s, VkImageView v) {
        ii[n] = {s, v, VK_IMAGE_LAYOUT_GENERAL};
        w[n] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        w[n].dstBinding = n, w[n].descriptorCount = 1, w[n].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, w[n].pImageInfo = &ii[n];
        return ++n, *this;
    }
    Binds& img(VkImageView v) {
        ii[n] = {VK_NULL_HANDLE, v, VK_IMAGE_LAYOUT_GENERAL};
        w[n] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        w[n].dstBinding = n, w[n].descriptorCount = 1, w[n].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, w[n].pImageInfo = &ii[n];
        return ++n, *this;
    }
    void run(VkCommandBuffer c, const Pipe& p, const void* pc, uint32_t bytes, uint32_t w_px, uint32_t h_px) {
        vk::vkCmdBindPipeline(c, VK_PIPELINE_BIND_POINT_COMPUTE, p.pipe);
        vk::vkCmdPushDescriptorSetKHR(c, VK_PIPELINE_BIND_POINT_COMPUTE, p.layout, 0, n, w);
        vk::vkCmdPushConstants(c, p.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, bytes, pc);
        vk::vkCmdDispatch(c, (w_px + 7) / 8, (h_px + 7) / 8, 1);
    }
};

struct WarpPc {   // (as rt_vr_warp.comp lays it out)
    float lin[4];
    float eye_tan[4];
    float rot0[4], rot1[4], rot2[4];
    float size[2];
    float shift, conv, max_px;
    uint32_t flags;
    float sight[2];   // (flag 128) where the game aims, in the picture's own frame: tangents, sideways and upwards
    float halo[4];
};
struct UiPc {
    float size[2];
    uint32_t flags, pad;
    float hole[4];    // (flag 4) a box of the interface left out of the panel: the game's own sight
};
struct FilmPc {   // (as rt_vr_film.comp lays it out)
    float rot0[4], rot1[4], rot2[4];
    float size[2];
    float field[2];
    uint32_t flags, pad[3];
};
struct NearPc {   // (as rt_vr_near.comp lays it out)
    float lin[4];
    float size[2];
    float pad[2];
    float halo[4];
};

bool make_passes() {
    VkDescriptorSetLayoutBinding b[5] = {};
    for (uint32_t i = 0; i < 3; ++i) b[i] = {i, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    b[3] = {3, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    b[4] = {4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    if (!vk::make_compute(rt_vr_warp_comp, sizeof rt_vr_warp_comp, b, 5, sizeof(WarpPc), p_warp.dsl, p_warp.layout, p_warp.pipe)) return false;
    VkDescriptorSetLayoutBinding nb[2] = {};
    nb[0] = {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    nb[1] = {1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    if (!vk::make_compute(rt_vr_near_comp, sizeof rt_vr_near_comp, nb, 2, sizeof(NearPc), p_near.dsl, p_near.layout, p_near.pipe)) return false;
    VkDescriptorSetLayoutBinding u[3] = {};
    u[0] = {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    u[1] = {1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    u[2] = {2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    if (!vk::make_compute(rt_vr_ui_comp, sizeof rt_vr_ui_comp, u, 3, sizeof(UiPc), p_ui.dsl, p_ui.layout, p_ui.pipe)) return false;
    VkDescriptorSetLayoutBinding fb[3] = {};
    fb[0] = {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    fb[1] = {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    fb[2] = {2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
    if (!vk::make_compute(rt_vr_film_comp, sizeof rt_vr_film_comp, fb, 3, sizeof(FilmPc), p_film.dsl, p_film.layout, p_film.pipe)) return false;
    VkSamplerCreateInfo si{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    si.magFilter = si.minFilter = VK_FILTER_LINEAR;
    si.addressModeU = si.addressModeV = si.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    if (vk::vkCreateSampler(vk::device, &si, nullptr, &samp_lin) != VK_SUCCESS) return false;
    si.magFilter = si.minFilter = VK_FILTER_NEAREST;
    return vk::vkCreateSampler(vk::device, &si, nullptr, &samp_near) == VK_SUCCESS;
}

// Our images at the size of the picture the game presents (mu held). Another size: new ones, the old left as they are
// (a pass in flight may still hold them; sizes change at most once in a run).
bool make_images(VkCommandBuffer c) {
    if (eye[0][0].w == fin_w && eye[0][0].h == fin_h && eye[0][0].image) return true;
    ui_ready.store(false, std::memory_order_release);
    ui_view_now.store(VK_NULL_HANDLE, std::memory_order_release);
    const VkImageUsageFlags use = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                                  VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    ui_rt = vk::Image{}, near_map = vk::Image{}, film_img = vk::Image{};
    bool ok = vk::make_image(ui_rt, fin_w, fin_h, kUiFormat, use | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
    ok = ok && vk::make_image(near_map, (fin_w + 15) / 16, (fin_h + 15) / 16, VK_FORMAT_R16_SFLOAT, use);
    ok = ok && vk::make_image(film_img, fin_w, fin_h, kPicFormat, use);
    for (int k = 0; k < kSets; ++k) {
        ui8[k] = vk::Image{}, eye[k][0] = vk::Image{}, eye[k][1] = vk::Image{};
        ok = ok && vk::make_image(ui8[k], fin_w, fin_h, VK_FORMAT_R8G8B8A8_UNORM, use);
        ok = ok && vk::make_image(eye[k][0], fin_w, fin_h, VK_FORMAT_R8G8B8A8_UNORM, use);
        ok = ok && vk::make_image(eye[k][1], fin_w, fin_h, VK_FORMAT_R8G8B8A8_UNORM, use);
    }
    if (!ok) {
        log_host("vr: the images could not be made (%u x %u): the end of the frame is left to the game", fin_w, fin_h);
        g_gpu = false;
        return false;
    }
    const VkClearColorValue zero{};
    const VkImageSubresourceRange all{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    std::vector<vk::Image*> made = {&ui_rt, &near_map};
    for (int k = 0; k < kSets; ++k) made.push_back(&ui8[k]), made.push_back(&eye[k][0]), made.push_back(&eye[k][1]);
    for (vk::Image* im : made) {
        vk::image_ready(c, *im);
        vk::vkCmdClearColorImage(c, im->image, VK_IMAGE_LAYOUT_GENERAL, &zero, 1, &all);
    }
    vk::full_barrier(c);
    log_host("vr: images made, %u x %u (interface; %d sets of panel and two eyes)", fin_w, fin_h, kSets);
    return true;
}

} // namespace

// ---- the first person: Link's body left out of the head's view, his arms kept ----
// (Link's body is taken away, his shadow and his arms stay, for his hands to follow the player's.)
// tools/vr-corps.py reads Link's models (child, adult) in this PC's game files and lists, mesh by
// mesh, the triangles whose vertices follow his arm bones. A mesh the game draws is known here by its index count and
// the first bytes of its index data. In the frames drawn with the first person's view, such a draw into the scene's
// targets keeps those triangles only; shadow maps and other targets keep the whole body (his shadow stays).
struct BodyMesh {
    uint32_t count = 0, unit = 0, model = 0, shape = 0;
    uint64_t hash = 0;
    std::vector<uint32_t> kept;   // his arms
    std::vector<uint32_t> body;   // all of him but his head (the player's eyes are in it)
};
std::vector<BodyMesh> body_meshes;
std::unordered_multimap<uint32_t, int> body_by_count;
int g_body = 1;                  // vr_body: "bras" 1 (the arms only), "entier" 0 (the whole body), "rien" 2 (nothing of Link)
const BodyMesh* body_now = nullptr;   // (the draw being made is this mesh of Link's: read back by draw_indices)
uint64_t st_body_draws = 0, st_body_whole = 0, st_body_frame = 0, st_body_last = 0, st_body_models[4] = {};
double g_fp_near_arms = 0.0;     // the near plane once the body is out of the view (0: the game's own)
// vr_cull_back: in the first person the game's camera is this many eye heights behind the player's eyes (what the game
// sorts out as "not seen" is judged from there: see view_built); vr_cull_probe = 1: study, without the hook V10
double g_cull_back = 1.0;
bool g_cull_probe = false;
std::atomic<uint64_t> st_fills{0}, st_fills_put{0}, st_pulled{0};

uint64_t fnv64(const uint8_t* p, size_t n) {
    uint64_t h = 0xcbf29ce484222325ull;
    for (size_t i = 0; i < n; ++i) h = (h ^ p[i]) * 0x100000001b3ull;
    return h;
}

void load_body() {
    body_meshes.clear(), body_by_count.clear();
    const std::string path = dll_dir() + "\\retraced\\vr-corps.bin";
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return void(log_host("vr: %s not found (tools\\vr-corps.py makes it): Link's body stays whole in the first person", path.c_str()));
    char magic[4] = {};
    uint32_t n = 0, n_models = 0;
    size_t triangles = 0, total = 0;
    if (std::fread(magic, 1, 4, f) == 4 && std::memcmp(magic, "VRC3", 4) == 0 && std::fread(&n_models, 4, 1, f) == 1 &&
        std::fread(&n, 4, 1, f) == 1 && n < 4096 && n_models <= 4 && hands::load(f, n_models)) {
        for (uint32_t i = 0; i < n; ++i) {
            uint32_t head[2] = {}, tail[4] = {};
            uint64_t hash = 0;
            if (std::fread(head, 4, 2, f) != 2 || std::fread(&hash, 8, 1, f) != 1 || std::fread(tail, 4, 4, f) != 4 || tail[2] > (1u << 24) ||
                tail[3] > (1u << 24))
                break;
            BodyMesh m;
            m.count = head[0], m.unit = head[1], m.hash = hash, m.model = tail[0], m.shape = tail[1];
            m.kept.resize(tail[2]), m.body.resize(tail[3]);
            if (tail[2] && std::fread(m.kept.data(), 4, tail[2], f) != tail[2]) break;
            if (tail[3] && std::fread(m.body.data(), 4, tail[3], f) != tail[3]) break;
            triangles += m.kept.size() / 3, total += m.count / 3;
            body_by_count.emplace(m.count, (int)body_meshes.size());
            body_meshes.push_back(std::move(m));
        }
    }
    std::fclose(f);
    if (body_meshes.empty()) log_host("vr: %s is not this build's (run tools\\vr-corps.py again): Link's body stays whole", path.c_str());
    log_host("vr: Link's body: %zu meshes known, %zu triangles of his arms kept out of %zu", body_meshes.size(), triangles, total);
}

bool body_hidden() { return g_body != 0 && !body_meshes.empty(); }

// ---- (study, vr_probe=1) the uniform blocks of Link's draws, and what is drawn near him ----
int g_probe = 0;
bool g_edge = true;              // vr_edge: a near thing's rim goes with it in the eyes' pictures (rt_vr_warp.comp, flag 64)
double g_halo_m = 0.16;          // vr_halo_cm: how far around a fairy her glow reaches (0: her glow keeps the scenery's depth)
uint64_t st_halo_frames = 0, st_halo_draws = 0;
// The fairy's fur, by the number of indices of its two levels of detail (Npc_Navi, read with tools/carte-objets.py
// --modele): the one piece of her that is drawn once, opaque, whatever the distance. Her bones: 13 matrices.
bool fairy_fur(int count) { return count == 2280 || count == 768; }
constexpr uint32_t kFairyBlock = 13 * 48;
bool g_depth_fix = false;        // vr_depth_fix: a translucent figure writes its depth in a frame seen through the headset (off: see draw())
bool g_whole = true;             // vr_whole: Link and what he holds are never faded out by the game (see uniform_seen)
uint64_t st_whole_blocks = 0, st_whole_depth = 0;
int draw_kind = 0;               // the draw being made: 0 nothing of ours, 1 a piece of Link's, 2 a rigid thing near him
int draw_model = 0;
float draw_offset[3] = {};
uint64_t st_posed_blocks = 0;
// Link's pose block as last seen (where it lies in the game's memory, its offset): what he holds is drawn before or
// after him in the frame, and needs his hands' places of that same frame
uint64_t pose_addr = 0;
uint32_t pose_n = 0;
int pose_model = 0;
float pose_offset[3] = {};
double thing_out[12] = {};
float thing_offset[3] = {};
uint64_t st_things = 0, st_bone_things = 0, thing_id = 0;
// the game's own camera at Link's eyes (he aims a thing): how far the hand points from where the game aims
bool aim_on = false;
double aim_yaw = 0, aim_pitch = 0, aim_pull = 0;
int64_t aim_time = 0;
uint64_t st_aim_views = 0, st_aim_starts = 0, st_cam_study = 0;
bool g_sticks_raw = false;       // (study) the sticks go to the game as they are
// In the third person the right stick turns the game's camera round Link, and no more: pushed up or down it takes that
// camera under him or five metres over his head, where it stays, and with the horizon kept level (vr_level) the player
// only goes up and down, as in a lift, without looking anywhere else (measured: 0.8 under his eyes to 5.3 over them).
// vr_cam_pitch = 1 lets it through, as in the game without a headset.
bool g_cam_pitch = false;
int g_sim_rstick[2] = {0, 0};    // (the fake headset) vr_sim_rx, vr_sim_ry: the right stick held there, in hundredths
int g_study = 0;                 // (study) vr_study = n: a line about the game's own camera every n views
double g_aim_field = 46.5;       // vr_aim_field: the game aims when its camera's field is narrower than this (degrees)
int g_sight = 1;                 // vr_aim_sight: a ring is drawn in the eyes where the game aims; its own sight leaves the panel
std::atomic<float> sight_world[3];   // where the game aims (world), written by the game's thread
std::atomic<int64_t> sight_time{0};
uint64_t sight_frame = ~0ull, st_sight_frames = 0;   // the frame whose eyes carry the ring
int32_t study_stick[4] = {};     // (the sticks and the buttons the game was last given)
uint64_t study_buttons = 0;
double cam_off_now[3] = {};      // (the game's camera against Link's eyes: beside, above; its field in degrees)   // (views in which the game aimed; how many times it began to)
double aim_first[2] = {}, aim_least[2] = {1e9, 1e9};   // (the hand against the game's aim: when it began, at its least since)
uint64_t draw_key = 0;
struct ProbeSeen {
    int stage;
    uint32_t bank, size;
};
std::vector<ProbeSeen> probe_seen;
std::vector<uint64_t> probe_near;
int probe_count = 0;
uint64_t probe_id = 0;
double probe_player[3] = {};

uint32_t uniform_seen(const RtUniform* u, void* out, uint32_t cap) {
    if (!u || !draw_kind) return 0;
    if (draw_kind == 6) {   // (study, vr_probe=5)
        if (u->stage == 0 && u->bank >= 5 && u->bytes && u->size >= 48 && u->size % 48 == 0) {
            float m[12];
            std::memcpy(m, u->bytes, 48);
            const double x = m[3] + draw_offset[0], y = m[7] + draw_offset[1], z = m[11] + draw_offset[2];
            const uint64_t id = (uint64_t)(uint32_t)probe_count * 1000003ull + (uint64_t)(int64_t)std::lround(x) * 7919ull + (uint64_t)(int64_t)std::lround(z);
            std::lock_guard lk(mu);
            if (std::isfinite(x) && std::find(probe_near.begin(), probe_near.end(), id) == probe_near.end() && probe_near.size() < 3000) {
                probe_near.push_back(id);
                log("vr probe 5: figure, %d indices, %u bones, at %.2f %.2f %.2f", probe_count, u->size / 48, x, y, z);
            }
        }
        return 0;
    }
    // a fairy: where she is (her bones' block: each matrix takes the model's own origin to the same point, her middle)
    if (draw_kind == 5) {
        if (u->stage == 0 && u->bank >= 5 && u->bytes && u->size == kFairyBlock) {
            float m[12];
            std::memcpy(m, u->bytes, 48);
            const float p[3] = {m[3] + draw_offset[0], m[7] + draw_offset[1], m[11] + draw_offset[2]};
            std::lock_guard lk(mu);
            if (cur.ctx_ok && std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
                float v[12];
                std::memcpy(v, cur.ctx, 48);   // (the frame's view: rows of the world-to-view matrix)
                const double z = v[8] * p[0] + v[9] * p[1] + v[10] * p[2] + v[11];
                if (z < -0.03 && (!cur.halo_ok || -z < cur.halo_d2)) cur.halo_ok = true, cur.halo_d2 = -z, std::memcpy(cur.halo, p, sizeof p);
                st_halo_draws++;
            }
        }
        return 0;
    }
    // Link, and what he holds, kept whole. The game fades its hero out when its own camera comes near him: aiming
    // upwards brings that camera within 1.5 m of him (seen: the player's arms and slingshot turned to a
    // ghost for as long as the aim was high), and so does a narrow place. In the first person the game's camera is
    // nowhere the player can see. The opacity the game gives a model is word 107 of its material block (bank 5 of the
    // vertex and the fragment stages; measured the same day: 1 for everything in the scene, 0.2 for Link's pieces and
    // his slingshot when faded): it is put back to 1 for Link's pieces and for what is in the player's hands, nothing
    // else (a ghost that fades in the game still fades). See also draw(): these draws write their depth again.
    if (g_whole && u->bank == 5 && (u->stage == 0 || u->stage == 1) && u->bytes && out && u->size >= 432 && u->size <= 4096 && u->size <= cap &&
        (draw_kind == 1 || draw_kind == 2 || (draw_kind == 4 && hands::held(thing_id)))) {
        float a;
        std::memcpy(&a, (const uint8_t*)u->bytes + 428, 4);
        // (strictly between: some materials hold 0 there at all times, a block of 640 bytes in the vertex stage among
        // them, and it is not their opacity: put to 1, Link's shield was drawn in rags.)
        if (a > 0.02f && a < 0.999f) {
            std::memcpy(out, u->bytes, u->size);
            const float one = 1.0f;
            std::memcpy((uint8_t*)out + 428, &one, 4);
            st_whole_blocks++;
            return u->size;
        }
    }
    // Link's pose block (the vertex stage's block of one 3 x 4 matrix a bone): his arms where the player's hands are
    if (draw_kind == 1 && u->stage == 0 && u->bytes && out && u->size <= cap) {
        const uint32_t n = hands::matrices(draw_model);
        if (n && u->size == n * 48) {
            hands::Frame fr;
            {
                std::lock_guard lk(mu);
                std::memcpy(fr.base, cur.given.base, sizeof fr.base), std::memcpy(fr.eye, cur.given.eye, sizeof fr.eye);
                std::memcpy(fr.head_q, cur.given.head.q, sizeof fr.head_q), std::memcpy(fr.head_p, cur.given.head.p, sizeof fr.head_p);
                fr.units = units();
            }
            pose_addr = u->guest_addr, pose_n = n, pose_model = draw_model, std::memcpy(pose_offset, draw_offset, sizeof pose_offset);
            if (hands::pose(draw_model, fr, draw_offset, (const uint8_t*)u->bytes, (uint8_t*)out, n, g_body ? 1 : 0)) {
                st_posed_blocks++;
                return u->size;
            }
        }
    }
    // a small figure with bones of its own near Link (his shield, his sword): its pose block, moved as one piece
    if (draw_kind == 4 && u->stage == 0 && u->bank >= 5 && u->bytes && out && u->size >= 48 && u->size % 48 == 0 && u->size <= 48 * 24 &&
        u->size <= cap && pose_addr) {
        float m[12];
        std::memcpy(m, u->bytes, 48);
        double pp[3];
        hands::Frame fr;
        {
            std::lock_guard lk2(fp_mu);
            std::memcpy(pp, player_pos, sizeof pp);
        }
        const double dx = m[3] + draw_offset[0] - pp[0], dy = m[7] + draw_offset[1] - pp[1], dz = m[11] + draw_offset[2] - pp[2];
        uint64_t pavail = 0;
        const uint8_t* pal = std::isfinite(dx) && dx * dx + dz * dz < 2.25 && dy > -0.5 && dy < 3.0 ? H->guest(pose_addr, &pavail) : nullptr;
        if (pal && pavail >= (uint64_t)pose_n * 48) {
            {
                std::lock_guard lk(mu);
                std::memcpy(fr.base, cur.given.base, sizeof fr.base), std::memcpy(fr.eye, cur.given.eye, sizeof fr.eye);
                std::memcpy(fr.head_q, cur.given.head.q, sizeof fr.head_q), std::memcpy(fr.head_p, cur.given.head.p, sizeof fr.head_p);
                fr.units = units();
            }
            if (hands::attached_block(pose_model, thing_id, fr, pose_offset, pal, pose_n, g_body ? 1 : 0, draw_offset, (const uint8_t*)u->bytes,
                                      (uint8_t*)out, u->size / 48)) {
                st_bone_things++;
                return u->size;
            }
        }
        return 0;
    }
    // what Link holds: its own matrix (the vertex stage's block beside the pose block), at the player's hand
    if (draw_kind == 2 && u->stage == 0 && u->bank == 4 && u->bytes && out && u->size >= 0x4c && u->size <= cap) {
        std::memcpy(out, u->bytes, u->size);
        float m[12];
        for (int i = 0; i < 12; ++i) m[i] = (float)thing_out[i];
        m[3] -= thing_offset[0], m[7] -= thing_offset[1], m[11] -= thing_offset[2];
        std::memcpy(out, m, 48);
        st_things++;
        return u->size;
    }
    if (g_probe == 2 && draw_kind == 3 && u->stage == 0 && u->bytes && u->size >= 48 && u->size % 48 == 0 && u->bank >= 5) {
        float m[12];
        std::memcpy(m, u->bytes, 48);
        double pp[3];
        {
            std::lock_guard lk2(fp_mu);
            std::memcpy(pp, player_pos, sizeof pp);
        }
        const double dx = m[3] + draw_offset[0] - pp[0], dy = m[7] + draw_offset[1] - pp[1], dz = m[11] + draw_offset[2] - pp[2];
        std::lock_guard lk(mu);
        if (std::isfinite(dx) && dx * dx + dz * dz < 2.25 && dy > -0.5 && dy < 2.5 &&
            std::find(probe_near.begin(), probe_near.end(), probe_id) == probe_near.end() && probe_near.size() < 300) {
            probe_near.push_back(probe_id);
            log("vr probe: skinned draw near Link (frame %llu): program %016llx, %d indices, bank %u with %u matrices, the first at %+.3f %+.3f %+.3f from his feet (rows %.2f %.2f %.2f | %.2f %.2f %.2f)",
                (unsigned long long)frames, (unsigned long long)u->program_key, probe_count, u->bank, u->size / 48, dx, dy, dz, m[0], m[1], m[2], m[4],
                m[5], m[6]);
        }
        return 0;
    }
    if (g_probe && draw_kind == 1) {
        std::lock_guard lk(mu);
        for (const ProbeSeen& p : probe_seen)
            if (p.stage == u->stage && p.bank == u->bank && p.size == u->size) return 0;
        if (probe_seen.size() >= 64) return 0;
        probe_seen.push_back({u->stage, u->bank, u->size});
        float f[12] = {};
        if (u->bytes && u->size >= 48) std::memcpy(f, u->bytes, 48);
        log("vr probe: Link's draw, program %016llx, stage %d, bank %u, %u bytes, source %u, guest %#llx: %.4g %.4g %.4g %.4g | %.4g %.4g %.4g %.4g | %.4g %.4g %.4g %.4g",
            (unsigned long long)u->program_key, u->stage, u->bank, u->size, u->source, (unsigned long long)u->guest_addr, f[0], f[1], f[2], f[3], f[4],
            f[5], f[6], f[7], f[8], f[9], f[10], f[11]);
        if (u->bytes && u->size >= 1024 && u->stage == 0) {
            const std::string dir = out_dir() + "\\vr";
            CreateDirectoryA(dir.c_str(), nullptr);
            char name[96];
            std::snprintf(name, sizeof name, "\\probe_bank%u_%u.bin", u->bank, u->size);
            if (FILE* f2 = std::fopen((dir + name).c_str(), "wb")) {
                std::fwrite(u->bytes, 1, u->size, f2);
                std::fclose(f2);
            }
        }
    }
    return 0;
}

const uint32_t* draw_indices(uint32_t* count) {
    const BodyMesh* m = body_now;
    if (!count) return nullptr;
    const std::vector<uint32_t>* list = !m || g_body == 2 ? nullptr : g_body == 1 ? &m->kept : &m->body;
    *count = list ? (uint32_t)list->size() : 0;
    return *count ? list->data() : nullptr;
}

bool g_effects = false;          // vr_effects
bool wanted() { return g_vr != 0; }
bool sim() { return g_vr == 2; }
bool effects() { return g_vr != 0 && g_effects; }
std::atomic<uint32_t> pub_scene_w{0}, pub_scene_h{0};   // (the scene's size, for the other modules: read without our lock)
bool scene_size(uint32_t* w, uint32_t* h) {
    *w = pub_scene_w.load(std::memory_order_relaxed), *h = pub_scene_h.load(std::memory_order_relaxed);
    return *w != 0 && *h != 0;
}

void configure() {
    if (g_configured) return;
    g_configured = true;
    t_start = now_ns();
    const std::string v = opt("vr", "0");
    g_vr = v == "sim" ? 2 : (v == "0" || v.empty() || v == "non") ? 0 : 1;
    if (!g_vr) return;
    // Our graphics effects are another project: with the VR mode on, none of
    // them is made ready, unless they are asked for in the VR mode too: vr_effects=1
    // lets them run (the world of rays, the per-pixel pictures, the light by rays, the clouds, the water), as the
    // shortcut asks for them. The headset is SDR: no HDR window in any case.
    g_effects = opt_int("vr_effects", 0) != 0;
    if (g_effects) {
        for (const char* k : {"hdr", "hdr_curve_patch"}) force_option(k, "0");
    } else {
        for (const char* k : {"rt_world", "gbuf", "hdr", "rt_air", "rt_water", "rt_sun", "rt_gi", "rt_refl", "rt_light", "rt_light_on",
                              "hdr_curve_patch", "light_terms", "light_oven"})
            force_option(k, "0");
    }
    const std::string m = opt("vr_mode", "libre");
    g_mode.store(m == "ecran" || m == "0" ? 0 : 1);
    g_level = opt_int("vr_level", 1) != 0;
    // (whole angles, in degrees; and a margin each way: the headset turns each picture to where the head is when it is
    // shown, and without a margin the edge the head turns towards has nothing to show)
    // (0 by default: the game draws its 3D scene with about 4.6 million points whatever picture is asked,
    // measured; a wider field spreads the same points thinner. The head asked ahead of time keeps the correction small.)
    const double margin = std::clamp(opt_f("vr_fov_margin", 0.0), 0.0, 20.0);
    g_tan_x = std::tan((opt_f("vr_fov_x", 108.0) * 0.5 + margin) * kPi / 180.0);
    g_tan_y = std::tan((opt_f("vr_fov_y", 110.0) * 0.5 + margin) * kPi / 180.0);
    g_predict = opt_int("vr_predict", 1) != 0;
    g_field_margin = margin * kPi / 180.0;
    g_eye_field = opt_int("vr_eye_field", 1) != 0;
    g_relief.store(std::clamp(opt_f("vr_relief", 100.0), 0.0, 200.0) * 0.01);
    g_units_auto = opt_f("vr_scale", 0.0) <= 0.0;
    g_units_per_m = opt_f("vr_units_per_m", 1.0) * 100.0 / std::clamp(g_units_auto ? 100.0 : opt_f("vr_scale", 100.0), 25.0, 400.0);
    g_ipd_m = std::clamp(opt_f("vr_ipd", 64.0), 40.0, 80.0) * 0.001;
    // (the farthest a point is moved for an eye: 240 pixels is a thing 9 cm from the eyes. It was 64 at first:
    // whatever was nearer than 40 cm was shown flat)
    g_max_px = std::clamp(opt_f("vr_shift_max", 240.0), 0.0, 480.0);
    g_screen_relief = std::clamp(opt_f("vr_screen_relief", 100.0), 0.0, 400.0) * 0.01;
    g_screen_conv = std::max(0.5, opt_f("vr_screen_conv", 6.0));
    g_ui_redirect = opt_int("vr_ui", 1) != 0;
    g_ui_ab = opt_int("vr_ui_ab", 0);
    g_show_holes = opt_int("vr_show_holes", 0) != 0;
    g_dump_every = opt_int("vr_dump_every", 0), g_dump_from = opt_int("vr_dump_from", 0);
    g_person.store(opt_int("vr_person", 1) == 3 ? 3 : 1);
    g_eye_h = opt_f("vr_eye_height", 0), g_eye_up = opt_f("vr_eye_up", 0.0) * 0.01, g_eye_child = opt_f("vr_eye_child", 1.0), g_eye_adult = opt_f("vr_eye_adult", 1.55), g_fp_forward = opt_f("vr_eye_forward", 14.0) * 0.01;
    g_turn_snap = std::clamp(opt_f("vr_turn_snap", 30.0), 0.0, 90.0), g_turn_speed = std::clamp(opt_f("vr_turn_speed", 120.0), 10.0, 720.0);
    g_move_head = opt("vr_move", "tete") != "corps";
    g_sticks_raw = opt("vr_sticks_raw", "0") == "1";
    g_cam_pitch = opt_int("vr_cam_pitch", 0) != 0;
    g_sim_rstick[0] = (int)std::clamp<int64_t>(opt_int("vr_sim_rx", 0), -100, 100), g_sim_rstick[1] = (int)std::clamp<int64_t>(opt_int("vr_sim_ry", 0), -100, 100);
    g_study = (int)opt_int("vr_study", 0);
    g_aim_field = std::clamp(opt_f("vr_aim_field", 46.5), 0.0, 90.0), g_sight = (int)opt_int("vr_aim_sight", 1);
    g_fp_near = std::clamp(opt_f("vr_near", 30.0), 5.0, 100.0) * 0.01;
    g_events_screen = opt("vr_events", "ecran") != "tete";
    g_stick_frame = (int)opt_int("vr_stick_frame", 1);
    {
        const std::string b = opt("vr_body", "bras");
        g_body = b == "entier" || b == "0" ? 0 : b == "rien" || b == "2" ? 2 : 1;
        g_fp_near_arms = std::clamp(opt_f("vr_near_arms", 0.0), 0.0, 100.0) * 0.01;
        g_cull_back = std::clamp(opt_f("vr_cull_back", 1.0), 0.0, 4.0), g_cull_probe = opt_int("vr_cull_probe", 0) != 0;
        g_film.store(opt_int("vr_film", 0) != 0), g_film_fov = std::clamp(opt_f("vr_film_fov", 90.0), 40.0, 105.0);
        g_film_smooth = std::clamp(opt_f("vr_film_smooth", 0.25), 0.0, 3.0);
        g_film_level = opt_int("vr_film_level", 1) != 0, g_film_ui = opt_int("vr_film_ui", 1) != 0;
        rec::configure();   // (the video recorder: it records the camera view)
        if (rec::wanted()) g_film.store(true);
        hands::configure();
        load_body();   // (his skeleton too: needed whatever the body's mode)
        g_trace_ordinary = opt_int("vr_trace", 0) != 0;
        g_probe = (int)opt_int("vr_probe", 0);
        g_depth_fix = opt_int("vr_depth_fix", 0) != 0;
        g_whole = opt_int("vr_whole", 1) != 0;
        g_halo_m = std::clamp(opt_f("vr_halo_cm", 16.0), 0.0, 100.0) * 0.01;
        g_edge = opt_int("vr_edge", 1) != 0;
    }
    sim_yaw = opt_f("vr_sim_yaw", 0), sim_pitch = opt_f("vr_sim_pitch", 0), sim_roll = opt_f("vr_sim_roll", 0);
    sim_pos[0] = opt_f("vr_sim_x", 0), sim_pos[1] = opt_f("vr_sim_y", 0), sim_pos[2] = opt_f("vr_sim_z", 0);
    sim_spin = opt_f("vr_sim_spin", 0), sim_sway = opt_f("vr_sim_sway", 0);
    sim_sleep_ms = opt_int("vr_sim_sleep_ms", 0);
    {   // the eyes until the headset tells its own: a Quest 3's (read from a real one), and for the fake headset
        // an optional turn outwards of each eye (vr_sim_cant, degrees), as some headsets have
        std::lock_guard lk(eyes_mu);
        const double cant = opt_f("vr_sim_cant", 0.0);
        const double d2r = kPi / 180.0;
        const float fl[4] = {(float)(-54 * d2r), (float)(40 * d2r), (float)(44 * d2r), (float)(-55 * d2r)};
        const float fr[4] = {(float)(-40 * d2r), (float)(54 * d2r), (float)(44 * d2r), (float)(-55 * d2r)};
        std::memcpy(g_eyes[0].fov, fl, sizeof fl), std::memcpy(g_eyes[1].fov, fr, sizeof fr);
        g_eyes[0].p[0] = -0.5 * g_ipd_m, g_eyes[1].p[0] = 0.5 * g_ipd_m;
        quat_axis(0, 1, 0, cant, g_eyes[0].q), quat_axis(0, 1, 0, -cant, g_eyes[1].q);
        if (g_eye_field) fit_field();
    }
    log_host("vr: %s, mode %s, field %.1f x %.1f degrees, %.3f game units a metre, our graphics effects %s",
             g_vr == 2 ? "fake headset" : "headset", g_mode.load() ? "libre" : "ecran", std::atan(g_tan_x) * 360.0 / kPi,
             std::atan(g_tan_y) * 360.0 / kPi, units(), g_effects ? "as the shortcut asks (vr_effects=1)" : "off");
}

void set_head(const Pose* head, int64_t display_time) {
    std::lock_guard lk(head_mu);
    g_head_ok = head != nullptr;
    if (head) g_head = *head, g_head_time = display_time;
}

void device_created(const RtHost* host, const RtDevice* d) {
    H = host;
    if (!g_vr || !d) return;
    const char* missing = nullptr;
    if (!vk::load_plain(d, &missing)) return void(log_host("vr: Vulkan function %s missing: no picture for the headset", missing ? missing : "?"));
    if (!make_passes()) return void(log_host("vr: our passes could not be made: no picture for the headset"));
    g_gpu = true;
    log_host("vr: passes ready (one eye from the picture and its depth; the interface apart)");
}

bool wants_ui() { return g_vr != 0; }

// Hook V6, each time the host works out a set with one colour target: is it the game's finished picture, where the
// interface is about to be drawn? Then our image takes its place for these draws (returns 1).
int ui_target(VkImage image, VkFormat format, VkImageUsageFlags usage, uint32_t w, uint32_t h, int has_depth, VkImageView* view,
              VkFormat* view_format) {
    if (!g_gpu || format != kPicFormat || has_depth || !fin_w || w != fin_w || h != fin_h) return 0;
    std::lock_guard lk(mu);
    pic_image = image, pic_usage = usage;
    // (the interface is what is drawn into the finished picture once the scene's tone curve has run: steps 228 of the
    // frame's anatomy. Without a scene, movies and menus: the picture is all there is, it stays whole.)
    // ("once the scene is drawn", not "once the tone curve we know has run": see draw(). The scene's own
    // image reaches the finished picture without a draw: every draw into the picture in a frame with a scene is the
    // interface's.)
    if (!cur.depth || !g_ui_redirect || !ui_ready.load(std::memory_order_acquire)) return 0;
    if (g_ui_ab > 0 && (frames / (uint64_t)g_ui_ab) % 2 == 1) return 0;   // (proof: every other stretch, the game's own way)
    cur.ui_redirected = true;
    cur.ui_sets++, st_ui_sets++;
    *view = ui_rt.view, *view_format = kUiFormat;
    return 1;
}

int ui_view(VkImageView v) { return v && v == ui_view_now.load(std::memory_order_relaxed) ? 1 : 0; }

// A camera block of the game: which of the views we gave is the nearest to its view? (mu held) Returns the largest
// difference between the two (1e30: we gave none), the place in the ring in *at.
double nearest_given(const uint8_t* ctx, int* at) {
    float v[12];
    std::memcpy(v, ctx, 48);
    double best = 1e30;
    *at = -1;
    for (int i = 0; i < kRing; ++i) {
        if (!ring[i].serial) continue;
        double e = 0;
        for (int k = 0; k < 12; ++k) e = std::max(e, (double)std::fabs(v[k] - ring[i].m[k]));
        if (e < best || (e == best && *at >= 0 && ring[i].serial > ring[*at].serial)) best = e, *at = i;   // (as near: the newer one)
    }
    return best;
}

namespace {
int draw_inner(const RtDraw* d);
uint64_t st_depth_forced = 0;
std::vector<uint64_t> depth_said;
}   // namespace

// The two eyes' pictures are made from one picture and its depth. A figure the game draws
// without writing its depth (the wings of a fairy, translucent) has the depth of what is behind it: in the headset it is
// painted on the wall behind, "flat": a fairy did not look like a thing in space. In a frame seen through the
// headset, such a figure (it has bones: never a flat glow, never the water) writes
// its depth: bit 4 of the answer (hook Z1 sets the draw's depth write).
//
// Off by default (vr_depth_fix=1 turns it on): a probe (vr_probe=4) showed that the fairy's wings and
// fur do write their depth; what was flat is her glow, which has no figure at all (see the halo, rt_vr_warp.comp). The
// figures this rule does catch (17 kinds in the forest, not identified) gain nothing we know of. Kept as a tool.
int draw(const RtDraw* d) {
    if (!g_vr || !d || !H) return 0;
    int r = draw_inner(d);
    // (Link's pieces and what he holds, when the game fades them out: it then draws them as glass, without writing their
    // depth. Kept whole (uniform_seen), they must write it again: the eyes' pictures are made from the depth, and arms
    // without depth would be laid on the scenery behind them.)
    if (g_whole && d->depth_test && !d->depth_write && d->depth_image &&
        (draw_kind == 1 || draw_kind == 2 || (draw_kind == 4 && hands::held(thing_id)))) {
        std::lock_guard lk(mu);
        if (d->depth_image == cur.depth) r |= 16, st_whole_depth++;
    }
    if (g_depth_fix || g_probe == 4) {
        std::lock_guard lk(mu);
        const bool figure = cur.matched && d->indexed && !d->indirect && !d->mesh && d->int_attrs && d->depth_image && d->depth_image == cur.depth &&
                            d->depth_test && !d->depth_write && d->n_colors >= 1 && scene_w && d->width == scene_w && d->height == scene_h &&
                            d->count >= 3 && d->count <= 60000;
        if (figure && g_depth_fix) r |= 16, st_depth_forced++;
        if (g_probe == 4 && cur.matched && d->indexed && d->int_attrs && scene_w && d->width == scene_w && d->height == scene_h) {
            const uint64_t id = d->program_key * 31ull + (uint64_t)(uint32_t)d->count * 4ull + (d->depth_write ? 1 : 0) + (d->depth_test ? 2 : 0);
            // (how many figures shaped like the fairy that follows Link a frame draws: are the forest's other fairies models too?)
            static uint64_t fur_frame = 0, fur_n = 0, fur_max = 0;
            if (d->count == 2280 || d->count == 768) {
                if (fur_frame != frames) fur_frame = frames, fur_n = 0;
                if (++fur_n > fur_max) fur_max = fur_n, log("vr probe 4: %llu draws of a fairy's fur (%d indices) in one frame", (unsigned long long)fur_n, d->count);
            }
            if (std::find(depth_said.begin(), depth_said.end(), id) == depth_said.end() && depth_said.size() < 1500) {
                depth_said.push_back(id);
                log("vr probe 4: figure, program %016llx, %d indices, depth test %u write %u, %u colour(s) format %d, depth %s, forced %d", (unsigned long long)d->program_key,
                    d->count, d->depth_test, d->depth_write, d->n_colors, (int)d->color0_format,
                    !d->depth_image ? "none" : d->depth_image == cur.depth ? "the scene's" : "another", figure ? 1 : 0);
            }
        }
    }
    return r;
}

namespace {
int draw_inner(const RtDraw* d) {
    const bool d32 = d->depth_format == VK_FORMAT_D32_SFLOAT || d->depth_format == VK_FORMAT_D32_SFLOAT_S8_UINT;
    std::lock_guard lk(mu);
    // The scene's targets. At first they were known by the game's tone curve program (ab3681946f93594a): a
    // frame without it was taken for a menu or a movie and shown flat on the big screen. But the game has several tone
    // curve programs: with the sun in the view it draws a00726fca31a4039 (the one with the sun's glare, after a small
    // "is the sun hidden" draw), which is what was seen at some angles in the headset. So the scene is now known
    // by what it is: a large 32-bit depth target shaped like the picture (never a square shadow map), its size checked
    // against the tone curves we know when one is seen.
    if (d->program_key == kTonemap || d->program_key == kTonemapGlare) cur.tonemap = true, tone_w = d->width, tone_h = d->height;
    const bool scene_sized = tone_w ? d->width == tone_w && d->height == tone_h
                                    : big_w && d->width <= big_w && d->height <= big_h && d->width != d->height &&
                                          std::fabs((double)d->width / d->height - (double)big_w / big_h) < 0.02;
    if (d32 && d->depth_write && d->width >= 1024 && !scene_sized) st_other_depths++;
    if (trace_now && trace_runs.size() < 4000) {
        const int cf = d->n_colors ? (int)d->color0_format : 0, df = d->depth_image ? (int)d->depth_format : 0;
        if (!trace_runs.empty() && trace_runs.back().key == d->program_key && trace_runs.back().cfmt == cf && trace_runs.back().dfmt == df &&
            trace_runs.back().w == d->width && trace_runs.back().h == d->height)
            trace_runs.back().n++, trace_runs.back().writes += d->depth_write ? 1 : 0, trace_runs.back().verts += (uint32_t)std::max(0, d->count);
        else
            trace_runs.push_back({d->program_key, cf, df, d->width, d->height, 1, d->n_colors, d->depth_write ? 1u : 0u, (uint32_t)std::max(0, d->count)});
    }
    if (d->depth_image && d->width >= 1024) {
        int i = 0;
        while (i < cur.n_depths && cur.depths[i].image != d->depth_image) ++i;
        if (i == cur.n_depths && cur.n_depths < 6)
            cur.depths[cur.n_depths++] = {d->depth_image, (int)d->depth_format, (uint32_t)d->depth_usage, d->width, d->height, 0, 0, d->program_key};
        if (i < cur.n_depths) cur.depths[i].draws++, cur.depths[i].writes += d->depth_write ? 1 : 0;
    } else if (!d->depth_image && d->width >= 1024 && d->n_colors >= 1 && d->program_key != 0xfc10fab78898d67eull &&
               d->program_key != 0xe86f9970e0324e77ull) {   // (not the interface's two programs: dozens of draws)
        cur.tail[cur.n_tail % 12] = {d->program_key, (int)d->color0_format, d->width, d->height, (uint32_t)d->count};
        cur.n_tail++;
    }
    // Any draw into the scene's depth target (prepass or lit pass, direct or not). Its vertex bank c3 is the camera
    // block for most programs, not all, and the first such draw of a frame is not always the gameplay camera's (found
    // with the real headset: at some head angles the frame was taken for another camera's and shown on the big
    // screen). So every block met is tried, a dozen a frame at most, until one is a view we gave.
    if (d32 && d->width >= 1024 && scene_sized) {
        if (d->depth_write && (uint64_t)d->width * d->height > (uint64_t)cur.w * cur.h)
            cur.depth = d->depth_image, cur.depth_format = d->depth_format, cur.depth_usage = d->depth_usage, cur.w = d->width,
            cur.h = d->height, last_scene[0] = d->width, last_scene[1] = d->height, scene_w = d->width, scene_h = d->height;
        if (d->depth_write && tone_w) pub_scene_w.store(d->width, std::memory_order_relaxed), pub_scene_h.store(d->height, std::memory_order_relaxed);
        if (!cur.matched && d->vs_c3 && d->vs_c3_size >= 0x130 && cur.n_tested < 12) {
            bool seen = false;
            for (int i = 0; i < cur.n_tested; ++i) seen = seen || cur.tested[i] == d->vs_c3;
            uint64_t avail = 0;
            if (!seen) {
                cur.tested[cur.n_tested++] = d->vs_c3;
                if (const uint8_t* c = H->guest(d->vs_c3, &avail); c && avail >= 0x130) {
                    cur.n_read++;
                    int at = -1;
                    const double err = nearest_given(c, &at);
                    float pr[16];
                    std::memcpy(pr, c + 0x70, 64);
                    const bool field = std::fabs(pr[0] * g_tan_x - 1.0) < 2e-3 && std::fabs(pr[5] * g_tan_y - 1.0) < 2e-3;
                    // (kept for the frame: a block with the headset's field before any other, then the nearest view)
                    if (!cur.ctx_ok || (field && !cur.ours_by_field) || (err < cur.best_err && (field || !cur.ours_by_field)))
                        std::memcpy(cur.ctx, c, 0x130), cur.ctx_ok = true;
                    cur.best_err = std::min(cur.best_err, err);
                    cur.ours_by_field = cur.ours_by_field || field;
                    if (at >= 0 && err < 1e-4) cur.matched = true, cur.given = ring[at], cur.match_order = cur.n_tested;
                }
            }
        }
    }
    // Link's body in a frame drawn with the first person's view: his arms' triangles only (see load_body)
    body_now = nullptr;
    if (cur.matched && cur.given.fp && d->indexed && !d->indirect && !d->mesh && d->count >= 3 && d->index_addr && scene_w &&
        d->width == scene_w && d->height == scene_h && !body_meshes.empty()) {
        const auto r = body_by_count.equal_range((uint32_t)d->count);
        if (r.first != r.second) {
            const uint32_t unit = d->index_type == 0 ? 1u : d->index_type == 1 ? 2u : 4u;
            const size_t n = std::min<size_t>((size_t)d->count * unit, 1024);
            uint64_t avail = 0;
            if (const uint8_t* ix = H->guest(d->index_addr, &avail); ix && avail >= n) {
                const uint64_t h = fnv64(ix, n);
                for (auto k = r.first; k != r.second; ++k)
                    if (body_meshes[k->second].unit == unit && body_meshes[k->second].hash == h) body_now = &body_meshes[k->second];
            }
        }
        if (body_now) {
            st_body_draws++, st_body_frame++;
            if (g_body && (body_now->kept.empty() || g_body == 2)) st_body_whole++;
            st_body_models[body_now->model & 3]++;
            draw_kind = 1, draw_key = d->program_key, draw_model = (int)(body_now->model & 3);
            // (what the game takes off world positions in this piece's blocks: beside its own matrix)
            draw_offset[0] = draw_offset[1] = draw_offset[2] = 0;
            uint64_t avail = 0;
            if (const uint8_t* shp = d->vs_c4 && d->vs_c4_size >= 0x4c ? H->guest(d->vs_c4, &avail) : nullptr; shp && avail >= 0x4c)
                std::memcpy(draw_offset, shp + 0x40, 12);
            const int blocks = g_probe || hands::matrices(draw_model) ? 8 : 0;   // (his pose block is ours to change)
            // (his arms alone, or all of him but his head: this piece's own triangles are given; a piece that is whole
            // in the list is left as it is)
            const bool same = g_body == 0 && body_now->body.size() == (size_t)d->count;
            if (same) body_now = nullptr;
            return (same ? 0 : 4) | blocks;
        }
    }
    draw_kind = 0;
    // A rigid thing drawn near Link in a first person's frame: what he holds follows the player's hand, what is
    // fastened to his body is left out with it (rt_hands.cpp tells which, once the thing has moved with one of his bones)
    if (hands::matrices(pose_model) && pose_addr && cur.matched && cur.given.fp && d->indexed && !d->indirect && !d->mesh && !d->int_attrs &&
        d->vs_c4 && d->vs_c4_size >= 0x4c && d->index_addr && scene_w && d->width == scene_w && d->height == scene_h) {
        uint64_t avail = 0, pavail = 0;
        const uint8_t* shp = H->guest(d->vs_c4, &avail);
        const uint8_t* pal = H->guest(pose_addr, &pavail);
        if (shp && avail >= 0x4c && pal && pavail >= (uint64_t)pose_n * 48) {
            float m[12], o[3];
            std::memcpy(m, shp, 48), std::memcpy(o, shp + 0x40, 12);
            double pp[3];
            {
                std::lock_guard lk2(fp_mu);
                std::memcpy(pp, player_pos, sizeof pp);
            }
            const double dx = m[3] + o[0] - pp[0], dy = m[7] + o[1] - pp[1], dz = m[11] + o[2] - pp[2];
            if (std::isfinite(dx) && dx * dx + dz * dz < 2.25 && dy > -0.5 && dy < 3.0) {
                double w[12];
                for (int i = 0; i < 12; ++i) w[i] = m[i];
                w[3] += o[0], w[7] += o[1], w[11] += o[2];
                hands::Frame fr;
                std::memcpy(fr.base, cur.given.base, sizeof fr.base), std::memcpy(fr.eye, cur.given.eye, sizeof fr.eye);
                std::memcpy(fr.head_q, cur.given.head.q, sizeof fr.head_q), std::memcpy(fr.head_p, cur.given.head.p, sizeof fr.head_p);
                fr.units = units();
                const uint64_t id = d->index_addr * 0x9e3779b97f4a7c15ull ^ (uint32_t)d->count;
                const int v = hands::attached_rigid(pose_model, id, w, fr, pose_offset, pal, pose_n, g_body ? 1 : 0, thing_out);
                if (v == 1) {
                    draw_kind = 2, std::memcpy(thing_offset, o, sizeof o);
                    return 8;
                }
                if (v == 2) return 4;   // (no indices are given for it: the draw is left out)
            }
        }
    }
    // A fairy (the one that follows Link, and any other drawn with her shape): her bones' block, which comes after this
    // call, tells where she is; her glow then takes her depth in the eyes' pictures (rt_vr_warp.comp).
    if (g_halo_m > 0.0 && cur.matched && d->indexed && !d->indirect && !d->mesh && d->int_attrs && d->depth_write && scene_w &&
        d->width == scene_w && d->height == scene_h && fairy_fur(d->count)) {
        draw_kind = 5;
        draw_offset[0] = draw_offset[1] = draw_offset[2] = 0;
        uint64_t avail = 0;
        if (const uint8_t* shp = d->vs_c4 && d->vs_c4_size >= 0x4c ? H->guest(d->vs_c4, &avail) : nullptr; shp && avail >= 0x4c)
            std::memcpy(draw_offset, shp + 0x40, 12);
        return 8;
    }
    // (study, vr_probe=5) every large figure with bones around Link, with its place: to find a character for a trial
    if (g_probe == 5 && cur.matched && d->indexed && !d->indirect && !d->mesh && d->int_attrs && d->depth_write && scene_w && d->width == scene_w &&
        d->height == scene_h && d->count >= 3000) {
        draw_kind = 6, probe_count = d->count;
        draw_offset[0] = draw_offset[1] = draw_offset[2] = 0;
        uint64_t avail = 0;
        if (const uint8_t* shp = d->vs_c4 && d->vs_c4_size >= 0x4c ? H->guest(d->vs_c4, &avail) : nullptr; shp && avail >= 0x4c)
            std::memcpy(draw_offset, shp + 0x40, 12);
        return 8;
    }
    // The same for a thing with bones of its own (Link's shield is drawn so): where it is is only known from its pose
    // block, which comes after this call
    if (g_probe != 2 && hands::matrices(pose_model) && pose_addr && cur.matched && cur.given.fp && d->indexed && !d->indirect && !d->mesh &&
        d->int_attrs && scene_w && d->width == scene_w && d->height == scene_h) {
        draw_kind = 4, thing_id = d->index_addr * 0x9e3779b97f4a7c15ull ^ (uint32_t)d->count;
        draw_offset[0] = draw_offset[1] = draw_offset[2] = 0;
        uint64_t avail = 0;
        if (const uint8_t* shp = d->vs_c4 && d->vs_c4_size >= 0x4c ? H->guest(d->vs_c4, &avail) : nullptr; shp && avail >= 0x4c)
            std::memcpy(draw_offset, shp + 0x40, 12);
        return 8;
    }
    if (g_probe == 2 && cur.matched && cur.given.fp && d->indexed && !d->indirect && !d->mesh && d->int_attrs && scene_w &&
        d->width == scene_w && d->height == scene_h && d->depth_write) {
        draw_kind = 3, draw_key = d->program_key, probe_count = d->count, probe_id = d->index_addr * 0x9e3779b97f4a7c15ull ^ (uint32_t)d->count;
        draw_offset[0] = draw_offset[1] = draw_offset[2] = 0;
        uint64_t avail = 0;
        if (const uint8_t* shp = d->vs_c4 && d->vs_c4_size >= 0x4c ? H->guest(d->vs_c4, &avail) : nullptr; shp && avail >= 0x4c)
            std::memcpy(draw_offset, shp + 0x40, 12);
        return 8;
    }
    // (study, vr_probe=3) every small rigid thing within 30 m of Link, once each, with its place in the world: to find
    // what can be taken hold of for a trial
    if (g_probe == 3 && cur.matched && d->indexed && !d->indirect && !d->mesh && !d->int_attrs && d->vs_c4 && d->vs_c4_size >= 0x4c && scene_w &&
        d->width == scene_w && d->height == scene_h && d->depth_write && d->count >= 12 && d->count <= 20000) {
        uint64_t avail = 0;
        if (const uint8_t* shp = H->guest(d->vs_c4, &avail); shp && avail >= 0x4c) {
            float m[12], o[3];
            std::memcpy(m, shp, 48), std::memcpy(o, shp + 0x40, 12);
            double pp[3];
            {
                std::lock_guard lk2(fp_mu);
                std::memcpy(pp, player_pos, sizeof pp);
            }
            const double x = m[3] + o[0], y = m[7] + o[1], z = m[11] + o[2];
            const double dx = x - pp[0], dy = y - pp[1], dz = z - pp[2];
            if (std::isfinite(dx) && dx * dx + dz * dz < 900.0 && std::fabs(dy) < 6.0) {
                const uint64_t id = (d->index_addr * 0x9e3779b97f4a7c15ull ^ (uint32_t)d->count) + (uint64_t)(int64_t)std::lround(x * 4) * 1000003ull +
                                    (uint64_t)(int64_t)std::lround(z * 4) * 7919ull;
                if (std::find(probe_near.begin(), probe_near.end(), id) == probe_near.end() && probe_near.size() < 3000) {
                    probe_near.push_back(id);
                    const double sc = std::sqrt((double)m[0] * m[0] + (double)m[4] * m[4] + (double)m[8] * m[8]);
                    log("vr probe 3: shape %016llx, %d indices, at %.3f %.3f %.3f (%.1f m from Link, %+.2f up), scale %.2f",
                        (unsigned long long)(d->index_addr * 0x9e3779b97f4a7c15ull ^ (uint32_t)d->count), d->count, x, y, z, std::sqrt(dx * dx + dz * dz), dy, sc);
                }
            }
        }
    }
    // (study) a rigid thing drawn near Link in a first person's frame: what he holds or wears
    if (g_probe && g_probe != 3 && cur.matched && cur.given.fp && d->indexed && !d->indirect && !d->mesh && !d->int_attrs && d->vs_c4 && d->vs_c4_size >= 0x4c &&
        scene_w && d->width == scene_w && d->height == scene_h && d->depth_write) {
        uint64_t avail = 0;
        if (const uint8_t* shp = H->guest(d->vs_c4, &avail); shp && avail >= 0x4c) {
            float m[12], o[3];
            std::memcpy(m, shp, 48), std::memcpy(o, shp + 0x40, 12);
            double pp[3];
            {
                std::lock_guard lk2(fp_mu);
                std::memcpy(pp, player_pos, sizeof pp);
            }
            const double dx = m[3] + o[0] - pp[0], dy = m[7] + o[1] - pp[1], dz = m[11] + o[2] - pp[2];
            if (std::isfinite(dx) && dx * dx + dz * dz < 1.0 && dy > -0.3 && dy < 2.2) {
                const uint64_t id = d->index_addr * 0x9e3779b97f4a7c15ull ^ (uint32_t)d->count;
                if (std::find(probe_near.begin(), probe_near.end(), id) == probe_near.end() && probe_near.size() < 300) {
                    probe_near.push_back(id);
                    log("vr probe: rigid draw near Link (frame %llu): program %016llx, %d indices, at %+.3f %+.3f %+.3f from his feet, matrix rows %.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f, offset %.3f %.3f %.3f",
                        (unsigned long long)frames, (unsigned long long)d->program_key, d->count, dx, dy, dz, m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9],
                        m[10], o[0], o[1], o[2]);
                }
            }
        }
    }
    static const uint64_t trace_at = (uint64_t)opt_int("vr_trace_frame", 0);   // (diagnosis: the end of that frame, draw by draw)
    if (trace_at && frames == trace_at && (cur.tonemap || d->program_key == kFinalCopy))
        log("vr trace: program %016llx, %u colour(s), format %d, usage %#x, %u x %u, depth format %d, test %u write %u, count %d",
            (unsigned long long)d->program_key, d->n_colors, (int)d->color0_format, (unsigned)d->color0_usage, d->width, d->height,
            (int)d->depth_format, d->depth_test, d->depth_write, d->count);
    // the copy into the picture the game presents: the frame is finished, our passes go right before it
    if (d->program_key == kFinalCopy && d->n_colors == 1 && d->width >= 1024 &&
        (d->color0_format == VK_FORMAT_R8G8B8A8_SRGB || d->color0_format == VK_FORMAT_R8G8B8A8_UNORM) && !cur.done) {
        if ((uint64_t)d->width * d->height > (uint64_t)big_w * big_h) big_w = d->width, big_h = d->height;
        if (d->width != big_w || d->height != big_h) {
            st_small_copies++;
            return 0;   // (a smaller copy on the way: not the picture)
        }
        cur.done = true;
        fin_w = d->width, fin_h = d->height;
        if (g_gpu) {
            inject_now = true;
            return 1;
        }
    }
    return 0;
}
}   // namespace

bool inject(VkCommandBuffer c) {
    std::lock_guard lk(mu);
    if (!inject_now) return false;
    inject_now = false;
    if (!g_gpu || !fin_w) return true;
    const bool fresh = !(eye[0][0].w == fin_w && eye[0][0].h == fin_h && eye[0][0].image);
    if (!make_images(c)) return true;
    st_injects++;
    const int set = (int)((pair_done.serial + 1) % kSets);   // (this frame's set of pictures)
    // Which view the frame was drawn with. The sure way: its camera block holds a view we gave. When no block could
    // be read (nothing but terrain and grass in sight), or when a block has the headset's own field but not exactly
    // a view we gave, the frame is still the head's: the view given as many builds ago as for the last sure frame.
    bool guessed = false;
    {
        std::lock_guard lk2(fp_mu);
        const bool head_lately = libre_time && now_ns() - libre_time < 150000000;
        if (cur.matched) {
            view_lag = given_serial - cur.given.serial;
            const double age = (now_ns() - cur.given.built) / 1e6;
            lag_max = std::max(lag_max, view_lag), lag_min = std::min(lag_min, view_lag);
            age_sum += age, age_n++, age_max = std::max(age_max, age), age_min = std::min(age_min, age);
            if (age_n % 600 == 0) age_recent = age_win / 600.0, age_win = 0;
            age_win += age;
        } else if (cur.depth && head_lately && (cur.ours_by_field || cur.n_read == 0)) {
            const uint64_t want = given_serial > view_lag ? given_serial - view_lag : 0;
            if (want && ring[want % kRing].serial == want) {
                cur.given = ring[want % kRing], cur.matched = true, guessed = true;
                if (cur.ours_by_field) st_guess_field++;
                else st_guess_blind++;
            }
        }
        if (cur.depth && head_lately && !cur.matched) st_unmatched_head++, worst_err = std::max(worst_err, cur.best_err < 1e29 ? cur.best_err : -1.0);
        if (cur.depth && !cur.tonemap) st_other_tone++;
    }
    if (cur.ctx_ok) {
        st_prepass_frames++;
        if (cur.matched) st_matched++;
        else st_unmatched++;
        if (cur.match_order > 1) st_matched_later++;
        last_matched = cur.matched, last_mismatch = cur.best_err < 1e29 ? cur.best_err : -1;
        std::memcpy(last_view, cur.ctx, 48), std::memcpy(last_proj, cur.ctx + 0x70, 64);
    }
    // the frame's projection: its own camera block's, or (no block read) the last one of a head's frame
    float frame_proj[16];
    if (cur.ctx_ok) std::memcpy(frame_proj, cur.ctx + 0x70, 64);
    else std::memcpy(frame_proj, good_proj, 64);
    if (cur.ctx_ok && cur.matched && !guessed) std::memcpy(good_proj, frame_proj, 64);
    else if (guessed && !cur.ours_by_field) std::memcpy(frame_proj, good_proj, 64);
    const VkImageView pv = pic_image && (pic_usage & VK_IMAGE_USAGE_SAMPLED_BIT) ? view_of(views_c, pic_image, kPicFormat, VK_IMAGE_ASPECT_COLOR_BIT) : VK_NULL_HANDLE;
    const bool d32 = cur.depth_format == VK_FORMAT_D32_SFLOAT;
    const VkImageView dv = cur.depth && d32 && (cur.depth_usage & VK_IMAGE_USAGE_SAMPLED_BIT) ? view_of(views_d, cur.depth, cur.depth_format, VK_IMAGE_ASPECT_DEPTH_BIT) : VK_NULL_HANDLE;
    const bool ui_sep = cur.ui_redirected && !fresh;
    int kind = 0;
    float shift = 0, conv = 0;
    if (pv) {
        float proj[16];
        std::memcpy(proj, frame_proj, 64);
        const bool scene = cur.depth && (cur.ctx_ok || guessed) && dv && std::isfinite(proj[0]) && proj[0] > 0.01f;
        const float f_px = scene ? 0.5f * (float)fin_w * proj[0] : 0.0f;   // pixels for one unit sideways at one unit away
        if (scene && cur.matched) {
            kind = 2, st_head_frames++;
            shift = f_px * (float)(0.5 * g_ipd_m * units());
        } else if (scene) {
            kind = 1, st_screen_frames++;
            shift = f_px * (float)(0.5 * g_ipd_m * units() * g_screen_relief);
            conv = (float)(1.0 / g_screen_conv);
        } else {
            st_flat_frames++;
        }
        if (cur.matched && kind != 2) {   // (a frame drawn with the head's view and not shown as one: why)
            const uint32_t why = (cur.depth ? 0u : 2u) | (cur.depth && !d32 ? 4u : 0u) |
                                 (cur.depth && !(cur.depth_usage & VK_IMAGE_USAGE_SAMPLED_BIT) ? 8u : 0u) | (cur.ctx_ok || guessed ? 0u : 16u) |
                                 (std::isfinite(proj[0]) && proj[0] > 0.01f ? 0u : 32u);
            for (int k = 0; k < 6; ++k) st_why[k] += (why >> k) & 1;
            st_why_frames++;
            if (st_why_logs < 40 && (st_why_logs < 6 || why != why_last || st_why_frames % 600 == 0)) {
                st_why_logs++;
                std::string s;
                char t[200];
                for (int i = 0; i < cur.n_depths; ++i) {
                    const auto& z = cur.depths[i];
                    std::snprintf(t, sizeof t, " [depth %d usage %#x %ux%u, %u draws, %u writing, first %016llx]", z.format, z.usage, z.w, z.h, z.draws, z.writes,
                                  (unsigned long long)z.first_key);
                    s += t;
                }
                s += " ; last draws without depth:";
                for (int i = std::max(0, cur.n_tail - 12); i < cur.n_tail; ++i) {
                    const auto& z = cur.tail[i % 12];
                    std::snprintf(t, sizeof t, " %016llx(%d %ux%u n%u)", (unsigned long long)z.key, z.format, z.w, z.h, z.count);
                    s += t;
                }
                log("vr: head frame %llu shown flat, why %#x (2 no depth, 4 depth format, 8 depth not sampled, 16 no camera block, 32 projection); scene %u x %u;%s",
                    (unsigned long long)frames, why, scene_w, scene_h, s.c_str());
            }
            why_last = why;
            if (trace_asked_flat < 2) trace_asked_flat++, trace_ask = 1;   // (the next frame, draw by draw)
        } else if (kind == 2 && !trace_ordinary_done && st_head_frames == 1500 && g_trace_ordinary) {   // (vr_trace=1)
            trace_ordinary_done = true, trace_ask = 1;
        }
        if (trace_now) {
            log("vr trace: frame %llu, kind %d, matched %d, tone curve %d, %zu runs (program, colour format, depth format, size, draws, depth writes, vertices)",
                (unsigned long long)frames, kind, cur.matched ? 1 : 0, cur.tonemap ? 1 : 0, trace_runs.size());
            std::string line;
            // (the scene's own draws, hundreds of programs into the same targets, are folded into one entry)
            size_t i = 0;
            int on_line = 0;
            while (i < trace_runs.size()) {
                size_t j = i;
                uint32_t n = 0, wr = 0;
                while (j < trace_runs.size() && trace_runs[j].cfmt == trace_runs[i].cfmt && trace_runs[j].dfmt == trace_runs[i].dfmt &&
                       trace_runs[j].w == trace_runs[i].w && trace_runs[j].h == trace_runs[i].h && trace_runs[j].dfmt != 0)
                    n += trace_runs[j].n, wr += trace_runs[j].writes, ++j;
                char t[160];
                if (j > i + 1) {
                    std::snprintf(t, sizeof t, " {%zu programs c%d d%d %ux%u n%u w%u}", j - i, trace_runs[i].cfmt, trace_runs[i].dfmt, trace_runs[i].w, trace_runs[i].h, n, wr);
                } else {
                    const TraceRun& r = trace_runs[i];
                    std::snprintf(t, sizeof t, " %016llx(c%d/%u d%d %ux%u n%u w%u v%u)", (unsigned long long)r.key, r.cfmt, r.colors, r.dfmt, r.w, r.h, r.n, r.writes, r.verts);
                    j = i + 1;
                }
                line += t;
                if (++on_line == 9) log("vr trace:%s", line.c_str()), line.clear(), on_line = 0;
                i = j;
            }
            if (!line.empty()) log("vr trace:%s", line.c_str());
        }
        vk::full_barrier(c);
        if (kind == 2) {
            // each eye's picture in the field and orientation the headset gives for that eye (or, the old way, in the
            // game picture's own field, the same each side)
            std::lock_guard lk3(eyes_mu);
            for (int e = 0; e < 2; ++e) {
                frame_eyes[e] = g_eyes[e];
                if (!g_eye_field) {
                    frame_eyes[e] = EyeGeom{};
                    frame_eyes[e].p[0] = (e ? 0.5 : -0.5) * g_ipd_m;
                    const float ax = std::atan(1.0f / proj[0]), ay = std::atan(1.0f / proj[5]);
                    frame_eyes[e].fov[0] = -ax, frame_eyes[e].fov[1] = ax, frame_eyes[e].fov[2] = ay, frame_eyes[e].fov[3] = -ay;
                }
            }
            shift = f_px * (float)(0.5 * g_ipd_m * units() * relief());
        }
        // the nearest fairy's glow, in the picture: where her middle is, how wide the glow is there, 1 / her distance
        float halo[4] = {0, 0, 0, 0};
        if (kind == 2 && cur.halo_ok && cur.ctx_ok && g_halo_m > 0.0) {
            float v[12];
            std::memcpy(v, cur.ctx, 48);
            const float* p = cur.halo;
            const double xv = v[0] * p[0] + v[1] * p[1] + v[2] * p[2] + v[3], yv = v[4] * p[0] + v[5] * p[1] + v[6] * p[2] + v[7];
            const double zv = -(v[8] * p[0] + v[9] * p[1] + v[10] * p[2] + v[11]);
            if (zv > 0.03) {
                halo[0] = (float)((proj[0] * xv / zv * 0.5 + 0.5) * fin_w), halo[1] = (float)((0.5 - proj[5] * yv / zv * 0.5) * fin_h);
                halo[2] = (float)(g_halo_m * units() * f_px / zv), halo[3] = (float)(1.0 / zv);
                if (halo[2] >= 2.0f && std::isfinite(halo[0]) && std::isfinite(halo[1])) st_halo_frames++;
                else halo[2] = 0;
            }
        }
        // where the game aims, in this picture: a ring is drawn there in each eye (the game's own sight sits in the middle
        // of the interface's panel, wherever the hand points)
        float sight[2] = {0, 0};
        bool sight_on = false;
        if (kind == 2 && cur.ctx_ok && g_sight && g_eye_field && now_ns() - sight_time.load(std::memory_order_relaxed) < 250000000) {
            float v[12];
            std::memcpy(v, cur.ctx, 48);
            const double gx = sight_world[0].load(std::memory_order_relaxed), gy = sight_world[1].load(std::memory_order_relaxed),
                         gz = sight_world[2].load(std::memory_order_relaxed);
            const double sx = v[0] * gx + v[1] * gy + v[2] * gz, sy = v[4] * gx + v[5] * gy + v[6] * gz, sz = -(v[8] * gx + v[9] * gy + v[10] * gz);
            if (sz > 0.05) sight[0] = (float)(sx / sz), sight[1] = (float)(sy / sz), sight_on = true, sight_frame = frames, st_sight_frames++;
        }
        const bool near_ok = kind != 0 && dv && near_map.view;
        if (near_ok) {   // (how near the scene comes, tile by tile: the eyes search no farther than that)
            NearPc np{};
            std::memcpy(np.halo, halo, sizeof halo);
            np.lin[0] = proj[10], np.lin[1] = proj[11], np.lin[2] = proj[14], np.lin[3] = proj[15];
            np.size[0] = (float)fin_w, np.size[1] = (float)fin_h;
            Binds nb;
            nb.tex(samp_near, dv).img(near_map.view);
            nb.run(c, p_near, &np, sizeof np, near_map.w, near_map.h);
            vk::full_barrier(c);
        }
        for (int e = 0; e < 2; ++e) {
            WarpPc pc{};
            pc.lin[0] = proj[10], pc.lin[1] = proj[11], pc.lin[2] = proj[14], pc.lin[3] = proj[15];
            pc.size[0] = (float)fin_w, pc.size[1] = (float)fin_h;
            pc.shift = e == 0 ? shift : -shift;   // (the left eye sees everything moved to the right)
            pc.conv = conv, pc.max_px = (float)g_max_px;
            // the head's own view keeps the interface for its panel; on the screen it is laid over, at the screen's depth
            pc.flags = (kind != 2 && ui_sep ? 1u : 0u) | (g_show_holes ? 2u : 0u) | (kind == 0 ? 4u : 0u) | (near_ok ? 16u : 0u) |
                       (halo[2] > 0 ? 32u : 0u) | (g_edge && kind != 0 ? 64u : 0u);
            std::memcpy(pc.halo, halo, sizeof halo);
            if (sight_on) pc.flags |= 128u, pc.sight[0] = sight[0], pc.sight[1] = sight[1];
            if (kind == 2) {
                const EyeGeom& g = frame_eyes[e];
                pc.shift = f_px * (float)(-g.p[0] * units() * relief());   // (the eye to the left sees everything moved to the right)
                if (g_eye_field) {
                    double r[3][3];
                    quat_rows(g.q, r);
                    for (int k = 0; k < 4; ++k) pc.eye_tan[k] = std::tan(g.fov[k]);
                    for (int k = 0; k < 3; ++k) pc.rot0[k] = (float)r[0][k], pc.rot1[k] = (float)r[1][k], pc.rot2[k] = (float)r[2][k];
                    pc.rot0[3] = 1.0f / proj[0], pc.rot1[3] = 1.0f / proj[5];
                    pc.flags |= 8u;
                }
            }
            Binds b;
            b.tex(samp_lin, pv).tex(samp_near, dv ? dv : pv).tex(samp_lin, ui_rt.view).img(eye[set][e].view).tex(samp_near, near_map.view);
            b.run(c, p_warp, &pc, sizeof pc, fin_w, fin_h);
        }
        vk::full_barrier(c);
    }
    // The camera view for filming, into the picture the game presents (the game's window on the desk): see
    // rt_vr_film.comp. The camera looks where the head looked a moment ago (it follows smoothly), level; when the head
    // has turned so fast that the camera's view would leave the picture, it is brought back towards the head's.
    bool filmed = false;
    if ((g_film.load(std::memory_order_relaxed) || rec::wanted()) && kind == 2 && pv && (pic_usage & VK_IMAGE_USAGE_STORAGE_BIT) && film_img.view && frame_proj[0] > 0 && frame_proj[5] > 0) {
        double hr[3][3];   // (the head's axes in the play space: columns)
        quat_rows(cur.given.head.q, hr);
        const double hf[3] = {-hr[0][2], -hr[1][2], -hr[2][2]}, hu[3] = {hr[0][1], hr[1][1], hr[2][1]};
        const int64_t tn = now_ns();
        const double dt = film_time ? std::clamp((tn - film_time) / 1e9, 0.0, 0.25) : 1.0;
        const double a = g_film_smooth > 1e-3 ? 1.0 - std::exp(-dt / g_film_smooth) : 1.0;
        film_time = tn;
        auto unit3 = [](double v[3]) {
            const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (n > 1e-9) v[0] /= n, v[1] /= n, v[2] /= n;
            return n > 1e-9;
        };
        for (int i = 0; i < 3; ++i) film_fwd[i] += (hf[i] - film_fwd[i]) * a, film_up[i] += ((g_film_level ? (i == 1 ? 1.0 : 0.0) : hu[i]) - film_up[i]) * a;
        if (!unit3(film_fwd)) film_fwd[0] = hf[0], film_fwd[1] = hf[1], film_fwd[2] = hf[2];
        const double tx = std::tan(g_film_fov * kPi / 360.0), ty = tx * 9.0 / 16.0, ptx = 1.0 / frame_proj[0], pty = 1.0 / frame_proj[5];
        double rot[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int step = 0; step <= 6; ++step) {
            const double w = step / 6.0;
            double f[3], u[3], r[3], b3[3];
            for (int i = 0; i < 3; ++i) f[i] = film_fwd[i] + (hf[i] - film_fwd[i]) * w, u[i] = film_up[i] + (hu[i] - film_up[i]) * w;
            if (!unit3(f)) continue;
            b3[0] = -f[0], b3[1] = -f[1], b3[2] = -f[2];
            r[0] = u[1] * b3[2] - u[2] * b3[1], r[1] = u[2] * b3[0] - u[0] * b3[2], r[2] = u[0] * b3[1] - u[1] * b3[0];   // right = up x back
            if (!unit3(r)) continue;
            u[0] = b3[1] * r[2] - b3[2] * r[1], u[1] = b3[2] * r[0] - b3[0] * r[2], u[2] = b3[0] * r[1] - b3[1] * r[0];   // up = back x right
            // the camera's axes seen from the head: R = H^T F (F's columns: right, up, back)
            double m[3][3];
            for (int i = 0; i < 3; ++i) {
                m[i][0] = hr[0][i] * r[0] + hr[1][i] * r[1] + hr[2][i] * r[2];
                m[i][1] = hr[0][i] * u[0] + hr[1][i] * u[1] + hr[2][i] * u[2];
                m[i][2] = hr[0][i] * b3[0] + hr[1][i] * b3[1] + hr[2][i] * b3[2];
            }
            bool inside = true;
            for (int k = 0; k < 4 && inside; ++k) {
                const double de[3] = {(k & 1 ? tx : -tx), (k & 2 ? ty : -ty), -1.0};
                const double dx = m[0][0] * de[0] + m[0][1] * de[1] + m[0][2] * de[2], dy = m[1][0] * de[0] + m[1][1] * de[1] + m[1][2] * de[2],
                             dz = m[2][0] * de[0] + m[2][1] * de[1] + m[2][2] * de[2];
                inside = dz < -0.05 && std::fabs(dx / -dz) < ptx * 0.995 && std::fabs(dy / -dz) < pty * 0.995;
            }
            if (inside || step == 6) {
                if (inside) std::memcpy(rot, m, sizeof rot);
                if (step) st_film_held++;
                // (brought back: the camera is where it was shown, so that it does not jump back the next frame)
                if (step) for (int i = 0; i < 3; ++i) film_fwd[i] = f[i], film_up[i] = u[i];
                break;
            }
        }
        FilmPc fp{};
        for (int k = 0; k < 3; ++k) fp.rot0[k] = (float)rot[0][k], fp.rot1[k] = (float)rot[1][k], fp.rot2[k] = (float)rot[2][k];
        fp.rot0[3] = (float)ptx, fp.rot1[3] = (float)pty;
        fp.size[0] = (float)fin_w, fp.size[1] = (float)fin_h;
        fp.field[0] = (float)tx, fp.field[1] = (float)ty;
        fp.flags = ui_sep && g_film_ui ? 1u : 0u;
        Binds fb;
        fb.tex(samp_lin, pv).tex(samp_lin, ui_rt.view).img(film_img.view);
        fb.run(c, p_film, &fp, sizeof fp, fin_w, fin_h);
        vk::full_barrier(c);
        fp.flags = 2u;
        Binds cb;
        cb.tex(samp_near, film_img.view).tex(samp_lin, ui_rt.view).img(pv);
        cb.run(c, p_film, &fp, sizeof fp, fin_w, fin_h);
        vk::full_barrier(c);
        filmed = true, st_film_frames++;
    } else {
        film_time = 0;
    }
    const VkClearColorValue zero{};
    const VkImageSubresourceRange all{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    if (ui_sep && pv) {
        st_ui_frames++;
        UiPc pc{};
        pc.size[0] = (float)fin_w, pc.size[1] = (float)fin_h;
        pc.flags = 2u | ((pic_usage & VK_IMAGE_USAGE_STORAGE_BIT) && !filmed ? 1u : 0u);   // (the camera view has the interface already)
        Binds b;
        b.tex(samp_near, ui_rt.view).img((pic_usage & VK_IMAGE_USAGE_STORAGE_BIT) ? pv : ui8[set].view).img(ui8[set].view);
        if (!(pic_usage & VK_IMAGE_USAGE_STORAGE_BIT)) pc.flags = 2u;
        if (sight_frame == frames) {   // (the game's own sight, measured in its interface: 0.467 to 0.533 across, 0.435 to 0.537 down)
            pc.flags |= 4u;
            pc.hole[0] = 0.455f * fin_w, pc.hole[1] = 0.42f * fin_h, pc.hole[2] = 0.545f * fin_w, pc.hole[3] = 0.55f * fin_h;
        }
        b.run(c, p_ui, &pc, sizeof pc, fin_w, fin_h);
        vk::full_barrier(c);
        vk::vkCmdClearColorImage(c, ui_rt.image, VK_IMAGE_LAYOUT_GENERAL, &zero, 1, &all);   // (empty for the next frame)
    } else if (!fresh) {
        vk::vkCmdClearColorImage(c, ui8[set].image, VK_IMAGE_LAYOUT_GENERAL, &zero, 1, &all);
        if (cur.ui_redirected) vk::vkCmdClearColorImage(c, ui_rt.image, VK_IMAGE_LAYOUT_GENERAL, &zero, 1, &all);
    }
    vk::full_barrier(c);
    last_kind = pv ? kind : -1;
    // the video recorder (rt_rec.cpp): the picture the game's window is about to show, with the interface
    if (pv && (rec::wanted() || rec::on())) rec::frame(c, pv);
    // the fake headset's pictures, now and then: the two eyes, the panel, the picture and the depth
    if (g_dump_every > 0 && pv && !dump_at && frames % (uint64_t)g_dump_every == 0 && frames >= (uint64_t)std::max(g_dump_every, g_dump_from)) {
        const VkDeviceSize px = (VkDeviceSize)fin_w * fin_h, dpx = (VkDeviceSize)cur.w * cur.h;
        const bool with_depth = dv && (cur.depth_usage & VK_IMAGE_USAGE_TRANSFER_SRC_BIT);
        const VkDeviceSize need = px * 16 + (with_depth ? dpx * 4 : 0);
        if (dump.size < need) {
            dump = vk::Buffer{};   // (a larger one: the old is left, once in a run)
            if (!vk::make_buffer(dump, need, VK_BUFFER_USAGE_TRANSFER_DST_BIT, true)) dump = vk::Buffer{};
        }
        if (dump.mapped) {
            VkBufferImageCopy r{};
            r.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
            r.imageExtent = {fin_w, fin_h, 1};
            VkImage srcs[4] = {eye[set][0].image, eye[set][1].image, ui8[set].image, pic_image};
            for (int i = 0; i < 4; ++i) {
                if (i == 3 && !(pic_usage & VK_IMAGE_USAGE_TRANSFER_SRC_BIT)) continue;
                r.bufferOffset = px * 4 * i;
                vk::vkCmdCopyImageToBuffer(c, srcs[i], VK_IMAGE_LAYOUT_GENERAL, dump.buffer, 1, &r);
            }
            if (with_depth) {
                VkBufferImageCopy z{};
                z.bufferOffset = px * 16;
                z.imageSubresource = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 0, 1};
                z.imageExtent = {cur.w, cur.h, 1};
                vk::vkCmdCopyImageToBuffer(c, cur.depth, VK_IMAGE_LAYOUT_GENERAL, dump.buffer, 1, &z);
            }
            vk::full_barrier(c);
            DumpHead& h = dump_head;
            h = DumpHead{};
            h.magic = 0x32525656u;   // "VVR2"
            h.eye_field = kind == 2 && g_eye_field ? 1 : 0;
            {
                std::lock_guard lk3(eyes_mu);
                for (int e = 0; e < 2; ++e) {
                    std::memcpy(h.decl_fov[e], frame_eyes[e].fov, 16), std::memcpy(h.decl_q[e], frame_eyes[e].q, 32), std::memcpy(h.decl_p[e], frame_eyes[e].p, 24);
                    std::memcpy(h.true_fov[e], g_eyes[e].fov, 16), std::memcpy(h.true_q[e], g_eyes[e].q, 32), std::memcpy(h.true_p[e], g_eyes[e].p, 24);
                }
            }
            h.w = fin_w, h.h = fin_h, h.kind = (uint32_t)kind, h.frame = (uint32_t)frames, h.ui_separate = ui_sep ? 1 : 0;
            h.depth_w = with_depth ? cur.w : 0, h.depth_h = with_depth ? cur.h : 0;
            float proj[16];
            std::memcpy(proj, frame_proj, 64);
            h.tan_x = proj[0] != 0 ? 1.0f / proj[0] : 0, h.tan_y = proj[5] != 0 ? 1.0f / proj[5] : 0;
            h.shift_px = shift, h.conv = conv;
            h.proj[0] = proj[10], h.proj[1] = proj[11], h.proj[2] = proj[14], h.proj[3] = proj[15];
            std::memcpy(h.view, cur.ctx, 48);
            std::memcpy(h.head_q, cur.given.head.q, sizeof h.head_q), std::memcpy(h.head_p, cur.given.head.p, sizeof h.head_p);
            dump_at = frames, dump_frame = frames;
        }
    }
    if (pv && !fresh) {
        float proj[16];
        std::memcpy(proj, frame_proj, 64);
        pair_done.serial++;
        pair_done.kind = kind, pair_done.head = cur.given.head;
        pair_done.tan_x = kind == 2 && proj[0] > 0 ? 1.0f / proj[0] : 1.0f, pair_done.tan_y = kind == 2 && proj[5] > 0 ? 1.0f / proj[5] : 1.0f;
        pair_done.w = fin_w, pair_done.h = fin_h;
        pair_done.eye[0] = eye[set][0].image, pair_done.eye[1] = eye[set][1].image, pair_done.ui = ui8[set].image;
        pair_done.ui_separate = ui_sep;
        pair_done.eg[0] = frame_eyes[0], pair_done.eg[1] = frame_eyes[1];
        pair_pending = true;
    }
    if (fresh) {   // (ready for the next frame's interface)
        ui_view_now.store(ui_rt.view, std::memory_order_release);
        ui_ready.store(true, std::memory_order_release);
    }
    return true;
}

// Start of a recording of the host: the commands of the stop before are all submitted, its pictures may be taken.
void recording() {
    if (!g_vr) return;
    std::lock_guard lk(mu);
    if (pair_pending) pair_out = pair_done, pair_pending = false;
}

bool pair_now(Pair* out) {
    std::lock_guard lk(mu);
    if (!pair_out.serial) return false;
    *out = pair_out;
    return true;
}

void set_ipd(double metres) { g_ipd_m = metres; }
void set_eyes(const EyeGeom eyes[2]) {
    std::lock_guard lk(eyes_mu);
    g_eyes[0] = eyes[0], g_eyes[1] = eyes[1];
    const double dx = eyes[1].p[0] - eyes[0].p[0], dy = eyes[1].p[1] - eyes[0].p[1], dz = eyes[1].p[2] - eyes[0].p[2];
    const double gap = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (gap > 0.04 && gap < 0.08) g_ipd_m = gap;
    if (g_eye_field) fit_field();
}
double relief() { return g_relief.load(std::memory_order_relaxed); }
void set_relief(double r) {
    g_relief.store(std::clamp(r, 0.0, 2.0));
    log_host("vr: relief %.0f %%", std::clamp(r, 0.0, 2.0) * 100.0);
}
double ipd_m() { return g_ipd_m; }
int mode() { return g_mode.load(std::memory_order_relaxed); }
void set_mode(int m) {
    g_mode.store(m ? 1 : 0);
    log_host("vr: mode %s", m ? "libre (the head drives the camera)" : "ecran (the game on a big screen)");
}
int person() { return g_person.load(std::memory_order_relaxed); }
bool film() { return g_film.load(std::memory_order_relaxed); }
void set_film(bool on) {
    g_film.store(on);
    log_host("vr: the camera view for filming is %s (the game's window on the desk)", on ? "on" : "off");
}
void set_person(int p) {
    g_person.store(p == 3 ? 3 : 1);
    log_host("vr: %s person", p == 3 ? "third" : "first");
}

int live_get(int id) {
    if (id == 13) return person() == 1 ? 1 : 0;
    if (id == 14) return mode() == 0 ? 1 : 0;
    if (id == 15) return g_turn_snap > 0 ? 1 : 0;
    if (id == 16) return g_body == 0 ? 1 : 0;
    if (id == 17) return hands::gestures_on() ? 1 : 0;
    if (id == 20) return hands::arms_on() ? 1 : 0;
    if (id == 18) return hands::tilt_degrees();
    if (id == 19) return hands::arm_cm();
    return 0;
}
void live_set(int id, int value) {
    if (!g_vr) return;
    if (id == 13) set_person(value ? 1 : 3), save_menu_option("vr_person", value ? 1 : 3);
    if (id == 14) set_mode(value ? 0 : 1), save_menu_option("vr_mode", value ? 0 : 1);
    if (id == 16) g_body = value ? 0 : 1, save_menu_option("vr_body", value ? 0 : 1), log_host("vr: Link's body %s", value ? "whole" : "his arms alone");
    if (id == 17) hands::set_gestures(value != 0), save_menu_option("vr_gestures", value ? 1 : 0);
    if (id == 20) hands::set_arms(value != 0), save_menu_option("vr_arms", value ? 1 : 0);
    if (id == 18) hands::set_tilt(value), save_menu_option("vr_hand_tilt", value);
    if (id == 19) hands::set_arm_cm(value), save_menu_option("vr_arm_cm", value);
    if (id == 15) {
        std::lock_guard lk(fp_mu);
        g_turn_snap = value ? 30.0 : 0.0;
        save_menu_option("vr_turn_snap", value ? 30 : 0);
    }
}

void pad_hook(uint64_t* buttons, int32_t stick[4]) {
    if (!g_vr || !buttons || !stick) return;
    uint64_t xb = 0;
    int32_t xs[4] = {};
    if (g_vr == 1 && xr::pad(&xb, xs)) {
        *buttons |= xb;
        for (int i = 0; i < 4; ++i)
            if (std::abs(xs[i]) > std::abs(stick[i])) stick[i] = xs[i];
    }
    if (g_vr != 1 && (g_sim_rstick[0] || g_sim_rstick[1])) stick[2] = g_sim_rstick[0] * 327, stick[3] = g_sim_rstick[1] * 327;
    std::lock_guard lk(fp_mu);
    const int64_t now = now_ns();
    const double dt = pad_time ? std::min((now - pad_time) / 1e9, 0.05) : 0.0;
    pad_time = now;
    std::memcpy(study_stick, stick, sizeof study_stick), study_buttons = *buttons;
    if (now - libre_time > 300000000) return;   // (the head drove no view these last frames: the sticks are the game's)
    bool aim_stick = false;
    {   // the player's movements as commands: a strike, the shield, taking and throwing, aiming with the hand
        hands::Moment mo;
        Pose hp;
        int64_t ht = 0;
        if (head_now(hp, ht)) std::memcpy(mo.head_q, hp.q, sizeof hp.q), std::memcpy(mo.head_p, hp.p, sizeof hp.p);
        mo.first_person = fp_now;
        mo.game_aims = aim_on && now - aim_time < 200000000;
        mo.aim_yaw = aim_yaw, mo.aim_pitch = aim_pitch, mo.pull = aim_pull;
        hands::gestures(mo, buttons, stick);
        for (int k = 0; k < 2; ++k)
            if (mo.shake[k] > 0.02f && g_vr == 1) xr::rumble(k, mo.shake[k], mo.shake[k]);
        aim_stick = mo.sticks_taken;   // (the right stick steers the game's aim: it is not a turn of the player)
    }
    if (g_sticks_raw) return;
    if (fp_now && !aim_stick) {
        // the right stick turns the player (by steps, or smoothly), not the game's camera
        const double x = stick[2] / 32767.0;
        if (g_turn_snap > 0) {
            if (std::fabs(x) > 0.6 && !turn_latch) turn_latch = true, fp_yaw -= (x > 0 ? 1.0 : -1.0) * g_turn_snap * kPi / 180.0, st_pad_turns++;
            if (std::fabs(x) < 0.3) turn_latch = false;
        } else if (std::fabs(x) > 0.24) {
            fp_yaw -= x * g_turn_speed * kPi / 180.0 * dt;
        }
        stick[2] = 0, stick[3] = 0;
    } else if (!fp_now && !aim_stick && g_level && !g_cam_pitch) {
        if (stick[3]) st_cam_pitch_held++;
        stick[3] = 0;   // (the third person: the right stick turns the game's camera, it does not raise or lower it)
    }
    // the left stick: "forward" must run along want_yaw (where the head looks in the first person; the game camera's own
    // heading in the third, where a turned head would otherwise make the game's camera chase Link round and round),
    // and the game takes the stick against the view we wrote, or against its own camera
    const double d = want_yaw - (g_stick_frame == 1 ? written_yaw : cam_yaw_now);
    const double c = std::cos(d), sn = std::sin(d), lx = stick[0], ly = stick[1];
    stick[0] = (int32_t)std::lround(std::clamp(lx * c - ly * sn, -32767.0, 32767.0));
    stick[1] = (int32_t)std::lround(std::clamp(lx * sn + ly * c, -32767.0, 32767.0));
}

void frame(uint64_t number) {
    if (!g_vr) return;
    {
        std::lock_guard lk(mu);
        frames = number;
        cur = Cur{};
        st_body_last = st_body_frame, st_body_frame = 0;
        hands::latch();
        trace_now = trace_ask > 0;
        if (trace_now) trace_ask--, trace_runs.clear();
        // (the copy was recorded a few frames ago: its commands have run; the file is written off the replay thread)
        if (dump_at && frames >= dump_at + 4 && dump.mapped) {
            const DumpHead h = dump_head;
            const size_t bytes = (size_t)h.w * h.h * 16 + (size_t)h.depth_w * h.depth_h * 4;
            std::string data((const char*)dump.mapped, bytes);
            dump_at = 0;
            st_dumps++;
            std::thread([h, data = std::move(data)] {
                const std::string dir = out_dir() + "\\vr";
                CreateDirectoryA(dir.c_str(), nullptr);
                char name[64];
                std::snprintf(name, sizeof name, "\\vr_%06u.bin", h.frame);
                if (FILE* f = std::fopen((dir + name).c_str(), "wb")) {
                    std::fwrite(&h, sizeof h, 1, f);
                    std::fwrite(data.data(), 1, data.size(), f);
                    std::fclose(f);
                    log("vr: pictures of frame %u written (%s)", h.frame, name + 1);
                }
            }).detach();
        }
    }
    if (sim_sleep_ms > 0) Sleep((DWORD)sim_sleep_ms);
}

std::string report_json() {
    if (!g_vr) return "{\"on\": false}";
    std::lock_guard lk(mu);
    char b[4200];
    const double fx = last_proj[0], fy = last_proj[5];
    std::snprintf(b, sizeof b,
                  "{\"on\": true, \"sim\": %s, \"mode\": \"%s\", \"picture_asked\": [%u, %u], \"picture\": [%u, %u], \"scene\": [%u, %u],\n"
                  "    \"views_given\": %llu, \"views_with_camera\": %llu, \"frames_with_scene\": %llu, \"frames_matched\": %llu, \"frames_unmatched\": %llu,\n"
                  "    \"last_matched\": %s, \"last_mismatch\": %.6g,\n"
                  "    \"game_camera\": {\"near\": %.6g, \"far\": %.6g, \"field_degrees\": %.4f, \"ratio\": %.5f, \"offset\": [%.5g, %.5g], \"eye\": [%.3f, %.3f, %.3f]},\n"
                  "    \"drawn\": {\"proj_x\": %.6f, \"proj_y\": %.6f, \"half_field_x_degrees\": %.3f, \"half_field_y_degrees\": %.3f, \"proj_22\": %.7g, \"proj_23\": %.7g, \"proj_32\": %.7g, \"proj_33\": %.7g,\n"
                  "      \"view\": [%.6f, %.6f, %.6f, %.4f, %.6f, %.6f, %.6f, %.4f, %.6f, %.6f, %.6f, %.4f]},\n"
                  "    \"camera_speed\": {\"now\": %.4f, \"max\": %.4f}, \"sleep_ms\": %lld,\n"
                  "    \"end_of_frame\": {\"ready\": %s, \"stops\": %llu, \"last_kind\": %d, \"head_frames\": %llu, \"screen_frames\": %llu, \"flat_frames\": %llu,\n"
                  "      \"interface_frames\": %llu, \"interface_target_sets\": %llu, \"picture_usage\": %u, \"dumps\": %llu,\n"
                  "      \"smaller_copies_left\": %llu, \"other_depth_targets_left\": %llu,\n"
                  "      \"view_found_after_another_block\": %llu, \"view_taken_by_field\": %llu, \"view_taken_blind\": %llu,\n"
                  "      \"head_frames_lost\": %llu, \"lost_worst_difference\": %.6g, \"view_lag\": [%llu, %llu, %llu],\n"
                  "      \"view_to_picture_ms\": {\"mean\": %.2f, \"min\": %.2f, \"max\": %.2f, \"recent\": %.2f}}}",
                  g_vr == 2 ? "true" : "false", g_mode.load() ? "libre" : "ecran", g_w, g_h, fin_w, fin_h, last_scene[0], last_scene[1],
                  (unsigned long long)st_views.load(), (unsigned long long)st_views_cam.load(), (unsigned long long)st_prepass_frames.load(),
                  (unsigned long long)st_matched.load(), (unsigned long long)st_unmatched.load(), last_matched ? "true" : "false", last_mismatch,
                  cam_seen[0], cam_seen[1], cam_seen[2] * 180.0 / kPi, cam_seen[3], cam_seen[4], cam_seen[5], eye_seen[0], eye_seen[1], eye_seen[2],
                  fx, fy, fx != 0 ? std::atan(1.0 / fx) * 180.0 / kPi : 0.0, fy != 0 ? std::atan(1.0 / fy) * 180.0 / kPi : 0.0,
                  last_proj[10], last_proj[11], last_proj[14], last_proj[15],
                  last_view[0], last_view[1], last_view[2], last_view[3], last_view[4], last_view[5], last_view[6], last_view[7],
                  last_view[8], last_view[9], last_view[10], last_view[11], speed_now, speed_max, (long long)sim_sleep_ms,
                  g_gpu ? "true" : "false", (unsigned long long)st_injects, last_kind, (unsigned long long)st_head_frames,
                  (unsigned long long)st_screen_frames, (unsigned long long)st_flat_frames, (unsigned long long)st_ui_frames,
                  (unsigned long long)st_ui_sets, (unsigned)pic_usage, (unsigned long long)st_dumps, (unsigned long long)st_small_copies,
                  (unsigned long long)st_other_depths, (unsigned long long)st_matched_later, (unsigned long long)st_guess_field,
                  (unsigned long long)st_guess_blind, (unsigned long long)st_unmatched_head, worst_err, (unsigned long long)(lag_min > lag_max ? 0 : lag_min),
                  (unsigned long long)view_lag, (unsigned long long)lag_max, age_n ? age_sum / age_n : 0.0, age_n ? age_min : 0.0, age_max, age_recent);
    std::string out = b;
    {
        char f[400];
        std::snprintf(f, sizeof f,
                      ",\n    \"head_frames_shown_flat\": {\"frames\": %llu, \"scenes_with_an_unknown_tone_curve\": %llu, \"no_depth\": %llu, \"depth_format\": %llu, \"depth_not_sampled\": %llu, "
                      "\"no_camera_block\": %llu, \"projection\": %llu}",
                      (unsigned long long)st_why_frames, (unsigned long long)st_other_tone, (unsigned long long)st_why[1], (unsigned long long)st_why[2],
                      (unsigned long long)st_why[3], (unsigned long long)st_why[4], (unsigned long long)st_why[5]);
        out.insert(out.size() - 1, f);
    }
    {
        char f[400];
        std::snprintf(f, sizeof f,
                      ",\n    \"body\": {\"mode\": \"%s\", \"meshes_known\": %zu, \"draws_changed\": %llu, \"of_them_left_out_whole\": %llu, "
                      "\"in_the_last_frame\": %llu, \"child_draws\": %llu, \"adult_draws\": %llu}",
                      g_body == 1 ? "bras" : g_body == 2 ? "rien" : "entier", body_meshes.size(), (unsigned long long)st_body_draws,
                      (unsigned long long)st_body_whole, (unsigned long long)st_body_last, (unsigned long long)st_body_models[0],
                      (unsigned long long)st_body_models[1]);
        out.insert(out.size() - 1, f);
        out.insert(out.size() - 1, ",\n    \"hands\": " + hands::report_json());
        {
            std::lock_guard lk2(fp_mu);
            char a[360];
            std::snprintf(a, sizeof a, ",\n    \"aim\": {\"views\": %llu, \"began\": %llu, \"hand_from_the_game_aim_degrees\": {\"when_it_began\": [%.1f, %.1f], \"least\": [%.1f, %.1f], \"now\": [%.1f, %.1f]}}",
                          (unsigned long long)st_aim_views, (unsigned long long)st_aim_starts, aim_first[0] * 180.0 / kPi, aim_first[1] * 180.0 / kPi,
                          aim_least[0] > 1e8 ? 0.0 : aim_least[0] * 180.0 / kPi, aim_least[1] > 1e8 ? 0.0 : aim_least[1] * 180.0 / kPi, aim_yaw * 180.0 / kPi, aim_pitch * 180.0 / kPi);
            out.insert(out.size() - 1, a);
            std::snprintf(a, sizeof a, ",\n    \"sight\": {\"on\": %s, \"frames_with_the_ring\": %llu}", g_sight ? "true" : "false", (unsigned long long)st_sight_frames);
            out.insert(out.size() - 1, a);
            std::snprintf(a, sizeof a, ",\n    \"kept_whole\": {\"on\": %s, \"blocks_made_opaque\": %llu, \"draws_writing_depth_again\": %llu}", g_whole ? "true" : "false",
                          (unsigned long long)st_whole_blocks, (unsigned long long)st_whole_depth);
            out.insert(out.size() - 1, a);
            std::snprintf(a, sizeof a, ",\n    \"camera_view_for_filming\": {\"on\": %s, \"field_degrees\": %.0f, \"follows_in_seconds\": %.2f, \"level\": %s, \"frames\": %llu, \"frames_brought_back_to_the_head\": %llu}",
                          g_film.load() ? "true" : "false", g_film_fov, g_film_smooth, g_film_level ? "true" : "false", (unsigned long long)st_film_frames, (unsigned long long)st_film_held);
            out.insert(out.size() - 1, a);
            out.insert(out.size() - 1, ",\n    \"video_recorder\": " + rec::report_json());
            std::snprintf(a, sizeof a, ",\n    \"sorting_camera\": {\"eye_heights_behind\": %.2f, \"study_without_hook\": %s, \"views_moved_back\": %llu, \"views_filled\": %llu, \"true_view_put_back\": %llu}",
                          g_cull_back, g_cull_probe ? "true" : "false", (unsigned long long)st_pulled.load(), (unsigned long long)st_fills.load(), (unsigned long long)st_fills_put.load());
            out.insert(out.size() - 1, a);
        }
        std::snprintf(f, sizeof f, ",\n    \"translucent_figures\": {\"depth_written\": %s, \"draws\": %llu},\n    \"fairy_glow\": {\"reach_cm\": %.0f, \"fairies_seen\": %llu, \"frames_with_her_depth\": %llu}",
                      g_depth_fix ? "true" : "false", (unsigned long long)st_depth_forced, g_halo_m * 100.0, (unsigned long long)st_halo_draws,
                      (unsigned long long)st_halo_frames);
        out.insert(out.size() - 1, f);
    }
    {
        std::lock_guard lk2(fp_mu);
        char f[900];
        std::snprintf(f, sizeof f,
                      ",\n    \"first_person\": {\"person\": %d, \"views\": %llu, \"player\": [%.3f, %.3f, %.3f], \"camera_target_above_feet\": %.3f, \"camera_target_beside\": %.3f, \"child\": %s,\n"
                      "      \"eye_height\": %.3f, \"heading_degrees\": %.1f, \"game_camera_heading_degrees\": %.1f, \"turns\": %llu,\n"
                      "      \"run\": {\"speed\": %.3f, \"against_game_camera_degrees\": %.1f, \"against_our_view_degrees\": %.1f},\n"
                      "      \"game_scenes\": {\"on_screen\": %s, \"started\": %llu, \"views_left_to_the_game\": %llu}}",
                      g_person.load(), (unsigned long long)st_fp_views.load(), player_pos[0], player_pos[1], player_pos[2], target_dy, target_dxz, player_child ? "true" : "false",
                      (g_eye_h > 0 ? g_eye_h : player_child ? g_eye_child : g_eye_adult) + g_eye_up, fp_yaw * 180.0 / kPi, cam_yaw_now * 180.0 / kPi, (unsigned long long)st_pad_turns.load(),
                      move_speed, move_vs_cam, move_vs_view, g_events_screen ? "true" : "false", (unsigned long long)st_event_starts.load(),
                      (unsigned long long)st_event_views.load());
        out.insert(out.size() - 1, f);
    }
    return out;
}

// The host's calls (see rt_vr.h). Plain C, exported by name.
namespace {

// The game's gameplay camera was just built: m is its view (3 rows of 4, world -> view, x right, y up, -z forward),
// cam its own near, far, field (radians, vertical), ratio and offset (null when the host could not tell). Returns
// bit 0 when m was changed, bit 1 when cam was (the host writes them back into the game's memory).
int fill_view(float* m) {
    st_fills.fetch_add(1, std::memory_order_relaxed);
    if (!g_vr || !m || g_cull_probe) return 0;
    std::lock_guard lk(mu);
    for (int k = 0; k < kRing; ++k) {   // (the newest first)
        const Given& g = ring[(given_serial + kRing - k) % kRing];
        if (!g.serial || g.cull11 == g.m[11]) continue;
        if (std::fabs(m[11] - g.cull11) > 1e-4f) continue;
        bool same = true;
        for (int i = 0; i < 11 && same; ++i) same = std::fabs(m[i] - g.m[i]) <= 1e-6f;
        if (!same) continue;
        m[11] = g.m[11];
        st_fills_put.fetch_add(1, std::memory_order_relaxed);
        return 1;
    }
    return 0;
}

int view_built(float* m, const double* eye_pos, const double* target, float* cam) {
    (void)target;
    if (!g_vr || !m) return 0;
    st_views.fetch_add(1, std::memory_order_relaxed);
    double r[3][3], t[3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) r[i][j] = m[i * 4 + j];
        t[i] = m[i * 4 + 3];
    }
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(t[i]) || !std::isfinite(r[i][0]) || !std::isfinite(r[i][1]) || !std::isfinite(r[i][2])) return 0;
    double e[3];   // the camera's position: -R^T t
    for (int j = 0; j < 3; ++j) e[j] = -(r[0][j] * t[0] + r[1][j] * t[1] + r[2][j] * t[2]);
    const int64_t now = now_ns();
    {
        std::lock_guard lk(mu);
        if (cam) std::memcpy(cam_seen, cam, sizeof cam_seen), st_views_cam.fetch_add(1, std::memory_order_relaxed);
        if (eye_pos) std::memcpy(eye_seen, eye_pos, sizeof eye_seen);
        if (!speed_t0) speed_t0 = now, std::memcpy(speed_from, e, sizeof e);
        else if (now - speed_t0 >= 1000000000) {
            const double dx = e[0] - speed_from[0], dy = e[1] - speed_from[1], dz = e[2] - speed_from[2];
            speed_now = std::sqrt(dx * dx + dy * dy + dz * dz) / ((now - speed_t0) / 1e9);
            if (speed_now < 200.0) speed_max = std::max(speed_max, speed_now);   // (a warp is not a speed)
            speed_t0 = now, std::memcpy(speed_from, e, sizeof e);
        }
    }
    // the game camera's heading (its "back" axis laid flat; a turn to the left is positive)
    double cam_back[3] = {r[2][0], 0, r[2][2]};
    {
        double n = std::sqrt(cam_back[0] * cam_back[0] + cam_back[2] * cam_back[2]);
        if (n < 0.05) {   // (looking straight down or up: its "up" axis tells the heading)
            const double s2 = r[2][1] > 0 ? -1.0 : 1.0;
            cam_back[0] = s2 * r[1][0], cam_back[2] = s2 * r[1][2];
            n = std::sqrt(cam_back[0] * cam_back[0] + cam_back[2] * cam_back[2]);
        }
        if (n > 1e-6) cam_back[0] /= n, cam_back[2] /= n;
        else cam_back[0] = 0, cam_back[2] = 1;
    }
    const double cam_yaw = std::atan2(cam_back[0], cam_back[2]);
    {
        std::lock_guard lk(fp_mu);
        cam_yaw_now = cam_yaw;
        if (target && now - player_time < 500000000) {   // where the game's camera aims, against Link's feet
            const double dx = target[0] - player_pos[0], dz = target[2] - player_pos[2];
            target_dy = target[1] - player_pos[1], target_dxz = std::sqrt(dx * dx + dz * dz);
            // (taken as the height of his eyes only while he stands still: the aim rises and leads when he runs.
            // Measured: 0.946 above the feet for the child, at rest.)
            if (target_dxz < 0.15 && target_dy > 0.3 && target_dy < 3.0 && move_speed < 0.2)
                eye_h_auto = eye_h_auto > 0 ? eye_h_auto + (target_dy - eye_h_auto) * 0.05 : target_dy;
        }
    }
    {   // does the game aim a thing? Then how far the hand that aims points from where the game aims: the gestures
        // steer the game's aim to the hand (rt_hands.cpp)
        std::lock_guard lk(fp_mu);
        aim_on = false;
        if (player_time && now - player_time < 500000000) {
            const double hgt = (g_eye_h > 0 ? g_eye_h : player_child ? g_eye_child : g_eye_adult);
            const double dx = e[0] - player_pos[0], dy = e[1] - (player_pos[1] + hgt), dz = e[2] - player_pos[2];
            {   // (study)
                const double beside = std::sqrt(dx * dx + dz * dz), fld = cam ? cam[2] * 180.0 / kPi : 0.0;
                cam_off_now[0] = beside, cam_off_now[1] = dy, cam_off_now[2] = fld;
                const uint64_t n = ++st_cam_study;
                if (g_study > 0 && n % (uint64_t)g_study == 0 && n < 400000) {
                    double gdx = 0, gdy = 0, gdz = 0;   // (the eye the game itself says, against Link's eyes)
                    if (eye_pos) gdx = eye_pos[0] - player_pos[0], gdy = eye_pos[1] - (player_pos[1] + hgt), gdz = eye_pos[2] - player_pos[2];
                    log("vr: study: the game's camera is %.2f beside Link's eyes, %.2f above (%.2f behind along its own view); it faces %.1f degrees, %.1f up; field %.1f; its target %.2f above his feet, %.2f beside; near %.3f; its said eye %.2f beside, %.2f above; sticks %d %d %d %d buttons %llx",
                        beside, dy, dx * r[2][0] + dy * r[2][1] + dz * r[2][2], cam_yaw * 180.0 / kPi, std::asin(std::clamp(-(double)r[2][1], -1.0, 1.0)) * 180.0 / kPi, fld, target_dy, target_dxz,
                        cam ? cam[0] : 0.0, std::sqrt(gdx * gdx + gdz * gdz), gdy, study_stick[0] / 328, study_stick[1] / 328, study_stick[2] / 328, study_stick[3] / 328,
                        (unsigned long long)study_buttons);
                }
            }
            // The game aims a thing (a slingshot, a bow). In this remake its camera does not go to Link's eyes: it stays
            // some 3.5 m behind him and turns about a point over his head, its field narrows from 50 to 40 degrees (45
            // when it looks 42 degrees up), and the game's own picture no longer shows Link
            // (measured). The right stick turns it: 75 degrees a second sideways, 60 up and down; the
            // left stick ends it. (The first guess, "its camera sits at his eyes", was wrong, and is no longer a sign:
            // in a narrow place the game's ordinary camera can come that close to his head.)
            const double behind = dx * r[2][0] + dy * r[2][1] + dz * r[2][2];
            const bool narrow = cam && cam[2] * 180.0 / kPi < g_aim_field && cam[2] > 0.2 && behind > 0.6 && behind < 6.0 && target_dxz < 0.8;
            hands::Hand hm;
            if (narrow && fp_init && hands::hand_now(hands::main_hand(), &hm)) {
                double ra[3][3];
                quat_rows(hm.aim_q, ra);
                double ap[3] = {-ra[0][2], -ra[1][2], -ra[2][2]};   // (where the hand points, play space)
                // with both hands (a bow, a slingshot): from the hand that draws to the hand that holds. When Link holds
                // the thing with both hands (its string is in his other hand), that line is the shot whatever way the
                // drawing hand's controller is turned, as long as the hand that holds is ahead of the one that draws
                // (against where the head looks); else, as before, only when the drawing hand points along it.
                aim_pull = 0;
                hands::Hand ho;
                if (hands::hand_now(1 - hands::main_hand(), &ho)) {
                    const double v[3] = {ho.p[0] - hm.p[0], ho.p[1] - hm.p[1], ho.p[2] - hm.p[2]};
                    const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                    bool line = n > 0.14 && n < 0.95 && (v[0] * ap[0] + v[1] * ap[1] + v[2] * ap[2]) > 0.5 * n;
                    Pose hd;
                    int64_t ht = 0;
                    if (!line && n > 0.14 && n < 0.95 && hands::two_handed() && head_now(hd, ht)) {
                        double rh[3][3];
                        quat_rows(hd.q, rh);
                        line = -(v[0] * rh[0][2] + v[1] * rh[1][2] + v[2] * rh[2][2]) > 0.25 * n;
                    }
                    if (line) ap[0] = v[0] / n, ap[1] = v[1] / n, ap[2] = v[2] / n, aim_pull = n;
                }
                const double bx = std::sin(fp_yaw), bz = std::cos(fp_yaw);   // (the level frame of the first person: its back)
                const double wx = bz * ap[0] + bx * ap[2], wy = ap[1], wz = -bx * ap[0] + bz * ap[2];
                const double gx = -r[2][0], gy = -r[2][1], gz = -r[2][2];   // (where the game aims)
                aim_yaw = std::remainder(std::atan2(-wx, -wz) - std::atan2(-gx, -gz), 2.0 * kPi);
                aim_pitch = std::asin(std::clamp(wy, -1.0, 1.0)) - std::asin(std::clamp(gy, -1.0, 1.0));
                // (study: does the game's aim come to the hand?)
                if (now - aim_time > 500000000) {
                    st_aim_starts++, aim_first[0] = aim_yaw, aim_first[1] = aim_pitch, aim_least[0] = aim_least[1] = 1e9;
                    log("vr: the game aims (its camera's field is %.1f degrees, %.2f behind Link's eyes): the hand points %.1f degrees to the side of it, %.1f above",
                        cam ? cam[2] * 180.0 / kPi : 0.0, behind, aim_yaw * 180.0 / kPi, aim_pitch * 180.0 / kPi);
                }
                if (std::fabs(aim_yaw) + std::fabs(aim_pitch) < std::fabs(aim_least[0]) + std::fabs(aim_least[1])) aim_least[0] = aim_yaw, aim_least[1] = aim_pitch;
                if (++st_aim_views % 30 == 0 && st_aim_views <= 1800)
                    log("vr: aim, view %llu: the hand is %.1f degrees to the side of the game's aim, %.1f above", (unsigned long long)st_aim_views, aim_yaw * 180.0 / kPi, aim_pitch * 180.0 / kPi);
                aim_on = true, aim_time = now;
                sight_world[0].store((float)gx, std::memory_order_relaxed), sight_world[1].store((float)gy, std::memory_order_relaxed);
                sight_world[2].store((float)gz, std::memory_order_relaxed), sight_time.store(now, std::memory_order_relaxed);
            }
        }
    }
    if (!g_mode.load(std::memory_order_relaxed)) return 0;   // ("ecran": the game's own camera)
    {   // (one of the game's scenes, on for a third of a second at least: its own camera, on the big screen)
        std::lock_guard lk(fp_mu);
        if (g_events_screen && event_since && now - event_last < 300000000 && now - event_since > 300000000) {
            st_event_views.fetch_add(1, std::memory_order_relaxed);
            fp_init = false;
            return 0;
        }
    }
    Pose head;
    int64_t time = 0;
    if (!head_now(head, time)) return 0;
    double rh0[3][3];
    quat_rows(head.q, rh0);
    const double head_yaw = std::atan2(rh0[0][2], rh0[2][2]);
    // the first person: the view at Link's eyes, facing our own heading
    bool fp = false;
    {
        std::lock_guard lk(fp_mu);
        fp = g_person.load(std::memory_order_relaxed) == 1 && player_time && now - player_time < 500000000;
        if (fp) {
            if (!fp_init) fp_yaw = cam_yaw, fp_init = true;   // (it starts the way the game's camera faces)
            const double hgt = (g_eye_h > 0 ? g_eye_h : player_child ? g_eye_child : g_eye_adult) + g_eye_up;
            cam_back[0] = std::sin(fp_yaw), cam_back[2] = std::cos(fp_yaw);
            e[0] = player_pos[0] - cam_back[0] * g_fp_forward, e[1] = player_pos[1] + hgt, e[2] = player_pos[2] - cam_back[2] * g_fp_forward;
            fp_time = now;
            st_fp_views.fetch_add(1, std::memory_order_relaxed);
            written_yaw = fp_yaw + head_yaw;
            want_yaw = fp_yaw + (g_move_head ? head_yaw : 0.0);
        } else {
            fp_init = false;
            written_yaw = cam_yaw + head_yaw;
            want_yaw = cam_yaw;
        }
        view_yaw_now = written_yaw;
        fp_now = fp, libre_time = now;
    }
    // the frame the head is laid on: the game camera's position, its heading, and (vr_level) the world's own up, so
    // that the horizon stays level whatever the game camera's pitch is
    double base[3][3];
    std::memcpy(base, r, sizeof base);
    if (g_level || fp) {
        const double* b = cam_back;
        base[0][0] = b[2], base[0][1] = 0, base[0][2] = -b[0];   // right = up x back
        base[1][0] = 0, base[1][1] = 1, base[1][2] = 0;
        base[2][0] = b[0], base[2][1] = 0, base[2][2] = b[2];
    }
    double rh[3][3];
    quat_rows(head.q, rh);
    // view = Rh^T (Base (x - e) - p k): rows of Rh^T Base, and the translation
    double be[3], out[3][4];
    for (int i = 0; i < 3; ++i) be[i] = base[i][0] * e[0] + base[i][1] * e[1] + base[i][2] * e[2] + head.p[i] * units();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) out[i][j] = rh[0][i] * base[0][j] + rh[1][i] * base[1][j] + rh[2][i] * base[2][j];
        out[i][3] = -(rh[0][i] * be[0] + rh[1][i] * be[1] + rh[2][i] * be[2]);
    }
    Given g;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j) g.m[i * 4 + j] = (float)out[i][j];
    g.head = head, g.time = time, g.built = now, g.fp = fp;
    std::memcpy(g.base, base, sizeof base), std::memcpy(g.eye, e, sizeof e);
    std::memcpy(m, g.m, sizeof g.m);
    // The game leaves out what its camera does not see. In the first person Link's own hands are under and behind the
    // camera: what he holds (his sword, his shield, a slingshot) was no longer drawn as soon as his hand left the view,
    // and so could not be shown in the player's hand: walking sword in hand, the sword was lost (proved in the fake
    // headset: with the head bent down 45 degrees it is never lost). The game is
    // given the same camera moved back along its own line of sight: all that the true view sees is inside the view of a
    // camera behind it with the same field, and Link's hands are in front of that one. The picture is still drawn
    // from the true place: the game fills its views' blocks after rt_vr_fill_view (hook V10) has put the true matrix back.
    g.cull11 = g.m[11];
    if (fp && g_cull_back > 0 && (g_cull_probe || st_fills.load(std::memory_order_relaxed))) {
        const double hgt = g_eye_h > 0 ? g_eye_h : player_child ? g_eye_child : g_eye_adult;
        m[11] = g.m[11] - (float)(g_cull_back * hgt);
        g.cull11 = m[11];
        if (g_cull_probe) g.m[11] = m[11];   // (study: no hook, the picture itself is drawn from behind)
        st_pulled.fetch_add(1, std::memory_order_relaxed);
    }
    int flags = 1;
    if (cam) {   // the field of the headset in place of the game's (its zooms are lost: the rule in a headset)
        // (the first person: near enough for his own hands once his body is out of the view; farther while his head
        // and hair are still drawn around the eyes)
        // his whole body without his head: near enough to see his chest and his hands)
        if (fp) cam[0] = std::max(cam[0], (float)(body_meshes.empty() ? g_fp_near : g_body == 0 ? std::min(g_fp_near, 0.12) : g_fp_near_arms));
        cam[2] = (float)(2.0 * std::atan(g_tan_y));
        cam[3] = (float)(g_tan_x / g_tan_y);
        cam[4] = 0, cam[5] = 0;
        flags |= 2;
    }
    std::lock_guard lk(mu);
    g.serial = ++given_serial;
    ring[g.serial % kRing] = g;
    return flags;
}

} // namespace
} // namespace rt::vr

extern "C" {

// Hook V3 (host/hos_main.c, at boot, before the game sets its graphics up): the size of the picture the game draws,
// and of the scene inside it, both packed as height << 32 | width (0: the game's own). Cameras keep the game's 16:9
// (the pixels are then not square: every picture that is not the head's own is shown on a 16:9 screen in the headset);
// the head's camera gets its own field and ratio through rt_vr_view.
__declspec(dllexport) void rt_vr_boot(volatile uint64_t* out, volatile uint64_t* scene) {
    using namespace rt;
    using namespace rt::vr;
    load_options();
    configure();
    if (!wanted()) return;
    if (g_vr == 1 && !xr::boot()) {
        // (no headset: the game starts as it does without the VR mode; our graphics effects stay off for this run)
        log_host("vr: no headset, the VR mode is left out for this run");
        g_vr = 0;
        return;
    }
    // the picture for one eye's field: its height (vr_h, lines), its width from the field's shape, in multiples of 8,
    // within the pixels of a 3840 x 2160 picture (the game's pool of render targets: host/nn_applet.c)
    // (with a headset: the height it advises for one eye, over the share of our field its own covers: ours is the
    // same above and below, 55 degrees each way, its own is about 44 up and 55 down)
    int64_t lines = opt_int("vr_h", 0);
    if (lines <= 0) {
        uint32_t rw = 0, rh = 0;
        if (g_vr == 1) xr::recommended(&rw, &rh);
        lines = rh ? (int64_t)(rh * (2.0 * g_tan_y) / (std::tan(44.0 * kPi / 180.0) + std::tan(55.0 * kPi / 180.0)) + 0.5)
                   : (int64_t)(2400.0 * g_tan_y / std::tan(55.0 * kPi / 180.0) + 0.5);
    }
    double h = (double)std::clamp<int64_t>(lines, 720, 4320) * std::clamp(opt_f("vr_res", 100.0), 25.0, 200.0) * 0.01;
    double w = h * g_tan_x / g_tan_y;
    if (const int64_t fw = opt_int("vr_w", 0); fw > 0) w = (double)fw;
    if (w * h > 3840.0 * 2160.0) {
        const double k = std::sqrt(3840.0 * 2160.0 / (w * h));
        w *= k, h *= k;
    }
    g_w = (uint32_t)(w / 8 + 0.5) * 8, g_h = (uint32_t)(h / 8 + 0.5) * 8;
    if (out) *out = (uint64_t)g_h << 32 | g_w;
    if (scene) *scene = (uint64_t)g_h << 32 | g_w;
    log("vr: the game is asked for a %u x %u picture (and scene)", g_w, g_h);
}

// Hook V7 (host/hos_main.c, where the host follows Link): his position (his feet) each time the game updates him, and
// whether one of the game's scenes is playing (the host's own "event playing"), and whether he is the child.
__declspec(dllexport) void rt_vr_player(const float* pos, int event_playing, int child) {
    using namespace rt;
    using namespace rt::vr;
    if (!wanted() || !pos) return;
    std::lock_guard lk(fp_mu);
    const int64_t now = now_ns();
    player_child = child != 0;
    if (event_playing) {
        if (!event_since || now - event_last > 300000000) event_since = now, st_event_starts.fetch_add(1, std::memory_order_relaxed);
        event_last = now;
    } else if (event_since && now - event_last > 300000000) {
        event_since = 0;
    }
    for (int i = 0; i < 3; ++i) player_pos[i] = pos[i];
    player_time = now;
    if (!player_t0) player_t0 = now, std::memcpy(player_from, player_pos, sizeof player_pos);
    else if (now - player_t0 >= 500000000) {   // (which way he runs, against the game's camera and against our view)
        const double dx = player_pos[0] - player_from[0], dz = player_pos[2] - player_from[2], dt = (now - player_t0) / 1e9;
        move_speed = std::sqrt(dx * dx + dz * dz) / dt;
        if (move_speed > 1.0 && move_speed < 100.0) {
            move_yaw = std::atan2(-dx, -dz);
            move_vs_cam = std::remainder(move_yaw - cam_yaw_now, 2.0 * kPi) * 180.0 / kPi;
            move_vs_view = std::remainder(move_yaw - view_yaw_now, 2.0 * kPi) * 180.0 / kPi;
        }
        player_t0 = now, std::memcpy(player_from, player_pos, sizeof player_pos);
    }
}

// Hook V4 (host/nn_hid.c): see rt::vr::pad_hook.
__declspec(dllexport) void rt_vr_pad(uint64_t* buttons, int32_t* stick) { rt::vr::pad_hook(buttons, stick); }
// Hooks V9 (host/hos_relaunch.c, the window closed) and V9b (host/hos_svc.c, the game quits): the link with the headset
// is closed before the process exits. 1: done or none; 0: not in time (the host then ends the process at once).
__declspec(dllexport) int rt_vr_shutdown(void) {
    rt::rec::shutdown();   // (a video being recorded is closed properly first)
    return rt::xr::shutdown(1500);
}
// Hooks V9c, V9d: 1 when the process must end at once, without unloading its libraries (see rt_xr.cpp, exit_hard)
__declspec(dllexport) int rt_vr_exit_hard(void) { return rt::xr::exit_hard(); }

__declspec(dllexport) int rt_vr_view(float* view34, const double* eye, const double* target, float* camera) {
    return rt::vr::view_built(view34, eye, target, camera);
}

// Hook V10 (the game's own routine that fills each view's blocks for the graphics card, before it reads the view): a
// view whose matrix is the one the game was given for its sorting (view_built: the camera moved back) gets the true
// one, so the picture is drawn from the player's eyes. 1: the matrix was changed.
__declspec(dllexport) int rt_vr_fill_view(float* view34) { return rt::vr::fill_view(view34); }

} // extern "C"
