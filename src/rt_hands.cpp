// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// VR mode: the player's hands, and Link's arms laid on them. See rt_hands.h.
//
// How the arms are posed. The game gives the card, for every piece of Link it draws, one block of 3 x 4 matrices: one
// for each bone his skin follows (found: the vertex stage's 13 440-byte block for the child, 280
// matrices). A "smooth" bone's matrix is its place in the world times the inverse of its place at rest; a "rigid"
// bone's is its place in the world. Positions in the block are the world's minus an offset the game keeps beside the
// piece's own matrix. Writing other matrices for the bones of the arms, in the copy of the block the card is given,
// moves Link's arms without touching the game: his shadow and the game's own logic keep the game's pose.
//
// The pose itself: the wrist is where the player's palm is (the hand's size is known from the skeleton), the elbow is
// found from the shoulder and the wrist (two bones of known lengths, bent towards a point below, beside and behind),
// the fingers bend by the trigger and the grip of a controller, or as the player's own fingers do.
#include "rt_hands.h"

#include "rt_diag.h"
#include "rt_vr.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

namespace rt::hands {
namespace {

constexpr double kPi = 3.14159265358979323846;

double opt_f(const char* key, double fallback) {
    const std::string v = opt(key, "");
    if (v.empty()) return fallback;
    char* end = nullptr;
    const double d = std::strtod(v.c_str(), &end);
    return end && end != v.c_str() ? d : fallback;
}

// ---- small geometry: v' = r v + t; r may carry one scale for all axes ----
struct V3 {
    double x = 0, y = 0, z = 0;
};
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(V3 a, double k) { return {a.x * k, a.y * k, a.z * k}; }
double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
double len(V3 a) { return std::sqrt(dot(a, a)); }
V3 unit(V3 a, V3 fallback = {0, 1, 0}) {
    const double n = len(a);
    return n > 1e-9 ? a * (1.0 / n) : fallback;
}
struct M3 {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};
M3 mul(const M3& a, const M3& b) {
    M3 o;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) o.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return o;
}
M3 transpose(const M3& a) {
    M3 o;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) o.m[i][j] = a.m[j][i];
    return o;
}
V3 mul(const M3& a, V3 v) {
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z, a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}
M3 scaled(const M3& a, double k) {
    M3 o = a;
    for (auto& r : o.m)
        for (double& v : r) v *= k;
    return o;
}
M3 columns(V3 a, V3 b, V3 c) {
    M3 o;
    o.m[0][0] = a.x, o.m[1][0] = a.y, o.m[2][0] = a.z;
    o.m[0][1] = b.x, o.m[1][1] = b.y, o.m[2][1] = b.z;
    o.m[0][2] = c.x, o.m[1][2] = c.y, o.m[2][2] = c.z;
    return o;
}
M3 about(V3 axis, double angle) {   // a turn of `angle` about a unit axis
    const double c = std::cos(angle), s = std::sin(angle), t = 1 - c, x = axis.x, y = axis.y, z = axis.z;
    M3 o;
    o.m[0][0] = t * x * x + c, o.m[0][1] = t * x * y - s * z, o.m[0][2] = t * x * z + s * y;
    o.m[1][0] = t * x * y + s * z, o.m[1][1] = t * y * y + c, o.m[1][2] = t * y * z - s * x;
    o.m[2][0] = t * x * z - s * y, o.m[2][1] = t * y * z + s * x, o.m[2][2] = t * z * z + c;
    return o;
}
M3 of_quat(const double q[4]) {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    M3 o;
    o.m[0][0] = 1 - 2 * (y * y + z * z), o.m[0][1] = 2 * (x * y - z * w), o.m[0][2] = 2 * (x * z + y * w);
    o.m[1][0] = 2 * (x * y + z * w), o.m[1][1] = 1 - 2 * (x * x + z * z), o.m[1][2] = 2 * (y * z - x * w);
    o.m[2][0] = 2 * (x * z - y * w), o.m[2][1] = 2 * (y * z + x * w), o.m[2][2] = 1 - 2 * (x * x + y * y);
    return o;
}
// the turn about `axis` (unit) that a rotation holds: its angle
double twist_about(const M3& r, V3 axis) {
    const double tr = r.m[0][0] + r.m[1][1] + r.m[2][2];
    double q[4];   // x y z w
    if (tr > 0) {
        const double s = std::sqrt(tr + 1.0) * 2;
        q[3] = 0.25 * s, q[0] = (r.m[2][1] - r.m[1][2]) / s, q[1] = (r.m[0][2] - r.m[2][0]) / s, q[2] = (r.m[1][0] - r.m[0][1]) / s;
    } else if (r.m[0][0] > r.m[1][1] && r.m[0][0] > r.m[2][2]) {
        const double s = std::sqrt(1.0 + r.m[0][0] - r.m[1][1] - r.m[2][2]) * 2;
        q[3] = (r.m[2][1] - r.m[1][2]) / s, q[0] = 0.25 * s, q[1] = (r.m[0][1] + r.m[1][0]) / s, q[2] = (r.m[0][2] + r.m[2][0]) / s;
    } else if (r.m[1][1] > r.m[2][2]) {
        const double s = std::sqrt(1.0 + r.m[1][1] - r.m[0][0] - r.m[2][2]) * 2;
        q[3] = (r.m[0][2] - r.m[2][0]) / s, q[0] = (r.m[0][1] + r.m[1][0]) / s, q[1] = 0.25 * s, q[2] = (r.m[1][2] + r.m[2][1]) / s;
    } else {
        const double s = std::sqrt(1.0 + r.m[2][2] - r.m[0][0] - r.m[1][1]) * 2;
        q[3] = (r.m[1][0] - r.m[0][1]) / s, q[0] = (r.m[0][2] + r.m[2][0]) / s, q[1] = (r.m[1][2] + r.m[2][1]) / s, q[2] = 0.25 * s;
    }
    const double a = 2.0 * std::atan2(q[0] * axis.x + q[1] * axis.y + q[2] * axis.z, q[3]);
    return std::remainder(a, 2.0 * kPi);
}
struct X {
    M3 r;
    V3 t;
};
X mul(const X& a, const X& b) { return {mul(a.r, b.r), mul(a.r, b.t) + a.t}; }
X inverse(const X& a) {
    const auto& m = a.r.m;
    const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                       m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    const double d = std::fabs(det) > 1e-12 ? 1.0 / det : 0.0;
    X o;
    o.r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * d, o.r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * d;
    o.r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * d, o.r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * d;
    o.r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * d, o.r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * d;
    o.r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * d, o.r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * d;
    o.r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * d;
    o.t = mul(o.r, a.t) * -1.0;
    return o;
}
X of_rows(const float* f) {
    X o;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) o.r.m[i][j] = f[i * 4 + j];
        (i == 0 ? o.t.x : i == 1 ? o.t.y : o.t.z) = f[i * 4 + 3];
    }
    return o;
}
void to_rows(const X& a, float* f) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) f[i * 4 + j] = (float)a.r.m[i][j];
        f[i * 4 + 3] = (float)(i == 0 ? a.t.x : i == 1 ? a.t.y : a.t.z);
    }
}
void to_rows(const X& a, double* f) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) f[i * 4 + j] = a.r.m[i][j];
        f[i * 4 + 3] = i == 0 ? a.t.x : i == 1 ? a.t.y : a.t.z;
    }
}
// the rotation that takes the frame (a, n) to the frame (a2, n2): a and a2 unit, n and n2 made square to them
M3 frame_to_frame(V3 a, V3 n, V3 a2, V3 n2) {
    n = unit(n - a * dot(n, a), {0, 0, 1});
    n2 = unit(n2 - a2 * dot(n2, a2), {0, 0, 1});
    return mul(columns(a2, n2, cross(a2, n2)), transpose(columns(a, n, cross(a, n))));
}

// ---- Link's skeleton ----
struct Bone {
    char name[32] = {};
    int parent = -1, smooth = -1, rigid = -1;
    X local, rest;   // against its parent; in the model at rest
};
struct Arm {
    int clav = -1, clav_assist = -1, arm1 = -1, arm2 = -1, wrist = -1, weapon = -1;
    int finger[5][3];
    std::vector<int> all;        // the upper arm and every bone under it, parents first
    int twist[2] = {-1, -1};     // helpers along the forearm that share the wrist's turn
    double twist_part[2] = {0.45, 0.85};
    double len1 = 0, len2 = 0;
    V3 d1, d2, hinge;            // at rest, in the model: the upper arm's and the forearm's directions, the elbow's axis
    M3 grip;                     // at rest, in the model: the axes of a grip pose laid on Link's hand (columns)
    V3 palm;                     // from the wrist to the middle of the palm, in the wrist's own frame
    V3 flex[5];                  // at rest, in the model: the axis each finger bends about
    Arm() {
        for (auto& f : finger)
            for (int& j : f) j = -1;
    }
};
struct Skel {
    bool ok = false;
    int n_smooth = 0, n_rigid = 0;
    std::vector<Bone> bones;
    std::vector<X> inv;          // per smooth matrix: the inverse of the bone's rest place
    Arm arm[2];
    int head = -1;
    std::vector<char> of_arm;    // per bone: it is one of an arm's (or what joins that arm to the body)
};
Skel skels[4];

int find(const Skel& s, const std::string& name) {
    for (size_t i = 0; i < s.bones.size(); ++i)
        if (name == s.bones[i].name) return (int)i;
    return -1;
}
int index_of(const Skel& s, int bone) {   // the bone's matrix in the pose block (-1: none)
    const Bone& b = s.bones[bone];
    return b.smooth >= 0 ? b.smooth : b.rigid >= 0 ? s.n_smooth + b.rigid : -1;
}

bool build(Skel& s) {
    for (size_t i = 0; i < s.bones.size(); ++i) {
        Bone& b = s.bones[i];
        if (b.parent >= (int)i) return false;   // (a bone comes after its parent)
        b.rest = b.parent < 0 ? b.local : mul(s.bones[b.parent].rest, b.local);
    }
    s.head = find(s, "Head");
    for (int side = 0; side < 2; ++side) {
        const std::string x = side ? "_R" : "_L";
        Arm& a = s.arm[side];
        a.clav = find(s, "Clavicle" + x), a.clav_assist = find(s, "Ph_Clavicle_Assist" + x);
        a.arm1 = find(s, "Arm_1" + x), a.arm2 = find(s, "Arm_2" + x), a.wrist = find(s, "Wrist" + x), a.weapon = find(s, "Weapon" + x);
        static const char letters[5] = {'A', 'B', 'C', 'D', 'E'};
        for (int f = 0; f < 5; ++f)
            for (int j = 0; j < 3; ++j) a.finger[f][j] = find(s, std::string("Finger_") + letters[f] + "_" + char('1' + j) + x);
        a.twist[0] = find(s, "Ph_Wrist_Assist_A" + x), a.twist[1] = find(s, "Ph_Wrist_Assist_B" + x);
        if (a.arm1 < 0 || a.arm2 < 0 || a.wrist < 0 || a.finger[1][0] < 0 || a.finger[2][0] < 0 || a.finger[4][0] < 0 || a.finger[0][1] < 0) return false;
        std::vector<char> under(s.bones.size(), 0);
        for (size_t i = 0; i < s.bones.size(); ++i) {
            under[i] = (int)i == a.arm1 || (s.bones[i].parent >= 0 && under[s.bones[i].parent]);
            if (under[i]) a.all.push_back((int)i);
        s.of_arm.resize(s.bones.size(), 0);
        for (int b : a.all) s.of_arm[(size_t)b] = 1;
        }
        auto at = [&](int b) { return s.bones[b].rest.t; };
        a.len1 = len(at(a.arm2) - at(a.arm1)), a.len2 = len(at(a.wrist) - at(a.arm2));
        a.d1 = unit(at(a.arm2) - at(a.arm1)), a.d2 = unit(at(a.wrist) - at(a.arm2));
        a.hinge = unit(cross(a.d1, V3{0, 0, 1}), {0, 1, 0});   // (the model faces +z: the elbow folds the hand forwards)
        const V3 f = unit(at(a.finger[2][0]) - at(a.wrist));
        V3 t = at(a.finger[1][0]) - at(a.finger[4][0]);   // from the little finger's side to the forefinger's
        t = unit(t - f * dot(t, f));
        const V3 xo = cross(f, t);
        a.grip = columns(xo, f * -1.0, t * -1.0);
        const M3 wr = transpose(s.bones[a.wrist].rest.r);
        a.palm = mul(wr, (at(a.finger[2][0]) - at(a.wrist)) * 0.62);
        const V3 palm_out = side ? xo * -1.0 : xo;   // (out of the palm)
        for (int k = 1; k < 5; ++k) {
            const V3 dir = a.finger[k][1] >= 0 ? unit(at(a.finger[k][1]) - at(a.finger[k][0])) : f;
            a.flex[k] = unit(cross(dir, palm_out));
        }
        {   // the thumb folds across the palm, towards the root of the forefinger
            const V3 dir = unit(at(a.finger[0][1]) - at(a.finger[0][0]));
            const V3 to = unit(unit(at(a.finger[1][0]) - at(a.finger[0][1])) + palm_out * 0.6);
            a.flex[0] = unit(cross(dir, to));
        }
        for (int k = 0; k < 2; ++k)   // (how far along the forearm a helper sits: that share of the wrist's turn)
            if (a.twist[k] >= 0 && s.bones[a.twist[k]].parent == a.arm2) {
                const V3 along = mul(transpose(s.bones[a.arm2].rest.r), a.d2);
                a.twist_part[k] = std::clamp(dot(s.bones[a.twist[k]].local.t, along) / std::max(a.len2, 1e-6), 0.2, 1.0);
            }
    }
    return true;
}

// ---- the hands ----
std::mutex mu;
Hand now_h[2], frame_h[2];
bool configured = false;
int g_on = 1;                    // vr_arms: 1 Link's arms follow the player's hands
double g_arm_m = 0.50;           // vr_arm_cm: shoulder to wrist of the player (the arms are drawn at that size)
double g_arm_scale = 0;          // vr_arm_scale: 0 from vr_arm_cm, else a fixed factor
double g_tilt = 0, g_lift = 0;   // vr_hand_tilt (degrees about the grip's x), vr_hand_lift (cm along the grip's -y)
double g_shoulder[3] = {0.17, 0.20, 0.07};   // vr_shoulder_*: beside, below and behind the eyes (metres)
int g_sim = 0;                   // vr_sim_hands
int g_swap = 1;                  // vr_swap_hands: what Link holds in his left hand goes to the player's right hand
int g_palm_mirror = 1;           // vr_palm_mirror: passed to the other hand, a thing stays on the palm's side (0: as before)
double g_hold_curl = 0.60;       // vr_hold_curl: how far the fingers close on a handle (1: a tight fist, through the handle)
uint64_t grip_frame[2] = {~0ull, ~0ull};   // the frame at which a thing drawn like a sword was last in each of the player's hands
bool g_two_hands = true;         // vr_two_hands: the string of a bow, the pouch of a slingshot go with the player's other hand
bool g_parts_study = false;      // (study) vr_parts_study = 1: the parts of a thing held with both hands, in the journal
int g_gestures = 1;              // vr_gestures: the player's movements are the game's commands
int g_main = 1;                  // vr_main_hand: the hand that strikes (droite 1, gauche 0)
double g_strike = 2.4;           // vr_strike_speed: metres a second of the blade's middle (0: off)
double g_throw = 1.3;            // vr_throw_speed: the hand's speed when it lets go: thrown; slower: put down
double g_aim_gain = 6.0;         // vr_aim_gain: the stick is pushed fully when the game's aim is 9.5 degrees from the hand
int g_aim_stick = 1;             // vr_aim_stick: the stick the game turns its aim with (droite 1, gauche 0)
double g_aim_least = 0.22;       // vr_aim_least: the least push that the game still follows
double g_aim_still = 0.0087;     // vr_aim_still (degrees): nearer than this, the aim is left where it is
// vr_reach_down, vr_reach_out (cm): how far below the eyes, or how far forward, an empty hand that closes takes hold.
// 60 and 45 at first: in the headset the hand was seldom low or far enough, and the action button had to be pressed
// by hand, where closing the grips on the thing should be enough.
double g_reach_down = 0.25, g_reach_out = 0.25;
// which of the game's buttons a gesture presses (the host's positions; found by pressing each in turn
// in the third person: in this remake Y draws the sword and strikes)
// (seen: Y draws the sword and strikes, B puts it away, ZL locks on and raises the shield, A acts)
int g_btn_strike = 2, g_btn_shield = 15, g_btn_action = 1, g_btn_away = 0;   // vr_btn_strike (Y), _shield (ZL), _action (A), _away (B)
int g_btn_item = 16;             // vr_btn_item (ZR): the thing Link aims is drawn while it is held, shot when it is let go
int g_item_taken = 1;            // vr_item_taken: a thing that comes out with that button and is held up (a bomb) is carried

int button_of(const std::string& name, int fallback) {
    static const struct {
        const char* name;
        int bit;
    } names[] = {{"B", 0}, {"A", 1}, {"Y", 2}, {"X", 3}, {"L", 9}, {"R", 10}, {"ZL", 15}, {"ZR", 16}};
    for (const auto& n : names)
        if (name == n.name) return n.bit;
    return fallback;
}
double sim_pose[2][6] = {{-22, -30, -36, 0, 0, 0}, {22, -30, -36, 0, 0, 0}};
double sim_trigger = 0, sim_squeeze = 0;
int64_t t0 = 0;
uint64_t st_posed = 0, st_frames = 0, st_reach = 0;
double last_k = 0, last_elbow[2] = {};
int g_back = 1;                  // vr_back: the sword and the shield are taken from the back and put away there
int g_strike_draws = 0;          // vr_strike_draws: a strike with an empty hand draws the sword (as before)
int g_wave_talk = 0;             // vr_wave_talk: a wave presses the action button (the character speaks), as before
std::atomic<int64_t> wave_t{0};  // when the player last waved
int g_torso = 1;                 // vr_torso: the rest of the skeleton under the player's head (0: left where the game has it)
double g_pull = 0.06;            // vr_shoulder_pull (cm): how far a shoulder follows a hand that is out of the arm's reach
double g_stretch = 1.4;          // vr_arm_stretch: how many times longer the arm may then be drawn
uint64_t st_stretched = 0;
X game_hand[4][2], our_hand[4][2];
bool held_ok[4][2] = {};
double held_scale[4] = {};

void quat_axis(double ax, double ay, double az, double degrees, double o[4]) {
    const double h = degrees * kPi / 360.0, s = std::sin(h);
    o[0] = ax * s, o[1] = ay * s, o[2] = az * s, o[3] = std::cos(h);
}
void quat_mul(const double a[4], const double b[4], double o[4]) {
    o[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    o[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    o[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    o[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}

// The fake headset's hands (vr_sim_hands): 1 held still; 2 slow circles; 3 the right hand strikes every two seconds;
// 4 the right hand waves; 5 the right hand reaches down, closes, rises and throws; 6 the left hand does; 7 the right
// hand takes the sword from the back, strikes, puts it away; 8 the left hand takes the shield; 9 both; 11 the right
// hand presses the item's button (a bomb comes out), closes, throws; 10 the archer:
// the left hand holds the thing out in front (vr_sim_hand_l), the right hand comes behind it on the line of its own
// yaw and pitch (vr_sim_hand_r), squeezes its trigger, draws back, holds, lets go (a round of 10 s). Poses: vr_sim_hand_l / _r =
// "x,y,z,yaw,pitch,roll" (cm from where the head was centred, degrees; 0 0 0: fingers forward, thumb up).
Hand sim_hand(int side) {
    Hand h;
    const double t = (now_ns() - t0) / 1e9;
    double x = sim_pose[side][0], y = sim_pose[side][1], z = sim_pose[side][2];
    double yaw = sim_pose[side][3], pitch = sim_pose[side][4], roll = sim_pose[side][5];
    double vx = 0, vy = 0, vz = 0;
    h.trigger = (float)sim_trigger, h.squeeze = (float)sim_squeeze;
    if (g_sim == 2) {
        const double w = kPi * (side ? 1.0 : 0.8);
        x += 8 * std::cos(t * w), y += 8 * std::sin(t * w), vx = -8 * w * std::sin(t * w), vy = 8 * w * std::cos(t * w);
        roll += 40 * std::sin(t * 1.3);
    } else if (g_sim == 3 && side == 1) {
        const double ph = std::fmod(t, 2.0);   // (a strike from the upper right to the lower left in 0.18 s)
        const double u = std::clamp((ph - 0.5) / 0.18, 0.0, 1.0), s = u * u * (3 - 2 * u);
        x += 25 - 50 * s, y += 30 - 45 * s, z += -10 * std::sin(s * kPi);
        if (ph > 0.5 && ph < 0.68) vx = -50 / 0.18, vy = -45 / 0.18;
        pitch += -60 * s, h.squeeze = 1;
    } else if (g_sim == 4 && side == 1) {
        x += 6 * std::sin(t * 9.0), y += 32, roll += 28 * std::sin(t * 9.0), vx = 54 * std::cos(t * 9.0);
        pitch += -20;
    } else if ((g_sim == 7 || g_sim == 9) && side == 1) {
        // to the back over the shoulder, closes, draws out in front, strikes once, holds, back again (put away), opens
        const double ph = std::fmod(t, 9.0);
        const double bx = 22 - x, by = -8 - y, bz = 14 - z;   // (from the hand's own place to the back)
        auto ease = [](double u) { u = std::clamp(u, 0.0, 1.0); return u * u * (3 - 2 * u); };
        double k = 0;   // how far towards the back
        if (ph < 1.0) k = ease(ph);
        else if (ph < 1.4) k = 1, h.squeeze = 1;
        else if (ph < 1.9) k = 1 - ease((ph - 1.4) / 0.5), h.squeeze = 1;
        else if (ph < 5.0) {
            h.squeeze = 1;
            const double u = std::clamp((ph - 3.0) / 0.18, 0.0, 1.0), sw = ph < 3.6 ? u * u * (3 - 2 * u) : 1 - ease((ph - 3.6) / 0.4);
            x += -45 * sw, y += -30 * sw, pitch += -50 * sw;
            if (ph > 3.0 && ph < 3.18) vx = -45 / 0.18, vy = -30 / 0.18;
        } else if (ph < 6.0) k = ease(ph - 5.0), h.squeeze = 1;
        else if (ph < 7.0) k = 1 - ease(ph - 6.0);
        x += bx * k, y += by * k, z += bz * k;
    } else if ((g_sim == 8 || g_sim == 9) && side == 0) {
        // the left hand: to the back, closes, draws out (the shield), holds, back again, opens
        const double ph = std::fmod(t + (g_sim == 9 ? 4.5 : 0.0), 9.0);
        const double bx = -22 - x, by = -8 - y, bz = 14 - z;
        auto ease = [](double u) { u = std::clamp(u, 0.0, 1.0); return u * u * (3 - 2 * u); };
        double k = 0;
        if (ph < 1.0) k = ease(ph);
        else if (ph < 1.4) k = 1, h.squeeze = 1;
        else if (ph < 1.9) k = 1 - ease((ph - 1.4) / 0.5), h.squeeze = 1;
        else if (ph < 5.0) h.squeeze = 1;
        else if (ph < 6.0) k = ease(ph - 5.0), h.squeeze = 1;
        else if (ph < 7.0) k = 1 - ease(ph - 6.0);
        x += bx * k, y += by * k, z += bz * k;
    } else if (g_sim == 11 && side == 1) {
        // the item's button for a moment (a bomb comes out), the hand closes, then goes forward fast and opens: thrown
        const double ph = std::fmod(t, 8.0);
        if (ph >= 1.0 && ph < 1.2) h.trigger = 1;
        if (ph >= 1.6 && ph < 3.25) h.squeeze = 1;
        if (ph >= 3.0 && ph < 3.25) y += 25 * (ph - 3.0) / 0.25, z += -55 * (ph - 3.0) / 0.25, vz = -55 / 0.25, vy = 25 / 0.25;
        else if (ph >= 3.25 && ph < 5.0) y += 25, z += -55;
    } else if (g_sim == 10) {
        yaw = sim_pose[1][3], pitch = sim_pose[1][4];   // (both hands along the line of the shot)
        if (side == 1) {
            const double ph = std::fmod(t, 10.0), cy = std::cos(yaw * kPi / 180.0), sy = std::sin(yaw * kPi / 180.0);
            const double cp = std::cos(pitch * kPi / 180.0), sp = std::sin(pitch * kPi / 180.0);
            auto ease = [](double u) { u = std::clamp(u, 0.0, 1.0); return u * u * (3 - 2 * u); };
            double pull = 16;   // cm behind the hand that holds
            if (ph >= 2.0 && ph < 7.0) pull = 16 + 30 * ease((ph - 2.0) / 1.5), h.trigger = 1;
            else if (ph >= 7.0 && ph < 7.4) pull = 46;
            else if (ph >= 7.4) pull = 46 - 30 * ease((ph - 7.4) / 1.0);
            x = sim_pose[0][0] + sy * cp * pull, y = sim_pose[0][1] - sp * pull, z = sim_pose[0][2] + cy * cp * pull;
        }
    } else if ((g_sim == 5 && side == 1) || (g_sim == 6 && side == 0)) {   // (6: the same with the left hand)
        const double ph = std::fmod(t, 6.0);
        if (ph < 1.5) y += -45 * std::min(ph, 1.0), z += -12 * std::min(ph, 1.0), pitch += 45 * std::min(ph, 1.0);
        else if (ph < 4.0) y += -45 + 55 * std::min((ph - 1.5), 1.0), z += -12, h.squeeze = 1;   // (from 75 cm under the eyes)
        else if (ph < 4.25) y += 10 + 25 * (ph - 4.0) / 0.25, z += -12 - 55 * (ph - 4.0) / 0.25, vz = -55 / 0.25, vy = 25 / 0.25, h.squeeze = 1;
        else y += 35, z += -67;
    }
    double qy[4], qp[4], qr[4], qn[4], a[4], b[4];
    quat_axis(0, 1, 0, yaw, qy), quat_axis(1, 0, 0, pitch, qp), quat_axis(0, 0, -1, roll, qr);
    quat_axis(1, 0, 0, 90, qn);   // (fingers forward, thumb up: the grip pose turned a quarter about x)
    quat_mul(qy, qp, a), quat_mul(a, qr, b), quat_mul(b, qn, h.q);
    h.p[0] = x * 0.01, h.p[1] = y * 0.01, h.p[2] = z * 0.01;
    h.vel[0] = vx * 0.01, h.vel[1] = vy * 0.01, h.vel[2] = vz * 0.01;
    std::memcpy(h.aim_q, b, sizeof b), std::memcpy(h.aim_p, h.p, sizeof h.p);
    h.tracked = true, h.time = now_ns();
    return h;
}

} // namespace

void configure() {
    if (configured) return;
    configured = true;
    t0 = now_ns();
    g_on = (int)opt_int("vr_arms", 1);
    g_arm_m = std::clamp(opt_f("vr_arm_cm", 55.0), 20.0, 90.0) * 0.01;
    g_pull = std::clamp(opt_f("vr_shoulder_pull", 6.0), 0.0, 60.0) * 0.01, g_stretch = std::clamp(opt_f("vr_arm_stretch", 1.4), 1.0, 2.0);
    g_torso = (int)opt_int("vr_torso", 1);
    g_back = (int)opt_int("vr_back", 1), g_strike_draws = (int)opt_int("vr_strike_draws", 0);
    g_wave_talk = (int)opt_int("vr_wave_talk", 0);
    g_arm_scale = std::clamp(opt_f("vr_arm_scale", 0.0), 0.0, 4.0);
    g_tilt = opt_f("vr_hand_tilt", 0.0), g_lift = opt_f("vr_hand_lift", 0.0) * 0.01;
    g_shoulder[0] = opt_f("vr_shoulder_side", 17.0) * 0.01, g_shoulder[1] = opt_f("vr_shoulder_down", 20.0) * 0.01;
    g_shoulder[2] = opt_f("vr_shoulder_back", 7.0) * 0.01;
    g_sim = (int)opt_int("vr_sim_hands", 0);
    g_swap = (int)opt_int("vr_swap_hands", 1), g_gestures = (int)opt_int("vr_gestures", 1);
    g_palm_mirror = (int)opt_int("vr_palm_mirror", 1);
    g_hold_curl = std::clamp(opt_f("vr_hold_curl", 0.60), 0.2, 1.0);
    g_two_hands = opt_int("vr_two_hands", 1) != 0;
    g_parts_study = opt_int("vr_parts_study", 0) != 0;
    g_main = opt("vr_main_hand", "droite") == "gauche" ? 0 : 1;
    g_strike = std::clamp(opt_f("vr_strike_speed", 2.4), 0.0, 20.0), g_throw = std::clamp(opt_f("vr_throw_speed", 1.3), 0.1, 20.0);
    g_aim_gain = std::clamp(opt_f("vr_aim_gain", 6.0), 0.2, 20.0);
    g_aim_stick = opt("vr_aim_stick", "droite") == "gauche" ? 0 : 1;
    g_aim_least = std::clamp(opt_f("vr_aim_least", 0.22), 0.0, 1.0), g_aim_still = std::clamp(opt_f("vr_aim_still", 0.5), 0.0, 10.0) * kPi / 180.0;
    g_reach_down = opt_f("vr_reach_down", 25.0) * 0.01, g_reach_out = opt_f("vr_reach_out", 25.0) * 0.01;
    g_btn_strike = button_of(opt("vr_btn_strike", "Y"), 2), g_btn_shield = button_of(opt("vr_btn_shield", "ZL"), 15);
    g_btn_action = button_of(opt("vr_btn_action", "A"), 1), g_btn_away = button_of(opt("vr_btn_away", "B"), 0);
    g_btn_item = button_of(opt("vr_btn_item", "ZR"), 16);
    g_item_taken = (int)opt_int("vr_item_taken", 1);
    sim_trigger = opt_f("vr_sim_trigger", 0.0), sim_squeeze = opt_f("vr_sim_squeeze", 0.0);
    for (int s = 0; s < 2; ++s) {
        const std::string v = opt(s ? "vr_sim_hand_r" : "vr_sim_hand_l", "");
        double d[6];
        if (!v.empty() && std::sscanf(v.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf", &d[0], &d[1], &d[2], &d[3], &d[4], &d[5]) == 6)
            std::memcpy(sim_pose[s], d, sizeof d);
    }
}

void set_hand(int side, const Hand& h) {
    std::lock_guard lk(mu);
    now_h[side & 1] = h;
}

bool hand_now(int side, Hand* out) {
    side &= 1;
    if (vr::sim()) {
        if (!g_sim) return false;
        *out = sim_hand(side);
        return true;
    }
    std::lock_guard lk(mu);
    if (!now_h[side].tracked || now_ns() - now_h[side].time > 300000000) return false;
    *out = now_h[side];
    return true;
}

void latch() {
    Hand h[2];
    const bool ok[2] = {hand_now(0, &h[0]), hand_now(1, &h[1])};
    std::lock_guard lk(mu);
    for (int s = 0; s < 2; ++s) frame_h[s] = ok[s] ? h[s] : Hand{};
    st_frames++;
}

bool hand_latched(int side, Hand* out) {
    std::lock_guard lk(mu);
    *out = frame_h[side & 1];
    return out->tracked;
}

bool load(FILE* f, uint32_t n_models) {
    bool any = false;
    for (uint32_t m = 0; m < n_models; ++m) {
        uint32_t head[4] = {};
        if (std::fread(head, 4, 4, f) != 4 || head[1] > 4096 || head[2] > 4096 || head[3] > 4096) return false;
        Skel local;
        Skel& s = head[0] < 4 ? skels[head[0]] : local;
        s = Skel{};
        s.n_smooth = (int)head[2], s.n_rigid = (int)head[3];
        s.bones.resize(head[1]);
        for (Bone& b : s.bones) {
            int32_t ids[3];
            float l[12];
            if (std::fread(b.name, 1, 32, f) != 32 || std::fread(ids, 4, 3, f) != 3 || std::fread(l, 4, 12, f) != 12) return false;
            b.name[31] = 0, b.parent = ids[0], b.smooth = ids[1], b.rigid = ids[2], b.local = of_rows(l);
        }
        s.inv.resize(head[2]);
        for (X& x : s.inv) {
            float l[12];
            if (std::fread(l, 4, 12, f) != 12) return false;
            x = of_rows(l);
        }
        s.ok = build(s);
        any = any || s.ok;
        if (s.ok)
            log_host("vr: Link's skeleton %u: %zu bones, %d + %d matrices; arm %.3f + %.3f game units, drawn %.2f times as large",
                     head[0], s.bones.size(), s.n_smooth, s.n_rigid, s.arm[0].len1, s.arm[0].len2,
                     g_arm_scale > 0 ? g_arm_scale : g_arm_m / std::max(s.arm[0].len1 + s.arm[0].len2, 1e-6));
        else
            log_host("vr: Link's skeleton %u could not be read: his arms keep the game's pose", head[0]);
    }
    return any;
}

uint32_t matrices(int model) {
    if (model < 0 || model >= 4 || !skels[model].ok || !g_on) return 0;
    return (uint32_t)(skels[model].n_smooth + skels[model].n_rigid);
}

namespace {

// ---- the arms of one frame, worked out once (the pose block is asked for at every piece of Link) ----
struct Solved {
    uint64_t frame = ~0ull;
    int body = -1;
    bool ok[2] = {};
    std::vector<X> w;            // the world place of every bone of the arms (ours)
    bool torso_ok = false;       // the rest of the skeleton laid under the player's head (see solve())
    X torso;                     // from the model at rest to the world
    X game_wrist[2], game_waist, game_head;
    double k = 1;                // how many times larger than the game's the arms are drawn
};
Solved solved[4];

X game_place(const Skel& s, int bone, const uint8_t* in, V3 off) {   // where the game's own pose has a bone (world)
    const int i = bone >= 0 ? index_of(s, bone) : -1;
    if (i < 0) return X{};
    X w = of_rows((const float*)(in + (size_t)i * 48));
    if (s.bones[bone].smooth >= 0) w = mul(w, s.bones[bone].rest);
    w.t = w.t + off;
    return w;
}

const Solved* solve(int model, const Frame& fr, const float offset[3], const uint8_t* in, uint32_t n, int body) {
    if (model < 0 || model >= 4 || !g_on) return nullptr;
    const Skel& s = skels[model];
    if (!s.ok || n != (uint32_t)(s.n_smooth + s.n_rigid) || !in) return nullptr;
    Solved& so = solved[model];
    uint64_t frame;
    Hand h[2];
    {
        std::lock_guard lk(mu);
        frame = st_frames, h[0] = frame_h[0], h[1] = frame_h[1];
    }
    if (so.frame == frame && so.body == body) return &so;
    so.frame = frame, so.body = body;
    so.w.assign(s.bones.size(), X{});
    const V3 off{offset[0], offset[1], offset[2]};
    M3 base;   // (rows: the world's directions of the play space's axes)
    std::memcpy(base.m, fr.base, sizeof base.m);
    const M3 to_world = transpose(base);
    const V3 eye{fr.eye[0], fr.eye[1], fr.eye[2]};
    const double u = fr.units;
    // the player's shoulders: under the head, turned as the head is turned (flat)
    const M3 head_r = mul(to_world, of_quat(fr.head_q));
    const V3 head_at = eye + mul(to_world, V3{fr.head_p[0], fr.head_p[1], fr.head_p[2]} * u);
    V3 fwd = mul(head_r, V3{0, 0, -1});
    fwd.y = 0;
    fwd = unit(fwd, {0, 0, -1});
    const V3 up{0, 1, 0}, right = cross(fwd, up);
    const double len_all = s.arm[0].len1 + s.arm[0].len2;
    const double k_free = g_arm_scale > 0 ? g_arm_scale : std::clamp(g_arm_m * u / std::max(len_all, 1e-6), 0.4, 4.0);
    so.k = body ? k_free : 1.0;
    // The rest of the skeleton, with the body out of the view: laid as the model stands at rest, under the player's
    // head, at the arms' size, its shoulders where the player's are taken to be. A sleeve or a shoulder follows the
    // arm's bones AND the chest's: with the chest left where the game has it (Link's own body, elsewhere and smaller),
    // such a point was torn between the two, so only the points that follow the arm alone were drawn: a sleeve cut down
    // to a ring, an arm open at the top, seen hollow from above. With the chest's bones here, the whole
    // sleeve and the shoulder are drawn (tools/vr-corps.py keeps them), and close the arm.
    so.torso_ok = false;
    if (body && g_torso && s.arm[0].arm1 >= 0 && s.arm[1].arm1 >= 0 && s.head >= 0 && find(s, "Waist") >= 0) {
        const V3 sl = s.bones[s.arm[0].arm1].rest.t, sr = s.bones[s.arm[1].arm1].rest.t;
        const V3 lat = unit(sr - sl, {1, 0, 0});
        V3 um = s.bones[s.head].rest.t - s.bones[find(s, "Waist")].rest.t;
        um = unit(um - lat * dot(um, lat), {0, 1, 0});
        const V3 fm = cross(um, lat);
        M3 r;   // (the model's own right, up and front to the world's)
        const V3 rows[3] = {{right.x * lat.x + up.x * um.x + fwd.x * fm.x, right.x * lat.y + up.x * um.y + fwd.x * fm.y, right.x * lat.z + up.x * um.z + fwd.x * fm.z},
                            {right.y * lat.x + up.y * um.x + fwd.y * fm.x, right.y * lat.y + up.y * um.y + fwd.y * fm.y, right.y * lat.z + up.y * um.z + fwd.y * fm.z},
                            {right.z * lat.x + up.z * um.x + fwd.z * fm.x, right.z * lat.y + up.z * um.y + fwd.z * fm.y, right.z * lat.z + up.z * um.z + fwd.z * fm.z}};
        for (int i = 0; i < 3; ++i) r.m[i][0] = rows[i].x, r.m[i][1] = rows[i].y, r.m[i][2] = rows[i].z;
        so.torso.r = scaled(r, k_free);
        const V3 mid_w = head_at + up * (-g_shoulder[1] * u) + fwd * (-g_shoulder[2] * u);
        so.torso.t = mid_w - mul(so.torso.r, (sl + sr) * 0.5);
        so.torso_ok = true;
    }
    so.game_waist = game_place(s, find(s, "Waist"), in, off);
    so.game_head = game_place(s, find(s, "Head_2") >= 0 ? find(s, "Head_2") : s.head, in, off);
    std::vector<X>& w = so.w;
    for (int side = 0; side < 2; ++side) {
        const Arm& a = s.arm[side];
        so.game_wrist[side] = game_place(s, a.wrist, in, off);
        so.ok[side] = h[side].tracked;
        if (!so.ok[side]) continue;
        const double sgn = side ? 1.0 : -1.0;
        const V3 shoulder_free = head_at + right * (sgn * g_shoulder[0] * u) + up * (-g_shoulder[1] * u) + fwd * (-g_shoulder[2] * u);
        // the hand: its place and its turn in the world
        const M3 tilt = about({1, 0, 0}, g_tilt * kPi / 180.0);
        const M3 grip_r = mul(mul(to_world, of_quat(h[side].q)), tilt);
        V3 grip_at = eye + mul(to_world, V3{h[side].p[0], h[side].p[1], h[side].p[2]} * u);
        grip_at = grip_at + mul(grip_r, V3{0, -g_lift * u, 0});
        double k = k_free;
        V3 shoulder = shoulder_free;
        if (body == 0) {   // his body is drawn: the arm stays on his shoulder and reaches the way the player's does
            k = 1.0;
            shoulder = game_place(s, a.arm1, in, off).t;
            grip_at = shoulder + (grip_at - shoulder_free) * (1.0 / k_free);
        }
        const M3 hand_r = mul(mul(grip_r, transpose(a.grip)), s.bones[a.wrist].rest.r);
        V3 wrist = grip_at - mul(hand_r, a.palm) * k;
        // shoulder, elbow, wrist
        double l1 = a.len1 * k, l2 = a.len2 * k, reach = (l1 + l2) * 0.995;
        V3 sw = wrist - shoulder;
        double d = len(sw);
        V3 wrist_at = wrist;
        if (d > reach) {
            st_reach++;
            if (body == 0) {
                wrist_at = shoulder + unit(sw) * reach;   // (his own arm is no longer than it is)
            } else {
                // The hand is farther than the arm is long. The shoulder follows a little: behind and below the eyes it
                // stays out of sight. Past that the arm is drawn longer (up to g_stretch times), then the shoulder
                // follows all the way. Before, the shoulder alone followed, unseen as it was thought: with the arm
                // stretched far down it came in front of the eyes, and the arm's open end with it (a hollow sleeve).
                shoulder = shoulder + unit(sw) * std::min(d - reach, g_pull * u);
                d = len(wrist - shoulder);
                if (d > reach && g_stretch > 1.0) {
                    k *= std::min(d / reach, g_stretch);
                    wrist = grip_at - mul(hand_r, a.palm) * k, wrist_at = wrist;
                    l1 = a.len1 * k, l2 = a.len2 * k, reach = (l1 + l2) * 0.995;
                    d = len(wrist - shoulder);
                    st_stretched++;
                }
                if (d > reach) shoulder = wrist - unit(wrist - shoulder) * reach;
            }
            sw = wrist_at - shoulder, d = std::min(len(sw), reach);
        }
        d = std::max(d, std::fabs(l1 - l2) + 1e-4);
        const V3 dir = unit(sw);
        const V3 pole = unit(up * -1.0 + right * (sgn * 0.55) + fwd * -0.4);
        const V3 bend = unit(pole - dir * dot(pole, dir), right * sgn);
        const double x = (l1 * l1 - l2 * l2 + d * d) / (2 * d), y = std::sqrt(std::max(l1 * l1 - x * x, 0.0));
        const V3 elbow = shoulder + dir * x + bend * y;
        const V3 dir1 = unit(elbow - shoulder), dir2 = unit(wrist_at - elbow);
        V3 nrm = cross(dir1, dir2);
        nrm = len(nrm) > 1e-5 ? unit(nrm) : unit(cross(dir1, bend * -1.0));
        last_elbow[side] = std::acos(std::clamp(dot(dir1, dir2), -1.0, 1.0)) * 180.0 / kPi;
        const M3 r1 = mul(frame_to_frame(a.d1, a.hinge, dir1, nrm), s.bones[a.arm1].rest.r);
        const M3 r2 = mul(frame_to_frame(a.d2, a.hinge, dir2, nrm), s.bones[a.arm2].rest.r);
        w[a.arm1] = {scaled(r1, k), shoulder};
        w[a.arm2] = {scaled(r2, k), elbow};
        w[a.wrist] = {scaled(hand_r, k), wrist_at};
        // how far the hand is turned about the forearm, against the rest pose: shared along the forearm
        const M3 rel = mul(transpose(r2), hand_r), rel0 = mul(transpose(s.bones[a.arm2].rest.r), s.bones[a.wrist].rest.r);
        const V3 along = mul(transpose(s.bones[a.arm2].rest.r), a.d2);
        const double turn = twist_about(mul(rel, transpose(rel0)), along);
        // the fingers: a controller's trigger and grip, or the player's own fingers
        float curl[5][3];
        if (h[side].fingers) {
            std::memcpy(curl, h[side].curl, sizeof curl);
        } else {
            float tr = std::clamp(h[side].trigger, 0.0f, 1.0f), sq = std::clamp(h[side].squeeze, 0.0f, 1.0f);
            // (a handle in this hand: the fingers close around it, not through it, and the forefinger with them)
            if (grip_frame[side] != ~0ull && st_frames - std::min(grip_frame[side], st_frames) < 6) {
                sq = std::min(sq, (float)g_hold_curl);
                tr = std::clamp(std::max(tr * (float)g_hold_curl, sq), 0.0f, std::max(sq, 0.75f * (float)g_hold_curl + 0.25f * tr));
            }
            const float th = std::max(tr, sq);
            curl[0][0] = 0.10f + 0.35f * th, curl[0][1] = 0.10f + 0.35f * th, curl[0][2] = 0.15f + 0.50f * th;
            curl[1][0] = 0.20f + 1.05f * tr, curl[1][1] = 0.30f + 1.25f * tr, curl[1][2] = 0.20f + 0.75f * tr;
            for (int f = 2; f < 5; ++f) curl[f][0] = 0.25f + 1.10f * sq, curl[f][1] = 0.35f + 1.25f * sq, curl[f][2] = 0.25f + 0.75f * sq;
        }
        for (int b : a.all) {
            if (b == a.arm1 || b == a.arm2 || b == a.wrist) continue;
            const Bone& bn = s.bones[b];
            X l = bn.local;
            for (int f = 0; f < 5; ++f)
                for (int j = 0; j < 3; ++j)
                    if (a.finger[f][j] == b) l.r = mul(l.r, about(mul(transpose(bn.rest.r), a.flex[f]), curl[f][j]));
            for (int t = 0; t < 2; ++t)
                if (a.twist[t] == b && bn.parent == a.arm2) l.r = mul(l.r, about(unit(mul(transpose(bn.local.r), along)), turn * a.twist_part[t]));
            w[b] = mul(w[bn.parent], l);
        }
    }
    if (so.ok[0] || so.ok[1]) last_k = k_free;
    return &so;
}

} // namespace

bool pose(int model, const Frame& fr, const float offset[3], const uint8_t* in, uint8_t* out, uint32_t n, int body) {
    const Solved* so = solve(model, fr, offset, in, n, body);
    if (!so || !out || (!so->ok[0] && !so->ok[1])) return false;
    const Skel& s = skels[model];
    const V3 off{offset[0], offset[1], offset[2]};
    std::memcpy(out, in, (size_t)n * 48);
    auto put = [&](int bone, X m) {
        const int i = bone >= 0 ? index_of(s, bone) : -1;
        if (i < 0 || (uint32_t)i >= n) return;
        m.t = m.t - off;
        if (s.bones[bone].smooth >= 0) m = mul(m, s.inv[s.bones[bone].smooth]);
        to_rows(m, (float*)(out + (size_t)i * 48));
    };
    if (so->torso_ok && s.of_arm.size() == s.bones.size())
        for (size_t b = 0; b < s.bones.size(); ++b)
            if (!s.of_arm[b]) put((int)b, mul(so->torso, s.bones[b].rest));
    for (int side = 0; side < 2; ++side) {
        if (!so->ok[side]) {
            // (a hand that is not followed: its arm is drawn as nothing, gathered at its shoulder; left where the game
            // has it, it showed small and elsewhere, and what joins it to the shoulder was stretched all the way there)
            if (so->torso_ok) {
                const Arm& a = s.arm[side];
                X gone;
                gone.r = scaled(gone.r, 0.0), gone.t = mul(so->torso, s.bones[a.arm1].rest).t;
                for (int b : a.all) put(b, gone);
            }
            continue;
        }
        const Arm& a = s.arm[side];
        for (int b : a.all) put(b, so->w[b]);
        // (alone, without the rest of the skeleton under the head: what joins the arm to the body follows the arm, or
        // the skin there is torn away. With it: the collar bone stays with the chest, as a shoulder does.)
        if (body != 0 && !so->torso_ok)
            for (int b : {a.clav, a.clav_assist})
                if (b >= 0) put(b, mul(so->w[a.arm1], mul(inverse(s.bones[a.arm1].rest), s.bones[b].rest)));
    }
    st_posed++;
    return true;
}

// ---- what Link holds or wears ----
namespace {
struct Track {
    int cand = -1;               // the bone it may be fastened to: 0 left hand, 1 right hand, 2 waist, 3 head
    int cls = -1;                // once sure: the same numbers
    X rel;                       // its place against that bone
    V3 bone_at;
    M3 bone_r0;                  // how the bone was turned when the thing was first seen against it
    double moved = 0;            // how far the bone has travelled while the thing kept its place against it
    double turned = 0;           // how far it has turned meanwhile (radians)
    uint64_t frame = ~0ull;
    int verdict = 0;
    X out;
    int nature = 0;              // what it is, once and for all: 1 drawn like a sword or a shield, 2 taken hold of
    bool taken = false;          // taken hold of (the game's action button), not drawn like a sword: it is "carried"
    int hand = -1;               // taken: the player's hand that took it
    V3 rest_at{0, 0, 0};         // where it was when first seen (a thing lying on the ground)
    bool apart = false;          // it has been seen away from Link's body (what he wears never is)
    bool worn = false;           // it has been seen fastened to his body (his waist, his head): his sword at rest, his shield
    bool by_him = false;         // at this frame it is within 40 cm of his hands, his waist or his head
    V3 let_go{0, 0, 0};          // just let go: how far the hand was from where the game has it, brought back to nothing
    int let_go_n = 0;            // over these frames (a thrown pot leaves the hand, it does not jump to above Link's head)
    // A thing held with both hands (the string of a bow, the pouch of a slingshot): the parts of it that the game has
    // at Link's other hand go with the player's other hand, so that the string is drawn between the player's two hands.
    bool two = false;
    V3 other_at{0, 0, 0};        // where the game has Link's other hand
    double other_reach = 0;      // a part this far from its place at rest: the thing is drawn out
    X other_d;                   // the move that takes such a part to the player's other hand
    int said = 0;                // (study) its parts were written to the journal this many times
    bool strung = false;         // a part of it was in the other hand at the last frame (it is let go a little farther)
    int64_t born = 0;            // when it was first seen
    int parts = 0;               // how many parts its own block has (0: not known; 1: one piece)
    bool palm = false;           // in Link's right hand, it came out while the game aimed: a thing with a handle (see attached())
    int held_cls = -1;           // drawn like a sword: the hand it was last held in,
    X held_rel;                  // and its place against that hand then
    // Which bone it goes with, judged again all along (see attached()): where it was against his two hands and his
    // waist at the last check, and where his hands were against his waist then.
    bool chk = false;
    X chk_rel[3];
    V3 chk_at[2];
    bool worn_known = false;     // its place at rest on his body is known,
    X worn_rel;                  // against his waist
    bool pulled = false;         // at this frame it is shown in the hand that has just drawn it from his back
};
struct TrackKey {
    uint64_t id;
    Track t;
};
std::vector<TrackKey> tracks;
uint64_t carried_frame = ~0ull, in_hand_frame[2] = {~0ull, ~0ull}, strung_frame = ~0ull;
uint64_t st_held = 0, st_hidden = 0, st_taken = 0, st_strings = 0, st_again = 0, st_between = 0, st_moved_on = 0, st_far = 0;
// The last drawing out of a thing held with both hands: the shear, about what point, at which frame, in which of
// Link's hands the thing is (an arrow that rests on that hand is laid along the same line).
M3 last_shear;
V3 last_shear_at{0, 0, 0};
uint64_t last_shear_frame = ~0ull, st_laid = 0;
int last_shear_cls = -1;
// What the player's commands last said (written by gestures(), on the game's input thread): when the action button
// was last pressed, when any other button was (a sword, a shield or another thing is drawn by one of them), and the
// hand that last closed while reaching.
std::atomic<int64_t> action_t{0}, drawn_t{0}, reach_t{0}, item_t{0};   // (item_t: the button of the thing Link uses, ZR)
std::atomic<int64_t> aims_t{0};   // when the game was last seen to aim a thing (gestures(), from the VR mode's own view of its camera)
// when the player last drew out what he took hold of on his back, for each of Link's own hands (0 left, 1 right)
std::atomic<int64_t> pull_t[2] = {};
constexpr int64_t kPullNs = 1100000000ll;
std::atomic<int> reach_side{-1};
constexpr int kLetGoFrames = 14;
}   // namespace

int attached(int model, uint64_t id, const double world[12], const Frame& fr, const float offset[3], const uint8_t* in, uint32_t n,
             int body, double out[12]) {
    const Solved* so = solve(model, fr, offset, in, n, body);
    if (!so) return 0;
    const Skel& s = skels[model];
    X item;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) item.r.m[i][j] = world[i * 4 + j];
        (i == 0 ? item.t.x : i == 1 ? item.t.y : item.t.z) = world[i * 4 + 3];
    }
    Track* t = nullptr;
    for (TrackKey& k : tracks)
        if (k.id == id) t = &k.t;
    if (!t) {
        if (tracks.size() >= 192) {   // (full: the one not seen for the longest goes, a thing that is held or worn last)
            size_t worst = 0;
            auto age = [&](const Track& x) { return (x.cls >= 0 ? 0ull : 1ull << 40) + (so->frame - std::min(x.frame, so->frame)); };
            for (size_t i = 1; i < tracks.size(); ++i)
                if (age(tracks[i].t) > age(tracks[worst].t)) worst = i;
            tracks.erase(tracks.begin() + (std::ptrdiff_t)worst);
        }
        tracks.push_back({id, Track{}});
        t = &tracks.back().t;
    }
    if (t->frame == so->frame) {   // (the same thing again in this frame: its other passes)
        if (t->verdict == 1) to_rows(t->out, out);
        return t->verdict;
    }
    const bool fresh = t->frame == ~0ull || so->frame - t->frame > 20;
    {   // (study: why a thing that was held is no longer)
        static int said = 0;
        if (fresh && t->frame != ~0ull && t->cls >= 0 && t->cls < 2 && said < 12)
            said++, log("vr: a thing (%016llx) held in a hand was not drawn for %llu frames", (unsigned long long)id, (unsigned long long)(so->frame - t->frame));
    }
    t->frame = so->frame;
    if (fresh) t->cand = t->cls = -1, t->moved = 0, t->taken = false, t->let_go_n = 0, t->rest_at = item.t, t->apart = false, t->born = now_ns();
    const X bones[4] = {so->game_wrist[0], so->game_wrist[1], so->game_waist, so->game_head};
    const double arm = s.arm[0].len1 + s.arm[0].len2;
    const double radius[4] = {arm * 1.15, arm * 1.15, arm * 1.0, arm * 0.7};
    // (a thing of the world, not one he wears: at some time it was more than 40 cm from his hands, his waist and his head.
    // Found: after an action button, what he wears on his back was "lifted" when he fell with his arms up.)
    {
        double least = 1e9;
        for (const X& b : bones) least = std::min(least, len(item.t - b.t));
        t->by_him = least <= 0.40;   // (game units: his own body's measure, whatever the player's size)
        if (!t->by_him) t->apart = true;
    }
    auto against = [&](int b) { return mul(inverse(bones[b]), item); };
    auto drift = [&](int b) {
        const X r = against(b);
        double turn = 0;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) turn = std::max(turn, std::fabs(r.r.m[i][j] - t->rel.r.m[i][j]));
        return std::max(len(r.t - t->rel.t), turn * arm * 0.5);
    };
    // (let go, or put away: it has left the bone's reach. Not "it moved against the bone": the game itself moves a
    // sword in the hand during a strike, and it was taken for let go, then taken again, several times a strike)
    if (t->cls >= 0 && len(item.t - bones[t->cls].t) > radius[t->cls] * 1.4) {
        static int said = 0;
        if (t->cls < 2 && said < 12)
            said++, log("vr: a thing (%016llx) held in a hand is now %.2f from it (reach %.2f): let go", (unsigned long long)id, len(item.t - bones[t->cls].t), radius[t->cls] * 1.4);
        if (t->cls < 2 && t->taken && t->verdict == 1) t->let_go = t->out.t - item.t, t->let_go_n = kLetGoFrames;
        t->cls = -1, t->cand = -1, t->taken = false;
    }
    // A thing drawn like a sword that is seen again where it was last held, against the same hand, is held again at
    // once. (Seen with the slingshot: at each shot the game stops drawing it for a moment; found
    // again, it had to "move with the wrist" once more before it came back to the player's hand, and Link standing
    // still to aim, it stayed out of sight until the next shot.)
    // (also from his back. The child's sword at rest is never far enough from his waist to be
    // "let go" by it; back in the hand at the place it rests there, it is held at once.)
    auto near_to = [&](const X& r, const X& ref, double within) {
        double turn = 0;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) turn = std::max(turn, std::fabs(r.r.m[i][j] - ref.r.m[i][j]));
        return std::max(len(r.t - ref.t), turn * arm * 0.5) < within;
    };
    t->pulled = false;
    if (t->nature == 1 && t->held_cls >= 0 && !t->taken) {
        const bool at_hand = near_to(against(t->held_cls), t->held_rel, arm * 0.12);
        if (t->cls != t->held_cls && at_hand) {
            t->cls = t->cand = t->held_cls, t->rel = t->held_rel, t->taken = false, t->hand = -1, t->chk = false;
            st_again++;
        } else if (!at_hand && (t->cls == 0 || t->cls == 1) && t->worn_known && near_to(against(2), t->worn_rel, arm * 0.10)) {
            // (and the other way: back at the place it rests on his body, it is put away at once)
            t->cls = t->cand = 2, t->rel = t->worn_rel, t->chk = false;
            st_moved_on++;
        }
        // The player has just drawn it from his back (gestures(): the hand closed there, then came out in front): it is
        // in his hand from that moment, as it rests there, while the game plays Link's own drawing of it.
        const int64_t pt = pull_t[t->held_cls].load(std::memory_order_relaxed);
        if (pt && now_ns() - pt < kPullNs && t->by_him && body) {
            if (t->cls != t->held_cls) t->cls = t->cand = t->held_cls, t->rel = t->held_rel, t->chk = false;
            t->pulled = !at_hand;
        }
    }
    if (t->cls < 0) {
        if (t->cand >= 0 && drift(t->cand) > arm * 0.10) t->cand = -1;           // (it does not keep its place against that bone)
        if (t->cand < 0) {
            double best = 1e9;
            for (int b = 0; b < 4; ++b) {
                const double d = len(item.t - bones[b].t) / radius[b];
                if (d < 1.0 && d < best) best = d, t->cand = b;
            }
            if (t->cand >= 0) t->rel = against(t->cand), t->bone_at = bones[t->cand].t, t->bone_r0 = bones[t->cand].r, t->moved = 0, t->turned = 0;
        } else {
            t->moved += len(bones[t->cand].t - t->bone_at), t->bone_at = bones[t->cand].t;
            {
                const M3 d = mul(transpose(t->bone_r0), bones[t->cand].r);
                t->turned = std::max(t->turned, std::acos(std::clamp((d.m[0][0] + d.m[1][1] + d.m[2][2] - 1.0) * 0.5, -1.0, 1.0)));
            }
            // (in a hand: the thing must also have followed the hand's turning, or be carried a long way: something
            // that merely walks beside Link, seen at the castle town, keeps its distance but does not turn
            // with his wrist)
            bool sure = t->cand < 2 ? (t->moved > arm * 0.30 && t->turned > 0.30) || t->moved > arm * 1.5 : t->moved > arm * 0.30;
            // Taken hold of: Link lifts what he takes above his head and then stands still, so nothing "moves with his
            // wrist" (seen with a rock of the forest: lifted by the gesture, never shown in the hand).
            // After the action button, a thing that lay still, has left its place and is now held up in his hands,
            // above his head, is taken.
            const int64_t now = now_ns(), a = action_t.load(std::memory_order_relaxed);
            const bool pressed = a && now - a < 4000000000ll && a > drawn_t.load(std::memory_order_relaxed);
            const bool lifted = t->cand < 2 && t->nature != 1 && t->apart && pressed && len(item.t - t->rest_at) > 0.25 &&
                                item.t.y > so->game_head.t.y - 0.05;
            // A thing that comes out with the item's button and is held up over his head (a bomb): taken hold of as well,
            // though it never lay anywhere: it is carried in the player's hand, and thrown by opening that hand. A
            // thing that comes out with that button is given a moment before it is called "drawn like a sword" (a
            // bomb passes by his hands on its way up).
            const int64_t it = item_t.load(std::memory_order_relaxed);
            const bool came_out = g_item_taken && it && now - it < 4000000000ll && t->born > it - 200000000ll && !t->nature;
            // (over his head, between his two hands: a bow raised to aim is as high, but out in front, in one hand.
            // Seen: the adult's bow was "taken hold of" and came to the wrong hand.)
            const V3 over = item.t - so->game_head.t;
            const bool held_up = came_out && t->cand < 2 && now - t->born > 300000000ll && over.y > -0.05 &&
                                 std::sqrt(over.x * over.x + over.z * over.z) < arm * 0.6 && len(item.t - bones[0].t) < radius[0] &&
                                 len(item.t - bones[1].t) < radius[1];
            if (came_out && !held_up && now - t->born < 700000000ll) sure = false;
            sure = sure || lifted || held_up;
            if (sure) {
                t->cls = t->cand;
                if (t->cls == 2) t->worn_known = true, t->worn_rel = t->rel;   // (its place at rest on his body)
                // Taken hold of, or drawn? The game's action button takes a thing from the ground; a sword comes with
                // the strike button. What was taken is carried by the hand that closed on it (the player's own hand,
                // whichever of Link's the game uses), and can be thrown or put down by opening that hand.
                // A thing keeps its nature (the shield is found again many times; it is never "taken hold of").
                if (t->cls < 2 && !t->nature) t->nature = pressed || held_up ? 2 : 1;
                t->taken = t->cls < 2 && t->nature == 2;
                if (t->cls < 2 && t->nature == 1) t->held_cls = t->cls, t->held_rel = t->rel;
                t->hand = -1;
                if (t->taken) {
                    const int rs = reach_side.load(std::memory_order_relaxed);
                    t->hand = rs >= 0 && now - reach_t.load(std::memory_order_relaxed) < 4000000000ll ? rs : g_main;
                    st_taken++;
                }
                static int said = 0;
                if (said++ < 24) {
                    static const char* const where[4] = {"his left hand", "his right hand", "his waist", "his head"};
                    const V3 r = t->rel.t;
                    log("vr: a thing (%016llx) keeps its place against %s (%.2f %.2f %.2f from it): %s", (unsigned long long)id, where[t->cls], r.x, r.y, r.z,
                        t->cls >= 2 ? (body ? "left out with his body" : "left as it is")
                                    : !t->taken ? "it follows the player's hand"
                                                : t->hand ? "taken hold of: carried by the player's right hand" : "taken hold of: carried by the player's left hand");
                }
            }
        }
    }
    // Where it is now, judged again all along. Link the child is small: his sword at rest on his back is within the
    // "reach" of his hand, so once drawn it was never seen to be put away, and it was laid on the player's hand as far
    // from it as his back is from his own hand (found, once the game no longer left it undrawn: "in
    // the grip of Link's left hand: 0.39 towards the wrist"). What a thing goes with is told by movement: each time one
    // of his hands has travelled against his waist, the thing has kept its place against the bone it is fastened to,
    // and not against the others. Worn and gone with a hand: it is in that hand. Held and stayed with his waist: it is
    // put away. (A strike moves the sword a little in the hand, and far from the waist: it stays held.)
    if (t->cls >= 0 && t->cls <= 2 && !t->taken) {
        const X inv_w = inverse(bones[2]);
        const V3 at[2] = {mul(inv_w, bones[0]).t, mul(inv_w, bones[1]).t};
        if (!t->chk) {
            for (int b = 0; b < 3; ++b) t->chk_rel[b] = against(b);
            t->chk_at[0] = at[0], t->chk_at[1] = at[1], t->chk = true;
        } else {
            auto since = [&](int b) {
                const X r = against(b);
                double turn = 0;
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j) turn = std::max(turn, std::fabs(r.r.m[i][j] - t->chk_rel[b].r.m[i][j]));
                return std::max(len(r.t - t->chk_rel[b].t), turn * arm * 0.5);
            };
            const double tr[2] = {len(at[0] - t->chk_at[0]), len(at[1] - t->chk_at[1])};
            const double apart = len((at[0] - at[1]) - (t->chk_at[0] - t->chk_at[1]));
            const double far = arm * 0.25, still = arm * 0.08, gone = arm * 0.20;
            int to = -1;
            bool judged = false;
            if (t->cls == 2) {
                for (int h = 0; h < 2; ++h)
                    if (tr[h] > far) {
                        judged = true;
                        if (since(h) < still && since(2) > gone) to = h;
                    }
            } else {
                if (tr[t->cls] > far) {
                    judged = true;
                    if (since(2) < still && since(t->cls) > gone) to = 2;
                    else if (since(t->cls) < still && t->nature == 1) t->held_cls = t->cls, t->held_rel = against(t->cls);   // (its place at rest in that hand)
                }
                if (to < 0 && apart > far) {
                    judged = true;
                    if (since(1 - t->cls) < still && since(t->cls) > gone) to = 1 - t->cls;
                }
            }
            if (to >= 0) {
                static int said = 0;
                static const char* const where[3] = {"his left hand", "his right hand", "his body"};
                if (said++ < 24) log("vr: a thing (%016llx) was with %s and now goes with %s", (unsigned long long)id, where[t->cls], where[to]);
                t->cls = t->cand = to, t->rel = against(to), t->hand = -1;
                if (to < 2) {
                    if (!t->nature) t->nature = 1;
                    if (t->nature == 1) t->held_cls = to, t->held_rel = t->rel;
                } else {
                    t->worn = true, t->worn_known = true, t->worn_rel = t->rel;
                }
                st_moved_on++;
            }
            if (judged) t->chk = false;   // (judged on this stretch: the next one starts from here)
        }
    } else {
        t->chk = false;
    }
    t->verdict = 0;
    if (t->cls < 0 && t->let_go_n > 0) {   // (just let go: from the hand to where the game has it, quickly)
        const double k = (double)t->let_go_n / kLetGoFrames;
        t->let_go_n--;
        t->out = item;
        t->out.t = item.t + t->let_go * (k * k);
        to_rows(t->out, out);
        t->verdict = 1;
        return 1;
    }
    // (held like a sword, but for now much farther from his hand than it rests in it: on its way to his back, before the
    // rule above has seen it there. Left out with his body rather than drawn that far from the player's hand.)
    const bool far_off = (t->cls == 0 || t->cls == 1) && body && t->nature == 1 && !t->taken && t->held_cls == t->cls && !t->pulled &&
                         len(against(t->cls).t) > len(t->held_rel.t) + arm * 0.40;
    if (far_off) {
        t->verdict = 2, st_far++;
    } else if (t->cls == 0 || t->cls == 1) {
        const int ours = t->taken && t->hand >= 0 ? t->hand : g_swap && body ? 1 - t->cls : t->cls;
        if (so->ok[ours]) {
            // the thing against the hand that holds it, laid on the player's hand the same way (through the two hands'
            // grip frames, which mirror each other: a sword held in Link's left hand sits right in the player's right)
            auto grip_of = [&](int side, const X& wrist) {
                const Arm& a = s.arm[side];
                X g;
                g.r = mul(transpose(s.bones[a.wrist].rest.r), a.grip), g.t = a.palm;
                return mul(wrist, g);
            };
            X hand = grip_of(ours, so->w[s.arm[ours].wrist]);
            if (t->taken) {   // (a sword is drawn at the hand's size, so that its hilt fits; a rock keeps the world's size)
                const double k = std::sqrt(hand.r.m[0][0] * hand.r.m[0][0] + hand.r.m[1][0] * hand.r.m[1][0] + hand.r.m[2][0] * hand.r.m[2][0]);
                if (k > 1e-6) hand.r = scaled(hand.r, 1.0 / k);
            }
            // The grip frame's x axis runs across the palm the same way for both hands (as OpenXR has it): out of the
            // back of the right hand, out of the palm of the left. A handle that lies in the palm of one of Link's
            // hands, laid with the same numbers on the player's other hand, was put behind the back of that hand:
            // with the slingshot and the bow the hand did not hold the handle, it sat beside it, shifted to the
            // right. Passed to the other hand, the thing's place is mirrored across the
            // palm (its place only: the thing itself is not turned inside out).
            // (just drawn from his back by the player: as it rests in that hand, not where Link's own drawing has it)
            const X shown = t->pulled ? mul(bones[t->cls], t->held_rel) : item;
            X in_grip = mul(inverse(grip_of(t->cls, bones[t->cls])), shown);
            // Not Link's shield: it is drawn as several pieces that do not share one origin (its plate, its back, its
            // strap), and with each piece's place mirrored on its own the shield came apart in front of the player's
            // face (with vr_palm_mirror = 0 it was whole again).
            // What is mirrored: the things in Link's left hand (his sword and the like: one handle), and, in his right
            // hand, those that came out while the game aimed (the slingshot, the bow, the arrow on it: their pieces
            // share one origin, measured). The shield, on his right arm when the sword is out, stays as it was.
            if (t->cls == 1 && !t->palm && now_ns() - aims_t.load(std::memory_order_relaxed) < 400000000ll && now_ns() - t->born < 3000000000ll)
                t->palm = true;
            const bool shield = t->cls == 1 && !t->palm;
            if (ours != t->cls && g_palm_mirror && !shield) in_grip.t.x = -in_grip.t.x;
            t->out = mul(hand, in_grip);
            {
                static int said = 0;
                if (!t->taken && said < 10 && (said == 0 || st_frames % 300 == 0))
                    said++, log("vr: a thing (%016llx, %d parts) in the grip of Link's %s hand: %.3f across the palm, %.3f towards the wrist, %.3f towards the little finger; laid on the player's %s hand%s",
                                (unsigned long long)id, t->parts, t->cls ? "right" : "left", mul(inverse(grip_of(t->cls, bones[t->cls])), item).t.x, in_grip.t.y, in_grip.t.z, ours ? "right" : "left",
                                shield ? " as it is (a shield)" : ours != t->cls ? ", on the palm's side" : "");
            }
            to_rows(t->out, out);
            t->verdict = 1, st_held++;
            t->two = false;
            if (g_two_hands && !t->taken && so->ok[1 - ours]) {
                const int oc = 1 - t->cls, oo = 1 - ours;
                t->two = true;
                t->other_at = mul(bones[oc].r, s.arm[oc].palm) + bones[oc].t;
                t->other_reach = arm * 0.55;   // (how far from its place at rest a part is when the thing is fully "drawn out")
                t->other_d = mul(grip_of(oo, so->w[s.arm[oo].wrist]), inverse(grip_of(oc, bones[oc])));
            }
            if (t->taken) carried_frame = so->frame;   // (carried: what the hand may throw or put down)
            else in_hand_frame[ours] = so->frame, grip_frame[ours] = so->frame;   // (held like a sword: that hand is not free)
        }
    } else if ((t->cls == 2 || t->cls == 3) && body) {
        t->verdict = 2, st_hidden++, t->worn = true;
    } else if (t->cls < 0 && body && (t->worn || t->nature == 1) && t->by_him) {
        // Between two places: what he wears or holds, on its way from his back to his hand (or found again and not
        // yet known to be in his hand), is at his own body, which is out of the view: it was drawn there, under
        // the player's head: the sword seemed lost, and lay down below. Left out with his
        // body until it is known to be in a hand. A thing that leaves him (a boomerang thrown) is drawn again as
        // soon as it is 40 cm away.
        t->verdict = 2, st_between++;
    }
    return t->verdict;
}

int attached_rigid(int model, uint64_t id, const double world[12], const Frame& fr, const float offset[3], const uint8_t* in, uint32_t n,
                   int body, double out[12]) {
    const int v = attached(model, id, world, fr, offset, in, n, body, out);
    if (v != 1 || last_shear_frame == ~0ull) return v;
    const Track* t = nullptr;
    for (const TrackKey& k : tracks)
        if (k.id == id) t = &k.t;
    // (drawn out at this frame or the one before: the arrow may be drawn before its bow)
    if (!t || t->taken || t->cls != last_shear_cls || t->frame < last_shear_frame || t->frame - last_shear_frame > 2) return v;
    X o;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) o.r.m[i][j] = out[i * 4 + j];
        (i == 0 ? o.t.x : i == 1 ? o.t.y : o.t.z) = out[i * 4 + 3];
    }
    o.r = mul(last_shear, o.r), o.t = mul(last_shear, o.t - last_shear_at) + last_shear_at;
    to_rows(o, out);
    st_laid++;
    return v;
}

int attached_block(int model, uint64_t id, const Frame& fr, const float offset[3], const uint8_t* link_in, uint32_t link_n, int body,
                   const float thing_offset[3], const uint8_t* in, uint8_t* out, uint32_t n) {
    if (!in || !out || !n) return 0;
    const V3 off{thing_offset[0], thing_offset[1], thing_offset[2]};
    X item = of_rows((const float*)in);
    item.t = item.t + off;
    double w[12], o[12];
    to_rows(item, w);
    for (TrackKey& k : tracks)   // (how many parts it has: told before it is placed)
        if (k.id == id) k.t.parts = (int)n;
    const int v = attached(model, id, w, fr, offset, link_in, link_n, body, o);
    for (TrackKey& k : tracks)   // (a thing seen for the first time: known for the next frame)
        if (k.id == id) k.t.parts = (int)n;
    if (v == 1) {
        X to;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) to.r.m[i][j] = o[i * 4 + j];
            (i == 0 ? to.t.x : i == 1 ? to.t.y : to.t.z) = o[i * 4 + 3];
        }
        const X d = mul(to, inverse(item));   // (the move that takes the thing from where the game has it to the hand)
        Track* tr = nullptr;
        for (TrackKey& k : tracks)
            if (k.id == id) tr = &k.t;
        if (tr && tr->two && n > 1 && g_parts_study && tr->said < 2 && (tr->said == 0 || st_frames % 240 == 0)) {   // (study: the thing's parts)
            tr->said++;
            std::string line;
            char b[96];
            for (uint32_t i = 0; i < n && i < 28; ++i) {
                X m = of_rows((const float*)(in + (size_t)i * 48));
                m.t = m.t + off;
                std::snprintf(b, sizeof b, " %u:(%.3f from its place at rest)", i, len(m.t - item.t));
                line += b;
            }
            log("vr: study: the thing %016llx has %u parts; the other hand is %.3f from its root, reach %.3f;%s", (unsigned long long)id, n,
                len(tr->other_at - item.t), tr->other_reach, line.c_str());
        }
        // A thing held with both hands (the pouch of a slingshot, the string of a bow): when the game has it drawn out
        // towards Link's other hand, it is drawn out between the player's two hands instead. The block holds, for each
        // part, the move from the thing's shape at rest to its place in the world, not the part's own place (measured
        // on the slingshot: parts that have not moved all read as the root). So nothing is asked of
        // the parts one by one: the whole drawn thing is sheared, a point moving by its share of the way from the
        // thing's root to where the game's other hand would be, times the way from there to the player's other hand.
        // The wood at the root stays as it is, what is in the other hand comes to the player's, and a straight elastic
        // in between stays straight. "Drawn out": a part is far from its place at rest, towards the other hand.
        // Only while the game aims a thing: a sword and its sheath are one figure of several parts too, and with the
        // sword in one hand and the sheath on the back, the sheath is "far from its place at rest, towards the other
        // hand" (seen: the sword was sheared into rags at each draw).
        bool string = false;
        M3 shear;
        V3 at{0, 0, 0};
        if (tr && tr->two && n > 1 && now_ns() - aims_t.load(std::memory_order_relaxed) < 400000000ll) {
            const V3 axis = tr->other_at - item.t;
            const double reach = len(axis);
            double far = 0;
            for (uint32_t i = 1; i < n; ++i) {
                const X m = of_rows((const float*)(in + (size_t)i * 48));
                const V3 moved = m.t + off - item.t;
                if (dot(moved, axis) > 0.6 * len(moved) * reach) far = std::max(far, len(moved));
            }
            const double k = reach > 0.08 ? std::clamp((far - 0.3 * tr->other_reach) / (0.7 * tr->other_reach), 0.0, 1.0) : 0.0;
            string = k > 0;
            tr->strung = string;
            if (string) {
                strung_frame = tr->frame;
                at = mul(d.r, item.t) + d.t;
                const V3 was = mul(d.r, tr->other_at) + d.t, is = mul(tr->other_d.r, tr->other_at) + tr->other_d.t;
                const double l = std::max(len(was - at), 1e-6);
                const V3 share = (was - at) * (1.0 / (l * l)), way = (is - was) * k;   // (a point's share of the way: (x - at) . share)
                const double wv[3] = {way.x, way.y, way.z}, sv[3] = {share.x, share.y, share.z};
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c) shear.m[r][c] += wv[r] * sv[c];
                last_shear = shear, last_shear_at = at, last_shear_frame = tr->frame, last_shear_cls = tr->cls;
            }
        }
        for (uint32_t i = 0; i < n; ++i) {
            X m = of_rows((const float*)(in + (size_t)i * 48));
            m.t = m.t + off;
            m = mul(d, m);
            if (string) m.r = mul(shear, m.r), m.t = mul(shear, m.t - at) + at;
            m.t = m.t - off;
            to_rows(m, (float*)(out + (size_t)i * 48));
        }
        if (string && st_strings++ < 3) log("vr: a part of the thing in one hand is in Link's other hand (a string, a pouch): it goes with the player's other hand");
    } else if (v == 2) {
        std::memset(out, 0, (size_t)n * 48);
    }
    return v;
}

bool carrying() {
    std::lock_guard lk(mu);
    return carried_frame != ~0ull && st_frames - carried_frame < 6;
}
bool two_handed() {
    std::lock_guard lk(mu);
    return strung_frame != ~0ull && st_frames - strung_frame < 6;
}
bool held(uint64_t id) {
    std::lock_guard lk(mu);
    for (const TrackKey& k : tracks)
        if (k.id == id) return k.t.verdict == 1 && k.t.frame != ~0ull && st_frames - std::min(k.t.frame, st_frames) < 6;
    return false;
}
namespace {
bool in_hand(int side) {
    std::lock_guard lk(mu);
    return in_hand_frame[side] != ~0ull && st_frames - in_hand_frame[side] < 6;
}
}   // namespace

// ---- the player's movements as the game's commands ----
namespace {
enum { PAD_SOUTH, PAD_EAST, PAD_WEST, PAD_NORTH, PAD_BACK, PAD_GUIDE, PAD_START, PAD_LSTICK, PAD_RSTICK, PAD_LSHOULDER,
       PAD_RSHOULDER, PAD_UP, PAD_DOWN, PAD_LEFT, PAD_RIGHT, PAD_LTRIGGER, PAD_RTRIGGER };   // (the host's positions)
struct Gest {
    bool armed = true, squeezed[2] = {}, grabbed[2] = {}, pinched[2] = {};
    bool reached[2] = {};        // the hand closed while reaching: a taking hold, sure only once a thing is in the hands
    int64_t reached_t[2] = {};
    int64_t b_until = 0, a_until = 0, a_from = 0, forward_until = 0, last_strike = 0, away_until = 0, open_since = 0;
    bool away_done = true;
    uint64_t aways = 0;
    double peak[2] = {};         // the hand's fastest of the last moments
    int64_t peak_t[2] = {};
    V3 peak_v[2] = {};           // and which way it went then
    double throw_x = 0, throw_y = 1;   // the stick while the thing is thrown: the way the hand went
    uint64_t prev_buttons = 0;
    double stick_from[3] = {};   // (bare hands) where the left hand's pinch began
    bool stick_on = false;
    double aim_sign[2] = {-1, 1}, aim_prev[2] = {};
    int aim_bad[2] = {};
    uint64_t aim_turned = 0;
    bool string_held = false;
    int64_t shot_until = 0;
    uint64_t shots = 0;
    // a wave of the open hand held up: its swings to and fro, and when the last one was
    int wave_n[2] = {}, wave_way[2] = {};
    int64_t wave_t[2] = {}, wave_done = 0;
    uint64_t waves = 0;
    uint64_t draws = 0;
    int64_t aim_check = 0;
    uint64_t strikes = 0, grabs = 0, throws = 0, drops = 0, aims = 0;
    // The back, over the shoulder: where Link carries his sword and his shield.
    bool at_back[2] = {};        // the hand is there
    bool back_grip[2] = {};      // it closed there, empty: it holds what is on the back, not yet drawn
    bool shield_held = false;    // the other hand took the shield from the back: its button is held while that hand is closed
    bool stowed[2] = {};         // what the hand held was put away on this visit to the back
    bool need_open[2] = {};      // after putting away: the hand must open before it strikes again
    int64_t stow_since[2] = {}, feel_until[2] = {};
    uint64_t unsheathes = 0, shields = 0, stows = 0;
} gs;
}   // namespace

bool gestures_on() { return g_gestures != 0; }
int main_hand() { return g_main; }
void set_gestures(bool on) { g_gestures = on ? 1 : 0; }
bool arms_on() { return g_on != 0; }
void set_arms(bool on) { g_on = on ? 1 : 0; }
int tilt_degrees() { return (int)std::lround(g_tilt); }
void set_tilt(int degrees) { g_tilt = std::clamp(degrees, -90, 90); }
int arm_cm() { return (int)std::lround(g_arm_m * 100.0); }
double link_arm(int model) {   // (shoulder to wrist of Link's own skeleton, game units; 0: not read)
    return model >= 0 && model < 4 && skels[model].ok ? skels[model].arm[0].len1 + skels[model].arm[0].len2 : 0.0;
}
void set_arm_cm(int cm) { g_arm_m = std::clamp(cm, 20, 90) * 0.01; }

void gestures(Moment& m, uint64_t* buttons, int32_t stick[4]) {
    if (!g_gestures || !buttons || !stick || !m.first_person) return;
    Hand h[2];
    const bool ok[2] = {hand_now(0, &h[0]), hand_now(1, &h[1])};
    const int64_t now = now_ns();
    if (g_sim >= 10 && ok[g_main] && h[g_main].trigger > 0.5f) *buttons |= 1ull << g_btn_item;   // (as the real controller's trigger is)
    const M3 head_r = of_quat(m.head_q);
    V3 fwd = mul(head_r, V3{0, 0, -1});
    fwd.y = 0;
    fwd = unit(fwd, {0, 0, -1});
    const V3 right = cross(fwd, V3{0, 1, 0});
    for (int side = 0; side < 2; ++side) {
        if (!ok[side]) {
            gs.squeezed[side] = gs.grabbed[side] = gs.pinched[side] = false;
            continue;
        }
        const Hand& x = h[side];
        const V3 v{x.vel[0], x.vel[1], x.vel[2]}, w{x.ang[0], x.ang[1], x.ang[2]};
        const V3 rel = V3{x.p[0], x.p[1], x.p[2]} - V3{m.head_p[0], m.head_p[1], m.head_p[2]};
        const double speed = len(v);
        if (speed > gs.peak[side] || now - gs.peak_t[side] > 150000000) gs.peak[side] = speed, gs.peak_t[side] = now, gs.peak_v[side] = v;
        const bool sq = x.squeeze > (gs.squeezed[side] ? 0.3f : 0.6f);
        // (reaching: the hand well below the lap of a seated player, or the arm stretched out in front)
        const bool reaching = rel.y < -g_reach_down || dot(rel, fwd) > g_reach_out;
        // The back, over the shoulder: a hand put there that takes hold of the weapon draws it by pulling it out, and,
        // as in BetterVR, a thing held that is brought to the back is put away:
        //  - the empty hand closes there: it holds what is on the back (felt in the controller); drawn out in front
        //    while still closed, the hand of the sword presses the strike button once (Link draws), the other hand
        //    holds the shield's button for as long as it stays closed (Link takes his shield);
        //  - a hand that holds something drawn comes there: Link puts it away, without the hand having to open.
        if (g_back) {
            const double behind = -dot(rel, fwd);
            const bool at_back = behind > (gs.at_back[side] ? -0.08 : 0.02) && rel.y > -0.42 && rel.y < 0.30 && std::fabs(dot(rel, right)) < 0.50;
            gs.at_back[side] = at_back;
            if (!sq) gs.back_grip[side] = false, gs.need_open[side] = false;
            if (!sq && side != g_main) gs.shield_held = false;
            if (at_back && sq && !gs.squeezed[side] && !in_hand(side) && !carrying() && !m.game_aims) gs.back_grip[side] = true, gs.feel_until[side] = now + 90000000;
            if (gs.back_grip[side] && !at_back) {   // (drawn out)
                gs.back_grip[side] = false, gs.feel_until[side] = now + 140000000;
                if (side == g_main) gs.b_until = now + 100000000, gs.last_strike = now, gs.armed = false, gs.unsheathes++;
                else gs.shield_held = true, gs.shields++;
                pull_t[g_swap ? 1 - side : side].store(now, std::memory_order_relaxed);   // (it is in that hand from now: attached())
            }
            // (also while the game aims: the hand that holds the bow, brought to the back, puts it away)
            if (at_back && sq && in_hand(side) && !gs.back_grip[side]) {
                if (!gs.stow_since[side]) gs.stow_since[side] = now;
                if (!gs.stowed[side] && now - gs.stow_since[side] > 120000000) {
                    gs.stowed[side] = true, gs.need_open[side] = true, gs.stows++, gs.feel_until[side] = now + 90000000;
                    if (side != g_main && gs.shield_held) gs.shield_held = false;   // (the shield alone: its button is let go)
                    else gs.away_until = now + 100000000, gs.away_done = true, gs.aways++;
                }
            } else {
                gs.stow_since[side] = 0;
            }
            if (!at_back) gs.stowed[side] = false;
            if (now < gs.feel_until[side]) m.shake[side] = std::max(m.shake[side], 0.35f);
        }
        // a strike: the closed hand that holds the sword moves fast (the middle of a blade half a metre long). With an
        // empty hand nothing (the sword is drawn from the back; vr_strike_draws = 1: a strike draws it, as before).
        if (side == g_main && g_strike > 0 && !gs.grabbed[side]) {
            const V3 blade = mul(of_quat(x.q), V3{0, 0, -0.30});
            const double tip = len(v + cross(w, blade));
            const bool may = !g_back || g_strike_draws || in_hand(side);
            if (gs.armed && sq && may && !m.game_aims && !gs.back_grip[side] && !gs.need_open[side] && tip > g_strike && now - gs.last_strike > 220000000)
                gs.b_until = now + 100000000, gs.last_strike = now, gs.armed = false, gs.strikes++;
            if (now < gs.b_until) m.shake[side] = 0.55f;   // (the strike is felt in the hand)
            if (!gs.armed && tip < g_strike * 0.4) gs.armed = true;
            // the hand that holds the sword opens and stays open: Link puts it away
            if (sq) gs.open_since = 0, gs.away_done = false;
            else if (!gs.open_since) gs.open_since = now;
            if (!sq && !gs.away_done && now - gs.open_since > 450000000 && in_hand(side) && !m.game_aims)
                gs.away_until = now + 100000000, gs.away_done = true, gs.aways++;
        }
        // taking hold: the hand closes while it reaches down or forwards (the game's action button: it takes what is
        // there, if anything). It counts as a hold only once a thing is in the hands; letting go then throws it when
        // the hand is fast, puts it down when it is slow (the game's own rule: thrown when Link moves, put down when
        // he stands).
        if (sq && !gs.squeezed[side] && reaching && !gs.at_back[side] && speed < 1.0 && !carrying() && !in_hand(side) && !m.game_aims) {
            gs.reached[side] = true, gs.reached_t[side] = now, gs.a_from = now, gs.a_until = now + 100000000, gs.grabs++;
            reach_side.store(side, std::memory_order_relaxed), reach_t.store(now, std::memory_order_relaxed);
        }
        if (side == g_main && sq && carrying() && !gs.grabbed[0] && !gs.grabbed[1] && !gs.reached[0] && !gs.reached[1])
            gs.grabbed[side] = true, gs.reached_t[side] = now;   // (a bomb taken out with its button: held by closing the hand)
        if (gs.reached[side]) {
            if (carrying()) gs.grabbed[side] = true, gs.reached[side] = false;
            else if (now - gs.reached_t[side] > 4000000000ll || !sq) gs.reached[side] = false;   // (Link bends and lifts: it takes a moment)
        }
        if (!sq && gs.squeezed[side] && gs.grabbed[side]) {
            gs.grabbed[side] = false;
            if (carrying()) {
                if (gs.peak[side] > g_throw) {
                    // (thrown the way the hand went, seen from above: the game throws the way Link runs)
                    const double tx = dot(gs.peak_v[side], right), ty = dot(gs.peak_v[side], fwd), tn = std::sqrt(tx * tx + ty * ty);
                    gs.throw_x = tn > 0.3 ? tx / tn : 0, gs.throw_y = tn > 0.3 ? ty / tn : 1;
                    gs.forward_until = now + 260000000, gs.a_from = now + 90000000, gs.a_until = now + 190000000, gs.throws++;
                } else {
                    gs.a_from = now, gs.a_until = now + 100000000, gs.drops++;
                }
            }
        }
        if (gs.grabbed[side] && !carrying() && now - gs.reached_t[side] > 5000000000ll) gs.grabbed[side] = false;   // (it broke, or was taken away)
        // A wave: the open hand held up near the face swings to and fro three times, and a character should answer in
        // his own way: the game's own answer to
        // someone who comes up to a character is its action button (he speaks); nothing is made up.
        {
            const double side_v = dot(v, right);
            const bool up_there = rel.y > -0.22 && !sq && dot(rel, fwd) > 0.10 && !m.game_aims;
            const int way = side_v > 0.45 ? 1 : side_v < -0.45 ? -1 : 0;
            if (!up_there || now - gs.wave_t[side] > 700000000) gs.wave_n[side] = 0, gs.wave_way[side] = 0;
            if (up_there && way && way != gs.wave_way[side]) gs.wave_way[side] = way, gs.wave_n[side]++, gs.wave_t[side] = now;
            // (opening the conversation was not what was meant: the characters should greet back with an animation of
            // their own, or the wave is of no use: the wave no longer presses the action
            // button (vr_wave_talk=1: as before); it is counted, for the characters' own greeting, to come)
            if (gs.wave_n[side] >= 4 && now - gs.wave_done > 2500000000) {
                gs.wave_done = now, gs.wave_n[side] = 0, gs.waves++;
                if (g_wave_talk) gs.a_from = now, gs.a_until = now + 100000000;
                wave_t.store(now, std::memory_order_relaxed);
            }
        }
        // the shield: the other hand closed, not reaching for something
        // (not while that hand has just closed on something: Link is lifting it, and the hand comes up closed)
        // (and, with the back in use: only a shield that is out, on the arm because the sword is drawn or just taken
        // from the back; an empty hand that closes in front holds nothing)
        if (side != g_main && sq && !gs.grabbed[side] && !gs.reached[side] && !reaching && !gs.back_grip[side] && !m.game_aims &&
            (!g_back || gs.shield_held || in_hand(side)))
            *buttons |= 1ull << g_btn_shield;
        gs.squeezed[side] = sq;
        // bare hands: the left pinch held and drawn is the stick; the right pinch is the action button
        if (x.fingers) {
            const bool pinch = x.trigger > (gs.pinched[side] ? 0.4f : 0.85f);
            if (side != g_main) {
                if (pinch && !gs.stick_on) gs.stick_on = true, gs.stick_from[0] = x.p[0], gs.stick_from[1] = x.p[1], gs.stick_from[2] = x.p[2];
                if (!pinch) gs.stick_on = false;
                if (gs.stick_on) {
                    const V3 d = V3{x.p[0], x.p[1], x.p[2]} - V3{gs.stick_from[0], gs.stick_from[1], gs.stick_from[2]};
                    double sx = dot(d, right) / 0.07, sy = dot(d, fwd) / 0.07;
                    const double n2 = std::sqrt(sx * sx + sy * sy);
                    if (n2 > 1.0) sx /= n2, sy /= n2;
                    if (n2 > 0.15) stick[0] = (int32_t)std::lround(sx * 32767.0), stick[1] = (int32_t)std::lround(sy * 32767.0);
                }
            } else if (pinch && !gs.pinched[side] && !gs.grabbed[side]) {
                gs.a_from = now, gs.a_until = now + 100000000;
            }
            gs.pinched[side] = pinch;
        }
    }
    if (now < gs.b_until) *buttons |= 1ull << g_btn_strike;
    if (now < gs.away_until) *buttons |= 1ull << g_btn_away;
    if (now >= gs.a_from && now < gs.a_until) *buttons |= 1ull << g_btn_action;
    if (now < gs.forward_until)   // (thrown: the game throws when Link moves, puts down when he stands)
        stick[0] = (int32_t)std::lround(gs.throw_x * 32767.0), stick[1] = (int32_t)std::lround(gs.throw_y * 32767.0);
    // (for the things Link holds: what comes after the action button was taken hold of, what comes after the strike
    // button was drawn; the player's own presses count as much as the gestures')
    {
        const uint64_t pressed = *buttons & ~gs.prev_buttons;   // (presses, not what is held: a shield kept up is not "drawn" again)
        gs.prev_buttons = *buttons;
        const uint64_t not_things = (1ull << PAD_LSTICK) | (1ull << PAD_RSTICK) | (1ull << PAD_START) | (1ull << PAD_BACK) | (1ull << PAD_GUIDE);
        if (pressed & (1ull << g_btn_action)) action_t.store(now, std::memory_order_relaxed);
        if (pressed & (1ull << g_btn_item)) item_t.store(now, std::memory_order_relaxed);
        if (pressed & ~((1ull << g_btn_action) | not_things)) drawn_t.store(now, std::memory_order_relaxed);
    }
    // A bow, a slingshot drawn with both hands. In this remake the thing's button (ZR: the trigger of the hand that
    // draws) held is the string drawn, and letting it go is the shot (read on the game's own controls screen, then
    // tried). Nothing is pressed for the player; the pull is felt in the hand that draws, the more the
    // farther the two hands are apart, and the shot is felt in both.
    if (m.game_aims) {
        aims_t.store(now, std::memory_order_relaxed);
        const bool drawn = (*buttons & (1ull << g_btn_item)) != 0;
        if (drawn) {
            if (!gs.string_held) gs.string_held = true, gs.draws++;
            m.shake[g_main] = std::max(m.shake[g_main], (float)std::clamp(0.08 + std::max(m.pull - 0.15, 0.0) * 1.4, 0.08, 0.6));
        } else if (gs.string_held) {
            gs.string_held = false, gs.shot_until = now + 90000000, gs.shots++;
        }
        if (now < gs.shot_until) m.shake[0] = m.shake[1] = 0.8f;
    } else {
        gs.string_held = false;
    }
    // aiming: the game's own aim is steered to where the hand points, for as long as the game aims (in this remake a
    // thing stays aimed after its button is let go, until Link walks). The game turns its aim with the right stick
    // (measured: 75 degrees a second sideways, 60 up and down, at once): that stick is ours meanwhile.
    if (m.game_aims && ok[g_main]) {
        m.sticks_taken = true;
        const double e[2] = {m.aim_yaw, m.aim_pitch};
        if (now - gs.aim_check > 350000000) {   // (which way the game takes the stick is not known: found by trying)
            for (int k = 0; k < 2; ++k) {
                // (the way is known for the game as it comes: right is right, up is up. A player may have turned the
                // game's camera settings the other way: the aim then runs away from the hand, twice in a row and fast,
                // which no hand does for long: the way is turned)
                if (std::fabs(gs.aim_prev[k]) > 0.10 && std::fabs(e[k]) > std::fabs(gs.aim_prev[k]) + 0.05) gs.aim_bad[k]++;
                else gs.aim_bad[k] = 0;
                if (gs.aim_bad[k] >= 2) gs.aim_sign[k] = -gs.aim_sign[k], gs.aim_bad[k] = 0, gs.aim_turned++;
                gs.aim_prev[k] = e[k];
            }
            gs.aim_check = now;
        }
        for (int k = 0; k < 2; ++k) {
            double c = std::clamp(e[k] * g_aim_gain, -1.0, 1.0);
            if (std::fabs(e[k]) > g_aim_still && std::fabs(c) < g_aim_least) c = c > 0 ? g_aim_least : -g_aim_least;   // (past the stick's dead zone)
            if (std::fabs(e[k]) <= g_aim_still) c = 0;
            stick[g_aim_stick * 2 + k] = (int32_t)std::lround(gs.aim_sign[k] * c * 32767.0);
        }
        gs.aims++;
    } else {
        gs.aim_prev[0] = gs.aim_prev[1] = 0;
    }
}

std::string report_json() {
    Hand h[2];
    const bool ok[2] = {hand_now(0, &h[0]), hand_now(1, &h[1])};
    char b[1800];
    std::snprintf(b, sizeof b,
                  "{\"arms_follow\": %s, \"skeletons\": [%s, %s], \"fake_hands\": %d, \"blocks_posed\": %llu, \"frames\": %llu, \"shoulder_pulled\": %llu, \"arm_drawn_longer\": %llu, "
                  "\"arm_scale\": %.3f, \"elbow_degrees\": [%.0f, %.0f],\n"
                  "      \"left\": {\"tracked\": %s, \"fingers\": %s, \"at_cm\": [%.1f, %.1f, %.1f], \"trigger\": %.2f, \"grip\": %.2f},\n"
                  "      \"right\": {\"tracked\": %s, \"fingers\": %s, \"at_cm\": [%.1f, %.1f, %.1f], \"trigger\": %.2f, \"grip\": %.2f},\n"
                  "      \"things\": {\"drawn_in_a_hand\": %llu, \"left_out_with_the_body\": %llu, \"left_out_on_their_way\": %llu, \"left_out_far_from_the_hand\": %llu, \"changed_place\": %llu, \"followed\": %zu, \"taken_hold_of\": %llu, \"carried_now\": %s},\n"
                  "      \"gestures\": {\"on\": %s, \"strikes\": %llu, \"grabs\": %llu, \"throws\": %llu, \"put_down\": %llu, \"put_away\": %llu, \"aim_steps\": %llu, \"bow_draws\": %llu, \"waves\": %llu, \"aim_signs\": [%.0f, %.0f], "
                  "\"drawn_from_the_back\": %llu, \"shield_from_the_back\": %llu, \"put_away_at_the_back\": %llu}}",
                  g_on ? "true" : "false", skels[0].ok ? "true" : "false", skels[1].ok ? "true" : "false", g_sim, (unsigned long long)st_posed,
                  (unsigned long long)st_frames, (unsigned long long)st_reach, (unsigned long long)st_stretched, last_k, last_elbow[0], last_elbow[1], ok[0] ? "true" : "false",
                  ok[0] && h[0].fingers ? "true" : "false", h[0].p[0] * 100, h[0].p[1] * 100, h[0].p[2] * 100, h[0].trigger, h[0].squeeze,
                  ok[1] ? "true" : "false", ok[1] && h[1].fingers ? "true" : "false", h[1].p[0] * 100, h[1].p[1] * 100, h[1].p[2] * 100, h[1].trigger,
                  h[1].squeeze, (unsigned long long)st_held, (unsigned long long)st_hidden, (unsigned long long)st_between, (unsigned long long)st_far, (unsigned long long)st_moved_on, tracks.size(), (unsigned long long)st_taken, carrying() ? "true" : "false",
                  g_gestures ? "true" : "false", (unsigned long long)gs.strikes, (unsigned long long)gs.grabs, (unsigned long long)gs.throws,
                  (unsigned long long)gs.drops, (unsigned long long)gs.aways, (unsigned long long)gs.aims, (unsigned long long)gs.draws, (unsigned long long)gs.waves, gs.aim_sign[0], gs.aim_sign[1],
                  (unsigned long long)gs.unsheathes, (unsigned long long)gs.shields, (unsigned long long)gs.stows);
    return b;
}

} // namespace rt::hands
