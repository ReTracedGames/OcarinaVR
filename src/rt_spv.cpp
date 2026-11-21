// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2026 the ReTraced project
//
// Word-level SPIR-V edits (no parser of the game's code, only additions at known places).
// Written from the SPIR-V specification (Khronos, unified1): module layout, opcodes and decoration numbers.
#include "rt_spv.h"

#include <algorithm>

namespace rt {
namespace {
enum : uint32_t {
    OpCapability = 17, OpEntryPoint = 15, OpExecutionMode = 16, OpTypePointer = 32, OpVariable = 59, OpDecorate = 71,
    OpMemberDecorate = 72, OpFunction = 54,
    CapTransformFeedback = 53, ModelVertex = 0, ModeXfb = 11, StorageOutput = 3,
    DecBuiltIn = 11, DecOffset = 35, DecXfbBuffer = 36, DecXfbStride = 37, BuiltInPosition = 0,
    // (the mesh-stage edit)
    OpExtension = 10, OpMemoryModel = 14, OpTypeBool = 20, OpTypeInt = 21, OpTypeFloat = 22, OpTypeArray = 28,
    OpTypeRuntimeArray = 29, OpTypeStruct = 30, OpConstant = 43, OpLoad = 61, OpStore = 62, OpAccessChain = 65,
    OpInBoundsAccessChain = 66, OpCompositeExtract = 81, OpConvertUToPtr = 120, OpIAdd = 128, OpIMul = 132,
    OpLogicalAnd = 167, OpSelect = 169, OpIEqual = 170, OpINotEqual = 171, OpULessThanEqual = 178, OpControlBarrier = 224,
    OpAtomicIAdd = 234, OpSetMeshOutputsEXT = 5295,
    CapInt64 = 11, CapPhysicalStorageBuffer = 5347, AddressingPhysical64 = 5348, ModelMeshEXT = 5365,
    StorageWorkgroup = 4, StoragePhysical = 5349, DecBlock = 2, DecArrayStride = 6, BuiltInLocalInvocationId = 27,
    BuiltInPrimitiveTriangleIndices = 5296, AccessAligned = 2, ModeOutputVertices = 26, ModeOutputPrimitives = 5270,
};
} // namespace

// The vertex stage's clip-space position (gl_Position) becomes a transform feedback output: buffer 0, 16 bytes a
// vertex. The module draws exactly as before while no transform feedback is active.
int spv_add_position_xfb(const uint32_t* w, size_t n, std::vector<uint32_t>& out) {
    if (n < 5 || w[0] != 0x07230203u) return 0;
    uint32_t entry = 0, pos_var = 0, pos_struct = 0, pos_member = 0;
    size_t last_cap = 0, last_entry_or_mode = 0, first_deco = 0;
    bool vertex = false;
    for (size_t i = 5; i < n;) {
        const uint32_t op = w[i] & 0xffff, len = w[i] >> 16;
        if (!len || i + len > n) return 0;
        if (op == OpCapability) {
            if (w[i + 1] == CapTransformFeedback) return 0;   // (already a transform feedback program)
            last_cap = i + len;
        } else if (op == OpEntryPoint) {
            if (w[i + 1] == ModelVertex) vertex = true, entry = w[i + 2];
            else return 0;   // (another stage in the module)
            last_entry_or_mode = i + len;
        } else if (op == OpExecutionMode) {
            if (len >= 3 && w[i + 2] == ModeXfb) return 0;
            last_entry_or_mode = i + len;
        } else if (op == OpDecorate) {
            if (!first_deco) first_deco = i;
            if (len >= 4 && w[i + 2] == DecBuiltIn && w[i + 3] == BuiltInPosition) pos_var = w[i + 1];
        } else if (op == OpMemberDecorate) {
            if (!first_deco) first_deco = i;
            if (len >= 5 && w[i + 3] == DecBuiltIn && w[i + 4] == BuiltInPosition) pos_struct = w[i + 1], pos_member = w[i + 2];
        } else if (op == OpFunction) {
            break;
        }
        i += len;
    }
    if (!vertex) return 0;
    bool member = false;
    if (!pos_var && pos_struct) {   // (the position is a member of an output block: the block's variable)
        member = true;
        uint32_t ptr = 0;
        for (size_t i = 5; i < n;) {
            const uint32_t op = w[i] & 0xffff, len = w[i] >> 16;
            if (!len || i + len > n) break;
            if (op == OpTypePointer && len >= 4 && w[i + 2] == StorageOutput && w[i + 3] == pos_struct) ptr = w[i + 1];
            if (op == OpVariable && len >= 4 && ptr && w[i + 1] == ptr && w[i + 3] == StorageOutput) pos_var = w[i + 2];
            if (op == OpFunction) break;
            i += len;
        }
    }
    if (!pos_var || !last_cap || !last_entry_or_mode || !first_deco || !(last_cap <= last_entry_or_mode && last_entry_or_mode <= first_deco))
        return -1;   // (a vertex module we cannot edit: the caller turns the capture off)
    out.clear();
    out.reserve(n + 20);
    auto put = [&](std::initializer_list<uint32_t> v) {
        std::vector<uint32_t> t(v);
        t[0] |= (uint32_t)t.size() << 16;
        out.insert(out.end(), t.begin(), t.end());
    };
    out.insert(out.end(), w, w + last_cap);
    put({OpCapability, CapTransformFeedback});
    out.insert(out.end(), w + last_cap, w + last_entry_or_mode);
    put({OpExecutionMode, entry, ModeXfb});
    out.insert(out.end(), w + last_entry_or_mode, w + first_deco);
    // (a block member takes Offset on the member, a variable of its own on itself)
    if (member) put({OpMemberDecorate, pos_struct, pos_member, DecOffset, 0});
    else put({OpDecorate, pos_var, DecOffset, 0});
    put({OpDecorate, pos_var, DecXfbBuffer, 0});
    put({OpDecorate, pos_var, DecXfbStride, 16});
    out.insert(out.end(), w + first_deco, w + n);
    return 1;
}

// The translator ends every mesh program the same way (read in the game's modules): OpSetMeshOutputsEXT,
// then the 32 invocations of the group copy the meshlet's vertices and triangles into the outputs,
//     gl_Position[v].c = ...        (v: local invocation + 0 / 32 / 64 / 96; the depth is written again as (z + w) / 2)
//     indices[p] = (i0, i1, i2)     (inside "if (p < count)")
// Those stores are mirrored into our buffers. No binding is added (the host builds its layouts from the translator's
// own description of the program): the buffers are reached by their device addresses, two 64-bit constants.
//   words: [0] slots taken so far this frame (atomic), [1] 1 while the scene's depth prepass is drawn, [15] waste;
//          from [16]: three indices a triangle slot, already numbered over the whole buffer
//   clip:  [0..3] waste; from [16]: four floats a vertex slot (clip space, depth as the game writes it)
// A meshlet takes as many slots as its program may output, rounded up to 32, vertices and triangles sharing the
// numbering. The game's programs declare 96, 126, 128 or 160 of each, and the 32 invocations write whole rounds of 32
// vertices: with 126 declared, vertices 126 and 127 are written past the end of the output (found: with 126
// slots those two landed on the first two vertices of the next meshlet, and with 128 slots for all, the programs of
// 160 spilled over: long stray triangles crossed the picture). Invocation 0 reserves the slots with one atomic add
// and hands the base to the others through a workgroup variable and a barrier. With the flag at 0, a meshlet of
// no triangle or the buffers full, every mirrored store lands in the waste words.
int spv_add_mesh_capture(const uint32_t* w, size_t n, uint64_t addr_words, uint64_t addr_clip, uint32_t slots, std::vector<uint32_t>& out) {
    if (n < 5 || w[0] != 0x07230203u) return 0;
    uint32_t bound = w[3];
    if (!bound || bound > (1u << 24)) return 0;
    uint32_t out_v = 0, out_p = 0;   // (the execution modes OutputVertices and OutputPrimitivesEXT)
    size_t last_cap = 0, last_deco = 0, first_fn = 0;
    uint32_t entries = 0, sets = 0;
    bool mesh = false, has_int64 = false;
    uint32_t pos_var = 0, idx_var = 0, lid_var = 0, t_u32 = 0, t_u64 = 0, t_f32 = 0, t_bool = 0;
    for (size_t i = 5; i < n;) {
        const uint32_t op = w[i] & 0xffff, len = w[i] >> 16;
        if (!len || i + len > n) return 0;
        if (op == OpFunction && !first_fn) first_fn = i;
        if (first_fn) {
            if (op == OpSetMeshOutputsEXT) sets++;
            i += len;
            continue;
        }
        if (op == OpCapability) {
            if (w[i + 1] == CapPhysicalStorageBuffer) return 0;   // (already reads by addresses: not one of the game's)
            if (w[i + 1] == CapInt64) has_int64 = true;
            last_cap = i + len;
        } else if (op == OpEntryPoint) {
            entries++, mesh = w[i + 1] == ModelMeshEXT;
        } else if (op == OpExecutionMode && len >= 4) {
            if (w[i + 2] == ModeOutputVertices) out_v = w[i + 3];
            if (w[i + 2] == ModeOutputPrimitives) out_p = w[i + 3];
        } else if (op == OpDecorate) {
            if (len >= 4 && w[i + 2] == DecBuiltIn) {
                if (w[i + 3] == BuiltInPosition) pos_var = w[i + 1];
                else if (w[i + 3] == BuiltInPrimitiveTriangleIndices) idx_var = w[i + 1];
                else if (w[i + 3] == BuiltInLocalInvocationId) lid_var = w[i + 1];
            }
            last_deco = i + len;
        } else if (op == OpMemberDecorate) {
            last_deco = i + len;
        } else if (op == OpTypeInt && len >= 4 && w[i + 3] == 0) {
            if (w[i + 2] == 32 && !t_u32) t_u32 = w[i + 1];
            if (w[i + 2] == 64 && !t_u64) t_u64 = w[i + 1];
        } else if (op == OpTypeFloat && len >= 3 && w[i + 2] == 32) {
            if (!t_f32) t_f32 = w[i + 1];
        } else if (op == OpTypeBool) {
            if (!t_bool) t_bool = w[i + 1];
        }
        i += len;
    }
    if (!mesh || entries != 1 || !idx_var) return 0;   // (another stage, or a mesh program of points or lines)
    if (!first_fn || sets != 1 || !pos_var || !lid_var || !t_u32 || !t_f32 || !last_cap || !last_deco) return -1;
    const uint32_t kPer = (std::max(out_v, out_p) + 31) / 32 * 32;
    if (!kPer || kPer > 256) return -1;
    // where each id we look up is defined (pointer types, variables, constants, access chains)
    std::vector<uint32_t> def(bound, 0);
    const uint32_t need[] = {0, 1, 2, 3, 4, 15, 16, 17, 18, 19, kPer, slots, 264};
    constexpr size_t kNeed = sizeof need / sizeof need[0];
    uint32_t cid[kNeed] = {};
    uint32_t p_wg_u32 = 0;
    for (size_t i = 5; i < first_fn;) {
        const uint32_t op = w[i] & 0xffff, len = w[i] >> 16;
        if (op == OpTypePointer && len >= 4) {
            if (w[i + 1] < bound) def[w[i + 1]] = (uint32_t)i;
            if (w[i + 2] == StorageWorkgroup && w[i + 3] == t_u32 && !p_wg_u32) p_wg_u32 = w[i + 1];
        } else if (op == OpVariable && len >= 4) {
            if (w[i + 2] < bound) def[w[i + 2]] = (uint32_t)i;
        } else if (op == OpConstant && len == 4) {
            if (w[i + 2] < bound) def[w[i + 2]] = (uint32_t)i;
            if (w[i + 1] == t_u32)
                for (size_t k = 0; k < kNeed; ++k)
                    if (!cid[k] && need[k] == w[i + 3]) cid[k] = w[i + 2];
        }
        i += len;
    }
    uint32_t t_lid = 0;   // (the type of gl_LocalInvocationID: what its variable's pointer type points to)
    if (const uint32_t v = def[lid_var]; v && (w[v] & 0xffff) == OpVariable)
        if (const uint32_t p = w[v + 1] < bound ? def[w[v + 1]] : 0; p && (w[p] & 0xffff) == OpTypePointer) t_lid = w[p + 3];
    if (!t_lid) return -1;

    std::vector<uint32_t> G, D;   // (new globals, new decorations)
    auto put = [](std::vector<uint32_t>& to, std::initializer_list<uint32_t> v) {
        const size_t at = to.size();
        to.insert(to.end(), v.begin(), v.end());
        to[at] |= (uint32_t)v.size() << 16;
    };
    auto nid = [&]() { return bound++; };
    if (!t_u64) t_u64 = nid(), put(G, {OpTypeInt, t_u64, 64, 0});
    if (!t_bool) t_bool = nid(), put(G, {OpTypeBool, t_bool});
    if (!p_wg_u32) p_wg_u32 = nid(), put(G, {OpTypePointer, p_wg_u32, StorageWorkgroup, t_u32});
    for (size_t k = 0; k < kNeed; ++k)
        if (!cid[k]) {
            bool twin = false;   // (the same value twice in the list: one constant)
            for (size_t j = 0; j < k; ++j)
                if (need[j] == need[k]) cid[k] = cid[j], twin = true;
            if (!twin) cid[k] = nid(), put(G, {OpConstant, t_u32, cid[k], need[k]});
        }
    const uint32_t c0 = cid[0], c1 = cid[1], c2 = cid[2], c3 = cid[3], c4 = cid[4], c15 = cid[5], c_per = cid[10], c_cap = cid[11], c264 = cid[12];
    const uint32_t c_addr_u = nid(), c_addr_f = nid();
    put(G, {OpConstant, t_u64, c_addr_u, (uint32_t)addr_words, (uint32_t)(addr_words >> 32)});
    put(G, {OpConstant, t_u64, c_addr_f, (uint32_t)addr_clip, (uint32_t)(addr_clip >> 32)});
    const uint32_t ra_u = nid(), st_u = nid(), pp_u = nid(), pe_u = nid(), ra_f = nid(), st_f = nid(), pp_f = nid(), pe_f = nid();
    put(G, {OpTypeRuntimeArray, ra_u, t_u32}), put(G, {OpTypeStruct, st_u, ra_u});
    put(G, {OpTypePointer, pp_u, StoragePhysical, st_u}), put(G, {OpTypePointer, pe_u, StoragePhysical, t_u32});
    put(G, {OpTypeRuntimeArray, ra_f, t_f32}), put(G, {OpTypeStruct, st_f, ra_f});
    put(G, {OpTypePointer, pp_f, StoragePhysical, st_f}), put(G, {OpTypePointer, pe_f, StoragePhysical, t_f32});
    put(D, {OpDecorate, ra_u, DecArrayStride, 4}), put(D, {OpDecorate, st_u, DecBlock}), put(D, {OpMemberDecorate, st_u, 0, DecOffset, 0});
    put(D, {OpDecorate, ra_f, DecArrayStride, 4}), put(D, {OpDecorate, st_f, DecBlock}), put(D, {OpMemberDecorate, st_f, 0, DecOffset, 0});
    const uint32_t arr2 = nid(), p_arr2 = nid(), v_base = nid();   // (the meshlet's base, shared by the 32 invocations)
    put(G, {OpTypeArray, arr2, t_u32, c2}), put(G, {OpTypePointer, p_arr2, StorageWorkgroup, arr2});
    put(G, {OpVariable, p_arr2, v_base, StorageWorkgroup});

    out.clear();
    out.reserve(n + 2048);
    out.insert(out.end(), w, w + 5);
    uint32_t base = 0, ok = 0, pu = 0, pf = 0;   // (ids of the block that follows OpSetMeshOutputsEXT)
    uint32_t n_pos = 0, n_idx = 0;
    for (size_t i = 5; i < n;) {
        const uint32_t op = w[i] & 0xffff, len = w[i] >> 16;
        if (i == first_fn) out.insert(out.end(), G.begin(), G.end());
        if (i < first_fn && op == OpMemoryModel && len >= 3) {
            put(out, {OpMemoryModel, AddressingPhysical64, w[i + 2]});
        } else if (i < first_fn && op == OpEntryPoint) {   // (every global the entry point uses is listed: ours too)
            out.push_back(((len + 1) << 16) | OpEntryPoint);
            out.insert(out.end(), w + i + 1, w + i + len);
            out.push_back(v_base);
        } else {
            out.insert(out.end(), w + i, w + i + len);
        }
        if (i >= first_fn && (op == OpAccessChain || op == OpInBoundsAccessChain) && len >= 5 && w[i + 2] < def.size()) def[w[i + 2]] = (uint32_t)i;
        if (i >= first_fn && op == OpSetMeshOutputsEXT && len >= 3) {
            const uint32_t count = w[i + 2];
            const uint32_t lid = nid(), lx = nid(), is0 = nid(), pflag = nid(), flag = nid(), en0 = nid(), nz = nid(), en = nid();
            const uint32_t doit = nid(), add = nid(), pcnt = nid(), old = nid(), slot = nid(), pb = nid(), pb0 = nid(), lim = nid(), fits = nid();
            pu = nid(), pf = nid(), base = nid(), ok = nid();
            put(out, {OpLoad, t_lid, lid, lid_var});
            put(out, {OpCompositeExtract, t_u32, lx, lid, 0});
            put(out, {OpIEqual, t_bool, is0, lx, c0});
            put(out, {OpConvertUToPtr, pp_u, pu, c_addr_u});
            put(out, {OpAccessChain, pe_u, pflag, pu, c0, c1});
            put(out, {OpLoad, t_u32, flag, pflag, AccessAligned, 4});
            put(out, {OpINotEqual, t_bool, en0, flag, c0});
            put(out, {OpINotEqual, t_bool, nz, count, c0});
            put(out, {OpLogicalAnd, t_bool, en, en0, nz});
            put(out, {OpLogicalAnd, t_bool, doit, is0, en});
            put(out, {OpSelect, t_u32, add, doit, c_per, c0});
            put(out, {OpAccessChain, pe_u, pcnt, pu, c0, c0});
            put(out, {OpAtomicIAdd, t_u32, old, pcnt, c1, c0, add});   // (scope: device; no ordering asked)
            put(out, {OpSelect, t_u32, slot, is0, c0, c1});
            put(out, {OpAccessChain, p_wg_u32, pb, v_base, slot});
            put(out, {OpStore, pb, old});
            put(out, {OpControlBarrier, c2, c2, c264});
            put(out, {OpAccessChain, p_wg_u32, pb0, v_base, c0});
            put(out, {OpLoad, t_u32, base, pb0});
            put(out, {OpIAdd, t_u32, lim, base, c_per});
            put(out, {OpULessThanEqual, t_bool, fits, lim, c_cap});
            put(out, {OpLogicalAnd, t_bool, ok, en, fits});
            put(out, {OpConvertUToPtr, pp_f, pf, c_addr_f});
        } else if (base && op == OpStore && len >= 3 && w[i + 1] < def.size() && def[w[i + 1]]) {
            const uint32_t a = def[w[i + 1]], alen = w[a] >> 16, var = w[a + 3], val = w[i + 2];
            if (var == pos_var && alen >= 5) {
                const uint32_t v = w[a + 4];
                uint32_t comp[4], e[4], nc = 0;
                if (alen == 6) {   // (one component: its index is a constant)
                    const uint32_t ci = w[a + 5] < def.size() ? def[w[a + 5]] : 0;
                    if (!ci || (w[ci] & 0xffff) != OpConstant || w[ci + 3] > 3) return -1;
                    comp[0] = w[ci + 3], e[0] = val, nc = 1;
                } else if (alen == 5) {   // (the whole position at once)
                    for (uint32_t c = 0; c < 4; ++c) comp[c] = c, e[c] = nid(), put(out, {OpCompositeExtract, t_f32, e[c], val, c});
                    nc = 4;
                } else {
                    return -1;
                }
                for (uint32_t k = 0; k < nc; ++k) {
                    const uint32_t sl = nid(), i4 = nid(), ii = nid(), idx = nid(), ptr = nid();
                    put(out, {OpIAdd, t_u32, sl, base, v});
                    put(out, {OpIMul, t_u32, i4, sl, c4});
                    put(out, {OpIAdd, t_u32, ii, i4, cid[6 + comp[k]]});          // (16 + component)
                    put(out, {OpSelect, t_u32, idx, ok, ii, cid[comp[k]]});       // (else a waste word)
                    put(out, {OpAccessChain, pe_f, ptr, pf, c0, idx});
                    put(out, {OpStore, ptr, e[k], AccessAligned, 4});
                }
                n_pos++;
            } else if (var == idx_var && alen == 5) {
                const uint32_t p = w[a + 4], sl = nid(), i3 = nid();
                put(out, {OpIAdd, t_u32, sl, base, p});
                put(out, {OpIMul, t_u32, i3, sl, c3});
                for (uint32_t k = 0; k < 3; ++k) {
                    const uint32_t e = nid(), g = nid(), ii = nid(), idx = nid(), ptr = nid();
                    put(out, {OpCompositeExtract, t_u32, e, val, k});
                    put(out, {OpIAdd, t_u32, g, e, base});
                    put(out, {OpIAdd, t_u32, ii, i3, cid[6 + k]});
                    put(out, {OpSelect, t_u32, idx, ok, ii, c15});
                    put(out, {OpAccessChain, pe_u, ptr, pu, c0, idx});
                    put(out, {OpStore, ptr, g, AccessAligned, 4});
                }
                n_idx++;
            }
        }
        i += len;
        if (i == last_cap) {
            put(out, {OpCapability, CapPhysicalStorageBuffer});
            if (!has_int64) put(out, {OpCapability, CapInt64});
            // OpExtension "SPV_KHR_physical_storage_buffer" (the name packed four characters a word, zero ended)
            static const char name[] = "SPV_KHR_physical_storage_buffer";
            uint32_t packed[8] = {};
            for (size_t k = 0; k < sizeof name; ++k) packed[k / 4] |= (uint32_t)(uint8_t)name[k] << (8 * (k % 4));
            out.push_back((9u << 16) | OpExtension);
            out.insert(out.end(), packed, packed + 8);
        }
        if (i == last_deco) out.insert(out.end(), D.begin(), D.end());
    }
    if (n_pos < 4 || n_idx < 1) return -1;
    out[3] = bound;
    return 1;
}

} // namespace rt
