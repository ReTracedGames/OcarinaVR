# VR mode, first person: which triangles of Link's model belong to his arms?
#
# The game draws Link piece by piece (skin, tunic, hair, boots...), each piece bound to the bones of his skeleton. This
# tool reads the model in the game files of this computer (<game>\romfs\Model\Player_*.bfres.zs), keeps for each piece
# the triangles whose three corners mostly follow the bones of the arms (from the upper arm to the fingers), and writes
# the result to <game>\retraced\vr-corps.bin. Our core (src/rt_vr.cpp) then knows each piece of Link when the game
# draws it in the headset's view and draws only those triangles; shadows and reflections keep the whole body.
#
# It also writes Link's skeleton (his bones, their place at rest): our core uses it to lay Link's arms where the
# player's hands are (headset controllers or bare hands), finger by finger.
#
# The file it writes holds only vertex numbers and bone poses taken from this computer's copy of the game: it is never
# distributed, it is made again here in a second.
#
#   python tools\vr-corps.py            writes <game>\retraced\vr-corps.bin (Link the child and the adult)
#   python tools\vr-corps.py --os       shows the skeleton and the bones kept
#   RETRACED_GAME_DIR=<folder>          the built game's folder (default: jeu beside this repository's root)
import math
import os
import struct
import sys

import zstandard as zstd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GAME = os.environ.get('RETRACED_GAME_DIR') or os.path.join(ROOT, 'jeu')   # (the built game's folder)
ROMFS = os.path.join(GAME, 'romfs')
OUT = os.path.join(GAME, 'retraced', 'vr-corps.bin')
MODELS = ['Player_Child.Player_Child', 'Player_Adult.Player_Adult']
# The bones kept: the upper arm and all that hangs from it (forearm, wrist, fingers, sleeve, bracelet).
ARM_ROOTS = ('Arm_1_L', 'Arm_1_R')
KEEP_SHARE = 0.0      # a vertex is "of the arm" when at least this share of its weight follows bones kept
# ... and when nearly all of its weight (ONE_ARM) follows that arm and what joins it to the body (the collarbone of the
# same side): the arms are drawn where the player's hands are, away from Link's body; a vertex that also followed the
# chest would be torn between the two.
# (0.98 at first: the sleeve was cut down to a ring and the arm, seen from above, was hollow. Since then our
# core lays the rest of the skeleton under the player's head (src/rt_hands.cpp, "torso"): a vertex of the sleeve or
# the shoulder that also follows the chest is no longer pulled elsewhere; half the weight on the arm and its
# collarbone is enough.)
ONE_ARM = 0.5
JOINTS = ('Clavicle', 'Ph_Clavicle_Assist')
# ... and when it lies past a plane that cuts the upper arm at this share of its length from the shoulder: a clean,
# ring-shaped cut instead of the ragged edge the weights alone give. (No cut since the whole shoulder is kept: -9.)
# For a test: --coupe 0.30
CUT = -9.0
# For a test: --coupe 0.30, --part 0.5 (KEEP_SHARE), --bras 0.98 (ONE_ARM), --sortie file (instead of the file the game
# reads: to count what a rule keeps without changing what is installed).
for _k, _a in enumerate(sys.argv):
    if _k + 1 < len(sys.argv):
        if _a == '--coupe':
            CUT = float(sys.argv[_k + 1])
        if _a == '--part':
            KEEP_SHARE = float(sys.argv[_k + 1])
        if _a == '--bras':
            ONE_ARM = float(sys.argv[_k + 1])
        if _a == '--sortie':
            OUT = sys.argv[_k + 1]
# A second list, for the setting "Link's body visible": all but his head (face, hair, cap), in which the player's
# eyes are.
HEAD_ROOT = 'Head'


def dictionaries():
    raw = open(os.path.join(ROMFS, 'Pack', 'ZsDic.pack.zs'), 'rb').read()
    sarc = zstd.ZstdDecompressor().decompress(raw, max_output_size=1 << 28)
    assert sarc[:4] == b'SARC'
    data_off = struct.unpack_from('<I', sarc, 0x0c)[0]
    n = struct.unpack_from('<H', sarc, 0x1a)[0]
    out = []
    for i in range(n):
        _, _, a, b = struct.unpack_from('<IIII', sarc, 0x20 + i * 16)
        out.append(zstd.ZstdCompressionDict(sarc[data_off + a:data_off + b]))
    return out


def unpack_file(path, dicts):
    raw = open(path, 'rb').read()
    for d in [None] + dicts:
        try:
            dc = zstd.ZstdDecompressor(dict_data=d) if d else zstd.ZstdDecompressor()
            return dc.decompress(raw, max_output_size=1 << 29)
        except zstd.ZstdError:
            continue
    raise RuntimeError('unreadable: ' + path)


class Model:
    """What is needed of a model file of the game (format "FRES", version 10.4 measured)."""

    def __init__(self, data):
        self.d = data
        assert data[:4] == b'FRES', 'not a model'
        self.version = struct.unpack_from('<I', data, 8)[0]
        fmdl = self.q(0x28)
        assert data[fmdl:fmdl + 4] == b'FMDL'
        info = self.q(0xb0)                      # (the buffer pool: flags, size, place in the file)
        self.buf_base = self.q(info + 8)
        self.fskl, self.fvtx0, self.fshp0 = self.q(fmdl + 0x18), self.q(fmdl + 0x20), self.q(fmdl + 0x28)
        self.n_vtx, self.n_shape = struct.unpack_from('<HH', data, fmdl + 0x68)
        assert data[self.fskl:self.fskl + 4] == b'FSKL'
        bones, m2b = self.q(self.fskl + 0x10), self.q(self.fskl + 0x18)
        n_bones, n_smooth, n_rigid = struct.unpack_from('<HHH', data, self.fskl + 0x38)
        self.bones = []
        self.rest = []       # per bone: parent, smooth matrix, rigid matrix, then its place against its parent (3 x 4)
        for i in range(n_bones):
            b = bones + i * 0x58
            idx, parent, smooth, rigid = struct.unpack_from('<Hhhh', data, b + 0x20)
            self.bones.append((self.name(self.q(b)), parent))
            sc = struct.unpack_from('<3f', data, b + 0x30)
            rx, ry, rz, _ = struct.unpack_from('<4f', data, b + 0x3c)
            tr = struct.unpack_from('<3f', data, b + 0x4c)
            # (angles about x, then y, then z: checked against the file's inverse matrices,
            # no difference over the 277 smooth bones of Link the child)
            cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
            r = [[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                 [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                 [-sy, cy * sx, cy * cx]]
            local = []
            for row in range(3):
                local += [r[row][0] * sc[0], r[row][1] * sc[1], r[row][2] * sc[2], tr[row]]
            self.rest.append((parent, smooth, rigid, local))
        inv = self.q(self.fskl + 0x20)
        self.inverse = [struct.unpack_from('<12f', data, inv + k * 48) for k in range(n_smooth)]
        self.n_smooth, self.n_rigid = n_smooth, n_rigid
        self.palette = list(struct.unpack_from('<%dH' % (n_smooth + n_rigid), data, m2b))   # (matrix -> bone)

    def q(self, off):
        return struct.unpack_from('<Q', self.d, off)[0]

    def name(self, off):
        n = struct.unpack_from('<H', self.d, off)[0]
        return self.d[off + 2:off + 2 + n].decode('ascii')

    def kept_bones(self):
        keep = [False] * len(self.bones)
        for i, (nm, parent) in enumerate(self.bones):   # (a bone always comes after its parent)
            keep[i] = nm in ARM_ROOTS or (parent >= 0 and keep[parent])
        return keep

    def sides(self):
        """Per bone: the arm it belongs to (0 left, 1 right, -1 none), and the arm it joins to the body."""
        arm = [-1] * len(self.bones)
        joint = [-1] * len(self.bones)
        for i, (nm, parent) in enumerate(self.bones):
            for side, sfx in enumerate(('_L', '_R')):
                if nm == 'Arm_1' + sfx:
                    arm[i] = side
                if nm in tuple(j + sfx for j in JOINTS):
                    joint[i] = side
            if arm[i] < 0 and parent >= 0 and arm[parent] >= 0:
                arm[i] = arm[parent]
        return arm, joint

    def head_bones(self):
        head = [False] * len(self.bones)
        for i, (nm, parent) in enumerate(self.bones):
            head[i] = nm == HEAD_ROOT or (parent >= 0 and head[parent])
        return head

    def rest_places(self):
        """The place of every bone at rest, in the model (3 x 4, rows)."""
        out = []
        for parent, smooth, rigid, l in self.rest:
            if parent < 0:
                out.append(list(l))
                continue
            a = out[parent]
            m = []
            for r in range(3):
                for c in range(4):
                    v = a[r * 4] * l[c] + a[r * 4 + 1] * l[4 + c] + a[r * 4 + 2] * l[8 + c]
                    m.append(v + (a[r * 4 + 3] if c == 3 else 0.0))
            out.append(m)
        return out

    def cuts(self):
        """Per arm: the shoulder, the direction of the upper arm, and the distance of the cutting plane."""
        places = self.rest_places()
        names = [b[0] for b in self.bones]
        out = []
        for sfx in ('_L', '_R'):
            a, b = places[names.index('Arm_1' + sfx)], places[names.index('Arm_2' + sfx)]
            pa, pb = (a[3], a[7], a[11]), (b[3], b[7], b[11])
            d = [pb[k] - pa[k] for k in range(3)]
            n = math.sqrt(sum(x * x for x in d))
            out.append((pa, [x / n for x in d], n * CUT))
        return out

    def shapes(self):
        d = self.d
        for s in range(self.n_shape):
            sh = self.fshp0 + s * 0x60
            assert d[sh:sh + 4] == b'FSHP'
            vt = self.q(sh + 0x10)
            assert d[vt:vt + 4] == b'FVTX'
            bone_index = struct.unpack_from('<H', d, sh + 0x54)[0]
            skin_count, n_mesh = struct.unpack_from('<BB', d, sh + 0x5a)
            attrs, sizes, strides = self.q(vt + 0x08), self.q(vt + 0x30), self.q(vt + 0x38)
            buf_off = struct.unpack_from('<I', d, vt + 0x48)[0]
            n_attr, n_buf = struct.unpack_from('<BB', d, vt + 0x4c)
            n_verts = struct.unpack_from('<I', d, vt + 0x50)[0]
            starts, at = [], self.buf_base + buf_off
            for k in range(n_buf):   # (a piece's buffers follow each other, each aligned on 8 bytes)
                starts.append(at)
                at = (at + struct.unpack_from('<I', d, sizes + k * 0x10)[0] + 7) & ~7
            stride = [struct.unpack_from('<I', d, strides + k * 0x10)[0] for k in range(n_buf)]
            where = {}
            for a in range(n_attr):
                rec = attrs + a * 0x10
                fmt, off, buf = struct.unpack_from('<IHH', d, rec + 8)
                where[self.name(self.q(rec))] = (fmt, off, buf)
            meshes = []
            marr = self.q(sh + 0x18)
            for m in range(n_mesh):
                rec = marr + m * 0x38
                ib_off, prim, fmt, count, first = struct.unpack_from('<IIIII', d, rec + 0x20)
                meshes.append((self.buf_base + ib_off, fmt, count, first, prim))
            yield dict(name=self.name(self.q(sh + 0x08)), bone=bone_index, skin=skin_count, n_verts=n_verts, where=where,
                       starts=starts, stride=stride, meshes=meshes)

    def read_attr(self, shape, attr, v):
        """The components of an integer or weight attribute of a vertex."""
        fmt, off, buf = shape['where'][attr]
        at = shape['starts'][buf] + v * shape['stride'][buf] + off
        kind, layout = fmt >> 8 & 0xff, fmt & 0xff   # (in the file: layout, then kind)
        layout, kind = fmt & 0xff, fmt >> 8 & 0xff
        d = self.d
        # (kind: 1 shares of 255 or 65535, 3 integers, 5 floats; layout: 2 one byte, 9 two bytes, 0x0b four bytes,
        # 0x0a one 16-bit word, 0x12 two words, 0x15 four words, 0x14 one 32-bit word, 0x17 four 32-bit words ...)
        table = {(0x03, 0x02): '<B', (0x03, 0x09): '<2B', (0x03, 0x0b): '<4B', (0x03, 0x0a): '<H', (0x03, 0x12): '<2H',
                 (0x03, 0x15): '<4H', (0x03, 0x14): '<I', (0x03, 0x17): '<4I',
                 (0x01, 0x02): '<B', (0x01, 0x09): '<2B', (0x01, 0x0b): '<4B', (0x01, 0x0a): '<H', (0x01, 0x12): '<2H',
                 (0x01, 0x15): '<4H', (0x05, 0x16): '<f', (0x05, 0x13): '<2f', (0x05, 0x18): '<3f', (0x05, 0x15): '<4f'}
        key = (layout, kind)
        if key not in table:
            raise RuntimeError('attribute %s: unknown format %#x' % (attr, fmt))
        vals = struct.unpack_from(table[key], d, at)
        if layout == 0x01:
            unit = 255.0 if table[key][-1] == 'B' else 65535.0
            return [x / unit for x in vals]
        return list(vals)

    def indices(self, mesh):
        at, fmt, count, first, prim = mesh
        unit = {0: 1, 1: 2, 2: 4}[fmt]
        raw = self.d[at:at + count * unit]
        return list(struct.unpack('<%d%s' % (count, {1: 'B', 2: 'H', 4: 'I'}[unit]), raw)), raw, unit


def fnv(raw):
    h = 0xcbf29ce484222325
    for c in raw:
        h = ((h ^ c) * 0x100000001b3) & 0xffffffffffffffff
    return h


def main():
    show = '--os' in sys.argv
    dicts = dictionaries()
    out = []
    skeletons = []
    for mi, mname in enumerate(MODELS):
        path = os.path.join(ROMFS, 'Model', mname + '.bfres.zs')
        if not os.path.exists(path):
            print(mname, ': not found')
            continue
        M = Model(unpack_file(path, dicts))
        keep = M.kept_bones()
        arm_of, joint_of = M.sides()
        cuts = M.cuts()
        head_of = M.head_bones()
        skeletons.append((mi, M))
        print('%s: version %#x, %d bones, %d of them kept, %d pieces' % (mname, M.version, len(M.bones), sum(keep), M.n_shape))
        if show:
            for i, (nm, parent) in enumerate(M.bones):
                depth, p = 0, parent
                while p >= 0:
                    depth, p = depth + 1, M.bones[p][1]
                print('   %3d %s%s%s' % (i, '  ' * depth, nm, '   <- kept' if keep[i] else ''))
        for si, sh in enumerate(M.shapes()):
            nv = sh['n_verts']
            arm = [False] * nv
            headv = [False] * nv
            if sh['skin'] == 0:
                arm = [keep[sh['bone']]] * nv
                headv = [head_of[sh['bone']]] * nv
            else:
                ia = [a for a in ('_i0', '_i1') if a in sh['where']]
                wa = [a for a in ('_w0', '_w1') if a in sh['where']]
                for v in range(nv):
                    idx, w = [], []
                    for a in ia:
                        idx += M.read_attr(sh, a, v)
                    for a in wa:
                        w += M.read_attr(sh, a, v)
                    idx = idx[:sh['skin']]
                    if not w:
                        w = [1.0] + [0.0] * (len(idx) - 1)
                    w = w[:len(idx)]
                    tot = sum(w) or 1.0
                    headv[v] = sum(wi for i, wi in zip(idx, w) if i < len(M.palette) and head_of[M.palette[i]]) / tot >= 0.5
                    for side in (0, 1):
                        own = sum(wi for i, wi in zip(idx, w) if i < len(M.palette) and arm_of[M.palette[i]] == side) / tot
                        near = sum(wi for i, wi in zip(idx, w) if i < len(M.palette) and joint_of[M.palette[i]] == side) / tot
                        if own >= KEEP_SHARE and own + near >= ONE_ARM:
                            pos = M.read_attr(sh, '_p0', v)
                            at, along, far = cuts[side]
                            if sum((pos[k] - at[k]) * along[k] for k in range(3)) >= far:
                                arm[v] = True
            line = '   %-48s %5d vertices, %d bones a vertex:' % (sh['name'], nv, sh['skin'])
            for mesh in sh['meshes']:
                idx, raw, unit = M.indices(mesh)
                first = mesh[3]
                if mesh[4] != 3:
                    line += ' (not triangles)'
                    continue
                kept, body = [], []
                for t in range(0, len(idx) - 2, 3):
                    a, b, c = idx[t] + first, idx[t + 1] + first, idx[t + 2] + first
                    if a < nv and b < nv and c < nv and arm[a] and arm[b] and arm[c]:
                        kept += idx[t:t + 3]
                    if not (a < nv and b < nv and c < nv and headv[a] and headv[b] and headv[c]):
                        body += idx[t:t + 3]
                line += ' %d/%d' % (len(kept) // 3, len(idx) // 3)
                out.append((len(idx), unit, fnv(raw[:min(len(raw), 1024)]), mi, si, kept, body))
            print(line)
    if show:
        return
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'wb') as f:
        f.write(struct.pack('<4sII', b'VRC3', len(skeletons), len(out)))
        for mi, M in skeletons:
            f.write(struct.pack('<IIII', mi, len(M.bones), M.n_smooth, M.n_rigid))
            for (nm, _), (parent, smooth, rigid, local) in zip(M.bones, M.rest):
                f.write(struct.pack('<32siii', nm.encode('ascii')[:31], parent, smooth, rigid))
                f.write(struct.pack('<12f', *local))
            for m in M.inverse:
                f.write(struct.pack('<12f', *m))
        for count, unit, h, mi, si, kept, body in out:
            f.write(struct.pack('<IIQIIII', count, unit, h, mi, si, len(kept), len(body)))
            f.write(struct.pack('<%dI' % len(kept), *kept))
            f.write(struct.pack('<%dI' % len(body), *body))
    print('written: %s (%d skeletons, %d meshes, %d bytes)' % (OUT, len(skeletons), len(out), os.path.getsize(OUT)))


if __name__ == '__main__':
    main()
