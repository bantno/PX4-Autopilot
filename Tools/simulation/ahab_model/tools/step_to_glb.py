"""Mesh the AHAB multibody STEP into hull.glb / wing.glb in the Gazebo model frame (m, x fwd, y left, z up).

    python3 -m venv ~/occ_venv && ~/occ_venv/bin/pip install cadquery-ocp numpy
    ~/occ_venv/bin/python step_to_glb.py AHAB_FINAL_simv2.stp . ../twin_tractor_seaplane/meshes
    python3 build_model.py

STEP frame (Inventor export, mm): x = span (+ = right), y = up, z = aft; model = (-z, -x, y) / 1000. The
result lines up with Nick's original mesh to 0.6 mm median (nose x +0.4832, keel z -0.1414).

Only the two outer wing panels tilt; the wing centre section, hatches and V-tail stay with the hull. Wing
panels are recognised by shape (long in span, thin, starting outboard of the fuselage), because the body
names do not survive this STEP reader.
"""
import json
import struct
import sys

import numpy as np

sys.path.insert(0, sys.argv[2])
import step_bodies as sb  # noqa: E402

STEP, _, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
DEFLECTION = 0.3  # mm


def to_model(V):
    return np.stack([-V[:, 2], -V[:, 0], V[:, 1]], axis=1) / 1000.0


def normals(V, F):
    n = np.zeros_like(V)
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])

    for k in range(3):
        np.add.at(n, F[:, k], fn)

    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1
    return n / ln


def write_glb(path, V, F, name, rgba):
    V = V.astype('<f4')
    N = normals(V.astype(np.float64), F).astype('<f4')
    I = F.astype('<u4').ravel()
    bv = [V.tobytes(), N.tobytes(), I.tobytes()]
    offs, blob = [], b''

    for b in bv:
        offs.append(len(blob))
        blob += b + b'\0' * ((4 - len(b) % 4) % 4)

    gltf = {
        'asset': {'version': '2.0', 'generator': 'ahab step_to_glb'},
        'scene': 0, 'scenes': [{'nodes': [0]}], 'nodes': [{'mesh': 0, 'name': name}],
        'materials': [{'name': name, 'pbrMetallicRoughness': {'baseColorFactor': rgba, 'metallicFactor': 0.0,
                                                               'roughnessFactor': 0.6}}],
        'meshes': [{'name': name, 'primitives': [{'attributes': {'POSITION': 0, 'NORMAL': 1}, 'indices': 2,
                                                  'material': 0}]}],
        'buffers': [{'byteLength': len(blob)}],
        'bufferViews': [{'buffer': 0, 'byteOffset': offs[0], 'byteLength': len(bv[0]), 'target': 34962},
                        {'buffer': 0, 'byteOffset': offs[1], 'byteLength': len(bv[1]), 'target': 34962},
                        {'buffer': 0, 'byteOffset': offs[2], 'byteLength': len(bv[2]), 'target': 34963}],
        'accessors': [{'bufferView': 0, 'componentType': 5126, 'count': len(V), 'type': 'VEC3',
                       'min': V.min(0).tolist(), 'max': V.max(0).tolist()},
                      {'bufferView': 1, 'componentType': 5126, 'count': len(N), 'type': 'VEC3'},
                      {'bufferView': 2, 'componentType': 5125, 'count': len(I), 'type': 'SCALAR'}],
    }
    js = json.dumps(gltf, separators=(',', ':')).encode()
    js += b' ' * ((4 - len(js) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(blob)

    with open(path, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, total))
        f.write(struct.pack('<II', len(js), 0x4E4F534A) + js)
        f.write(struct.pack('<II', len(blob), 0x004E4942) + blob)


bodies = sb.solids(STEP)


def is_wing_panel(solid):
    (x0, y0, z0, x1, y1, z1), _, _ = sb.info(solid)
    return (x1 - x0) > 500 and (y1 - y0) < 60 and min(abs(x0), abs(x1)) > 100


wing = [i for i, (_, s) in enumerate(bodies) if is_wing_panel(s)]
GROUPS = {'hull': [i for i in range(len(bodies)) if i not in wing], 'wing': wing}

if len(wing) != 2:
    sys.exit('expected 2 wing panels, found %d' % len(wing))

colour = {'hull': [0.92, 0.92, 0.92, 1.0], 'wing': [0.85, 0.87, 0.92, 1.0]}

for group, idx in GROUPS.items():
    Vs, Fs, n = [], [], 0

    for i in idx:
        V, F = sb.mesh(bodies[i][1], DEFLECTION)
        Vs.append(to_model(V))
        Fs.append(F + n)
        n += len(V)

    V = np.concatenate(Vs)
    F = np.concatenate(Fs)
    write_glb('%s/%s.glb' % (OUT, group), V, F, group, colour[group])
    print('%s: %d verts %d tris  x[%.4f %.4f] y[%.4f %.4f] z[%.4f %.4f] m'
          % (group, len(V), len(F), *[f(V[:, k]) for k in range(3) for f in (np.min, np.max)]))
