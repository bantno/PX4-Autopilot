"""Place the wingtip floats (Float_Mount_Assm.stp) on the AHAB model: floats.glb + float buoyancy boxes.

    ~/occ_venv/bin/python float_to_model.py Float_Mount_Assm.stp . ../twin_tractor_seaplane/meshes ../collision_boxes.json
    python3 build_model.py

Placement (user, 2026-10-13): float centreline 748.5 mm from the aircraft centreline, float front 11 mm ahead of
the wing leading edge, one per side (mirrored), pitched 3 deg nose-up about the strut tops. The struts hang from
the wing underside, so the floats are part of the tilting wing.

Float file frame (Inventor, mm): x along the float (+ = nose: the +x end's bottom sweeps up like a ski),
y = down from the wing (strut free ends at y ~ -3), z = left (struts centred at z 637.5).
The wing's leading edge and lower surface at the float station are read from wing.glb.

Floats are solid (367 cm^3, ~ the elliptical-section envelope), so their buoyancy boxes carry the exact solid
volume (span slices x chord slices, box height = local thickness), like the wing panels.
"""
import json
import struct
import sys

import numpy as np

sys.path.insert(0, sys.argv[2])
import step_bodies as sb  # noqa: E402
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common  # noqa: E402
from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.gp import gp_Pnt  # noqa: E402

STEP, _, MESH_DIR, BOXES = sys.argv[1:5]

SPAN = 0.7485          # m, float centreline from aircraft centreline
NOSE_AHEAD = 0.011     # m, float front ahead of the wing leading edge
INCIDENCE_DEG = 3.0    # float pitched nose-up relative to the hull waterline (user, 2026-10-13), about the strut tops
LATERAL_CENTRE = 637.5  # mm, float/strut centre in the file's z


def load_glb_positions(path):
    b = open(path, 'rb').read()
    n, = struct.unpack('<I', b[12:16])
    j = json.loads(b[20:20 + n])
    a = j['accessors'][0]
    bv = j['bufferViews'][a['bufferView']]
    off = 20 + n + 8
    return np.frombuffer(b[off:], dtype='<f4', count=a['count'] * 3, offset=bv.get('byteOffset', 0)).reshape(-1, 3)


bodies = sb.solids(STEP)
float_body = [s for n, s in bodies if n.startswith('Wingtip')]
struts = [s for n, s in bodies if n.startswith('Wing_Float_Carbon')]

if len(float_body) != 1 or len(struts) != 2:
    sys.exit('expected 1 float and 2 struts, found %d / %d' % (len(float_body), len(struts)))

# wing leading edge and lower surface at the float station
W = load_glb_positions(MESH_DIR + '/wing.glb')
station = np.abs(np.abs(W[:, 1]) - SPAN) < 0.01
x_le = W[station, 0].max()

(fx0, fy0, fz0, fx1, fy1, fz1), _, _ = sb.info(float_body[0])
x_nose_file = sb.mesh(float_body[0], 0.1)[0][:, 0].max()   # tight nose (the B-rep box is padded)


def strut_top(s):
    (x0, y0, z0, x1, y1, z1), _, _ = sb.info(s)
    return 0.5 * (x0 + x1), y0


# strut free ends touch the wing lower surface at their chord stations
offsets = []

for s in struts:
    sx, sy = strut_top(s)
    mx = x_le + NOSE_AHEAD + (sx - x_nose_file) / 1000.0
    near = station & (np.abs(W[:, 0] - mx) < 0.01)
    offsets.append(W[near, 2].min() + sy / 1000.0)

z_ref = float(np.mean(offsets))  # model z of file y = 0

# incidence pivot: midway between the strut tops, where they meet the wing
pivot_x = float(np.mean([x_le + NOSE_AHEAD + (strut_top(s)[0] - x_nose_file) / 1000.0 for s in struts]))
pivot_z = z_ref
ca, sa = np.cos(np.radians(INCIDENCE_DEG)), np.sin(np.radians(INCIDENCE_DEG))


def place(V, side):
    """File (mm) -> model (m) for the float on side +1 (left, +y) or -1 (right)."""
    x = x_le + NOSE_AHEAD + (V[:, 0] - x_nose_file) / 1000.0
    z = z_ref - V[:, 1] / 1000.0
    # file x (nose) / y (down) / z form a right-handed set with z = left, so side +1 is a proper rotation and
    # side -1 its mirror image (which flips the triangle winding)
    y = side * (SPAN + (V[:, 2] - LATERAL_CENTRE) / 1000.0)
    # nose-up incidence: rotate about the strut tops in the x-z plane (points ahead of the pivot go up)
    dx, dz = x - pivot_x, z - pivot_z
    return np.stack([pivot_x + ca * dx - sa * dz + x_shift, y, pivot_z + sa * dx + ca * dz], axis=1)


# the 11 mm nose lead was measured on the installed (inclined) float: shift so the rotated nose keeps it
x_shift = 0.0
x_shift = x_le + NOSE_AHEAD - place(sb.mesh(float_body[0], 0.1)[0], 1)[:, 0].max()


# --- visuals: float + struts + mount, both sides
Vs, Fs, n = [], [], 0

for side in (1, -1):
    for _, s in bodies:
        V, F = sb.mesh(s, 0.3)
        P = place(V, side)
        Fs.append((F if side > 0 else F[:, ::-1]) + n)   # mirroring flips the winding
        Vs.append(P)
        n += len(V)

V = np.concatenate(Vs)
F = np.concatenate(Fs)
sb.write_glb(MESH_DIR + '/floats.glb', V, F, 'floats', [0.95, 0.55, 0.15, 1.0])
print('floats.glb: %d tris, x[%.4f %.4f] z[%.4f %.4f] m, |y| %.4f..%.4f'
      % (len(F), V[:, 0].min(), V[:, 0].max(), V[:, 2].min(), V[:, 2].max(), np.abs(V[:, 1]).min(), np.abs(V[:, 1]).max()))

# --- buoyancy boxes: float only (struts/mount are negligible), exact solid volume per cell
DXF, DZF = 25.0, 22.0  # mm cells along the float and across it
boxes, total = [], 0.0

for x0 in np.arange(fx0, fx1, DXF):
    for z0 in np.arange(fz0, fz1, DZF):
        cell = BRepPrimAPI_MakeBox(gp_Pnt(x0, fy0 - 1, z0), gp_Pnt(x0 + DXF, fy1 + 1, z0 + DZF)).Shape()
        common = BRepAlgoAPI_Common(float_body[0], cell)
        g = GProp_GProps()
        BRepGProp.VolumeProperties_s(common.Shape(), g)

        if g.Mass() < 50.0:
            continue

        c = g.CentreOfMass()
        height = g.Mass() / (DXF * DZF)    # mm, keeps the cell volume
        total += g.Mass()

        for side in (1, -1):
            p = place(np.array([[c.X(), c.Y(), c.Z()]]), side)[0]
            boxes.append({'pos': p.round(5).tolist(), 'size': [DXF / 1000, DZF / 1000, round(height / 1000, 5)]})

with open(BOXES) as f:
    cb = json.load(f)

cb['wing'] = [b for b in cb['wing'] if not b.get('float')] + [dict(b, float=True) for b in boxes]

# hydrodynamic surface panels for both floats (WaterHydrodynamics plugin), model frame, on the wing
panels = []

for side in (1, -1):
    c, nrm, a = sb.surface_panels(float_body, lambda V, side=side: place(V, side), deflection=4.0)

    if side < 0:
        nrm = -nrm   # place() mirrors y, which flips the triangle winding and so the computed normal

    panels += [{'pos': ci.round(5).tolist(), 'n': ni.round(4).tolist(), 'area': round(float(ai), 7)}
               for ci, ni, ai in zip(c, nrm, a)]

cb['panels_float'] = panels
print('float panels: %d (both sides), %.4f m^2 per float' % (len(panels), sum(p['area'] for p in panels) / 2))
cb['floats_source'] = STEP.split('/')[-1]

with open(BOXES, 'w') as f:
    json.dump(cb, f, indent=1)

print('float boxes: %d (both sides), %.0f cm^3 per float' % (len(boxes), total / 1000))
print('wing LE at station %.4f m, strut attach z_ref %.4f m' % (x_le, z_ref))
