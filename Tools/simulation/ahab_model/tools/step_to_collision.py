"""Volume-preserving collision boxes for the AHAB Gazebo model, from the multibody STEP.

gz-sim buoyancy only works on box/sphere/cylinder collisions, so the CAD solids are cut into a grid of cells
and each non-empty cell becomes one box with the cell's exact solid volume, centred on that volume's centroid.
Buoyancy force and centre of buoyancy then follow the real hull shape at any heel/trim.

    ~/occ_venv/bin/python step_to_collision.py AHAB_FINAL_simv2.stp . ../collision_boxes.json
    python3 build_model.py

Fuselage + hatches: the CAD fuselage is an open thin-walled shell, so its solid volume is only the skin. For
buoyancy the sealed outer envelope is what counts: each DX x DZ cell gets a box spanning the outermost surface hits
of spanwise rays cast through the tessellated fuselage/hatch surface (3x3 rays per cell, widths averaged).
V-tail panels (solid): cells of DX along x, DZ in z, split left/right; box width carries the exact volume.
Wing panels: cells of DY along the span, full chord/thickness; box chord length carries the volume.
All boxes are in the model frame (x fwd, y left, z up, m); build_model.py re-expresses the wing boxes relative
to the current tilt pivot.
"""
import json
import sys

import numpy as np

sys.path.insert(0, sys.argv[2])
import step_bodies as sb  # noqa: E402
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common  # noqa: E402
from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.gp import gp_Pnt  # noqa: E402

STEP, _, OUT = sys.argv[1], sys.argv[2], sys.argv[3]

DX, DZ = 47.0, 25.0     # mm, hull cells (STEP z = -model x, STEP y = model z)
DY_WING = 100.0         # mm, wing cells along the span
MIN_VOL = 2.0e3         # mm^3, ignore slivers


def to_model(p):
    x, y, z = p
    return np.array([-z, -x, y]) / 1000.0


def cell(solids, lo, hi):
    """Volume (mm^3) and centroid (STEP frame) of the solids inside the axis-aligned box lo..hi."""
    box = BRepPrimAPI_MakeBox(gp_Pnt(*lo), gp_Pnt(*hi)).Shape()
    V, M = 0.0, np.zeros(3)

    for s in solids:
        common = BRepAlgoAPI_Common(s, box)

        if not common.IsDone():
            continue

        g = GProp_GProps()
        BRepGProp.VolumeProperties_s(common.Shape(), g)

        if g.Mass() > 1e-6:
            c = g.CentreOfMass()
            V += g.Mass()
            M += g.Mass() * np.array([c.X(), c.Y(), c.Z()])

    return V, (M / V if V > 0 else None)


def bounds(solids):
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)

    for s in solids:
        (x0, y0, z0, x1, y1, z1), _, _ = sb.info(s)
        lo = np.minimum(lo, (x0, y0, z0))
        hi = np.maximum(hi, (x1, y1, z1))

    return lo, hi


bodies = [s for _, s in sb.solids(STEP)]
wing_idx = [i for i, s in enumerate(bodies)
            if (lambda b: (b[3] - b[0]) > 500 and (b[4] - b[1]) < 60 and min(abs(b[0]), abs(b[3])) > 100)(sb.info(s)[0])]
hull = [s for i, s in enumerate(bodies) if i not in wing_idx]
vtail_idx = [i for i in range(len(bodies)) if i not in wing_idx and sb.info(bodies[i])[0][4] > 200 and sb.info(bodies[i])[0][5] > 400]
fuse = [s for i, s in enumerate(bodies) if i not in wing_idx and i not in vtail_idx]   # fuselage + hatches
vtail = [bodies[i] for i in vtail_idx]
wing = [bodies[i] for i in wing_idx]

out = {'hull': [], 'wing': [], 'source': STEP.split('/')[-1]}
total = {'hull': 0.0, 'wing': 0.0}

# --- fuselage + hatches: outer envelope from spanwise ray hits (STEP frame: x span, y up, z aft)
tris = []

for s in fuse:
    V, F = sb.mesh(s, 0.5)
    tris.append(V[F])

T = np.concatenate(tris)                      # (n, 3 vertices, xyz)
lo, hi = bounds(fuse)


def ray_span(y, z):
    """Outermost x hits of the line {y, z} through the surface, or None."""
    a, b, c = T[:, 0], T[:, 1], T[:, 2]
    d = (b[:, 1] - c[:, 1]) * (a[:, 2] - c[:, 2]) + (c[:, 2] - b[:, 2]) * (a[:, 1] - c[:, 1])
    ok = np.abs(d) > 1e-12
    w0 = np.where(ok, ((b[:, 1] - c[:, 1]) * (z - c[:, 2]) + (c[:, 2] - b[:, 2]) * (y - c[:, 1])) / np.where(ok, d, 1), -1)
    w1 = np.where(ok, ((c[:, 1] - a[:, 1]) * (z - c[:, 2]) + (a[:, 2] - c[:, 2]) * (y - c[:, 1])) / np.where(ok, d, 1), -1)
    w2 = 1 - w0 - w1
    hit = ok & (w0 >= 0) & (w1 >= 0) & (w2 >= 0)

    if hit.sum() < 2:
        return None

    x = w0[hit] * a[hit, 0] + w1[hit] * b[hit, 0] + w2[hit] * c[hit, 0]
    return x.min(), x.max()


for z0 in np.arange(lo[2], hi[2], DX):
    for y0 in np.arange(lo[1], hi[1], DZ):
        spans = []

        for fz in (0.17, 0.5, 0.83):
            for fy in (0.17, 0.5, 0.83):
                r = ray_span(y0 + fy * DZ, z0 + fz * DX)
                spans.append(r if r is not None else (0.0, 0.0))

        width = np.mean([r[1] - r[0] for r in spans])

        if width * DX * DZ < MIN_VOL:
            continue

        mid = np.mean([(r[0] + r[1]) / 2 for r in spans if r[1] > r[0]])
        centre = to_model((mid, y0 + DZ / 2, z0 + DX / 2))
        out['hull'].append({'pos': centre.round(5).tolist(), 'size': [DX / 1000, round(width / 1000, 5), DZ / 1000]})
        total['hull'] += width * DX * DZ

# --- V-tail panels (solid): x (STEP z) slices x z (STEP y) layers x left/right (STEP x sign)
hull = vtail
lo, hi = bounds(hull)

for z0 in np.arange(lo[2], hi[2], DX):
    for y0 in np.arange(lo[1], hi[1], DZ):
        for xs in ((lo[0] - 1, 0.0), (0.0, hi[0] + 1)):
            V, c = cell(hull, (xs[0], y0, z0), (xs[1], y0 + DZ, z0 + DX))

            if V < MIN_VOL:
                continue

            width = V / (DX * DZ)            # mm, along span (model y)
            cm = to_model(c)
            # centre the box on the volume centroid in y; x/z centred on the cell (cell-sized)
            centre = to_model((c[0], y0 + DZ / 2, z0 + DX / 2))
            centre[1] = cm[1]
            out['hull'].append({'pos': centre.round(5).tolist(), 'size': [DX / 1000, round(width / 1000, 5), DZ / 1000]})
            total['hull'] += V

# --- wing panels: span slices, relative to the pivot
lo, hi = bounds(wing)

for x0 in np.arange(lo[0], hi[0], DY_WING):
    V, c = cell(wing, (x0, lo[1] - 1, lo[2] - 1), (x0 + DY_WING, hi[1] + 1, hi[2] + 1))

    if V < MIN_VOL:
        continue

    thick = 40.0
    chord = V / (DY_WING * thick)            # mm, along model x
    centre = to_model(c)
    out['wing'].append({'pos': centre.round(5).tolist(), 'size': [round(chord / 1000, 5), DY_WING / 1000, thick / 1000]})
    total['wing'] += V

with open(OUT, 'w') as f:
    json.dump(out, f, indent=1)

for k in ('hull', 'wing'):
    print('%s: %d boxes, %.2f L (%.1f N buoyancy fully submerged)' % (k, len(out[k]), total[k] / 1e6, total[k] / 1e6 * 9.81))
