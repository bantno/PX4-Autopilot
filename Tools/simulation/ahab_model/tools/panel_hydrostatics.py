#!/usr/bin/env python3
"""Compare buoyancy from the closed hydro panel surface (pressure integration) with the box-collision buoyancy.

The WaterHydrodynamics panels (hydro_panels.txt) are closed surfaces per link (sum n*A ~ 0), so integrating the
hydrostatic pressure p = rho*g*depth over them gives the buoyant force and its centre exactly up to panel size
(divergence theorem). That lets the plugin produce buoyancy itself and the ~310 buoyancy boxes (60 % of the gz step,
Phase 0 benchmark 2026-10-14) be dropped. This tool is the offline gate: it compares panel buoyancy against the box
buoyancy that gz's graded Buoyancy system computes, over draft x pitch x heel (heel to 180 deg = inverted).

    panel_hydrostatics.py            # prints worst errors and the equilibrium float attitude for both models
"""
import json
import math
import os

import numpy as np

import merge_boxes as mb   # box sampling + hydrostatics helpers

HERE = os.path.dirname(os.path.abspath(__file__))
PANELS = os.path.join(HERE, '..', 'twin_tractor_seaplane', 'hydro_panels.txt')
RHO_G = 1000.0 * 9.8


def load_panels(path=PANELS):
    rows = [l.split() for l in open(path) if l.strip() and not l.startswith('#')]
    p = np.array([[float(v) for v in r[1:4]] for r in rows])
    n = np.array([[float(v) for v in r[4:7]] for r in rows])
    a = np.array([float(r[7]) for r in rows])
    return p, n, a


def panel_buoyancy(p, n, a, origin_z, roll, pitch):
    """Buoyant force (equivalent displaced volume [m^3]) and centre of buoyancy from pressure integration."""
    R = mb._rot(roll, pitch)
    pw = p @ R.T
    pw[:, 2] += origin_z
    nw = n @ R.T
    depth = np.maximum(0.0, -pw[:, 2])
    f = -(RHO_G * depth * a)[:, None] * nw          # pressure pushes against the outward normal
    F = f.sum(0)
    M = np.cross(pw, f).sum(0)
    vol = F[2] / RHO_G

    if vol <= 1e-7:
        return vol, np.zeros(3)

    # line of action of a vertical force through the CB: M = cb x F -> cb_x = -M_y/F_z, cb_y = M_x/F_z
    cb = np.array([-M[1] / F[2], M[0] / F[2], 0.0])
    return vol, cb


def main():
    d = json.load(open(mb.BOXES_JSON))
    boxes = d['hull'] + d['wing']
    pb, wb = mb._samples(boxes)
    pp, nn, aa = load_panels()
    worst = {'vol_rel': 0.0, 'cbx_mm': 0.0, 'cby_mm': 0.0}
    worst_at = {}

    for z in (0.10, 0.14, 0.16, 0.18, 0.22):
        for pitch in (-10, -5, 0, 5, 10):
            for roll in (0, 15, 30, 60, 90, 120, 150, 180):
                r, pt = math.radians(roll), math.radians(pitch)
                vb, cbb = mb.hydrostatics(pb, wb, z, r, pt)
                vp, cbp = panel_buoyancy(pp, nn, aa, z, r, pt)

                if vb < 2e-4:   # skip near-dry poses (relative errors meaningless)
                    continue

                e = {'vol_rel': abs(vp - vb) / vb, 'cbx_mm': abs(cbp[0] - cbb[0]) * 1000,
                     'cby_mm': abs(cbp[1] - cbb[1]) * 1000}

                for k, v in e.items():
                    if v > worst[k]:
                        worst[k] = v
                        worst_at[k] = (z, pitch, roll, round(vb, 5), round(vp, 5))

    print('panel vs box buoyancy, worst over draft x pitch x heel:')

    for k in worst:
        print('  %-8s %.3g   at (origin_z, pitch, roll, V_box, V_panel) = %s' % (
            k, worst[k] * (100 if k == 'vol_rel' else 1), worst_at.get(k)))

    print('  (vol_rel in %, CB in mm)')


if __name__ == '__main__':
    main()


def mass_cg(sdf=os.path.join(HERE, '..', 'twin_tractor_seaplane', 'model.sdf')):
    """Total mass and CG (model frame, wing level) from the generated SDF (link pose + inertial pose)."""
    import xml.etree.ElementTree as ET
    m_tot, mc = 0.0, np.zeros(3)

    for link in ET.parse(sdf).getroot().find('model').findall('link'):
        lp = np.array([float(v) for v in (link.findtext('pose') or '0 0 0 0 0 0').split()[:3]])
        ine = link.find('inertial')
        m = float(ine.findtext('mass'))
        ip = np.array([float(v) for v in (ine.findtext('pose') or '0 0 0 0 0 0').split()[:3]])
        m_tot += m
        mc += m * (lp + ip)

    return m_tot, mc / m_tot


def equilibrium_draft(vol_fn, target, roll, pitch, lo=-0.6, hi=0.8):
    """origin_z where displaced volume == target (bisection; volume decreases as origin_z rises)."""
    for _ in range(50):
        mid = 0.5 * (lo + hi)

        if vol_fn(mid, roll, pitch)[0] > target:
            lo = mid
        else:
            hi = mid

    return 0.5 * (lo + hi)


def righting_curve(pitch_deg=3.0):
    d = json.load(open(mb.BOXES_JSON))
    pb, wb = mb._samples(d['hull'] + d['wing'])
    pp, nn, aa = load_panels()
    m, cg = mass_cg()
    target = m / 1000.0
    box = lambda z, r, p: mb.hydrostatics(pb, wb, z, r, p)
    pan = lambda z, r, p: panel_buoyancy(pp, nn, aa, z, r, p)
    print('mass %.3f kg, CG %s m (model frame); pitch fixed %.0f deg' % (m, np.round(cg, 4), pitch_deg))
    print('%5s | %8s %8s | %9s %9s' % ('heel', 'z_box', 'z_pan', 'GZ_box mm', 'GZ_pan mm'))
    pt = math.radians(pitch_deg)
    rows = []

    for heel in range(0, 181, 15):
        r = math.radians(heel)
        out = []

        for fn in (box, pan):
            z = equilibrium_draft(fn, target, r, pt)
            _, cb = fn(z, r, pt)
            g = mb._rot(r, pt) @ cg
            out += [z, (cb[1] - g[1]) * 1000]   # lateral lever arm: CB - CG (y, world)

        rows.append((heel,) + tuple(out))
        print('%5d | %8.4f %8.4f | %9.1f %9.1f' % (heel, out[0], out[2], out[1], out[3]))

    return rows
