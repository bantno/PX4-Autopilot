#!/usr/bin/env python3
"""Merge the STEP-derived buoyancy boxes into fewer, larger boxes (simulation speed).

Every collision shape costs gz physics time each step whether or not it touches anything; the ~310 boxes from
tools/step_to_collision.py were ~60 % of the gz step (Phase 0 benchmark, 2026-10-14). Boxes are merged only
along x, within one z-layer and one y-band, so the vertical section (waterplane area, heave/roll stiffness) is
kept; each merged box keeps the exact volume and x-centroid of its parts.

    merge_boxes.py [--tol 0.25] [--max-len 0.35]   # prints counts and the hydrostatic comparison

The hydrostatic check samples both box sets on a grid and compares submerged volume and centre of buoyancy over
draft x pitch x heel (heel 0..180 deg covers capsize/self-right), as relative/absolute errors.
"""
import argparse
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BOXES_JSON = os.path.join(HERE, '..', 'collision_boxes.json')


def _key(b):
    # same z-layer (centre and height), same y-band (centre and width class)
    return (round(b['pos'][2], 4), round(b['size'][2], 4), round(b['pos'][1], 3))


def merge(boxes, tol=0.25, max_len=0.35, gap=1e-3):
    """Greedy merge of x-contiguous boxes sharing z-layer and y-centre whose widths (y size) stay within
    tol (relative) of the run's first box and whose merged length stays <= max_len [m]."""
    groups = {}

    for b in boxes:
        groups.setdefault(_key(b), []).append(b)

    out = []

    for _, g in sorted(groups.items()):
        g.sort(key=lambda b: b['pos'][0])
        run = [g[0]]

        def flush(run):
            x0 = min(b['pos'][0] - b['size'][0] / 2 for b in run)
            x1 = max(b['pos'][0] + b['size'][0] / 2 for b in run)
            h = run[0]['size'][2]
            vol = sum(b['size'][0] * b['size'][1] * b['size'][2] for b in run)
            xc = sum(b['pos'][0] * b['size'][0] * b['size'][1] for b in run) / sum(
                b['size'][0] * b['size'][1] for b in run)
            length = x1 - x0
            width = vol / (length * h)
            out.append({'pos': [xc, run[0]['pos'][1], run[0]['pos'][2]], 'size': [length, width, h]})

        for b in g[1:]:
            last = run[-1]
            contiguous = (b['pos'][0] - b['size'][0] / 2) - (last['pos'][0] + last['size'][0] / 2) < gap
            x0 = run[0]['pos'][0] - run[0]['size'][0] / 2
            within = abs(b['size'][1] - run[0]['size'][1]) <= tol * max(run[0]['size'][1], 1e-6)
            short = (b['pos'][0] + b['size'][0] / 2) - x0 <= max_len

            if contiguous and within and short:
                run.append(b)
            else:
                flush(run)
                run = [b]

        flush(run)

    return out


def _samples(boxes, h=0.004):
    """Grid points (N,3) and their volume weights for a list of axis-aligned boxes."""
    pts, w = [], []

    for b in boxes:
        n = [max(1, int(math.ceil(s / h))) for s in b['size']]
        axes = [b['pos'][i] - b['size'][i] / 2 + (np.arange(n[i]) + 0.5) * b['size'][i] / n[i] for i in range(3)]
        g = np.stack(np.meshgrid(*axes, indexing='ij'), -1).reshape(-1, 3)
        pts.append(g)
        w.append(np.full(len(g), b['size'][0] * b['size'][1] * b['size'][2] / len(g)))

    return np.concatenate(pts), np.concatenate(w)


def _rot(roll, pitch):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Ry @ Rx


def hydrostatics(pts, w, origin_z, roll, pitch):
    """Submerged volume [m^3] and centre of buoyancy (world, model origin at x=y=0, z=origin_z), water at z=0."""
    p = pts @ _rot(roll, pitch).T
    p[:, 2] += origin_z
    sub = p[:, 2] < 0.0
    v = w[sub].sum()
    cb = (p[sub] * w[sub, None]).sum(0) / v if v > 0 else np.zeros(3)
    return v, cb


def compare(a, b, label=''):
    pa, wa = _samples(a)
    pb, wb = _samples(b)
    worst = {'vol_rel': 0.0, 'cb_mm': 0.0, 'gz_mm': 0.0}
    rows = []

    for z in (0.10, 0.14, 0.16, 0.18, 0.22):
        for pitch in (-10, -5, 0, 5, 10):
            for roll in (0, 15, 30, 60, 90, 120, 150, 180):
                va, ca = hydrostatics(pa, wa, z, math.radians(roll), math.radians(pitch))
                vb, cbb = hydrostatics(pb, wb, z, math.radians(roll), math.radians(pitch))

                if va < 1e-5:
                    continue

                rel = abs(vb - va) / va
                d = np.linalg.norm(cbb - ca) * 1000
                # righting-arm proxy: lateral CB offset (y) at heel -> GZ about the origin
                gz = abs(cbb[1] - ca[1]) * 1000
                worst['vol_rel'] = max(worst['vol_rel'], rel)
                worst['cb_mm'] = max(worst['cb_mm'], d)
                worst['gz_mm'] = max(worst['gz_mm'], gz)
                rows.append((z, pitch, roll, va, rel, d))

    print('%s boxes %d -> %d   worst: volume %.2f %%, CB %.1f mm, lateral CB (GZ) %.1f mm' % (
        label, len(a), len(b), 100 * worst['vol_rel'], worst['cb_mm'], worst['gz_mm']))
    return worst, rows


def merged_boxes(tol=0.25, max_len=0.35):
    with open(BOXES_JSON) as fp:
        d = json.load(fp)

    return {'hull': merge(d['hull'], tol, max_len), 'wing': merge(d['wing'], tol, max_len)}, d


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tol', type=float, default=0.25)
    ap.add_argument('--max-len', type=float, default=0.35)
    args = ap.parse_args()
    m, d = merged_boxes(args.tol, args.max_len)
    # hull and wing compared together in the wing-level (flight/float) configuration, model frame
    compare(d['hull'] + d['wing'], m['hull'] + m['wing'], 'all')
    compare(d['hull'], m['hull'], 'hull')
    compare(d['wing'], m['wing'], 'wing')


if __name__ == '__main__':
    main()
