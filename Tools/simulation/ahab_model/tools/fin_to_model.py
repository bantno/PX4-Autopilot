"""Place the ventral fin (keel rudder / skeg, `ventral fin.stp`) on the AHAB model: fin.glb, panels, a buoyancy box.

    ~/occ_venv/bin/python fin_to_model.py "ventral fin.stp" . ../twin_tractor_seaplane/meshes ../collision_boxes.json
    python3 build_model.py

Why: at planing speed the hull's water side force acts well ahead of the CG (the wetted forebody), so the
aircraft is directionally unstable on the water and water-loops - on the real aircraft too (user, 2026-10-13). The
fix there was this fixed fin taped flush to the hull bottom, about 50 mm behind the step, which puts water side
force far aft.

Fin file frame (Inventor, mm): same axes as AHAB_FINAL_simv2.stp (x = lateral, 3 mm thick; y = up, straight tip
edge at the bottom, curved root shaped to the afterbody; z = aft), but its own origin. Placement: leading edge
FIN_BEHIND_STEP behind the step (493 mm aft of the nose, from the hull keel profile), root resting flush on the
hull bottom (from the hull's downward-facing hydro panels). The fin is on the hull (base_link).
"""
import json
import sys

import numpy as np

sys.path.insert(0, sys.argv[2])
import step_bodies as sb  # noqa: E402

STEP, _, MESH_DIR, BOXES = sys.argv[1:5]
FIN_BEHIND_STEP = 0.050   # m, user: "about 50 mm behind the step"

fin = sb.solids(STEP)[0][1]
Vf, Ff = sb.mesh(fin, 0.2)

with open(BOXES) as f:
    cb = json.load(f)

# hull bottom along the centreline from the hull's downward-facing panels
hp = np.array([p['pos'] for p in cb['panels_hull']])
hn = np.array([p['n'] for p in cb['panels_hull']])
keel = hp[(hn[:, 2] < -0.5) & (np.abs(hp[:, 1]) < 0.03) & (hp[:, 2] < -0.05)]   # hull bottom only (not the wing root)
bins = np.arange(keel[:, 0].min(), keel[:, 0].max() + 0.01, 0.01)
profile = []

for x0 in bins:
    m = (keel[:, 0] >= x0) & (keel[:, 0] < x0 + 0.01)

    if m.any():
        profile.append((x0 + 0.005, keel[m, 2].min()))

px, pz = np.array(profile).T
bottom = lambda x: np.interp(x, px[::-1] if px[0] > px[-1] else px, pz[::-1] if px[0] > px[-1] else pz)

# step: from the hull.glb keel profile (bottom rises 38 mm between 483 and 503 mm aft of the nose; the merged
# hydro panels are too coarse to locate it reliably)
nose_x = 0.4832
STEP_AFT_OF_NOSE = 0.493
step_x = nose_x - STEP_AFT_OF_NOSE
print('step at %.0f mm aft of the nose' % (STEP_AFT_OF_NOSE * 1000))

# fin file -> model axes: model = (-z, -x, y) / 1000, like the aircraft STEP; then translate
R = lambda V: np.stack([-V[:, 2], -V[:, 0], V[:, 1]], axis=1) / 1000.0
M = R(Vf)
le_x = step_x - FIN_BEHIND_STEP
M[:, 0] += le_x - M[:, 0].max()          # leading edge (front-most point) at the target station
M[:, 1] -= M[:, 1].mean()                # on the centreline
root = M[:, 2] > np.percentile(M[:, 2], 80)   # upper (root) vertices
M[:, 2] += np.median(bottom(M[root, 0]) - M[root, 2])   # root flush with the hull bottom


def place(V):
    P = R(V)
    P[:, 0] += M[:, 0].max() - R(Vf)[:, 0].max()
    P[:, 1] += M[:, 1].mean() - R(Vf)[:, 1].mean()
    P[:, 2] += M[:, 2].max() - R(Vf)[:, 2].max()
    return P


sb.write_glb(MESH_DIR + '/fin.glb', place(Vf), Ff, 'fin', [0.2, 0.2, 0.2, 1.0])
print('fin.glb: x %.4f..%.4f (%.0f..%.0f mm aft of nose), z %.4f..%.4f, |y| <= %.4f'
      % (M[:, 0].min(), M[:, 0].max(), (nose_x - M[:, 0].max()) * 1000, (nose_x - M[:, 0].min()) * 1000,
         M[:, 2].min(), M[:, 2].max(), np.abs(M[:, 1]).max()))

c, n, a = sb.surface_panels([fin], place, deflection=1.0)
cb['panels_fin'] = [{'pos': ci.round(5).tolist(), 'n': ni.round(4).tolist(), 'area': round(float(ai), 7)}
                    for ci, ni, ai in zip(c, n, a)]

# one thin box carrying the fin's volume (buoyancy is negligible, but it also gives ground/water contact)
(_, _, _, _, _, _), vol, _ = sb.info(fin)
lo, hi = M.min(axis=0), M.max(axis=0)
size = [hi[0] - lo[0], vol / 1e9 / ((hi[0] - lo[0]) * (hi[2] - lo[2])), hi[2] - lo[2]]
cb['hull'] = [b for b in cb['hull'] if not b.get('fin')] + \
    [{'pos': ((lo + hi) / 2).round(5).tolist(), 'size': [round(v, 5) for v in size], 'fin': True}]
cb['fin_source'] = STEP.split('/')[-1]

with open(BOXES, 'w') as f:
    json.dump(cb, f, indent=1)

print('fin panels: %d, %.4f m^2 (both faces); box %s' % (len(a), a.sum(), [round(float(v), 4) for v in size]))
