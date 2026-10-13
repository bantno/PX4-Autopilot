#!/usr/bin/env python3
"""VSPAERO .stab -> gz-sim AdvancedLiftDrag coefficients for the AHAB Gazebo model.

AdvancedLiftDrag is a single linear stability-derivative model (one value per derivative), so the
alpha x beta tables that JSBSim uses are reduced here:

- static terms are least-squares fits over the flight window (alpha -4..12 deg, beta -6..6 deg);
  the residual of each fit is reported so the linearization error is visible;
- control derivatives are the window median of the ConGrp columns (they are nearly constant), split
  per surface and converted to per-degree (the plugin multiplies by the joint angle in degrees);
- rate derivatives are NOT taken from the .stab (they are noisy across alpha, see
  documentation/ahab_rate_derivative_identification.md); they come from rate_derivatives.yaml, and only
  if that has no value does this fall back to the VSPAERO window median, flagged as such.

Sign conventions (checked against gz-sim 8 AdvancedLiftDrag.cc and JSBSim twin_tractor.xml FCS):
the plugin works in body FRD with standard aero signs, alpha/beta from the body velocity, and rates
nondimensionalized as p*b/2V, q*c/2V, r*b/2V -- the same as the VSPAERO rows CL/CD/CS/CMl/CMm/CMn.
VSPAERO groups (same as stab_to_jsbsim.py): ConGrp_1 aileron, + = left TE up / right TE down;
ConGrp_2 elevator, + = both TE down; ConGrp_3 rudder, + = left TE up / right TE down.
Model joints (and PX4 servo outputs) are + = trailing edge UP on every surface.
"""

import argparse
import importlib.util
import json
import math
import os
import statistics
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..', '..', '..'))
STAB_TO_JSBSIM = os.path.join(REPO, 'Tools/simulation/jsbsim/jsbsim_bridge/models/TwinTractor/tools/stab_to_jsbsim.py')
DEFAULT_STAB = os.path.join(REPO, 'AHAB_combined_betasym.stab')
DEFAULT_RATES = os.path.join(HERE, '..', 'rate_derivatives.yaml')

SREF = 0.425   # m^2, VSPAERO Sref_ (= JSBSim wingarea)
CREF = 0.25    # m,   VSPAERO Cref_
BREF = 1.70    # m,   VSPAERO Bref_
AR = BREF**2 / SREF

ALPHA_WIN = (-4.0, 12.0)
BETA_WIN = (-6.0, 6.0)

# Placeholder until the incoming aero/stall model: VSPAERO (VLM) has no stall.
ALPHA_STALL_RAD = math.radians(15.0)

D2R = math.pi / 180.0

RATE_TERMS = ['CLp', 'CLq', 'CLr', 'CDp', 'CDq', 'CDr', 'CYp', 'CYq', 'CYr',
              'Cellp', 'Cellq', 'Cellr', 'Cemp', 'Cemq', 'Cemr', 'Cenp', 'Cenq', 'Cenr']
# plugin name -> (.stab row, rate column)
ROW = {'CL': 'CL', 'CD': 'CD', 'CY': 'CS', 'Cell': 'CMl', 'Cem': 'CMm', 'Cen': 'CMn'}


def load_parser():
    spec = importlib.util.spec_from_file_location('stab_to_jsbsim', STAB_TO_JSBSIM)
    mod = importlib.util.module_from_spec(spec)
    argv = sys.argv
    sys.argv = [argv[0]]
    spec.loader.exec_module(mod)
    sys.argv = argv
    return mod


def lsq(x, y):
    """y = a + b x; returns (a, b, rms residual)."""
    A = np.vstack([np.ones_like(x), x]).T
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ coef
    return coef[0], coef[1], float(np.sqrt(np.mean(res**2)))


def fit(stab_path, rates_path):
    st = load_parser()
    grid, alphas, betas = st.parse_stab(stab_path)
    C = st.COL_IDX
    report = []
    a_win = [a for a in alphas if ALPHA_WIN[0] <= a <= ALPHA_WIN[1]]
    b_win = [b for b in betas if BETA_WIN[0] <= b <= BETA_WIN[1]]

    if 0.0 not in betas:
        sys.exit('ERROR: .stab has no beta = 0 blocks')

    # --- longitudinal, beta = 0
    al = np.array([a * D2R for a in a_win])
    CL = np.array([grid[(a, 0.0)]['CL'][C['Total']] for a in a_win])
    CD = np.array([grid[(a, 0.0)]['CD'][C['Total']] for a in a_win])
    Cm = np.array([grid[(a, 0.0)]['CMm'][C['Total']] for a in a_win])

    CL0, CLa, rCL = lsq(al, CL)
    CD0, k, rCD = lsq(CL**2, CD)
    eff = 1.0 / (math.pi * AR * k)
    Cem0, Cema, rCm = lsq(al, Cm)
    report.append('CL  = %.4f + %.3f a        (rms %.4f)' % (CL0, CLa, rCL))
    report.append('CD  = %.4f + CL^2/(pi*%.2f*%.2f)  (rms %.5f)  -- VSPAERO, see README: ~4-5x low vs flight'
                  % (CD0, AR, eff, rCD))
    report.append('Cm  = %.4f + %.3f a        (rms %.4f; Cm is curved, slope -0.87..-1.82 across the window)'
                  % (Cem0, Cema, rCm))

    # --- lateral: regress Total vs beta over the whole (alpha, beta) window
    bb, CY, Cl, Cn = [], [], [], []

    for a in a_win:
        for b in b_win:
            r = grid[(a, b)]
            bb.append(b * D2R)
            CY.append(r['CS'][C['Total']])
            Cl.append(r['CMl'][C['Total']])
            Cn.append(r['CMn'][C['Total']])

    bb = np.array(bb)
    _, CYb, rCY = lsq(bb, np.array(CY))
    _, Cellb, rCl = lsq(bb, np.array(Cl))
    _, Cenb, rCn = lsq(bb, np.array(Cn))
    report.append('CYb %.3f  Clb %.4f  Cnb %.4f  (rms %.4f %.4f %.4f)' % (CYb, Cellb, Cenb, rCY, rCl, rCn))

    def median(row, col):
        return statistics.median(grid[(a, b)][row][C[col]] for a in a_win for b in b_win)

    # --- controls: per surface, per degree, joint + = TE up
    G = {g: {p: median(ROW[p], g) for p in ROW} for g in ('ConGrp_1', 'ConGrp_2', 'ConGrp_3')}
    surfaces = {}

    for p in ROW:
        sym = p in ('CL', 'CD', 'Cem')
        ail = 0.0 if sym else 0.5 * G['ConGrp_1'][p]   # antisymmetric group: symmetric part not identifiable
        surfaces.setdefault('servo_0', {})[p] = ail * D2R                                         # left aileron
        surfaces.setdefault('servo_1', {})[p] = -ail * D2R                                        # right aileron
        surfaces.setdefault('servo_2', {})[p] = (-0.5 * G['ConGrp_2'][p] + 0.5 * G['ConGrp_3'][p]) * D2R  # left rv
        surfaces.setdefault('servo_3', {})[p] = (-0.5 * G['ConGrp_2'][p] - 0.5 * G['ConGrp_3'][p]) * D2R  # right rv

    report.append('aileron group symmetric residual (ignored): CL %.4f CD %.5f Cm %.4f per rad'
                  % (G['ConGrp_1']['CL'], G['ConGrp_1']['CD'], G['ConGrp_1']['Cem']))

    # --- rate derivatives
    rates = {}
    rate_src = {}
    given = {}

    if rates_path and os.path.exists(rates_path):
        with open(rates_path) as f:
            given = (yaml.safe_load(f) or {}).get('derivatives', {}) or {}

    col = {'p': 'p', 'q': 'q', 'r': 'r'}

    for t in RATE_TERMS:
        v = given.get(t)

        if isinstance(v, dict):
            v = v.get('value')

        if v is not None:
            rates[t] = float(v)
            rate_src[t] = 'rate_derivatives.yaml'
        else:
            rates[t] = median(ROW[t[:-1]], col[t[-1]])
            rate_src[t] = 'VSPAERO window median (FALLBACK)'

    return {
        'area': SREF, 'mac': CREF, 'AR': AR, 'eff': eff,
        'CL0': CL0, 'CLa': CLa, 'CD0': CD0, 'Cem0': Cem0, 'Cema': Cema,
        'CYb': CYb, 'Cellb': Cellb, 'Cenb': Cenb,
        'alpha_stall': ALPHA_STALL_RAD, 'Cema_stall': Cema,
        'rates': rates, 'rate_source': rate_src,
        'surfaces': surfaces,
        'stab': os.path.relpath(stab_path, REPO),
        'report': report,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stab', default=DEFAULT_STAB)
    ap.add_argument('--rates', default=DEFAULT_RATES)
    ap.add_argument('--json', help='write the fitted coefficients')
    args = ap.parse_args()

    out = fit(args.stab, args.rates)

    for line in out['report']:
        print(line)

    print('control derivatives per degree (joint + = TE up):')

    for s, d in out['surfaces'].items():
        print('  %s ' % s + ' '.join('%s %+.5f' % (k, v) for k, v in d.items()))

    print('rate derivatives:')

    for t in RATE_TERMS:
        print('  %-6s %+8.4f  %s' % (t, out['rates'][t], out['rate_source'][t]))

    if args.json:
        with open(args.json, 'w') as f:
            json.dump(out, f, indent=1)

    return 0


if __name__ == '__main__':
    sys.exit(main())
