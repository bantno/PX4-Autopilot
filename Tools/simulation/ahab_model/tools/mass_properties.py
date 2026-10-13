#!/usr/bin/env python3
"""AHAB mass properties: measured bifilar-pendulum data -> Gazebo link inertials.

Reads Tools/simulation/ahab_model/mass_properties.yaml (procedure:
documentation/ahab_mass_properties_procedure.md), computes the whole-aircraft and wing-assembly inertia about
their CGs, derives the hull (tilt mechanism included) by parallel-axis subtraction at wing-level, removes the
small moving parts that are separate links in the model (ailerons, props on the wing; ruddervators on the hull),
and reports everything in the Gazebo model frame (x forward, y left, z up; Nick's mesh frame).

Any body whose measurements are still null falls back to the PLACEHOLDER block in the yaml, with a warning.

    python3 mass_properties.py                 # report
    python3 mass_properties.py --json out.json # also write link data for build_model.py
    python3 mass_properties.py --selftest      # synthetic known-answer test
"""

import argparse
import json
import math
import os
import statistics
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_YAML = os.path.join(HERE, '..', 'mass_properties.yaml')

# Datum <-> model frame. Nose tip and keel bottom located from the bounds of ahab_main.glb after its node
# transform and the hull visual rotation (rpy 1.5708 0 0): nose x = +0.4832, keel z = -0.1415.
NOSE_X_MODEL = 0.4832
KEEL_Z_MODEL = -0.1415

# Child-link CG locations in the model frame (from Nick's ahab_base.xacro geometry), +y side; mirrored for -y.
PART_POS_MODEL = {
    'aileron': (-0.143, 0.6178, 0.135),      # hinge (-0.118, 0.6178, 0.135) + 25 mm aft
    'ruddervator': (-0.6205, 0.1508, 0.169),  # hinge (-0.5955, 0.1508, 0.169) + 25 mm aft
    'prop': (0.092, 0.2373, 0.1408),          # hub
}
PART_PARENT = {'aileron': 'wing', 'prop': 'wing', 'ruddervator': 'hull'}

# JSBSim twin_tractor reference (4.0 kg) for the comparison line only.
JSBSIM_I = {'xx': 0.1012, 'yy': 0.0521, 'zz': 0.073}


def part_inertia(name, m):
    """Own-axis inertia of a child part (thin plate / disc approximations, as in Nick's xacro)."""
    if name == 'aileron':
        return np.diag([m * 0.325**2 / 12, m * 0.055**2 / 12, m * (0.325**2 + 0.055**2) / 12])

    if name == 'ruddervator':
        return np.diag([m * 0.255**2 / 12, m * 0.051**2 / 12, m * (0.255**2 + 0.051**2) / 12])

    # prop: disc of radius 0.127 m spinning about model x
    return np.diag([m * 0.127**2 / 3, m * 0.127**2 / 6, m * 0.127**2 / 6])


def datum_to_model(p):
    x, y, z = p
    return np.array([NOSE_X_MODEL - x, -y, KEEL_Z_MODEL + z])


# datum axes (x aft, y right, z up) -> model axes (x fwd, y left, z up)
R_DM = np.diag([-1.0, -1.0, 1.0])


def bifilar(m, L, d, T, g):
    """Bifilar pendulum: I = m g d^2 T^2 / (16 pi^2 L)."""
    return m * g * d * d * T * T / (16.0 * math.pi**2 * L)


def axis_inertia(test, m, g):
    """Inertia about the CG axis from one bifilar test; returns (I, notes)."""
    T = statistics.median(test['times']) / test['swings']
    cradle = test.get('cradle')
    m_tot = m
    I_cradle = 0.0

    if cradle:
        Tc = statistics.median(cradle['times']) / cradle['swings']
        I_cradle = bifilar(cradle['mass'], test['L'], test['d'], Tc, g)
        m_tot = m + cradle['mass']

    I = bifilar(m_tot, test['L'], test['d'], T, g) - I_cradle
    e = test.get('axis_offset') or 0.0
    I -= m * e * e
    spread = (max(test['times']) - min(test['times'])) / statistics.median(test['times'])
    return I, 'T=%.3f s, trial spread %.1f %%' % (T, 100 * spread)


def tensor_about(I_cg, m, cg, origin):
    """Parallel-axis shift of a full inertia tensor from the CG to `origin`."""
    r = np.asarray(cg) - np.asarray(origin)
    return I_cg + m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))


def body_from_yaml(doc, name, warnings):
    """Return dict(mass, cg[datum], I[3x3 datum about cg], measured: bool)."""
    b = doc[name]
    g = doc.get('g', 9.80665)
    pend = b['pendulum']
    axes = ('roll', 'pitch', 'yaw')
    have = (b['mass'] is not None and b['cg']['x'] is not None and b['cg']['z'] is not None
        and all(pend[a]['L'] and pend[a]['d'] and pend[a]['times'] for a in axes))

    if not have:
        filled = b['mass'] is not None or any(pend[a]['times'] for a in axes)

        if filled:
            sys.exit('ERROR: %s measurements are partially filled; complete mass, cg x/z and all three '
                 'pendulum axes (L, d, times) or clear them' % name)

        ph = doc['placeholder'][name]
        warnings.append('%s: PLACEHOLDER mass properties (not measured)' % name)
        I = np.diag([ph['I']['xx'], ph['I']['yy'], ph['I']['zz']])
        return {'mass': ph['mass'], 'cg': np.array([ph['cg']['x'], ph['cg']['y'], ph['cg']['z']]),
            'I': I, 'measured': False, 'notes': {}}

    m = b['mass']
    notes = {}
    Ivals = {}

    for a in axes:
        Ivals[a], notes[a] = axis_inertia(pend[a], m, g)

    I = np.diag([Ivals['roll'], Ivals['pitch'], Ivals['yaw']])
    return {'mass': m, 'cg': np.array([b['cg']['x'], b['cg']['y'] or 0.0, b['cg']['z']]),
        'I': I, 'measured': True, 'notes': notes}


def check_tensor(name, I, warnings):
    ev = np.linalg.eigvalsh(I)

    if ev.min() <= 0:
        warnings.append('%s: inertia not positive definite (eigenvalues %s)' % (name, np.round(ev, 5)))
        return

    a, b, c = sorted(ev)

    if a + b < c * 0.999:
        warnings.append('%s: principal moments violate the triangle inequality (%.4f + %.4f < %.4f)'
                % (name, a, b, c))


def compute(doc):
    warnings = []
    ac = body_from_yaml(doc, 'aircraft', warnings)
    wg = body_from_yaml(doc, 'wing', warnings)

    piv = doc['tilt_pivot']

    if piv['x'] is None or piv['z'] is None:
        piv = doc['placeholder']['tilt_pivot']
        warnings.append('tilt pivot: PLACEHOLDER location (not measured)')

    pivot = np.array([piv['x'], 0.0, piv['z']])

    parts = {}

    for p in ('aileron', 'ruddervator', 'prop'):
        v = doc['parts'].get(p)

        if v is None:
            v = doc['placeholder']['parts'][p]
            warnings.append('%s mass: estimate %.3f kg (not weighed)' % (p, v))

        parts[p] = v

    # Hull = aircraft - wing, everything about the datum origin (wing level).
    O = np.zeros(3)
    m_h = ac['mass'] - wg['mass']

    if m_h <= 0:
        sys.exit('ERROR: wing mass >= aircraft mass')

    cg_h = (ac['mass'] * ac['cg'] - wg['mass'] * wg['cg']) / m_h
    I_h_O = tensor_about(ac['I'], ac['mass'], ac['cg'], O) - tensor_about(wg['I'], wg['mass'], wg['cg'], O)
    # Shift back from the datum origin to the hull CG.
    hull = {'mass': m_h, 'cg': cg_h,
            'I': I_h_O - m_h * (np.dot(cg_h, cg_h) * np.eye(3) - np.outer(cg_h, cg_h))}

    # Remove child parts (point masses + own inertia) from their parents -> link "core" bodies.
    def remove_parts(body, names):
        m = body['mass']
        first = m * body['cg']
        I_O = tensor_about(body['I'], m, body['cg'], O)

        for n in names:
            for side in (1, -1):
                pm = parts[n]
                x, y, z = PART_POS_MODEL[n]
                # model -> datum for the part position
                p_d = np.array([NOSE_X_MODEL - x, -(side * y), z - KEEL_Z_MODEL])
                I_part = R_DM.T @ part_inertia(n, pm) @ R_DM
                I_O = I_O - tensor_about(I_part, pm, p_d, O)
                first = first - pm * p_d
                m -= pm

        cg = first / m
        I_cg = I_O - m * (np.dot(cg, cg) * np.eye(3) - np.outer(cg, cg))
        return {'mass': m, 'cg': cg, 'I': I_cg}

    hull_core = remove_parts(hull, [n for n, par in PART_PARENT.items() if par == 'hull'])
    wing_core = remove_parts(wg, [n for n, par in PART_PARENT.items() if par == 'wing'])

    for name, body in (('aircraft', ac), ('wing assembly', wg), ('hull', hull),
               ('hull_link', hull_core), ('wing_link', wing_core)):
        check_tensor(name, body['I'], warnings)

    # Recompose the aircraft from the model links (round-trip check of the arithmetic and the part handling).
    m_r = hull_core['mass'] + wing_core['mass']
    first = hull_core['mass'] * hull_core['cg'] + wing_core['mass'] * wing_core['cg']
    I_r = tensor_about(hull_core['I'], hull_core['mass'], hull_core['cg'], O) \
        + tensor_about(wing_core['I'], wing_core['mass'], wing_core['cg'], O)

    for n, pm in parts.items():
        for side in (1, -1):
            x, y, z = PART_POS_MODEL[n]
            p_d = np.array([NOSE_X_MODEL - x, -(side * y), z - KEEL_Z_MODEL])
            I_r = I_r + tensor_about(R_DM.T @ part_inertia(n, pm) @ R_DM, pm, p_d, O)
            first = first + pm * p_d
            m_r += pm

    cg_r = first / m_r
    I_r_cg = I_r - m_r * (np.dot(cg_r, cg_r) * np.eye(3) - np.outer(cg_r, cg_r))
    recompose = {
        'mass_err': abs(m_r - ac['mass']) / ac['mass'],
        'cg_err_m': float(np.linalg.norm(cg_r - ac['cg'])),
        'I_err': float(np.max(np.abs(np.diag(I_r_cg) - np.diag(ac['I'])) / np.diag(ac['I']))),
    }

    def to_model(body, origin_model=np.zeros(3)):
        cg_m = datum_to_model(body['cg'])
        return {'mass': float(body['mass']),
            'cg': (cg_m - origin_model).tolist(),
            'I': (R_DM @ body['I'] @ R_DM.T).tolist()}

    pivot_model = datum_to_model(pivot)
    out = {
        'measured': {'aircraft': ac['measured'], 'wing': wg['measured']},
        'pivot_model': pivot_model.tolist(),
        'aircraft_cg_model': datum_to_model(ac['cg']).tolist(),
        'hull_link': to_model(hull_core),                  # pose relative to the model origin
        'wing_link': to_model(wing_core, pivot_model),     # pose relative to the pivot (wing link origin)
        'parts': {n: {'mass': m, 'pos_model': PART_POS_MODEL[n], 'I': np.diag(part_inertia(n, m)).tolist()}
              for n, m in parts.items()},
        'tilt_drive': doc.get('tilt_drive', {}),
        'warnings': warnings,
    }
    return out, ac, wg, hull, recompose


def report(out, ac, wg, hull, recompose):
    def line(name, b):
        I = b['I']
        print('  %-14s m=%6.3f kg  cg(datum)=(%6.3f, %6.3f, %6.3f) m  Ixx=%.4f Iyy=%.4f Izz=%.4f Ixz=%+.4f'
              % (name, b['mass'], *b['cg'], I[0][0], I[1][1], I[2][2], I[0][2]))

    print('Bodies (datum: x aft of nose, y right, z up from keel; inertia about own CG):')
    line('aircraft', ac)
    line('wing assembly', wg)
    line('hull', hull)

    for name in ('aircraft', 'wing'):
        b = ac if name == 'aircraft' else wg

        for a, n in b.get('notes', {}).items():
            print('    %s %-5s %s' % (name, a, n))

    print('\nGazebo links (model frame: x fwd, y left, z up):')

    for name in ('hull_link', 'wing_link'):
        L = out[name]
        I = L['I']
        ref = 'model origin' if name == 'hull_link' else 'pivot'
        print('  %-10s m=%6.3f kg  cg=(%+.4f, %+.4f, %+.4f) rel. %s  Ixx=%.5f Iyy=%.5f Izz=%.5f Ixz=%+.5f'
              % (name, L['mass'], *L['cg'], ref, I[0][0], I[1][1], I[2][2], I[0][2]))

    print('  pivot (model) = (%+.4f, %+.4f, %+.4f)' % tuple(out['pivot_model']))
    print('\nRecomposition check: mass %.2e, CG %.2e m, inertia %.2e (relative)'
          % (recompose['mass_err'], recompose['cg_err_m'], recompose['I_err']))
    print('JSBSim (4.0 kg) for comparison: Ixx=%.4f Iyy=%.4f Izz=%.4f'
          % (JSBSIM_I['xx'], JSBSIM_I['yy'], JSBSIM_I['zz']))

    if out['warnings']:
        print('\nWARNINGS:')

        for w in out['warnings']:
            print('  - ' + w)


def selftest():
    """Known answer: hull = box, wing = thin plate above it. Synthesize the pendulum timings with the inverse
    bifilar formula, run the pipeline, and check the hull comes back."""
    g = 9.80665
    L, d, n = 1.8, 0.6, 20

    def box_I(m, a, b, c):  # dims along x, y, z
        return np.diag([m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12])

    hull = {'mass': 3.6, 'cg': np.array([0.44, 0.0, 0.12]), 'I': box_I(3.6, 1.0, 0.16, 0.21)}
    wing = {'mass': 1.4, 'cg': np.array([0.48, 0.0, 0.285]), 'I': box_I(1.4, 0.25, 1.82, 0.03)}
    m = hull['mass'] + wing['mass']
    cg = (hull['mass'] * hull['cg'] + wing['mass'] * wing['cg']) / m
    O = np.zeros(3)
    I_O = tensor_about(hull['I'], hull['mass'], hull['cg'], O) + tensor_about(wing['I'], wing['mass'], wing['cg'], O)
    I_ac = I_O - m * (np.dot(cg, cg) * np.eye(3) - np.outer(cg, cg))

    def times(mass, Ival):
        T = math.sqrt(Ival * 16 * math.pi**2 * L / (mass * g * d * d))
        return [T * n * f for f in (0.998, 1.0, 1.003)]

    def pend(mass, I):
        return {ax: {'L': L, 'd': d, 'swings': n, 'times': times(mass, I[i][i]), 'axis_offset': 0.0, 'cradle': None}
            for ax, i in (('roll', 0), ('pitch', 1), ('yaw', 2))}

    doc = {
        'g': g,
        'aircraft': {'mass': m, 'cg': {'x': cg[0], 'y': 0.0, 'z': cg[2]}, 'pendulum': pend(m, I_ac)},
        'wing': {'mass': wing['mass'], 'cg': {'x': wing['cg'][0], 'y': 0.0, 'z': wing['cg'][2]},
             'pendulum': pend(wing['mass'], wing['I'])},
        'tilt_pivot': {'x': 0.48, 'z': 0.285},
        'parts': {'aileron': 0.02, 'ruddervator': 0.015, 'prop': 0.025},
    }
    out, ac, wg, hull_c, rec = compute(doc)
    got = np.diag(hull_c['I'])
    want = np.diag(hull['I'])
    err = np.max(np.abs(got - want) / want)
    print('selftest: hull inertia recovered %s vs true %s (max rel err %.2e)' % (np.round(got, 5), np.round(want, 5), err))
    print('selftest: hull CG %s vs true %s' % (np.round(hull_c['cg'], 5), hull['cg']))
    print('selftest: recomposition %s' % rec)
    ok = err < 1e-9 and np.allclose(hull_c['cg'], hull['cg']) and rec['I_err'] < 1e-9 and rec['mass_err'] < 1e-12
    print('selftest: %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('yaml', nargs='?', default=DEFAULT_YAML)
    ap.add_argument('--json', help='write link data for build_model.py')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    with open(args.yaml) as f:
        doc = yaml.safe_load(f)

    out, ac, wg, hull, rec = compute(doc)
    report(out, ac, wg, hull, rec)

    if args.json:
        with open(args.json, 'w') as f:
            json.dump(out, f, indent=1)

    return 0


if __name__ == '__main__':
    sys.exit(main())
