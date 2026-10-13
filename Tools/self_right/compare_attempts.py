#!/usr/bin/env python3
"""Same-definition metrics for self-right attempts, e.g. Gazebo vs the real 2026-10-03 flips.

    python3 Tools/self_right/compare_attempts.py <log.ulg> <label> [<log.ulg> <label> ...]

Metrics follow the ledger in documentation/self_right_full_auto_test_2026-10-03.md (times from the start of
Righting): RotateWing duration, wing angle at Righting entry, first hull motion (> 3 deg theta change) and the
throttle then, over-center latch time and theta, peak pitch/roll rate and acceleration up to 1 s after
over-center, pitch extreme, and the settled roll/pitch 3 s after over-center. The first activation with a
Righting phase in each log is used. The throttle printed at over-center is the latch cycle's value, which can
already read 0 after the cut.
"""
import sys

import numpy as np
import pyulog

KEYS = ['theta_entry', 'rotate_s', 'wing_at_righting', 'theta_rotate', 'first_motion', 'over_center', 'theta_cut',
        'peak_q', 'peak_p', 'peak_acc', 'pitch_extreme', 'settled']


def load(path):
    u = pyulog.ULog(path, ['self_right_status', 'wing_tilt_status', 'vehicle_angular_velocity', 'sensor_combined',
                           'vehicle_attitude'])
    return {x.name: x.data for x in u.data_list if x.multi_id == 0}


def attitude(d):
    a = d['vehicle_attitude']
    q0, q1, q2, q3 = (a['q[%d]' % i] for i in range(4))
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    return a['timestamp'] / 1e6, roll, pitch


def at(t, x, t0):
    return x[np.searchsorted(t, t0).clip(0, len(t) - 1)]


def metrics(path, label):
    d = load(path)
    s = d['self_right_status']
    ts = s['timestamp'] / 1e6
    st = s['state']
    th = np.degrees(s['pitch_from_upright'])
    thr = s['throttle']
    righting = np.where(st == 3)[0]

    if len(righting) == 0:
        return None

    t_rw0 = ts[np.where(st == 2)[0][0]]
    t_r0 = ts[righting[0]]
    oc = np.where(s['over_center'] & (ts >= t_r0))[0]
    t_oc = ts[oc[0]]
    theta0 = th[ts >= t_r0][0]
    moved = np.where((ts >= t_r0) & (np.abs(th - theta0) > 3))[0]

    w = d['wing_tilt_status']
    tw = w['timestamp'] / 1e6
    av = d['vehicle_angular_velocity']
    ta = av['timestamp'] / 1e6
    win = (ta >= t_r0) & (ta <= t_oc + 1.0)
    sc = d['sensor_combined']
    tc = sc['timestamp'] / 1e6
    acc = np.sqrt(sum(sc['accelerometer_m_s2[%d]' % i] ** 2 for i in range(3)))
    winc = (tc >= t_r0) & (tc <= t_oc + 1.0)
    tt, roll, pitch = attitude(d)
    wint = (tt >= t_r0) & (tt <= t_oc + 1.0)

    return dict(label=label,
                theta_entry=th[ts >= t_rw0][0],
                rotate_s=t_r0 - t_rw0,
                wing_at_righting=at(tw, np.degrees(w['measured_angle']), t_r0),
                theta_rotate='%.0f -> %.0f' % (th[ts >= t_rw0][0], theta0),
                first_motion='+%.2f s @ thr %.2f' % (ts[moved[0]] - t_r0, thr[moved[0]]) if len(moved) else 'n/a',
                over_center='+%.2f s @ thr %.2f' % (t_oc - t_r0, thr[oc[0]]),
                theta_cut=th[oc[0]],
                peak_q=np.abs(av['xyz[1]'][win]).max(),
                peak_p=np.abs(av['xyz[0]'][win]).max(),
                peak_acc=acc[winc].max(),
                pitch_extreme=pitch[wint][np.argmax(np.abs(pitch[wint]))],
                settled='%+.1f / %+.1f' % (at(tt, roll, t_oc + 3.0), at(tt, pitch, t_oc + 3.0)))


def table(rows):
    lines = ['%-18s' % '' + ''.join('%-20s' % r['label'] for r in rows)]

    for k in KEYS:
        lines.append('%-18s' % k + ''.join('%-20s' % (('%.2f' % r[k]) if isinstance(r[k], (float, np.floating))
                                                      else r[k]) for r in rows))

    return '\n'.join(lines)


def main():
    rows = [metrics(p, l) for p, l in zip(sys.argv[1::2], sys.argv[2::2])]
    print(table([r for r in rows if r is not None]))


if __name__ == '__main__':
    main()
