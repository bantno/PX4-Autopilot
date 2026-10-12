#!/usr/bin/env python3
"""
Visualize autonomous self-righting attempts from PX4 ULogs.

Finds every activation of the self_right state machine (self_right_status.state > 0) in
the given logs and writes:

  * one figure per attempt (<log>_attempt<N>.png): state bands, pitch-from-upright vs
    SR_OVERCTR, body rates, throttle command vs motor PWM, battery current/voltage, wing
    setpoint/measured/command with encoder-dropout shading, accel magnitude, and an IMU
    vibration spectrogram (prop line), with markers for over-center cut, the slam (min
    theta after the cut), leaving the Self Right mode, and disarm.
  * overview.png: theta and wing angle for all attempts that reached over-center,
    aligned at the cut.
  * a summary table on stdout (and summary.csv).

Usage:
    python3 Tools/self_right/plot_attempts.py <log.ulg | dir> [...] [--out DIR]
            [--only-flips] [--no-spectrogram] [--dpi N]

Defaults: figures go to <first log's dir>/self_right_figures/. Requires pyulog, numpy,
matplotlib. First used for the 2026-10-03 full-auto water test.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np
import pyulog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ---------------------------------------------------------------- palette (matches Tools/flight_report.py)
C = dict(blue='#2a78d6', orange='#eb6834', aqua='#1baf7a', yellow='#eda100',
         magenta='#e87ba4', green='#008300', violet='#4a3aa7', red='#e34948')
SERIES = [C[k] for k in ('blue', 'orange', 'aqua', 'yellow', 'magenta', 'green', 'violet', 'red')]
INK = '#0b0b0b'; INK2 = '#52514e'; MUTED = '#898781'
GRID = '#e1e0d9'; AXIS = '#c3c2b7'; SURFACE = '#fcfcfb'

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'text.color': INK, 'axes.labelcolor': INK2, 'xtick.color': MUTED, 'ytick.color': MUTED,
    'axes.edgecolor': AXIS, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
    'axes.spines.top': False, 'axes.spines.right': False,
    'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.titlesize': 10,
    'axes.titleweight': 'bold', 'figure.titlesize': 13, 'figure.titleweight': 'bold',
    'lines.linewidth': 1.6, 'legend.frameon': False, 'legend.fontsize': 8,
})

STATE_NAMES = {1: 'Verify', 2: 'RotateWing', 3: 'Righting', 4: 'Cut', 5: 'Disarm'}
# light state-band tints (identity carried by the band label, not color alone)
STATE_TINT = {1: '#ecebe6', 2: '#dbe8f8', 3: '#fbe1d6', 4: '#d6f0e5', 5: '#e6e3f4'}
ABORT_NAMES = {0: 'none', 1: 'STICK', 2: 'TILT_TIMEOUT', 3: 'RIGHTING_TIMEOUT', 4: 'VERIFY_FAIL'}
DISARM_REASONS = {0: 'transition', 1: 'RC stick', 2: 'RC switch', 3: 'command internal',
                  4: 'command external', 5: 'mission', 6: 'auto disarm land',
                  7: 'auto disarm preflight', 8: 'kill switch', 9: 'lockdown', 10: 'failure detector',
                  11: 'shutdown', 12: 'unit test'}
NAV_SELF_RIGHT = 16
G = 9.80665


# ---------------------------------------------------------------- log access
class Log:
    def __init__(self, path):
        self.path = path
        self.name = os.path.splitext(os.path.basename(path))[0]
        self.u = pyulog.ULog(path)
        self.t0 = np.int64(self.u.start_timestamp)
        self.params = self.u.initial_parameters

    def topic(self, name, multi=0):
        return next((d.data for d in self.u.data_list if d.name == name and d.multi_id == multi), None)

    def t(self, data, key='timestamp'):
        # int64 first: a few early samples carry timestamps before log start
        return (data[key].astype(np.int64) - self.t0) / 1e6


def euler_from_q(d):
    w, x, y, z = (d[f'q[{i}]'] for i in range(4))
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    return roll, pitch


def runs(mask):
    """(start_idx, end_idx) inclusive index pairs of contiguous True runs."""
    idx = np.flatnonzero(np.diff(np.r_[0, mask.astype(np.int8), 0]))
    return list(zip(idx[::2], idx[1::2] - 1))


def fifo_series(log, name):
    """Flatten a sensor_*_fifo topic into (t, xyz[N,3]) at its native rate."""
    d = log.topic(name)
    if d is None:
        return None, None
    n = d['samples'].astype(int)
    K = 1 + max(int(k[2:-1]) for k in d if k.startswith('x['))
    dt = d['dt'].astype(float) * 1e-6
    k = np.arange(K)[None, :]
    t = log.t(d, 'timestamp_sample')[:, None] - (n[:, None] - 1 - k) * dt[:, None]
    valid = k < n[:, None]
    xyz = np.stack([np.stack([d[f'{a}[{j}]'] for j in range(K)], 1) for a in 'xyz'], -1).astype(float)
    xyz *= d['scale'][:, None, None]
    return t[valid], xyz[valid]


# ---------------------------------------------------------------- attempt extraction
class Attempt:
    pass


def find_attempts(log):
    sr = log.topic('self_right_status')
    if sr is None:
        return []
    ts = log.t(sr)
    st = sr['state'].astype(int)
    th = np.degrees(sr['pitch_from_upright'])
    out = []
    for i0, i1 in runs(st > 0):
        a = Attempt()
        a.log, a.idx = log, len(out) + 1
        a.t_start, a.t_end = ts[i0], ts[i1] + 0.01
        seg = slice(i0, i1 + 1)
        a.states = [(int(st[i]), ts[i]) for i in range(i0, i1 + 1) if i == i0 or st[i] != st[i - 1]]
        a.abort = int(sr['abort_reason'][i1])
        a.over_center = bool(sr['over_center'][seg].any())
        cut = [t for s, t in a.states if s == 4]
        a.t_cut = cut[0] if cut else None
        rig = [t for s, t in a.states if s == 3]
        a.t_righting = rig[0] if rig else None
        a.theta_entry = th[i0]
        a.theta_cut = float(np.interp(a.t_cut, ts, th)) if a.t_cut is not None else np.nan
        a.t_slam, a.theta_slam = None, np.nan
        if a.over_center and a.t_cut is not None:
            m = (ts >= a.t_cut) & (ts <= a.t_cut + 1.5)
            j = np.flatnonzero(m)[np.argmin(th[m])]
            a.t_slam, a.theta_slam = ts[j], th[j]
        a.t_oc_latch = a.t_cut if (a.over_center and a.abort == 0) else None
        a.outcome = ('flip' if a.over_center and a.abort == 0 else
                     f'abort: {ABORT_NAMES.get(a.abort, a.abort)}')

        # how the mode ended: left Self Right nav state, or disarmed
        vs = log.topic('vehicle_status')
        tv = log.t(vs)
        a.t_mode_exit = a.t_disarm = None
        a.disarm_reason = None
        after = np.flatnonzero(tv >= a.t_start)
        for k in after:
            if a.t_mode_exit is None and vs['nav_state'][k] != NAV_SELF_RIGHT and vs['arming_state'][k] == 2:
                a.t_mode_exit = tv[k]
            if vs['arming_state'][k] != 2:
                a.t_disarm = tv[k]
                a.disarm_reason = DISARM_REASONS.get(int(vs['latest_disarming_reason'][k]),
                                                     int(vs['latest_disarming_reason'][k]))
                break
        out.append(a)
    return out


# ---------------------------------------------------------------- plotting helpers
def shade_states(ax, a, label=False):
    edges = a.states + [(0, a.t_end)]
    for (s, t_s), (_, t_e) in zip(edges[:-1], edges[1:]):
        ax.axvspan(t_s - a.t_ref, t_e - a.t_ref, color=STATE_TINT.get(s, '#eee'), lw=0, zorder=0)
        if label and t_e - t_s > 0.05:
            ax.text((t_s + t_e) / 2 - a.t_ref, 1.02, STATE_NAMES.get(s, s), transform=ax.get_xaxis_transform(),
                    ha='center', va='bottom', fontsize=8, color=INK2)


def event_lines(ax, a, text=False):
    ev = []
    if a.t_oc_latch is not None:
        ev.append((a.t_oc_latch, C['red'], 'over-center cut'))
    if a.t_slam is not None:
        ev.append((a.t_slam, C['violet'], 'slam (min θ)'))
    if a.t_mode_exit is not None and a.t_mode_exit < a.t_end + 5:
        ev.append((a.t_mode_exit, C['orange'], 'left Self Right'))
    if a.t_disarm is not None:
        ev.append((a.t_disarm, INK, f'disarm ({a.disarm_reason})'))
    for t, c, lbl in ev:
        ax.axvline(t - a.t_ref, color=c, lw=1.0, ls='--', zorder=3)
        if text:
            ax.text(t - a.t_ref, 0.98, ' ' + lbl, color=INK2, fontsize=7.5, rotation=90,
                    ha='left', va='top', transform=ax.get_xaxis_transform())


def win(log, data, key, a, pre, post):
    t = log.t(data) - a.t_ref
    m = (t >= pre) & (t <= post)
    return t[m], data[key][m]


# ---------------------------------------------------------------- per-attempt figure
def plot_attempt(a, out_dir, dpi, spectrogram=True):
    log = a.log
    a.t_ref = a.t_start
    pre = -1.0
    post = (min(a.t_disarm, a.t_end + 8) if a.t_disarm is not None else a.t_end + 3) - a.t_ref + 0.5

    sr = log.topic('self_right_status')
    wt = log.topic('wing_tilt_status')
    bat = log.topic('battery_status')
    ao = log.topic('actuator_outputs')
    att = log.topic('vehicle_attitude')
    av = log.topic('vehicle_angular_velocity')
    sc = log.topic('sensor_combined')
    enc = log.topic('sensor_encoder')
    p = log.params

    panels = ['theta', 'rates', 'throttle', 'current', 'voltage', 'wing', 'wingcmd', 'accel']
    if spectrogram and log.topic('sensor_accel_fifo') is not None:
        panels.append('spec')
    heights = {'theta': 1.6, 'wing': 1.4, 'spec': 1.4}
    fig, axs = plt.subplots(len(panels), 1, figsize=(12, 1.55 * sum(heights.get(k, 1) for k in panels) + 1),
                            sharex=True, gridspec_kw={'height_ratios': [heights.get(k, 1) for k in panels]})
    ax = dict(zip(panels, axs))

    for k, x in ax.items():
        if k != 'spec':
            shade_states(x, a, label=(k == 'theta'))
        event_lines(x, a, text=(k == 'theta'))

    # theta
    t, th = win(log, sr, 'pitch_from_upright', a, pre, post)
    x = ax['theta']
    x.plot(t, np.degrees(th), color=C['blue'], label='θ (pitch from upright)')
    oc = np.degrees(p.get('SR_OVERCTR', np.nan))
    inv = np.degrees(np.arccos(-p.get('SR_INV_THR', np.nan)))
    x.axhline(oc, color=C['red'], lw=1, ls=':')
    x.text(pre, oc, f' SR_OVERCTR {oc:.0f}°', color=INK2, fontsize=7.5, va='bottom')
    x.axhline(inv, color=MUTED, lw=1, ls=':')
    x.text(pre, inv, f' inverted gate {inv:.0f}°', color=INK2, fontsize=7.5, va='top')
    x.set_ylim(-5, 185); x.set_yticks([0, 45, 90, 135, 180]); x.set_ylabel('θ [deg]')

    # body rates
    x = ax['rates']
    for i, (lbl, c) in enumerate((('p roll', C['aqua']), ('q pitch', C['blue']), ('r yaw', C['yellow']))):
        tt, v = win(log, av, f'xyz[{i}]', a, pre, post)
        x.plot(tt, v, color=c, lw=1.2, label=lbl)
    x.set_ylabel('rate [rad/s]'); x.legend(loc='upper left', ncol=3)

    # throttle: SR command vs PWM normalized onto the same 0..1 scale
    x = ax['throttle']
    tt, v = win(log, sr, 'throttle', a, pre, post)
    x.plot(tt, v, color=C['orange'], label='self_right throttle cmd')
    pmin, pmax = p.get('PWM_MAIN_MIN1', 1000), p.get('PWM_MAIN_MAX1', 2000)
    for ch, c, ls in ((0, INK2, '-'), (1, MUTED, '--')):
        tt, v = win(log, ao, f'output[{ch}]', a, pre, post)
        x.step(tt, (v - pmin) / (pmax - pmin), where='post', color=c, lw=1.1, ls=ls, marker='.', ms=3,
               label=f'motor {ch + 1} PWM (logged {1 / np.median(np.diff(log.t(ao))):.0f} Hz)')
    x.set_ylim(-0.05, 1.1); x.set_ylabel('throttle [0-1]'); x.legend(loc='upper left', ncol=3)

    # battery
    tt, v = win(log, bat, 'current_a', a, pre, post)
    ax['current'].plot(tt, v, color=C['red']); ax['current'].set_ylabel('current [A]')
    tt, v = win(log, bat, 'voltage_v', a, pre, post)
    ax['voltage'].plot(tt, v, color=C['green']); ax['voltage'].set_ylabel('voltage [V]')

    # wing
    x = ax['wing']
    tt, sp = win(log, wt, 'setpoint', a, pre, post)
    _, meas = win(log, wt, 'measured_angle', a, pre, post)
    _, ok = win(log, wt, 'encoder_valid', a, pre, post)
    x.plot(tt, np.degrees(sp), color=C['orange'], lw=1.2, ls='--', label='setpoint')
    x.plot(tt, np.degrees(meas), color=C['blue'], label='measured')
    for r0, r1 in runs(ok == 0):
        x.axvspan(tt[r0], tt[min(r1 + 1, len(tt) - 1)], facecolor='none', edgecolor=C['red'],
                  hatch='xx', lw=0, alpha=0.6)
    if (ok == 0).any():
        x.fill_between([], [], facecolor='none', edgecolor=C['red'], hatch='xx', alpha=0.6,
                       label='encoder invalid (wing not driven)')
    if enc is not None:
        te = log.t(enc) - a.t_ref
        m = (te >= pre) & (te <= post)
        gaps = np.flatnonzero(np.diff(te[m]) > 0.05)
        for g in gaps:
            x.axvspan(te[m][g], te[m][g + 1], color=INK, alpha=0.12, lw=0, hatch='//')
        if len(gaps):
            x.plot([], [], color=INK, alpha=0.3, lw=6, label='no encoder samples (I2C)')
    tol = np.degrees(p.get('SR_TILT_TOL', 0))
    tsp = np.degrees(p.get('SR_TILT_SP', np.nan))
    x.axhspan(tsp - tol, tsp + tol, color=C['orange'], alpha=0.08, lw=0)
    x.set_ylabel('wing [deg]'); x.legend(loc='lower left', ncol=4)

    x = ax['wingcmd']
    tt, v = win(log, wt, 'output', a, pre, post)
    x.plot(tt, v, color=C['violet'], lw=1.2)
    x.set_ylim(-1.1, 1.1); x.set_ylabel('tilt cmd [-1,1]')

    # accel magnitude
    tt, _ = win(log, sc, 'timestamp', a, pre, post)
    mag = np.sqrt(sum(win(log, sc, f'accelerometer_m_s2[{i}]', a, pre, post)[1].astype(float) ** 2
                      for i in range(3))) / G
    ax['accel'].plot(tt, mag, color=C['magenta'], lw=1.0)
    ax['accel'].set_ylabel('|accel| [g]')

    # vibration spectrogram (prop line)
    if 'spec' in ax:
        x = ax['spec']
        tf, xyz = fifo_series(log, 'sensor_accel_fifo')
        m = (tf >= a.t_ref + pre) & (tf <= a.t_ref + post)
        tf, xyz = tf[m] - a.t_ref, xyz[m]
        if len(tf) > 1024:
            fs = 1 / np.median(np.diff(tf))
            W, H = 512, 64
            win_ = np.hanning(W)
            starts = np.arange(0, len(tf) - W, H)
            P = np.zeros((W // 2 + 1, len(starts)))
            for j, s0 in enumerate(starts):
                seg = xyz[s0:s0 + W] - xyz[s0:s0 + W].mean(0)
                P[:, j] = (np.abs(np.fft.rfft(seg * win_[:, None], axis=0)) ** 2).sum(1)
            fr = np.fft.rfftfreq(W, 1 / fs)
            fm = fr <= min(1200, fs / 2)
            PdB = 10 * np.log10(P[fm] + 1e-12)
            x.pcolormesh(tf[starts + W // 2], fr[fm], PdB, shading='auto', cmap='Blues',
                         vmin=np.percentile(PdB, 30), vmax=np.percentile(PdB, 99.5))
            x.set_ylabel('accel vib [Hz]')
            x.grid(False)

    axs[-1].set_xlabel(f's since mode entry ({log.name} +{a.t_start:.1f} s)')
    axs[-1].set_xlim(pre, post)
    rel = [f'cut θ={a.theta_cut:.0f}°' if a.t_cut is not None else None,
           f'Righting→cut {a.t_cut - a.t_righting:.2f} s' if a.t_cut and a.t_righting else None,
           f'slam +{a.t_slam - a.t_cut:.2f} s after cut' if a.t_slam is not None else None]
    fig.suptitle(f'{log.name} attempt {a.idx}: {a.outcome}   ' + '   '.join(r for r in rel if r), x=0.01, ha='left')
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    path = os.path.join(out_dir, f'{log.name}_attempt{a.idx}.png')
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    return path


# ---------------------------------------------------------------- overview
def plot_overview(attempts, out_dir, dpi):
    flips = [a for a in attempts if a.t_oc_latch is not None]
    if not flips:
        return None
    fig, (x1, x2) = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    for i, a in enumerate(flips):
        c = SERIES[i % len(SERIES)] if len(flips) <= len(SERIES) else None
        log = a.log
        sr, wt = log.topic('self_right_status'), log.topic('wing_tilt_status')
        lbl = f'{log.name}#{a.idx}'
        t = log.t(sr) - a.t_cut
        m = (t >= -2) & (t <= 3.5)
        x1.plot(t[m], np.degrees(sr['pitch_from_upright'][m]), color=c, label=lbl)
        t = log.t(wt) - a.t_cut
        m = (t >= -2) & (t <= 3.5)
        x2.plot(t[m], np.degrees(wt['measured_angle'][m]), color=c, label=lbl)
        if a.t_slam is not None:
            x1.plot(a.t_slam - a.t_cut, a.theta_slam, 'o', ms=8, color=c, mec=SURFACE, mew=2)
    for x in (x1, x2):
        x.axvline(0, color=C['red'], lw=1, ls='--')
    x1.text(0, 1.0, ' over-center cut', color=INK2, fontsize=8, transform=x1.get_xaxis_transform(), va='top')
    x1.set_ylabel('θ pitch from upright [deg]'); x1.set_yticks([0, 45, 90, 135, 180])
    x1.set_title('All flips aligned at the over-center cut (dot = slam, min θ)', loc='left')
    x2.set_ylabel('wing measured [deg]  (gaps = encoder invalid)')
    x2.set_xlabel('s relative to over-center cut')
    x1.legend(loc='upper right', ncol=1)
    fig.tight_layout()
    path = os.path.join(out_dir, 'overview.png')
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    return path


# ---------------------------------------------------------------- summary
def summary_rows(attempts):
    rows = []
    for a in attempts:
        log = a.log
        bat = log.topic('battery_status')
        tb = log.t(bat)
        m = (tb >= a.t_start) & (tb <= a.t_end)
        rows.append(dict(
            log=log.name, attempt=a.idx, start_s=round(a.t_start, 2), outcome=a.outcome,
            theta_entry=round(a.theta_entry, 1),
            righting_to_cut_s=round(a.t_cut - a.t_righting, 2) if a.t_cut and a.t_righting else '',
            theta_cut=round(a.theta_cut, 1) if a.t_cut else '',
            slam_after_cut_s=round(a.t_slam - a.t_cut, 2) if a.t_slam is not None else '',
            mode_exit_after_cut_s=(round(a.t_mode_exit - a.t_cut, 2)
                                   if a.t_mode_exit is not None and a.t_cut is not None else ''),
            peak_current_a=round(float(bat['current_a'][m].max()), 0) if m.any() else '',
            min_voltage_v=round(float(bat['voltage_v'][m].min()), 2) if m.any() else '',
            disarm=a.disarm_reason or '',
            SR_OVERCTR_deg=round(np.degrees(log.params.get('SR_OVERCTR', np.nan)), 1),
            TILT_KP=log.params.get('TILT_KP', ''),
        ))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('paths', nargs='+', help='.ulg files and/or directories of them')
    ap.add_argument('--out', help='output directory (default <first log dir>/self_right_figures)')
    ap.add_argument('--only-flips', action='store_true', help='skip attempts that never reached over-center')
    ap.add_argument('--no-spectrogram', action='store_true', help='skip the IMU vibration panel (faster)')
    ap.add_argument('--dpi', type=int, default=110)
    args = ap.parse_args()

    files = []
    for p in args.paths:
        files += sorted(glob.glob(os.path.join(p, '*.ulg'))) if os.path.isdir(p) else [p]
    if not files:
        sys.exit('no .ulg files found')
    out_dir = args.out or os.path.join(os.path.dirname(os.path.abspath(files[0])), 'self_right_figures')
    os.makedirs(out_dir, exist_ok=True)

    attempts = []
    for f in files:
        try:
            log = Log(f)
        except Exception as e:  # noqa: BLE001 - keep going over a folder of mixed logs
            print(f'skip {f}: {e}', file=sys.stderr)
            continue
        found = find_attempts(log)
        if args.only_flips:
            found = [a for a in found if a.t_oc_latch is not None]
        for a in found:
            print(f'{log.name} attempt {a.idx} ({a.outcome}) -> {plot_attempt(a, out_dir, args.dpi, not args.no_spectrogram)}')
        attempts += found

    if not attempts:
        sys.exit('no self_right activations found')
    ov = plot_overview(attempts, out_dir, args.dpi)
    if ov:
        print(f'overview -> {ov}')

    rows = summary_rows(attempts)
    with open(os.path.join(out_dir, 'summary.csv'), 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    cols = list(rows[0])
    width = {c: max(len(c), *(len(str(r[c])) for r in rows)) for c in cols}
    print('\n' + '  '.join(c.ljust(width[c]) for c in cols))
    for r in rows:
        print('  '.join(str(r[c]).ljust(width[c]) for c in cols))
    print(f'\nsummary -> {os.path.join(out_dir, "summary.csv")}')


if __name__ == '__main__':
    main()
