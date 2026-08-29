#!/usr/bin/env python3
"""
Flight-test figure generation from a PX4 ULog.

Generates a set of PNG figures characterizing a manual fixed-wing flight
(takeoff roll, climb, attitude/rates, stick response, landing, power,
estimator health) plus an optional JSON summary/timeseries export suitable
for building an interactive report.

Flight events (arm, throttle-up, liftoff, land, disarm) are auto-detected:
liftoff is wheels-off (altitude rising), not the land-detector transition,
which on fixed-wing can fire several seconds early/late.

Usage:
    python3 Tools/flight_report.py <log.ulg> [--out DIR] [--json PATH] [--dpi N]

Defaults: figures go to <log dir>/figures/. Requires pyulog, numpy, matplotlib.

First used for the PLAHAB takeoff test 2026-08-28 (log_214).
"""
import argparse
import json
import os
import sys

import numpy as np
import pyulog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# ---------------------------------------------------------------- palette
C = dict(blue='#2a78d6', orange='#eb6834', aqua='#1baf7a', yellow='#eda100',
         magenta='#e87ba4', green='#008300', violet='#4a3aa7', red='#e34948')
INK = '#0b0b0b'; INK2 = '#52514e'; MUTED = '#898781'
GRID = '#e1e0d9'; AXIS = '#c3c2b7'; SURFACE = '#fcfcfb'
SEQ = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b']

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'text.color': INK, 'axes.labelcolor': INK2, 'xtick.color': MUTED, 'ytick.color': MUTED,
    'axes.edgecolor': AXIS, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
    'axes.spines.top': False, 'axes.spines.right': False,
    'font.family': 'DejaVu Sans', 'font.size': 10, 'axes.titlesize': 11,
    'axes.titleweight': 'bold', 'figure.titlesize': 14, 'figure.titleweight': 'bold',
    'lines.linewidth': 1.6, 'legend.frameon': False,
})


def euler_from_q(q):
    w, x, y, z = q
    roll = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1, 1)))
    yaw = np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw


def segments(t, mask_arr, gap=2.0):
    """Contiguous True-runs of mask_arr as (t_start, t_end) tuples."""
    idx = np.where(mask_arr)[0]
    if len(idx) == 0:
        return []
    segs = []
    s = p = idx[0]
    for i in idx[1:]:
        if t[i] - t[p] > gap:
            segs.append((t[s], t[p]))
            s = i
        p = i
    segs.append((t[s], t[p]))
    return segs


class Flight:
    """Loads a ULog and auto-detects the flight events used by every figure."""

    def __init__(self, path):
        self.ulog = pyulog.ULog(path)
        u = self.ulog

        def get(name, idx=0):
            return next(d for d in u.data_list if d.name == name and d.multi_id == idx)
        self.get = get

        lp = get('vehicle_local_position')
        self.t_lp_abs = lp.data['timestamp'] / 1e6
        self.alt = -lp.data['z']
        self.gs = np.hypot(lp.data['vx'], lp.data['vy'])
        self.climb = -lp.data['vz']
        self.x_n, self.y_e = lp.data['x'], lp.data['y']

        att = get('vehicle_attitude')
        self.t_att_abs = att.data['timestamp'] / 1e6
        self.roll, self.pitch, self.yaw = euler_from_q(
            np.array([att.data[f'q[{i}]'] for i in range(4)]))

        am = get('actuator_motors')
        self.t_am_abs = am.data['timestamp'] / 1e6
        ctrl = np.array([am.data[f'control[{i}]'] for i in range(12)])
        with np.errstate(invalid='ignore'):
            self.motor_idx = [i for i in range(12)
                              if np.isfinite(ctrl[i]).any() and np.nanmax(np.abs(ctrl[i])) > 0]
        if not self.motor_idx:
            sys.exit('no active motor channels found')
        self.motors = [np.nan_to_num(ctrl[i]) for i in self.motor_idx]
        self.thr_mean = np.mean(self.motors, axis=0)

        self._detect_events()

    def _detect_events(self):
        vs = self.get('vehicle_status')
        t_vs = vs.data['timestamp'] / 1e6
        armed = vs.data['arming_state'] == 2
        # armed spans as (arm transition, disarm transition) pairs
        spans = []
        start = t_vs[0] if armed[0] else None
        for i in range(1, len(armed)):
            if armed[i] and not armed[i - 1]:
                start = t_vs[i]
            elif not armed[i] and armed[i - 1] and start is not None:
                spans.append((start, t_vs[i]))
                start = None
        if start is not None:
            spans.append((start, t_vs[-1]))
        if not spans:
            sys.exit('no armed period found in log')

        # pick the armed span with the largest altitude excursion (the flight)
        def alt_range(sp):
            m = (self.t_lp_abs >= sp[0]) & (self.t_lp_abs <= sp[1])
            return np.ptp(self.alt[m]) if m.any() else 0
        span = max(spans, key=alt_range)
        self.t_arm, self.t_disarm = span

        # throttle-up: first sustained mean motor command above 0.3 while armed
        m = (self.t_am_abs >= self.t_arm) & (self.t_am_abs <= self.t_disarm)
        idx = np.where(m & (self.thr_mean > 0.3))[0]
        if len(idx) == 0:
            sys.exit('throttle never exceeded 0.3 while armed — no takeoff in log?')
        self.t_thrup = self.t_am_abs[idx[0]]

        # ground altitude and liftoff: first climb away from ground after throttle-up
        pre = (self.t_lp_abs > self.t_arm) & (self.t_lp_abs < self.t_thrup)
        alt0 = np.median(self.alt[pre]) if pre.any() else self.alt[
            np.searchsorted(self.t_lp_abs, self.t_thrup)]
        m = (self.t_lp_abs > self.t_thrup) & (self.t_lp_abs < self.t_disarm)
        idx = np.where(m & (self.alt > alt0 + 0.5) & (self.climb > 0.3))[0]
        if len(idx) == 0:
            sys.exit('no liftoff detected (alt never rose 0.5 m above ground)')
        i_lift = idx[0]
        self.t_liftoff = self.t_lp_abs[i_lift]
        self.alt_ground = alt0

        # ground roll stats
        i0 = np.searchsorted(self.t_lp_abs, self.t_thrup)
        self.ground_roll_m = float(np.hypot(self.x_n[i_lift] - self.x_n[i0],
                                            self.y_e[i_lift] - self.y_e[i0]))
        self.ground_roll_s = float(self.t_liftoff - self.t_thrup)
        self.liftoff_gs = float(self.gs[i_lift])
        self.liftoff_pitch = float(self.pitch[np.searchsorted(self.t_att_abs, self.t_liftoff)])

        # land detected: landed 0->1 after liftoff (fallback: disarm)
        self.t_land = self.t_disarm
        try:
            ld = self.get('vehicle_land_detected')
            t_ld = ld.data['timestamp'] / 1e6
            landed = ld.data['landed'].astype(bool)
            for i in range(1, len(landed)):
                if landed[i] and not landed[i - 1] and t_ld[i] > self.t_liftoff:
                    self.t_land = t_ld[i]
                    break
        except StopIteration:
            pass

        # runway heading at brake release
        self.runway_hdg = float(np.median(
            self.yaw[(self.t_att_abs >= self.t_thrup) & (self.t_att_abs <= self.t_thrup + 1)]))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', help='ULog file (.ulg)')
    ap.add_argument('--out', help='output directory for figures (default: <log dir>/figures)')
    ap.add_argument('--json', help='also write a summary + downsampled-timeseries JSON here')
    ap.add_argument('--dpi', type=int, default=150)
    args = ap.parse_args()

    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.log)) or '.', 'figures')
    os.makedirs(out, exist_ok=True)

    F = Flight(args.log)
    T0 = F.t_liftoff
    log_name = os.path.basename(args.log)

    def ts(d):
        return d.data['timestamp'] / 1e6 - T0

    get = F.get
    t_lp = F.t_lp_abs - T0
    alt, gs, climb, x_n, y_e = F.alt, F.gs, F.climb, F.x_n, F.y_e
    t_att = F.t_att_abs - T0
    roll, pitch, yaw = F.roll, F.pitch, F.yaw
    t_am = F.t_am_abs - T0

    av = get('vehicle_angular_velocity')
    t_av = ts(av)
    p_rate, q_rate, r_rate = (np.degrees(av.data[f'xyz[{i}]']) for i in range(3))

    mc = get('manual_control_setpoint')
    t_mc = ts(mc)
    stk_roll, stk_pitch, stk_yaw, stk_thr = (
        mc.data[k] for k in ('roll', 'pitch', 'yaw', 'throttle'))

    bat = get('battery_status')
    t_bat = ts(bat)
    volt, curr = bat.data['voltage_v'], bat.data['current_a']
    mah = bat.data['discharged_mah']
    power = volt * curr
    e_wh = np.concatenate([[0], np.cumsum(np.diff(t_bat) * 0.5 * (power[1:] + power[:-1])) / 3600.0])

    try:
        wind = get('wind')
        t_w = ts(wind)
        wind_spd = np.hypot(wind.data['windspeed_north'], wind.data['windspeed_east'])
    except StopIteration:
        t_w = wind_spd = None

    FL = dict(arm=F.t_arm - T0, thrup=F.t_thrup - T0, lift=0.0,
              land=F.t_land - T0, disarm=F.t_disarm - T0)
    print('events (s rel. liftoff): ' + ', '.join(f'{k}={v:.1f}' for k, v in FL.items()))
    print(f'ground roll {F.ground_roll_m:.0f} m / {F.ground_roll_s:.1f} s, '
          f'liftoff {F.liftoff_gs:.1f} m/s at {F.liftoff_pitch:.1f} deg pitch, '
          f'runway hdg {F.runway_hdg:.0f} deg')

    def mask(t, a, b):
        return (t >= a) & (t <= b)

    def event_lines(ax, events=('lift', 'land')):
        for e in events:
            ax.axvline(FL[e], color=MUTED, lw=1.0, ls=(0, (4, 3)), zorder=1)

    def label_events(ax, events=('lift', 'land')):
        lbl = dict(lift='liftoff', land='land detected', thrup='throttle-up',
                   arm='arm', disarm='disarm')
        for e in events:
            ax.annotate(lbl[e], (FL[e], 1.01), xycoords=('data', 'axes fraction'),
                        ha='center', fontsize=8, color=MUTED)

    def finish(fig, path):
        fig.savefig(os.path.join(out, path), dpi=args.dpi, bbox_inches='tight')
        plt.close(fig)
        print('wrote', path)

    W = (FL['arm'] - 2, FL['disarm'] + 2)
    mcol = [C['blue'], C['orange'], C['aqua'], C['violet']]

    def plot_motors(ax, tmask):
        for k, (i, mo) in enumerate(zip(F.motor_idx, F.motors)):
            ax.plot(t_am[tmask], mo[tmask], color=mcol[k % 4], label=f'motor {i}')

    # ------------------------------------------------ 1. flight overview
    fig, axs = plt.subplots(5, 1, figsize=(11, 12), sharex=True)
    fig.suptitle(f'Flight overview — {log_name}')
    m = mask(t_lp, *W)
    axs[0].plot(t_lp[m], alt[m], color=C['blue'])
    axs[0].set_ylabel('altitude (m)')
    axs[0].set_title('Altitude')
    label_events(axs[0])
    axs[1].plot(t_lp[m], gs[m], color=C['blue'], label='groundspeed')
    if t_w is not None:
        mw = mask(t_w, *W)
        axs[1].plot(t_w[mw], wind_spd[mw], color=C['orange'], label='wind estimate')
    axs[1].set_ylabel('speed (m/s)')
    axs[1].set_title('Groundspeed and wind')
    axs[1].legend(loc='upper right', ncol=2)
    axs[2].plot(t_lp[m], climb[m], color=C['blue'])
    axs[2].axhline(0, color=AXIS, lw=1)
    axs[2].set_ylabel('climb rate (m/s)')
    axs[2].set_title('Climb rate')
    mm = mask(t_am, *W)
    plot_motors(axs[3], mm)
    axs[3].set_ylabel('motor cmd (0–1)')
    axs[3].set_title('Motor commands')
    axs[3].legend(loc='upper right', ncol=4)
    mb = mask(t_bat, *W)
    axs[4].plot(t_bat[mb], power[mb], color=C['blue'])
    axs[4].set_ylabel('power (W)')
    axs[4].set_title('Electrical power')
    axs[4].set_xlabel('time since liftoff (s)')
    for ax in axs:
        event_lines(ax)
    finish(fig, '01_flight_overview.png')

    # ------------------------------------------------ 2. ground track
    fig, ax = plt.subplots(figsize=(9, 9))
    m = mask(t_lp, FL['thrup'], FL['land'] + 2)
    pts = np.array([y_e[m], x_n[m]]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    norm = plt.Normalize(0, np.nanmax(alt[m] - F.alt_ground))
    lc = LineCollection(segs,
                        cmap=matplotlib.colors.LinearSegmentedColormap.from_list('seq', SEQ),
                        norm=norm, linewidths=2.2)
    lc.set_array((alt[m] - F.alt_ground)[:-1])
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, label='altitude AGL (m)', shrink=0.75)
    cb.outline.set_visible(False)
    i_lift = np.searchsorted(t_lp, 0.0)
    i_td = np.searchsorted(t_lp, FL['land'])
    ax.scatter(y_e[i_lift], x_n[i_lift], s=90, color=C['green'], zorder=5, marker='^')
    ax.annotate('  liftoff', (y_e[i_lift], x_n[i_lift]), fontsize=9, color=INK2)
    ax.scatter(y_e[i_td], x_n[i_td], s=90, color=C['red'], zorder=5, marker='v')
    ax.annotate('  touchdown', (y_e[i_td], x_n[i_td]), fontsize=9, color=INK2)
    ax.set_aspect('equal')
    ax.autoscale()
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Ground track, colored by altitude')
    finish(fig, '02_ground_track.png')

    # ------------------------------------------------ 3. attitude & rates
    fig, axs = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    fig.suptitle('Attitude and body rates — full flight')
    ma = mask(t_att, *W)
    axs[0].plot(t_att[ma], roll[ma], color=C['blue'], label='roll')
    axs[0].plot(t_att[ma], pitch[ma], color=C['orange'], label='pitch')
    axs[0].axhline(0, color=AXIS, lw=1)
    axs[0].set_ylabel('angle (deg)')
    axs[0].set_title('Roll and pitch')
    axs[0].legend(loc='upper right', ncol=2)
    label_events(axs[0])
    axs[1].plot(t_att[ma], yaw[ma], color=C['aqua'])
    axs[1].set_ylabel('heading (deg)')
    axs[1].set_title('Heading')
    mr = mask(t_av, *W)
    axs[2].plot(t_av[mr], p_rate[mr], color=C['blue'], label='p (roll)')
    axs[2].plot(t_av[mr], q_rate[mr], color=C['orange'], label='q (pitch)')
    axs[2].plot(t_av[mr], r_rate[mr], color=C['aqua'], label='r (yaw)')
    axs[2].axhline(0, color=AXIS, lw=1)
    axs[2].set_ylabel('rate (deg/s)')
    axs[2].set_title('Body rates')
    axs[2].legend(loc='upper right', ncol=3)
    ms = mask(t_mc, *W)
    axs[3].plot(t_mc[ms], stk_roll[ms], color=C['blue'], label='roll stick')
    axs[3].plot(t_mc[ms], stk_pitch[ms], color=C['orange'], label='pitch stick')
    axs[3].plot(t_mc[ms], stk_yaw[ms], color=C['aqua'], label='yaw stick')
    axs[3].plot(t_mc[ms], stk_thr[ms], color=C['yellow'], label='throttle')
    axs[3].axhline(0, color=AXIS, lw=1)
    axs[3].set_ylabel('stick (-1…1)')
    axs[3].set_title('Pilot inputs')
    axs[3].set_xlabel('time since liftoff (s)')
    axs[3].legend(loc='upper right', ncol=4)
    for ax in axs:
        event_lines(ax)
    finish(fig, '03_attitude_rates.png')

    # ------------------------------------------------ 4. takeoff zoom
    TK = (FL['thrup'] - 2, 14)
    fig, axs = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
    fig.suptitle('Takeoff — ground roll, rotation, initial climb')
    m = mask(t_am, *TK)
    plot_motors(axs[0], m)
    ms = mask(t_mc, *TK)
    axs[0].plot(t_mc[ms], stk_thr[ms], color=C['yellow'], label='throttle stick')
    axs[0].set_ylabel('command (0–1)')
    axs[0].set_title('Throttle and motor commands')
    axs[0].legend(loc='lower right', ncol=3)
    for e, lab in [('thrup', 'throttle-up'), ('lift', 'liftoff')]:
        for ax in axs:
            ax.axvline(FL[e], color=MUTED, lw=1.0, ls=(0, (4, 3)))
        axs[0].annotate(lab, (FL[e], 1.02), xycoords=('data', 'axes fraction'),
                        ha='center', fontsize=8, color=MUTED)
    m = mask(t_lp, *TK)
    axs[1].plot(t_lp[m], gs[m], color=C['blue'], label='groundspeed')
    axs[1].plot(t_lp[m], climb[m], color=C['orange'], label='climb rate')
    axs[1].axhline(0, color=AXIS, lw=1)
    axs[1].set_ylabel('m/s')
    axs[1].set_title('Groundspeed and climb rate')
    axs[1].legend(loc='upper left', ncol=2)
    axs[2].plot(t_lp[m], alt[m] - F.alt_ground, color=C['blue'])
    axs[2].set_ylabel('altitude AGL (m)')
    axs[2].set_title('Altitude')
    ma = mask(t_att, *TK)
    axs[3].plot(t_att[ma], pitch[ma], color=C['orange'], label='pitch')
    axs[3].plot(t_att[ma], roll[ma], color=C['blue'], label='roll')
    axs[3].axhline(0, color=AXIS, lw=1)
    axs[3].set_ylabel('angle (deg)')
    axs[3].set_title('Pitch and roll')
    axs[3].legend(loc='upper left', ncol=2)
    hdg = F.runway_hdg
    axs[4].plot(t_att[ma], (yaw[ma] - hdg + 180) % 360 - 180, color=C['aqua'],
                label=f'heading − {hdg:.0f}°')
    mr = mask(t_av, *TK)
    axs[4].plot(t_av[mr], r_rate[mr], color=C['violet'], label='yaw rate (deg/s)')
    axs[4].axhline(0, color=AXIS, lw=1)
    axs[4].set_ylabel('deg, deg/s')
    axs[4].set_title(f'Directional control (runway heading ≈ {hdg:.0f}°)')
    axs[4].set_xlabel('time since liftoff (s)')
    axs[4].legend(loc='upper left', ncol=2)
    finish(fig, '04_takeoff_zoom.png')

    # ------------------------------------------------ 5. climb performance
    m_air = mask(t_lp, 0, FL['land'])
    alt_max = np.nanmax(alt[m_air])
    idx = np.where(m_air & (alt > F.alt_ground + 0.9 * (alt_max - F.alt_ground)))[0]
    t_top = t_lp[idx[0]] if len(idx) else FL['land']
    CL = (0, float(np.clip(t_top + 5, 30, 120)))
    fig, axs = plt.subplots(2, 2, figsize=(11, 7.5))
    fig.suptitle(f'Initial climb performance (liftoff → {alt_max - F.alt_ground:.0f} m)')
    m = mask(t_lp, *CL)
    axs[0, 0].plot(t_lp[m], alt[m] - F.alt_ground, color=C['blue'])
    axs[0, 0].set_xlabel('time since liftoff (s)')
    axs[0, 0].set_ylabel('altitude AGL (m)')
    axs[0, 0].set_title('Altitude vs time')
    axs[0, 1].plot(t_lp[m], climb[m], color=C['blue'])
    axs[0, 1].axhline(0, color=AXIS, lw=1)
    axs[0, 1].set_xlabel('time since liftoff (s)')
    axs[0, 1].set_ylabel('climb rate (m/s)')
    axs[0, 1].set_title('Climb rate vs time')
    axs[1, 0].scatter(gs[m], climb[m], s=10, color=C['blue'], alpha=0.45, edgecolors='none')
    axs[1, 0].axhline(0, color=AXIS, lw=1)
    axs[1, 0].set_xlabel('groundspeed (m/s)')
    axs[1, 0].set_ylabel('climb rate (m/s)')
    axs[1, 0].set_title('Climb rate vs groundspeed')
    mm = mask(t_am, *CL)
    axs[1, 1].plot(t_am[mm], F.thr_mean[mm], color=C['blue'])
    axs[1, 1].set_xlabel('time since liftoff (s)')
    axs[1, 1].set_ylabel('mean motor cmd (0–1)')
    axs[1, 1].set_title('Throttle during climb')
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    finish(fig, '05_climb_performance.png')

    # ------------------------------------------------ 6. landing zoom
    LD = (FL['land'] - 25, FL['disarm'] + 1)
    fig, axs = plt.subplots(4, 1, figsize=(10, 10.5), sharex=True)
    fig.suptitle('Landing — final approach, touchdown, rollout')
    m = mask(t_lp, *LD)
    axs[0].plot(t_lp[m], alt[m] - F.alt_ground, color=C['blue'], label='altitude AGL')
    axs[0].plot(t_lp[m], gs[m], color=C['orange'], label='groundspeed (m/s)')
    axs[0].set_ylabel('m, m/s')
    axs[0].set_title('Altitude and groundspeed')
    axs[0].legend(loc='upper right', ncol=2)
    label_events(axs[0], ('land',))
    ma = mask(t_att, *LD)
    axs[1].plot(t_att[ma], roll[ma], color=C['blue'], label='roll')
    axs[1].plot(t_att[ma], pitch[ma], color=C['orange'], label='pitch')
    axs[1].axhline(0, color=AXIS, lw=1)
    axs[1].set_ylabel('angle (deg)')
    axs[1].set_title('Attitude')
    axs[1].legend(loc='upper left', ncol=2)
    mr = mask(t_av, *LD)
    axs[2].plot(t_av[mr], r_rate[mr], color=C['aqua'], label='yaw rate')
    axs[2].plot(t_av[mr], p_rate[mr], color=C['blue'], label='roll rate')
    axs[2].axhline(0, color=AXIS, lw=1)
    axs[2].set_ylabel('rate (deg/s)')
    axs[2].set_title('Yaw and roll rates')
    axs[2].legend(loc='upper left', ncol=2)
    mm = mask(t_am, *LD)
    plot_motors(axs[3], mm)
    axs[3].set_ylabel('motor cmd (0–1)')
    axs[3].set_title('Motor commands')
    axs[3].set_xlabel('time since liftoff (s)')
    axs[3].legend(loc='upper right', ncol=4)
    for ax in axs:
        event_lines(ax, ('land',))
    finish(fig, '06_landing_zoom.png')

    # ------------------------------------------------ 7. axis response
    def resample(t_src, v, t_grid):
        return np.interp(t_grid, t_src, v)

    t_grid = np.arange(1.0, FL['land'] - 4, 0.02)
    resp_stats = {}
    if len(t_grid) > 500:
        # overlay window: the 20 s of flight with the most roll-stick activity
        sg = resample(t_mc, stk_roll, t_grid)
        win = int(20 / 0.02)
        if len(sg) > win:
            c1 = np.cumsum(sg); c2 = np.cumsum(sg**2)
            var = (c2[win:] - c2[:-win]) / win - ((c1[win:] - c1[:-win]) / win)**2
            i0 = int(np.argmax(var))
            seg = (float(t_grid[i0]), float(t_grid[i0] + 20))
        else:
            seg = (float(t_grid[0]), float(t_grid[-1]))

        roll_g = resample(t_att, roll, t_grid)
        gs_g = np.clip(resample(t_lp, gs, t_grid), 5, None)
        r_coord = np.degrees(9.81 * np.tan(np.radians(np.clip(roll_g, -80, 80))) / gs_g)
        yaw_active = np.abs(resample(t_mc, stk_yaw, t_grid)).max() > 0.3

        fig, axs = plt.subplots(3, 2, figsize=(12, 11))
        fig.suptitle('Axis response (airborne segment)')
        cfg = [('Roll', stk_roll, p_rate, C['blue']),
               ('Pitch', stk_pitch, q_rate, C['orange']),
               ('Yaw', stk_yaw, r_rate, C['aqua'])]
        for i, (name, stick, rate, col) in enumerate(cfg):
            r = resample(t_av, rate, t_grid)
            if name == 'Yaw' and not yaw_active:
                # yaw stick unused: characterize turn coordination instead
                A = np.vstack([r_coord, np.ones_like(r_coord)]).T
                (gain, bias), *_ = np.linalg.lstsq(A, r, rcond=None)
                r2 = np.corrcoef(r_coord, r)[0, 1] ** 2
                resp_stats['yaw'] = dict(gain=float(gain), lag=0.0, r2=float(r2),
                                         note='turn coordination (yaw stick unused)')
                msg = mask(t_grid, *seg)
                axs[i, 0].plot(t_grid[msg], r[msg], color=col, label='measured yaw rate')
                axs[i, 0].plot(t_grid[msg], r_coord[msg], color=MUTED, lw=1.4,
                               ls=(0, (5, 2)), label='coordinated-turn g·tanφ/V')
                axs[i, 0].set_title('Yaw: turn coordination (yaw stick unused)')
                axs[i, 1].scatter(r_coord[::5], r[::5], s=6, color=col, alpha=0.25,
                                  edgecolors='none')
                xs = np.array([np.nanmin(r_coord), np.nanmax(r_coord)])
                axs[i, 1].plot(xs, xs * gain + bias, color=INK2, lw=1.6)
                axs[i, 1].plot(xs, xs, color=MUTED, lw=1.2, ls=(0, (5, 2)))
                axs[i, 1].set_xlabel('coordinated-turn yaw rate (deg/s)')
                axs[i, 1].set_title(f'Yaw: slope {gain:.2f} of ideal (dashed), R²={r2:.2f}')
            else:
                s = resample(t_mc, stick, t_grid)
                s0, r0 = s - s.mean(), r - r.mean()
                lags = np.arange(0, 51)
                cc = [np.corrcoef(s0[:len(s0) - k] if k else s0,
                                  r0[k:] if k else r0)[0, 1] for k in lags]
                best = int(lags[np.nanargmax(np.abs(cc))])
                lag_s = best * 0.02
                s_sh = s[:len(s) - best] if best else s
                r_sh = r[best:] if best else r
                A = np.vstack([s_sh, np.ones_like(s_sh)]).T
                (gain, bias), *_ = np.linalg.lstsq(A, r_sh, rcond=None)
                r2 = np.corrcoef(s_sh, r_sh)[0, 1] ** 2
                resp_stats[name.lower()] = dict(gain=float(gain), lag=float(lag_s),
                                                r2=float(r2))
                msg = mask(t_grid, *seg)
                axs[i, 0].plot(t_grid[msg], r[msg], color=col, label='measured rate')
                axs[i, 0].plot(t_grid[msg], s[msg] * gain + bias, color=MUTED, lw=1.4,
                               ls=(0, (5, 2)), label='stick × gain')
                axs[i, 0].set_title(f'{name}: measured vs stick-predicted rate')
                axs[i, 1].scatter(s_sh[::5], r_sh[::5], s=6, color=col, alpha=0.25,
                                  edgecolors='none')
                xs = np.array([np.nanmin(s_sh), np.nanmax(s_sh)])
                axs[i, 1].plot(xs, xs * gain + bias, color=INK2, lw=1.6)
                axs[i, 1].set_xlabel(f'{name.lower()} stick (-1…1)')
                axs[i, 1].set_title(f'{name}: gain {gain:+.0f} deg/s per stick, '
                                    f'lag {lag_s*1000:.0f} ms, R²={r2:.2f}')
            axs[i, 0].axhline(0, color=AXIS, lw=1)
            axs[i, 0].set_ylabel(f'{name.lower()} rate (deg/s)')
            axs[i, 0].legend(loc='upper right', ncol=2, fontsize=8)
            axs[i, 1].set_ylabel(f'{name.lower()} rate (deg/s)')
        axs[2, 0].set_xlabel('time since liftoff (s)')
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        finish(fig, '07_stick_response.png')

        # -------------------------------------------- 11. authority vs speed
        # Control authority varies with dynamic pressure. Fit rate = K*(V/Vref)^n
        # * stick for n = 0, 1, 2 and keep the law with the best R² per axis
        # (classically roll ~V, pitch initial response ~V²; a narrow speed band
        # or maneuver/speed correlation can make lower orders win).
        vref = float(np.round(np.median(gs_g)))
        fig, axs = plt.subplots(1, 2, figsize=(11, 4.6))
        fig.suptitle(f'Control authority vs speed (reference {vref:.0f} m/s)')
        for ax, (name, stick, rate, col) in zip(axs, cfg[:2]):
            s = resample(t_mc, stick, t_grid)
            r = resample(t_av, rate, t_grid)
            laws = {}
            for expn in (0, 1, 2):
                reg = s * (gs_g / vref) ** expn
                s0, r0 = reg - reg.mean(), r - r.mean()
                cc = [np.corrcoef(s0[:len(s0) - k] if k else s0,
                                  r0[k:] if k else r0)[0, 1] for k in range(51)]
                k = int(np.nanargmax(np.abs(cc)))
                rs, rr = (reg[:len(reg) - k], r[k:]) if k else (reg, r)
                A = np.vstack([rs, np.ones_like(rs)]).T
                (g, b), *_ = np.linalg.lstsq(A, rr, rcond=None)
                pred = rs * g + b
                r2 = 1 - np.sum((rr - pred)**2) / np.sum((rr - rr.mean())**2)
                laws[expn] = (float(g), k, float(r2))
            best = max(laws, key=lambda e: laws[e][2])
            g_ref, kbest, r2b = laws[best]
            resp_stats[name.lower()].update(
                speed_law_exp=best, gain_at_vref=g_ref, vref=vref, r2_speed_law=r2b)
            # per-speed-bin plain gains at the best law's lag
            s_sh = s[:len(s) - kbest] if kbest else s
            r_sh = r[kbest:] if kbest else r
            V_sh = gs_g[:len(gs_g) - kbest] if kbest else gs_g
            edges = np.quantile(V_sh, np.linspace(0, 1, 6))
            xs, ys, ns = [], [], []
            for lo, hi in zip(edges[:-1], edges[1:]):
                m = (V_sh >= lo) & (V_sh < hi) & (np.abs(s_sh) > 0.05)
                if m.sum() < 200:
                    continue
                A = np.vstack([s_sh[m], np.ones(m.sum())]).T
                (g, b), *_ = np.linalg.lstsq(A, r_sh[m], rcond=None)
                xs.append(float(V_sh[m].mean()))
                ys.append(float(g))
                ns.append(int(m.sum()))
            if ns:
                ax.scatter(xs, ys, s=[30 + 120 * n / max(ns) for n in ns],
                           color=col, zorder=5, label='per-speed-bin fit (size = samples)')
            vv = np.linspace(edges[0] - 1, edges[-1] + 1, 50)
            ax.plot(vv, g_ref * (vv / vref) ** best, color=INK2, lw=1.6, ls=(0, (5, 2)),
                    label=f'best law V^{best}: {g_ref:.0f} °/s @ {vref:.0f} m/s (R²={r2b:.2f})')
            ax.set_xlabel('groundspeed (m/s)')
            ax.set_ylabel('gain (deg/s per full stick)')
            ax.set_title(name)
            ax.legend(loc='upper left', fontsize=8)
        fig.tight_layout(rect=(0, 0, 1, 0.92))
        finish(fig, '11_authority_vs_speed.png')

    # ------------------------------------------------ 8. power & energy
    fig, axs = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    fig.suptitle('Battery and energy')
    mb = mask(t_bat, *W)
    axs[0].plot(t_bat[mb], volt[mb], color=C['blue'])
    axs[0].set_ylabel('voltage (V)')
    axs[0].set_title('Battery voltage')
    label_events(axs[0])
    axs[1].plot(t_bat[mb], curr[mb], color=C['blue'])
    axs[1].set_ylabel('current (A)')
    axs[1].set_title('Battery current')
    axs[2].plot(t_bat[mb], power[mb], color=C['blue'])
    axs[2].set_ylabel('power (W)')
    axs[2].set_title('Power')
    axs[3].plot(t_bat[mb], e_wh[mb] - e_wh[mb][0], color=C['blue'], label='energy (Wh)')
    axs[3].plot(t_bat[mb], (mah[mb] - mah[mb][0]) / 100, color=C['orange'],
                label='charge (mAh ÷ 100)')
    axs[3].set_ylabel('Wh, mAh÷100')
    axs[3].set_title('Cumulative energy and charge')
    axs[3].set_xlabel('time since liftoff (s)')
    axs[3].legend(loc='upper left', ncol=2)
    for ax in axs:
        event_lines(ax)
    finish(fig, '08_power_energy.png')

    # ------------------------------------------------ 9. estimator health
    try:
        itr = get('estimator_innovation_test_ratios')
        gps_data = get('vehicle_gps_position')
        t_itr = ts(itr)
        t_gps = ts(gps_data)
        sats = gps_data.data['satellites_used']
        fig, axs = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
        fig.suptitle('Estimator health (EKF innovation test ratios; >1 = measurement rejected)')
        mi = mask(t_itr, *W)
        axs[0].plot(t_itr[mi], itr.data['gps_hpos[0]'][mi], color=C['blue'], label='GPS pos N')
        axs[0].plot(t_itr[mi], itr.data['gps_hpos[1]'][mi], color=C['orange'], label='GPS pos E')
        axs[0].plot(t_itr[mi], itr.data['gps_vpos'][mi], color=C['aqua'], label='GPS pos D')
        axs[0].set_title('GPS position innovations')
        axs[1].plot(t_itr[mi], itr.data['gps_hvel[0]'][mi], color=C['blue'], label='GPS vel N')
        axs[1].plot(t_itr[mi], itr.data['gps_hvel[1]'][mi], color=C['orange'], label='GPS vel E')
        axs[1].plot(t_itr[mi], itr.data['gps_vvel'][mi], color=C['aqua'], label='GPS vel D')
        axs[1].set_title('GPS velocity innovations')
        axs[2].plot(t_itr[mi], itr.data['baro_vpos'][mi], color=C['blue'], label='baro height')
        axs[2].plot(t_itr[mi], itr.data['heading'][mi], color=C['orange'], label='heading')
        axs[2].set_title('Baro and heading innovations')
        for ax in axs[:3]:
            ax.axhline(1, color=C['red'], lw=1.2, ls=(0, (4, 3)))
            ax.set_ylabel('test ratio')
            ax.legend(loc='upper right', ncol=3)
        label_events(axs[0])
        mg = mask(t_gps, *W)
        axs[3].plot(t_gps[mg], sats[mg], color=C['blue'])
        axs[3].set_ylabel('satellites used')
        axs[3].set_title('GPS satellites')
        axs[3].set_xlabel('time since liftoff (s)')
        for ax in axs:
            event_lines(ax)
        finish(fig, '09_estimator_health.png')
    except (StopIteration, KeyError) as e:
        print('skipping estimator figure:', e)

    # ------------------------------------------------ 10. maneuver zooms
    ma_fl = mask(t_att, 2, FL['land'] - 2)
    segs40 = [(a, b) for a, b in segments(t_att, ma_fl & (np.abs(roll) > 40))
              if b - a >= 3.0]
    segs40.sort(key=lambda s: s[1] - s[0], reverse=True)
    picks = sorted(segs40[:2])
    if picks:
        fig, axs = plt.subplots(3, len(picks), figsize=(6 * len(picks), 10),
                                sharex='col', squeeze=False)
        fig.suptitle('Maneuver zooms — sustained-bank segments (|roll| > 40°)')
        for ci, (a, b) in enumerate(picks):
            a, b = a - 3, b + 3
            ma = mask(t_att, a, b)
            axs[0, ci].plot(t_att[ma], roll[ma], color=C['blue'], label='roll')
            axs[0, ci].plot(t_att[ma], pitch[ma], color=C['orange'], label='pitch')
            axs[0, ci].axhline(0, color=AXIS, lw=1)
            axs[0, ci].set_title(f't = {a + 3:.0f}–{b - 3:.0f} s')
            axs[0, ci].legend(loc='upper right', ncol=2, fontsize=8)
            mr = mask(t_av, a, b)
            axs[1, ci].plot(t_av[mr], p_rate[mr], color=C['blue'], label='p')
            axs[1, ci].plot(t_av[mr], q_rate[mr], color=C['orange'], label='q')
            axs[1, ci].plot(t_av[mr], r_rate[mr], color=C['aqua'], label='r')
            axs[1, ci].axhline(0, color=AXIS, lw=1)
            axs[1, ci].legend(loc='upper right', ncol=3, fontsize=8)
            ml = mask(t_lp, a, b)
            axs[2, ci].plot(t_lp[ml], alt[ml] - F.alt_ground, color=C['blue'],
                            label='altitude AGL (m)')
            axs[2, ci].plot(t_lp[ml], gs[ml], color=C['orange'], label='groundspeed (m/s)')
            axs[2, ci].legend(loc='upper right', ncol=2, fontsize=8)
            axs[2, ci].set_xlabel('time since liftoff (s)')
        axs[0, 0].set_ylabel('angle (deg)')
        axs[1, 0].set_ylabel('rate (deg/s)')
        axs[2, 0].set_ylabel('m, m/s')
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        finish(fig, '10_maneuvers.png')

    # ------------------------------------------------ summary / JSON export
    mfl = mask(t_lp, 0, FL['land'])
    mb_fl = mask(t_bat, 0, FL['land'])
    summary = dict(
        log=log_name,
        flight_time_s=float(FL['land']),
        ground_roll_m=round(F.ground_roll_m, 1),
        ground_roll_s=round(F.ground_roll_s, 2),
        liftoff_gs=round(F.liftoff_gs, 1),
        liftoff_pitch=round(F.liftoff_pitch, 1),
        runway_hdg=round(F.runway_hdg, 1),
        max_alt_agl=float(np.nanmax(alt[mfl]) - F.alt_ground),
        max_gs=float(np.nanmax(gs[mfl])),
        max_climb=float(np.nanmax(climb[mfl])),
        min_climb=float(np.nanmin(climb[mfl])),
        energy_wh=float(e_wh[mb_fl][-1] - e_wh[mb_fl][0]) if mb_fl.any() else None,
        mah=float(mah[mb_fl][-1] - mah[mb_fl][0]) if mb_fl.any() else None,
        mean_power=float(np.mean(power[mb_fl])) if mb_fl.any() else None,
        max_power=float(np.max(power[mb_fl])) if mb_fl.any() else None,
        wind_mean=float(np.mean(wind_spd[mask(t_w, 0, FL['land'])]))
        if t_w is not None else None,
        response=resp_stats,
        events_abs_s={k: float(v + T0) for k, v in FL.items()},
    )
    print(json.dumps(summary, indent=1))

    if args.json:
        def ds(t, v, a=W[0], b=W[1], hz=5):
            tg = np.arange(a, b, 1.0 / hz)
            return np.round(np.interp(tg, t, v), 3).tolist()

        export = dict(
            summary=summary,
            events={k: round(v, 1) for k, v in FL.items()},
            t=np.round(np.arange(W[0], W[1], 0.2), 2).tolist(),
            alt=ds(t_lp, alt - F.alt_ground), gs=ds(t_lp, gs), climb=ds(t_lp, climb),
            roll=ds(t_att, roll), pitch=ds(t_att, pitch), yaw=ds(t_att, yaw),
            p=ds(t_av, p_rate), q=ds(t_av, q_rate), r=ds(t_av, r_rate),
            stk_roll=ds(t_mc, stk_roll), stk_pitch=ds(t_mc, stk_pitch),
            stk_yaw=ds(t_mc, stk_yaw), stk_thr=ds(t_mc, stk_thr),
            motors={i: ds(t_am, mo) for i, mo in zip(F.motor_idx, F.motors)},
            volt=ds(t_bat, volt), curr=ds(t_bat, curr), pwr=ds(t_bat, power),
            wh=ds(t_bat, e_wh - e_wh[np.searchsorted(t_bat, 0.0)]),
            wind=ds(t_w, wind_spd) if t_w is not None else None,
            track_e=ds(t_lp, y_e, FL['thrup'], FL['land'] + 2),
            track_n=ds(t_lp, x_n, FL['thrup'], FL['land'] + 2),
            track_alt=ds(t_lp, alt - F.alt_ground, FL['thrup'], FL['land'] + 2),
        )
        with open(args.json, 'w') as f:
            json.dump(export, f)
        print('wrote', args.json)


if __name__ == '__main__':
    main()
