#!/usr/bin/env python3
"""Pool-log figure: how the manual self-right seeded the SR_* parameters."""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog

# palette (dataviz reference, light mode)
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK2 = '#52514e'
GRID = '#e7e6e2'
BLUE = '#2a78d6'    # series 1: attitude
ORANGE = '#eb6834'  # series 2: thrust
GREEN = '#008300'   # status: success
FAIL_TINT = '#efeeeb'
OK_TINT = '#e3edda'

u = ULog('/home/brian/PX4/PX4-Autopilot/pool_self-right_test_22_05_44.ulg')
d = {x.name: x for x in u.data_list}

att = d['vehicle_attitude'].data
thr = d['vehicle_thrust_setpoint'].data

t0 = thr['timestamp'][0] / 1e6
t_att = att['timestamp'] / 1e6 - t0
q1, q2 = att['q[1]'], att['q[2]']
theta = np.degrees(np.arccos(np.clip(1.0 - 2.0 * (q1 * q1 + q2 * q2), -1.0, 1.0)))
t_thr = thr['timestamp'] / 1e6 - t0
thrust = thr['xyz[0]']

T_END = 36.0
ma, mt = t_att <= T_END, t_thr <= T_END

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10.5, 7.0), dpi=200,
                               gridspec_kw={'height_ratios': [1.15, 1], 'hspace': 0.12})
fig.patch.set_facecolor(SURFACE)

for ax in (ax1, ax2):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color(INK2)
        ax.spines[s].set_linewidth(0.8)
    ax.tick_params(colors=INK2, labelsize=9)
    # attempt windows
    ax.axvspan(2.7, 4.7, color=FAIL_TINT, zorder=0)
    ax.axvspan(6.0, 8.8, color=FAIL_TINT, zorder=0)
    ax.axvspan(28.6, 31.3, color=OK_TINT, zorder=0)

# ---- top: pitch-from-upright ----
ax1.plot(t_att[ma], theta[ma], color=BLUE, linewidth=2.0)
ax1.set_ylabel('pitch from upright  θ  [deg]', fontsize=10, color=INK)
ax1.set_ylim(-6, 200)
ax1.set_yticks([0, 45, 90, 135, 180])

ax1.axhline(80, color=INK2, linewidth=1.2, linestyle=(0, (5, 4)))
ax1.text(0.4, 84, 'SR_OVERCTR = 1.4 rad (80°) — autonomous cut threshold',
         fontsize=8.5, color=INK, va='bottom')

ax1.text(0.4, 158, 'inverted float (~170°)', fontsize=8.5, color=INK2, va='top')
ax1.text(35.6, 12, 'upright (~7°)', fontsize=8.5, color=INK2, ha='right')

# attempt annotations
ax1.annotate('attempt 1\nfell back (123°)', xy=(4.5, 122.7), xytext=(6.2, 60),
             fontsize=8.5, color=INK, ha='center',
             arrowprops=dict(arrowstyle='-', color=INK2, lw=0.8))
ax1.annotate('attempt 2\nfell back (107°)', xy=(8.5, 107.1), xytext=(11.5, 60),
             fontsize=8.5, color=INK, ha='center',
             arrowprops=dict(arrowstyle='-', color=INK2, lw=0.8))
ax1.plot([4.5, 8.5], [122.7, 107.1], 'o', color=BLUE, ms=6,
         markerfacecolor=SURFACE, markeredgewidth=1.6)

ax1.plot(31.3, 90.2, 'o', color=GREEN, ms=7, markerfacecolor=SURFACE, markeredgewidth=1.8)
ax1.annotate('manual cut at ~90° —\nflip completes on\nmomentum + buoyancy',
             xy=(31.3, 90.2), xytext=(26.0, 132), fontsize=8.5, color=GREEN,
             ha='center',
             arrowprops=dict(arrowstyle='-', color=GREEN, lw=0.9))
ax1.text(30.0, 8, 'threshold set between:\n> 80° commits, 107° does not',
         fontsize=8.5, color=INK2, ha='right', va='bottom')

# ---- bottom: thrust ----
ax2.plot(t_thr[mt], thrust[mt], color=ORANGE, linewidth=2.0)
ax2.set_ylabel('symmetric thrust command  [0–1]', fontsize=10, color=INK)
ax2.set_xlabel('time  [s]', fontsize=10, color=INK)
ax2.set_ylim(-0.05, 1.42)
ax2.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
ax2.set_xlim(0, T_END)

ax2.axhline(1.0, color=INK2, linewidth=1.2, linestyle=(0, (5, 4)))
ax2.text(0.4, 1.04, 'SR_THR_MAX = 1.0 — every effective attempt needed full throttle',
         fontsize=8.5, color=INK, va='bottom')

ax2.annotate('0.56 held for 10 s:\nno progress (θ stays ~162°)', xy=(15, 0.57),
             xytext=(15, 0.88), fontsize=8.5, color=INK, ha='center',
             arrowprops=dict(arrowstyle='-', color=INK2, lw=0.8))

ax2.annotate('ramp ~1 s\n→ SR_RAMP_T = 1.0 s', xy=(29.3, 0.85), xytext=(25.2, 0.42),
             fontsize=8.5, color=INK, ha='center',
             arrowprops=dict(arrowstyle='-', color=INK2, lw=0.8))

# commit-duration bracket
ax2.annotate('', xy=(28.6, 1.22), xytext=(31.3, 1.22),
             arrowprops=dict(arrowstyle='<->', color=INK, lw=1.0))
ax2.text(28.0, 1.22, 'throttle-up → commit: 2.7 s\n→ SR_TIMEOUT = 5 s (~2× margin)',
         fontsize=8.5, color=INK, ha='right', va='center')

# window captions on top panel
ax1.text(3.7, 193, 'failed attempts', fontsize=8.5, color=INK2, ha='center', va='top')
ax1.text(29.95, 193, 'success', fontsize=8.5, color=GREEN, ha='center', va='top')

fig.suptitle('Seeding the self-right parameters from the manual pool flip',
             fontsize=13, fontweight='bold', color=INK, x=0.125, ha='left', y=0.975)
fig.text(0.125, 0.935, 'pool_self-right_test_22_05_44.ulg — 2026-07-24, hand-flown recovery: '
         'two failed attempts, one success', fontsize=9.5, color=INK2)

fig.subplots_adjust(top=0.90, bottom=0.08, left=0.08, right=0.97)
out = '/tmp/claude-1000/-home-brian-PX4-PX4-Autopilot/f210f796-21ca-4614-97e0-5c7f1ed7ebb1/scratchpad/self_right_param_seeding.png'
fig.savefig(out, facecolor=SURFACE)
print(out)
