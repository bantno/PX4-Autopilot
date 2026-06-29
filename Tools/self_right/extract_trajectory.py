#!/usr/bin/env python3
"""Extract a self-righting thrust trajectory and tuning seeds from a ULog.

Run this on a log of a *manual* (or SITL) righting maneuver to characterise the maneuver and seed
the self_right module parameters (SR_*). It locates the righting window (sustained throttle while the
aircraft rotates from inverted toward upright), then:

  * prints suggested SR_THR_MAX, SR_RAMP_T, SR_OVERCTR, SR_TIMEOUT values, and
  * writes a per-sample CSV table (time, throttle, pitch-from-upright, pitch-rate) usable as the
    open-loop replay trajectory (SR_STRATEGY=0).

See .CLAUDE/self_right_architecture.md for how each value feeds the controller.

Usage:
    Tools/self_right/extract_trajectory.py <log.ulg> [--out trajectory.csv] [--thr-on 0.3]
"""

import argparse
import math
import sys

try:
    from pyulog import ULog
except ImportError:
    sys.exit("pyulog is required: pip install pyulog")


def pitch_from_upright(q0, q1, q2, q3):
    """Angle [rad] of the body +Z axis from world-up (0 = upright, pi = inverted).

    Matches Quaternion::dcm_z()(2): the NED-down component of body +Z.
    """
    dcm_z_down = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    return math.acos(max(-1.0, min(1.0, dcm_z_down)))


def first_field(ulog, topic, names):
    data = ulog.get_dataset(topic).data
    for n in names:
        if n in data:
            return data[n]
    raise KeyError(f"none of {names} in {topic}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="ULog file (.ulg)")
    ap.add_argument("--out", default="trajectory.csv", help="output CSV path")
    ap.add_argument("--thr-on", type=float, default=0.3,
                    help="throttle above which the maneuver is considered active")
    ap.add_argument("--motor", type=int, default=0, help="actuator_motors control[] index to read")
    args = ap.parse_args()

    ulog = ULog(args.log)

    motors = ulog.get_dataset("actuator_motors")
    thr = motors.data[f"control[{args.motor}]"]
    thr_t = motors.data["timestamp"]

    att = ulog.get_dataset("vehicle_attitude")
    att_t = att.data["timestamp"]
    q = [att.data[f"q[{i}]"] for i in range(4)]

    # Resolve the active window: from first throttle > thr_on to the last, contiguous-ish.
    active = [i for i, v in enumerate(thr) if v is not None and v > args.thr_on
              and not math.isnan(v)]
    if not active:
        sys.exit(f"no samples with throttle > {args.thr_on}; try a lower --thr-on")

    i0, i1 = active[0], active[-1]
    t0, t1 = thr_t[i0], thr_t[i1]
    duration = (t1 - t0) * 1e-6

    # Ramp time: time from maneuver start to reaching 90% of peak throttle.
    peak = max(thr[i] for i in range(i0, i1 + 1))
    ramp_t = duration
    for i in range(i0, i1 + 1):
        if thr[i] is not None and thr[i] >= 0.9 * peak:
            ramp_t = (thr_t[i] - t0) * 1e-6
            break

    # Walk attitude over the window to find the minimum pitch-from-upright reached (the closest the
    # aircraft got to level) and the angle at which it crossed "halfway" (a tipping-point proxy).
    theta_min = math.pi
    theta_start = None
    rows = []
    for k, ts in enumerate(att_t):
        if ts < t0 or ts > t1:
            continue
        theta = pitch_from_upright(q[0][k], q[1][k], q[2][k], q[3][k])
        theta_min = min(theta_min, theta)
        if theta_start is None:
            theta_start = theta
        # nearest throttle sample
        j = min(range(i0, i1 + 1), key=lambda m: abs(thr_t[m] - ts))
        rows.append(((ts - t0) * 1e-6, thr[j], theta))

    # Over-center proxy: midpoint between the starting (inverted) angle and level.
    overctr = (theta_start or math.pi) * 0.5

    print(f"# Self-righting trajectory from {args.log}")
    print(f"#   window: {duration:.2f} s, {len(rows)} attitude samples")
    print(f"#   start pitch-from-upright: {math.degrees(theta_start or math.pi):.0f} deg")
    print(f"#   min   pitch-from-upright: {math.degrees(theta_min):.0f} deg "
          f"({'reached upright' if theta_min < 0.35 else 'did NOT reach upright'})")
    print("# Suggested parameters:")
    print(f"param set SR_THR_MAX {peak:.2f}")
    print(f"param set SR_RAMP_T  {ramp_t:.1f}")
    print(f"param set SR_OVERCTR {overctr:.2f}")
    print(f"param set SR_TIMEOUT {max(2.0, duration * 1.5):.1f}")

    with open(args.out, "w") as f:
        f.write("t_s,throttle,pitch_from_upright_rad\n")
        for t, th, theta in rows:
            f.write(f"{t:.4f},{th:.4f},{theta:.4f}\n")
    print(f"# wrote {len(rows)} rows to {args.out}")


if __name__ == "__main__":
    main()
