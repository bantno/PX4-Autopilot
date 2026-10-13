#!/usr/bin/env python3
"""Water-model calibration scenarios for the AHAB Gazebo model (headless SITL in the ahab_lake world).

    python3 Tools/simulation/ahab_model/tools/water_scenarios.py [float drop taxi takeoff selfright] [--gui]

Builds nothing: run `make px4_sitl_default` (and tools/build_model.py) first. Each scenario starts a fresh PX4 +
Gazebo session in build/px4_sitl_default/rootfs/water_<scenario>/, drives it through the PX4 shell clients
(actuator_test, param) and MAVLink (self-right mode + arm), then reads the session's ULog.

  float     floating attitude at rest (trim, roll) - no hydrodynamic force at rest
  drop      0.5 m drop: heave oscillation count and settling time
  taxi      throttle steps 15/30/50/80/100 %, wing level: steady speed, trim, pitchpole check
  takeoff   full throttle, up-elevator 0.3 until airborne then neutral (open loop: water-loops, see README)
  autotakeoff  PX4 auto (runway) takeoff from the water: heading hold, liftoff time/distance, climb
  mission   takeoff -> climb -> PX4 land on the lake -> flip inverted -> self-right
  selfright inverted start, SR_OVERCTR 1.4: flip metrics vs the 2026-10-03 logs (Tools/self_right/compare_attempts.py)

Pass criteria are in CRITERIA and the README. Prints one summary table per scenario.
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

import numpy as np
import pyulog

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..', '..', '..', '..'))
BUILD = os.path.join(REPO, 'build', 'px4_sitl_default')
BIN = os.path.join(BUILD, 'bin')
REAL_FLIPS = os.path.expanduser('~/Logs/10-3-Self-Righting-Full-Auto')

sys.path.insert(0, os.path.join(REPO, 'Tools', 'self_right'))
import compare_attempts  # noqa: E402

FLIGHT_PARAMS = {'NAV_DLL_ACT': 0, 'COM_RC_IN_MODE': 4, 'NAV_RCL_ACT': 0}


def kill_sims():
    """Stop any PX4 SITL / gz server: px4 by exact process name, gz only when its command line starts with it
    (a loose pattern would also match shells whose command line mentions these strings)."""
    subprocess.run(['pkill', '-x', 'px4'])
    subprocess.run(['pkill', '-f', '^gz sim'])


class Sim:
    def __init__(self, name, pose=None, gui=False):
        # never overlap with a leftover session (same ports and gz topics)
        kill_sims()

        time.sleep(2)
        self.dir = os.path.join(BUILD, 'rootfs', 'water_' + name)
        shutil.rmtree(self.dir, ignore_errors=True)
        os.makedirs(self.dir)
        env = dict(os.environ, PX4_SIMULATOR='gz', PX4_GZ_WORLD='ahab_lake', PX4_SYS_AUTOSTART='4242',
                   PX4_SIM_MODEL='gz_twin_tractor_seaplane')
        env.pop('GTK_PATH', None)

        if pose:
            env['PX4_GZ_MODEL_POSE'] = pose

        if gui:
            env.update(PX4_GZ_FOLLOW='1', PX4_GZ_FOLLOW_OFFSET_X='-2.5', PX4_GZ_FOLLOW_OFFSET_Y='-2.5',
                       PX4_GZ_FOLLOW_OFFSET_Z='1.5')
        else:
            env['HEADLESS'] = '1'

        self.log = open(os.path.join(self.dir, 'px4.log'), 'w')
        self.proc = subprocess.Popen([os.path.join(BIN, 'px4'), '-d', '../../etc'], cwd=self.dir, env=env,
                                     stdout=self.log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        self.wait_for(lambda: 'Startup script returned' in open(self.log.name).read(), 120, 'startup')
        self.wait_for(lambda: 'cs_tilt_align: True' in self.px4('listener', 'estimator_status_flags'), 60, 'EKF align')

        # the hull panels must come from the panel file, not the box-collision fallback (plugin errors land in
        # px4.log; its info messages only go to gz's own console log)
        log = open(self.log.name).read()

        if 'WaterHydrodynamics: cannot open' in log or 'not found on GZ_SIM_RESOURCE_PATH' in log:
            raise RuntimeError('WaterHydrodynamics did not load its panel file (see %s)' % self.log.name)
        self.t0 = time.time()

    def wait_for(self, cond, timeout, what):
        end = time.time() + timeout

        while time.time() < end:
            if cond():
                return

            time.sleep(0.5)

        raise RuntimeError('timed out waiting for ' + what)

    def px4(self, cmd, *args):
        return subprocess.run([os.path.join(BIN, 'px4-' + cmd)] + [str(a) for a in args], capture_output=True,
                              text=True, timeout=20).stdout

    def params(self, **kv):
        for k, v in kv.items():
            self.px4('param', 'set', k, v)

    def actuators(self, values, seconds):
        """Hold output functions {function: value} for `seconds` (actuator_test refreshed every second)."""
        end = time.time() + seconds

        while time.time() < end:
            for fn, v in values.items():
                self.px4('actuator_test', 'set', '-f', fn, '-v', v, '-t', 2)

            time.sleep(1.0)

    def stop_actuators(self, functions):
        for fn in functions:
            self.px4('actuator_test', 'set', '-f', fn, '-v', 0, '-t', 1)

    def set_pose(self, z, q=(0, 0, 0, 1)):
        subprocess.run(['gz', 'service', '-s', '/world/ahab_lake/set_pose', '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req',
                        'name: "twin_tractor_seaplane_0", position: {x: 0, y: 0, z: %f}, '
                        'orientation: {x: %f, y: %f, z: %f, w: %f}' % ((z,) + tuple(q))], capture_output=True)

    def self_right(self):
        from pymavlink import mavutil
        m = mavutil.mavlink_connection('udpin:0.0.0.0:14540')
        m.wait_heartbeat(timeout=10)

        for cmd, p in ((176, (1, 12)), (400, (1,))):   # DO_SET_MODE custom 12 = SELF_RIGHT, then arm
            p = list(p) + [0] * (7 - len(p))
            m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *p)
            m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            time.sleep(0.5)

        m.close()

    def close(self):
        self.t_close = time.time()   # wall clock at the log end, to map wall-clock marks onto log time
        self.px4('shutdown')
        time.sleep(2)

        kill_sims()
        self.proc.wait(timeout=20)
        self.log.close()

    def ulog(self):
        return sorted(glob.glob(os.path.join(self.dir, 'log', '*', '*.ulg')))[-1]


def truth(path):
    u = pyulog.ULog(path, ['vehicle_attitude_groundtruth', 'vehicle_local_position_groundtruth', 'esc_status'])
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    a = d['vehicle_attitude_groundtruth']
    q0, q1, q2, q3 = (a['q[%d]' % i] for i in range(4))
    out = dict(ta=a['timestamp'] / 1e6,
               roll=np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))),
               pitch=np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1))),
               yaw=np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)))
    p = d['vehicle_local_position_groundtruth']
    tp = p['timestamp'] / 1e6
    yaw = np.interp(tp, out['ta'], np.unwrap(out['yaw']))
    out.update(tp=tp, up=-p['z'], x=p['x'], y=p['y'],
               v_fwd=p['vx'] * np.cos(yaw) + p['vy'] * np.sin(yaw), speed=np.hypot(p['vx'], p['vy']))
    return out


def window(t, x, t0, t1):
    m = (t >= t0) & (t <= t1)
    return x[m]


def end_time(path):
    return truth(path)['ta'][-1]


# ------------------------------------------------------------------------------------------------ scenarios
def scenario_float(gui):
    sim = Sim('float', gui=gui)
    time.sleep(12)
    sim.close()
    d = truth(sim.ulog())
    t1 = d['ta'][-1]
    pitch = window(d['ta'], d['pitch'], t1 - 6, t1).mean()
    roll = window(d['ta'], d['roll'], t1 - 6, t1).mean()
    up = window(d['tp'], d['up'], t1 - 6, t1).mean()
    return [('trim pitch', '%+.1f deg' % pitch), ('roll', '%+.1f deg' % roll), ('origin height', '%.3f m' % up),
            ('PASS', abs(roll) < 2 and -2 < pitch < 12)]


def scenario_drop(gui):
    sim = Sim('drop', gui=gui)
    time.sleep(8)
    sim.set_pose(0.5)
    time.sleep(10)
    sim.close()
    d = truth(sim.ulog())
    # the drop is the sudden jump in height
    jump = np.where(np.diff(d['up']) > 0.2)[0]
    t_drop = d['tp'][jump[0] + 1] if len(jump) else d['tp'][-1] - 10
    t, up = d['tp'], d['up']
    rest = window(t, up, t[-1] - 2, t[-1]).mean()
    seg = window(t, up, t_drop + 0.3, t[-1]) - rest
    peaks = np.sum((seg[1:-1] > seg[:-2]) & (seg[1:-1] > seg[2:]) & (np.abs(seg[1:-1]) > 0.005))
    ts = window(t, t, t_drop + 0.3, t[-1])
    outside = np.where(np.abs(seg) > 0.005)[0]
    settle = (ts[outside[-1]] - t_drop) if len(outside) else 0.0
    return [('heave overshoots > 5 mm', int(peaks)), ('settling time (5 mm)', '%.1f s' % settle),
            ('PASS', 1 <= peaks <= 4 and settle < 8)]


def scenario_taxi(gui):
    sim = Sim('taxi', gui=gui)
    time.sleep(5)
    marks = []

    for thr in (0.15, 0.30, 0.50, 0.80, 1.00):
        t_a = time.time()
        sim.actuators({101: thr, 102: thr, 201: 0.0, 202: 0.0, 203: 0.0, 204: 0.0}, 12)   # surfaces held neutral
        marks.append((thr, t_a, time.time()))

    sim.stop_actuators((101, 102, 201, 202, 203, 204))
    time.sleep(3)
    sim.close()
    d = truth(sim.ulog())
    offset = d['ta'][-1] - sim.t_close
    rows, ok, speeds = [], True, []

    for thr, a, b in marks:
        ta, tb = a + offset, b + offset
        v = window(d['tp'], d['v_fwd'], tb - 4, tb).mean()
        pitch = window(d['ta'], d['pitch'], tb - 4, tb).mean()
        pmin = window(d['ta'], d['pitch'], ta, tb).min()
        rmax = np.abs(window(d['ta'], d['roll'], ta, tb)).max()
        capsized = pmin < -45 or rmax > 60
        ok &= not capsized
        speeds.append(v)
        rows.append(('%3d %% throttle' % (thr * 100), 'v %.1f m/s  trim %+.1f deg  min pitch %+.0f  max |roll| %.0f%s'
                     % (v, pitch, pmin, rmax, '  CAPSIZED' if capsized else '')))

    ok &= all(b >= a - 0.2 for a, b in zip(speeds, speeds[1:]))
    return rows + [('PASS', ok)]


def scenario_takeoff(gui):
    """Full throttle with moderate up-elevator; elevator back to neutral once airborne (no autopilot)."""
    sim = Sim('takeoff', gui=gui)
    time.sleep(5)

    def height():
        out = sim.px4('listener', 'vehicle_local_position_groundtruth')

        for line in out.splitlines():
            if line.strip().startswith('z:'):
                return -float(line.split()[1])

        return float('nan')

    rest = height()
    elevator, airborne_at, end = 0.3, None, time.time() + 20

    while time.time() < end:
        # ailerons held at exactly 0: with COM_PREARM_MODE 2 the outputs are live while disarmed and the FW
        # attitude controller would otherwise drive them (full deflection seen on the water)
        for fn, v in ((101, 1.0), (102, 1.0), (201, 0.0), (202, 0.0), (203, elevator), (204, elevator)):
            sim.px4('actuator_test', 'set', '-f', fn, '-v', v, '-t', 2)

        if airborne_at is None and height() > rest + 0.3:
            airborne_at, elevator, end = time.time(), 0.0, time.time() + 4

        time.sleep(0.3)

    sim.stop_actuators((101, 102, 201, 202, 203, 204))
    time.sleep(2)
    sim.close()

    d = truth(sim.ulog())
    u = pyulog.ULog(sim.ulog(), ['esc_status'])
    e = [x for x in u.data_list if x.name == 'esc_status'][0].data
    te = e['timestamp'] / 1e6
    t0 = te[np.where(e['esc[0].esc_rpm'] > 1700)[0][0]]
    rest = window(d['tp'], d['up'], t0 - 2, t0).mean()
    off = np.where((d['tp'] > t0) & (d['up'] > rest + 0.30))[0]

    if not len(off):
        vmax = window(d['tp'], d['speed'], t0, t0 + 20).max()
        return [('left the water', 'no (max speed %.1f m/s)' % vmax), ('PASS', False)]

    i, i0 = off[0], np.searchsorted(d['tp'], t0)
    dist = np.hypot(d['x'][i] - d['x'][i0], d['y'][i] - d['y'][i0])
    peak = window(d['tp'], d['up'], d['tp'][i], d['tp'][i] + 4).max() - rest
    return [('left the water', '+%.1f s, %.0f m run, %.1f m/s' % (d['tp'][i] - t0, dist, d['speed'][i])),
            ('climb in the next 4 s', '%.1f m' % peak),
            ('PASS', d['tp'][i] - t0 < 15 and dist < 100)]


def scenario_autotakeoff(gui):
    """PX4 closed-loop auto takeoff from the water (runway takeoff: heading hold with the ruddervators, rotation at
    the rotate airspeed). Open loop the hull water-loops at planing speed, as a real seaplane does without rudder."""
    sim = Sim('autotakeoff', gui=gui)
    sim.params(**FLIGHT_PARAMS)
    time.sleep(3)
    sim.px4('commander', 'arm')
    time.sleep(1)
    sim.px4('commander', 'takeoff')
    time.sleep(30)
    sim.close()

    d = truth(sim.ulog())
    u = pyulog.ULog(sim.ulog(), ['esc_status'])
    e = [x for x in u.data_list if x.name == 'esc_status'][0].data
    te = e['timestamp'] / 1e6
    run = np.where(e['esc[0].esc_rpm'] > 900)[0]

    if not len(run):
        return [('takeoff', 'motors never spun up (arming/takeoff refused, see px4.log)'), ('PASS', False)]

    t0 = te[run[0]]
    rest = window(d['tp'], d['up'], t0 - 2, t0).mean()
    yaw0 = d['yaw'][np.searchsorted(d['ta'], t0)]
    off = np.where((d['tp'] > t0) & (d['up'] > rest + 0.30))[0]
    # heading hold on the water only: from throttle-up to liftoff (after that PX4 turns onto its loiter)
    t_off = d['tp'][off[0]] if len(off) else d['tp'][-1]
    m = (d['ta'] > t0) & (d['ta'] < t_off)
    heading_dev = np.degrees(np.abs(np.angle(np.exp(1j * (d['yaw'][m] - yaw0))))).max()
    rows = [('max heading dev. on water', '%.0f deg' % heading_dev)]

    if not len(off):
        return rows + [('left the water', 'no (max speed %.1f m/s)' % window(d['tp'], d['speed'], t0, t0 + 30).max()),
                       ('PASS', False)]

    i, i0 = off[0], np.searchsorted(d['tp'], t0)
    dist = np.hypot(d['x'][i] - d['x'][i0], d['y'][i] - d['y'][i0])
    climb = window(d['tp'], d['up'], d['tp'][i], d['tp'][-1]).max() - rest
    return rows + [('left the water', '+%.1f s, %.0f m run, %.1f m/s' % (d['tp'][i] - t0, dist, d['speed'][i])),
                   ('max height reached', '%.1f m' % climb),
                   ('PASS', d['tp'][i] - t0 < 15 and dist < 100 and heading_dev < 30)]


def scenario_mission(gui):
    """Water takeoff -> climb -> PX4 land on the lake -> flip inverted where it floats -> self-right."""
    sim = Sim('mission', gui=gui)
    sim.params(SR_EN=1, **FLIGHT_PARAMS)
    time.sleep(3)

    def listen(topic, field):
        for line in sim.px4('listener', topic).splitlines():
            if line.strip().startswith(field + ':'):
                return line.split(':', 1)[1].strip()

        return None

    def wait(cond, timeout):
        end = time.time() + timeout

        while time.time() < end and not cond():
            time.sleep(1.0)

        return cond()

    rows = []
    sim.px4('commander', 'arm')
    time.sleep(1)
    sim.px4('commander', 'takeoff')
    climbed = wait(lambda: -float(listen('vehicle_local_position_groundtruth', 'z') or 0) > 25, 60)
    rows.append(('takeoff + climb to 25 m', 'yes' if climbed else 'no'))
    time.sleep(10)

    sim.px4('commander', 'land')
    landed = wait(lambda: listen('vehicle_land_detected', 'landed') == 'True', 180)
    rows.append(('landed on the water', 'yes' if landed else 'no (timeout)'))
    wait(lambda: listen('actuator_armed', 'armed') == 'False', 20)
    time.sleep(5)

    # flip it inverted where it floats (gz world is ENU: x = east = NED y, y = north = NED x)
    n = float(listen('vehicle_local_position_groundtruth', 'x') or 0)
    e = float(listen('vehicle_local_position_groundtruth', 'y') or 0)
    subprocess.run(['gz', 'service', '-s', '/world/ahab_lake/set_pose', '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req',
                    'name: "twin_tractor_seaplane_0", position: {x: %f, y: %f, z: 0.4}, '
                    'orientation: {x: 1, y: 0, z: 0, w: 0}' % (e, n)], capture_output=True)
    time.sleep(6)
    # a teleport is invisible to the gyros: restart the estimator so it re-aligns from the accelerometer
    sim.px4('ekf2', 'stop')
    time.sleep(1)
    sim.px4('ekf2', 'start')
    wait(lambda: 'cs_tilt_align: True' in sim.px4('listener', 'estimator_status_flags'), 30)
    time.sleep(3)
    sim.self_right()
    time.sleep(10)
    sim.close()

    r = compare_attempts.metrics(sim.ulog(), 'SIM')

    if r is None:
        return rows + [('self-right', 'no Righting phase'), ('PASS', False)]

    return rows + [('self-right first motion', r['first_motion']), ('over-center', r['over_center']),
                   ('settled roll / pitch', r['settled']), ('PASS', climbed and landed)]


def scenario_selfright(gui):
    sim = Sim('selfright', pose='0,0,0.4,3.14159,0,0', gui=gui)
    sim.params(SR_EN=1, SR_OVERCTR=1.4, **FLIGHT_PARAMS)
    time.sleep(5)
    sim.self_right()
    time.sleep(8)
    sim.close()
    rows = [compare_attempts.metrics(sim.ulog(), 'SIM')]
    real = [('18_49_03', '#1'), ('18_53_54', '#2'), ('18_55_11', '#3'), ('19_17_59', '#4'), ('19_24_55', '#5')]

    for name, label in real:
        p = os.path.join(REAL_FLIPS, name + '.ulg')

        if os.path.exists(p):
            rows.append(compare_attempts.metrics(p, label))

    print(compare_attempts.table([r for r in rows if r is not None]))
    sim_row = rows[0]

    if sim_row is None:
        return [('flip', 'no Righting phase'), ('PASS', False)]

    t_oc = float(sim_row['over_center'].split('s')[0].strip('+ '))
    thr_first = float(sim_row['first_motion'].split('thr')[1]) if 'thr' in sim_row['first_motion'] else 1.0
    return [('first hull motion', sim_row['first_motion']), ('over-center', sim_row['over_center']),
            ('PASS', 0.5 <= thr_first <= 0.8 and 1.1 <= t_oc <= 1.6)]


SCENARIOS = dict(float=scenario_float, drop=scenario_drop, taxi=scenario_taxi, takeoff=scenario_takeoff,
                 autotakeoff=scenario_autotakeoff, selfright=scenario_selfright, mission=scenario_mission)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenarios', nargs='*', default=['float', 'drop', 'taxi', 'autotakeoff', 'selfright'])
    ap.add_argument('--gui', action='store_true')
    args = ap.parse_args()

    for name in args.scenarios:
        print('=== %s' % name, flush=True)

        try:
            for k, v in SCENARIOS[name](args.gui):
                print('  %-26s %s' % (k, v), flush=True)

        except Exception as e:  # keep going with the other scenarios
            print('  ERROR: %s' % e, flush=True)
            kill_sims()

    return 0


if __name__ == '__main__':
    sys.exit(main())
