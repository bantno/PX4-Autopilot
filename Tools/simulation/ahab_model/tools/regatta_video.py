#!/usr/bin/env python3
"""AHAB onboard-camera videos in the ahab_regatta world (Sydney regatta course). Two scenarios:

  takeoff    spawn on the west end of the rowing course facing east, upload a mission (water takeoff along the
             course, cruise down it at --alt, the required landing at the far end), switch to Mission, arm, and record
             until --cruise seconds of sim time after reaching cruise height
  selfright  spawn inverted at rest at the same spot, SELF_RIGHT + arm over MAVLink (as selfright_video.py does in
             ahab_lake, SR_OVERCTR 1.4), record until the module has disarmed and the hull has settled

Writes, clocked in sim time (real-time playback):
  - <out>.webm          the forward / wing / down cameras side by side, with a status line
  - <out>_forward.webm  the forward camera at full resolution

    python3 Tools/simulation/ahab_model/tools/regatta_video.py {takeoff,selfright} [--fps 30] [--rtf 0.15] [--out F]

Needs `make px4_sitl_default` (incl. the MaritimeWaveVisual plugin). As in selfright_video.py the model is rebuilt
with the cameras at --fps for the recording and the previous model.sdf is put back afterwards. Kills any running
px4 / gz sim first.
"""
import argparse
import math
import os
import subprocess
import sys
import threading
import time

os.environ.setdefault('GZ_IP', '127.0.0.1')   # gz-transport on loopback: a Wi-Fi drop otherwise stalls PX4 <-> gz
os.environ['AHAB_GZ_WORLD'] = 'ahab_regatta'

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.transport13 import Node  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_SDF = os.path.join(HERE, '..', 'twin_tractor_seaplane', 'model.sdf')
sys.path.insert(0, HERE)
import water_scenarios as ws  # noqa: E402

WORLD = 'ahab_regatta'
SPAWN = (-850.0, -15.0, 0.3, -0.04)   # ENU x, y, z, yaw: west end of the course, facing down it (east)
SR_STATES = ['idle', 'verify', 'rotate wing', 'righting', 'cut', 'disarm']   # SelfRightStatus STATE_*
# The gz-sim magnetometer field at this site does not match PX4's world magnetic model (its declination comes out
# mirrored, see the GZBridge magnetometer note; -26 deg measured from sensor_mag and the ground-truth attitude vs
# the model's +12.6). Given to PX4 as a fixed declination at boot (PX4_PARAM_*), the at-rest heading estimate is
# still ~29 deg off (39 without), but it converges to the true heading within seconds of the takeoff run (GNSS
# velocity) and the run itself held the true heading in testing; the mission geometry uses the ground truth.
MAG_DECL_DEG = -26.0
H = 360
BAR = 30
PANELS = [('forward', '/world/%s/model/twin_tractor_seaplane_0/link/base_link/sensor/camera_forward/image' % WORLD,
           640),
          ('wing (left, looking inboard)', '/ahab/camera_wing/image', 640),
          ('down (survey)', '/ahab/camera_down/image', 480)]
EARTH_R = 6371000.0


class Recorder:
    """Latest frame of each camera; one tiled frame (and one full-size forward frame) per forward-camera frame."""

    def __init__(self, path, fps):
        self.w = sum(p[2] for p in PANELS)
        fourcc = cv2.VideoWriter_fourcc(*'VP80')
        self.tiled = cv2.VideoWriter(path, fourcc, fps, (self.w, H + 2 * BAR))
        self.forward_path = os.path.splitext(path)[0] + '_forward.webm'
        self.forward = None
        self.fourcc, self.fps = fourcc, fps

        if not self.tiled.isOpened():
            raise RuntimeError('OpenCV cannot write VP8 WebM to %s' % path)

        self.latest = [np.zeros((H, w, 3), np.uint8) for _, _, w in PANELS]
        self.lock = threading.Lock()
        self.history = [(0.0, 'on the water')]   # (sim time, status text), appended by the scenario
        self.t0 = None
        self.next_t = None   # sim time of the next video frame
        self.last_stamp = 0.0
        self.frames = 0
        self.closed = False

    def status(self, t):
        """Scenario status at sim time t (the frames lag the physics while rendering catches up)."""
        h = self.history
        return h[next((k for k in range(len(h) - 1, -1, -1) if h[k][0] <= t), 0)][1]

    def add(self, t, text):
        with self.lock:
            self.history.append((t, text))

    def callback(self, i):
        def f(m):
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
            full = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
            small = cv2.resize(full, (PANELS[i][2], H), interpolation=cv2.INTER_AREA)

            with self.lock:
                self.latest[i] = small

                if i == 0 and not self.closed:
                    t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
                    self.last_stamp = t

                    if self.next_t is None:
                        self.next_t = t

                    # one video frame per 1/fps of sim time: real-time playback even if a render was skipped
                    while self.next_t <= t + 1e-6:
                        self.write(self.next_t, full)
                        self.next_t += 1.0 / self.fps

        return f

    def write(self, t, forward_full):
        if self.t0 is None:
            self.t0 = t

        status = 't=%5.1f s   %s' % (t - self.t0, self.status(t))
        frame = np.zeros((H + 2 * BAR, self.w, 3), np.uint8)
        x = 0

        for (name, _, w), img in zip(PANELS, self.latest):
            frame[BAR:BAR + H, x:x + w] = img
            cv2.putText(frame, name, (x + 8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            x += w
            cv2.line(frame, (x - 1, 0), (x - 1, BAR + H), (80, 80, 80), 1)

        cv2.putText(frame, status, (8, H + BAR + 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 1, cv2.LINE_AA)
        self.tiled.write(frame)

        if self.forward is None:
            h, w = forward_full.shape[:2]
            self.forward = cv2.VideoWriter(self.forward_path, self.fourcc, self.fps, (w, h))

        fw = forward_full.copy()
        cv2.putText(fw, status, (12, fw.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2, cv2.LINE_AA)
        self.forward.write(fw)
        self.frames += 1

    def close(self):
        with self.lock:
            self.closed = True
            self.tiled.release()

            if self.forward is not None:
                self.forward.release()


def offset(lat, lon, north, east):
    return (lat + math.degrees(north / EARTH_R),
            lon + math.degrees(east / (EARTH_R * math.cos(math.radians(lat)))))


def course_mission(lat, lon, heading, alt, takeoff_alt):
    """(command, lat, lon, alt, p1, p2) items along heading: takeoff 400 m ahead, cruise down the course, then the
    landing the airframe requires (MIS_TKO_LAND_REQ): DO_LAND_START, loiter down 800 m short of the east end, land
    there at a ~2 deg glide (under FW_LND_ANG 3). The recording stops during cruise, before the landing."""
    ch, sh = math.cos(heading), math.sin(heading)
    at = lambda d: offset(lat, lon, d * ch, d * sh)  # noqa: E731
    return [(22, *at(400.0), takeoff_alt, 0, 0),   # NAV_TAKEOFF
            (16, *at(1000.0), alt, 0, 0),           # NAV_WAYPOINT
            (16, *at(1500.0), alt, 0, 0),
            (189, 0.0, 0.0, 0.0, 0, 0),             # DO_LAND_START
            (31, *at(1150.0), 25.0, 1, 80.0),       # NAV_LOITER_TO_ALT (heading required, 80 m radius)
            (21, *at(1950.0), 0.0, 0, 0)]           # NAV_LAND


def set_rtf(rtf):
    r = subprocess.run(['gz', 'service', '-s', '/world/%s/set_physics' % WORLD, '--reqtype', 'gz.msgs.Physics',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', 'real_time_factor: %f' % rtf],
                       capture_output=True, text=True)
    return 'true' in r.stdout


def mavlink():
    from pymavlink import mavutil
    m = mavutil.mavlink_connection('udpin:0.0.0.0:14540', source_system=255, source_component=190)
    m.wait_heartbeat(timeout=10)
    return m


def upload(m, items):
    m.mav.mission_count_send(m.target_system, m.target_component, len(items), 0)
    end = time.time() + 20

    while time.time() < end:
        msg = m.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'], blocking=True, timeout=2)

        if msg is None:
            continue

        if msg.get_type() == 'MISSION_ACK':
            return msg.type == 0

        cmd, lat, lon, alt, p1, p2 = items[msg.seq]
        frame = 2 if cmd >= 176 else 6   # PX4 takes DO_* items only in MAV_FRAME_MISSION
        m.mav.mission_item_int_send(m.target_system, m.target_component, msg.seq, frame, cmd, 0, 1, p1, p2, 0,
                                    float('nan'), int(lat * 1e7), int(lon * 1e7), alt, 0)

    return False


def command(m, cmd, *p):
    p = list(p) + [0] * (7 - len(p))
    m.mav.command_long_send(m.target_system, m.target_component, cmd, 0, *p)
    ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    return ack is not None and ack.result == 0


def field(text, name):
    """Scalar from `listener` output (drops the '(x seconds ago)' suffix of timestamps)."""
    for line in text.splitlines():
        if line.strip().startswith(name + ':'):
            return float(line.split(':', 1)[1].split('(')[0])

    return float('nan')


def vec(text, name):
    for line in text.splitlines():
        if line.strip().startswith(name + ':'):
            inner = line.split('[', 1)[1].split(']', 1)[0]   # 'q: [w, x, y, z] (Roll: ...)'
            return [float(v) for v in inner.split(',')]

    return None


def check_heading(sim, note):
    """True heading from ground truth, and the estimator's heading for comparison (see MAG_DECL_DEG)."""
    w, x, y, z = vec(sim.px4('listener', 'vehicle_attitude_groundtruth', '-n', '1'), 'q')
    true_hdg = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    ekf_hdg = math.degrees(field(sim.px4('listener', 'vehicle_local_position', '-n', '1'), 'heading'))
    err = (ekf_hdg - true_hdg + 180.0) % 360.0 - 180.0
    note('heading true %.1f / estimated %.1f deg%s' % (true_hdg, ekf_hdg,
                                                      '  (sim mag mismatch, see MAG_DECL_DEG)' if abs(err) > 5 else ''))
    return math.radians(true_hdg)


def run_takeoff(sim, rec, args, note):
    heading = check_heading(sim, note)
    gpos = sim.px4('listener', 'vehicle_global_position', '-n', '1')
    lat, lon = field(gpos, 'lat'), field(gpos, 'lon')
    m = mavlink()

    if not upload(m, course_mission(lat, lon, heading, args.alt, 30.0)):
        raise RuntimeError('mission upload rejected')

    note('mission uploaded (heading %.1f deg)' % math.degrees(heading))
    sim.wait_for(lambda: 'valid: True' in sim.px4('listener', 'mission_result', '-n', '1'), 20,
                 'a feasible mission (navigator rejected it, see px4.log)')

    if not set_rtf(args.rtf):
        note('could not set the real-time factor')

    time.sleep(4 / args.rtf)   # a few seconds of floating at the start of the video
    command(m, 176, 1, 4, 4)   # DO_SET_MODE: custom AUTO / MISSION
    phase = 'takeoff run'

    if not command(m, 400, 1):   # arm
        raise RuntimeError('arming refused (see px4.log)')

    note('armed in Mission')
    cruise_since = None
    end = time.time() + 900 / args.rtf

    while time.time() < end:
        gt = sim.px4('listener', 'vehicle_local_position_groundtruth', '-n', '1')
        h, vx, vy = -field(gt, 'z'), field(gt, 'vx'), field(gt, 'vy')
        ts = field(gt, 'timestamp') * 1e-6

        if not all(math.isfinite(v) for v in (h, vx, vy, ts)):
            time.sleep(0.2)
            continue

        if phase == 'takeoff run' and h > 1.0:
            phase = 'climb'
            note('left the water')
        elif phase == 'climb' and h > args.alt - 3.0:
            phase = 'cruise'
            cruise_since = ts
            note('at cruise height')

        rec.add(ts, '%s   height %5.1f m   ground speed %4.1f m/s' % (phase, h, math.hypot(vx, vy)))

        if phase == 'cruise' and ts - cruise_since > args.cruise:
            note('%.0f s of cruise flown; waiting for the camera frames to catch up' % args.cruise)
            sim.wait_for(lambda: rec.last_stamp >= ts, 600, 'camera frames to reach the end of cruise')
            break

        time.sleep(0.2)

    m.close()


def run_selfright(sim, rec, args, note):
    sim.params(SR_EN=1, SR_OVERCTR=1.4)

    if not set_rtf(args.rtf):
        note('could not set the real-time factor')

    rec.history[0] = (0.0, 'capsized (inverted at rest)')
    time.sleep(4 / args.rtf)   # a few seconds of floating upside down
    sim.self_right()           # DO_SET_MODE custom 12 (SELF_RIGHT), then arm
    note('SELF_RIGHT + arm sent')
    seen_active, done_at = False, None
    end = time.time() + 120 / args.rtf

    while time.time() < end:
        sr = sim.px4('listener', 'self_right_status', '-n', '1')
        ts = field(sim.px4('listener', 'vehicle_local_position_groundtruth', '-n', '1'), 'timestamp') * 1e-6
        active = 'active: True' in sr
        state = int(field(sr, 'state')) if math.isfinite(field(sr, 'state')) else 0
        tilt = math.degrees(field(sr, 'pitch_from_upright'))
        thr = field(sr, 'throttle')

        if not math.isfinite(ts):
            time.sleep(0.2)
            continue

        seen_active |= active

        if active:
            text = 'SELF_RIGHT: %s   %3.0f deg from upright   throttle %.2f' % (SR_STATES[min(state, 5)], tilt, thr)
        elif seen_active:
            text = 'self-right done (disarmed)   %3.0f deg from upright' % tilt
            done_at = done_at or ts
        else:
            text = 'capsized (inverted at rest)   %3.0f deg from upright' % tilt

        rec.add(ts, text)

        if done_at is not None and ts - done_at > args.settle:
            note('done: %.0f deg from upright; waiting for the camera frames to catch up' % tilt)
            sim.wait_for(lambda: rec.last_stamp >= ts, 600, 'camera frames to reach the end')
            break

        time.sleep(0.2)

    if not seen_active:
        note('SELF_RIGHT never became active (see px4.log)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenario', choices=['takeoff', 'selfright'])
    ap.add_argument('--fps', type=int, default=30, help='camera rate while recording = video frame rate')
    ap.add_argument('--cruise', type=float, default=60.0, help='sim seconds to record once at cruise height')
    ap.add_argument('--alt', type=float, default=40.0, help='cruise height above the water [m]')
    ap.add_argument('--settle', type=float, default=6.0, help='selfright: sim seconds to keep recording after')
    ap.add_argument('--out', help='output .webm (default: videos/<scenario>_regatta_<time>.webm)')
    ap.add_argument('--rtf', type=float, default=0.15,
                    help='sim speed while recording: gz skips camera frames it cannot render in time')
    ap.add_argument('--gui', action='store_true', help='also open the Gazebo GUI')
    args = ap.parse_args()

    out = os.path.abspath(args.out or os.path.join(HERE, '..', 'videos',
                                                   time.strftime(args.scenario + '_regatta_%Y%m%d_%H%M%S.webm')))
    os.makedirs(os.path.dirname(out), exist_ok=True)

    with open(MODEL_SDF) as fp:
        model_sdf = fp.read()

    subprocess.run([sys.executable, os.path.join(HERE, 'build_model.py')], check=True, stdout=subprocess.DEVNULL,
                   env=dict(os.environ, AHAB_CAMERA_HZ=str(args.fps)))

    def restore():
        with open(MODEL_SDF, 'w') as fp:
            fp.write(model_sdf)

    rec = Recorder(out, args.fps)
    os.environ.update(PX4_PARAM_EKF2_DECL_TYPE='0', PX4_PARAM_EKF2_MAG_DECL=str(MAG_DECL_DEG))

    try:
        x, y, z, yaw = SPAWN
        roll = math.pi if args.scenario == 'selfright' else 0.0
        sim = ws.Sim('regatta_video', pose='%f,%f,%f,%f,0,%f' % (x, y, z + 0.1, roll, yaw), gui=args.gui)
    except BaseException:
        restore()
        raise

    node = Node()
    log = []

    def note(s):
        line = '[%s] %s' % (time.strftime('%H:%M:%S'), s)
        print(line, flush=True)
        log.append(line)

    try:
        for i, (_, topic, _) in enumerate(PANELS):
            node.subscribe(Image, topic, rec.callback(i))

        sim.params(**ws.FLIGHT_PARAMS)
        time.sleep(3)
        (run_takeoff if args.scenario == 'takeoff' else run_selfright)(sim, rec, args, note)
    finally:
        for _, topic, _ in PANELS:
            node.unsubscribe(topic)

        rec.close()
        sim.close()
        restore()

    if rec.frames == 0:
        print('no camera frames: was model.sdf built with AHAB_CAMERAS=0?')
        return 1

    print('video   %s (%d frames at %d fps, %.1f s sim time)' % (out, rec.frames, args.fps, rec.frames / args.fps))
    print('forward %s' % rec.forward_path)
    print('ulog    %s' % sim.ulog())
    return 0


if __name__ == '__main__':
    rc = main()
    sys.stdout.flush()
    os._exit(rc)  # skip gz-transport's thread teardown at interpreter exit (it aborts with "terminate called")
