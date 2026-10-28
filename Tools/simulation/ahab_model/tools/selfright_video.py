#!/usr/bin/env python3
"""Self-right demo with the AHAB camera feeds: runs the water_scenarios.py selfright sequence (inverted start,
SR_OVERCTR 1.4, SELF_RIGHT mode + arm over MAVLink) and records the forward / wing / down cameras side by side.

    python3 Tools/simulation/ahab_model/tools/selfright_video.py [--fps 30] [--out FILE.webm] [--stills] [--gui]

Needs `make px4_sitl_default`. The model's cameras run at 10 Hz by default; for the recording the model is rebuilt
with AHAB_CAMERA_HZ=--fps and the previous model.sdf is put back afterwards. Writes a VP8 WebM (plays in browsers,
VLC, mpv) in sim time, clocked by the forward camera, to Tools/simulation/ahab_model/videos/ by default, and prints
the flip metrics from the session's ULog.
"""
import argparse
import os
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_SDF = os.path.join(HERE, '..', 'twin_tractor_seaplane', 'model.sdf')
sys.path.insert(0, HERE)
import water_scenarios as ws  # noqa: E402

H = 360   # panel height, px
BAR = 30  # label / status bar height, px
PANELS = [('forward', '/world/ahab_lake/model/twin_tractor_seaplane_0/link/base_link/sensor/camera_forward/image', 640),
          ('wing (left, looking inboard)', '/ahab/camera_wing/image', 640),
          ('down (survey)', '/ahab/camera_down/image', 480)]


class Recorder:
    """Keeps the latest frame of each camera and writes one tiled frame per forward-camera frame."""

    def __init__(self, path, fps, stills_dir=None):
        self.w = sum(p[2] for p in PANELS)
        self.writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'VP80'), fps, (self.w, H + 2 * BAR))

        if not self.writer.isOpened():
            raise RuntimeError('OpenCV cannot write VP8 WebM to %s' % path)

        self.latest = [np.zeros((H, w, 3), np.uint8) for _, _, w in PANELS]
        self.lock = threading.Lock()
        self.stills_dir = stills_dir
        self.fps = fps
        self.phase = 'spawned inverted'
        self.t0 = None
        self.frames = 0
        self.closed = False

    def callback(self, i):
        def f(m):
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
            a = cv2.cvtColor(cv2.resize(a, (PANELS[i][2], H), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2BGR)

            with self.lock:
                self.latest[i] = a

                if i == 0 and not self.closed:
                    self.write(m.header.stamp.sec + m.header.stamp.nsec * 1e-9)

        return f

    def write(self, t):
        if self.t0 is None:
            self.t0 = t

        frame = np.zeros((H + 2 * BAR, self.w, 3), np.uint8)
        x = 0

        for (name, _, w), img in zip(PANELS, self.latest):
            frame[BAR:BAR + H, x:x + w] = img
            cv2.putText(frame, name, (x + 8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            x += w
            cv2.line(frame, (x - 1, 0), (x - 1, BAR + H), (80, 80, 80), 1)

        cv2.putText(frame, 't=%5.1f s  %s' % (t - self.t0, self.phase), (8, H + BAR + 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 1, cv2.LINE_AA)
        self.writer.write(frame)
        self.frames += 1

        if self.stills_dir and self.frames % self.fps == 0:
            cv2.imwrite(os.path.join(self.stills_dir, 't%03d.jpg' % (self.frames // self.fps)), frame)

    def close(self):
        with self.lock:
            self.closed = True
            self.writer.release()


def restore(model_sdf):
    # put back the model.sdf from before the recording (the 10 Hz default, or whatever was built)
    with open(MODEL_SDF, 'w') as fp:
        fp.write(model_sdf)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--fps', type=int, default=30, help='camera rate while recording = video frame rate')
    ap.add_argument('--out', help='output .webm (default: videos/selfright_cameras_<time>.webm)')
    ap.add_argument('--stills', action='store_true', help='also save one JPEG per sim second next to the video')
    ap.add_argument('--hold', type=float, default=15, help='wall seconds to record after SELF_RIGHT + arm')
    ap.add_argument('--gui', action='store_true', help='also open the Gazebo GUI')
    args = ap.parse_args()

    out = args.out or os.path.join(HERE, '..', 'videos', time.strftime('selfright_cameras_%Y%m%d_%H%M%S.webm'))
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    stills = None

    if args.stills:
        stills = os.path.splitext(out)[0] + '_stills'
        os.makedirs(stills, exist_ok=True)

    with open(MODEL_SDF) as fp:
        model_sdf = fp.read()

    subprocess.run([sys.executable, os.path.join(HERE, 'build_model.py')], check=True, stdout=subprocess.DEVNULL,
                   env=dict(os.environ, AHAB_CAMERA_HZ=str(args.fps)))
    rec = Recorder(out, args.fps, stills)

    try:
        sim = ws.Sim('selfright_video', pose='0,0,0.4,3.14159,0,0', gui=args.gui)

    except BaseException:
        restore(model_sdf)
        raise

    node = Node()

    try:
        for i, (_, topic, _) in enumerate(PANELS):
            node.subscribe(Image, topic, rec.callback(i))

        sim.params(SR_EN=1, SR_OVERCTR=1.4, **ws.FLIGHT_PARAMS)
        time.sleep(5)
        rec.phase = 'SELF_RIGHT + arm'
        sim.self_right()
        time.sleep(args.hold)
        rec.phase = 'done'
        time.sleep(2)

    finally:
        for _, topic, _ in PANELS:
            node.unsubscribe(topic)

        rec.close()
        sim.close()
        restore(model_sdf)

    if rec.frames == 0:
        print('no camera frames: was model.sdf built with AHAB_CAMERAS=0?')
        return 1

    print('video   %s (%d frames at %d fps, %.1f s sim time)' % (out, rec.frames, args.fps, rec.frames / args.fps))

    if stills:
        print('stills  %s' % stills)

    row = ws.compare_attempts.metrics(sim.ulog(), 'SIM')
    print(ws.compare_attempts.table([row]) if row else 'flip: no Righting phase in %s' % sim.ulog())
    return 0


if __name__ == '__main__':
    rc = main()
    sys.stdout.flush()
    os._exit(rc)  # skip gz-transport's thread teardown at interpreter exit (it aborts with "terminate called")
