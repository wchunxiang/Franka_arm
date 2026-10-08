#!/usr/bin/env python3
"""
fake_arm_publisher.py  –  Publish a synthetic /arm_state stream (no robot needed)

Use it on the recording PC to test arm_video_recorder.py: the fake magnet tip
moves between random targets and cycles through MOVING -> REACHED and
MOVING -> NOT_REACHED, so all overlay colours can be checked.

Usage (roscore must be running):
    python3 fake_arm_publisher.py [--topic /arm_state] [--rate 30] [--length_mm 140]
"""

import argparse
import json
import time
from math import radians

import numpy as np
import rospy
from std_msgs.msg import String


def rxyz(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


def flange_of(tip, length):
    p = np.array(tip[:3]) - rxyz(*tip[3:]) @ np.array([0.0, 0.0, length])
    return [float(v) for v in p] + [float(v) for v in tip[3:]]


def main():
    parser = argparse.ArgumentParser(description="Synthetic /arm_state publisher for testing the recorder")
    parser.add_argument('--topic', default='/arm_state')
    parser.add_argument('--rate', type=float, default=30.0)
    parser.add_argument('--length_mm', type=float, default=140.0)
    parser.add_argument('--move_s', type=float, default=2.0, help='duration of each move')
    parser.add_argument('--hold_s', type=float, default=2.0, help='hold time after each move')
    args = parser.parse_args()

    rospy.init_node('fake_arm_publisher', anonymous=True)
    pub = rospy.Publisher(args.topic, String, queue_size=10)
    rate = rospy.Rate(args.rate)
    length = args.length_mm * 1e-3
    rng = np.random.default_rng(0)

    home = np.array([0.543, 0.24, 0.31, radians(-90), 0.0, 0.0])
    start = home.copy()
    target = home.copy()
    t_move = time.time() + args.hold_s       # IDLE first
    seq = 0
    n_move = 0
    print(f"Publishing fake arm state on {args.topic} at {args.rate:.0f} Hz (Ctrl+C to stop)")

    while not rospy.is_shutdown():
        now = time.time()
        if now >= t_move + args.move_s + args.hold_s:      # start a new move
            start = target.copy()
            target = home + np.r_[rng.uniform(-0.02, 0.02, 3), rng.uniform(-0.2, 0.2, 3)]
            t_move = now
            n_move += 1

        if n_move == 0:
            state, tip, exec_ok, tgt = "IDLE", home, None, None
        elif now < t_move + args.move_s:
            a = (now - t_move) / args.move_s
            state, tip, exec_ok, tgt = "MOVING", start + a * (target - start), None, target
        else:
            fail = (n_move % 3 == 0)                         # every 3rd move ends 5 mm off
            tip = target + (np.r_[0.005, 0, 0, 0, 0, 0] if fail else 0.0)
            state, exec_ok, tgt = ("NOT_REACHED" if fail else "REACHED"), True, target

        err_pos = None if tgt is None else float(np.linalg.norm(tip[:3] - tgt[:3]) * 1e3)
        err_rot = None if tgt is None else float(np.degrees(np.max(np.abs(tip[3:] - tgt[3:]))))
        seq += 1
        msg = {
            "seq": seq, "stamp": now, "source": "fake_arm_publisher",
            "mode": "manual" if n_move % 2 else "auto", "state": state, "exec_ok": exec_ok,
            "tip": [float(v) for v in tip], "flange": flange_of(tip, length),
            "target_tip": None if tgt is None else [float(v) for v in tgt],
            "target_flange": None if tgt is None else flange_of(tgt, length),
            "target_joints": None, "err_pos_mm": err_pos, "err_rot_deg": err_rot,
            "joints": [0.0, 0.3, 0.0, -2.0, 1.5, 1.5, -0.8],
            "waypoint": n_move, "n_waypoints": 100, "traj_file": "fake",
            "ee": {"type": "straight", "length_mm": args.length_mm},
        }
        pub.publish(String(data=json.dumps(msg)))
        rate.sleep()


if __name__ == '__main__':
    main()
