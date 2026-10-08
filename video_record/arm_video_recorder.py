#!/usr/bin/env python3
"""
arm_video_recorder.py  –  Record the Epiphan video with the live Franka arm state burned in

Runs on the RECORDING PC (the ROS master).  It subscribes to /arm_state
(std_msgs/String with JSON, published by the arm PC), draws the magnet-tip pose
and the panda_link8 pose (absolute, base frame) in a box whose colour shows the
reach state, shows a live window and, while recording, writes

    <output_folder>/<YYYYmmdd_HHMM_SS>.avi   video with the overlay burned in
    <output_folder>/<YYYYmmdd_HHMM_SS>.csv   one row per video frame (same name)

Overlay colours:  MOVING yellow | REACHED green | NOT_REACHED red | IDLE white |
                  NO_DATA / NO_POSE grey (no fresh arm message / no pose)
Keys (live window focused):
    r        start / stop recording (each start = new .avi + .csv pair)
    i j k l  move the overlay up / left / down / right   (I J K L = big steps)
    c        move the overlay to the next corner
    s        save the overlay position to the config file
    q / Esc  quit

Usage (full instructions: GUIDE.md at the repository root):
    python3 arm_video_recorder.py [--config recorder_config.json] [--no-ros]
"""

import argparse
import csv
import json
import os
import threading
import time
from collections import deque
from datetime import datetime
from math import degrees

import cv2
import numpy as np

STATES_DEFAULT_BGR = {
    "MOVING": [0, 255, 255],
    "REACHED": [0, 255, 0],
    "NOT_REACHED": [0, 0, 255],
    "IDLE": [255, 255, 255],
    "NO_POSE": [160, 160, 160],
    "NO_DATA": [160, 160, 160],
}

POSE_NAMES = ["x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"]
CSV_COLUMNS = (
    ["frame_idx", "t_frame_unix", "t_rel_s", "state", "data_age_s",
     "arm_seq", "arm_stamp_unix", "source", "mode", "exec_ok",
     "waypoint", "n_waypoints", "traj_file"]
    + ["tip_" + n for n in POSE_NAMES]
    + ["link8_" + n for n in POSE_NAMES]
    + ["target_tip_" + n for n in POSE_NAMES]
    + ["err_pos_mm", "err_rot_deg"]
    + ["q%d_rad" % i for i in range(1, 8)]
    + ["ee_type", "ee_length_mm"]
)


# ---------------------------------------------------------------------------
#  Config
# ---------------------------------------------------------------------------

def load_config(path):
    path = os.path.abspath(os.path.expanduser(path))
    with open(path, 'r') as f:
        cfg = json.load(f)
    cfg["_path"] = path
    return cfg


def resolve_path(cfg, path):
    """Relative paths are relative to the config file."""
    path = os.path.expanduser(path)
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(os.path.dirname(cfg["_path"]), path))


def save_overlay_position(cfg, overlay):
    with open(cfg["_path"], 'r') as f:
        on_disk = json.load(f)
    on_disk.setdefault("overlay", {})
    on_disk["overlay"]["anchor"] = overlay.anchor
    on_disk["overlay"]["offset_px"] = [int(v) for v in overlay.offset]
    with open(cfg["_path"], 'w') as f:
        json.dump(on_disk, f, indent=4)
        f.write("\n")
    print(f"Overlay position saved: anchor={overlay.anchor} offset_px={overlay.offset}")


# ---------------------------------------------------------------------------
#  Arm state from ROS
# ---------------------------------------------------------------------------

class ArmStateCache:
    """Latest /arm_state message (parsed JSON) + local receive time."""

    def __init__(self):
        self._lock = threading.Lock()
        self._msg = None
        self._recv = None
        self._recv_times = deque(maxlen=200)

    def update(self, text):
        try:
            msg = json.loads(text)
        except ValueError:
            return
        now = time.time()
        with self._lock:
            self._msg = msg
            self._recv = now
            self._recv_times.append(now)

    def get(self):
        """-> (message dict or None, age in s or None)."""
        with self._lock:
            if self._msg is None:
                return None, None
            return self._msg, time.time() - self._recv

    def rate_hz(self, window=1.0):
        now = time.time()
        with self._lock:
            return sum(1 for t in self._recv_times if now - t <= window) / window


def start_ros(topic, cache):
    import rosgraph
    import rospy
    from std_msgs.msg import String

    if not rosgraph.is_master_online():
        raise RuntimeError("ROS master not reachable (ROS_MASTER_URI=%s). Start `roscore` first, "
                           "or run with --no-ros." % os.environ.get("ROS_MASTER_URI", "?"))
    rospy.init_node('arm_video_recorder', anonymous=True, disable_signals=True)
    rospy.Subscriber(topic, String, lambda m: cache.update(m.data), queue_size=5)
    print(f"Subscribed to {topic}")
    return rospy


# ---------------------------------------------------------------------------
#  Camera
# ---------------------------------------------------------------------------

class Camera:
    """imageio (as in record.py, e.g. '<video4>') or OpenCV (device index or video file)."""

    def __init__(self, cam_cfg):
        self.backend = cam_cfg.get("backend", "imageio")
        source = cam_cfg["source"]
        if self.backend == "imageio":
            import imageio
            self.reader = imageio.get_reader(source)
            self.iter = iter(self.reader)
        elif self.backend == "opencv":
            src = int(source) if str(source).isdigit() else source
            self.cap = cv2.VideoCapture(src)
            if not self.cap.isOpened():
                raise RuntimeError(f"Cannot open camera source {source!r}")
        else:
            raise ValueError(f"Unknown camera backend {self.backend!r} (use 'imageio' or 'opencv')")
        print(f"Camera: {self.backend} {source!r}")

    def read(self):
        """Next frame as a BGR uint8 array, or None at the end of the stream."""
        if self.backend == "imageio":
            try:
                frame = next(self.iter)
            except StopIteration:
                return None
            return cv2.cvtColor(np.asarray(frame), cv2.COLOR_RGB2BGR)
        ok, frame = self.cap.read()
        return frame if ok else None

    def close(self):
        if self.backend == "imageio":
            self.reader.close()
        else:
            self.cap.release()


# ---------------------------------------------------------------------------
#  Overlay (burned into the video)
# ---------------------------------------------------------------------------

ANCHORS = ["top_right", "top_left", "bottom_left", "bottom_right"]


def _pos_line(label, p):
    return f"{label:<6}X{p[0] * 1e3:8.1f}  Y{p[1] * 1e3:8.1f}  Z{p[2] * 1e3:8.1f} mm"


def _rot_line(p):
    return f"{'':<6}R{degrees(p[3]):8.1f}  P{degrees(p[4]):8.1f}  Y{degrees(p[5]):8.1f} deg"


class Overlay:
    def __init__(self, ov_cfg):
        self.anchor = ov_cfg.get("anchor", "top_right")
        if self.anchor not in ANCHORS:
            self.anchor = "top_right"
        self.offset = [int(v) for v in ov_cfg.get("offset_px", [20, 20])]
        self.step = int(ov_cfg.get("step_px", 5))
        self.big_step = int(ov_cfg.get("big_step_px", 40))
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = float(ov_cfg.get("font_scale", 0.6))
        self.thickness = int(ov_cfg.get("thickness", 1))
        self.bg_alpha = float(ov_cfg.get("bg_alpha", 0.5))
        self.colors = dict(STATES_DEFAULT_BGR)
        for k, v in ov_cfg.get("colors_bgr", {}).items():
            self.colors[k] = list(v)

    def color(self, state):
        return tuple(int(c) for c in self.colors.get(state, STATES_DEFAULT_BGR["NO_DATA"]))

    @staticmethod
    def text_lines(st, state):
        if st is None:
            return [f"STATE {state}", "TIP    ---", "", "LINK8  ---", "", "no fresh /arm_state message"]
        lines = []
        err = ""
        if st.get("err_pos_mm") is not None:
            err += f"   err {st['err_pos_mm']:.2f} mm"
        if st.get("err_rot_deg") is not None:
            err += f" {st['err_rot_deg']:.2f} deg"
        lines.append(f"STATE {state}{err}")
        for label, key in (("TIP", "tip"), ("LINK8", "flange")):
            p = st.get(key)
            if p:
                lines += [_pos_line(label, p), _rot_line(p)]
            else:
                lines += [f"{label:<6}---", ""]
        info = [str(st.get("mode", ""))]
        if st.get("waypoint") is not None:
            info.append(f"wp {st['waypoint']}/{st.get('n_waypoints')}")
        ee = st.get("ee") or {}
        if ee:
            info.append(f"{ee.get('type', '')} {ee.get('length_mm', 0):.0f} mm")
        lines.append(" | ".join(info))
        return lines

    def draw(self, frame, st, state):
        lines = self.text_lines(st, state)
        color = self.color(state)
        sizes = [cv2.getTextSize(l or " ", self.font, self.font_scale, self.thickness)[0] for l in lines]
        line_h = max(s[1] for s in sizes) + int(10 * self.font_scale) + 4
        pad = 8
        w = max(s[0] for s in sizes) + 2 * pad
        h = line_h * len(lines) + pad
        H, W = frame.shape[:2]
        ox, oy = self.offset
        x0 = ox if self.anchor.endswith("left") else W - ox - w
        y0 = oy if self.anchor.startswith("top") else H - oy - h
        x0 = int(min(max(x0, 0), max(W - w, 0)))
        y0 = int(min(max(y0, 0), max(H - h, 0)))

        roi = frame[y0:y0 + h, x0:x0 + w]
        roi[:] = (roi.astype(np.float32) * (1.0 - self.bg_alpha)).astype(np.uint8)
        for i, line in enumerate(lines):
            if line:
                cv2.putText(frame, line, (x0 + pad, y0 + (i + 1) * line_h), self.font,
                            self.font_scale, color, self.thickness, cv2.LINE_AA)

    def move(self, dx, dy):
        """Move in screen directions (dx > 0 = right, dy > 0 = down)."""
        self.offset[0] += -dx if self.anchor.endswith("right") else dx
        self.offset[1] += -dy if self.anchor.startswith("bottom") else dy
        self.offset = [max(0, v) for v in self.offset]

    def next_corner(self):
        self.anchor = ANCHORS[(ANCHORS.index(self.anchor) + 1) % len(ANCHORS)]


# ---------------------------------------------------------------------------
#  Recording: .avi + .csv with the same name
# ---------------------------------------------------------------------------

def _fmt(v, f="{:.6f}"):
    return "" if v is None else f.format(v)


def make_row(frame_idx, t_frame, t0, st, age, fresh):
    row = [frame_idx, f"{t_frame:.6f}", f"{t_frame - t0:.6f}"]
    if st is None or not fresh:
        row += ["NO_DATA", _fmt(age)]
        return row + [""] * (len(CSV_COLUMNS) - len(row))

    def pose(key):
        p = st.get(key)
        return [_fmt(v) for v in p] if p else [""] * 6

    joints = st.get("joints") or []
    ee = st.get("ee") or {}
    exec_ok = st.get("exec_ok")
    row += [st.get("state", ""), _fmt(age),
            st.get("seq", ""), _fmt(st.get("stamp")), st.get("source", ""), st.get("mode", ""),
            "" if exec_ok is None else int(bool(exec_ok)),
            "" if st.get("waypoint") is None else st["waypoint"],
            "" if st.get("n_waypoints") is None else st["n_waypoints"],
            st.get("traj_file") or ""]
    row += pose("tip") + pose("flange") + pose("target_tip")
    row += [_fmt(st.get("err_pos_mm"), "{:.4f}"), _fmt(st.get("err_rot_deg"), "{:.4f}")]
    row += [_fmt(q) for q in joints[:7]] + [""] * (7 - len(joints[:7]))
    row += [ee.get("type", ""), _fmt(ee.get("length_mm"), "{:.2f}")]
    return row


class Recording:
    def __init__(self, folder, frame_size, fps, fourcc="XVID"):
        os.makedirs(folder, exist_ok=True)
        stem = datetime.now().strftime("%Y%m%d_%H%M_%S")
        base = os.path.join(folder, stem)
        k = 1
        while os.path.exists(base + ".avi") or os.path.exists(base + ".csv"):
            base = os.path.join(folder, f"{stem}_{k}")
            k += 1
        self.video_path = base + ".avi"
        self.csv_path = base + ".csv"
        self.name = os.path.basename(base)
        self.frame_size = frame_size
        self.writer = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*fourcc), fps, frame_size)
        if not self.writer.isOpened():
            raise RuntimeError(f"Cannot open video writer for {self.video_path}")
        self._f = open(self.csv_path, 'w', newline='')
        self._csv = csv.writer(self._f)
        self._csv.writerow(CSV_COLUMNS)
        self.t0 = time.time()
        self.n_frames = 0
        self._last_flush = self.t0
        print(f"Recording started: {self.video_path}  +  {os.path.basename(self.csv_path)}")

    def write(self, frame, t_frame, st, age, fresh):
        if (frame.shape[1], frame.shape[0]) != self.frame_size:
            frame = cv2.resize(frame, self.frame_size)
        self.writer.write(frame)
        self._csv.writerow(make_row(self.n_frames, t_frame, self.t0, st, age, fresh))
        self.n_frames += 1
        if t_frame - self._last_flush > 1.0:
            self._f.flush()
            self._last_flush = t_frame

    def elapsed(self):
        return time.time() - self.t0

    def close(self):
        self.writer.release()
        self._f.close()
        dur = self.elapsed()
        fps = self.n_frames / dur if dur > 0 else 0.0
        print(f"Recording stopped: {self.video_path}")
        print(f"  frames {self.n_frames}, duration {dur:.1f} s, true fps {fps:.2f}, log {self.csv_path}")


# ---------------------------------------------------------------------------
#  Status line (live window only, not saved)
# ---------------------------------------------------------------------------

def draw_status(img, rec, arm_rate, fresh, has_ros):
    H, W = img.shape[:2]
    bar_h = 30
    img[H - bar_h:H, :] = (img[H - bar_h:H, :].astype(np.float32) * 0.3).astype(np.uint8)
    y = H - 9
    font = cv2.FONT_HERSHEY_SIMPLEX
    if rec is not None:
        cv2.circle(img, (16, H - bar_h // 2), 7, (0, 0, 255), -1, cv2.LINE_AA)
        t = int(rec.elapsed())
        text = f"REC  {rec.name}   {t // 60:02d}:{t % 60:02d}   {rec.n_frames} frames   [r] stop"
        cv2.putText(img, text, (30, y), font, 0.55, (0, 0, 255), 1, cv2.LINE_AA)
    else:
        cv2.circle(img, (16, H - bar_h // 2), 7, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(img, "NOT RECORDING   [r] record  [ijkl] move  [c] corner  [s] save pos  [q] quit",
                    (30, y), font, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
    if not has_ros:
        arm_text, arm_color = "ARM: ROS off", (160, 160, 160)
    elif fresh:
        arm_text, arm_color = f"ARM: {arm_rate:.0f} Hz", (0, 255, 0)
    else:
        arm_text, arm_color = "ARM: NO DATA", (0, 0, 255)
    (tw, _), _ = cv2.getTextSize(arm_text, font, 0.55, 1)
    cv2.putText(img, arm_text, (W - tw - 10, y), font, 0.55, arm_color, 1, cv2.LINE_AA)


# ---------------------------------------------------------------------------
#  Main loop
# ---------------------------------------------------------------------------

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        description="Record the Epiphan video with the Franka arm state overlay + per-frame CSV log. See GUIDE.md.")
    parser.add_argument('--config', default=os.path.join(here, 'recorder_config.json'),
                        help='recorder JSON config')
    parser.add_argument('--no-ros', action='store_true',
                        help='run without ROS (camera + overlay test, state = NO_DATA)')
    args = parser.parse_args()

    cfg = load_config(args.config)
    ros_cfg = cfg.get("ros", {})
    stale_s = float(ros_cfg.get("stale_s", 0.5))
    fps = float(cfg.get("fps", 20))
    folder = resolve_path(cfg, cfg.get("output_folder", "../../data_fluoro/"))
    fourcc = cfg.get("fourcc", "XVID")
    win = cfg.get("window_name", "Arm video recorder")

    cache = ArmStateCache()
    rospy = None
    if not args.no_ros:
        rospy = start_ros(ros_cfg.get("topic", "/arm_state"), cache)

    cam = Camera(cfg["camera"])
    overlay = Overlay(cfg.get("overlay", {}))
    rec = None
    print(f"Output folder: {folder}")
    print("Keys: r record/stop | i j k l move overlay (I J K L big) | c corner | s save position | q quit")

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while True:
            loop_start = time.time()
            frame = cam.read()
            if frame is None:
                print("Camera stream ended.")
                break
            t_frame = time.time()

            st, age = cache.get()
            fresh = st is not None and age <= stale_s
            state = st.get("state", "NO_DATA") if fresh else "NO_DATA"
            overlay.draw(frame, st if fresh else None, state)

            if rec is not None:
                rec.write(frame, t_frame, st, age, fresh)

            display = frame.copy()
            draw_status(display, rec, cache.rate_hz(), fresh, rospy is not None)
            cv2.imshow(win, display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('r'):
                if rec is None:
                    rec = Recording(folder, (frame.shape[1], frame.shape[0]), fps, fourcc)
                else:
                    rec.close()
                    rec = None
            elif key in (ord('i'), ord('k'), ord('j'), ord('l')):
                d = overlay.step
                overlay.move({ord('j'): -d, ord('l'): d}.get(key, 0), {ord('i'): -d, ord('k'): d}.get(key, 0))
            elif key in (ord('I'), ord('K'), ord('J'), ord('L')):
                d = overlay.big_step
                overlay.move({ord('J'): -d, ord('L'): d}.get(key, 0), {ord('I'): -d, ord('K'): d}.get(key, 0))
            elif key == ord('c'):
                overlay.next_corner()
            elif key == ord('s'):
                save_overlay_position(cfg, overlay)

            delay = 1.0 / fps - (time.time() - loop_start)
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        print("\nCtrl+C - stopping ...")
    finally:
        if rec is not None:
            rec.close()
        cam.close()
        cv2.destroyAllWindows()
        if rospy is not None:
            rospy.signal_shutdown("recorder closed")


if __name__ == '__main__':
    main()
