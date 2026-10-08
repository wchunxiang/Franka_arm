"""
arm_state_publisher.py  –  Stream the live arm / magnet-tip state to the recording PC

Publishes std_msgs/String on config["state_publisher"]["topic"] (default
/arm_state) at "rate_hz".  The string is one JSON object (see GUIDE.md):

  seq, stamp (arm-PC unix time), source, mode, state, exec_ok,
  tip[6], flange[6]            actual pose, base frame, m / rad (rxyz)
  target_tip[6], target_flange[6], target_joints[7]   or null
  err_pos_mm, err_rot_deg      actual vs target (null if unknown)
  joints[7], waypoint, n_waypoints, traj_file,
  ee {type, length_mm}

state:
  IDLE         no target commanded yet
  MOVING       a command is being planned / executed
  REACHED      motion finished and error <= reach_tol_mm / reach_tol_deg
  NOT_REACHED  motion failed or error above tolerance
  NO_POSE      the flange pose is not available from tf
The state is re-evaluated at every publish, so an arm that drifts or is pushed
away from its target turns NOT_REACHED.

The actual flange pose comes from tf (base_frame -> flange_frame), which
robot_state_publisher provides from the franka joint states.  This runs in a
rospy.Timer thread, independently of the (blocking) MoveIt calls.
"""

import json
import threading
import time

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from utils.kinematics import (link_back_straight, link_forward_straight,
                              euler_to_quat, quat_to_euler, rot_err_deg)

IDLE = 'IDLE'
MOVING = 'MOVING'
REACHED = 'REACHED'
NOT_REACHED = 'NOT_REACHED'
NO_POSE = 'NO_POSE'


def _floats(values):
    return None if values is None else [float(v) for v in values]


class ArmStatePublisher:
    """Publishes the arm state; the controllers call set_target / motion_done."""

    def __init__(self, cfg, source):
        sp_cfg = cfg["state_publisher"]
        robot_cfg = cfg["robot"]
        ee_cfg = cfg["end_effector"]

        self.source = source
        self.length = ee_cfg["length_mm"] * 1e-3
        self.ee_info = {"type": ee_cfg.get("type", "straight"),
                        "length_mm": float(ee_cfg["length_mm"])}
        self.tol_pos_mm = float(sp_cfg.get("reach_tol_mm", 1.0))
        self.tol_rot_deg = float(sp_cfg.get("reach_tol_deg", 1.0))
        self.base_frame = robot_cfg.get("base_frame", "panda_link0")
        self.flange_frame = robot_cfg.get("flange_frame", "panda_link8")

        self._lock = threading.Lock()
        self._seq = 0
        self._mode = 'init'
        self._moving = False
        self._exec_ok = None
        self._target_tip = None
        self._target_joints = None
        self._waypoint = None
        self._n_waypoints = None
        self._traj_file = None
        self._joints = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._pub = rospy.Publisher(sp_cfg.get("topic", "/arm_state"), String, queue_size=10)
        self._joint_sub = rospy.Subscriber(robot_cfg.get("joint_states_topic", "/joint_states"),
                                           JointState, self._joint_cb, queue_size=10)
        rate_hz = float(sp_cfg.get("rate_hz", 30))
        self._timer = rospy.Timer(rospy.Duration(1.0 / rate_hz), self._publish)
        print(f"[arm_state] publishing on {sp_cfg.get('topic', '/arm_state')} at {rate_hz:.0f} Hz "
              f"(tol {self.tol_pos_mm} mm / {self.tol_rot_deg} deg)")

    # ------------------------------------------------------------------
    # API used by the controllers
    # ------------------------------------------------------------------

    def set_mode(self, mode):
        with self._lock:
            self._mode = mode

    def set_waypoint(self, index, n_waypoints=None, traj_file=None):
        with self._lock:
            self._waypoint = index
            self._n_waypoints = n_waypoints
            self._traj_file = traj_file

    def set_target(self, tip=None, joints=None):
        """A new motion is starting.  Give the tip target [x,y,z,r,p,y] or a joint target."""
        with self._lock:
            self._target_tip = _floats(tip)
            self._target_joints = _floats(joints)
            self._moving = True
            self._exec_ok = None

    def motion_done(self, exec_ok):
        """The motion command returned; exec_ok = MoveIt planning + execution succeeded."""
        with self._lock:
            self._moving = False
            self._exec_ok = bool(exec_ok)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _joint_cb(self, msg):
        arm = sorted((n, p) for n, p in zip(msg.name, msg.position) if 'finger' not in n)
        if len(arm) >= 7:
            joints = [float(p) for _, p in arm[:7]]
            with self._lock:
                self._joints = joints

    def _lookup_flange(self):
        """Actual flange pose -> ([x, y, z, roll, pitch, yaw], quaternion) or (None, None)."""
        try:
            t = self._tf_buffer.lookup_transform(self.base_frame, self.flange_frame,
                                                 rospy.Time(0), rospy.Duration(0.02))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None, None
        tr = t.transform.translation
        rq = t.transform.rotation
        q = [rq.x, rq.y, rq.z, rq.w]
        roll, pitch, yaw = quat_to_euler(q)
        return [tr.x, tr.y, tr.z, roll, pitch, yaw], q

    def _publish(self, _event=None):
        flange, q_act = self._lookup_flange()
        tip = None if flange is None else link_forward_straight(*flange, self.length)

        with self._lock:
            target_tip = self._target_tip
            target_joints = self._target_joints
            joints = self._joints
            moving = self._moving
            exec_ok = self._exec_ok
            self._seq += 1
            seq = self._seq
            mode = self._mode
            waypoint, n_waypoints, traj_file = self._waypoint, self._n_waypoints, self._traj_file

        # --- errors to the target ---
        err_pos_mm = err_rot_deg = None
        target_flange = None
        if target_tip is not None:
            target_flange = link_back_straight(*target_tip, self.length)
            if tip is not None:
                err_pos_mm = 1e3 * float(np.linalg.norm(np.array(tip[:3]) - np.array(target_tip[:3])))
                err_rot_deg = rot_err_deg(q_act, euler_to_quat(*target_tip[3:]))
        elif target_joints is not None and joints is not None:
            # joint-space move: largest joint error, reported as the rotation error
            err_rot_deg = float(np.degrees(np.max(np.abs(np.array(joints) - np.array(target_joints)))))
            err_pos_mm = None

        # --- 3-state reach logic ---
        if moving:
            state = MOVING
        elif target_tip is None and target_joints is None:
            state = IDLE if flange is not None else NO_POSE
        elif not exec_ok:
            state = NOT_REACHED
        elif target_tip is not None:
            if tip is None:
                state = NO_POSE
            else:
                ok = err_pos_mm <= self.tol_pos_mm and err_rot_deg <= self.tol_rot_deg
                state = REACHED if ok else NOT_REACHED
        else:  # joint target
            if err_rot_deg is None:
                state = NO_POSE
            else:
                state = REACHED if err_rot_deg <= self.tol_rot_deg else NOT_REACHED

        payload = {
            "seq": seq,
            "stamp": time.time(),
            "source": self.source,
            "mode": mode,
            "state": state,
            "exec_ok": exec_ok,
            "tip": _floats(tip),
            "flange": _floats(flange),
            "target_tip": target_tip,
            "target_flange": _floats(target_flange),
            "target_joints": target_joints,
            "err_pos_mm": err_pos_mm,
            "err_rot_deg": err_rot_deg,
            "joints": joints,
            "waypoint": waypoint,
            "n_waypoints": n_waypoints,
            "traj_file": traj_file,
            "ee": self.ee_info,
        }
        self._pub.publish(String(data=json.dumps(payload)))
