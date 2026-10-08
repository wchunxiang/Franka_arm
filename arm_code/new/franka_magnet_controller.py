#!/usr/bin/env python3
"""
franka_magnet_controller.py  –  Merged controller for Franka Emika FR3 + magnet end-effector

This script merges the functionality of:
  - controller_CX_3D_L_Xray.py   (low-level MoveIt planning & execution)
  - arm_control_magnet_3D_joy_tracking_L_Xray.py  (joystick-based high-level control)

Key features over the original split design:
  1. Hand-guide initial pose: the user physically moves the arm to the desired
     starting position, then the script reads back the current end-effector pose
     (position + orientation) and uses it as the initial reference.
  2. All parameters are loaded from a JSON config file instead of being hard-coded.
  3. Single ROS node – no inter-process pub/sub latency.
  4. Correct quaternion ↔ Euler conversion using tf.transformations with the
     'rxyz' (rotating-frame XYZ) convention, matching the original code.
  5. Per-axis step sizes for finer control.
  6. Yaw angle is now adjustable via the joystick to help escape singularities.

Usage:
  1. Power on the FR3 and release brakes via Desk.
  2. (Optional) Hand-guide the arm to the desired starting pose.
  3. Activate FCI via Desk.
  4. Run:  rosrun <your_package> franka_magnet_controller.py --config config.json
  5. In the terminal, press Enter to capture the current pose as initial pose.
  6. Use the joystick for precise control.

Dependencies:
  - ROS Noetic, MoveIt 1.1.x, franka_ros
  - sensor_msgs, geometry_msgs, std_msgs, visualization_msgs
  - tf.transformations, numpy, cv2 (optional, for display)
  - utils.arduino_control, utils.show, utils.link_back, utils.data_record

Author: (refactored with assistance from Claude)
"""

import sys
import os
import json
import copy
import argparse
import time
import threading

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
import std_msgs.msg
from std_msgs.msg import Bool, Float64MultiArray
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PointStamped
from sensor_msgs.msg import Joy

import tf.transformations as ttf
from tf.transformations import (quaternion_from_euler, euler_from_quaternion,
                                 quaternion_matrix, euler_from_matrix)
import numpy as np
from math import radians, degrees, pi

# Optional imports – degrade gracefully if not available
try:
    import cv2 as cv
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# ---------------------------------------------------------------------------
# Utility: link_back_L  (copied from utils/link_back.py so the script is self-
# contained; the original module is still importable if preferred)
# ---------------------------------------------------------------------------

def link_back_L(end_x, end_y, end_z, end_roll, end_pitch, end_yaw,
                end_effector_height, end_effector_length):
    """Convert magnet-tip pose → panda_link8 pose.

    The L-shaped end-effector has a vertical segment (height) and a horizontal
    segment (length).  The magnet tip is offset from link8 by:
        p_local = [0, end_effector_length, end_effector_height, 1]
    expressed in the end-effector's own frame.

    NOTE on angle swap: the L-shape geometry requires swapping pitch ↔ yaw
    when building the rotation matrix.  This is intentional and was verified
    on the real robot.

    Parameters
    ----------
    end_x .. end_yaw : float
        Magnet-tip pose in the base frame (metres, radians).
    end_effector_height, end_effector_length : float
        Physical dimensions of the L-bracket (metres).

    Returns
    -------
    list of 6 floats : [link_x, link_y, link_z, link_roll, link_pitch, link_yaw]
        Pose of panda_link8 that places the magnet tip at the requested pose.
    """
    p_end = np.mat([end_x, end_y, end_z, 1.0]).T
    p_local = np.mat([0, end_effector_length, end_effector_height, 1.0]).T

    # --- angle swap for L-shape ---
    link_roll = end_roll
    link_yaw  = end_pitch   # swap
    link_pitch = end_yaw    # swap

    quaternion = ttf.quaternion_from_euler(link_roll, link_pitch, link_yaw, 'rxyz')
    Rq = np.mat(ttf.quaternion_matrix(quaternion))
    p_link = p_end - Rq @ p_local

    link_x, link_y, link_z = np.array(p_link).reshape(-1)[:3]
    return [link_x, link_y, link_z, link_roll, link_pitch, link_yaw]


def link_forward_L(link_x, link_y, link_z, link_roll, link_pitch, link_yaw,
                   end_effector_height, end_effector_length):
    """Inverse of link_back_L: panda_link8 pose → magnet-tip pose.

    Given the current link8 pose (as reported by MoveIt), compute the
    corresponding magnet-tip pose.  This is needed when reading back the
    hand-guided initial position.

    The maths:
        p_end = p_link + R @ p_local
    with the same angle-swap convention as link_back_L.

    Parameters
    ----------
    link_x .. link_yaw : float
        Pose of panda_link8 in the base frame (metres, radians).
    end_effector_height, end_effector_length : float
        Physical dimensions of the L-bracket (metres).

    Returns
    -------
    list of 6 floats : [end_x, end_y, end_z, end_roll, end_pitch, end_yaw]
        Magnet-tip pose in the base frame.
    """
    p_link = np.mat([link_x, link_y, link_z, 1.0]).T
    p_local = np.mat([0, end_effector_length, end_effector_height, 1.0]).T

    quaternion = ttf.quaternion_from_euler(link_roll, link_pitch, link_yaw, 'rxyz')
    Rq = np.mat(ttf.quaternion_matrix(quaternion))
    p_end = p_link + Rq @ p_local

    end_x, end_y, end_z = np.array(p_end).reshape(-1)[:3]

    # Reverse the angle swap: link_roll=end_roll, link_yaw=end_pitch, link_pitch=end_yaw
    end_roll  = link_roll
    end_pitch = link_yaw    # reverse swap
    end_yaw   = link_pitch  # reverse swap
    return [end_x, end_y, end_z, end_roll, end_pitch, end_yaw]


# ---------------------------------------------------------------------------
# Utility: read current link8 pose from MoveIt and convert to Euler
# ---------------------------------------------------------------------------

def get_link8_euler(group):
    """Read the current panda_link8 pose and return (x, y, z, roll, pitch, yaw).

    MoveIt's get_current_pose() returns a geometry_msgs/PoseStamped whose
    orientation is a quaternion.  We convert it to Euler angles using the
    same 'rxyz' (rotating-frame XYZ) convention that the rest of the code
    uses for quaternion_from_euler.

    tf.transformations represents quaternions as [x, y, z, w], which matches
    the field order in geometry_msgs/Quaternion (x, y, z, w).

    Returns
    -------
    tuple of 6 floats : (x, y, z, roll, pitch, yaw) in metres and radians
    """
    pose = group.get_current_pose().pose

    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    # geometry_msgs/Quaternion fields: x, y, z, w
    # tf.transformations expects [x, y, z, w]  — same order, no shuffle needed
    q = [pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z,
         pose.orientation.w]

    # Use the SAME 'rxyz' convention as quaternion_from_euler elsewhere
    roll, pitch, yaw = euler_from_quaternion(q, axes='rxyz')

    return (x, y, z, roll, pitch, yaw)


def verify_euler_roundtrip(roll, pitch, yaw, label=""):
    """Debug helper: convert Euler → quaternion → Euler and print both.

    Call this once at startup to verify the conversion is self-consistent.
    """
    q = quaternion_from_euler(roll, pitch, yaw, 'rxyz')
    r2, p2, y2 = euler_from_quaternion(q, axes='rxyz')
    print("[%s] Original  RPY: (%.4f, %.4f, %.4f) rad  = (%.1f, %.1f, %.1f) deg",
                  label, roll, pitch, yaw, degrees(roll), degrees(pitch), degrees(yaw))
    print("[%s] Roundtrip RPY: (%.4f, %.4f, %.4f) rad  = (%.1f, %.1f, %.1f) deg",
                  label, r2, p2, y2, degrees(r2), degrees(p2), degrees(y2))
    err = max(abs(roll - r2), abs(pitch - p2), abs(yaw - y2))
    if err > 1e-6:
        rospy.logwarn("[%s] Euler roundtrip error = %.6e  — CHECK CONVENTION!", label, err)
    else:
        print("[%s] Euler roundtrip OK (err=%.2e)", label, err)


# ---------------------------------------------------------------------------
# Configuration loader
# ---------------------------------------------------------------------------

def load_config(path):
    """Load and validate the JSON configuration file.

    Returns a plain dict.  We do NOT convert units here; the caller is
    responsible for mm→m conversions where needed.
    """
    with open(path, 'r') as f:
        cfg = json.load(f)
    print("Loaded config from %s", path)
    return cfg


# ===========================================================================
#  FrankaMagnetController – the merged, single-node controller
# ===========================================================================

class FrankaMagnetController:
    """Unified controller: MoveIt planning + joystick input + motor control."""

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def __init__(self, cfg):
        """
        Parameters
        ----------
        cfg : dict
            Parsed JSON config (see load_config).
        """
        self.cfg = cfg

        # ---- extract frequently-used config values ----
        robot_cfg = cfg["robot"]
        ee_cfg    = cfg["end_effector"]
        step_cfg  = cfg["step_sizes"]
        motor_cfg = cfg["motor"]
        traj_cfg  = cfg["trajectory"]
        disp_cfg  = cfg["display"]

        magnet_key = ee_cfg["magnet_key"]
        mag_set    = ee_cfg["magnet_settings"][magnet_key]

        self.end_effector_height = ee_cfg["end_effector_height_mm"] * 1e-3        # → metres
        self.end_effector_length = mag_set["end_effector_length_mm"] * 1e-3        # → metres
        self.vel_scale = mag_set["vel"]
        self.acc_scale = mag_set["acc"]
        self.arm_acc   = robot_cfg["arm_acc"]

        # Per-axis step sizes
        self.step = {
            'x_small': step_cfg["x"]["small_m"],   'x_big': step_cfg["x"]["big_m"],
            'y_small': step_cfg["y"]["small_m"],    'y_big': step_cfg["y"]["big_m"],
            'z_small': step_cfg["z"]["small_m"],    'z_big': step_cfg["z"]["big_m"],
            'roll_small':  step_cfg["roll"]["small_rad"],  'roll_big':  step_cfg["roll"]["big_rad"],
            'pitch_small': step_cfg["pitch"]["small_rad"], 'pitch_big': step_cfg["pitch"]["big_rad"],
            'yaw_small':   step_cfg["yaw"]["small_rad"],   'yaw_big':   step_cfg["yaw"]["big_rad"],
        }

        # Singularity thresholds & replan counts
        self.thresh_singular = robot_cfg["singularity_thresholds"]
        self.planning_repeat = robot_cfg["planning_repeat"]

        # Flags
        self.b_motor_enable  = motor_cfg["enable"]
        self.b_show_text     = disp_cfg["show_text"] and HAS_CV2
        self.b_debug         = cfg.get("debug_mode", False)
        self.b_constraint    = cfg.get("use_path_constraints", False)
        self.b_first_move    = True   # first planning call after init

        # ---- ROS / MoveIt initialisation ----
        rospy.init_node('franka_magnet_controller', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(robot_cfg["group_name"])

        self.group.set_max_velocity_scaling_factor(self.vel_scale)
        self.group.set_max_acceleration_scaling_factor(self.acc_scale)

        # Print basic info
        print("Planning frame : %s", self.group.get_planning_frame())
        print("End effector   : %s", self.group.get_end_effector_link())
        print("Robot groups   : %s", self.robot.get_group_names())

        # Rviz trajectory display (optional, for debugging)
        self.display_traj_pub = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # ---- Visualisation markers (optional) ----
        self.marker_pub = rospy.Publisher(
            '/visualization_marker',
            visualization_msgs.msg.Marker, queue_size=20)
        rospy.sleep(0.5)
        self._remove_all_markers()
        self.marker_id = 0

        # ---- Motor (stepper / servo) ----
        if self.b_motor_enable:
            from utils.arduino_control import Stepper_servo
            self.stepper = Stepper_servo(
                serial_no=motor_cfg["serial_no"],
                servo_vel_step=motor_cfg["step_rotation"])

        # ---- 3-D text display ----
        if self.b_show_text:
            from utils.show import Show_3D
            self.show = Show_3D(robot_xyz_init=tuple(disp_cfg["robot_xyz_init_mm"]))

        # ---- Trajectory data ----
        traj_path = traj_cfg["file_path"]
        if os.path.isfile(traj_path):
            self.traject = np.loadtxt(traj_path)                # shape (5, N)
            mag_h = traj_cfg["mag_height_init_mm"] * 1e-3
            self.traject[2, :] -= mag_h
            print("Loaded trajectory: %s  (%d waypoints)", traj_path, self.traject.shape[1])
        else:
            rospy.logwarn("Trajectory file not found: %s — tracking mode disabled", traj_path)
            self.traject = None

        # ---- State variables ----
        # These represent the MAGNET TIP pose (not link8)
        self.x = self.y = self.z = -1.0
        self.roll = self.pitch = self.yaw = -1.0
        self.pose = np.zeros(6)

        self.b_run = False
        self.b_reach_flag = False

        # Tracking mode state
        self.b_tracking = False
        self.b_tracking_first = True
        self.indice_tracking = 0
        self.pose_origin = np.zeros(6)
        self.b_indice_change = True
        self.dx = self.dy = self.dz = 0.0
        self.d_alpha = self.d_phi = self.d_yaw = 0.0
        self.joy_start_time = 0.0

        # ---- Joystick subscriber ----
        self.joy_sub = rospy.Subscriber("joy", Joy, self._joy_callback, queue_size=10)

        # ---- Shutdown hook ----
        rospy.on_shutdown(self._shutdown)

        # ---- Optional: go to joints_init as fallback ----
        rate1 = rospy.Rate(1)
        if robot_cfg.get("use_joints_init", False):
            joints = robot_cfg.get("joints_init", [])
            if joints:
                print("Moving to joints_init (fallback mode)...")
                self.group.go(joints, wait=True)
                rate1.sleep()

        # ================================================================
        #  HAND-GUIDE INITIAL POSE CAPTURE
        # ================================================================
        print("="*60)
        print("  HAND-GUIDE MODE")
        print("  The arm should already be at the desired starting pose.")
        print("  Press ENTER in this terminal to capture the current pose.")
        print("="*60)

        # Block until the user confirms (runs in a separate thread to not
        # freeze ROS callbacks, but we don't expect any callbacks yet)
        input(">>> Press ENTER to capture the current pose as initial pose... ")

        # Read back link8 pose from MoveIt
        lx, ly, lz, lr, lp, lyw = get_link8_euler(self.group)
        print("link8 pose read-back:  xyz=(%.4f, %.4f, %.4f)  rpy=(%.2f, %.2f, %.2f) deg",
                      lx, ly, lz, degrees(lr), degrees(lp), degrees(lyw))

        # ---- Verify Euler round-trip on the read-back angles ----
        verify_euler_roundtrip(lr, lp, lyw, label="link8 read-back")

        # Convert link8 pose → magnet-tip pose using the forward model
        mag = link_forward_L(lx, ly, lz, lr, lp, lyw,
                             self.end_effector_height, self.end_effector_length)
        self.x, self.y, self.z = mag[0], mag[1], mag[2]
        self.roll, self.pitch, self.yaw = mag[3], mag[4], mag[5]
        self.pose = np.array(mag)

        print("Magnet-tip initial pose:  xyz=(%.4f, %.4f, %.4f)  rpy=(%.2f, %.2f, %.2f) deg",
                      self.x, self.y, self.z,
                      degrees(self.roll), degrees(self.pitch), degrees(self.yaw))

        # ---- Double-check: convert back to link8 and compare ----
        lb = link_back_L(self.x, self.y, self.z, self.roll, self.pitch, self.yaw,
                         self.end_effector_height, self.end_effector_length)
        err_pos = np.linalg.norm(np.array([lx, ly, lz]) - np.array(lb[:3]))
        err_ang = np.linalg.norm(np.array([lr, lp, lyw]) - np.array(lb[3:]))
        print("Forward/back consistency:  pos_err=%.6f m,  ang_err=%.6f rad", err_pos, err_ang)
        if err_pos > 1e-4 or err_ang > 1e-3:
            rospy.logwarn("Forward/back consistency error is large — please verify end-effector dimensions!")

        # Store the initial pose for display offset calculations
        self.pose_init = np.array(mag)

        # Also record the initial joint configuration (for singularity recovery)
        self.joints_at_init = self.group.get_current_joint_values()
        print("Initial joints: %s", self.joints_at_init)

        print("Initialisation complete.  Joystick control is now active.")

    # ------------------------------------------------------------------
    # MoveIt planning helpers  (from controller_CX)
    # ------------------------------------------------------------------

    def _plan_and_execute(self, pose_goal_msg):
        """Plan a path to pose_goal_msg (geometry_msgs/Pose) and execute it.

        Includes singularity detection and re-planning logic.

        Returns True on success, False if all planning attempts failed.
        """
        group = self.group

        # Optionally apply path constraints (after the very first move)
        if (not self.b_first_move) and self.b_constraint:
            pass  # group.set_path_constraints(self.end_constraints)
        else:
            group.clear_path_constraints()

        try:
            flag, trajectory = self._plan_path(pose_goal_msg)
            if not flag:
                # Try recovering via the initial joint configuration
                rospy.logwarn("Planning failed — attempting recovery via initial joints...")
                group.go(self.joints_at_init, wait=True)
                flag, trajectory = self._plan_path(pose_goal_msg)
        except Exception as e:
            rospy.logerr("Planning exception: %s", str(e))
            return False

        if not flag:
            rospy.logerr("All planning attempts failed.")
            return False

        t0 = time.time()
        group.execute(trajectory, wait=True)
        print("Execution time: %.3f s", time.time() - t0)

        if self.b_first_move:
            self.b_first_move = False

        return True

    def _plan_path(self, pose_goal):
        """Set pose target, plan, and check for singularity jumps."""
        group = self.group
        group.set_pose_target(pose_goal)
        flag_plan, trajectory, plan_time, error_code = group.plan()

        # Check for large joint jumps (singularity indicator)
        joints_current = np.array(group.get_current_joint_values())
        joints_next    = np.array(trajectory.joint_trajectory.points[-1].positions)
        error_joints   = np.abs(joints_current - joints_next)[:-1]  # ignore last (finger) joint
        error_sum      = np.sum(error_joints)

        flag = True
        if error_sum > self.thresh_singular[0]:
            flag = False
            rospy.logwarn("Singularity detected (joint error=%.3f) — replanning...", error_sum)
            for i, (thresh, rep) in enumerate(zip(self.thresh_singular, self.planning_repeat)):
                flag, trajectory, err = self._replan(pose_goal, thresh, rep)
                if flag:
                    print("Replan %d succeeded (error=%.3f)", i, err)
                    break
                else:
                    rospy.logwarn("Replan %d failed (error=%.3f)", i, err)

        return flag, trajectory

    def _replan(self, pose_goal, thresh, repeat):
        """Try multiple plans and pick the one with smallest joint jump."""
        trajects = []
        indexes  = []
        group = self.group

        for _ in range(repeat):
            group.set_pose_target(pose_goal)
            _, traj, _, _ = group.plan()
            jc = np.array(group.get_current_joint_values())
            pts = traj.joint_trajectory.points
            jn = np.array([pts[k].positions for k in range(len(pts))])
            n = jc.shape[0]
            jc = jc.reshape(-1, n)
            jn = jn.reshape(-1, n)[1:, :]
            errs = np.sum(np.abs(jc - jn)[:, :-1], axis=-1)
            trajects.append(traj)
            indexes.append(np.max(errs))

        best = int(np.argmin(indexes))
        ok = min(indexes) < thresh
        return ok, trajects[best], indexes[best]

    # ------------------------------------------------------------------
    # Move helpers  (from arm_control)
    # ------------------------------------------------------------------

    def move_to_magnet_pose(self, pose):
        """Move the arm so that the magnet tip reaches the given 6-DOF pose.

        Parameters
        ----------
        pose : array-like of 6 floats
            [x, y, z, roll, pitch, yaw] of the magnet tip in metres/radians.

        The function:
          1. Checks whether the requested pose differs from the current one
             (within tolerance) to avoid redundant moves.
          2. Calls link_back_L to compute the required panda_link8 pose.
          3. Plans & executes via MoveIt.
          4. Blocks until the motion is complete.
        """
        xyz_new = np.array(pose[:3])
        xyz_old = np.array([self.x, self.y, self.z])
        cond_trans = np.any(np.abs(xyz_new - xyz_old) > (self.arm_acc * 0.99))

        rpy_new = np.array(pose[3:])
        rpy_old = np.array([self.roll, self.pitch, self.yaw])
        cond_rot = np.any(np.abs(rpy_new - rpy_old) > radians(0.99))

        if not (cond_trans or cond_rot):
            return  # nothing to do

        # Update internal state
        if cond_trans:
            self.x, self.y, self.z = xyz_new
        if cond_rot:
            self.roll, self.pitch, self.yaw = rpy_new
        self.pose = np.array(pose)
        self.b_run = True

        # Compute link8 target
        lp = link_back_L(self.x, self.y, self.z,
                         self.roll, self.pitch, self.yaw,
                         self.end_effector_height, self.end_effector_length)

        # Build geometry_msgs/Pose
        wpose = Pose()
        wpose.position.x = np.round(lp[0] / self.arm_acc, 0) * self.arm_acc
        wpose.position.y = np.round(lp[1] / self.arm_acc, 0) * self.arm_acc
        wpose.position.z = np.round(lp[2] / self.arm_acc, 0) * self.arm_acc

        q = quaternion_from_euler(lp[3], lp[4], lp[5], 'rxyz')
        wpose.orientation.x = q[0]
        wpose.orientation.y = q[1]
        wpose.orientation.z = q[2]
        wpose.orientation.w = q[3]

        t0 = time.time()
        ok = self._plan_and_execute(wpose)
        self.b_run = False

        # Log
        print("magnet xyz=(%.3f, %.3f, %.3f)  rpy=(%.1f, %.1f, %.1f) deg  | %.2f s  %s",
                      self.x, self.y, self.z,
                      degrees(self.roll), degrees(self.pitch), degrees(self.yaw),
                      time.time() - t0, "OK" if ok else "FAIL")

    # ------------------------------------------------------------------
    # Joystick callback
    # ------------------------------------------------------------------

    def _joy_callback(self, msg):
        """Process joystick input (Xbox-style controller).

        Button mapping (same as original):
          D-pad up/down/left/right  → axes[-1], axes[-2]
          X=buttons[0]  Y=buttons[3]  B=buttons[2]  A=buttons[1]
          LB=buttons[4]  RB=buttons[5]
          Start=buttons[-3]  Back=buttons[-4]

        Control modes
        -------------
        Manual mode (b_tracking=False):
          D-pad alone        → small dx / dy
          D-pad + LB         → big dx / dy
          D-pad + RB         → dz (small=up/down, big=left/right)
          X/B alone          → small d_pitch
          X/B + RB           → big d_pitch
          Y/A alone          → small d_roll
          Y/A + RB           → big d_roll
          X/B + LB + RB      → small d_yaw  (NEW — helps escape singularities)
          Start              → enter tracking mode

        Tracking mode (b_tracking=True):
          B / X (alone)      → next / previous trajectory step
          D-pad / buttons    → same offsets as manual but applied on top of trajectory
          Start              → stay in tracking
          Back               → exit tracking

        Motor (if enabled):
          Y + LB (no RB)     → rotate motor CW
          A + LB (no RB)     → rotate motor CCW
          X + LB (no RB)     → speed up
          B + LB (no RB)     → speed down
          LB + RB            → stop motor
        """
        self.joy_start_time = time.time()

        # --- Parse buttons ---
        b_up    = msg.axes[-1] > 0.5
        b_down  = msg.axes[-1] < -0.5
        b_left  = msg.axes[-2] > 0.5
        b_right = msg.axes[-2] < -0.5

        b_X = (msg.buttons[0] == 1)
        b_A = (msg.buttons[1] == 1)
        b_B = (msg.buttons[2] == 1)
        b_Y = (msg.buttons[3] == 1)

        b_LB = (msg.buttons[4] == 1)
        b_RB = (msg.buttons[5] == 1)

        b_start = (msg.buttons[-3] == 1)
        b_back  = (msg.buttons[-4] == 1)

        # --- Mode switching ---
        if b_start:
            if not self.b_tracking:
                self.b_tracking = True
                self.b_tracking_first = True
        if b_back:
            self.b_tracking = False

        # --- Compute deltas ---
        if self.b_tracking:
            self._handle_tracking(b_up, b_down, b_left, b_right,
                                  b_X, b_A, b_B, b_Y, b_LB, b_RB)
        else:
            self._handle_manual(b_up, b_down, b_left, b_right,
                                b_X, b_A, b_B, b_Y, b_LB, b_RB)

        # --- Motor control (independent of tracking/manual) ---
        if self.b_motor_enable:
            if b_Y and b_LB and not b_RB:
                self.stepper.servo_rotate(1000)
            if b_A and b_LB and not b_RB:
                self.stepper.servo_rotate(-1000)
            if b_X and b_LB and not b_RB:
                self.stepper.stepper_vel_adjust(1)
            if b_B and b_LB and not b_RB:
                self.stepper.stepper_vel_adjust(-1)
            if b_RB and b_LB:
                self.stepper.servo_stop()

    # --- Manual mode handler ---

    def _handle_manual(self, b_up, b_down, b_left, b_right,
                       b_X, b_A, b_B, b_Y, b_LB, b_RB):
        dx = dy = dz = 0.0
        d_roll = d_pitch = d_yaw = 0.0
        s = self.step

        # dx (right/left on D-pad, no bumpers or with LB for big)
        if b_right and not b_LB and not b_RB:
            dx = s['x_small']
        if b_right and b_LB and not b_RB:
            dx = s['x_big']
        if b_left and not b_LB and not b_RB:
            dx = -s['x_small']
        if b_left and b_LB and not b_RB:
            dx = -s['x_big']

        # dy (up/down on D-pad, no bumpers or with LB for big)
        if b_up and not b_LB and not b_RB:
            dy = s['y_small']
        if b_up and b_LB and not b_RB:
            dy = s['y_big']
        if b_down and not b_LB and not b_RB:
            dy = -s['y_small']
        if b_down and b_LB and not b_RB:
            dy = -s['y_big']

        # dz (D-pad with RB only)
        if b_up and b_RB and not b_LB:
            dz = s['z_small']
        if b_down and b_RB and not b_LB:
            dz = -s['z_small']
        if b_right and b_RB and not b_LB:
            dz = s['z_big']
        if b_left and b_RB and not b_LB:
            dz = -s['z_big']

        # d_pitch (X/B, no bumpers or with RB for big)
        if b_X and not b_RB and not b_LB:
            d_pitch = -s['pitch_small']
        if b_B and not b_RB and not b_LB:
            d_pitch = s['pitch_small']
        if b_X and b_RB and not b_LB:
            d_pitch = -s['pitch_big']
        if b_B and b_RB and not b_LB:
            d_pitch = s['pitch_big']

        # d_roll (Y/A, no bumpers or with RB for big)
        if b_Y and not b_RB and not b_LB:
            d_roll = -s['roll_small']
        if b_A and not b_RB and not b_LB:
            d_roll = s['roll_small']
        if b_Y and b_RB and not b_LB:
            d_roll = -s['roll_big']
        if b_A and b_RB and not b_LB:
            d_roll = s['roll_big']

        # d_yaw (X/B with BOTH LB+RB — NEW for singularity avoidance)
        if b_X and b_LB and b_RB:
            d_yaw = -s['yaw_small']
        if b_B and b_LB and b_RB:
            d_yaw = s['yaw_small']
        if b_Y and b_LB and b_RB:
            d_yaw = -s['yaw_big']
        if b_A and b_LB and b_RB:
            d_yaw = s['yaw_big']

        new_pose = [self.x + dx, self.y + dy, self.z + dz,
                    self.roll + d_roll, self.pitch + d_pitch, self.yaw + d_yaw]
        self.move_to_magnet_pose(new_pose)

    # --- Tracking mode handler ---

    def _handle_tracking(self, b_up, b_down, b_left, b_right,
                         b_X, b_A, b_B, b_Y, b_LB, b_RB):
        if self.traject is None:
            rospy.logwarn_throttle(5, "No trajectory loaded — tracking mode unavailable")
            return

        s = self.step

        if self.b_tracking_first:
            print(">>>>> Start trajectory tracking <<<<<")
            self.b_tracking_first = False
            self.indice_tracking = 0
            self.pose_origin = np.array([self.x, self.y, self.z,
                                         self.roll, self.pitch, self.yaw]).copy()

        self.b_indice_change = False

        # Step through trajectory with B/X (no bumpers)
        if b_B and not b_LB and not b_RB:
            self.indice_tracking += 1
            self.b_indice_change = True
        if b_X and not b_LB and not b_RB:
            self.indice_tracking -= 1
            self.b_indice_change = True

        n_pts = self.traject.shape[1]
        self.indice_tracking = int(np.sign(self.indice_tracking) *
                                   (abs(self.indice_tracking) % n_pts))
        print("Tracking: %d / %d", self.indice_tracking, n_pts)

        dx_t, dy_t, dz_t, da_t, dp_t = self.traject[:, self.indice_tracking]

        # Manual adjustment on top of trajectory
        dx = dy = dz = 0.0
        d_roll = d_pitch = 0.0

        if b_right and not b_LB and not b_RB: dx = s['x_small']
        if b_right and b_LB and not b_RB:     dx = s['x_big']
        if b_left  and not b_LB and not b_RB: dx = -s['x_small']
        if b_left  and b_LB and not b_RB:     dx = -s['x_big']

        if b_up   and not b_LB and not b_RB: dy = s['y_small']
        if b_up   and b_LB and not b_RB:     dy = s['y_big']
        if b_down and not b_LB and not b_RB: dy = -s['y_small']
        if b_down and b_LB and not b_RB:     dy = -s['y_big']

        if b_up   and b_RB and not b_LB: dz = s['z_small']
        if b_down and b_RB and not b_LB: dz = -s['z_small']
        if b_right and b_RB and not b_LB: dz = s['z_big']
        if b_left  and b_RB and not b_LB: dz = -s['z_big']

        if b_X and b_RB and not b_LB: d_pitch = -s['pitch_small']
        if b_B and b_RB and not b_LB: d_pitch = s['pitch_small']
        if b_Y and b_RB and not b_LB: d_roll = -s['roll_small']
        if b_A and b_RB and not b_LB: d_roll = s['roll_small']

        if self.b_indice_change:
            self.dx = self.dy = self.dz = 0.0
            self.d_alpha = self.d_phi = self.d_yaw = 0.0
        else:
            self.dx += dx
            self.dy += dy
            self.dz += dz
            self.d_alpha += d_roll
            self.d_phi   += d_pitch

        new_pose = self.pose_origin + np.array([
            dx_t + self.dx,
            dy_t + self.dy,
            dz_t + self.dz,
            da_t + self.d_alpha,
            dp_t + self.d_phi,
            0    + self.d_yaw
        ])

        self.move_to_magnet_pose(new_pose)

    # ------------------------------------------------------------------
    # Display loop  (optional, runs on the main thread)
    # ------------------------------------------------------------------

    def run_display(self):
        """Blocking loop that updates the 3-D text overlay (if enabled)."""
        if not self.b_show_text:
            rospy.spin()
            return

        pi0 = self.pose_init
        while not rospy.is_shutdown():
            x_mm    = int(round((self.x     - pi0[0]) * 1000))
            y_mm    = int(round((self.y     - pi0[1]) * 1000))
            z_mm    = int(round((self.z     - pi0[2]) * 1000))
            alpha_d = int(degrees(self.roll  - pi0[3]))
            theta_d = int(degrees(self.pitch - pi0[4]))

            if self.b_run:
                self.show.draw([x_mm, y_mm, z_mm, alpha_d, theta_d], color=(0, 255, 0))
            else:
                self.show.draw([x_mm, y_mm, z_mm, alpha_d, theta_d], color=(255, 255, 255))

    # ------------------------------------------------------------------
    # Rviz marker helpers
    # ------------------------------------------------------------------

    def _remove_all_markers(self):
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.ns = "/"
        marker.action = visualization_msgs.msg.Marker.DELETEALL
        self.marker_pub.publish(marker)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def _shutdown(self):
        print("Shutting down franka_magnet_controller...")
        if self.b_show_text and HAS_CV2:
            try:
                cv.destroyAllWindows()
            except Exception:
                pass


# ===========================================================================
#  Entry point
# ===========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Franka FR3 magnet arm controller (merged)")
    parser.add_argument('--config', type=str, default='/home/roboticarm/catkin_ws/src/arm_control_magnet/src/config.json',
                        help='Path to JSON configuration file')
    # rospy strips ROS args; use rospy.myargv() so argparse doesn't choke
    args = parser.parse_args(rospy.myargv()[1:])

    cfg = load_config(args.config)

    try:
        ctrl = FrankaMagnetController(cfg)
        ctrl.run_display()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
