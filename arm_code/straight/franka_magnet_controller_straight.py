#!/usr/bin/env python3
"""
franka_magnet_controller_straight.py  –  Joystick controller, Franka FR3 + STRAIGHT magnet end effector

Straight-end-effector version of arm_code/new/franka_magnet_controller.py:
  * magnet tip <-> panda_link8 with the coaxial model (utils/kinematics.py),
    tool length = config_straight.json -> end_effector.length_mm
  * initial pose: config_straight.json -> init.mode = "hand_guide" (default) | "fixed_pose"
  * streams the live arm / tip state on /arm_state for the recording PC
  * motor disabled by default; then LB+X/B/Y/A rotate the yaw (3rd Euler angle)

Usage (full instructions and the joystick map: GUIDE.md at the repository root):
  rosrun arm_control_magnet franka_magnet_controller_straight.py [--config config_straight.json]
"""

import sys
import os
import argparse
import time
from types import SimpleNamespace
from math import radians, degrees

import numpy as np
import rospy
import moveit_commander
from sensor_msgs.msg import Joy

from utils.config_io import load_config, resolve_path, load_trajectory
from utils.kinematics import link_back_straight, link_pose_msg
from utils.init_pose import initialise_pose
from utils.arm_state_publisher import ArmStatePublisher

try:
    import cv2 as cv
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class FrankaMagnetController:
    """MoveIt planning + joystick input (+ optional motor) for the straight end effector."""

    def __init__(self, cfg):
        self.cfg = cfg
        robot_cfg = cfg["robot"]
        step_cfg = cfg["step_sizes"]
        motor_cfg = cfg["motor"]
        traj_cfg = cfg["trajectory"]
        disp_cfg = cfg["display"]
        self.joy_cfg = cfg["joystick"]

        self.ee_length = cfg["end_effector"]["length_mm"] * 1e-3        # metres
        self.arm_acc = robot_cfg["arm_acc"]
        self.step = {
            'x_small': step_cfg["x"]["small_m"], 'x_big': step_cfg["x"]["big_m"],
            'y_small': step_cfg["y"]["small_m"], 'y_big': step_cfg["y"]["big_m"],
            'z_small': step_cfg["z"]["small_m"], 'z_big': step_cfg["z"]["big_m"],
            'roll_small': step_cfg["roll"]["small_rad"], 'roll_big': step_cfg["roll"]["big_rad"],
            'pitch_small': step_cfg["pitch"]["small_rad"], 'pitch_big': step_cfg["pitch"]["big_rad"],
            'yaw_small': step_cfg["yaw"]["small_rad"], 'yaw_big': step_cfg["yaw"]["big_rad"],
        }
        self.thresh_singular = robot_cfg["singularity_thresholds"]
        self.planning_repeat = robot_cfg["planning_repeat"]

        self.b_motor_enable = motor_cfg.get("enable", False)
        self.b_show_text = disp_cfg.get("show_text", False) and HAS_CV2

        # ---- ROS / MoveIt ----
        rospy.init_node('franka_magnet_controller_straight', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(robot_cfg["group_name"])
        self.group.set_max_velocity_scaling_factor(robot_cfg["vel_scale"])
        self.group.set_max_acceleration_scaling_factor(robot_cfg["acc_scale"])
        print(f"Planning frame : {self.group.get_planning_frame()}")
        print(f"End effector   : {self.group.get_end_effector_link()}")
        print(f"Tool           : straight, {cfg['end_effector']['length_mm']:.1f} mm")

        # ---- live state stream for the recording PC (starts immediately) ----
        self.state_pub = ArmStatePublisher(cfg, source='franka_magnet_controller_straight')
        self.state_pub.set_mode('init')

        # ---- motor (optional) ----
        if self.b_motor_enable:
            from utils.arduino_control import Stepper_servo
            self.stepper = Stepper_servo(serial_no=motor_cfg["serial_no"],
                                         servo_vel_step=motor_cfg["step_rotation"])
        print(f"Motor          : {'ENABLED (LB+X/B/Y/A = motor)' if self.b_motor_enable else 'disabled (LB+X/B/Y/A = yaw)'}")

        # ---- optional text window ----
        if self.b_show_text:
            from utils.show import Show_3D
            self.show = Show_3D(robot_xyz_init=tuple(disp_cfg["robot_xyz_init_mm"]))

        # ---- tracking trajectory (5xN or 6xN, relative offsets) ----
        self.traj_path = resolve_path(cfg, traj_cfg.get("tracking_file", ""))
        if self.traj_path and os.path.isfile(self.traj_path):
            self.traject = load_trajectory(self.traj_path)          # (6, N)
            self.traject[2, :] -= traj_cfg.get("tracking_mag_height_init_mm", 0) * 1e-3
            print(f"Tracking trajectory: {self.traj_path} ({self.traject.shape[1]} waypoints)")
        else:
            print(f"WARNING: tracking trajectory not found: {self.traj_path} - tracking mode disabled")
            self.traject = None

        # ---- state (MAGNET TIP pose, not link8) ----
        self.x = self.y = self.z = -1.0
        self.roll = self.pitch = self.yaw = -1.0
        self.pose = np.zeros(6)
        self.b_run = False
        self.b_first_move = True
        self.b_tracking = False
        self.b_tracking_first = True
        self.indice_tracking = 0
        self.pose_origin = np.zeros(6)
        self.dx = self.dy = self.dz = 0.0
        self.d_alpha = self.d_phi = self.d_yaw = 0.0
        self.joints_at_init = cfg["init"].get("joints_init") or None

        rospy.on_shutdown(self._shutdown)

        # ---- initial pose (hand-guide or fixed) ----
        tip0 = initialise_pose(self.group, cfg, self.move_to_magnet_pose, self.state_pub)
        self.x, self.y, self.z, self.roll, self.pitch, self.yaw = tip0
        self.pose = np.array(tip0)
        self.pose_init = np.array(tip0)
        self.joints_at_init = self.group.get_current_joint_values()

        # joystick only after the initial pose is set
        self.state_pub.set_mode('manual')
        self.joy_sub = rospy.Subscriber("joy", Joy, self._joy_callback, queue_size=10)
        print("Initialisation complete. Joystick control is active (manual mode).")

    # ------------------------------------------------------------------
    # MoveIt planning
    # ------------------------------------------------------------------

    def _plan_and_execute(self, pose_goal_msg):
        """Plan to a panda_link8 Pose and execute.  Returns True on success."""
        group = self.group
        group.clear_path_constraints()
        try:
            flag, trajectory = self._plan_path(pose_goal_msg)
            if not flag and self.joints_at_init:
                print("Planning failed - recovering via the initial joints ...")
                group.go(self.joints_at_init, wait=True)
                flag, trajectory = self._plan_path(pose_goal_msg)
        except Exception as e:
            print(f"ERROR planning: {e}")
            return False
        if not flag:
            print("ERROR: all planning attempts failed.")
            return False

        t0 = time.time()
        ok = group.execute(trajectory, wait=True)
        group.stop()
        print(f"Execution time: {time.time() - t0:.3f} s")
        self.b_first_move = False
        return bool(ok)

    def _plan_path(self, pose_goal):
        """Set the pose target, plan, and check for singularity jumps."""
        group = self.group
        group.set_pose_target(pose_goal)
        _, trajectory, _, _ = group.plan()

        joints_current = np.array(group.get_current_joint_values())
        joints_next = np.array(trajectory.joint_trajectory.points[-1].positions)
        error_sum = np.sum(np.abs(joints_current - joints_next)[:-1])

        flag = True
        if error_sum > self.thresh_singular[0]:
            flag = False
            print(f"Singularity detected (joint error={error_sum:.3f}) - replanning ...")
            for i, (thresh, rep) in enumerate(zip(self.thresh_singular, self.planning_repeat)):
                flag, trajectory, err = self._replan(pose_goal, thresh, rep)
                print(f"Replan {i} {'succeeded' if flag else 'failed'} (error={err:.3f})")
                if flag:
                    break
        return flag, trajectory

    def _replan(self, pose_goal, thresh, repeat):
        """Plan several times and keep the plan with the smallest joint jump."""
        trajects, indexes = [], []
        group = self.group
        for _ in range(repeat):
            group.set_pose_target(pose_goal)
            _, traj, _, _ = group.plan()
            jc = np.array(group.get_current_joint_values())
            pts = traj.joint_trajectory.points
            jn = np.array([pts[k].positions for k in range(len(pts))])
            n = jc.shape[0]
            errs = np.sum(np.abs(jc.reshape(-1, n) - jn.reshape(-1, n)[1:, :])[:, :-1], axis=-1)
            trajects.append(traj)
            indexes.append(np.max(errs))
        best = int(np.argmin(indexes))
        return indexes[best] < thresh, trajects[best], indexes[best]

    # ------------------------------------------------------------------
    # Move the magnet tip
    # ------------------------------------------------------------------

    def move_to_magnet_pose(self, pose):
        """Move the magnet tip to pose=[x, y, z, roll, pitch, yaw] (m, rad).  Returns True on success."""
        xyz_new = np.array(pose[:3], dtype=float)
        xyz_old = np.array([self.x, self.y, self.z])
        cond_trans = np.any(np.abs(xyz_new - xyz_old) > (self.arm_acc * 0.99))
        rpy_new = np.array(pose[3:], dtype=float)
        rpy_old = np.array([self.roll, self.pitch, self.yaw])
        cond_rot = np.any(np.abs(rpy_new - rpy_old) > radians(0.99))
        if not (cond_trans or cond_rot):
            return True     # nothing to do

        if cond_trans:
            self.x, self.y, self.z = xyz_new
        if cond_rot:
            self.roll, self.pitch, self.yaw = rpy_new
        target = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
        self.pose = np.array(target)

        wpose = link_pose_msg(link_back_straight(*target, self.ee_length), self.arm_acc)

        self.b_run = True
        self.state_pub.set_target(tip=target)
        t0 = time.time()
        ok = self._plan_and_execute(wpose)
        self.state_pub.motion_done(ok)
        self.b_run = False

        print(f"tip xyz=({self.x:.4f}, {self.y:.4f}, {self.z:.4f}) m  "
              f"rpy=({degrees(self.roll):.1f}, {degrees(self.pitch):.1f}, {degrees(self.yaw):.1f}) deg  "
              f"| {time.time() - t0:.2f} s  {'OK' if ok else 'FAIL'}")
        return ok

    # ------------------------------------------------------------------
    # Joystick
    # ------------------------------------------------------------------

    def _read_buttons(self, msg):
        j = self.joy_cfg
        ud = msg.axes[j["axis_dpad_up_down"]]
        lr = msg.axes[j["axis_dpad_left_right"]]
        btn = lambda name: msg.buttons[j[name]] == 1
        return SimpleNamespace(
            up=ud > 0.5, down=ud < -0.5, left=lr > 0.5, right=lr < -0.5,
            X=btn("button_X"), A=btn("button_A"), B=btn("button_B"), Y=btn("button_Y"),
            LB=btn("button_LB"), RB=btn("button_RB"),
            start=btn("button_start"), back=btn("button_back"))

    def _joy_callback(self, msg):
        """Joystick map: see GUIDE.md.  Start = tracking mode, Back = manual mode."""
        b = self._read_buttons(msg)

        if b.start and not self.b_tracking:
            self.b_tracking = True
            self.b_tracking_first = True
        if b.back:
            self.b_tracking = False
        self.state_pub.set_mode('tracking' if self.b_tracking else 'manual')
        if not self.b_tracking:
            self.state_pub.set_waypoint(None)

        if self.b_tracking:
            self._handle_tracking(b)
        else:
            self._handle_manual(b)

        # motor (only when enabled): LB + Y/A rotate, LB + X/B speed, LB+RB stop
        if self.b_motor_enable:
            if b.Y and b.LB and not b.RB:
                self.stepper.servo_rotate(1000)
            if b.A and b.LB and not b.RB:
                self.stepper.servo_rotate(-1000)
            if b.X and b.LB and not b.RB:
                self.stepper.stepper_vel_adjust(1)
            if b.B and b.LB and not b.RB:
                self.stepper.stepper_vel_adjust(-1)
            if b.RB and b.LB:
                self.stepper.servo_stop()

    def _xyz_offsets(self, b):
        """D-pad: dx/dy (small; +LB big), dz with RB (up/down small, right/left big)."""
        s = self.step
        dx = dy = dz = 0.0
        if b.right and not b.LB and not b.RB: dx = s['x_small']
        if b.right and b.LB and not b.RB:     dx = s['x_big']
        if b.left and not b.LB and not b.RB:  dx = -s['x_small']
        if b.left and b.LB and not b.RB:      dx = -s['x_big']

        if b.up and not b.LB and not b.RB:    dy = s['y_small']
        if b.up and b.LB and not b.RB:        dy = s['y_big']
        if b.down and not b.LB and not b.RB:  dy = -s['y_small']
        if b.down and b.LB and not b.RB:      dy = -s['y_big']

        if b.up and b.RB and not b.LB:        dz = s['z_small']
        if b.down and b.RB and not b.LB:      dz = -s['z_small']
        if b.right and b.RB and not b.LB:     dz = s['z_big']
        if b.left and b.RB and not b.LB:      dz = -s['z_big']
        return dx, dy, dz

    def _yaw_offset(self, b):
        """Yaw (3rd Euler angle): X/B small, Y/A big, with LB+RB; with LB alone when the motor is disabled."""
        s = self.step
        d_yaw = 0.0
        yaw_combo = b.LB and (b.RB or not self.b_motor_enable)
        if yaw_combo:
            if b.X: d_yaw = -s['yaw_small']
            if b.B: d_yaw = s['yaw_small']
            if b.Y: d_yaw = -s['yaw_big']
            if b.A: d_yaw = s['yaw_big']
        return d_yaw

    def _handle_manual(self, b):
        s = self.step
        dx, dy, dz = self._xyz_offsets(b)

        d_pitch = 0.0       # X/B: small, +RB big
        if b.X and not b.RB and not b.LB: d_pitch = -s['pitch_small']
        if b.B and not b.RB and not b.LB: d_pitch = s['pitch_small']
        if b.X and b.RB and not b.LB:     d_pitch = -s['pitch_big']
        if b.B and b.RB and not b.LB:     d_pitch = s['pitch_big']

        d_roll = 0.0        # Y/A: small, +RB big
        if b.Y and not b.RB and not b.LB: d_roll = -s['roll_small']
        if b.A and not b.RB and not b.LB: d_roll = s['roll_small']
        if b.Y and b.RB and not b.LB:     d_roll = -s['roll_big']
        if b.A and b.RB and not b.LB:     d_roll = s['roll_big']

        d_yaw = self._yaw_offset(b)

        self.move_to_magnet_pose([self.x + dx, self.y + dy, self.z + dz,
                                  self.roll + d_roll, self.pitch + d_pitch, self.yaw + d_yaw])

    def _handle_tracking(self, b):
        if self.traject is None:
            print("No tracking trajectory loaded - tracking mode unavailable")
            return
        s = self.step

        if self.b_tracking_first:
            print(">>>>> Start trajectory tracking <<<<<")
            self.b_tracking_first = False
            self.indice_tracking = 0
            self.pose_origin = np.array([self.x, self.y, self.z, self.roll, self.pitch, self.yaw])
            self.dx = self.dy = self.dz = 0.0
            self.d_alpha = self.d_phi = self.d_yaw = 0.0

        # B / X (no bumpers): next / previous waypoint
        b_indice_change = False
        if b.B and not b.LB and not b.RB:
            self.indice_tracking += 1
            b_indice_change = True
        if b.X and not b.LB and not b.RB:
            self.indice_tracking -= 1
            b_indice_change = True

        n_pts = self.traject.shape[1]
        self.indice_tracking = int(np.sign(self.indice_tracking) * (abs(self.indice_tracking) % n_pts))
        col = self.indice_tracking % n_pts
        print(f"Tracking: {self.indice_tracking} / {n_pts}")
        dx_t, dy_t, dz_t, da_t, dp_t, dyaw_t = self.traject[:, col]

        # manual correction on top of the trajectory
        dx, dy, dz = self._xyz_offsets(b)
        d_pitch = d_roll = 0.0
        if b.X and b.RB and not b.LB: d_pitch = -s['pitch_small']
        if b.B and b.RB and not b.LB: d_pitch = s['pitch_small']
        if b.Y and b.RB and not b.LB: d_roll = -s['roll_small']
        if b.A and b.RB and not b.LB: d_roll = s['roll_small']
        d_yaw = self._yaw_offset(b)

        if b_indice_change:
            self.dx = self.dy = self.dz = 0.0
            self.d_alpha = self.d_phi = self.d_yaw = 0.0
        else:
            self.dx += dx
            self.dy += dy
            self.dz += dz
            self.d_alpha += d_roll
            self.d_phi += d_pitch
            self.d_yaw += d_yaw

        new_pose = self.pose_origin + np.array([dx_t + self.dx, dy_t + self.dy, dz_t + self.dz,
                                                da_t + self.d_alpha, dp_t + self.d_phi,
                                                dyaw_t + self.d_yaw])
        self.state_pub.set_waypoint(col, n_pts, self.traj_path)
        self.move_to_magnet_pose(new_pose)

    # ------------------------------------------------------------------
    # Optional text window / spin
    # ------------------------------------------------------------------

    def run_display(self):
        if not self.b_show_text:
            rospy.spin()
            return
        pi0 = self.pose_init
        while not rospy.is_shutdown():
            vals = [int(round((self.x - pi0[0]) * 1000)), int(round((self.y - pi0[1]) * 1000)),
                    int(round((self.z - pi0[2]) * 1000)),
                    int(degrees(self.roll - pi0[3])), int(degrees(self.pitch - pi0[4]))]
            self.show.draw(vals, color=(0, 255, 0) if self.b_run else (255, 255, 255))

    def _shutdown(self):
        print("Shutting down franka_magnet_controller_straight ...")
        if self.b_motor_enable:
            try:
                self.stepper.servo_stop()
            except Exception:
                pass
        if self.b_show_text:
            try:
                cv.destroyAllWindows()
            except Exception:
                pass


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        description="Joystick controller for the Franka FR3 with the STRAIGHT magnet end effector. See GUIDE.md.")
    parser.add_argument('--config', type=str, default=os.path.join(here, 'config_straight.json'),
                        help='JSON configuration file')
    args = parser.parse_args(rospy.myargv()[1:])

    cfg = load_config(args.config)
    try:
        ctrl = FrankaMagnetController(cfg)
        ctrl.run_display()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
