#!/usr/bin/env python3
"""
execute_magnet_path_straight.py  –  Execute a magnet trajectory, Franka FR3 + STRAIGHT magnet end effector

Straight-end-effector version of arm_code/new/execute_magnet_path.py.
Loads a 6xN (x, y, z, alpha, phi, yaw) or 5xN trajectory of offsets relative to
the initial magnet-tip pose and drives the arm through every waypoint
(point-by-point MoveIt plan + execute).  The live state is streamed on
/arm_state for the recording PC.

Modes:  auto      run all waypoints with --delay seconds between them (default)
        manual    press ENTER before each waypoint
        joystick  press B on the joystick before each waypoint

Usage (full instructions: GUIDE.md at the repository root):
  rosrun arm_control_magnet execute_magnet_path_straight.py [--trajectory FILE] [--mode auto] [--motor]
"""

import sys
import os
import argparse
import time
from math import degrees

import numpy as np
import rospy
import moveit_commander
from sensor_msgs.msg import Joy

from utils.config_io import load_config, resolve_path, load_trajectory
from utils.kinematics import link_back_straight, link_pose_msg
from utils.init_pose import initialise_pose
from utils.arm_state_publisher import ArmStatePublisher


class TrajectoryExecutor:
    """Load a trajectory and execute it on the Franka FR3."""

    def __init__(self, cfg, traj_file, mode='auto', point_delay=0.1, motor_on=False,
                 use_cartesian=False, cartesian_fraction_thresh=0.9, chunk_size=50):
        self.cfg = cfg
        self.mode = mode
        self.point_delay = point_delay
        self.use_cartesian = use_cartesian
        self.cartesian_thresh = cartesian_fraction_thresh
        self.chunk_size = chunk_size
        self.traj_file = traj_file

        robot_cfg = cfg["robot"]
        self.ee_length = cfg["end_effector"]["length_mm"] * 1e-3
        self.arm_acc = robot_cfg["arm_acc"]
        self.thresh_singular = robot_cfg["singularity_thresholds"]
        self.planning_repeat = robot_cfg["planning_repeat"]

        # --- trajectory: (6, N) -> (N, 6) ---
        self.traj = load_trajectory(traj_file).T
        print(f"Loaded trajectory: {traj_file} ({self.traj.shape[0]} waypoints)")

        # --- ROS / MoveIt ---
        rospy.init_node('execute_magnet_path_straight', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(robot_cfg["group_name"])
        self.group.set_max_velocity_scaling_factor(robot_cfg["vel_scale"])
        self.group.set_max_acceleration_scaling_factor(robot_cfg["acc_scale"])
        print(f"Planning frame: {self.group.get_planning_frame()}")
        print(f"End effector  : {self.group.get_end_effector_link()}")
        print(f"Tool          : straight, {cfg['end_effector']['length_mm']:.1f} mm")

        # --- live state stream for the recording PC ---
        self.state_pub = ArmStatePublisher(cfg, source='execute_magnet_path_straight')
        self.state_pub.set_mode('init')

        # --- motor (optional; starts rotating when the trajectory starts) ---
        self.stepper = None
        if motor_on:
            from utils.arduino_control import Stepper_servo
            motor_cfg = cfg["motor"]
            self.stepper = Stepper_servo(serial_no=motor_cfg["serial_no"],
                                         servo_vel_step=motor_cfg["step_rotation"])
            self.stepper.servo_stop()
        print(f"Motor         : {'ENABLED' if motor_on else 'disabled'}")

        # --- joystick (joystick mode only) ---
        self.joy_proceed = False
        if mode == 'joystick':
            self.joy_b_index = cfg["joystick"]["button_B"]
            self.joy_sub = rospy.Subscriber("joy", Joy, self._joy_cb, queue_size=10)

        # --- initial pose ---
        self.joints_at_init = cfg["init"].get("joints_init") or None
        self.base_pose = np.array(initialise_pose(self.group, cfg, self._move_tip, self.state_pub))
        self.joints_at_init = self.group.get_current_joint_values()
        if cfg["init"].get("mode", "hand_guide") == "fixed_pose":
            input(">>> Press ENTER to start the trajectory ... ")

    # ------------------------------------------------------------------

    def _joy_cb(self, msg):
        if msg.buttons[self.joy_b_index] == 1:
            self.joy_proceed = True

    def _tip_to_pose_msg(self, tip):
        return link_pose_msg(link_back_straight(*tip, self.ee_length), self.arm_acc)

    def _move_tip(self, tip):
        """Move the magnet tip to tip=[x,y,z,r,p,y] with state publishing.  Returns True on success."""
        self.state_pub.set_target(tip=tip)
        ok = self._plan_and_execute_single(self._tip_to_pose_msg(tip))
        self.state_pub.motion_done(ok)
        return ok

    def _plan_and_execute_single(self, pose_msg):
        """Plan + execute one panda_link8 pose target with singularity detection.  Returns True on success."""
        group = self.group
        try:
            group.clear_path_constraints()
            group.set_pose_target(pose_msg)
            _, traj, _, _ = group.plan()

            jc = np.array(group.get_current_joint_values())
            jn = np.array(traj.joint_trajectory.points[-1].positions)
            err = np.sum(np.abs(jc - jn)[:-1])

            if err > self.thresh_singular[0]:
                print(f"Singularity (err={err:.2f}) - replanning ...")
                trajects, errors = [], []
                for thresh, rep in zip(self.thresh_singular, self.planning_repeat):
                    trajects, errors = [], []
                    for _ in range(rep):
                        group.set_pose_target(pose_msg)
                        _, t, _, _ = group.plan()
                        jn2 = np.array(t.joint_trajectory.points[-1].positions)
                        trajects.append(t)
                        errors.append(np.sum(np.abs(jc - jn2)[:-1]))
                    best = int(np.argmin(errors))
                    if errors[best] < thresh:
                        traj = trajects[best]
                        print(f"Replan OK (err={errors[best]:.2f} < {thresh:.2f})")
                        break
                else:
                    print("WARNING: all replans exceeded the thresholds - executing the best one")
                    traj = trajects[int(np.argmin(errors))]

            ok = group.execute(traj, wait=True)
            group.stop()
            return bool(ok)
        except Exception as e:
            print(f"ERROR planning/executing: {e}")
            return False

    # ------------------------------------------------------------------

    def execute(self):
        N = self.traj.shape[0]
        bp = self.base_pose
        tips = [list(bp + self.traj[i, :]) for i in range(N)]

        if self.stepper is not None:
            self.stepper.servo_rotate(1000)
        self.state_pub.set_mode(self.mode)

        done = False
        if self.use_cartesian:
            print(f"Trying Cartesian path planning for all {N} waypoints ...")
            done = self._try_cartesian(tips)
            if not done:
                print("Cartesian planning failed - falling back to point-by-point.")

        if not done:
            print(f"Executing point-by-point ({self.mode} mode) ...")
            for i in range(N):
                if rospy.is_shutdown():
                    break
                if self.mode == 'manual':
                    input(f"  [{i + 1}/{N}] Press ENTER to proceed ...")
                elif self.mode == 'joystick':
                    print(f"  [{i + 1}/{N}] Press B on the joystick to proceed ...")
                    self.joy_proceed = False
                    while not self.joy_proceed and not rospy.is_shutdown():
                        rospy.sleep(0.05)

                self.state_pub.set_waypoint(i, N, self.traj_file)
                t0 = time.time()
                ok = self._move_tip(tips[i])
                print(f"  [{i + 1}/{N}] tip xyz=({tips[i][0]:.4f}, {tips[i][1]:.4f}, {tips[i][2]:.4f}) "
                      f"yaw={degrees(tips[i][5]):.1f} deg | {time.time() - t0:.2f} s  {'OK' if ok else 'FAIL'}")
                if self.mode == 'auto' and self.point_delay > 0:
                    rospy.sleep(self.point_delay)

        # back to the joints of the initial pose
        print("Returning to the initial pose ...")
        self.state_pub.set_mode('return')
        self.state_pub.set_waypoint(None, N, self.traj_file)
        self.state_pub.set_target(joints=self.joints_at_init)
        ok = self.group.go(self.joints_at_init, wait=True)
        self.group.stop()
        self.state_pub.motion_done(ok)
        if self.stepper is not None:
            self.stepper.servo_stop()
        self.state_pub.set_mode('done')
        print("Trajectory execution complete!")

    def _try_cartesian(self, tips, eef_step=0.002, jump_thresh=0.0):
        """Execute via compute_cartesian_path in chunks.  Returns True if the whole trajectory ran."""
        group = self.group
        N = len(tips)
        idx = 0
        while idx < N:
            end = min(idx + self.chunk_size, N)
            waypoints = [self._tip_to_pose_msg(t) for t in tips[idx:end]]
            trajectory, fraction = group.compute_cartesian_path(waypoints, eef_step, jump_thresh)
            print(f"  Cartesian chunk [{idx + 1}..{end}] / {N}: fraction = {fraction * 100:.1f} %")
            if fraction < self.cartesian_thresh:
                return False
            self.state_pub.set_waypoint(end - 1, N, self.traj_file)
            self.state_pub.set_target(tip=tips[end - 1])
            ok = group.execute(trajectory, wait=True)
            group.stop()
            self.state_pub.motion_done(ok)
            idx = end
        return True

    def stop_motor(self):
        if self.stepper is not None:
            self.stepper.servo_stop()


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(
        description="Execute a magnet trajectory with the STRAIGHT end effector. See GUIDE.md.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--config', type=str, default=os.path.join(here, 'config_straight.json'),
                        help='JSON configuration file')
    parser.add_argument('--trajectory', type=str, default=None,
                        help='Trajectory .txt (5xN or 6xN); default: config trajectory.execute_file')
    parser.add_argument('--mode', type=str, default='auto', choices=['auto', 'manual', 'joystick'],
                        help='Execution mode')
    parser.add_argument('--delay', type=float, default=0.1,
                        help='Delay between waypoints in auto mode (s)')
    parser.add_argument('--motor', action='store_true',
                        help='Rotate the magnet motor during the trajectory (overrides motor.enable=false)')
    parser.add_argument('--cartesian', action='store_true',
                        help='Try MoveIt Cartesian path planning first')
    parser.add_argument('--cartesian_thresh', type=float, default=0.9,
                        help='Minimum fraction for a Cartesian chunk to be accepted')
    parser.add_argument('--chunk_size', type=int, default=50,
                        help='Waypoints per Cartesian planning chunk')
    args = parser.parse_args(rospy.myargv()[1:])

    cfg = load_config(args.config)
    traj_file = resolve_path(cfg, args.trajectory or cfg["trajectory"]["execute_file"])
    motor_on = args.motor or cfg["motor"].get("enable", False)

    executor = None
    try:
        executor = TrajectoryExecutor(cfg, traj_file, mode=args.mode, point_delay=args.delay,
                                      motor_on=motor_on, use_cartesian=args.cartesian,
                                      cartesian_fraction_thresh=args.cartesian_thresh,
                                      chunk_size=args.chunk_size)
        executor.execute()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    finally:
        if executor is not None:
            executor.stop_motor()


if __name__ == '__main__':
    main()
