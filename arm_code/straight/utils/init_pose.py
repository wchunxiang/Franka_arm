"""
init_pose.py  –  Initial magnet-tip pose for the straight end effector

config_straight.json -> init.mode:
  "hand_guide"  (default) the arm is moved by hand (Desk, brakes open), the user
                presses ENTER and the current pose becomes the initial pose.
  "fixed_pose"  the arm goes to init.joints_init, then the magnet tip is moved
                to init.fixed_tip_pose.
"""

from math import radians, degrees

import numpy as np

from utils.kinematics import get_link8_euler, link_forward_straight, link_back_straight


def _fmt(tip):
    return (f"xyz=({tip[0]:.4f}, {tip[1]:.4f}, {tip[2]:.4f}) m  "
            f"rpy=({degrees(tip[3]):.1f}, {degrees(tip[4]):.1f}, {degrees(tip[5]):.1f}) deg")


def read_tip_pose(group, length):
    """Current magnet-tip pose [x, y, z, roll, pitch, yaw] from MoveIt."""
    link = get_link8_euler(group)
    tip = link_forward_straight(*link, length)

    # consistency check: tip -> link8 must give back the read pose
    lb = link_back_straight(*tip, length)
    err_pos = np.linalg.norm(np.array(link[:3]) - np.array(lb[:3]))
    if err_pos > 1e-6:
        print(f"WARNING: forward/back kinematics mismatch ({err_pos:.2e} m) - check end_effector.length_mm")
    return tip


def initialise_pose(group, cfg, move_tip_fn, state_pub=None):
    """Bring the arm to its initial pose and return the magnet-tip pose (6 floats).

    move_tip_fn(tip) must move the magnet tip to tip=[x,y,z,r,p,y] and return
    True on success (the controllers pass their own move function).
    """
    init_cfg = cfg["init"]
    length = cfg["end_effector"]["length_mm"] * 1e-3
    mode = init_cfg.get("mode", "hand_guide")

    if mode == "fixed_pose":
        joints = init_cfg.get("joints_init") or []
        if joints:
            print("Moving to init.joints_init ...")
            if state_pub is not None:
                state_pub.set_target(joints=joints)
            ok = group.go(joints, wait=True)
            group.stop()
            if state_pub is not None:
                state_pub.motion_done(ok)
            if not ok:
                print("WARNING: could not reach init.joints_init")

        fp = init_cfg["fixed_tip_pose"]
        tip = [fp["x_m"], fp["y_m"], fp["z_m"],
               radians(fp["roll_deg"]), radians(fp["pitch_deg"]), radians(fp["yaw_deg"])]
        print(f"Moving magnet tip to init.fixed_tip_pose: {_fmt(tip)}")
        if move_tip_fn(tip):
            print("Initial pose reached.")
            return tip
        print("WARNING: fixed_tip_pose not reached - using the current pose instead.")
        tip = read_tip_pose(group, length)
        print(f"Initial magnet-tip pose: {_fmt(tip)}")
        return tip

    if mode != "hand_guide":
        print(f"WARNING: unknown init.mode '{mode}', using hand_guide")

    print("=" * 60)
    print("  HAND-GUIDE MODE")
    print("  Move the arm by hand to the desired starting pose.")
    print("  Then press ENTER here to capture it as the initial pose.")
    print("=" * 60)
    input(">>> Press ENTER to capture the current pose ... ")
    tip = read_tip_pose(group, length)
    print(f"Initial magnet-tip pose: {_fmt(tip)}")
    return tip
