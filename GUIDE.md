# Franka straight end effector + video recorder: usage guide

## 1. Overview

| PC | IP | Role | Runs |
|---|---|---|---|
| **Recording PC** | 134.105.56.32 | **ROS master** | `roscore`, `video_record/arm_video_recorder.py` |
| **Arm PC** (RT kernel) | 134.105.57.59 | ROS slave | franka_control + MoveIt, joystick, `arm_code/straight/*` |

```
Arm PC: controller / path executor --(/arm_state, 30 Hz, std_msgs/String JSON)--> Recording PC: recorder
                                                                                  |-> <stamp>.avi  (video + overlay)
Epiphan grabber -----------------------------------------------------------------> |-> <stamp>.csv  (1 row per frame)
```

- `arm_code/straight/` is for the **straight (coaxial) end effector**. It is a self-contained catkin package `arm_control_magnet_straight`.
- `arm_code/new` (L-shape) and `arm_code/old` are unchanged.

## 2. One-time setup

**Recording PC `~/.bashrc`:**
```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=134.105.56.32
export ROS_IP=134.105.56.32          # no spaces around '='
```
```bash
pip3 install opencv-python imageio imageio-ffmpeg     # + ROS Noetic (ros-noetic-ros-base)
```

**Arm PC `~/.bashrc`:**
```bash
export ROS_MASTER_URI=http://134.105.56.32:11311/
export ROS_HOSTNAME=134.105.57.59
export ROS_IP=134.105.57.59
```
```bash
cd ~/catkin_ws/src && ln -s <repo>/arm_code/straight arm_control_magnet_straight
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

**Network check.** Each PC must `ping` the other. If a firewall is on, allow all traffic from the other PC's IP, because ROS uses random ports.

## 3. Configure (edit the JSON, no code changes)

**Arm PC: `arm_code/straight/config.json`**

| Field | Meaning | Default |
|---|---|---|
| `end_effector.length_mm` | flange → magnet tip distance along the flange z-axis | `140` |
| `init.mode` | `hand_guide` (move by hand, press ENTER) or `fixed_pose` | `hand_guide` |
| `init.joints_init`, `init.fixed_tip_pose` | used by `fixed_pose`: joints first, then the tip pose (m, deg) | old straight setup |
| `motor.enable` | `false`: LB+X/B/Y/A rotate yaw. `true`: those keys drive the magnet motor | `false` |
| `state_publisher.reach_tol_mm / reach_tol_deg` | "reached" tolerance (green vs red) | `1` / `1` |
| `trajectory.tracking_file` | trajectory for joystick tracking mode (Start button) | |
| `trajectory.execute_file` | default trajectory for `execute_magnet_path.py` | |
| `robot.vel_scale / acc_scale` | MoveIt speed and acceleration scaling | `0.12` / `0.03` |

**Recording PC: `video_record/recorder_config.json`**

| Field | Meaning | Default |
|---|---|---|
| `camera.source` | Epiphan device (`<videoN>`; find it with `v4l2-ctl --list-devices`) | `<video4>` |
| `fps` | recording frame rate | `20` |
| `output_folder` | where the .avi/.csv go (relative paths are relative to the JSON) | `../../data_fluoro/` |
| `overlay.anchor`, `overlay.offset_px` | overlay corner and offset (also set live with keys + `s`) | `top_right`, `[20, 20]` |

## 4. Every session, in this order

1. **Recording PC:** `roscore`
2. **Arm PC, Desk:** unlock the brakes. For `hand_guide`, move the arm by hand to the start pose now. Then **activate FCI**.
3. **Arm PC, terminal 1:** `roslaunch arm_control_magnet_straight robot_bringup.launch robot_ip:=172.16.0.2`
4. **Arm PC, terminal 2:** pick one:
   - Joystick control: `rosrun arm_control_magnet_straight franka_magnet_controller.py`
   - Run a path: `rosrun arm_control_magnet_straight execute_magnet_path.py --mode auto --trajectory <file.txt>`
     - `--mode manual`: ENTER before each waypoint.
     - `--mode joystick`: B before each waypoint.
     - `--motor`: rotate the magnet during the path.
5. **Recording PC:** `cd <repo>/video_record && python3 arm_video_recorder.py`
6. **Arm PC, terminal 2:** press **ENTER** to capture the start pose (`fixed_pose`: the arm moves there by itself).
7. **Recording PC:** press **`r`** to start recording, **`r`** again to stop, **`q`** to quit.

> All-in-one alternative: `roslaunch arm_control_magnet_straight franka_magnet_controller.launch` or `execute_path.launch` (`mode:=… trajectory:=…`). Use it only if your roslaunch terminal accepts the ENTER key. Otherwise use steps 3–4.

**New random paths** (no ROS needed): `python3 arm_code/straight/path_random/generate_magnet_path.py -o my_path --seed 1 [--optimise_yaw]`. This writes `my_path.txt` (6×N offsets) and `my_path_plot.png`.

## 5. Joystick (`franka_magnet_controller.py`)

Default steps: 2 mm / 50 mm and 2° / 5° (`step_sizes`). **Start** = tracking mode, **Back** = manual mode.

| Input | Manual mode | Tracking mode |
|---|---|---|
| D-pad ←/→, ↑/↓ | x −/+, y +/− (small) | same, as a correction |
| LB + D-pad | x, y (big) | same |
| RB + D-pad ↑/↓ (→/←) | z +/− small (big) | same |
| X / B | pitch −/+ small | **previous / next waypoint** |
| RB + X / B | pitch −/+ big | pitch correction (small) |
| Y / A | roll −/+ small | – |
| RB + Y / A | roll −/+ big | roll correction (small) |
| LB + X / B *(motor off)* | yaw −/+ small | yaw correction small |
| LB + Y / A *(motor off)* | yaw −/+ big | yaw correction big |
| LB + RB + X/B (Y/A) | yaw small (big) | same |
| *Motor on:* LB + Y / A | motor CW / CCW | same |
| *Motor on:* LB + X / B | motor faster / slower | same |
| *Motor on:* LB + RB | motor stop | same |

In tracking mode, corrections add up until the waypoint changes. Yaw is the rotation about the tool axis: it does not move the tip, so use it to get out of singularities.

## 6. Recorder window

| Overlay colour | State | Meaning |
|---|---|---|
| yellow | `MOVING` | a command is being planned / executed |
| green | `REACHED` | motion done, tip error ≤ tolerance (default 1 mm / 1°) |
| red | `NOT_REACHED` | planning/execution failed or error > tolerance |
| white | `IDLE` | no target commanded yet |
| grey | `NO_DATA` / `NO_POSE` | no /arm_state for > 0.5 s / tf pose unavailable |

- The overlay (state + error, **TIP** and **LINK8** absolute x/y/z in mm and roll/pitch/yaw in deg, mode / waypoint / EE) is **burned into the saved video**.
- The bottom **status line** (`● REC <name> mm:ss N frames` or `NOT RECORDING`, plus the arm message rate) appears **only in the live window**.

| Key | Action |
|---|---|
| `r` | start / stop recording (each start = a new file pair) |
| `i` `j` `k` `l` | move the overlay up / left / down / right (`I J K L` = big steps) |
| `c` | move the overlay to the next corner |
| `s` | save the overlay position to `recorder_config.json` |
| `q` / `Esc` | quit (open files are closed properly) |

## 7. Output files

`<output_folder>/<YYYYmmdd_HHMM_SS>.avi` and **`.csv` with the same name**: one CSV row per video frame. Positions are in m and angles in rad (rxyz Euler) in the robot base frame.

| Columns | Content |
|---|---|
| `frame_idx, t_frame_unix, t_rel_s` | frame number (row *i* = video frame *i*), recording-PC time |
| `state, data_age_s` | reach state; age of the arm message (recording-PC clock) |
| `arm_seq, arm_stamp_unix, source, mode, exec_ok, waypoint, n_waypoints, traj_file` | arm message info (`arm_stamp_unix` = arm-PC clock; `waypoint` = 0-based trajectory column) |
| `tip_*`, `link8_*` | actual magnet-tip and panda_link8 pose (`x_m … yaw_rad`) |
| `target_tip_*` | commanded tip pose (empty for joint moves) |
| `err_pos_mm, err_rot_deg` | actual vs target error (joint moves: max joint error in `err_rot_deg`) |
| `q1_rad … q7_rad` | joint angles |
| `ee_type, ee_length_mm` | end-effector geometry |

When there is no fresh arm data, `state = NO_DATA` and the arm columns are empty. If you need the two PCs' clocks aligned, sync them with `chrony`.

## 8. Quick checks and troubleshooting

| Check / symptom | What to do |
|---|---|
| Is the stream arriving? | Recording PC: `rostopic hz /arm_state` (≈ 30 Hz) and `rostopic echo -n1 /arm_state` |
| Test the recorder without the robot | `roscore`, then `python3 fake_arm_publisher.py`, then `python3 arm_video_recorder.py` (cycles all colours) |
| Test the camera only | `python3 arm_video_recorder.py --no-ros` |
| Grey `NO_DATA` / `ARM: NO DATA` | arm script not running, or a network problem: check `echo $ROS_MASTER_URI $ROS_IP` on both PCs, `ping`, firewall |
| `ROS master not reachable` | start `roscore` on the recording PC first |
| Camera fails to open | wrong `camera.source`; try another `<videoN>` (`v4l2-ctl --list-devices`) |
| Always red after a move | read the arm terminal (planning failed?); check `end_effector.length_mm` and `reach_tol_*` |
| `NO_POSE` | tf `panda_link0 → panda_link8` missing: is `robot_bringup.launch` running? Do `robot.base_frame / flange_frame` match? |
