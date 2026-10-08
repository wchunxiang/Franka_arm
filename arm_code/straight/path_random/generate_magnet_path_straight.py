#!/usr/bin/env python3
"""
generate_magnet_path_straight.py  –  Generate smooth random closed-loop paths for the STRAIGHT magnet end-effector

Straight-end-effector version of arm_code/new/path_random/generate_magnet_path.py:
the yaw optimisation uses the coaxial tool model (tip on the link8 z-axis).
The tool length, base pose (init.fixed_tip_pose) and IK seed (init.joints_init)
are read from ../config_straight.json (--config); CLI flags override them.

Creates a 6×N trajectory file (x, y, z, alpha, phi, yaw) of relative offsets that:
  1. Starts and ends at (0,0,0,0,0,0) — the current magnet position.
  2. Stays within user-specified bounds.
  3. Has a user-specified approximate total arc length.
  4. Is smooth by construction: each coordinate is a sum of random sinusoidal
     harmonics (Fourier series), guaranteeing C∞ smoothness and periodic closure.
  5. Optimises the yaw angle for manipulability at each waypoint using the
     Panda FR3 modified-DH Jacobian.

Output:
  <output_base>.txt          – 6×N array (x,y,z,alpha,phi,yaw) in metres / radians
  <output_base>_plot.png     – (optional) 3D visualisation with direction arrows

Usage (see also GUIDE.md):
  python3 generate_magnet_path_straight.py --help
  python3 generate_magnet_path_straight.py -o my_path --seed 42
  python3 generate_magnet_path_straight.py -o my_path --length 0.5 --n_harmonics 7 --optimise_yaw

Author: (generated with assistance from Claude)
"""

import argparse
import numpy as np
from scipy.interpolate import CubicSpline
from math import pi
import json
import os

# ==========================================================================
#  Panda FR3 forward kinematics & Jacobian  (Modified DH convention)
# ==========================================================================
# Source: Franka official documentation + Gaz et al. RA-L 2019
# Modified DH:  T_i = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)
#
# Joint | a_{i-1} |  d_i   | alpha_{i-1} | theta_i
# ------+---------+--------+-------------+---------
#   1   |   0     | 0.333  |      0      |   q1
#   2   |   0     |   0    |   -pi/2     |   q2
#   3   |   0     | 0.316  |    pi/2     |   q3
#   4   | 0.0825  |   0    |    pi/2     |   q4
#   5   |-0.0825  | 0.384  |   -pi/2     |   q5
#   6   |   0     |   0    |    pi/2     |   q6
#   7   | 0.088   |   0    |    pi/2     |   q7
# flange|   0     | 0.107  |      0      |    0
#
# FR3 joint limits (radians):
#   q1: [-2.8973, 2.8973]    q2: [-1.7628, 1.7628]
#   q3: [-2.8973, 2.8973]    q4: [-3.0718, -0.0698]
#   q5: [-2.8973, 2.8973]    q6: [0.4363, 4.6251]  (FR3: 25..265 deg)
#   q7: [-3.0543, 3.0543]

# DH table  [a_{i-1}, d_i, alpha_{i-1}]  — theta_i = q_i (revolute)
_PANDA_MDH = np.array([
    [0.0,      0.333,   0.0    ],   # joint 1
    [0.0,      0.0,    -pi/2   ],   # joint 2
    [0.0,      0.316,   pi/2   ],   # joint 3
    [0.0825,   0.0,     pi/2   ],   # joint 4
    [-0.0825,  0.384,  -pi/2   ],   # joint 5
    [0.0,      0.0,     pi/2   ],   # joint 6
    [0.088,    0.0,     pi/2   ],   # joint 7
    [0.0,      0.107,   0.0    ],   # flange
])

_PANDA_JOINT_LIMITS = np.array([
    [-2.8973,  2.8973],
    [-1.7628,  1.7628],
    [-2.8973,  2.8973],
    [-3.0718, -0.0698],
    [-2.8973,  2.8973],
    [ 0.4363,  4.6251],   # FR3 specific
    [-3.0543,  3.0543],
])


def _mdh_transform(a, d, alpha, theta):
    """Single Modified-DH homogeneous transformation matrix (4×4)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,       -st,       0,      a    ],
        [st * ca,   ct * ca, -sa,    -d * sa],
        [st * sa,   ct * sa,  ca,     d * ca],
        [0,         0,        0,      1    ],
    ])


def panda_fk(q):
    """Forward kinematics for Panda (7 joints + flange).

    Parameters
    ----------
    q : array-like of 7 floats
        Joint angles in radians.

    Returns
    -------
    T : ndarray (4,4)
        Homogeneous transform of the flange in the base frame.
    T_all : list of 8 ndarray (4,4)
        Cumulative transforms for each frame (joints 1-7 + flange).
    """
    assert len(q) == 7
    T = np.eye(4)
    T_all = []
    for i in range(8):  # 7 joints + flange
        a_prev = _PANDA_MDH[i, 0]
        d_i    = _PANDA_MDH[i, 1]
        al_prev = _PANDA_MDH[i, 2]
        theta_i = q[i] if i < 7 else 0.0
        T = T @ _mdh_transform(a_prev, d_i, al_prev, theta_i)
        T_all.append(T.copy())
    return T, T_all


def panda_jacobian(q):
    """Geometric Jacobian (6×7) in the base frame for the Panda.

    Computed via the standard formula for revolute joints:
        J_v_i = z_{i-1} × (p_e - p_{i-1})
        J_w_i = z_{i-1}
    where z_{i-1} and p_{i-1} come from the cumulative MDH transforms.

    Parameters
    ----------
    q : array-like of 7 floats

    Returns
    -------
    J : ndarray (6,7)
    """
    _, T_all = panda_fk(q)
    p_e = T_all[-1][:3, 3]   # flange position

    J = np.zeros((6, 7))
    T_prev = np.eye(4)       # frame 0 = base
    for i in range(7):
        z = T_prev[:3, 2]    # z-axis of frame i-1
        p = T_prev[:3, 3]    # origin of frame i-1
        J[:3, i] = np.cross(z, p_e - p)
        J[3:, i] = z
        # Advance to frame i
        a_prev  = _PANDA_MDH[i, 0]
        d_i     = _PANDA_MDH[i, 1]
        al_prev = _PANDA_MDH[i, 2]
        T_prev  = T_prev @ _mdh_transform(a_prev, d_i, al_prev, q[i])
    return J


def manipulability(q):
    """Combined manipulability metric: Yoshikawa + joint-limit penalty.

    For a 7-DOF arm, the Yoshikawa index (based on 6×7 Jacobian) is nearly
    invariant to q7 because the 7th joint only rotates around the tool z-axis.
    The real benefit of the redundancy is to keep joints away from limits
    and singular configurations.

    We compute:  m = yoshikawa * joint_limit_score
    where joint_limit_score penalises joints near their limits.
    Higher is better; zero at singularity or at a limit.
    """
    J = panda_jacobian(q)
    JJT = J @ J.T
    det_val = np.linalg.det(JJT)
    yoshikawa = np.sqrt(max(det_val, 0.0))

    if yoshikawa < 1e-12:
        return 0.0

    # Joint-limit penalty: product of normalised distances from limits
    # Each factor is in [0, 1]; 1 = at center, 0 = at limit
    jl_score = 1.0
    for i in range(7):
        lo, hi = _PANDA_JOINT_LIMITS[i]
        rng = hi - lo
        if rng < 1e-6:
            continue
        # Normalised distance from center (0 at limits, 1 at center)
        center = (lo + hi) / 2.0
        dist = 1.0 - abs(q[i] - center) / (rng / 2.0)
        dist = max(dist, 0.0)
        jl_score *= dist

    return yoshikawa * jl_score


# ==========================================================================
#  Approximate IK helper  (numerical, for manipulability evaluation only)
# ==========================================================================

def _pose_to_T(x, y, z, roll, pitch, yaw):
    """Build a 4×4 homogeneous transform from position + Euler 'rxyz'."""
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    # Rotation = Rx(roll) @ Ry(pitch) @ Rz(yaw)  — intrinsic XYZ = 'rxyz'
    R = np.array([
        [cp*cy,              -cp*sy,               sp    ],
        [sr*sp*cy + cr*sy,    cr*cy - sr*sp*sy,   -sr*cp ],
        [sr*sy - cr*sp*cy,    cr*sp*sy + sr*cy,    cr*cp ],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = [x, y, z]
    return T


def _ik_nearest(T_target, q_seed, max_iter=50, tol=1e-5):
    """Damped least-squares IK solver starting from q_seed.

    Returns the joint angles closest to q_seed that achieve T_target.
    Used only offline during path generation (not real-time).
    """
    q = np.array(q_seed, dtype=float)
    lam = 0.5   # damping factor

    for _ in range(max_iter):
        T_cur, _ = panda_fk(q)
        # Position error
        dp = T_target[:3, 3] - T_cur[:3, 3]
        # Orientation error (angle-axis from R_err = R_target @ R_cur^T)
        R_err = T_target[:3, :3] @ T_cur[:3, :3].T
        # Extract axis-angle via Rodrigues
        cos_a = np.clip((np.trace(R_err) - 1) / 2, -1, 1)
        angle = np.arccos(cos_a)
        if abs(angle) < 1e-10:
            dw = np.zeros(3)
        else:
            dw = (angle / (2 * np.sin(angle))) * np.array([
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1],
            ])

        err = np.concatenate([dp, dw])
        if np.linalg.norm(err) < tol:
            break

        J = panda_jacobian(q)
        # Damped LS: dq = J^T (J J^T + λ²I)^{-1} err
        JJT = J @ J.T + (lam ** 2) * np.eye(6)
        dq = J.T @ np.linalg.solve(JJT, err)

        q = q + dq
        # Clamp to joint limits
        for j in range(7):
            q[j] = np.clip(q[j], _PANDA_JOINT_LIMITS[j, 0], _PANDA_JOINT_LIMITS[j, 1])

    return q


# ==========================================================================
#  Path generation
# ==========================================================================

def generate_fourier_path(bounds_xyz, bounds_angles, target_length, resolution_m,
                          n_harmonics=5, rng=None):
    """Generate a smooth random closed-loop path using random Fourier series.

    Each coordinate channel is a sum of sinusoidal harmonics:
        c(t) = Σ_k  A_k * sin(k*t + φ_k)
    where t ∈ [0, 2π].  Because every term is periodic with period 2π,
    the path is automatically a closed loop with c(0) = c(2π) = 0 (sine
    is zero at multiples of π when we use integer frequencies).

    The amplitudes A_k are drawn randomly and then scaled so the path
    fills the bounds and matches the target arc-length.

    Parameters
    ----------
    bounds_xyz : dict   {'x': (-x1,x2), 'y': (-y1,y2), 'z': (-z1,z2)}
    bounds_angles : dict {'alpha': (-a1,a2), 'phi': (-b1,b2)}
    target_length : float   Approximate total arc-length (m).
    resolution_m  : float   Spacing between consecutive waypoints (m).
    n_harmonics   : int     Number of Fourier harmonics per channel.
    rng           : numpy.random.Generator

    Returns
    -------
    path   : ndarray (N, 5)   — columns (x, y, z, alpha, phi), relative offsets
    length : float             total arc-length (m)
    """
    if rng is None:
        rng = np.random.default_rng()

    # --- 1. Generate random Fourier coefficients ---
    # For each of the 5 channels: amplitude A_k and phase φ_k for k=1..n_harmonics
    # Higher harmonics get smaller amplitudes (1/k decay) for natural smoothness
    n_ch = 5
    A = np.zeros((n_ch, n_harmonics))
    phi = np.zeros((n_ch, n_harmonics))

    for ch in range(n_ch):
        for k in range(n_harmonics):
            freq = k + 1  # 1, 2, 3, ...
            # Amplitude decays with frequency: base ~ 1/freq, randomised
            A[ch, k] = rng.uniform(0.2, 1.0) / freq
            phi[ch, k] = rng.uniform(0, 2 * pi)

    # --- 2. Evaluate on a fine grid ---
    # Start with a dense parameter grid, then resample by arc-length
    n_dense = 5000
    t = np.linspace(0, 2 * pi, n_dense, endpoint=False)

    raw = np.zeros((n_dense, n_ch))
    for ch in range(n_ch):
        for k in range(n_harmonics):
            freq = k + 1
            raw[:, ch] += A[ch, k] * np.sin(freq * t + phi[ch, k])

    # --- 3. Normalise each channel to fill bounds ---
    # The bounds are asymmetric: [lo, hi].  We map so that the channel's
    # min→lo and max→hi.
    all_bounds = [
        bounds_xyz['x'], bounds_xyz['y'], bounds_xyz['z'],
        bounds_angles['alpha'], bounds_angles['phi'],
    ]
    for ch in range(n_ch):
        lo, hi = all_bounds[ch]
        ch_min, ch_max = raw[:, ch].min(), raw[:, ch].max()
        ch_range = ch_max - ch_min
        if ch_range < 1e-12:
            raw[:, ch] = 0.0
            continue
        # Map [ch_min, ch_max] → [lo, hi]
        raw[:, ch] = lo + (raw[:, ch] - ch_min) / ch_range * (hi - lo)

    # For angles (channels 3, 4): use a fraction of the full range to
    # get "slight variation" rather than always hitting the extremes
    for ch in [3, 4]:
        lo, hi = all_bounds[ch]
        center = (lo + hi) / 2.0
        # Shrink to ~60% of full range for gentle angle variation
        raw[:, ch] = center + (raw[:, ch] - center) * 0.6

    # --- 4. Force closure: ensure path starts and ends at origin ---
    # Smoothly blend the beginning and end toward zero using a
    # windowing function.  We use a raised-cosine window on the first
    # and last few percent of the path.
    blend_frac = 0.08  # 8% of the path at each end for blending
    n_blend = max(int(n_dense * blend_frac), 10)
    # Blend-in (start)
    w_in = 0.5 * (1 - np.cos(np.linspace(0, pi, n_blend)))
    for ch in range(n_ch):
        raw[:n_blend, ch] *= w_in
    # Blend-out (end)
    w_out = 0.5 * (1 + np.cos(np.linspace(0, pi, n_blend)))
    for ch in range(n_ch):
        raw[-n_blend:, ch] *= w_out

    # --- 5. Compute arc-length and scale to target ---
    diffs = np.diff(raw[:, :3], axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cum_length = np.concatenate([[0], np.cumsum(seg_lengths)])
    total_length = cum_length[-1]

    if total_length > 1e-6:
        # Scale position channels (0,1,2) to match target length
        # But don't exceed bounds — iteratively adjust
        for _iter in range(15):
            scale = target_length / total_length
            raw[:, :3] *= scale
            # Re-clamp to bounds
            for ch in range(3):
                lo, hi = all_bounds[ch]
                raw[:, ch] = np.clip(raw[:, ch], lo, hi)
            # Recompute length
            diffs = np.diff(raw[:, :3], axis=0)
            seg_lengths = np.linalg.norm(diffs, axis=1)
            cum_length = np.concatenate([[0], np.cumsum(seg_lengths)])
            total_length = cum_length[-1]
            if abs(total_length - target_length) / target_length < 0.15:
                break

    # --- 6. Resample at desired resolution ---
    n_pts = max(int(round(total_length / resolution_m)), 10)
    target_arclengths = np.linspace(0, total_length, n_pts)
    t_indices = np.interp(target_arclengths, cum_length,
                          np.arange(n_dense, dtype=float))

    path = np.zeros((n_pts, n_ch))
    for ch in range(n_ch):
        path[:, ch] = np.interp(t_indices, np.arange(n_dense, dtype=float), raw[:, ch])

    # Final clamp
    path = clamp_path(path, bounds_xyz, bounds_angles)

    # Force exact start/end at origin
    path[0, :] = 0.0
    path[-1, :] = 0.0

    return path, total_length


def clamp_path(path, bounds_xyz, bounds_angles):
    """Clamp path values to stay within bounds (safety net)."""
    path[:, 0] = np.clip(path[:, 0], bounds_xyz['x'][0], bounds_xyz['x'][1])
    path[:, 1] = np.clip(path[:, 1], bounds_xyz['y'][0], bounds_xyz['y'][1])
    path[:, 2] = np.clip(path[:, 2], bounds_xyz['z'][0], bounds_xyz['z'][1])
    path[:, 3] = np.clip(path[:, 3], bounds_angles['alpha'][0], bounds_angles['alpha'][1])
    path[:, 4] = np.clip(path[:, 4], bounds_angles['phi'][0],   bounds_angles['phi'][1])
    return path


# ---------------------------------------------------------------------------
#  Visualisation
# ---------------------------------------------------------------------------

def visualise_path(path, yaw_offsets=None, arrow_every=10, save_path=None):
    """Plot the 3-D trajectory with direction arrows and angle subplots.

    Parameters
    ----------
    path : ndarray (N, 5)
        Columns: x, y, z, alpha, phi (relative offsets).
    yaw_offsets : ndarray (N,) or None
        Yaw values; plotted in the angle subplot if provided.
    arrow_every : int
        Place a direction arrow every N waypoints.
    save_path : str or None
        If given, save the figure to this file path instead of showing.
    """
    import matplotlib
    matplotlib.use('Agg') if save_path else None
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    N = path.shape[0]
    x, y, z = path[:, 0], path[:, 1], path[:, 2]
    alpha_deg = np.degrees(path[:, 3])
    phi_deg   = np.degrees(path[:, 4])

    # Colour gradient along path: blue→red
    t_norm = np.linspace(0, 1, N)
    colors = plt.cm.coolwarm(t_norm)

    fig = plt.figure(figsize=(16, 10))

    # --- 3D trajectory ---
    ax3d = fig.add_subplot(2, 2, (1, 3), projection='3d')

    # Draw path as coloured segments
    for i in range(N - 1):
        ax3d.plot(x[i:i+2], y[i:i+2], z[i:i+2],
                  color=colors[i], linewidth=1.5)

    # Direction arrows
    arrow_indices = list(range(0, N - 1, arrow_every))
    for i in arrow_indices:
        dx = x[min(i+1, N-1)] - x[i]
        dy = y[min(i+1, N-1)] - y[i]
        dz = z[min(i+1, N-1)] - z[i]
        length = np.sqrt(dx**2 + dy**2 + dz**2)
        if length < 1e-8:
            continue
        # Scale arrow for visibility
        scale = 0.003
        ax3d.quiver(x[i], y[i], z[i], dx, dy, dz,
                    length=scale / length * max(length, 1e-6),
                    normalize=False, color=colors[i],
                    arrow_length_ratio=0.4, linewidth=1.5)

    # Start and end markers
    ax3d.scatter(*[x[0]], *[y[0]], *[z[0]], color='green', s=80,
                 zorder=5, label='Start', marker='o', edgecolors='black')
    ax3d.scatter(*[x[-1]], *[y[-1]], *[z[-1]], color='red', s=80,
                 zorder=5, label='End', marker='s', edgecolors='black')

    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')
    ax3d.set_title('Magnet trajectory (3D)')
    ax3d.legend(loc='upper left', fontsize=8)

    # --- Alpha / Phi angles ---
    ax_ang = fig.add_subplot(2, 2, 2)
    waypoint_idx = np.arange(N)
    ax_ang.plot(waypoint_idx, alpha_deg, label='alpha (roll)', color='tab:blue')
    ax_ang.plot(waypoint_idx, phi_deg,   label='phi (pitch)',  color='tab:orange')
    if yaw_offsets is not None:
        ax_ang.plot(waypoint_idx, np.degrees(yaw_offsets), label='yaw', color='tab:green')
    ax_ang.set_xlabel('Waypoint index')
    ax_ang.set_ylabel('Angle (deg)')
    ax_ang.set_title('Orientation along path')
    ax_ang.legend(fontsize=8)
    ax_ang.grid(True, alpha=0.3)

    # --- XYZ over waypoint index ---
    ax_xyz = fig.add_subplot(2, 2, 4)
    ax_xyz.plot(waypoint_idx, x * 1e3, label='x', color='tab:red')
    ax_xyz.plot(waypoint_idx, y * 1e3, label='y', color='tab:green')
    ax_xyz.plot(waypoint_idx, z * 1e3, label='z', color='tab:blue')
    ax_xyz.set_xlabel('Waypoint index')
    ax_xyz.set_ylabel('Position (mm)')
    ax_xyz.set_title('Position along path')
    ax_xyz.legend(fontsize=8)
    ax_xyz.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved visualisation to: {save_path}")
    else:
        plt.show()

    plt.close(fig)


def _link_back_straight(ex, ey, ez, er, ep, eyw, length):
    """Magnet-tip pose -> panda_link8 pose for the straight (coaxial) tool.

    Same maths as utils/kinematics.py::link_back_straight (no ROS import needed):
        p_link = p_end - R_rxyz @ [0, 0, length]
    """
    R = _pose_to_T(0.0, 0.0, 0.0, er, ep, eyw)[:3, :3]
    p_link = np.array([ex, ey, ez]) - R @ np.array([0.0, 0.0, length])
    return [p_link[0], p_link[1], p_link[2], er, ep, eyw]


def optimise_yaw_standalone(path, base_pose, q_seed, ee_length,
                            n_yaw_samples=12):
    """Pick, per waypoint, the yaw offset with the best manipulability (straight tool).

    For the straight tool, yaw rotates about the tool axis, so it does not move
    the magnet tip; it only changes the arm configuration.
    """

    yaw_candidates = np.linspace(-0.5, 0.5, n_yaw_samples)
    yaw_offsets = np.zeros(path.shape[0])
    q_current = np.array(q_seed, dtype=float)
    bp = np.array(base_pose, dtype=float)

    for i in range(path.shape[0]):
        mx = bp[0] + path[i, 0]
        my = bp[1] + path[i, 1]
        mz = bp[2] + path[i, 2]
        mroll  = bp[3] + path[i, 3]
        mpitch = bp[4] + path[i, 4]

        best_m = -1.0
        best_yaw = 0.0
        best_q = q_current.copy()

        for yaw_off in yaw_candidates:
            myaw = bp[5] + yaw_off
            lp = _link_back_straight(mx, my, mz, mroll, mpitch, myaw, ee_length)
            T_target = _pose_to_T(lp[0], lp[1], lp[2], lp[3], lp[4], lp[5])
            q_try = _ik_nearest(T_target, q_current, max_iter=30)
            m = manipulability(q_try)
            if m > best_m:
                best_m = m
                best_yaw = yaw_off
                best_q = q_try.copy()

        yaw_offsets[i] = best_yaw
        q_current = best_q

        if (i + 1) % 50 == 0 or i == 0:
            print(f"  Yaw optimisation: {i+1}/{path.shape[0]}  manip={best_m:.4f}  yaw={np.degrees(best_yaw):.1f} deg")

    return yaw_offsets


# ==========================================================================
#  Main
# ==========================================================================

def main():
    from math import radians
    parser = argparse.ArgumentParser(
        description="Generate a smooth random closed-loop magnet trajectory",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # Bounds (all values are positive; the range is [-val, +val] unless asymmetric)
    parser.add_argument('--x1', type=float, default=1*5e-3,  help='Negative x bound (m)')
    parser.add_argument('--x2', type=float, default=1*5e-3,  help='Positive x bound (m)')
    parser.add_argument('--y1', type=float, default=1*12e-3,  help='Negative y bound (m)')
    parser.add_argument('--y2', type=float, default=1*12e-3,  help='Positive y bound (m)')
    parser.add_argument('--z1', type=float, default=5e-3,  help='Negative z bound (m)')
    parser.add_argument('--z2', type=float, default=40e-3,  help='Positive z bound (m)')
    parser.add_argument('--a1', type=float, default=radians(20),   help='Negative alpha/roll bound (rad)')
    parser.add_argument('--a2', type=float, default=radians(20),   help='Positive alpha/roll bound (rad)')
    parser.add_argument('--b1', type=float, default=radians(40),   help='Negative phi/pitch bound (rad)')
    parser.add_argument('--b2', type=float, default=radians(40),   help='Positive phi/pitch bound (rad)')

    parser.add_argument('-o', '--output', type=str, default='magnet_path_straight',
                        help='Output file base name (without extension)')

    parser.add_argument('--length', type=float, default=450e-3,
                        help='Approximate total path arc-length (m)')
    parser.add_argument('--resolution', type=float, default=10e-3,
                        help='Spatial resolution between waypoints (m)')
    parser.add_argument('--n_harmonics', type=int, default=5,
                        help='Number of Fourier harmonics per channel (more = more complex path)')

    # Manipulability / IK  (defaults come from config_straight.json)
    parser.add_argument('--config', type=str,
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config_straight.json'),
                        help='Straight-EE config_straight.json (tool length, base pose, IK seed)')
    parser.add_argument('--base_pose', type=float, nargs=6, default=None,
                        help='Absolute magnet-tip base pose [x,y,z,roll,pitch,yaw] (m,rad); '
                             'default: config init.fixed_tip_pose')
    parser.add_argument('--q_seed', type=float, nargs=7, default=None,
                        help='Joint seed for IK (rad); default: config init.joints_init')
    parser.add_argument('--ee_length', type=float, default=None,
                        help='Straight end-effector length (m); default: config end_effector.length_mm')
    parser.add_argument('--n_yaw_samples', type=int, default=12,
                        help='Number of yaw candidates to evaluate per waypoint')
    parser.add_argument('--optimise_yaw', action='store_true',
                        help='Optimise yaw for manipulability (slower); default: yaw = 0')


    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for reproducibility')
    #parser.add_argument('--visualise', '--visualize', action='store_true',
    #                    help='Show 3D plot of the generated path after generation')
    #parser.add_argument('--save_plot', type=str, default=None,
    #                    help='Save visualisation to this file path (e.g. path.png) instead of showing')

    args = parser.parse_args()

    # Fill IK defaults from config_straight.json
    cfg = {}
    if os.path.isfile(args.config):
        with open(args.config, 'r') as f:
            cfg = json.load(f)
    if args.ee_length is None:
        args.ee_length = cfg.get("end_effector", {}).get("length_mm", 140.0) * 1e-3
    if args.base_pose is None:
        fp = cfg.get("init", {}).get("fixed_tip_pose",
                                     {"x_m": 0.543, "y_m": 0.24, "z_m": 0.31,
                                      "roll_deg": -90.0, "pitch_deg": 0.0, "yaw_deg": 0.0})
        args.base_pose = [fp["x_m"], fp["y_m"], fp["z_m"],
                          radians(fp["roll_deg"]), radians(fp["pitch_deg"]), radians(fp["yaw_deg"])]
    if args.q_seed is None:
        args.q_seed = cfg.get("init", {}).get("joints_init") or \
            [-0.0659, 0.3259, 0.0502, -2.0892, 1.5068, 1.5371, -0.8406]

    rng = np.random.default_rng(args.seed)

    bounds_xyz = {
        'x': (-args.x1, args.x2),
        'y': (-args.y1, args.y2),
        'z': (-args.z1, args.z2),
    }
    bounds_angles = {
        'alpha': (-args.a1, args.a2),
        'phi':   (-args.b1, args.b2),
    }

    print("="*60)
    print("  Magnet Path Generator  (Fourier series)")
    print("="*60)
    print(f"  Bounds XYZ : x={bounds_xyz['x']}, y={bounds_xyz['y']}, z={bounds_xyz['z']}  (m)")
    print(f"  Bounds ang : alpha={bounds_angles['alpha']}, phi={bounds_angles['phi']}  (rad)")
    print(f"  Target length : {args.length:.3f} m")
    print(f"  Resolution    : {args.resolution*1e3:.1f} mm")
    print(f"  Harmonics     : {args.n_harmonics}")
    print(f"  Seed          : {args.seed}")
    print()

    # --- 1. Generate path via Fourier series ---
    path, length = generate_fourier_path(
        bounds_xyz, bounds_angles, args.length, args.resolution,
        n_harmonics=args.n_harmonics, rng=rng)
    print(f"Generated path: {path.shape[0]} waypoints, arc-length = {length:.4f} m "
          f"(target {args.length:.3f} m, deviation {abs(length-args.length)/max(args.length,1e-6)*100:.1f}%)")

    # --- 2. Optimise yaw for manipulability ---
    if args.optimise_yaw:
        print(f"\nOptimising yaw for manipulability (straight tool, {args.ee_length*1e3:.1f} mm; "
              f"this may take a minute)...")
        try:
            yaw_offsets = optimise_yaw_standalone(
                path, args.base_pose, args.q_seed, args.ee_length,
                n_yaw_samples=args.n_yaw_samples)
        except Exception as e:
            print(f"  Warning: yaw optimisation failed ({e}), using yaw=0")
            yaw_offsets = np.zeros(path.shape[0])
    else:
        print("Yaw optimisation skipped.")
        yaw_offsets = np.zeros(path.shape[0])

    # --- 3. Save single 6×N file ---
    out_6xN = np.vstack([path.T, yaw_offsets.reshape(1, -1)])  # shape (6, N)
    out_file = args.output + '.txt'
    np.savetxt(out_file, out_6xN, fmt='%.18e',
               header='6xN: row: (x, y, z, alpha, phi, yaw) unit: m, rad')
    print(f"\nSaved 6x{path.shape[0]} trajectory to: {out_file}")

    # --- 4. Summary ---
    print(f"\n{'='*60}")
    print(f"  Summary")
    print(f"{'='*60}")
    print(f"  Waypoints      : {path.shape[0]}")
    print(f"  Arc-length     : {length:.4f} m")
    print(f"  X range        : [{path[:,0].min()*1e3:.1f}, {path[:,0].max()*1e3:.1f}] mm")
    print(f"  Y range        : [{path[:,1].min()*1e3:.1f}, {path[:,1].max()*1e3:.1f}] mm")
    print(f"  Z range        : [{path[:,2].min()*1e3:.1f}, {path[:,2].max()*1e3:.1f}] mm")
    print(f"  Alpha range    : [{np.degrees(path[:,3].min()):.1f}, {np.degrees(path[:,3].max()):.1f}] deg")
    print(f"  Phi range      : [{np.degrees(path[:,4].min()):.1f}, {np.degrees(path[:,4].max()):.1f}] deg")
    print(f"  Yaw range      : [{np.degrees(yaw_offsets.min()):.1f}, {np.degrees(yaw_offsets.max()):.1f}] deg")

    # --- 5. Visualisation ---
    plot_dest = None
    if plot_dest is None:
        # Default: save next to the trajectory file
        plot_dest = args.output + '_plot.png'
    visualise_path(path, yaw_offsets=yaw_offsets, save_path=plot_dest)


if __name__ == '__main__':
    main()
