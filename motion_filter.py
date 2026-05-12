"""
Filter the reconstructed 6-DOF trajectory so it is physically reproducible
on a Stewart platform.

Two stages:
  1. Bandpass filter — removes integration drift (highpass) and sensor noise /
     vibrations beyond actuator bandwidth (lowpass).
  2. Workspace saturation — smooth-clamps each DOF to the platform limits
     so the inverse kinematics stays within the feasible workspace.
"""

import numpy as np
from scipy.signal import butter, sosfiltfilt
from copy import deepcopy

from config import FILTER, WORKSPACE
from motion_reconstruction import Trajectory6DOF


def _butter_hp_sos(fs: float, f_cutoff: float, order: int) -> np.ndarray:
    """High-pass Butterworth SOS coefficients."""
    nyq = fs / 2.0
    return butter(order, f_cutoff / nyq, btype="high", output="sos")


def _butter_lp_sos(fs: float, f_cutoff: float, order: int) -> np.ndarray:
    """Low-pass Butterworth SOS coefficients."""
    nyq = fs / 2.0
    return butter(order, f_cutoff / nyq, btype="low", output="sos")


# NOTE: We intentionally do NOT use a combined bandpass filter.
# When the HP corner is very low relative to the sample rate (e.g. 0.3 Hz at
# 100 Hz → normalised frequency 0.006), scipy's butter() bandpass design
# becomes numerically ill-conditioned: the polynomial coefficients lose
# precision and the filter gains > 1 near the HP corner, amplifying the signal
# by 2–4× instead of attenuating it.
# Applying HP and LP as two separate passes avoids this entirely.


def _soft_clamp(x: np.ndarray, limit: float) -> np.ndarray:
    """
    Smooth saturation using tanh: limit * tanh(x / limit).

    Properties:
    - For |x| << limit: output ≈ x  (unity gain, no distortion)
    - For |x| >> limit: output → ±limit  (saturation)
    - At |x| == limit: output = limit * tanh(1) ≈ 0.76 * limit

    IMPORTANT: do NOT use a sharpness > 1 multiplier here.
    sharpness=5 would give gain≈5 in the linear region, amplifying the signal
    instead of clamping it.  The standard form (sharpness=1) is correct.
    """
    return limit * np.tanh(x / limit)


def bandpass_filter(trajectory: Trajectory6DOF) -> Trajectory6DOF:
    """Apply zero-phase HP then LP Butterworth filters to all 6 DOF channels."""
    traj = deepcopy(trajectory)
    fs = traj.sample_rate

    sos_hp = _butter_hp_sos(fs, FILTER.highpass_hz, FILTER.filter_order)
    sos_lp = _butter_lp_sos(fs, FILTER.lowpass_hz, FILTER.filter_order)

    pose = traj.pose_array  # (N, 6)
    for col in range(6):
        x = sosfiltfilt(sos_hp, pose[:, col])   # remove drift
        x = sosfiltfilt(sos_lp, x)              # remove high-freq noise
        pose[:, col] = x
    traj.pose_array = pose

    return traj


def saturate_workspace(trajectory: Trajectory6DOF) -> Trajectory6DOF:
    """Soft-clamp each DOF to the Stewart platform workspace limits."""
    traj = deepcopy(trajectory)
    limits = WORKSPACE.as_array  # [surge, sway, heave, roll, pitch, yaw]

    pose = traj.pose_array
    for col in range(6):
        pose[:, col] = _soft_clamp(pose[:, col], limits[col])
    traj.pose_array = pose

    return traj


def filter_trajectory(trajectory: Trajectory6DOF) -> Trajectory6DOF:
    """
    Full filtering pipeline: bandpass → workspace saturation.

    Parameters
    ----------
    trajectory : raw 6-DOF trajectory from motion_reconstruction

    Returns
    -------
    Filtered trajectory safe for Stewart platform inverse kinematics.
    """
    filtered = bandpass_filter(trajectory)
    filtered = saturate_workspace(filtered)
    return filtered
