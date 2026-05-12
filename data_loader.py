"""
Load sensor data from exported .mat files.

Expects files produced by export_for_python.m (plain double arrays, no
MATLAB timetable objects).  File naming: Sensor_<location>_<horse>_exported.mat

Key variables in each file:
  acc_time / acc_xyz   — accelerometer  (N,) s, (N,3) m/s²
  ori_time / ori_xyz   — orientation    (N,) s, (N,3) deg [Azimuth, Pitch, Roll]
  ang_time / ang_xyz   — gyroscope      (N,) s, (N,3) rad/s
  mag_time / mag_xyz   — magnetometer   (N,) s, (N,3) µT
  pos_time / pos_data  — GPS            (N,) s, (N,M) various
"""

import numpy as np
from pathlib import Path
from dataclasses import dataclass

from config import RESAMPLE_RATE_HZ


@dataclass
class SensorData:
    """Uniformly-sampled sensor data from a single recording."""
    time: np.ndarray           # (N,) seconds from start, uniform
    acceleration: np.ndarray   # (N, 3) m/s²  — X, Y, Z
    orientation: np.ndarray    # (N, 3) degrees — Azimuth, Pitch, Roll
    angular_velocity: np.ndarray  # (N, 3) rad/s — X, Y, Z
    sample_rate: float         # Hz
    horse: str
    sensor_location: str       # "Back", "Left", "Right"


def load_mat_file(path: str | Path) -> dict:
    """
    Load an exported .mat file (produced by export_for_python.m).
    Returns a dict with keys: acc, ori, ang, mag — each a dict with
    'time' (N,) and 'values' (N,3).
    """
    import scipy.io as sio

    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"MAT file not found: {path}")

    raw = sio.loadmat(str(path), squeeze_me=True)

    result = {}
    pairs = [
        ("Acceleration",    "acc_time", "acc_xyz"),
        ("Orientation",     "ori_time", "ori_xyz"),
        ("AngularVelocity", "ang_time", "ang_xyz"),
        ("MagneticField",   "mag_time", "mag_xyz"),
    ]
    for key, t_key, v_key in pairs:
        if t_key in raw and v_key in raw:
            t = np.asarray(raw[t_key], dtype=float).flatten()
            v = np.asarray(raw[v_key], dtype=float)
            if v.ndim == 1:
                v = v.reshape(-1, 1)
            result[key] = {"time": t, "values": v}

    return result


def load_sensor_data(mat_path: str | Path, horse: str = "",
                     sensor_location: str = "") -> SensorData:
    """
    Load and return uniformly-resampled sensor data from an exported .mat file.

    Parameters
    ----------
    mat_path : path to the *_exported.mat file
    horse : horse name (for metadata)
    sensor_location : sensor placement name (for metadata)
    """
    raw = load_mat_file(mat_path)

    # Find the common time span across all sensors
    t_min = 0.0
    t_max = float("inf")
    for key in ["Acceleration", "Orientation", "AngularVelocity"]:
        if key in raw:
            t = raw[key]["time"]
            t_min = max(t_min, t[0])
            t_max = min(t_max, t[-1])

    # Create uniform time base
    n_samples = int((t_max - t_min) * RESAMPLE_RATE_HZ) + 1
    t_uniform = np.linspace(t_min, t_max, n_samples)

    def resample(key: str, n_cols: int = 3) -> np.ndarray:
        if key not in raw:
            return np.zeros((n_samples, n_cols))
        t_raw = raw[key]["time"]
        v_raw = raw[key]["values"]
        # Ensure at least n_cols columns
        if v_raw.ndim == 1:
            v_raw = v_raw.reshape(-1, 1)
        while v_raw.shape[1] < n_cols:
            v_raw = np.column_stack([v_raw, np.zeros(len(v_raw))])
        v_raw = v_raw[:, :n_cols]

        out = np.zeros((n_samples, n_cols))
        for c in range(n_cols):
            out[:, c] = np.interp(t_uniform, t_raw, v_raw[:, c])
        return out

    return SensorData(
        time=t_uniform,
        acceleration=resample("Acceleration"),
        orientation=resample("Orientation"),
        angular_velocity=resample("AngularVelocity"),
        sample_rate=RESAMPLE_RATE_HZ,
        horse=horse,
        sensor_location=sensor_location,
    )
