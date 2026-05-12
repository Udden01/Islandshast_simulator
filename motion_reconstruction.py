"""
Reconstruct 6-DOF saddle motion from phone sensor data.

Produces a trajectory: [surge, sway, heave, roll, pitch, yaw] over time.

- Orientation (roll, pitch, yaw) is taken directly from the phone's fused
  orientation output (the phone already runs an internal Kalman filter).
- Translation (surge, sway, heave) is obtained by rotating the raw
  acceleration into the world frame, subtracting gravity, and double-integrating.
  The inevitable drift is removed by the highpass filter in motion_filter.py.
"""

import numpy as np
from dataclasses import dataclass
from data_loader import SensorData


@dataclass
class Trajectory6DOF:
    """Six degree-of-freedom trajectory over time."""
    time: np.ndarray        # (N,) seconds
    surge: np.ndarray       # (N,) metres — forward/back  (X world)
    sway: np.ndarray        # (N,) metres — left/right    (Y world)
    heave: np.ndarray       # (N,) metres — up/down       (Z world)
    roll: np.ndarray        # (N,) radians — rotation about X
    pitch: np.ndarray       # (N,) radians — rotation about Y
    yaw: np.ndarray         # (N,) radians — rotation about Z
    sample_rate: float      # Hz

    @property
    def pose_array(self) -> np.ndarray:
        """Return (N, 6) array: [surge, sway, heave, roll, pitch, yaw]."""
        return np.column_stack([
            self.surge, self.sway, self.heave,
            self.roll, self.pitch, self.yaw,
        ])

    @pose_array.setter
    def pose_array(self, arr: np.ndarray):
        self.surge = arr[:, 0]
        self.sway = arr[:, 1]
        self.heave = arr[:, 2]
        self.roll = arr[:, 3]
        self.pitch = arr[:, 4]
        self.yaw = arr[:, 5]


# ─── Rotation utilities ─────────────────────────────────────────────────────

def _euler_to_rotation_matrix(azimuth: float, pitch: float, roll: float) -> np.ndarray:
    """
    Build rotation matrix from phone orientation (Euler angles in radians).

    The phone convention (MATLAB Mobile):
      - Azimuth: rotation about Z (yaw)
      - Pitch:   rotation about X (nose up/down)
      - Roll:    rotation about Y (tilt left/right)

    Returns R that transforms body-frame vectors to world-frame:
        v_world = R @ v_body
    """
    caz, saz = np.cos(azimuth), np.sin(azimuth)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)

    # R = Rz(azimuth) · Ry(roll) · Rx(pitch)
    R = np.array([
        [caz * cr - saz * sp * sr,  -saz * cp,  caz * sr + saz * sp * cr],
        [saz * cr + caz * sp * sr,   caz * cp,  saz * sr - caz * sp * cr],
        [-cp * sr,                   sp,         cp * cr],
    ])
    return R


def _rotate_acceleration_to_world(acc_body: np.ndarray,
                                    orientation_deg: np.ndarray) -> np.ndarray:
    """
    Rotate body-frame accelerations into world frame using orientation.

    Parameters
    ----------
    acc_body : (N, 3) acceleration in phone body frame [m/s²]
    orientation_deg : (N, 3) [azimuth, pitch, roll] in degrees

    Returns
    -------
    acc_world : (N, 3) acceleration in world frame [m/s²]
    """
    ori_rad = np.radians(orientation_deg)
    acc_world = np.zeros_like(acc_body)

    for i in range(len(acc_body)):
        R = _euler_to_rotation_matrix(ori_rad[i, 0], ori_rad[i, 1], ori_rad[i, 2])
        acc_world[i] = R @ acc_body[i]

    return acc_world


# ─── Double integration ─────────────────────────────────────────────────────

def _cumulative_trapz(y: np.ndarray, dt: float) -> np.ndarray:
    """Cumulative trapezoidal integration along axis 0."""
    result = np.zeros_like(y)
    for i in range(1, len(y)):
        result[i] = result[i - 1] + 0.5 * (y[i - 1] + y[i]) * dt
    return result

# ─── Kalman filter for 1D position reconstruction ──────────────────────────

def _kalman_1d(acc: np.ndarray, dt: float) -> np.ndarray:
    """
    1D Kalman filter for position reconstruction from acceleration.
    Mimics MATLAB kalman_1D function.
    
    Parameters
    ----------
    acc : (N,) acceleration array in m/s²
    dt : float, time step in seconds
    
    Returns
    -------
    pos : (N,) position in metres
    """
    N = len(acc)
    
    # State: [position, velocity]
    x = np.array([0.0, 0.0])
    
    # Matrices
    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([0.5 * dt**2, dt])
    C = np.array([1.0, 0.0])
    
    # Covariances
    Q = np.array([[1e-4, 0.0], [0.0, 1e-2]])
    R = 1e-2
    P = np.eye(2)
    
    pos = np.zeros(N)
    
    for k in range(N):
        # Predict
        x = A @ x + B * acc[k]
        P = A @ P @ A.T + Q
        
        # Measurement: 0 (drift suppression)
        z = 0.0
        
        # Update
        S = C @ P @ C.T + R
        K = P @ C.T / S
        x = x + K * (z - C @ x)
        P = (np.eye(2) - np.outer(K, C)) @ P
        
        pos[k] = x[0]
    
    return pos

# ──────────────────────────────────────────────────────────────────────────
# translation using sinusoidal signal
# ──────────────────────────────────────────────────────────────────────────

def sinus_translation(time, amplitude, freq, axis):

    x = amplitude * np.sin(2 * np.pi * freq * time)

    surge = np.zeros_like(time)
    sway = np.zeros_like(time)
    heave = np.zeros_like(time)

    #Temp
    surge_freq = freq
    sway_freq = freq
    heave_freq = freq

    phase= 0.0
    surge_phase = np.radians(-180)
    sway_phase = np.radians(-55)
    heave_phase = phase

    if axis == "surge":
        surge = x
    elif axis == "sway":
        sway = x
    elif axis == "heave":
        heave = x
    elif axis == "all":       
        surge = 0.007 * np.sin(2 * np.pi * surge_freq * time+ surge_phase)
        sway = 0.008 * np.sin(2 * np.pi * sway_freq * time + sway_phase)
        heave = 0.011 * np.sin(2 * np.pi * heave_freq * time + heave_phase)
    return surge, sway, heave


# ─── Main reconstruction ────────────────────────────────────────────────────

GRAVITY = np.array([0.0, 0.0, 9.81])  # m/s², world-frame, pointing up


def reconstruct_motion(sensor: SensorData, amplitude: float, freq: float, axis: str) -> Trajectory6DOF:
    """
    Reconstruct 6-DOF saddle trajectory from phone sensor data.

    Parameters
    ----------
    sensor : SensorData from data_loader
    amplitude : float, amplitude of the sinusoidal translation
    freq : float, frequency of the sinusoidal translation
    axis : str, the axis along which to apply the translation ("surge", "sway", or "heave")

    Returns
    -------
    Trajectory6DOF with translations in metres and rotations in radians.
    """
    dt = 1.0 / sensor.sample_rate
    N = len(sensor.time)


    # ── Orientation ──────────────────────────────────────────────────────
    # Phone gives [Azimuth, Pitch, Roll] in degrees.
    # Subtract mean to centre around neutral platform pose.
    ori_deg = sensor.orientation.copy()
    ori_mean = np.mean(ori_deg, axis=0)
    ori_centred_deg = ori_deg - ori_mean

    # # Map to platform convention: roll(X), pitch(Y), yaw(Z)
    # # Phone:  col 0 = Azimuth (yaw), col 1 = Pitch, col 2 = Roll
    roll_rad = np.radians(ori_centred_deg[:, 2])    # Roll
    pitch_rad = np.radians(ori_centred_deg[:, 1])   # Pitch
    yaw_rad = np.radians(ori_centred_deg[:, 0])     # Azimuth → Yaw
    # kalman filter
    # ── Translation via Kalman filter ────────────────────────────────────
    # Rotate acceleration to world frame
    acc_world = _rotate_acceleration_to_world(sensor.acceleration, ori_deg)
    
    # Remove gravity from Z axis
    acc_linear = acc_world.copy()
    acc_linear[:, 2] -= GRAVITY[2]
    
    # Apply Kalman filter to each axis
    surge = _kalman_1d(acc_linear[:, 0], dt)
    sway = _kalman_1d(acc_linear[:, 1], dt)
    heave = _kalman_1d(acc_linear[:, 2], dt)

    return Trajectory6DOF(
        time=sensor.time.copy(),
        surge=surge,
        sway=sway,
        heave=heave,
        roll=roll_rad,
        pitch=pitch_rad,
        yaw=yaw_rad,
        sample_rate=sensor.sample_rate,
    )
