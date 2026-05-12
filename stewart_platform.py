"""
Stewart–Gough platform geometry, inverse kinematics, and actuator analysis.

6-6 configuration: 6 base joints and 6 platform joints arranged in alternating
pairs on circles of different radii.
"""

import numpy as np
from dataclasses import dataclass

from config import PLATFORM
from motion_reconstruction import Trajectory6DOF


# ─── Geometry ────────────────────────────────────────────────────────────────

def _base_platform_angles(delta_b: float) -> np.ndarray:
    """
    Compute base platform attachment angles with +1 offset.
    
    Angles for k ∈ [0, 5]:
        θ_k = (2π/3)·floor((k+1)/2) + (-1)^k · (Δ_b/2)
    
    Parameters
    ----------
    delta_b : float
        Half-angle separation (radians) between leg pairs.
    
    Returns
    -------
    angles : (6,) array of angles in radians.
    """
    k = np.arange(6)
    return (2 * np.pi / 3) * np.floor((k + 1) / 2) + ((-1) ** k) * (delta_b / 2)


def _upper_platform_angles(delta_p: float, offset: float = np.pi / 3) -> np.ndarray:
    """
    Compute upper platform attachment angles.
    
    Angles for k ∈ [0, 5]:
        θ_k = (2π/3)·floor(k/2) - (-1)^k · (Δ_p/2) + offset
    
    Parameters
    ----------
    delta_p : float
        Half-angle separation (radians) between leg pairs.
    offset : float
        Angular offset (default π/3).
    
    Returns
    -------
    angles : (6,) array of angles in radians.
    """
    k = np.arange(6)
    return (2 * np.pi / 3) * np.floor(k / 2) - ((-1) ** k) * (delta_p / 2) + offset


def _joint_positions_from_angles(radius: float, angles: np.ndarray) -> np.ndarray:
    """
    Compute 6 joint positions from angles on a circle.
    
    Parameters
    ----------
    radius : float
        Radius of the circle.
    angles : (6,) array
        Angles in radians for each joint.
    
    Returns
    -------
    joints : (6, 3) array of [x, y, z] positions.
    """
    joints = np.zeros((6, 3))
    for i in range(6):
        joints[i, 0] = radius * np.cos(angles[i])
        joints[i, 1] = radius * np.sin(angles[i])
    return joints


def _joint_positions_on_circle(radius: float, pair_half_angle_deg: float,
                                offset_deg: float = 0.0) -> np.ndarray:
    """
    Compute 6 joint positions arranged in 3 pairs on a circle.

    Each pair of joints is separated by 2 × pair_half_angle_deg.
    Pairs are spaced 120° apart.  An optional angular offset rotates
    the entire pattern.

    Returns
    -------
    joints : (6, 3) array of [x, y, z] positions (z=0 for base, z=0 for
             platform in local frame).
    """
    joints = np.zeros((6, 3))
    for pair in range(3):
        centre_angle = np.radians(pair * 120.0 + offset_deg)
        half = np.radians(pair_half_angle_deg)
        for side, sign in enumerate([-1, 1]):
            idx = pair * 2 + side
            angle = centre_angle + sign * half
            joints[idx, 0] = radius * np.cos(angle)
            joints[idx, 1] = radius * np.sin(angle)
    return joints


@dataclass
class StewartPlatform:
    """Stewart platform geometry and inverse kinematics."""

    base_joints: np.ndarray       # (6, 3) base attachment points
    platform_joints_local: np.ndarray  # (6, 3) platform attachment points in platform frame
    neutral_height: float         # metres

    @classmethod
    def from_config(cls) -> "StewartPlatform":
        """Create a platform from config.py parameters."""
        # Base platform with +1 offset
        delta_b = np.radians(PLATFORM.base_pair_half_angle_deg)
        base_angles = _base_platform_angles(delta_b)
        base = _joint_positions_from_angles(PLATFORM.base_radius_m, base_angles)
        
        # Upper/platform with standard offset scheme
        delta_p = np.radians(PLATFORM.platform_pair_half_angle_deg)
        platform_offset = np.radians(PLATFORM.platform_rotation_deg)
        platform_angles = _upper_platform_angles(delta_p, offset=platform_offset)
        plat = _joint_positions_from_angles(PLATFORM.platform_radius_m, platform_angles)
        
        return cls(
            base_joints=base,
            platform_joints_local=plat,
            neutral_height=PLATFORM.neutral_height_m,
        )

    # ── Rotation matrix ──────────────────────────────────────────────────

    @staticmethod
    def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Rotation matrix R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        return np.array([
            [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
            [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
            [-sp,      cp * sr,                  cp * cr],
        ])

    # ── Forward geometry (platform joints in world frame) ────────────────

    def platform_joints_world(self, surge: float, sway: float, heave: float,
                               roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Compute world-frame platform joint positions for a given pose.

        Translation is relative to the neutral pose (0,0, neutral_height).
        """
        R = self.rotation_matrix(roll, pitch, yaw)
        t = np.array([surge, sway, self.neutral_height + heave])

        # Transform each platform joint
        world = np.zeros((6, 3))
        for i in range(6):
            world[i] = t + R @ self.platform_joints_local[i]
        return world

    # ── Inverse kinematics ───────────────────────────────────────────────

    def leg_vectors(self, surge: float, sway: float, heave: float,
                     roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        Compute the 6 leg vectors (platform_joint − base_joint).

        Returns
        -------
        legs : (6, 3) vectors from base to platform joints.
        """
        pj = self.platform_joints_world(surge, sway, heave, roll, pitch, yaw)
        return pj - self.base_joints

    def leg_lengths(self, surge: float, sway: float, heave: float,
                     roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Compute the 6 leg lengths for a given pose.  Returns (6,) array."""
        legs = self.leg_vectors(surge, sway, heave, roll, pitch, yaw)
        return np.linalg.norm(legs, axis=1)

    @property
    def neutral_leg_length(self) -> float:
        """Leg length at the home / neutral pose."""
        return self.leg_lengths(0, 0, 0, 0, 0, 0)[0]

    # ── Batch inverse kinematics over a trajectory ───────────────────────

    def compute_trajectory_ik(self, traj: Trajectory6DOF) -> "IKResult":
        """
        Run inverse kinematics for every sample in the trajectory.

        Returns an IKResult with leg lengths, velocities, accelerations, and forces.
        """
        N = len(traj.time)
        dt = 1.0 / traj.sample_rate

        lengths = np.zeros((N, 6))
        platform_joints = np.zeros((N, 6, 3))

        for i in range(N):
            pj = self.platform_joints_world(
                traj.surge[i], traj.sway[i], traj.heave[i],
                traj.roll[i], traj.pitch[i], traj.yaw[i],
            )
            platform_joints[i] = pj
            lengths[i] = np.linalg.norm(pj - self.base_joints, axis=1)

        # Velocities and accelerations via central differences
        velocities = np.gradient(lengths, dt, axis=0)
        accelerations = np.gradient(velocities, dt, axis=0)

        # Simplified force estimation: F = m/6 × (g + a_leg) per leg
        mass_per_leg = PLATFORM.total_mass_kg / 6.0
        forces = mass_per_leg * (9.81 + accelerations)  # very rough estimate

        return IKResult(
            time=traj.time,
            leg_lengths=lengths,
            leg_velocities=velocities,
            leg_accelerations=accelerations,
            leg_forces=forces,
            platform_joints=platform_joints,
            base_joints=self.base_joints.copy(),
            sample_rate=traj.sample_rate,
        )


@dataclass
class IKResult:
    """Results from running inverse kinematics over a trajectory."""
    time: np.ndarray             # (N,) seconds
    leg_lengths: np.ndarray      # (N, 6) metres
    leg_velocities: np.ndarray   # (N, 6) m/s
    leg_accelerations: np.ndarray  # (N, 6) m/s²
    leg_forces: np.ndarray       # (N, 6) Newtons (rough estimate)
    platform_joints: np.ndarray  # (N, 6, 3) world positions
    base_joints: np.ndarray      # (6, 3) fixed base positions
    sample_rate: float

    def print_actuator_summary(self):
        """Print a table of actuator requirements."""
        neutral = np.mean(self.leg_lengths, axis=0)

        print("\n╔══════════════════════════════════════════════════════════════════╗")
        print("║               ACTUATOR REQUIREMENTS SUMMARY                     ║")
        print("╠══════════════════════════════════════════════════════════════════╣")
        print(f"║  {'Leg':>3s}  {'Neutral':>8s}  {'Min':>8s}  {'Max':>8s}  "
              f"{'Stroke':>8s}  {'MaxVel':>8s}  {'MaxFrc':>8s}  ║")
        print(f"║  {'':>3s}  {'(mm)':>8s}  {'(mm)':>8s}  {'(mm)':>8s}  "
              f"{'(mm)':>8s}  {'(mm/s)':>8s}  {'(N)':>8s}  ║")
        print("╠══════════════════════════════════════════════════════════════════╣")

        for leg in range(6):
            L = self.leg_lengths[:, leg] * 1000  # to mm
            V = np.abs(self.leg_velocities[:, leg]) * 1000  # to mm/s
            F = np.abs(self.leg_forces[:, leg])  # N

            print(f"║  {leg + 1:>3d}  {neutral[leg] * 1000:>8.1f}  "
                  f"{L.min():>8.1f}  {L.max():>8.1f}  "
                  f"{L.max() - L.min():>8.1f}  "
                  f"{V.max():>8.1f}  {F.max():>8.1f}  ║")

        print("╠══════════════════════════════════════════════════════════════════╣")

        # Overall maximums
        all_stroke = (self.leg_lengths.max(axis=0) - self.leg_lengths.min(axis=0)) * 1000
        all_vel = np.abs(self.leg_velocities).max(axis=0) * 1000
        all_force = np.abs(self.leg_forces).max(axis=0)

        print(f"║  {'MAX':>3s}  {'':>8s}  {'':>8s}  {'':>8s}  "
              f"{all_stroke.max():>8.1f}  "
              f"{all_vel.max():>8.1f}  {all_force.max():>8.1f}  ║")
        print("╚══════════════════════════════════════════════════════════════════╝")
        print()
