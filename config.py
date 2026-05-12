"""
Configuration for the horse-riding Stewart platform simulator.
All tuneable parameters in one place.
"""

import numpy as np
from dataclasses import dataclass, field


# ─── Data selection ──────────────────────────────────────────────────────────

HORSE = "Baldur"  # "Albin" or "Baldur" or "Sinus" or "Sigge" or "Sam"

# Use a specific straight-section window index (0-6) or None for full recording
SEGMENT_INDEX: int | None = 4 #4

# Serial port for the Arduino. Set this to e.g. "COM5" on Windows or "/dev/ttyACM0" on Linux.
# Leave as None to auto-select the only available serial device when possible.
SERIAL_PORT: str | None = None

# Clap sync markers (seconds from recording start)
CLAP_MARKERS = {
    "Albin": 260.22,
    "Baldur": 241.74,
    "Sigge": 0.0,
    "Sinus": 0.0,
    "Sam": 0.0,
}

# Pre-annotated straight-riding segments [start, end] in seconds
STRAIGHT_WINDOWS = {
    "Baldur": np.array([
        [120, 125],
        [135, 141],
        [151, 157],
        [167, 171],
        [181, 187],
        [196, 203],
        [211, 219],
    ]),
    "Albin": np.array([
        [126, 133],
        [143, 149],
        [160, 167],
        [177, 185],
        [194, 201],
        [210, 219],
        [228, 234],
    ]),
    "Sinus": np.array([
        [20, 90]
    ]),
    "Sigge": np.array([
        [10, 25]
    ]),
}


# ─── Resampling ──────────────────────────────────────────────────────────────

RESAMPLE_RATE_HZ = 100.0  # uniform sample rate for all signals


# ─── Motion filtering ───────────────────────────────────────────────────────

@dataclass
class FilterConfig:
    highpass_hz: float = 0.3    # removes drift / DC from integration
    lowpass_hz: float = 4.0    # removes sensor noise & fast vibrations 
    # 4.0 hz kanske filterar bort för mycket. stride brukar ligga 3-4 hz.
    filter_order: int = 4       # Butterworth order (applied zero-phase)


FILTER = FilterConfig()


# ─── Stewart platform workspace limits ───────────────────────────────────────

@dataclass
class WorkspaceLimits:
    """Maximum allowable motion for saturation (soft-clamp)."""
    surge_m: float = 0.080      # ±80 mm
    sway_m: float = 0.080       # ±80 mm
    heave_m: float = 0.080      # ±80 mm
    roll_deg: float = 15.0      # ±15°
    pitch_deg: float = 15.0     # ±15°
    yaw_deg: float = 5.0        # ±5°

    @property
    def as_array(self) -> np.ndarray:
        """Return limits as [surge, sway, heave, roll, pitch, yaw] in SI (m, rad)."""
        return np.array([
            self.surge_m,
            self.sway_m,
            self.heave_m,
            np.radians(self.roll_deg),
            np.radians(self.pitch_deg),
            np.radians(self.yaw_deg),
        ])


WORKSPACE = WorkspaceLimits()


# ─── Stewart platform geometry ───────────────────────────────────────────────

@dataclass
class PlatformGeometry:
    """6-6 Stewart–Gough platform geometry."""
    base_radius_m: float = 0.21        # radius of base joint circle
    platform_radius_m: float = 0.125    # radius of platform joint circle
    neutral_height_m: float = 0.28     # height of platform above base at home, home position is centered at 50 mm actuator length
    base_pair_half_angle_deg: float = 5.5   # half-angle within each pair of base joints
    platform_pair_half_angle_deg: float = 9.0  # half-angle within each pair of platform joints
    # platform_rotation_deg: float = 60.0  # rotation offset of platform pairs vs base pairs
    platform_rotation_deg: float = 60.0  # rotation offset of platform pairs vs base pairs

    platform_mass_kg: float = 10.0     # platform structure mass
    rider_mass_kg: float = 5.0        # rider mass
    saddle_mass_kg: float = 5.0       # saddle + mounting hardware

    @property
    def total_mass_kg(self) -> float:
        return self.platform_mass_kg + self.rider_mass_kg + self.saddle_mass_kg


PLATFORM = PlatformGeometry()


# ─── Visualization ───────────────────────────────────────────────────────────

@dataclass
class VizConfig:
    playback_speed: float = 1.0      # 1.0 = real time
    animation_fps: int = 30           # frames per second for animation
    trail_seconds: float = 1.0        # trailing path duration


VIZ = VizConfig()
