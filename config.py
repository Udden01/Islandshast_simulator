"""
Configuration for the horse-riding Stewart platform simulator.
All tuneable parameters in one place.
"""

import numpy as np
from dataclasses import dataclass, field

HORSE = "Baldur"  # "Albin" eller "Baldur" eller "Sinus" eller "Sigge" eller "Sam" välj häst

# vilken raksträcka? none kör hela
SEGMENT_INDEX: int | None = 4 #4

# Serial port "COM5" Windows."/dev/ttyACM0" Linux.
# None välj automatiskt
SERIAL_PORT: str | None = None

# för att synka ljudet med videon, används inte i simuleringen
CLAP_MARKERS = {
    "Albin": 260.22,
    "Baldur": 241.74,
    "Sigge": 0.0,
    "Sinus": 0.0,
    "Sam": 0.0,
}

# definera raksträckor 
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



RESAMPLE_RATE_HZ = 100.0  # samplerate för mätdatan som vi resamplar till

@dataclass
class FilterConfig:
    highpass_hz: float = 0.3    #hp tar bort långsamma rörelse
    lowpass_hz: float = 4.0    # lp sensorbrus
  
    filter_order: int = 4       # hur brant filter


FILTER = FilterConfig()


# max translation och rotation vi tillåter för plattformen
# appliceras med en mjuk mättnadsfunktion 

@dataclass
class WorkspaceLimits:
    
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


# ─── Stewartplattformens geometri ───────────────────────────────────────────────

@dataclass
class PlatformGeometry:
    """6-6 Stewart–Gough platform geometry."""
    base_radius_m: float = 0.21         # Basradie för den cirkel där de sex nedre lederna är fixerade i golvet
    platform_radius_m: float = 0.125    # Plattformsradie för den cirkel där de sex övre lederna är fixerade i den rörliga plattformen
    neutral_height_m: float = 0.28     # Vertikalt avstånd mellan bas och plattform i neutralläge, där hemmaläget är centrerat vid 50 mm
    base_pair_half_angle_deg: float = 5.5   # Halva det interna vinkelparet mellan lederna i basen
    platform_pair_half_angle_deg: float = 9.0  #  Halva det interna vinkelparet mellan lederna i plattformen
    # platform_rotation_deg: float = 60.0  # Rotationsförskjutning mellan plattformens och basens ledpar
    platform_rotation_deg: float = 60.0  # Rotationsförskjutning mellan plattformens och basens ledpar

    #användes för att dimensionera komponenter
    platform_mass_kg: float = 10.0     # platformens tyngd
    rider_mass_kg: float = 5.0        # personens tyngd
    saddle_mass_kg: float = 5.0       # sadelns tyngd

    @property
    def total_mass_kg(self) -> float:
        return self.platform_mass_kg + self.rider_mass_kg + self.saddle_mass_kg #returerar bara totalvikten för dimensionering


PLATFORM = PlatformGeometry()


# ───  konstanter för animeringen ───────────────────────────────────────────────────────────

@dataclass
class VizConfig:
    playback_speed: float = 1.0      # återspelnings hastighet, 1 är normalhastighet
    animation_fps: int = 30           # fps för animationen
    trail_seconds: float = 1.0        # hur många sekunder av rörelsehistoriken ska visas i animationen


VIZ = VizConfig()
