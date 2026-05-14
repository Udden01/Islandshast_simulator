
# Stewart–Gough-plattformens geometri, invers kinematik och aktuatoranalys.
# 6-6 konfiguration: 6 basleder och 6 plattformsleder arrangerade i alternerande par på cirklar med olika radier.

import numpy as np
from dataclasses import dataclass

from config import PLATFORM
from motion_reconstruction import Trajectory6DOF


# ─── Geometry ────────────────────────────────────────────────────────────────

def _base_platform_angles(delta_b: float) -> np.ndarray:
    # Beräkna infästningsvinklar för basplattformen med +1-förskjutning.

    # Vinklar för k ∈ [0, 5]:
    #     θ_k = (2π/3)·floor((k+1)/2) + (-1)^k · (Δ_b/2)
    
    # Parametrar
    # ----------
    # delta_b : float
    #     Halvvinkelseparation (i radianer) mellan benparen.
    
    # Returns
    # -------
    # angles : (6,) Array med 6 vinklar i radianer.

    k = np.arange(6)
    return (2 * np.pi / 3) * np.floor((k + 1) / 2) + ((-1) ** k) * (delta_b / 2)


def _upper_platform_angles(delta_p: float, offset: float = np.pi / 3) -> np.ndarray:
    # Beräkna infästningsvinklar för plattformen.

    # Vinklar för k ∈ [0, 5]:
    #     θ_k = (2π/3)·floor(k/2) - (-1)^k · (Δ_p/2) + offset
    
    # Parametrar
    # ----------
    # delta_p : float
    #     Halvvinkelseparation (i radianer) mellan benparen.
    # offset : float
    #     Angulär förskjutning (standard π/3).
    
    # Returns
    # -------
    # angles : (6,) Array med 6 vinklar i radianer.
    k = np.arange(6)
    return (2 * np.pi / 3) * np.floor(k / 2) - ((-1) ** k) * (delta_p / 2) + offset


def _joint_positions_from_angles(radius: float, angles: np.ndarray) -> np.ndarray:
    # beräknar 6 joint(led) positioner från vinklar på en cirkel.
    
    # Parametrar
    # ----------
    # radius : float
    #     Radie i meter på cirkeln som lederna placeras på.
    # angles : (6,) array
    #     Vinklar i radianer för varje led.
    
    # Returns
    # -------
    # joints : (6, 3) array av [x, y, z] positioner.
    joints = np.zeros((6, 3))
    for i in range(6):
        joints[i, 0] = radius * np.cos(angles[i])
        joints[i, 1] = radius * np.sin(angles[i])
    return joints


def _joint_positions_on_circle(radius: float, pair_half_angle_deg: float,
                                offset_deg: float = 0.0) -> np.ndarray:
    # beräknar 6 joint positioner arrangerade i 3 par på en cirkel.

    # Varje par av joints(leder) är separerade av 2 × pair_half_angle_deg.
    # Par är utplacerade med 120° i mellan. En valfri "angular offset"(vinkel försjutning) roterar
    # hela mönstret.

    # Returns
    # -------
    # joints : (6, 3) array av [x, y, z] positioner (z=0 for base, z=0 for
    #          platform in local frame).
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
    # Stewart–plattform geometri och invers kinematik

    base_joints: np.ndarray       # (6, 3) bas infästningspunkter i världets koordinater
    platform_joints_local: np.ndarray  # (6, 3) plattformens infästningspunkter i plattformens lokala koordinater
    neutral_height: float         # meter, höjden i neutral position (surge=0, sway=0, heave=0, roll=0, pitch=0, yaw=0)

    @classmethod
    def from_config(cls) -> "StewartPlatform":
        #Skapar en platform baserat på konfigurationsparametrar från config.py.
        # För basen används +1-förskjutning
        delta_b = np.radians(PLATFORM.base_pair_half_angle_deg)# konvertera från grader till radianer
        base_angles = _base_platform_angles(delta_b)# beräkna basvinklarna med +1-förskjutning
        base = _joint_positions_from_angles(PLATFORM.base_radius_m, base_angles)# beräkna baslederna positioner på en cirkel med given radie och vinklar
        
        # För plattformen används standardförskjutning (π/3) som i _upper_platform_angles.
        delta_p = np.radians(PLATFORM.platform_pair_half_angle_deg)# konvertera från grader till radianer
        platform_offset = np.radians(PLATFORM.platform_rotation_deg)# konvertera plattformens rotationsförskjutning från grader till radianer
        platform_angles = _upper_platform_angles(delta_p, offset=platform_offset)# beräkna plattformens vinklar med given halvvinkelseparation och rotationsförskjutning
        plat = _joint_positions_from_angles(PLATFORM.platform_radius_m, platform_angles)# beräkna plattformens leder positioner i det lokala koordinatsystemet (z=0) på en cirkel med given radie och vinklar
        
        return cls(
            base_joints=base,
            platform_joints_local=plat,
            neutral_height=PLATFORM.neutral_height_m,
        )

    # ── Rotation matrix ──────────────────────────────────────────────────

    @staticmethod
    def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        # rotations matris R = Rz(yaw) · Ry(pitch) · Rx(roll)
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        

        return np.array([
            [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
            [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
            [-sp,      cp * sr,                  cp * cr],
        ])

    # Geometri framåt: beräkna plattformens ledpositioner(joints) i världets koordinater för en given pose (surge, sway, heave, roll, pitch, yaw).

    def platform_joints_world(self, surge: float, sway: float, heave: float,
                               roll: float, pitch: float, yaw: float) -> np.ndarray:
        # beräknar world-frame positioner för plattformens leder baserat på den lokala konfigurationen och den givna pose(position).
        # Translationen är relativ till neutral position (0, 0, neutral_height).

        # Rotation och translation
        R = self.rotation_matrix(roll, pitch, yaw)# beräkna rotationsmatrisen från roll, pitch, yaw
        t = np.array([surge, sway, self.neutral_height + heave])# beräkna translationen i world frame baserat på surge, sway, heave och neutral height

        # Transform each platform joint
        world = np.zeros((6, 3))# transformera varje plattformens led från lokal till världets koordinater genom att rotera och sedan translatera
        for i in range(6):
            world[i] = t + R @ self.platform_joints_local[i]
        return world

    # Inverse kinematrik
    # För en given pose (surge, sway, heave, roll, pitch, yaw) beräknar leg vectors och leg lengths.

    def leg_vectors(self, surge: float, sway: float, heave: float,
                     roll: float, pitch: float, yaw: float) -> np.ndarray:
        # beräknar de 6 leg (ben) vektorerna (platform_joint − base_joint) för en given pose.
        # Returns 
        # -------
        # legs : (6, 3) vektorer från basen till plattformens leder(joints).

        # Först beräkna plattformens ledpositioner i världets koordinater, sedan subtrahera baslederna.
        pj = self.platform_joints_world(surge, sway, heave, roll, pitch, yaw)
        return pj - self.base_joints

    def leg_lengths(self, surge: float, sway: float, heave: float,
                     roll: float, pitch: float, yaw: float) -> np.ndarray:
        # beräknar de 6 ben(leg) längderna för en givenpose genom att ta normerna leg vektorerna. 
        # Returns
        # -------   
        # lengths : (6,) array med längder i meter.

        legs = self.leg_vectors(surge, sway, heave, roll, pitch, yaw)
        return np.linalg.norm(legs, axis=1)

    @property
    def neutral_leg_length(self) -> float:
        # Beräknar benlängden i neutral position (surge=0, sway=0, heave=0, roll=0, pitch=0, yaw=0).
        return self.leg_lengths(0, 0, 0, 0, 0, 0)[0]

    # För varje sample i en 6-DOF-trajectory, beräkna leg lengths, velocities, accelerations, och uppskattade krafter.

    def compute_trajectory_ik(self, traj: Trajectory6DOF) -> "IKResult":
        # kör invers kinematik för varje sample i traj och beräkna leg lengths, velocities, accelerations, och uppskattade krafter.
        # Returns 
        # -------
        # IKResult : en dataklass som innehåller arrays för leg lengths, velocities, accelerations, och forces för varje leg (ben) över tid.

        # För enkelhetens skull kan krafter uppskattas som F = m/6 × (g + a_leg) per ben, där m är total massan av plattformen och a_leg är benets acceleration.
        N = len(traj.time) # antal samples i traj
        dt = 1.0 / traj.sample_rate # tidssteg mellan samples, används för att beräkna velocities och accelerations via central differences.
    
        lengths = np.zeros((N, 6))# array för att lagra benlängder över tid
        platform_joints = np.zeros((N, 6, 3))# array för att lagra plattformens ledpositioner i världets koordinater över tid

        for i in range(N):
            pj = self.platform_joints_world(
                traj.surge[i], traj.sway[i], traj.heave[i],
                traj.roll[i], traj.pitch[i], traj.yaw[i],
            )# beräknar plattformens ledpositioner i världets koordinater för varje sample i traj
            platform_joints[i] = pj
            lengths[i] = np.linalg.norm(pj - self.base_joints, axis=1)

        #hastigheter och accelerationer genom central differences: v[i] ≈ (x[i+1] - x[i-1]) / (2*dt) och a[i] ≈ (v[i+1] - v[i-1]) / (2*dt)
        velocities = np.gradient(lengths, dt, axis=0)# beräkna benhastigheter över tid
        accelerations = np.gradient(velocities, dt, axis=0)# beräkna benaccelerationer över tid

        #simplifierad kraftuppskattning: F = m/6 × (g + a_leg) per ben, där m är total massan av plattformen och a_leg är benets acceleration.
        mass_per_leg = PLATFORM.total_mass_kg / 6.0# uppskatta kraften på varje ben baserat på dess acceleration och tyngdkraften
        forces = mass_per_leg * (9.81 + accelerations)  # 9.81 m/s² är tyngdaccelerationen, väldigt grov uppskattning som inte tar hänsyn till dynamiska effekter eller benens riktning.

        return IKResult(
            time=traj.time,
            leg_lengths=lengths,
            leg_velocities=velocities,
            leg_accelerations=accelerations,
            leg_forces=forces,
            platform_joints=platform_joints,
            base_joints=self.base_joints.copy(),
            sample_rate=traj.sample_rate,
        )# returnera en IKResult dataklass som innehåller alla beräknade arrays och information.


@dataclass
class IKResult:
    #resultatet från att köra invers kinematik över en trajectory, inklusive benlängder, hastigheter, accelerationer, och uppskattade krafter över tid.
    time: np.ndarray             # (N,) sekunder
    leg_lengths: np.ndarray      # (N, 6) meter
    leg_velocities: np.ndarray   # (N, 6) m/s
    leg_accelerations: np.ndarray  # (N, 6) m/s²
    leg_forces: np.ndarray       # (N, 6) Newtons (grov uppskattning)
    platform_joints: np.ndarray  # (N, 6, 3) world positioner för plattformens leder över tid
    base_joints: np.ndarray      # (6, 3) fixerade base(bas) positioner
    sample_rate: float

    def print_actuator_summary(self):
        """Print a table of actuator requirements."""
        #Printer en tabell som sammanfattar aktuatorernas krav
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
            L = self.leg_lengths[:, leg] * 1000  # till mm
            V = np.abs(self.leg_velocities[:, leg]) * 1000  # till mm/s
            F = np.abs(self.leg_forces[:, leg])  # N

            print(f"║  {leg + 1:>3d}  {neutral[leg] * 1000:>8.1f}  "
                  f"{L.min():>8.1f}  {L.max():>8.1f}  "
                  f"{L.max() - L.min():>8.1f}  "
                  f"{V.max():>8.1f}  {F.max():>8.1f}  ║")

        print("╠══════════════════════════════════════════════════════════════════╣")

        # Maximum värden 
        all_stroke = (self.leg_lengths.max(axis=0) - self.leg_lengths.min(axis=0)) * 1000
        all_vel = np.abs(self.leg_velocities).max(axis=0) * 1000
        all_force = np.abs(self.leg_forces).max(axis=0)

        print(f"║  {'MAX':>3s}  {'':>8s}  {'':>8s}  {'':>8s}  "
              f"{all_stroke.max():>8.1f}  "
              f"{all_vel.max():>8.1f}  {all_force.max():>8.1f}  ║")
        print("╚══════════════════════════════════════════════════════════════════╝")
        print()
