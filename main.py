"""
Horse Riding Simulator — Stewart Platform Motion Analysis

Pipeline:
  1. Load phone sensor data (.mat) from a horse ride
  2. Reconstruct 6-DOF saddle trajectory (translations + rotations)
  3. Filter to Stewart platform workspace (bandpass + saturation)
  4. Compute Stewart platform inverse kinematics (6 leg lengths)
  5. Analyse actuator requirements and display 3D animation

Configuration: edit config.py to change horse, segment, filter, geometry, etc.
"""


from pathlib import Path

import numpy as np

from config import HORSE, SEGMENT_INDEX, STRAIGHT_WINDOWS
from config import SERIAL_PORT
from data_loader import load_sensor_data
from motion_reconstruction import reconstruct_motion, Trajectory6DOF
from motion_filter import filter_trajectory
from stewart_platform import StewartPlatform
from visualization import plot_analysis, animate_platform, show_all
from send_to_arduino import stream_ik_to_arduino, bygg_paket, stream_unit_step_to_arduino
import serial
import time


DATA_DIR = Path(__file__).parent  # .mat filer är projectets root 
amplitud = 0.005   # meter
freq = 0.2          # Hz
axis = "all"      # surge / sway / heave / all

def _slice_trajectory(traj: Trajectory6DOF, t0: float, t1: float) -> Trajectory6DOF: #kan användas för att bara köra en del av ridningen (en raksträcka)
    """Extract a time window from a trajectory, resetting time to start at 0."""
    mask = (traj.time >= t0) & (traj.time <= t1)
    t = traj.time[mask] - traj.time[mask][0]
    return Trajectory6DOF(
        time=t,
        surge=traj.surge[mask],
        sway=traj.sway[mask],
        heave=traj.heave[mask],
        roll=traj.roll[mask],
        pitch=traj.pitch[mask],
        yaw=traj.yaw[mask],
        sample_rate=traj.sample_rate,
    )

# ─────────────────────────────────────────────────────────────
# Blend two segments (C1 continuity)
# ─────────────────────────────────────────────────────────────
def _blend_segments(seg1: Trajectory6DOF, seg2: Trajectory6DOF, blend_samples: int) -> Trajectory6DOF: #blandar 2 segmenter så det blir en mjuk övergång, minska rykicghet emllan segment
    dt = 1.0 / seg1.sample_rate

    def blend_array(a1, a2): #balndar arrayerna för en DOF
        p0 = a1[-1]
        p1 = a2[0]

        v0 = (a1[-1] - a1[-2]) / dt
        v1 = (a2[1] - a2[0]) / dt

        t = np.linspace(0, 1, blend_samples)

        # Cubic Hermite, mjuk och deriverbar övergång som tar hänsyn till både position och hastighet i start och slutpunkterna mellan datan: 
        h00 = 2*t**3 - 3*t**2 + 1
        h10 = t**3 - 2*t**2 + t
        h01 = -2*t**3 + 3*t**2
        h11 = t**3 - t**2

        return h00*p0 + h10*v0*dt + h01*p1 + h11*v1*dt

    time = seg1.time[-1] + np.arange(1, blend_samples + 1) * dt #skapa en tidsvektor för blendningen som börjar efter seg1 och har lika många punkter som blend_samples

    return Trajectory6DOF( #skapa en ny Trajectory6DOF som är blendningen av seg1 och seg2)
        time=time,
        surge=blend_array(seg1.surge, seg2.surge),
        sway=blend_array(seg1.sway, seg2.sway),
        heave=blend_array(seg1.heave, seg2.heave),
        roll=blend_array(seg1.roll, seg2.roll),
        pitch=blend_array(seg1.pitch, seg2.pitch),
        yaw=blend_array(seg1.yaw, seg2.yaw),
        sample_rate=seg1.sample_rate,
    )

def _slice_trajectory_and_stitch(traj, windows, blend_time=0.1): #skär ut segment från traj baserat på windows/raksträckorna, och blanda dem med blend_time för att få en mjuk övergång mellan segmenten
    segments = [] 
    time_offset = 0.0
    dt = 1.0 / traj.sample_rate
    blend_samples = int(blend_time / dt)

    prev_segment = None

    for t0, t1 in windows: #loopa genom alla windows/raksträckor, skär ut segmentet, blanda det med föregående segment om det finns, och lägg till det i listan av segment
        mask = (traj.time >= t0) & (traj.time <= t1)
        t = traj.time[mask]

        t = t - t[0] + time_offset

        segment = Trajectory6DOF(
            time=t,
            surge=traj.surge[mask],
            sway=traj.sway[mask],
            heave=traj.heave[mask],
            roll=traj.roll[mask],
            pitch=traj.pitch[mask],
            yaw=traj.yaw[mask],
            sample_rate=traj.sample_rate,
        )

        if prev_segment is not None: #
            blend = _blend_segments(prev_segment, segment, blend_samples)
            segments.append(blend)
            time_offset = blend.time[-1]

            shift = time_offset - segment.time[0] + dt
            segment = Trajectory6DOF(
                time=segment.time + shift,
                surge=segment.surge,
                sway=segment.sway,
                heave=segment.heave,
                roll=segment.roll,
                pitch=segment.pitch,
                yaw=segment.yaw,
                sample_rate=segment.sample_rate,
            )

        segments.append(segment)
        time_offset = segment.time[-1]
        prev_segment = segment

    return Trajectory6DOF( #stitcha ihop alla segment i listan till en enda Trajectory6DOF, genom att konkatenera deras arrayer. Tidsvektorn kommer att vara kontinuerlig tack vare blendningen och tidsförskjutningen.
        time=np.concatenate([seg.time for seg in segments]),
        surge=np.concatenate([seg.surge for seg in segments]),
        sway=np.concatenate([seg.sway for seg in segments]),
        heave=np.concatenate([seg.heave for seg in segments]),
        roll=np.concatenate([seg.roll for seg in segments]),
        pitch=np.concatenate([seg.pitch for seg in segments]),
        yaw=np.concatenate([seg.yaw for seg in segments]),
        sample_rate=segments[0].sample_rate,
    )
def sinus_translation(time: np.ndarray, amplitude: float, freq: float, axis: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]: #skapar en sinusformad translation i en given axel (surge/sway/heave) för testning av plattformen
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


def main():
        # Send positions + velocities to Arduino
    SEND_TO_ARDUINO = False
    simulering_köras = True
    Sinus_rörelse = False

    # Initialize platform early since both paths need it
    platform = StewartPlatform.from_config()
    print(f"Neutral leg length: {platform.neutral_leg_length * 1000:.1f} mm")
    
    if Sinus_rörelse is False:
        #1. laddar sensordata från .mat filer
        mat_file = DATA_DIR / f"Sensor_Back_{HORSE}_exported.mat"
        print(f"Loading {mat_file.name} ...")
        sensor_full = load_sensor_data(mat_file, horse=HORSE, sensor_location="Back")
        print(f"  Loaded {len(sensor_full.time)} samples, "
            f"{sensor_full.time[-1]:.1f} s, {sensor_full.sample_rate:.0f} Hz")
        # 2. rekonstruerar 6-DOF rörelse från komplettsensordata, med amplitud och frekvensskalning
        # Viktigt! att köra på hela inspelningen för att undvika filtertransienter, 
        # bandpassfiltret behöver flera sekunder på sig att stabilisera sig så korta segment (10s) kan få kraftiga tranisenter som förstör signalen.
        print("Reconstructing 6-DOF motion (full recording) ...")

        raw_full = reconstruct_motion(sensor_full, amplitud ,freq,axis) #

        print("Filtering trajectory for Stewart platform workspace ...")
        filtered_full = filter_trajectory(raw_full)

        # ── 3. Optionally slice to a straight-riding segment for display ─────
        #if semgent_index is not None, loop through all segments, stitch them together.
        windows = STRAIGHT_WINDOWS[HORSE]

        if SEGMENT_INDEX is not None:
            t0, t1 = windows[SEGMENT_INDEX]

            raw_trajectory = _slice_trajectory(raw_full, t0, t1)
            filtered_trajectory = _slice_trajectory(filtered_full, t0, t1)
        else:
            raw_trajectory = _slice_trajectory_and_stitch(raw_full, windows)
            filtered_trajectory = _slice_trajectory_and_stitch(filtered_full, windows)
            # raw_trajectory = raw_full
            # filtered_trajectory = filtered_full

        # ── 5. Stewart platform inverse kinematics ───────────────────────────
        print("Running inverse kinematics ...")
        ik_result = platform.compute_trajectory_ik(filtered_trajectory)
    elif Sinus_rörelse is True:
        print("Applying sinusoidal translation for testing ...")
        # Generate independent time array for sine wave test
        sample_rate = 100.0  # Hz
        duration = 10.0  # seconds
        num_samples = int(sample_rate * duration)
        time_array = np.linspace(0, duration, num_samples)
        
        surge, sway, heave = sinus_translation(
             time_array,
             amplitude=0.03, # meter
             freq=0.2,
             axis="heave",
         )
        
        # Create trajectory with zeros for rotations
        test_trajectory = Trajectory6DOF(
            time=time_array,
            surge=surge,
            sway=sway,
            heave=heave,
            roll=np.zeros_like(time_array),
            pitch=np.zeros_like(time_array),
            yaw=np.zeros_like(time_array),
            sample_rate=sample_rate,
        )
        
        # For visualization, use same trajectory for both raw and filtered
        raw_trajectory = test_trajectory
        filtered_trajectory = test_trajectory
        
        ik_result = platform.compute_trajectory_ik(test_trajectory)


    if SEND_TO_ARDUINO:
        #stream_ik_to_arduino("COM5", 115200, ik_result)     # Windows
        #stream_ik_to_arduino("/dev/ttyACM0", 115200, ik_result)  # Linux
        stream_ik_to_arduino(
          port=SERIAL_PORT,
          baudrate=115200,
          ik_result=ik_result,
          neutral_leg_length_m=platform.neutral_leg_length,
          actuator_center_mm=50.0,
          actuator_min_mm=0.0,
          actuator_max_mm=100.0,
          send_every_nth=5,
          startup_delay_s=3.0,
          batch_size=1,
          wait_between_batches_s=0.05,
          calibrate_before_stream=True,
          calibration_wait_s=18.0,
        )

    #ik_result = platform.compute_trajectory_ik(raw_trajectory)
    #filtrerar vi inte vi absolut yaw och platformen kommer vara 90graders felvriden.
    # ── 6. Actuator analysis ─────────────────────────────────────────────
    ik_result.print_actuator_summary()
    leg_mm = ik_result.leg_lengths * 1000.0

    leg_mm = ik_result.leg_lengths * 1000.0
    neutral_mm = platform.neutral_leg_length * 1000.0
    relative_mm = leg_mm - neutral_mm
    if simulering_köras:
        for i in range(6):
            print(f"Leg {i+1} first 10 relative values (mm):")
            print(relative_mm[:10, i])

        for i in range(6):
            print(
                f"Leg {i+1}: rel min={relative_mm[:, i].min():.2f} mm, "
                f"rel max={relative_mm[:, i].max():.2f} mm"
            )

        # ── 7. Visualize ─────────────────────────────────────────────────────
        print("Generating plots ...")
        plot_analysis(raw_trajectory, filtered_trajectory, ik_result)
        _fig, _anim = animate_platform(platform, filtered_trajectory, ik_result)
        #_fig, _anim = animate_platform(platform, raw_trajectory, ik_result)§
        print("Displaying — close windows to exit.")
        show_all()
    

if __name__ == "__main__":
    main()
