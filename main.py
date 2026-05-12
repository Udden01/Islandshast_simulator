"""
Main fil för tölt simulation med Stewart-plattform på fysiska simulatorn

Pipeline:
  1. Load phone sensor data (.mat) from a horse ride
  2. Reconstruct 6-DOF saddle trajectory (translations + rotations)
  3. Filter to Stewart platform workspace (bandpass + saturation)
  4. Compute Stewart platform inverse kinematics (6 leg lengths)
  5. Analyse actuator requirements and display 3D animation

Configuration: edit config.py to change horse, segment, filter, geometry, etc.
"""


#nödvändiga imports
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


#Ladda in datafilerna med rådata
DATA_DIR = Path(__file__).parent   
amplitud = 0.005   # meter
freq = 0.2          # Hz
axis = "all"      #  kan vara surge / sway / heave / all

#Funktion som skär ut en del av rörelsebanan
def _slice_trajectory(traj: Trajectory6DOF, t0: float, t1: float) -> Trajectory6DOF: #kan användas för att bara köra en del av ridningen (en raksträcka)
    
    #filter som väljer tidpunkter som ska finnas i utagna intervallet
    mask = (traj.time >= t0) & (traj.time <= t1)

    #ny tidsvekor för valda intervallet
    t = traj.time[mask] - traj.time[mask][0]

    #returnerar ny rörelsebana med endast valt tidsintervall
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

#Funktion som skär ut flera segment och blandar dem för att få en mjuk övergång mellan segmenterna,
def _blend_segments(seg1: Trajectory6DOF, seg2: Trajectory6DOF, blend_samples: int) -> Trajectory6DOF: #blandar 2 segmenter så det blir en mjuk övergång, minska rykicghet emllan segment
    #tidssteg mellan två mätpunkter
    dt = 1.0 / seg1.sample_rate

    #blandar rörelsekomponenterna för varje DOF separat
    def blend_array(a1, a2): 

        #start värde av segment 2 och slutvärde av segment 1
        p0 = a1[-1]
        p1 = a2[0]

        #hastighet i slutet av segment 1 och i början av segment 2
        v0 = (a1[-1] - a1[-2]) / dt
        v1 = (a2[1] - a2[0]) / dt

        #normaliserad tidsvektor för blandingen
        t = np.linspace(0, 1, blend_samples)

        # Cubic Hermite, mjuk och deriverbar övergång som tar hänsyn till både position och hastighet i start och slutpunkterna mellan datan: 
        h00 = 2*t**3 - 3*t**2 + 1
        h10 = t**3 - 2*t**2 + t
        h01 = -2*t**3 + 3*t**2
        h11 = t**3 - t**2

        #returnerar mjuk interpolerad övergång mellan segmenten
        return h00*p0 + h10*v0*dt + h01*p1 + h11*v1*dt

    #skapa en tidsvektor för blendningen som börjar efter seg1 och har lika många punkter som blend_samples
    time = seg1.time[-1] + np.arange(1, blend_samples + 1) * dt 

    #skapa en ny Trajectory6DOF som är blandning av seg1 och seg2, alltså alla mjuka övergångar för alla sex frihetsgrader
    return Trajectory6DOF( 
        time=time,
        surge=blend_array(seg1.surge, seg2.surge),
        sway=blend_array(seg1.sway, seg2.sway),
        heave=blend_array(seg1.heave, seg2.heave),
        roll=blend_array(seg1.roll, seg2.roll),
        pitch=blend_array(seg1.pitch, seg2.pitch),
        yaw=blend_array(seg1.yaw, seg2.yaw),
        sample_rate=seg1.sample_rate,
    )

#Klipper ut fler tidsintervall och sätter ihop de till en rörelsebana
def _slice_trajectory_and_stitch(traj, windows, blend_time=0.1):
    segments = [] 
    time_offset = 0.0
    dt = 1.0 / traj.sample_rate
    #beräknar hur många mätpunkter som behövs för att blanda övergången mellan segmenten, baserat på önskad blendningstid och samplingsfrekvensen
    blend_samples = int(blend_time / dt)

    #variabel för lagra föregående segment
    prev_segment = None

    #går igeom alla tidsintervaller som ska klippas ut, skapar segment för varje intervall och blandar dem med föregående segment för att få en mjuk övergång
    for t0, t1 in windows: 
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

        if prev_segment is not None: 
            #blandar så det blir mjukövergång mellan föregående segment och det nya segmentet
            blend = _blend_segments(prev_segment, segment, blend_samples)
            segments.append(blend)
            time_offset = blend.time[-1]

            #beräknas hur mycket ett segment behöver förskjutas i tid för att det ska följa direkt efter det föregående segmentet, inklusive blendningstiden. Detta säkerställer att tidsvektorn är kontinuerlig och att det inte finns några hopp i tiden mellan segmenten.
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

    #returnerar en ny rörelsebana med alla segment med mjuka övergångar 
    return Trajectory6DOF( 
        time=np.concatenate([seg.time for seg in segments]),
        surge=np.concatenate([seg.surge for seg in segments]),
        sway=np.concatenate([seg.sway for seg in segments]),
        heave=np.concatenate([seg.heave for seg in segments]),
        roll=np.concatenate([seg.roll for seg in segments]),
        pitch=np.concatenate([seg.pitch for seg in segments]),
        yaw=np.concatenate([seg.yaw for seg in segments]),
        sample_rate=segments[0].sample_rate,
    )

#funktion som skapar en sinusformad translationrörelse
def sinus_translation(time: np.ndarray, amplitude: float, freq: float, axis: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]: #skapar en sinusformad translation i en given axel (surge/sway/heave) för testning av plattformen
    
    #sinusformad rörelse baserat på tid, amplitud och frekvens
    x = amplitude * np.sin(2 * np.pi * freq * time)

    surge = np.zeros_like(time)
    sway = np.zeros_like(time)
    heave = np.zeros_like(time)

    #alla tre axlarna har samma frekvens
    surge_freq = freq
    sway_freq = freq
    heave_freq = freq

    #definierar fasförskjutning för alla rörelseriktningar
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
    #returnerar de tre translationsrörelserna
    return surge, sway, heave

#definierar prograammets huvudfunktion
def main():
    
    SEND_TO_ARDUINO = False
    simulering_köras = True
    Sinus_rörelse = False

    # skapar Stewart-plattform utifrån konfigurationen i config.py
    platform = StewartPlatform.from_config()
    print(f"ben langd: {platform.neutral_leg_length * 1000:.1f} mm")
    
    if Sinus_rörelse is False:
        #laddar sensordata från .mat filer
        mat_file = DATA_DIR / f"Sensor_Back_{HORSE}_exported.mat"
        print(f"Loading {mat_file.name} ...")
        sensor_full = load_sensor_data(mat_file, horse=HORSE, sensor_location="Back")
        print(f"  Loaded {len(sensor_full.time)} samples, "
            f"{sensor_full.time[-1]:.1f} s, {sensor_full.sample_rate:.0f} Hz")
        # rekonstruerar 6-DOF rörelse från komplettsensordata, med amplitud och frekvensskalning
        # Viktigt! att köra på hela inspelningen för att undvika filtertransienter, 
        # bandpassfiltret behöver flera sekunder på sig att stabilisera sig så korta segment (10s) kan få kraftiga tranisenter som förstör signalen.
        print("Reconstructing 6-DOF motion (full recording) ...")

        raw_full = reconstruct_motion(sensor_full, amplitud ,freq,axis) #

        print("Filtering trajectory for Stewart platform workspace ...")
        filtered_full = filter_trajectory(raw_full)

        # skär ut segmentet som ska köras, eller blanda flera segment för att få en mjuk övergång mellan dem

        if SEGMENT_INDEX is not None:
            t0, t1 = windows[SEGMENT_INDEX]

            raw_trajectory = _slice_trajectory(raw_full, t0, t1)
            filtered_trajectory = _slice_trajectory(filtered_full, t0, t1)
        else:
            raw_trajectory = _slice_trajectory_and_stitch(raw_full, windows)
            filtered_trajectory = _slice_trajectory_and_stitch(filtered_full, windows)
            # raw_trajectory = raw_full
            # filtered_trajectory = filtered_full

        # Stewart platform inverse kinematics ───────────────────────────
        print("Running inverse kinematics ...")
        ik_result = platform.compute_trajectory_ik(filtered_trajectory)
    elif Sinus_rörelse is True:
        print("Applying sinusoidal translation for testing ...")
        #genererar en tidsvektor baserat på önskad samplingsfrekvens och varaktighet för den sinusformade rörelsen
        sample_rate = 100.0  # Hz
        duration = 10.0  # seconds
        num_samples = int(sample_rate * duration)
        time_array = np.linspace(0, duration, num_samples)
        
        #skapar sinusformade rörelser i surge, sway och heave baserat på den genererade tidsvektorn, med specificerad amplitud och frekvens
        surge, sway, heave = sinus_translation(
             time_array,
             amplitude=0.03, # meter
             freq=0.2,
             axis="heave",
         )
        
        # skapar en test rörelsebana med de genererade sinusformade rörelserna, där alla rotationskomponenter (roll, pitch, yaw) är noll, och specificerar samplingsfrekvensen
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
        
        # För testning av plattformen 
        raw_trajectory = test_trajectory
        filtered_trajectory = test_trajectory
        
        ik_result = platform.compute_trajectory_ik(test_trajectory)



    #kontrollerar om all data skickats till Arduino
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
    # analys av ställdon─────────────────────────────────────────────
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

        #visualisering ─────────────────────────────────────────────────────
        print("Generating plots ...")
        plot_analysis(raw_trajectory, filtered_trajectory, ik_result)
        _fig, _anim = animate_platform(platform, filtered_trajectory, ik_result)
        #_fig, _anim = animate_platform(platform, raw_trajectory, ik_result)§
        print("Displaying — close windows to exit.")
        show_all()
    

if __name__ == "__main__":
    main()
