import numpy as np
import matplotlib.pyplot as plt
import seaborn
from pathlib import Path
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from config import HORSE, SEGMENT_INDEX, STRAIGHT_WINDOWS
from data_loader import load_sensor_data
from motion_reconstruction import reconstruct_motion
from motion_filter import filter_trajectory

#variabler
radie_B=10.5
radie_P=6.25
gamma_B=np.deg2rad(13) #13 grader standard för steward
delta_b=np.deg2rad(13) #13 grader standard för steward
gamma_P =np.deg2rad(9.75) #should be between 8 and 10 for good steering
delta_p =np.deg2rad(9.75) #should be between 8 and 10 for good steering
pi = np.pi


def basplatform(delta_b):
    k= np.arange(6)
    return (2*pi/3)*np.floor((k+1)/2) + ((-1)**k) * (delta_b/2)

def upperplatform(delta_p):
    k= np.arange(6)
    return (2*np.pi/3) * np.floor(k/2)-((-1)**k) * (delta_p/2)+np.pi/3

#säger vart ställdona är placerade
phi_b =basplatform(delta_b)
phi_p = upperplatform(delta_p)


#säger hur ställdon är placerade i förhållande till varandra
psi_B = np.array([ 
    -gamma_B, 
    gamma_B,
    2*pi/3 - gamma_B, 
    2*pi/3 + gamma_B, 
    2*pi/3 + 2*pi/3 - gamma_B, 
    2*pi/3 + 2*pi/3 + gamma_B])

psi_P = np.array([ 
    pi/3 + 2*pi/3 + 2*pi/3 + gamma_P,
    pi/3 + -gamma_P, 
    pi/3 + gamma_P,
    pi/3 + 2*pi/3 - gamma_P, 
    pi/3 + 2*pi/3 + gamma_P, 
    pi/3 + 2*pi/3 + 2*pi/3 - gamma_P])

#definierar punkter där ställdon joinar platformerna 
def joint_punkter(radius, angles, z=0.0):
    x=radius *np.cos(angles)
    y=radius * np.sin(angles)
    z= np.full_like(x,z,dtype=float)
    return np.column_stack((x,y,z))

#baspunkterna där ställdonen är fästa på basen
B=joint_punkter(radie_B,phi_b)
B=np.transpose(B)

#punkterna där ställdonen är fästa på plattformen
P=joint_punkter(radie_P,phi_p)
P=np.transpose(P)


#utgångsposition  för platformen
ut_pos= np.array([0,0,2*radie_B])

#linjära ställdon variabler som kommer att användas i animationen 

l=np.zeros((3,6))
lll= np.zeros((6))

#tilting- rörelse framåt/bakåt
def Rotx(theta):
    c, s= np.cos(theta), np.sin(theta)
    return np.array([
        [1,0,0],
        [0,c,-s],
        [0,s,c]
    ])

#pitching- rörelse up/ner
def Roty(theta):
    c, s= np.cos(theta), np.sin(theta)
    return np.array([
        [c,0,s],
        [0,1,0],
        [-s,0,c]
    ])

#vänster/höger vridning
def Rotz(theta):
    c, s= np.cos(theta), np.sin(theta)
    return np.array([
        [c,-s,0],
        [s,c,0],
        [0,0,1]
    ])

#kombination av rotationer
trans= np.transpose(np.array([0,0,0]))
rotation = np.transpose (np.array([0,0,pi/6]))

#beräkning av ställdonens position och längd i utgångspositionen
R= np.matmul (np.matmul(Rotx(rotation[0]),Roty(rotation[1])), Rotz(rotation[2]))

# vektorer från basen till plattformens ställdonspunkter
l = np.repeat(trans[:, np.newaxis], 6, axis=1) + np.repeat(ut_pos[:, np.newaxis], 6, axis=1) + np.matmul(R, P) - B 
lll = np.linalg.norm(l, axis=0)

# Position av plattformens ställdonspunkter i världskordinater
L = l + B

#visa i 3d 
def plot3D_line(ax, vec_arr_origin, vec_arr_dest, color_):
    for i in range(6):
        ax.plot([vec_arr_origin[0, i] , vec_arr_dest[0, i]],
        [vec_arr_origin[1, i], vec_arr_dest[1, i]],
        [vec_arr_origin[2, i],vec_arr_dest[2, i]],
        color=color_)

#kinematik: beräkna ställdonens position och längd för en given translation och rotation börjar här
def compute_forward_kinematics(trans_vec, rot_vec): #kollar placeringen på alla ställdon
  
    #sätter in translationerna i variabler, kollar hur plattformen flyttas
    tx, ty, tz = trans_vec[0], trans_vec[1], trans_vec[2]

    #sätter in rotationerna i variabler, kollar hur plattformen vrider sig
    roll, pitch, yaw = rot_vec[0], rot_vec[1], rot_vec[2]
    
    #skapar rotationsmatrisen från roll, pitch och yaw (rotationerna)
    Rx = Rotx(roll)
    Ry = Roty(pitch)
    Rz = Rotz(yaw)

    #kombinerar rotationerna i rätt ordning (först roll, sen pitch, sen yaw) till en stor rotationsmatris
    R = np.matmul(np.matmul(Rx, Ry), Rz)
    
    # Plattformens mittpunkt i 3D ploten
    platform_origin = np.array([tx, ty, tz + 2*radie_B])
    
    # 3D rotationen av plattformens ställdonspunkter
    P_rotated = np.matmul(R, P)

    #rotera och flytta plattformens ställdonspunkter till deras position i 3D ploten
    L = P_rotated + platform_origin[:, np.newaxis]
    
    #vektorer från basen till plattformens ställdonspunkter
    leg_vecs = L - B
    #längderna på ställdonen
    leg_lengths = np.linalg.norm(leg_vecs, axis=0)
    
    return leg_vecs, leg_lengths, L

#matematiken av hästens rörelse och hur den påverkar plattformen
def load_and_filter_motion(horse_name="Baldur", segment_idx=0, amplitude=0.0158, freq=3.7, axis="all"):
    
    #hämtar filen med rörelsedata för den valda hästen och segmentet
    DATA_DIR = Path(__file__).parent
    
    mat_files = list(DATA_DIR.glob(f"Sensor_*_{horse_name}_exported.mat"))
    if not mat_files:
        raise FileNotFoundError(f"No exported .mat file found for horse: {horse_name}")
    
    #läser in sensordata från filen
    sensor_data = load_sensor_data(mat_files[0], horse=horse_name, sensor_location="Back")
    
    #rekonstruerar rörelsen i 6 DOF (surge, sway, heave, roll, pitch, yaw) från sensordatan
    raw_traj = reconstruct_motion(sensor_data, amplitude, freq, axis)
    
    # Tar ut segmentet av rörelsen som är rakt fram (om segment_idx är angivet)
    if segment_idx is not None and horse_name in STRAIGHT_WINDOWS:
        windows = STRAIGHT_WINDOWS[horse_name]
        if segment_idx < len(windows):
            t0, t1 = windows[segment_idx]
            mask = (raw_traj.time >= t0) & (raw_traj.time <= t1)
            t = raw_traj.time[mask] - raw_traj.time[mask][0]
            raw_traj = type(raw_traj)(
                time=t,
                surge=raw_traj.surge[mask],
                sway=raw_traj.sway[mask],
                heave=raw_traj.heave[mask],
                roll=raw_traj.roll[mask],
                pitch=raw_traj.pitch[mask],
                yaw=raw_traj.yaw[mask],
                sample_rate=raw_traj.sample_rate,
            )
    
    #tar bort höga frekvenskomponenter som inte kan återskapas av ställdonen, tar bort brus
    filt_traj = filter_trajectory(raw_traj)
    
    return filt_traj

#visualisering av animationen
def animate_simulation(trajectory):
    # Förbered data för animationen
    fig = plt.figure(figsize=(12, 6))
    
    # 3D fönstret
    ax_anim = fig.add_subplot(121, projection='3d')
    ax_anim.set_xlim3d(-10, 10)
    ax_anim.set_ylim3d(-10, 10)
    ax_anim.set_zlim3d(0, 20)
    ax_anim.set_xlabel('X (m)')
    ax_anim.set_ylabel('Y (m)')
    ax_anim.set_zlabel('Z (m)')
    ax_anim.set_title('Stewart Platform Simulation')
    
    # tiden
    ax_time = fig.add_subplot(222)
    ax_time.plot(trajectory.time, trajectory.heave * 1000, label='Heave (mm)', color='C0')
    ax_time.set_ylabel('Height (mm)')
    ax_time.set_xlabel('Time (s)')
    ax_time.grid(True, alpha=0.3)
    ax_time.legend()
    
    ax_rot = fig.add_subplot(224)
    ax_rot.plot(trajectory.time, np.rad2deg(trajectory.roll), label='Roll (°)', color='C1')
    ax_rot.plot(trajectory.time, np.rad2deg(trajectory.pitch), label='Pitch (°)', color='C2')
    ax_rot.set_ylabel('Angle (°)')
    ax_rot.set_xlabel('Time (s)')
    ax_rot.grid(True, alpha=0.3)
    ax_rot.legend()
    
    # genomskinlig bas och plattform
    base_poly = [list(np.transpose(B))]
    base_collection = Poly3DCollection(base_poly, facecolors='green', alpha=0.25, edgecolor='darkgreen')
    ax_anim.add_collection3d(base_collection)
    
    # linjer som representerar ställdonen, kommer att uppdateras i animationen
    N = len(trajectory.time)
    legs_list = []
    platforms_list = []
    
    #gå igenom alla tidssteg i rörelsen
    for i in range(N):
        trans = np.array([trajectory.surge[i], trajectory.sway[i], trajectory.heave[i]])
        rots = np.array([trajectory.roll[i], trajectory.pitch[i], trajectory.yaw[i]])
        leg_v, _, platform_pos = compute_forward_kinematics(trans, rots)
        legs_list.append(leg_v)
        platforms_list.append(platform_pos)
    
    # Plota
    leg_lines = [ax_anim.plot([], [], [], 'o-', color='C1', alpha=0.6)[0] for _ in range(6)]
    platform_poly_collection = None
    time_marker, = ax_time.plot([], [], 'r|', markersize=12)
    rot_marker, = ax_rot.plot([], [], 'r|', markersize=12)
    title_text = ax_anim.text2D(0.5, 0.95, '', transform=ax_anim.transAxes, 
                                ha='center', fontsize=11, weight='bold')
    
    #updaterar ploten med röreslen i varje tidssteg
    def update_frame(frame_num):
        nonlocal platform_poly_collection
        
        # hämtar plattformens position för det aktuella tidssteget
        platform_pos = platforms_list[frame_num]
        
        # uppdaterar ställdonens linjer
        for leg_idx in range(6):
            xs = [B[0, leg_idx], platform_pos[0, leg_idx]]
            ys = [B[1, leg_idx], platform_pos[1, leg_idx]]
            zs = [B[2, leg_idx], platform_pos[2, leg_idx]]
            leg_lines[leg_idx].set_data(xs, ys)
            leg_lines[leg_idx].set_3d_properties(zs)
        
        # uppdaterar pltaformen
        if platform_poly_collection:
            try:
                platform_poly_collection.remove()
            except:
                pass
        platform_poly = [list(np.transpose(platform_pos))]
        platform_poly_collection = Poly3DCollection(platform_poly, facecolors='blue', alpha=0.25, edgecolor='darkblue')
        ax_anim.add_collection3d(platform_poly_collection)
        
        # uupdaterar tiden och rotationsmarkörerna i tidsserierna
        time_marker.set_data([trajectory.time[frame_num]], [trajectory.heave[frame_num] * 1000])
        rot_marker.set_data([trajectory.time[frame_num]], [np.rad2deg(trajectory.roll[frame_num])])
        
        # titeln visar aktuell tid i simuleringen
        title_text.set_text(f"t = {trajectory.time[frame_num]:.2f} s (frame {frame_num+1}/{N})")
        
        return leg_lines + [time_marker, rot_marker, title_text]
    
    anim = FuncAnimation(fig, update_frame, frames=N, interval=1000/trajectory.sample_rate, blit=False, repeat=True)
    
    return fig, anim


if __name__ == "__main__":
    print("\n" + "="*70)
    print("  STEWART PLATFORM MOTION SIMULATION")
    print("="*70 + "\n")
    
    try:
        # Load filtered trajectory from motion data
        trajectory = load_and_filter_motion(horse_name="Baldur", segment_idx=0)
        
        # Animate it
        fig, anim = animate_simulation(trajectory)
        plt.tight_layout()
        plt.show()
        
    except Exception as e:
       
        print("\n" + "="*70)
        print("  FALLING BACK: Static 3D Visualization (baseline geometry)")
        print("="*70 + "\n")
        
        # Original static plot
        ax = plt.axes(projection='3d')
        ax.set_xlim3d(-10, 10)
        ax.set_ylim3d(-10, 10)
        ax.set_zlim3d(0, 20)
        ax.add_collection3d(Poly3DCollection([list(np.transpose(B))], facecolors='green', alpha=0.25))
        ax.add_collection3d(Poly3DCollection([list(np.transpose(L))], facecolors='blue', alpha=0.25))
        plot3D_line(ax, B, L, 'orange')
        plt.show()
