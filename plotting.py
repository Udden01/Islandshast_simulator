import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax= plt.axes(projection='3d')
#plt.show()

#https://raw.org/research/inverse-kinematics-of-a-stewart-platform/ har steward paltform pseudocode and kematics

#tilting forward/back
def Rotx(a):
    c, s= np.cos(a), np.sin(a)
    return np.array([
        [1,0,0],
        [0,c,-s],
        [0,s,c]
    ])

#pitching up/down
def Roty(a):
    c, s= np.cos(a), np.sin(a)
    return np.array([
        [c,0,s],
        [0,1,0],
        [-s,0,c]
    ])

#left/right turn
def Rotz(a):
    c, s= np.cos(a), np.sin(a)
    return np.array([
        [c,-s,0],
        [s,c,0],
        [0,0,1]
    ])

def euler_xyz(roll,pitch,yaw):
    return Rotz(yaw) @ Roty(pitch) @ Rotx(roll)

def upperplatform(delta_p):
    k= np.arange(6)
    return (2*np.pi/3) * np.floor(k/2)-((-1)**k) * (delta_p/2)+np.pi/3

def basplatform(delta_b):
    k= np.arange(6)
    return (2*np.pi/3)*np.floor((k+1)/2) + ((-1)**k) * (delta_b/2)

def basankare(delta_b):
    k= np.arange(6)
    phi_b=basplatform(delta_b)
    return phi_b+(np.pi/2)*((-1)**k)

def joinpunkterpåplatform(radius, angles, z=0.0):
    x=radius *np.cos(angles)
    y=radius * np.sin(angles)
    z= np.full_like(x,z,dtype=float)
    return np.column_stack((x,y,z))

bas_radie= 10.0
pltform_radie=7.0


#utefter formlerna på sidan

delta_b=np.deg2rad(24)
delta_p =np.deg2rad(18)

roll =np.deg2rad(12)
pitch = np.deg2rad(-8)
yaw=np.deg2rad(20)
t=np.array([1.,-0.5,9.0])

phi_b =basplatform(delta_b)
phi_p = upperplatform(delta_p)
beta= basankare(delta_b)

B= joinpunkterpåplatform(bas_radie, phi_b,z=0.0)
P_local = joinpunkterpåplatform(pltform_radie,phi_p,z=0.0)

R= euler_xyz(roll,pitch,yaw)
P=(R@ P_local.T).T + t

benlangd=np.linalg.norm(P-B,axis=1)

#plot

B_closed = np.vstack([B,B[0]])
ax.plot(B_closed[:,0], B_closed[:,1], B_closed[:,2], linewidth =2, label='base')
P_closed = np.vstack([P,P[0]])
ax.plot(P_closed[:,0], P_closed[:,1], P_closed[:,2], linewidth =2, label='UPPERPLAT')

ax.scatter(B[:,0], B[:,1], B[:,2],s=50)
ax.scatter(P[:,0], P[:,1], P[:,2],s=50)

#SLUTET ÄR AI hjälp
for k in range(6):
    ax.plot(
        [B[k, 0], P[k, 0]],
        [B[k, 1], P[k, 1]],
        [B[k, 2], P[k, 2]],
        '--'
    )

    ax.text(B[k, 0], B[k, 1], B[k, 2], f'B{k}', fontsize=10)
    ax.text(P[k, 0], P[k, 1], P[k, 2], f'P{k}', fontsize=10)

# optional: show beta direction at each base anchor
servo_len = 2.0
for k in range(6):
    d = np.array([np.cos(beta[k]), np.sin(beta[k]), 0.0])
    p0 = B[k]
    p1 = B[k] + servo_len * d
    ax.plot(
        [p0[0], p1[0]],
        [p0[1], p1[1]],
        [p0[2], p1[2]],
        ':'
    )

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Stewart Platform Matching Screenshot Angle Layout')
ax.legend()

# equal-ish axis scaling
all_pts = np.vstack([B, P])
mins = all_pts.min(axis=0)
maxs = all_pts.max(axis=0)
center = (mins + maxs) / 2
radius = max(maxs - mins) / 2

ax.set_xlim(center[0] - radius, center[0] + radius)
ax.set_ylim(center[1] - radius, center[1] + radius)
ax.set_zlim(0, center[2] + radius)

plt.tight_layout()
plt.show()
