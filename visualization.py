"""
Visualization for the Stewart platform horse-riding simulator.

Window 1: 3D animated Stewart platform following the filtered trajectory.
Window 2: Multi-panel analysis plots (original vs filtered motion, leg lengths, etc.).
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed for 3d projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from motion_reconstruction import Trajectory6DOF
from stewart_platform import StewartPlatform, IKResult
from config import VIZ


# ─── Colour palette ─────────────────────────────────────────────────────────

_LEG_COLORS = ["#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#42d4f4"]
_DOF_LABELS = ["Surge (m)", "Sway (m)", "Heave (m)",
               "Roll (rad)", "Pitch (rad)", "Yaw (rad)"]


# ─── Analysis plots ─────────────────────────────────────────────────────────

def plot_analysis(raw: Trajectory6DOF, filtered: Trajectory6DOF,
                  ik: IKResult):
    """
    Multi-panel comparison figure.

    Top 6 panels: original (grey) vs filtered (coloured) for each DOF.
    """
    fig, axes = plt.subplots(3, 2, figsize=(25, 20), sharex=True)
    fig.suptitle("Rörelseanalys — Original vs Filtrerad", fontsize=25)

    raw_pose = raw.pose_array
    filt_pose = filtered.pose_array

    # DOF comparison plots (left: surge/sway/heave, right: roll/pitch/yaw)
    left_indices = [0, 1, 2]
    right_indices = [3, 4, 5]
    for row in range(3):
        left_i = left_indices[row]
        right_i = right_indices[row]

        left_ax = axes[row, 0]
        left_ax.plot(raw.time, raw_pose[:, left_i], color="0.7", lw=0.8, label="Original")
        left_ax.plot(filtered.time, filt_pose[:, left_i], color="C0", lw=1.0, label="Filtrerad")
        left_ax.set_ylabel(_DOF_LABELS[left_i], fontsize=25)
        left_ax.grid(True, alpha=0.3)
        left_ax.tick_params(axis="both", labelsize=20)
        if row == 0:
            left_ax.legend(fontsize=20)

        right_ax = axes[row, 1]
        right_ax.plot(raw.time, raw_pose[:, right_i], color="0.7", lw=0.8, label="Original")
        right_ax.plot(filtered.time, filt_pose[:, right_i], color="C0", lw=1.0, label="Filtrerad")
        right_ax.set_ylabel(_DOF_LABELS[right_i], fontsize=25)
        right_ax.grid(True, alpha=0.3)
        right_ax.tick_params(axis="both", labelsize=20)

    # Restore x-axis numbers on the bottom row only
    for ax in axes[2, :]:
        ax.set_xlabel("Tid (s)", fontsize=25)
        ax.tick_params(axis="x", labelbottom=True)

    fig.tight_layout(rect=[0, 0.03, 1, 0.97])
    return fig


# ─── 3D Animation ───────────────────────────────────────────────────────────

def animate_platform(platform: StewartPlatform, traj: Trajectory6DOF,
                     ik: IKResult):
    """
    Create a 3D matplotlib animation of the Stewart platform.

    The animation runs at the configured FPS and playback speed.
    """
    # Subsample to animation FPS
    total_time = traj.time[-1] - traj.time[0]
    n_frames_full = len(traj.time)
    step = max(1, int(traj.sample_rate / (VIZ.animation_fps * VIZ.playback_speed)))
    frame_indices = np.arange(0, n_frames_full, step)
    n_frames = len(frame_indices)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Axis limits
    R = max(platform.base_joints[:, :2].max(), 0.7)
    ax.set_xlim(-R, R)
    ax.set_ylim(-R, R)
    ax.set_zlim(-0.1, platform.neutral_height + 0.4)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Stewartplattform — Hästsadelrörelse")

    # Draw base plate (static)
    base = platform.base_joints
    base_order = _convex_order_2d(base[:, :2])
    base_polygon = base[base_order]
    base_poly = Poly3DCollection([base_polygon], alpha=0.15, facecolor="steelblue",
                                  edgecolor="steelblue", linewidth=1.5)
    ax.add_collection3d(base_poly)

    # Base joint markers
    ax.scatter(base[:, 0], base[:, 1], base[:, 2],
               c="steelblue", s=40, zorder=5, depthshade=False)

    # Initialise animated elements
    leg_lines = []
    for i in range(6):
        (line,) = ax.plot([], [], [], color=_LEG_COLORS[i], lw=2.5)
        leg_lines.append(line)

    plat_poly = Poly3DCollection([], alpha=0.25, facecolor="orange",
                                  edgecolor="darkorange", linewidth=1.5)
    ax.add_collection3d(plat_poly)

    plat_scatter = ax.scatter([], [], [], c="darkorange", s=40, zorder=5,
                               depthshade=False)

    time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, fontsize=10)

    def _update(frame_num):
        idx = frame_indices[frame_num]
        pj = ik.platform_joints[idx]  # (6, 3)

        # Update legs
        for i in range(6):
            bx, by, bz = base[i]
            px, py, pz = pj[i]
            leg_lines[i].set_data_3d([bx, px], [by, py], [bz, pz])

        # Update platform plate
        order = _convex_order_2d(pj[:, :2])
        plat_poly.set_verts([pj[order]])

        # Update platform joint markers
        plat_scatter._offsets3d = (pj[:, 0], pj[:, 1], pj[:, 2])

        time_text.set_text(f"t = {traj.time[idx]:.2f} s")

        return leg_lines + [plat_poly, plat_scatter, time_text]

    interval_ms = 1000.0 / VIZ.animation_fps
    anim = FuncAnimation(fig, _update, frames=n_frames,
                         interval=interval_ms, blit=False, repeat=True)

    return fig, anim


def _convex_order_2d(xy: np.ndarray) -> np.ndarray:
    """Return indices that order 2D points counter-clockwise around centroid."""
    cx, cy = xy.mean(axis=0)
    angles = np.arctan2(xy[:, 1] - cy, xy[:, 0] - cx)
    return np.argsort(angles)


def show_all():
    """Display all open matplotlib figures."""
    plt.show()
