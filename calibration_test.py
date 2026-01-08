import numpy as np
import matplotlib.pyplot as plt
import yaml
from pathlib import Path

class CalibrationDemo:
    def __init__(self):
        self.vr_points = []
        self.mocap_points = []
        self.R = None
        self.t = None

    def solve_calibration(self):
        self.calibrated = True

        P = np.array(self.vr_points)      # Nx3
        Q = np.array(self.mocap_points)   # Nx3

        if P.shape[0] < 3:
            self.get_logger().error("Not enough motion for calibration")
            return

        # --- Centroids ---
        p_mean = P.mean(axis=0)
        q_mean = Q.mean(axis=0)

        # --- Centered trajectories ---
        P_c = P - p_mean
        Q_c = Q - q_mean

        # --- Least-squares rotation ---
        H = P_c.T @ Q_c
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # Fix reflection
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = q_mean - R @ p_mean

        # --- Error ---
        P_aligned = (R @ P.T).T + t
        errors = np.linalg.norm(P_aligned - Q, axis=1)

        print("=== Test Calibration Complete ===")
        print(f"Rotation:\n{R}")
        print(f"Translation:\n{t}")
        print(f"RMS error: {np.sqrt(np.mean(errors**2)):.4f} m")

        # Save transformation
        self.R = R
        self.t = t

    def show_plots(self):
        """Display 3D plots: raw and transformed VR points"""
        P = np.array(self.vr_points)      # Nx3
        Q = np.array(self.mocap_points)   # Nx3

        # --- Plot 1: raw ---
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, projection='3d')
        ax1.scatter(Q[:,0], Q[:,1], Q[:,2], c='b', label='Mocap')
        ax1.scatter(P[:,0], P[:,1], P[:,2], c='r', label='VR (raw)')
        ax1.set_title("Raw VR vs Mocap")
        ax1.set_xlabel("X"); ax1.set_ylabel("Y"); ax1.set_zlabel("Z")
        ax1.legend()
        ax1.grid(True)

        # --- Plot 2: transformed ---
        P_transformed = (self.R @ P.T).T + self.t
        fig2 = plt.figure()
        ax2 = fig2.add_subplot(111, projection='3d')
        ax2.scatter(Q[:,0], Q[:,1], Q[:,2], c='b', label='Mocap')
        ax2.scatter(P_transformed[:,0], P_transformed[:,1], P_transformed[:,2],
                    c='r', label='VR (transformed)')
        ax2.set_title("VR aligned to Mocap frame")
        ax2.set_xlabel("X"); ax2.set_ylabel("Y"); ax2.set_zlabel("Z")
        ax2.legend()
        ax2.grid(True)

        plt.show()  # blocks until closed

def generate_demo_data():
    """
    Create a complex VR trajectory, transform it into mocap space,
    and add Gaussian noise.
    """

    np.random.seed(42)  # reproducible noise

    # --- Time ---
    N = 2000
    t = np.linspace(0, 4*np.pi, N)

    # --- Complex VR trajectory (non-planar, non-uniform) ---
    vr_traj = np.vstack([
        0.25 * np.sin(1.3 * t) + 0.05 * np.sin(5.0 * t),
        0.20 * np.cos(0.9 * t) + 0.04 * np.cos(3.7 * t),
        0.15 * np.sin(0.7 * t) + 0.02 * t
    ]).T

    # --- Add VR tracking noise (e.g., headset/controller jitter) ---
    vr_noise_sigma = 0.00005  # 2 mm
    vr_traj += vr_noise_sigma * np.random.randn(N, 3)

    # --- Ground-truth rotation (roll, pitch, yaw) ---
    roll  = np.deg2rad(20)
    pitch = np.deg2rad(-15)
    yaw   = np.deg2rad(35)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [ 0,             1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])

    R_gt = Rz @ Ry @ Rx

    # --- Ground-truth translation ---
    t_gt = np.array([0.45, -0.30, 0.25])

    # --- Transform into mocap space ---
    mocap_traj = (R_gt @ vr_traj.T).T + t_gt

    # --- Add mocap measurement noise ---
    mocap_noise_sigma = 0.01  # 1 mm
    mocap_traj += mocap_noise_sigma * np.random.randn(N, 3)

    print("Ground-truth Rotation:\n", R_gt)
    print("Ground-truth Translation:\n", t_gt)
    print(f"VR noise σ = {vr_noise_sigma} m")
    print(f"Mocap noise σ = {mocap_noise_sigma} m")

    return vr_traj, mocap_traj



if __name__ == "__main__":
    vr, mocap = generate_demo_data()

    demo = CalibrationDemo()
    demo.vr_points = vr
    demo.mocap_points = mocap

    demo.solve_calibration()
    demo.show_plots()
