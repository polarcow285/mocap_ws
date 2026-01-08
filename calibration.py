#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import Markers
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.spatial.transform import Rotation as R
import zmq
from zmq_utils import ZMQKeypointSubscriber
from geometry_msgs.msg import PointStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import yaml
from pathlib import Path


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('transform_keypoints')

        # Subscribe to raw keypoints from VR
        print("Initializing vr raw keypoint subscriber...")
        self.vr_keypoint_subscriber = Subscriber(self, PointStamped, '/vr_keypoint')
        
        # Subscribe to markers in mocap
        print("Initializing mocap marker subscriber...")
        self.markers_subscriber = Subscriber(self, Markers, '/markers')
        
        # time syncrhonizer
        ts = ApproximateTimeSynchronizer(
            [self.vr_keypoint_subscriber, self.markers_subscriber],
            10,      
            0.01 # slop is the maximum allowed time difference between messages for them to be considered "synchronized"
        )
        ts.registerCallback(self.calibrate)

        # --- Calibration storage ---
        self.vr_points = []
        self.mocap_points = []

        self.calibration_delay = 3.0 # seconds. amount of time to wait before calibrating
        self.calibration_duration = 10.0  # seconds
        self.start_time = self.get_clock().now()
        self.calibrated = False


    def calibrate(self, vr_msg, mocap_msg):
        if self.calibrated:
            return
        # --- Check calibration time ---
        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds * 1e-9
        
        if elapsed < self.calibration_delay:
            return

        if elapsed >= self.calibration_duration + self.calibration_delay:
            self.solve_calibration()
            self.show_plots()

        

        # --- Require exactly two mocap markers ---
        if len(mocap_msg.markers) != 2:
            self.get_logger().warn(
                f"Expected 2 mocap markers, got {len(mocap_msg.markers)}"
            )
            return

        # --- Compute mocap thumb midpoint ---
        m1 = mocap_msg.markers[0].translation
        m2 = mocap_msg.markers[1].translation

        mocap_thumb = np.array([
            0.5 * (m1.x + m2.x),
            0.5 * (m1.y + m2.y),
            0.5 * (m1.z + m2.z),
        ])

         # --- VR thumb ---
        p = vr_msg.point
        vr_thumb = np.array([p.x, p.y, -p.z]) # Unity uses a left handed coordinate system, so we negate the z coordinate

        # --- Store points ---
        self.vr_points.append(vr_thumb)
        self.mocap_points.append(mocap_thumb)
        

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

        self.get_logger().info("=== Thumb Calibration Complete ===")
        self.get_logger().info(f"Rotation:\n{R}")
        self.get_logger().info(f"Translation:\n{t}")
        self.get_logger().info(f"RMS error: {np.sqrt(np.mean(errors**2)):.4f} m")

        # Save transformation
        self.R = R
        self.t = t
        self.save_transform(R, t)

    def save_transform(self, R, t):
        data = {
            'rotation_matrix': R.tolist(),
            'translation': t.tolist()
        }

        path = Path.home() / '.ros' / 'vr_to_mocap.yaml'
        with open(path, 'w') as f:
            yaml.dump(data, f)

        self.get_logger().info(f"Saved calibration to {path}")   
         
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

def main():
    rclpy.init()
    node = CalibrationNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()