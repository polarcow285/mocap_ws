#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from mocap4r2_msgs.msg import Markers, RigidBodies
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.spatial.transform import Rotation as R
import yaml
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from pathlib import Path
from geometry_msgs.msg import PointStamped, Pose, PoseArray

def load_transform(self):
    path = Path.home() / '.ros' / 'vr_to_mocap.yaml'

    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    R = np.array(data['rotation_matrix'])
    t = np.array(data['translation'])

    return R, t

class Tracker(Node):
    def __init__(self):
        super().__init__('transform_keypoints')
        self.collection_duration = 10.0  # seconds
        self.start_time = self.get_clock().now()
        self.collecting = True

        # Subscribe to raw keypoints from VR
        print("Initializing vr raw keypoints subscriber...")
        self.vr_subscriber = Subscriber(self, PoseArray, '/vr_keypoints')

        # Subscribe to rigid bodies in mocap
        print("Initializing mocap rigid body subscriber...")
        self.rb_subscriber = Subscriber(self, RigidBodies, '/rigid_bodies')

        # Subscribe to markers in mocap
        print("Initializing mocap markers subscriber...")
        self.markers_subscriber = Subscriber(self, Markers, '/markers')

        # time syncrhonizer
        ts = ApproximateTimeSynchronizer(
            [self.vr_subscriber, self.rb_subscriber, self.markers_subscriber],
            10,      
            0.01 # slop is the maximum allowed time difference between messages for them to be considered "synchronized"
        )

        ts.registerCallback(self.track)
        self.data = {
            "keypoints": [],
            "rigid_bodies": {}
        }
        # For calibration
        self.wrist_points = []
        self.vr_points = []

        self.R = None
        self.t = None

        

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')

        self.ax.set_xlim(-0.75, 0.75)
        self.ax.set_ylim(-0.75, 0.75)
        self.ax.set_zlim(-0.75, 0.75)

        self.quivers = []
        self.scatter = self.ax.scatter([], [], [], c='b', marker='o')

    def track(self, vr_keypoints, rigid_bodies, wrist_markers):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        if not self.collecting:
            return

        if elapsed >= self.collection_duration:
            self.collecting = False
            self.get_logger().info(
                f"Data collection finished ({elapsed:.2f}s). Calibrating..."
            )
            self.calibrate()
            self.get_logger().info(
                f"Transforming Keypoints..."
            )
            self.transform_keypoints()
            self.get_logger().info(
                f"Saving..."
            )
            self.save_data()
            # Show calibration plot
            self.show_plots(self.data["keypoints"][:, 0, :], self.wrist_points)
            return
        
        if len(wrist_markers.markers) == 2:
            m1 = wrist_markers.markers[0].translation
            m2 = wrist_markers.markers[1].translation
            mocap_wrist = np.array([
                0.5 * (m1.x + m2.x),
                0.5 * (m1.y + m2.y),
                0.5 * (m1.z + m2.z),
            ])
        else:
            self.get_logger().warn(
                f"Expected 2 mocap markers, got {len(wrist_markers.markers)}"
            )
            mocap_wrist = np.array([float("inf"), float("inf"), float("inf"),])

        self.wrist_points.append(mocap_wrist)

        #Hand VR keypoints

        kp_vr = np.array([
            [p.position.x, p.position.y, -p.position.z] # Unity uses a left handed frame, so we negate the z coordinate
            for p in vr_keypoints.poses
        ])
        self.vr_points.append(kp_vr)

        
        # Rigid bodies
        for rb in rigid_bodies.rigidbodies:
            rb_id = rb.rigid_body_name

            pos = np.array([
                rb.pose.position.x,
                rb.pose.position.y,
                rb.pose.position.z
            ])

            quat = np.array([
                rb.pose.orientation.x,
                rb.pose.orientation.y,
                rb.pose.orientation.z,
                rb.pose.orientation.w
            ])

            if rb_id not in self.data["rigid_bodies"]:
                self.data["rigid_bodies"][rb_id] = {
                    "pos": [],
                    "quat": []
                }
            self.data["rigid_bodies"][rb_id]["pos"].append(pos)
            self.data["rigid_bodies"][rb_id]["quat"].append(quat)

    def calibrate(self):
        # Select the 0th keypoint to get the wrist keypoint
        P_all = np.array(self.vr_points)[:, 0, :]      # Nx3
        Q_all = np.array(self.wrist_points )  # Nx3
        
        # Filter out points where wrist markers could not be tracked
        valid = ~np.isinf(Q_all).any(axis=1)
        P = P_all[valid] 
        Q = Q_all[valid]

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

        self.get_logger().info("=== Calibration Complete ===")
        self.get_logger().info(f"Rotation:\n{R}")
        self.get_logger().info(f"Translation:\n{t}")
        self.get_logger().info(f"RMS error: {np.sqrt(np.mean(errors**2)):.4f} m")

        # Save transformation
        self.R = R
        self.t = t

    def transform_keypoints(self):
        kp_vr = np.array(self.vr_points)  # (T,24,3)              # (T, 24, 3)
        kp_mocap = kp_vr @ self.R.T + self.t
        self.data["keypoints"] = kp_mocap

    def save_data(self):
        # Save a npz of the following (T is timestep). For each timestep, save:
        # "rb1_pos": (T,3)
        # "rb1_quat": (T,4)
        # ...repeat for each rigid body
        # "keypoints: (T, 24, 3)"
        save_dict = {
            "keypoints": np.stack(self.data["keypoints"])  # (T,N,3)
        }
        for rb_id, rb_data in self.data["rigid_bodies"].items():
            save_dict[f"rb{rb_id}_pos"] = np.stack(rb_data["pos"])
            save_dict[f"rb{rb_id}_quat"] = np.stack(rb_data["quat"])

        np.savez("tracking_data.npz", **save_dict)
        self.get_logger().info("Saved tracking_data.npz")
        
    def show_plots(self, vr_points, mocap_points):
        """Display 3D plots: transformed VR points"""
        P = np.array(vr_points)      # Nx3
        Q = np.array(mocap_points)   # Nx3

        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, projection='3d')
        ax1.scatter(Q[:,0], Q[:,1], Q[:,2], c='b', label='Mocap')
        ax1.scatter(P[:,0], P[:,1], P[:,2], c='r', label='VR (raw)')
        ax1.set_title("Transformed VR vs Mocap")
        ax1.set_xlabel("X"); ax1.set_ylabel("Y"); ax1.set_zlabel("Z")
        ax1.legend()
        ax1.grid(True)

        plt.show()

def main():
    rclpy.init()
    node = Tracker()
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