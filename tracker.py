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

        # Subscribe to markers in mocap
        print("Initializing mocap rigid body subscriber...")
        self.rb_subscriber = Subscriber(self, RigidBodies, '/rigid_bodies')

        # time syncrhonizer
        ts = ApproximateTimeSynchronizer(
            [self.vr_subscriber, self.rb_subscriber],
            10,      
            0.01 # slop is the maximum allowed time difference between messages for them to be considered "synchronized"
        )
        ts.registerCallback(self.track)
        self.data = {
            "keypoints": [],
            "rigid_bodies": {}
        }

        self.R_vm, self.t_vm = load_transform(self)

        

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')

        self.ax.set_xlim(-0.75, 0.75)
        self.ax.set_ylim(-0.75, 0.75)
        self.ax.set_zlim(-0.75, 0.75)

        self.quivers = []
        self.scatter = self.ax.scatter([], [], [], c='b', marker='o')

    def track(self, vr_keypoints, rigid_bodies):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        if not self.collecting:
            return

        if elapsed >= self.collection_duration:
            self.collecting = False
            self.get_logger().info(
                f"Data collection finished ({elapsed:.2f}s). Saving to disk..."
            )
            self.save_data()
            return

        kp_vr = np.array([
            [p.position.x, p.position.y, -p.position.z] # Unity uses a left handed frame, so we negate the z coordinate
            for p in vr_keypoints.poses
        ])

        # Transform vr keypoints to mocap space
        kp_mocap = (self.R_vm @ kp_vr.T).T + self.t_vm
        self.data["keypoints"].append(kp_mocap)
        
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