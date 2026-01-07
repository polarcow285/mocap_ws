#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import Markers, RigidBodies
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.spatial.transform import Rotation as R
import zmq
from zmq_utils import ZMQKeypointSubscriber
import yaml
import numpy as np
"""
NOT READY
"""
def load_transform(self):
    path = Path.home() / '.ros' / 'vr_to_mocap.yaml'

    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    R = np.array(data['rotation_matrix'])
    t = np.array(data['translation'])

    return R, t
class Track(Node):
    def __init__(self):
        super().__init__('transform_keypoints')
        host = '192.10.0.12'
        keypoint_port = 5555
        # Raw keypoints from VR (in tracker space)
        print("Initializing raw keypoints subscriber...")
        self.raw_keypoints_subscriber = ZMQKeypointSubscriber(host, keypoint_port, 'raw_keypoints')
        print("Raw keypoints subscriber initialized. Listening to port {}".format(keypoint_port))

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')

        self.ax.set_xlim(-0.75, 0.75)
        self.ax.set_ylim(-0.75, 0.75)
        self.ax.set_zlim(-0.75, 0.75)

        self.quivers = []
        self.scatter = self.ax.scatter([], [], [], c='b', marker='o')

    def cb(self, raw_keypoints):
        # Viusalize keypoints
        xs = [k[0] for k in raw_keypoints]
        ys = [k[1] for k in raw_keypoints]
        zs = [k[2] for k in raw_keypoints]

        self.scatter._offsets3d = (xs, ys, zs)
        self.ax.set_title(f'({len(raw_keypoints)} keypoints)')
        plt.draw()
        plt.pause(0.001)

        # Transform vr keypoints to mocap space

        # Apply transformation from vr space to mocap space

        # 


    def is_fig_open(self):
        """Check if the matplotlib figure window is still open"""
        return plt.fignum_exists(self.fig.number)

        

def main():
    rclpy.init()
    node = TransformKeypoints()
    try:
        while rclpy.ok() and node.is_fig_open():
            raw_keypoints = node.raw_keypoints_subscriber.recv_keypoints()
            if raw_keypoints is not None:
                node.cb(raw_keypoints)
            plt.pause(0.001)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        # Ensure matplotlib GUI closes gracefully
        plt.ioff()  # Turn off interactive mode
        plt.close('all')  # Close all figures
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()