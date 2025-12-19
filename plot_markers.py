#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mocap4r2_msgs.msg import Markers, RigidBodies
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.spatial.transform import Rotation as R

class MarkerPlotter(Node):
    def __init__(self):
        super().__init__('marker_plotter')
        self.sub = self.create_subscription(Markers, '/markers', self.cb, 10)
        self.rigid_body_sub = self.create_subscription(RigidBodies, '/rigid_bodies', self.cb_rigid_bodies, 10)
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')

        self.ax.set_xlim(-0.75, 0.75)
        self.ax.set_ylim(-0.75, 0.75)
        self.ax.set_zlim(-0.75, 0.75)

        self.quivers = []
        self.scatter = self.ax.scatter([], [], [], c='b', marker='o')

    def cb(self, msg: Markers):
        xs = [m.translation.x for m in msg.markers]
        ys = [m.translation.y for m in msg.markers]
        zs = [m.translation.z for m in msg.markers]

        self.scatter._offsets3d = (xs, ys, zs)
        self.ax.set_title(f'Frame {msg.frame_number} ({len(msg.markers)} markers)')
        plt.draw()
        plt.pause(0.001)
        
    def cb_rigid_bodies(self, msg: RigidBodies):
        # Remove all existing quivers
        for q in self.quivers:
            q.remove()
        self.quivers.clear()
        
        # Create new quivers for each rigid body
        for rb in msg.rigidbodies:
            pos = rb.pose.position
            quat = rb.pose.orientation  # x, y, z, w

            # Quaternion → rotation matrix
            R_mat = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()
            colors = ['r', 'g', 'b']
            
            # Create quivers for each axis (X, Y, Z)
            for i, c in enumerate(colors):
                q = self.ax.quiver(
                    pos.x, pos.y, pos.z,
                    R_mat[0, i], R_mat[1, i], R_mat[2, i],
                    length=0.1,  # Adjusted length to fit your axis limits
                    color=c,
                    arrow_length_ratio=0.3
                )
                self.quivers.append(q)
            #self.ax.text(f"RB {rb.rigid_body_name}", fontsize=9)
        plt.draw()
        plt.pause(0.001)
    def shutdown(self):
        self.get_logger().info('Shutting down...')
        plt.close(self.fig)

def main():
    rclpy.init()
    node = MarkerPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    plt.close('all')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()