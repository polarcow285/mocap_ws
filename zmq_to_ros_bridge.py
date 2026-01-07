#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import zmq
from zmq_utils import ZMQKeypointSubscriber
"""
A bridge that subscribes the vr keypoints being pubished over ZMQ by OpenTeach and publishes (the thumb)
keypoint over ROS. 
"""
class Bridge(Node):
    def __init__(self, calibration_mode: bool):
        super().__init__('bridge')
        
        self.calibration_mode = calibration_mode

        host = '192.10.0.12'
        keypoint_port = 5555
        # Raw keypoints from VR (in tracker space)
        print("Initializing raw keypoints subscriber...")
        self.raw_keypoints_subscriber = ZMQKeypointSubscriber(host, keypoint_port, 'raw_keypoints')
        print("Raw keypoints subscriber initialized. Listening to port {}".format(keypoint_port))

        self.vr_keypoint_publisher = self.create_publisher(PointStamped, "/vr_keypoint", 10)
        
    def publish_to_ros(self, raw_keypoints):
        # Get the thump tip index
        THUMB_TIP_INDEX = 5
        thumb_tip_keypoint = raw_keypoints[THUMB_TIP_INDEX]
        
        # put into PointStamped message
        ros_keypoint = PointStamped()
        ros_keypoint.header.stamp = self.get_clock().now().to_msg()
        ros_keypoint.point.x = thumb_tip_keypoint[0]
        ros_keypoint.point.y = thumb_tip_keypoint[1]
        ros_keypoint.point.z = thumb_tip_keypoint[2]
        print(thumb_tip_keypoint)
        
        # publish PointStamped to ROS
        self.vr_keypoint_publisher.publish(ros_keypoint)


        

def main():
    rclpy.init()
    node = Bridge()
    try:
        while rclpy.ok():
            raw_keypoints = node.raw_keypoints_subscriber.recv_keypoints()
            node.publish_to_ros(raw_keypoints)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()