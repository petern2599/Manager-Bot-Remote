import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import sys
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation

class StaticBroadcasterNode(Node):
    """
    Node that broadcasts all static transforms
    """
    def __init__(self,name):
        super().__init__(name)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms()
        self.get_logger().info("Static Broadcaster Node Started")
        
    def make_transforms(self):
        transform_stamp = TransformStamped()

        transform_stamp.header.stamp = self.get_clock().now().to_msg()
        transform_stamp.header.frame_id = "base_footprint"
        transform_stamp.child_frame_id = "laser"

        transform_stamp.transform.translation.x = -0.01337
        transform_stamp.transform.translation.y = 0.0
        transform_stamp.transform.translation.z = 0.0

        rot2 = Rotation.from_euler('xyz',[0,0,0],degrees=True)

        rot_quat2 = rot2.as_quat()
        transform_stamp.transform.rotation.x = rot_quat2[0]
        transform_stamp.transform.rotation.y = rot_quat2[1]
        transform_stamp.transform.rotation.z = rot_quat2[2]
        transform_stamp.transform.rotation.w = rot_quat2[3]
        
        self.static_broadcaster.sendTransform(transform_stamp)


def main(args=None):
    rclpy.init(args=args)

    node = StaticBroadcasterNode("static_broadcaster_node")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
