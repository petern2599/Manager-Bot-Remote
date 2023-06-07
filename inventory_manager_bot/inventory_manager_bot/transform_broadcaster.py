import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

class TransformBroadcasterNode(Node):
    """
    Node that converts state estimation from kalman filter
    into transform for SLAM
    """
    def __init__(self,name,sub_vel_name, sub_pos_name, sub_ang_vel_name,sub_angle_name):
        super().__init__(name)
        self.kal_vel_subscriber = self.create_subscription(Vector3,sub_vel_name,self._kal_vel_callback,10)
        self.kal_pos_subscriber = self.create_subscription(Vector3,sub_pos_name,self._kal_pos_callback,10)
        self.kal_ang_vel_subscriber = self.create_subscription(Vector3,sub_ang_vel_name,self._kal_ang_vel_callback,10)
        self.kal_angle_subscriber = self.create_subscription(Vector3,sub_angle_name,self._kal_angle_callback,10)

        self._setup_subscriber_variables()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.01,self.broadcast)

        self.get_logger().info("Transform Broadcaster Node Started")

    def broadcast(self):
        transform_stamp = TransformStamped()

        transform_stamp.header.stamp = self.get_clock().now().to_msg()
        transform_stamp.header.frame_id = "odom"
        transform_stamp.child_frame_id = "base_footprint"

        transform_stamp.transform.translation.x = -self.pos_y
        transform_stamp.transform.translation.y = self.pos_x
        transform_stamp.transform.translation.z = 0.0

        rot = Rotation.from_euler('xyz',[0,0,-self.angle_z],degrees=False)

        rot_quat = rot.as_quat()
        transform_stamp.transform.rotation.x = rot_quat[0]
        transform_stamp.transform.rotation.y = rot_quat[1]
        transform_stamp.transform.rotation.z = rot_quat[2]
        transform_stamp.transform.rotation.w = rot_quat[3]

        self.tf_broadcaster.sendTransform(transform_stamp)

    def _setup_subscriber_variables(self):
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.ang_vel_z = 0.0
        self.angle_z = 0.0

    def _kal_vel_callback(self,msg):
        self.vel_x = msg.x
        self.vel_y = msg.y

    def _kal_pos_callback(self,msg):
        self.pos_x = msg.x
        self.pos_y = msg.y

    def _kal_ang_vel_callback(self,msg):
        self.ang_vel_z = msg.z

    def _kal_angle_callback(self,msg):
        self.angle_z = msg.z

def main(args=None):
    rclpy.init(args=args)

    node = TransformBroadcasterNode("transform_broadcaster_node","kal_vel","kal_pos","kal_ang_vel","kal_angle")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
