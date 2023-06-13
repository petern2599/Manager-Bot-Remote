import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Vector3
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation

class MapTransformListenerNode(Node):
    
    def __init__(self,name,map_pos_name,map_angle_name):
        super().__init__(name)
        self.target_frame = "map"
        self.reference_frame = "base_footprint"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

        self.map_pos_publisher = self.create_publisher(Vector3,map_pos_name,10)
        self.map_angle_publisher = self.create_publisher(Vector3,map_angle_name,10)
        
        self.timer = self.create_timer(0.01,self.listen)

        self.get_logger().info("Transform Listener Node Started")

    def listen(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame,self.reference_frame,Time())
            translation_x = tf.transform.translation.y
            translation_y = -tf.transform.translation.x
            rotation_x = tf.transform.rotation.x
            rotation_y = tf.transform.rotation.y
            rotation_z = tf.transform.rotation.z
            rotation_w = tf.transform.rotation.w

            rot = Rotation.from_quat([rotation_x,rotation_y,rotation_z,rotation_w])

            rot_euler = rot.as_euler('xyz',degrees=True)
            heading = -rot_euler[2]
            heading = self.convert_heading_angle(heading)

            position = Vector3()
            position.x = translation_x
            position.y = translation_y
            position.z = 0.0

            angle = Vector3()
            angle.x = 0.0
            angle.y = 0.0
            angle.z = heading

            self.map_pos_publisher.publish(position)
            self.map_angle_publisher.publish(angle)
            
            # print("x: {} | y: {} | heading : {}".format(round(translation_x,2),round(translation_y,2),round(heading,2)))
            
        except Exception as e:
            print("Unable to get transform")

    def convert_heading_angle(self,heading):
        if heading >= 0:
            return heading
        elif heading < 0:
            return 360 + heading
        


def main(args=None):
    rclpy.init(args=args)

    node = MapTransformListenerNode("map_transform_listener_node","map_pos",'map_angle')

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
