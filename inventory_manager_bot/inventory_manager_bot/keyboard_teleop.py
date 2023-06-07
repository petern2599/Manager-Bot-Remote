import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from geometry_msgs.msg import Twist
import sys

class KeyboardTeleopNode(Node):
    """
    Node that sends teleop commands based on keyboard input
    """
    def __init__(self,name,pub_name):
        super().__init__(name)
        self.keyboard_publisher = self.create_publisher(Twist,pub_name,10)
        self.get_logger().info("Keyboard Teleop Node active")
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.get_user_input()

    def get_user_input(self):
        msg = Twist()
        while True:
            key = input('Enter key: ')
            if key == 'w':
                self.linear_x = round(self.linear_x + 0.1,2)
            elif key == 's':
                self.linear_x = round(self.linear_x - 0.1,2)
            elif key == 'a':
                self.angular_z = round(self.angular_z - 0.05,2)
            elif key == 'd':
                self.angular_z = round(self.angular_z + 0.05,2)
            else:
                self.linear_x = 0.0
                self.angular_z = 0.0
            
            msg.linear.x = self.linear_x
            msg.angular.z = self.angular_z
            
            self.keyboard_publisher.publish(msg)
            

def main(args=None):
    rclpy.init(args=args)

    node = KeyboardTeleopNode("keyboard_publisher_node","twist")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
