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
        self.linear_x = 0
        self.angular_z = 0
        self.max = 255
        self.get_user_input()

    def get_user_input(self):
        msg = Twist()
        while True:
            print("\n")
            key = input('Enter key: ')
            if key == 'w':
                if self.linear_x < self.max and self.linear_x >=0:
                    self.linear_x = 70
            elif key == 's':
                if self.linear_x < self.max and self.linear_x >=0:
                    self.linear_x  = -70
            elif key == 'a':
                if self.angular_z < self.max and self.angular_z >=0:
                    self.angular_z = -200
            elif key == 'd':
                if self.angular_z < self.max and self.angular_z >=0:
                    self.angular_z = 200
            else:
                self.linear_x = 0
                self.angular_z = 0
            
            sys.stdout.write("\r Linear PWM Signal: {} | Angular PWM Signal: {}".format(self.linear_x,self.angular_z))
            msg.linear.x = float(self.linear_x)
            msg.angular.z = float(self.angular_z)
            
            self.keyboard_publisher.publish(msg)
            

def main(args=None):
    rclpy.init(args=args)

    node = KeyboardTeleopNode("keyboard_publisher_node","twist")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
