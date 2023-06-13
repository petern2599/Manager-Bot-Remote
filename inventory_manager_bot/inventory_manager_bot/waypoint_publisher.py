import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from inventory_management_interfaces.msg import Waypoints


class WaypointPublisherNode(Node):
    
    def __init__(self,name):
        super().__init__(name)
        self.map_pos_publisher = self.create_publisher(Waypoints,"waypoints",10)
        
        self.timer = self.create_timer(0.01,self.publish_waypoints)

        self.get_logger().info("Transform Listener Node Started")

    def publish_waypoints(self):
        
        msg = Waypoints()
        msg.x_waypoints = [0.0]
        msg.y_waypoints = [1.0]
        self.map_pos_publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)

    node = WaypointPublisherNode("map_transform_listener_node")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
