import rclpy
from rclpy.node import Node
from inventory_management_interfaces.srv import CallRobotToWaitingArea
from functools import partial

class RobotCallerClientNode(Node):
    def __init__(self,name):
        super().__init__(name)

        self.get_logger().info("Caller Client Node active")

    def call_server(self,user_number):
        client = self.create_client(CallRobotToWaitingArea,"robot_caller")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = CallRobotToWaitingArea.Request()
        request.user_number = user_number

        future = client.call_async(request)
        return future.result()
    
def main(args=None):
    rclpy.init(args=args)
    node = RobotCallerClientNode("robot_caller_client")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()