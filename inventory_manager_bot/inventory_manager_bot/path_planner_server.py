import rclpy
from rclpy.node import Node
from inventory_management_interfaces.srv import RequestPath
from inventory_manager_bot.inventory_db_connector import InventoryDBConnector
import numpy as np
import cv2

class MapNode():
    def __init__(self,x,y):
        self._x = x
        self._y = y
        self._cost = 0.0
        self._is_obstacle = False

    def set_cost(self,cost):
        self._cost = cost

    def set_obstacle(self):
        self._is_obstacle = True
    
    
class RobotCallerServerNode(Node):
    def __init__(self,name,file):
        super().__init__(name)

        self.map_data = cv2.imread(file)
        self.map_graph = self.generate_map_nodes(self.map_data.shape[1],self.map_data.shape[0])
        
        self.server = self.create_service(RequestPath, 
                                          "path_planner",
                                          self.get_path_waypoints)
        
        self.get_logger().info("Path Planner Server Node active")

    def get_path_waypoints(self,request,response):
        x_init = request.x_init
        y_init = request.y_init
        x_goal = request.x_goal
        y_goal = request.y_goal

    def generate_map_nodes(self,x_max,y_max):
        self.map_graph = np.empty([x_max,y_max])
        

        

def main(args=None):
    rclpy.init(args=args)
    node = RobotCallerServerNode("robot_caller_server")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()