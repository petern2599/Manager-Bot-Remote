import rclpy
from rclpy.node import Node
from inventory_management_interfaces.srv import RequestPath
from inventory_manager_bot.inventory_db_connector import InventoryDBConnector
import numpy as np
import cv2
import math
import sys

class MapNode():
    def __init__(self,x,y,index):
        self.x = x
        self.y = y
        self.index = index
        self.cost = 0.0
        self.is_obstacle = False

    def set_cost(self,cost):
        self.cost = cost

    def set_obstacle(self):
        self.is_obstacle = True
    
    
class RobotCallerServerNode(Node):
    def __init__(self,name,file):
        super().__init__(name)

        self._map_data = cv2.imread(file)
        self._resolution_factor = 0.05
        self._min_pos_x = -2.42
        self._min_pos_y = -0.95
        self._width = int(self._map_data.shape[1])
        self._height = int(self._map_data.shape[0])
        self._max_pos_x, self._max_pos_y = self.find_max_pos(self._map_data.shape[1],self._map_data.shape[0])
        self._map_graph, self._max_index = self._generate_map_nodes()
        cv2.namedWindow("Inflation Map")
        cv2.namedWindow("Original")
        self.inflate_obstacles(0.12)
        
        self._server = self.create_service(RequestPath, 
                                          "path_planner",
                                          self.get_path_waypoints)
        
        self.get_logger().info("Path Planner Server Node active")

    def get_path_waypoints(self,request,response):
        x_init = request.x_init
        y_init = request.y_init
        x_goal = request.x_goal
        y_goal = request.y_goal

    def _generate_map_nodes(self):
        map_graph = {}
        map_index = 0
        current_pos_x = self._min_pos_x
        current_pos_y = self._min_pos_y
        step_size = 0.01

        for j in range(int((self._max_pos_y - self._min_pos_y)*100)):
            for i in range(int((self._max_pos_x-self._min_pos_x)*100)):
                map_graph[map_index] = MapNode(current_pos_x,current_pos_y,map_index)
                map_cell_pos_x, map_cell_pos_y = self.interpolate_map_cell_position(current_pos_x,current_pos_y)
                
                is_obstacle = self.check_for_obstacle(map_cell_pos_x,map_cell_pos_y)

                if is_obstacle:
                    map_graph[map_index].set_obstacle()
                
                current_pos_x = round(current_pos_x+ step_size,2)
                map_index += 1
            current_pos_x = self._min_pos_x
            current_pos_y = round(current_pos_y+ step_size,2)

        return map_graph,map_index
        
    
    def find_max_pos(self,width,height):
        max_pos_x = round(width*self._resolution_factor + self._min_pos_x,2)
        max_pos_y = round(height*self._resolution_factor + self._min_pos_y,2)
        return max_pos_x,max_pos_y

    def check_for_obstacle(self,x,y):
        value = np.array(self._map_data[y][x])
        return np.array_equal(value,np.zeros(3))
        
    def interpolate_map_cell_position(self,x,y):
        map_cell_pos_x = int((x - self._min_pos_x)/self._resolution_factor)
        map_cell_pos_y = int(self._height-1) - int((y - self._min_pos_y)/self._resolution_factor)
        return map_cell_pos_x,map_cell_pos_y
    
    def inflate_obstacles(self,radius):
        queue = []
        for index in range(self._max_index):
            if self._map_graph[index].is_obstacle:
                queue.append(index)
        
        counter = 0
        max_counter = len(queue)
        print(max_counter)
        for index in queue:
            sys.stdout.write("\r=== Obstacle Inflation Progress: {}% (Counter: {}) ===".format(int(counter/max_counter*100),counter))
            sys.stdout.flush()
            x = self._map_graph[index].x
            y = self._map_graph[index].y
            local_min_x = round(x - radius,2)
            local_min_y = round(y - radius,2)
            local_max_x = round(x + radius,2)
            local_max_y = round(y + radius,2)

            local_graph_indexes = self.get_local_graph_indexes(local_min_x,local_min_y,local_max_x,local_max_y)
            map_copy = self._map_data.copy()

            for index in local_graph_indexes:
                local_x = self._map_graph[index].x
                local_y = self._map_graph[index].y
                map_cell_pos_x, map_cell_pos_y = self.interpolate_map_cell_position(local_x,local_y)
                within_radius = self.calculate_inflation_radius_pos(local_x,local_y,x,y,radius)
                if within_radius:
                    self._map_graph[index].set_obstacle()
                    # map_copy = cv2.circle(map_copy,(map_cell_pos_x,map_cell_pos_y),1,(0,0,255),-1)
                    # cv2.imshow("map",map_copy)
                    # cv2.waitKey(1)
            
            counter+=1

        for index in range(self._max_index):    
            if self._map_graph[index].is_obstacle:
                x = self._map_graph[index].x
                y = self._map_graph[index].y
                map_cell_pos_x, map_cell_pos_y = self.interpolate_map_cell_position(x,y)
                map_copy = cv2.circle(map_copy,(map_cell_pos_x,map_cell_pos_y),1,(255,0,0),-1)
                # cv2.imshow("map",map_copy)
                # cv2.waitKey(1)

        for index in queue:    
            x = self._map_graph[index].x
            y = self._map_graph[index].y
            map_cell_pos_x, map_cell_pos_y = self.interpolate_map_cell_position(x,y)
            map_copy = cv2.circle(map_copy,(map_cell_pos_x,map_cell_pos_y),1,(0,0,255),-1)
        cv2.imshow("Inflation Map",map_copy)
        cv2.waitKey(1)

        cv2.imshow("Original",self._map_data)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def get_local_graph_indexes(self,min_x,min_y,max_x,max_y):
        graph_indexes = []
        step_size = 0.01
        current_x = min_x
        current_y = min_y
        while current_y < max_y:
            while current_x < max_x:
                index = self.get_graph_index(current_x,current_y)
                if index > 0 and index < self._max_index:
                    graph_indexes.append(index)
                current_x = round(current_x+step_size,2)
            current_x = min_x
            current_y = round(current_y+step_size,2)
        
        return graph_indexes

    def get_graph_index(self,x,y):
        index_range_x = int((self._max_pos_x-self._min_pos_x)*100)
        dist_x = int((x-self._min_pos_x)*100)
        dist_y = int((y-self._min_pos_y)*100)
        # print("Max x = {} | Min x = {}".format(self._max_pos_x,self._min_pos_x))
        # print("Max y = {} | Min y = {}".format(self._max_pos_y,self._min_pos_y))
        # print("Dist x = {} | Dist y = {}".format(dist_x,dist_y))
        return (dist_y*index_range_x)+dist_x

    def calculate_inflation_radius_pos(self,x,y,center_x,center_y,radius):
        euclidean_dist = math.sqrt(math.pow(x-center_x,2) + math.pow(y-center_y,2))
        # print(euclidean_dist)
        if euclidean_dist > radius:
            return False
        return True
        
    
def main(args=None):
    rclpy.init(args=args)
    file = "/home/petern25/ros2_ws/my_map_save.pgm"
    node = RobotCallerServerNode("robot_caller_server",file)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()