import rclpy
from rclpy.node import Node
from inventory_management_interfaces.srv import RequestPath
import numpy as np
import cv2
import math
import sys

class MapNode():
    """
    This is a class that describes a map node with information used
    for path-planning
    """
    def __init__(self,x,y,index):
        self.x = x
        self.y = y
        self.index = index
        self.cost = None
        self.is_obstacle = False
        self.is_start = False
        self.is_goal = False
        self.linked_index = None

    def set_cost(self,cost):
        self.cost = cost

    def set_obstacle(self):
        self.is_obstacle = True

    def set_as_start(self):
        self.is_start = True

    def set_as_goal(self):
        self.is_goal = True
    
    def set_linked_index(self,index):
        self.linked_index = index
    
    def reset(self):
        self.cost = None
        self.is_start = False
        self.is_goal = False
        self.linked_index = None
    
class InvalidPositionException(Exception):
    """Raised when position in request is out-of-bounds from map"""
    def __init__(self,message):
        self.message = "InvalidPositionException: " + message
        super().__init__(self.message)

class ObstacleDetectedException(Exception):
    """Raised when position is an obstacle"""
    def __init__(self,message):
        self.message = "ObstacleDetectedException: " + message
        super().__init__(self.message)

class PathPlannerServerNode(Node):
    """
    This is a ROS service to plan a path using
    A* path-planning algorithm
    """
    def __init__(self,name):
        super().__init__(name)

        self.declare_parameter("map")
        map_file = self.get_parameter("map").value
        self._map_data = cv2.imread(map_file)

        # These parameters are given in the associated YAML file with map file
        self._resolution_factor = 0.05
        self._min_pos_x = -2.42
        self._min_pos_y = -0.95

        # This is the node graph step size for map
        self._graph_step_size = 0.01

        self._width = int(self._map_data.shape[1])
        self._height = int(self._map_data.shape[0])

        # Obtaining the max positions associated with map indexing
        self._max_pos_x, self._max_pos_y = self._find_max_pos(self._map_data.shape[1],self._map_data.shape[0])

        # Generating node graph of the map and getting the maximum index
        self._map_graph, self._max_index = self._generate_map_nodes()

        # Uncomment to see visualization of map and obstacle inflation
        # --------------------------------------
        # cv2.namedWindow("Inflation Map")
        # cv2.namedWindow("Original")
        # --------------------------------------

        # Inflate obstacle nodes to accomodate robot's size
        radius_size_m = 0.12
        self._inflate_obstacles(radius_size_m)

        # Uncomment to verify if position-to-index conversion is correct
        # --------------------------------------
        # print(self.verify_pos_index_conversion())
        # --------------------------------------
        
        # Starting ROS service
        self._server = self.create_service(RequestPath, 
                                          "path_planner",
                                          self.get_path_waypoints)
        
        print("\n")
        self.get_logger().info("Path Planner Server Node active")

    def verify_pos_index_conversion(self,debug=False):
        """
        Used to verify if all positions used to generate node graph
        is correctly represented by associated node index
        """
        current_pos_x = self._min_pos_x
        current_pos_y = self._min_pos_y
        count = 0
        while current_pos_y < self._max_pos_y:
            while current_pos_x < self._max_pos_x:
                index = self.get_graph_index(current_pos_x,current_pos_y,True)
                
                x = self._map_graph[index].x
                y = self._map_graph[index].y

                if debug == True:
                    print(current_pos_x,current_pos_y)
                    print(index)
                    print(x,y)

                # Prints out if position-to-index is incorrect
                if x != current_pos_x or y != current_pos_y:
                    print("Count: {}".format(count))
                    print("Current_pos_x = {} | Current_pos_y = {}".format(current_pos_x,current_pos_y))
                    print("Index x = {} | Index y = {}".format(x,y))
                    print("Index = {}".format(index))
                    return False

                current_pos_x = round(current_pos_x + self._graph_step_size,2)
            current_pos_x = self._min_pos_x
            current_pos_y = round(current_pos_y + self._graph_step_size,2)
        return True
        
    def get_path_waypoints(self,request,response):
        """
        Main function to run for request calls by clients
        """
        x_init = round(request.x_init,2)
        y_init = round(request.y_init,2)
        x_goal = round(request.x_goal,2)
        y_goal = round(request.y_goal,2)
        
        try:
            # Gets node index for start and goal positions and setting up initial values
            start_index = self._get_graph_index(x_init,y_init)
            self._map_graph[start_index].set_as_start()
            self._map_graph[start_index].set_cost(0.0)
            goal_index = self._get_graph_index(x_goal,y_goal)
            self._map_graph[goal_index].set_as_goal()

            # Checks if position is out-of-bounds from map
            if start_index < 0 or start_index > self._max_index:
                raise InvalidPositionException("The following position is out-of-bound of map: ({},{})".format(x_init,y_init))
            elif goal_index < 0 or goal_index > self._max_index:
                raise InvalidPositionException("The following position is out-of-bound of map: ({},{})".format(x_goal,y_goal))
            
            # Checks if positions given are an obstacle
            if self._map_graph[start_index].is_obstacle == True:
                raise ObstacleDetectedException("The following position is an obstacle: ({},{})".format(x_init,y_init))
            elif self._map_graph[goal_index].is_obstacle == True:
                raise ObstacleDetectedException("The following position is an obstacle: ({},{})".format(x_goal,y_goal))
            
            # Creating variabls to store indexes that needs to be visited or has already been visited
            indexes_to_visit = set()
            index_visited = set()

            # Set current index being visited
            current_index = start_index

            # Uncomment for visualization of path-planning
            # --------------------------------------
            # cv2.namedWindow("A* Path-Planning")
            # map_copy = self._map_data.copy()
            # --------------------------------------

            while current_index != goal_index:
                #Add current index to index visited
                index_visited.add(current_index)

                # Having some form of progression during path-planning
                nodes_visited = len(index_visited)
                sys.stdout.write("\r=== Nodes Visited: {} ===".format(nodes_visited))
                sys.stdout.flush()

                current_x = self._map_graph[current_index].x
                current_y = self._map_graph[current_index].y
                goal_x = self._map_graph[goal_index].x
                goal_y = self._map_graph[goal_index].y
                
                #Get adjacent indexes
                adjacent_indexes = self._get_adjacent_nodes(current_index)

                for index in adjacent_indexes:
                    # Add adjacent indexes to visit later if not already visited
                    if index not in index_visited and self._map_graph[index].is_obstacle == False:
                        indexes_to_visit.add(index)
                    
                    node_x = self._map_graph[index].x
                    node_y = self._map_graph[index].y

                    # Get local cost from current node to adjacent node
                    local_cost = self._calculate_euclidean_distance(current_x,current_y,node_x,node_y)
                    # Get heuristic cost from adjacent node to goal node
                    heuristic_cost = self._calculate_euclidean_distance(node_x,node_y,goal_x,goal_y)
                    # Combine the local and heuristic cost together along with current node's cost from start
                    total_cost = local_cost + heuristic_cost + self._map_graph[current_index].cost

                    #If the adjacent node is not linked to another node, link it to current node
                    if self._map_graph[index].linked_index == None and index not in index_visited:
                        self._map_graph[index].set_linked_index(current_index)

                    # If there is no cost for adjacent node, set it to total cost
                    if self._map_graph[index].cost == None:
                        self._map_graph[index].set_cost(total_cost)
                    #If the adjacent cost is greater than the current total cost, set it to total cost and relink to current node
                    elif self._map_graph[index].cost > total_cost:
                        self._map_graph[index].set_cost(total_cost)
                        self._map_graph[index].set_linked_index(current_index)



                # Get the lowest cost node from indexes to visit to visit next
                index_to_visit_next = self._get_lowest_cost_index(indexes_to_visit)

                # Remove index to visit next from indexes to visit later
                indexes_to_visit.remove(index_to_visit_next)
                # Set next index as current index
                current_index = index_to_visit_next

                # Uncomment for visualization
                # --------------------------------------
                # map_cell_pos_x, map_cell_pos_y = self.interpolate_map_cell_position(self._map_graph[current_index].x,self._map_graph[current_index].y)
                # map_copy = cv2.circle(map_copy,(map_cell_pos_x,map_cell_pos_y),1,(0,0,255),-1)
                # cv2.imshow("A* Path-Planning",map_copy)
                # cv2.waitKey(1)
                # --------------------------------------

            # Uncomment for visualization
            # --------------------------------------
            # cv2.destroyAllWindows()
            # --------------------------------------

            print("\n")
            print("[STATUS] Path to goal location found!")
            

            # Setting up variables for traceback to generate path
            traceback_index = goal_index
            traceback_list = []
            x_waypoints = []
            y_waypoints = []

            print("\n")
            while traceback_index != start_index:

                # Creating some form of progression and notifying how many nodes are used in path
                sys.stdout.write("\r=== Nodes traced {} out of {} nodes ===".format(len(traceback_list),self._max_index))
                sys.stdout.flush()

                x = self._map_graph[traceback_index].x
                y = self._map_graph[traceback_index].y
                traceback_list.insert(0,traceback_index)
                x_waypoints.insert(0,x)
                y_waypoints.insert(0,y)
                traceback_index = self._map_graph[traceback_index].linked_index
            
            # Uncomment for visualization of path generated
            # --------------------------------------
            # cv2.namedWindow("Path Generated")
            # map_copy = self._map_data.copy()

            # for current_index in traceback_list:
            #     map_cell_pos_x, map_cell_pos_y = self.interpolate_map_cell_position(self._map_graph[current_index].x,self._map_graph[current_index].y)
            #     map_copy = cv2.circle(map_copy,(map_cell_pos_x,map_cell_pos_y),1,(0,0,255),-1)

            # map_cell_pos_x, map_cell_pos_y = self.interpolate_map_cell_position(self._map_graph[start_index].x,self._map_graph[start_index].y)
            # map_copy = cv2.circle(map_copy,(map_cell_pos_x,map_cell_pos_y),3,(255,0,0),-1)
            # map_cell_pos_x, map_cell_pos_y = self.interpolate_map_cell_position(self._map_graph[goal_index].x,self._map_graph[goal_index].y)
            # map_copy = cv2.circle(map_copy,(map_cell_pos_x,map_cell_pos_y),3,(0,255,0),-1)
            
            # cv2.imshow("Path Generated",map_copy)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            # -------------------------------------

            response.is_successful = True
            response.x_waypoints = x_waypoints
            response.y_waypoints = y_waypoints
            return response

        except InvalidPositionException as e:
            print(e)
            response.is_successful = False
            response.x_waypoints = []
            response.y_waypoints = []
            return response
        except ObstacleDetectedException as e:
            print(e)
            response.is_successful = False
            response.x_waypoints = []
            response.y_waypoints = []
            return response
        finally:
            self.reset_all_nodes()

    def _generate_map_nodes(self):
        """
        Generates a node graph of the map
        """
        map_graph = {}
        map_index = 0
        current_pos_x = self._min_pos_x
        current_pos_y = self._min_pos_y
        
        for j in range(int((self._max_pos_y - self._min_pos_y)*100)):
            for i in range(int((self._max_pos_x-self._min_pos_x)*100)):
                map_graph[map_index] = MapNode(current_pos_x,current_pos_y,map_index)
                map_cell_pos_x, map_cell_pos_y = self._interpolate_map_cell_position(current_pos_x,current_pos_y)
                
                is_obstacle = self._check_for_obstacle(map_cell_pos_x,map_cell_pos_y)

                if is_obstacle:
                    map_graph[map_index].set_obstacle()
                
                current_pos_x = round(current_pos_x+ self._graph_step_size,2)
                map_index += 1
            current_pos_x = self._min_pos_x
            current_pos_y = round(current_pos_y+ self._graph_step_size,2)

        print("=== Map Generated ===")
        return map_graph,map_index
        
    def _find_max_pos(self,width,height):
        """
        Using the parameters of the map's .pgm and .yaml files,
        get the max position in the x and y axes
        """
        max_pos_x = round(width*self._resolution_factor + self._min_pos_x,2)-0.01
        max_pos_y = round(height*self._resolution_factor + self._min_pos_y,2)-0.01
        return max_pos_x,max_pos_y

    def _check_for_obstacle(self,x,y):
        """
        Checks if the position in the map is an obstacle (black) or not (white/gray)
        """
        value = np.array(self._map_data[y][x])
        return np.array_equal(value,np.zeros(3))
        
    def _interpolate_map_cell_position(self,x,y):
        """
        Interpolates the map cell from a position
        """
        map_cell_pos_x = int((x - self._min_pos_x)/self._resolution_factor)
        map_cell_pos_y = int(self._height-1) - int((y - self._min_pos_y)/self._resolution_factor)
        return map_cell_pos_x,map_cell_pos_y
    
    def _inflate_obstacles(self,radius):
        """
        Find obstacle nodes and inflate them by making nearby nodes with a radius
        obstacles as well
        """
        # Setting a queue of all known obstacle nodes
        queue = []
        # Search through all indexes for obstacles and append to queue
        for index in range(self._max_index):
            if self._map_graph[index].is_obstacle:
                queue.append(index)
        
        counter = 1
        max_counter = len(queue)
        for index in queue:
            # Creating a form of progression for obstacle inflation
            sys.stdout.write("\r=== Obstacle Inflation Progress: {}% ===".format(int(counter/max_counter*100)))
            sys.stdout.flush()

            x = self._map_graph[index].x
            y = self._map_graph[index].y

            # Get the dimensions of the area to focus on
            local_min_x = round(x - radius,2)
            local_min_y = round(y - radius,2)
            local_max_x = round(x + radius,2)
            local_max_y = round(y + radius,2)

            # Obtain the node indexes of all the map nodes in the local area focused on
            local_graph_indexes = self._get_local_graph_indexes(local_min_x,local_min_y,local_max_x,local_max_y)
            
            # Uncomment for visualization of local areas searched
            # --------------------------------------
            # map_copy = self._map_data.copy()
            # --------------------------------------
            for index in local_graph_indexes:
                local_x = self._map_graph[index].x
                local_y = self._map_graph[index].y
                within_radius = self._calculate_inflation_radius_pos(local_x,local_y,x,y,radius)
                if within_radius:
                    self._map_graph[index].set_obstacle()

                    # Uncomment for visualization of local areas searched
                    # --------------------------------------
                    # map_cell_pos_x, map_cell_pos_y = self._interpolate_map_cell_position(local_x,local_y)
                    # map_copy = cv2.circle(map_copy,(map_cell_pos_x,map_cell_pos_y),1,(0,0,255),-1)
                    # cv2.imshow("map",map_copy)
                    # cv2.waitKey(1)
                    # --------------------------------------
            
            counter+=1

        # Uncomment for visualization of obstacle inflation
        # --------------------------------------
        # for index in range(self._max_index):    
        #     if self._map_graph[index].is_obstacle:
        #         x = self._map_graph[index].x
        #         y = self._map_graph[index].y
        #         map_cell_pos_x, map_cell_pos_y = self._interpolate_map_cell_position(x,y)
        #         map_copy = cv2.circle(map_copy,(map_cell_pos_x,map_cell_pos_y),1,(255,0,0),-1)
        #         cv2.imshow("map",map_copy)
        #         cv2.waitKey(1)

        # for index in queue:    
        #     x = self._map_graph[index].x
        #     y = self._map_graph[index].y
        #     map_cell_pos_x, map_cell_pos_y = self._interpolate_map_cell_position(x,y)
        #     map_copy = cv2.circle(map_copy,(map_cell_pos_x,map_cell_pos_y),1,(0,0,255),-1)

        # cv2.imshow("Inflation Map",map_copy)
        # cv2.waitKey(1)

        # cv2.imshow("Original",self._map_data)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # --------------------------------------

    def _get_local_graph_indexes(self,min_x,min_y,max_x,max_y):
        """
        Takes the min and max positions for the x and y axes to determine
        the local node indexes within the area
        """
        graph_indexes = []
        current_x = min_x
        current_y = min_y
        while current_y < max_y:
            while current_x < max_x:
                index = self._get_graph_index(current_x,current_y)
                if index > 0 and index < self._max_index:
                    graph_indexes.append(index)
                current_x = round(current_x+self._graph_step_size,2)
            current_x = min_x
            current_y = round(current_y+self._graph_step_size,2)
        
        return graph_indexes

    def _get_graph_index(self,x,y,debug=False):
        """
        Converts a position into its associated node index in the map
        """
        # Checks if the position is within the map, if not return a -1
        if x < self._min_pos_x or x > self._max_pos_x:
            return -1
        if y < self._min_pos_y or y > self._max_pos_y:
            return -1
        
        index_range_x = int((self._max_pos_x-self._min_pos_x)*100)
        dist_x = int(round(round(x-self._min_pos_x,2)*100))
        dist_y = int(round(round(y-self._min_pos_y,2)*100))
        if debug:
            print("x = {} | y = {}".format(x,y))
            print("Dist x = {} | Dist y = {}".format(dist_x,dist_y))
            print("Index range = {}".format(index_range_x))
            print("Result Index = {}".format((dist_y*index_range_x)+dist_x))
        return (dist_y*index_range_x)+dist_x

    def _calculate_inflation_radius_pos(self,x,y,center_x,center_y,radius):
        """
        Calculates if the position is within the inflation radius to be made as
        an obstacle or not
        """
        euclidean_dist = self._calculate_euclidean_distance(x,y,center_x,center_y)
        if euclidean_dist > radius:
            return False
        return True
    
    def _calculate_euclidean_distance(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(abs(x2-x1),2) + math.pow(abs(y2-y1),2))
    
    def _get_adjacent_nodes(self,current_index):
        """
        Gets the adjacent node index surrounding the current index given
        """
        x = self._map_graph[current_index].x
        y = self._map_graph[current_index].y

        node_list = []
        adjacent_nodes = self._calculate_adjacent_node_positions(x,y)
        for adj_x,adj_y in adjacent_nodes:
            adjacent_index = self._get_graph_index(adj_x,adj_y)
            if adjacent_index > 0 and adjacent_index <= self._max_index:
                node_list.append(adjacent_index)

        return node_list

    def _calculate_adjacent_node_positions(self,x,y):
        """
        Calculates the adjacent node positions from the position of the current node
        """
        pos_list = []
        left_x = x - self._graph_step_size
        left_y = y + self._graph_step_size
        pos_list.append((left_x,left_y))
        for j in range(2):
            left_y -= self._graph_step_size
            pos_list.append((left_x,left_y))

        middle_x = x
        middle_y = y + self._graph_step_size
        pos_list.append((middle_x,middle_y))
        pos_list.append((middle_x,middle_y-(self._graph_step_size*2)))

        right_x = x + self._graph_step_size
        right_y = y + self._graph_step_size
        pos_list.append((right_x,right_y))
        for j in range(2):
            right_y -= self._graph_step_size
            pos_list.append((right_x,right_y))
        
        return pos_list
        
    def _get_lowest_cost_index(self,indexes):
        """
        From a list of node indexes, finds the index with the lowest cost 
        """
        cost = 100000
        index_to_visit_next = None
        for index in indexes:
            if self._map_graph[index].cost < cost:
                cost = self._map_graph[index].cost
                index_to_visit_next = index
        return index_to_visit_next
    
    def reset_all_nodes(self):
        """
        Resets all the attributes for all map nodes for next path-planning
        """
        print("\n")
        for index in range(self._max_index):
            sys.stdout.write("\r=== Nodes Reset Progress: {}% ===".format(int((index+1)/self._max_index*100)))
            sys.stdout.flush()
            self._map_graph[index].reset()

        print("\n")
        print("[STATUS] Ready for next service call!")

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerServerNode("robot_caller_server")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()