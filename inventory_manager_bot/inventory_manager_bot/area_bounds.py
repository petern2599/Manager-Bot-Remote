import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import json
import math

class AreaBoundsNode(Node):
    """
    Node that grabs video frames from camera and publishes it to ROS.
    """
    def __init__(self,name,map_pos_name,area_name):
        super().__init__(name)
        self.map_pos_subscriber = self.create_subscription(Vector3,map_pos_name,self.map_pos_callback,10)
        self.area_publisher = self.create_publisher(String,area_name,10)
        self.setup_subscriber_variables()
        self.closest_area = ""
        with open("/home/petern25/ros2_ws/src/inventory_manager_bot/config/poi.json","r") as json_file:
            self.recorded_location_dict = json.load(json_file)

        self.distance_threshold = 0.25

        self.get_logger().info("Area Bounds Node active")
        self.timer = self.create_timer(0.01,self.check_area)
        
    def setup_subscriber_variables(self):
        self.x = 0.0
        self.y = 0.0

    def map_pos_callback(self,msg):
        self.x = round(msg.x,2)
        self.y = round(msg.y,2)
    
    def check_area(self):
        is_in_bounds = False
        if self.closest_area == "":
            self.scan_dictionary()
            if self.closest_area != "":
                is_in_bounds = self.check_in_bounds()
        else:
            is_in_bounds = self.check_in_bounds()

        if is_in_bounds == False:
            self.closest_area = ""
            
        msg = String()
        msg.data = self.closest_area
        self.area_publisher.publish(msg)

    def scan_dictionary(self):
        poi_checked = 0
        for poi,locations in self.recorded_location_dict.items():
            entrance_distance = self._calculate_euclidean_distance(self.x,self.y,locations[0][0],locations[0][1])
            exit_distance = self._calculate_euclidean_distance(self.x,self.y,locations[1][0],locations[1][1])
            if entrance_distance <= self.distance_threshold or exit_distance <= self.distance_threshold:
                self.closest_area = poi
                break
            else:
                poi_checked += 1

        if poi_checked == len(self.recorded_location_dict.keys()):
            self.closest_area = ""

    def check_in_bounds(self):
        first_point = (self.recorded_location_dict[self.closest_area][0][0],self.recorded_location_dict[self.closest_area][0][1])
        second_point = (self.recorded_location_dict[self.closest_area][1][0],self.recorded_location_dict[self.closest_area][1][1])

        first_point_distance = self._calculate_euclidean_distance(self.x,self.y,first_point[0],first_point[1])
        second_point_distance = self._calculate_euclidean_distance(self.x,self.y,second_point[0],second_point[1])
        distance_between_points = self._calculate_euclidean_distance(first_point[0],first_point[1],second_point[0],second_point[1])

        if first_point_distance > distance_between_points or second_point_distance > distance_between_points:
            return False
        else:
            return True
        
    def _calculate_euclidean_distance(self,x1,y1,x2,y2):
        return math.sqrt(math.pow(abs(x2-x1),2) + math.pow(abs(y2-y1),2))
    
    def _calculate_angle(self,point1,point2):
        dist_x = point2[0] - point1[0]
        dist_y = point2[1] - point1[1]
        angle = math.degrees(math.atan2(dist_x,dist_y))
        if angle >=0:
            return angle
        else:
            return 360 + angle
        
            

def main(args=None):
    rclpy.init(args=args)

    node = AreaBoundsNode("area_bounds_node","map_pos","area")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        