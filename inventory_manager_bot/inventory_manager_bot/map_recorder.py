import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import json
import os

class MapRecorderNode(Node):
    """
    Node that grabs video frames from camera and publishes it to ROS.
    """
    def __init__(self,name,map_pos_name):
        super().__init__(name)
        self._map_pos_subscriber = self.create_subscription(Vector3,map_pos_name,self._map_pos_callback,10)
        self._setup_subscriber_variables()
        self._name_input = ""
        self._location_list = []
        self._point1 = None
        self._point2 = None
        self._location_updated = False

        with open("/home/petern25/ros2_ws/src/inventory_manager_bot/config/poi.json","r") as json_file:
            self._recorded_location_dict = json.load(json_file)

        self.get_logger().info("Map Recorder Node active")

        self._timer = self.create_timer(0.1,self._record)

        
    def _record(self):
        if self._point2 != None:
            print("\n[STATUS]Location has been appended\n")
            self._point2 = None
        if self._name_input == "":
            os.system("clear")
            print("\n==========================")
            print("Add name to recorded map locations")
            print('To quit, just enter nothing')
            print("==========================\n")
            self._name_input = input("Enter name of recorded location: ")
            self._location_list = []
            os.system("clear")
            if self._name_input == "":
                poi_json = json.dumps(self._recorded_location_dict,indent=4)
                with open("/home/petern25/ros2_ws/src/inventory_manager_bot/config/poi.json","w") as outfile:
                    outfile.write(poi_json)
                exit(0)
        else:
            if self._point1 == None:
                if self._location_updated == True:
                    print("\n[STATUS]Location has been updated\n")
                    print("\n==========================")
                    print("First Location of {} (1/2)".format(self._name_input))
                    print("----------------------------")
                    print("Now press 'r' to append location")
                    print("==========================\n")
                    command_input = input("Enter command: ")  
                    if command_input == 'r':
                        self._point1 = (self._x,self._y)
                        self._location_list.append(self._point1)
                        self._location_updated = False
                        os.system('clear')
                else:
                    print("\n==========================")
                    print("First Location of {} (1/2)".format(self._name_input))
                    print("----------------------------")
                    print("Press 'u' to update location")
                    print("==========================\n")
                    command_input = input("Enter command: ")        
                    if command_input == 'u':
                        self._location_updated = True
                        os.system('clear')
                
            else:
                if self._location_updated == True:
                    print("\n[STATUS]Location has been updated\n")
                    print("\n==========================")
                    print("Second Location of {} (2/2)".format(self._name_input))
                    print("----------------------------")
                    print("Now press 'r' to append location")
                    print("==========================\n")
                    command_input = input("Enter command: ")  
                    if command_input == 'r':
                        self._point2 = (self._x,self._y)
                        self._location_list.append(self._point2)
                        self._recorded_location_dict[self._name_input] = self._location_list
                        self._point1 = None
                        self._name_input=""
                        self._location_updated = False
                        os.system('clear')
                else:
                    print("\n[STATUS]Location has been appended\n")
                    print("\n==========================")
                    print("Second Location of {} (2/2)".format(self._name_input))
                    print("----------------------------")
                    print("Press 'u' to update location")
                    print("==========================\n")
                    command_input = input("Enter command: ")        
                    if command_input == 'u':
                        self._location_updated = True
                        os.system('clear')
                        




    def _setup_subscriber_variables(self):
        self._x = 0.0
        self._y = 0.0

    def _map_pos_callback(self,msg):
        self._x = round(msg.x,2)
        self._y = round(msg.y,2)
            

def main(args=None):
    rclpy.init(args=args)

    node = MapRecorderNode("map_recorder_node","map_pos")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        