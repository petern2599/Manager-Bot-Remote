import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class MapLocation():
    def __init__(self,x,y):
        self.x = x
        self.y = y

class MapLocationList():
    def __init__(self,name,location_array):
        self.name = name
        self.locations = location_array

class MapRecorderNode(Node):
    """
    Node that grabs video frames from camera and publishes it to ROS.
    """
    def __init__(self,name,map_pos_name):
        super().__init__(name)
        self.map_pos_subscriber = self.create_subscription(Vector3,map_pos_name,self.map_pos_callback,10)
        self.setup_subscriber_variables()
        self.recorded_location_dict = dict()
        self.name_input = ""
        self.get_logger().info("Map Recorder Node active")

        while True:
            if self.name_input == "":
                print("\n==========================")
                print("Add name to recorded map locations")
                print('To quit, just enter nothing')
                print("==========================\n")
                self.name_input = input("Enter name of recorded location: ")
                location_list = []
                if self.name_input == "":
                    break
            else:
                print("\n==========================")
                print("To append current location, press 'r'")
                print("Otherwise, to stop recording, press 'q'")
                print("==========================\n")
                command_input = input("Enter command: ")
                
                if command_input == 'r':
                    print("Got position")
                    location = MapLocation(self.x,self.y)
                    location_list.append(location)
                elif command_input == 'q':
                    print("Ending recording...")
                    self.recorded_location_dict[self.name_input] = MapLocationList(self.name_input,location_list)
                    print("Currently recorded {} entries".format(len(self.recorded_location_dict)))
                    self.name_input = ""

        exit(0)
        
    def setup_subscriber_variables(self):
        self.x = 0.0
        self.y = 0.0

    def map_pos_callback(self,msg):
        self.x = msg.x
        self.y = msg.y

            

def main(args=None):
    rclpy.init(args=args)

    node = MapRecorderNode("map_recorder_node","video_frame")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        