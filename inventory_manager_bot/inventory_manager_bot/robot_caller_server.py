import rclpy
from rclpy.node import Node
from inventory_management_interfaces.srv import CallRobotToWaitingArea
from inventory_manager_bot.inventory_db_connector import InventoryDBConnector

class RobotCallerServerNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.declare_parameter("robots")
        self.total_number_of_robots = self.get_parameter("robots").value
        self.declare_parameter("waiting_areas")
        self.total_number_of_waiting_areas = self.get_parameter("waiting_areas").value
        self.declare_parameter("user")
        user = self.get_parameter("user").value
        self.declare_parameter("password")
        password = self.get_parameter("password").value
        self.declare_parameter("host")
        host = self.get_parameter("host").value
        self.declare_parameter("port")
        port = self.get_parameter("port").value
        self.declare_parameter("db")
        db = self.get_parameter("db").value

        self.conn = InventoryDBConnector()
        self.conn.connect_to_db(db,host,user,password,port)


        self.server = self.create_service(CallRobotToWaitingArea, 
                                          "robot_caller",
                                          self.call_robot_to_waiting_area)
        
        self.get_logger().info("Caller Server Node active")

        self.test_dict = {"robot_0":True,"robot_1":False}
        self.test_dict2 = {"area_0":False,"area_1":True}

    
    def set_handler(self,user_number):
        """
        Designate the appropriate user that the robot
        will attend to when it is in an idle state.
        """
        print("Handler set to: {}".format(user_number))

    def find_available_robot(self):
        """
        Select a robot that is in an idle state to attend
        to user.
        """
        for i in range(self.total_number_of_robots):
            print("Checking robot {}".format(i))
            if self.test_dict["robot_{}".format(i)] == True:
                print("robot_{}".format(i) + " is available")
                selected_robot = "robot_{}".format(i)
                break
        
        return selected_robot
    
    def get_waiting_area_number(self):
        """
        Get the waiting area number to meet with the robot.
        """
        for i in range(self.total_number_of_waiting_areas):
            print("Checking waiting area {}".format(i))
            if self.test_dict2["area_{}".format(i)] == True:
                print("area_{}".format(i) + " is available")
                selected_waiting_area = i
                break
        
        return selected_waiting_area
    
    def get_aisles(self,queue):
        aisles_to_search = set()
        for item in queue:
            aisle = self.conn.get_aisle_from_product_no(item.split('-')[1])
            aisles_to_search.add(aisle[0])
        
        print(aisles_to_search)

    def call_robot_to_waiting_area(self,request,response):
        """
        Sends the location to the selected robot to go to.
        """
        self.set_handler(request.user_number)
        self.get_aisles(request.queue)
        response.selected_robot = self.find_available_robot()
        response.waiting_area_number = self.get_waiting_area_number()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotCallerServerNode("robot_caller_server")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()