import rclpy
from rclpy.node import Node
from inventory_manager_bot.inventory_db_connector import InventoryDBConnector
from inventory_management_interfaces.msg import DetectedLabels
from datetime import date

class InventoryValidatorNode(Node):
    def __init__(self,name,sub_name):
        super().__init__(name)
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

        self.label_subscriber = self.create_subscription(DetectedLabels,sub_name,self.labels_callback,10)

    def labels_callback(self,msg):
        detected_labels = msg.labels
        for label in detected_labels:
            result = self.conn.check_if_product_no_exists(label)
            print(type(date.today()))
            if result == None:
                self.get_logger().info("[WARNING] Unknown item detected: {}".format(label))
            elif result != None and result[7] > date.today():
                self.get_logger().info("[INFO] Item found in database: {}".format(label))
            elif result != None and result[7] < date.today():
                self.get_logger().info("[WARNING] Item found in database is past checkout date: {}".format(label))
            elif result != None and result[7] == date.today():
                self.get_logger().info("[WARNING] Item found in database and today is checkout date: {}".format(label))
                #Code for validating placement
            

    
    

def main(args=None):
    rclpy.init(args=args)
    node = InventoryValidatorNode("inventory_validation_node","detected_labels")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()