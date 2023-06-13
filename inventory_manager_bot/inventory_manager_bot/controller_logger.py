import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import pandas as pd
import datetime
import time

class ControlLoggerNode(Node):
    """
    Node that subscribes to raw IMU data and log it in a csv
    """
    def __init__(self,name,sub_name_angle):
        super().__init__(name)
        #Declaring Parameters for accelerometer offsets to calibrate readings
        self.declare_parameter("path")
        self.csv_path = self.get_parameter("path").value

        #Initializing subscribers to raw data
        self.angle_subscriber = self.create_subscription(Vector3,sub_name_angle,self._angle_callback,10)

        self._setup_initial_values()

        #Create timer to log imu data
        self.timer = self.create_timer(0.01,self._log_pwm_data)
        self.get_logger().info("Serial Logger Node active")

    def _setup_initial_values(self):
        """
        Set up initial values for using trapezoid rule for numerical integration,
        subscrbed kalman values, and the headers used for the dataframe
        """
        self._angle_z = 0.0
        self._prev_time = time.time()

        self._df = pd.DataFrame(columns=['dt','heading'])

    def _angle_callback(self,msg):
        self._angle_z = msg.z

    def _log_pwm_data(self):
        """
        Creates a new row of obtained data and appends it to dataframe
        """
        current_time = time.time()
        
        self._df.loc[len(self._df.index)] = [current_time-self._prev_time, self._angle_z]
        
        self._prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ControlLoggerNode("control_logger_node","map_angle")

        rclpy.spin(node)

        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Saving dataframe to csv...")
        node._df.drop([0])
        node._df.to_csv(node.csv_path+str(datetime.datetime.now())+".csv",index=False)

if __name__ == "__main__":
    main()
        
