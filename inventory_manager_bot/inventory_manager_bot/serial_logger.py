import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import pandas as pd
import datetime
import time

class SerialLoggerNode(Node):
    """
    Node that subscribes to raw IMU data and log it in a csv
    """
    def __init__(self,name,sub_name_accel,sub_name_gyro,sub_name_vel):
        super().__init__(name)
        #Declaring Parameters for accelerometer offsets to calibrate readings
        self.declare_parameter("path")
        self.csv_path = self.get_parameter("path").value

        #Initializing subscribers to raw data
        self.raw_accel_subscriber = self.create_subscription(Vector3,sub_name_accel,self._accel_callback,10)
        self.raw_gyro_subscriber = self.create_subscription(Vector3,sub_name_gyro,self._gyro_callback,10)
        self.raw_vel_subscriber = self.create_subscription(Float64,sub_name_vel,self._vel_callback,10)

        self._setup_initial_values()

        #Create timer to log imu data
        self.timer = self.create_timer(0.01,self._log_imu_data)
        self.get_logger().info("Serial Logger Node active")

    def _setup_initial_values(self):
        """
        Set up initial values for using trapezoid rule for numerical integration,
        subscrbed kalman values, and the headers used for the dataframe
        """
        self._accel_x = 0.0
        self._accel_y = 0.0
        self._accel_z = 0.0

        self._ang_vel_x = 0.0
        self._ang_vel_y = 0.0
        self._ang_vel_z = 0.0

        self._vel = 0.0
        self._prev_time = time.time()

        self._df = pd.DataFrame(columns=['dT','Accel_x','Accel_y','Accel_z','Gyro_x','Gyro_y','Gyro_z','Vel'])

    def _accel_callback(self,msg):
        self._accel_x = round(msg.x,2)
        self._accel_y = round(msg.y,2)
        self._accel_z = round(msg.z,2)
    
    def _gyro_callback(self,msg):
        self._ang_vel_x = round(msg.x,2)
        self._ang_vel_y = round(msg.y,2)
        self._ang_vel_z = round(msg.z,2)

    def _vel_callback(self,msg):
        self._vel = round(msg.data,2)

    def _log_imu_data(self):
        """
        Creates a new row of obtained data and appends it to dataframe
        """
        current_time = time.time()
        
        self._df.loc[len(self._df.index)] = [current_time-self._prev_time, self._accel_x, self._accel_y, self._accel_z, \
                                              self._ang_vel_x,self._ang_vel_y,self._ang_vel_z,self._vel]
        
        self._prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SerialLoggerNode("imu_logger_node","raw_accel","raw_gyro",'raw_vel')

        rclpy.spin(node)

        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Saving dataframe to csv...")
        node._df.drop([0])
        node._df.to_csv(node.csv_path+str(datetime.datetime.now())+".csv",index=False)

if __name__ == "__main__":
    main()
        
