import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node

class ImageSubscriberNode(Node):
    """
    Node that grabs video frames from camera and publishes it to ROS.
    """
    def __init__(self,name,sub_name):
        super().__init__(name)
        self.image_subscriber = self.create_subscription(Image,sub_name,self.image_callback,10)
        self.get_logger().info("Image Subscriber Node active")
        self.bridge = CvBridge()

    def image_callback(self,msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow("Camera Subscription", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    node = ImageSubscriberNode("image_subscriber_node","video_frame")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
