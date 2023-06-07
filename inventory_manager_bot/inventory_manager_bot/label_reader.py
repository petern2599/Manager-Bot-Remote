import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from inventory_management_interfaces.msg import DetectedLabels
from pyzbar.pyzbar import decode
import numpy as np

class LabelReaderNode(Node):
    """
    Node that grabs video frames from camera and publishes it to ROS.
    """
    def __init__(self,name,sub_name0,pub_name):
        super().__init__(name)
        #Subscribe to image publisher
        self.image_subscriber0 = self.create_subscription(Image,sub_name0,self.image_callback0,10)
        #Create publisher to publish detected labels
        self.label_publisher = self.create_publisher(DetectedLabels,pub_name,10)
        self.get_logger().info("Label Reader Node active")

        #Set up CVBridge to convert Image msg to cv2 format
        self.bridge = CvBridge()

        #Create QR detector
        #self.detector = cv2.QRCodeDetector()

        cv2.namedWindow("Camera Subscription",cv2.WINDOW_AUTOSIZE)
        self.labels_msg = DetectedLabels()
        self.detected_labels = []
        self.detection_points = []

    def image_callback0(self,msg):
        #Convert frame from Image msg to cv2 format
        frame = self.bridge.imgmsg_to_cv2(msg)
        #Detect QR codes and decode values
        # det_ret, values, points, straight_qrcodes = self.detector.detectAndDecodeMulti(frame)
        result = decode(frame)
        if len(result) != 0:
            for detection in result:
                label_value = detection.data.decode('utf-8')
                self.detected_labels.append(label_value)
                
                points = np.array(detection.polygon,np.int32)
                self.detection_points.append(points)
                
            
        #Push labels array to msg and publish
        self.labels_msg.labels = self.detected_labels
        self.label_publisher.publish(self.labels_msg)

        #Draw detection box on frame
        if len(result) != 0:
            new_frame = self.draw_detection_box(self.detected_labels,self.detection_points,frame)
            cv2.imshow("Camera Subscription", new_frame)
            cv2.waitKey(1)
        else:
            cv2.imshow("Camera Subscription", frame)
            cv2.waitKey(1)

        #Clear array for next frame
        self.detected_labels.clear()
        self.detection_points.clear()

    def draw_detection_box(self,values,points,img):
        for label_count in range(len(values)):
            cv2.polylines(img,[points[label_count]],True,(255,0,0),4)

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_size = 0.3
            font_thickness = 1

            x_points = []
            y_points = []
            for i in range(4):
                x_points.append(points[label_count][i][0])
                y_points.append(points[label_count][i][1])
                
            x_min = min(x_points)
            x_max = max(x_points)
            y_min = min(y_points)
            y_max = max(y_points)

            pos_x =  x_max - int((x_max-x_min)*.90)
            pos_y =  y_max - int((y_max-y_min)/2)

            
            cv2.putText(img,values[label_count],(pos_x,pos_y), 
                            font,
                            font_size, 
                            (0,0,255), 
                            font_thickness, 
                            lineType = cv2.LINE_AA)
        return img

    def __del__(self):
        print("Destroying all windows")
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)

    node = LabelReaderNode("image_subscriber_node","video_frame","detected_labels")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
