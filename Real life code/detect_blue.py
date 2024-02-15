import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point 

class BlueContainerDetector(Node):
    def __init__(self):
        super().__init__('blue_container_detector')
        self.subscription = self.create_subscription(
            Image,
            # '/camera/image_raw',
            '/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # Create a publisher for midpoint
        self.midpoint_publisher = self.create_publisher(Point, 'blue_container_midpoint', 10)

    def listener_callback(self, data):
        try:
            # Convert the ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert to HSL
        hsv_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)

        # Define range of blue color in HSL
        # lower_blue = np.array([95, 50, 50])
        # upper_blue = np.array([115, 255, 255])
        lower_blue= np.array([88, 5, 245])
        upper_blue=np.array([100, 50, 255])
        
        # Threshold the HSL image to get only blue colors
        mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
        # mask = cv2.bitwise_and(cv2_img, cv2_img, mask = masks)
        # Get image dimensions
        height, width = mask.shape[:2]

        # Ignore upper third of the image
        mask[:height//4, :] = 0

        # Ignore 10% from the left and 10% from the right
        mask[:, :width//20] = 0
        mask[:, -width//20:] = 0
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_size_threshold = 2000

        for contour in contours:
            if cv2.contourArea(contour) < contour_size_threshold:
                continue
            # Find the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Calculate the midpoint
            mid_point = Point()
            mid_point.x = x + w // 2.0
            mid_point.y = y * 1.0

            w_h_ratio = w / h
            if 0.3 <= w_h_ratio <= 0.482 and (y + h) > 2 * height // 3 and y < 2 * height // 3:
                # You can now use these values for your servoing algorithm
                print("Midpoint:", mid_point.x, mid_point.y)

                # Publish the midpoint
                self.midpoint_publisher.publish(mid_point)

                # Optionally, draw the bounding box and midpoint on the image
                cv2.rectangle(cv2_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(cv2_img, (int(mid_point.x), int(mid_point.y)), 5, (0, 0, 255), -1)

                print('width', w)
                print('height', h)
                print('ratio', w/h)
                
            # Display the resulting frame
            cv2.imshow('Frame', cv2_img)
            cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)
    print("hello")
    blue_container_detector = BlueContainerDetector()
    rclpy.spin(blue_container_detector)
    blue_container_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
