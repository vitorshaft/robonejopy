import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CorridorFollower(Node):
    def __init__(self):
        super().__init__('corridor_follower')
        self.publisher_cmd_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.publisher_img_nav = self.create_publisher(Image, '/imagemNav', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.linear_speed = 0.2
        self.kp_angular = 0.005

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        twist = Twist()
        twist.linear.x = self.linear_speed

        midpoint = self.process_image(frame)
        if midpoint is not None:
            error = midpoint - frame.shape[1] / 2
            twist.angular.z = -self.kp_angular * error
        else:
            twist.angular.z = 0.0

        self.publisher_cmd_vel.publish(twist)

        # Publish the image with detections
        image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_img_nav.publish(image_msg)

    def process_image(self, frame):
        # Parameters for brightness and contrast adjustment
        contrast = 75
        brightness = 75
        # Adjust brightness and contrast
        adjusted_frame = cv2.convertScaleAbs(frame, alpha=contrast / 50.0, beta=brightness - 50)

        # Convert to grayscale
        gray = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2GRAY)
        
        # Canny edge detection parameters
        canny_thresh1 = 5
        canny_thresh2 = 95
        edges = cv2.Canny(gray, canny_thresh1, canny_thresh2)

        # HoughLinesP parameters
        hough_thresh = 15
        min_line_length = 30
        max_line_gap = 10
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=hough_thresh, minLineLength=min_line_length, maxLineGap=max_line_gap)
        
        if lines is not None:
            left_lines = []
            right_lines = []
            frame_center = frame.shape[1] / 2
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) > 0.5:  # Ignore nearly horizontal lines
                    if x1 < frame_center and x2 < frame_center:
                        left_lines.append(line[0])
                    elif x1 > frame_center and x2 > frame_center:
                        right_lines.append(line[0])

            left_midpoint = self.calculate_midpoint(left_lines)
            right_midpoint = self.calculate_midpoint(right_lines)

            if left_midpoint is not None and right_midpoint is not None:
                midpoint = (left_midpoint + right_midpoint) // 2
                # Draw lines
                for line in left_lines + right_lines:
                    x1, y1, x2, y2 = line
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # Draw midpoint
                cv2.circle(frame, (midpoint, frame.shape[0] // 2), 5, (255, 0, 0), -1)
                return midpoint

        return None

    def calculate_midpoint(self, lines):
        if len(lines) == 0:
            return None
        x_values = [line[0] for line in lines] + [line[2] for line in lines]
        return sum(x_values) // len(x_values)

def main(args=None):
    rclpy.init(args=args)
    node = CorridorFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
