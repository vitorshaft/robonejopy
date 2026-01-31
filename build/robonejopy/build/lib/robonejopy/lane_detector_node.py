import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from .lane_detection import LaneDetection
import cv2  # Certifique-se de importar o OpenCV aqui

class LaneDetectFollower(Node):
    def __init__(self):
        super().__init__('lane_detect_follower_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.camera_callback, 
            1
        )
        
        # Adicionando um publisher para a imagem processada
        self.processed_image_pub = self.create_publisher(Image, '/camera/processed_image', 1)

        self.declare_parameter('Kp', 0.01)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.05)
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value

        self.wheel_base = 0.42  # m
        self.wheel_radius = 0.0625

        self.max_desire_linear_vel = 0.3  # m/s
        self.max_omega = 33.51  # rad/s
        self.last_cte = 0

        self.bridge = CvBridge()
        self.counter = 0

    def camera_callback(self, data):
        self.get_logger().info("Received image from /camera/image_raw")

        self.counter += 1
        if self.counter % 3 != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if cv_image is None or cv_image.size == 0:
                self.get_logger().error("Empty image received")
                return
        except CvBridgeError as e:
            self.get_logger().error(f"Error conversion to CV2: {e}")
            return

        lane_detection_object = LaneDetection()
        cte, angle, final_img = lane_detection_object.processImage(cv_image)

        if final_img is None or final_img.size == 0:
            self.get_logger().error("Empty processed image")
            return

        cmd_vel = Twist()
        if cte is not None and angle is not None:
            angular_z = self.Kp * cte + self.Kd * (cte - self.last_cte)
            self.last_cte = cte
            linear_x = self.max_desire_linear_vel
            angular_z = max(min(angular_z, 2.0), -2.0)
        else:
            angular_z = self.Kp * self.last_cte * 1.9
            linear_x = self.max_desire_linear_vel
            angular_z = max(min(angular_z, 2.0), -2.0)

        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel)

        try:
            processed_image_msg = self.bridge.cv2_to_imgmsg(final_img, encoding="bgr8")
            self.processed_image_pub.publish(processed_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting processed image to ROS Image message: {e}")

    def clean_up(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    lane_detect_follower_object = LaneDetectFollower()
    try:
        rclpy.spin(lane_detect_follower_object)
    finally:
        lane_detect_follower_object.clean_up()
        lane_detect_follower_object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
