import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraImagePublisher(Node):
    def __init__(self, camera_url):
        super().__init__('camera_image_publisher')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.capture = cv2.VideoCapture(camera_url)
        if not self.capture.isOpened():
            self.get_logger().error(f"Failed to open camera at {camera_url}")
        else:
            self.get_logger().info(f"Camera opened successfully at {camera_url}")

    def publish_images(self):
        ret, frame = self.capture.read()
        if not ret or frame is None or frame.size == 0:
            self.get_logger().error("Failed to capture image from camera")
            return

        try:
            img = cv2.resize(frame, (160, 120))  # Ajustar o tamanho conforme necessário
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.image_publisher.publish(image_msg)
            self.get_logger().info("Published image to /camera/image_raw")
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {e}")

def main():
    camera_url = '/dev/video0'  # Substitua pelo endereço da sua câmera
    rclpy.init()
    node = CameraImagePublisher(camera_url)
    try:
        while rclpy.ok():
            node.publish_images()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
