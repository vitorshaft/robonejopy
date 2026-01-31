import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # Importando o tipo Point
from cv_bridge import CvBridge
import cv2

class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Point, '/conducao', 10)
        # Caminho completo para o classificador em cascata
        cascade_path = 'haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        try:
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            for (x, y, w, h) in faces:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                center_x = x + w // 2
                center_y = y + h // 2
                self.publish_coordinates(center_x, center_y)
            
        except:
            cv2.imshow('Face Detection', cv_image)
            pass
        cv2.waitKey(1)

    def publish_coordinates(self, center_x, center_y):
        point = Point()
        point.x = center_x
        point.y = center_y
        self.publisher.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
