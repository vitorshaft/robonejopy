import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import math

class ObjectDetectionNode(Node):
    def __init__(self, camera_url):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/aves', 10)
        self.capture = cv2.VideoCapture(camera_url)

        # model
        self.model = YOLO('/home/vitor/robonejo/best.pt')

        # object classes
        self.class_names = ["chicken"]

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.image_callback)

    def image_callback(self):
        ret, img = self.capture.read()
        if ret:
            # Redimensionar a imagem para reduzir a latência
            img = cv2.resize(img, (320, 240))  # Redimensionar para 640x480 (pode ajustar conforme necessário)

            results = self.model(img, stream=True)

            for r in results:
                boxes = r.boxes

                for box in boxes:
                    # bounding box
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                    # confidence
                    confidence = math.ceil((box.conf[0]*100))/100
                    if(confidence > 0.4):
                        print("Confidence --->",confidence)

                        # class name
                        cls = int(box.cls[0])
                        print("Class name -->", self.class_names[cls])

                        # object details
                        org = [x1, y1]
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        fontScale = 1
                        color = (255, 0, 0)
                        thickness = 2

                        cv2.putText(img, self.class_names[cls], org, font, fontScale, color, thickness)

            # Publish the image with detected objects
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.publisher.publish(image_msg)

def main():
    camera_url = '/dev/video0' #http://192.168.76.224:81/stream'  # Substitua pelo endereço IP da sua câmera
    rclpy.init()
    node = ObjectDetectionNode(camera_url)
    #rclpy.spin(node)
    while rclpy.ok():
        node.image_callback()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
