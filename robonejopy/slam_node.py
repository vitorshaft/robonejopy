import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
import serial
import time
import tf_transformations as tf

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        # Assinando tópicos de câmera e IMU
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # Publicando a pose estimada e comandos de movimento
        self.publisher_pose = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Iniciando parâmetros
        self.bridge = CvBridge()
        self.orb_slam = cv2.ORB_create()
        self.imudata = None
        
        # Serial para comunicação com o Arduino
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
        
        # Variáveis de controle de movimento
        self.current_pose = PoseWithCovarianceStamped()
        self.current_orientation = None

    def image_callback(self, msg):
        # Converter imagem ROS para OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        keypoints, descriptors = self.orb_slam.detectAndCompute(frame, None)

        # Processar e estimar a pose (placeholder para seu SLAM)
        if keypoints is not None:
            # Atualiza a pose (aproximação)
            self.current_pose.pose.pose.position.x += 0.05  # Exemplo de atualização
            
            # Publicar a pose
            self.publisher_pose.publish(self.current_pose)
    
    def imu_callback(self, msg):
        # Atualizar orientação com base nos dados da IMU
        orientation_q = msg.orientation
        roll, pitch, yaw = tf.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        
        # Atualizar a pose estimada com a orientação
        self.current_pose.pose.pose.orientation = orientation_q
        
    def send_cmd_vel(self, linear, angular):
        # Enviar comandos para o Arduino via Serial
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_cmd_vel.publish(twist)
        
        if angular > 0.5:
            self.serial_port.write(str(1).encode())  # Exemplo
        elif linear > 0:
            self.serial_port.write(str(2).encode())
        else:
            self.serial_port.write(str(6).encode())
        
        time.sleep(0.1)  # Delay para enviar novos comandos

def main(args=None):
    rclpy.init(args=args)
    slam_node = SLAMNode()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
