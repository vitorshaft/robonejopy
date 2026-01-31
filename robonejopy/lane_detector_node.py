import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from .lane_detection import LaneDetection
import cv2
import numpy as np

class LaneDetectFollower(Node):
    def __init__(self):
        super().__init__('lane_detect_follower_node')
        
        # Instancia o detector uma única vez para performance
        self.lane_detector = LaneDetection()

        # Publishers e Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.camera_callback, 
            1
        )
        self.processed_image_pub = self.create_publisher(Image, '/camera/processed_image', 1)

        # Parâmetros PID
        self.declare_parameter('Kp', 0.01)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('Kd', 0.05)
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value

        # Constantes do Robô
        self.max_desire_linear_vel = 0.3  # m/s
        self.last_cte = 0.0
        
        # --- Lógica de Blindagem (Fail-safe) ---
        self.lost_lane_counter = 0
        self.MAX_LOST_FRAMES = 15  # Quantos frames esperar antes de parar totalmente
        
        # Estado e utilitários
        self.running = True
        self.bridge = CvBridge()
        self.counter = 0

    def camera_callback(self, data):
        if not self.running:
            return

        # Filtro de Frequência para aliviar a CPU do Raspberry Pi
        self.counter += 1
        if self.counter % 3 != 0:
            return

        try:
            # Conversão e redimensionamento para 320x240 (Essencial para não dar SegFault)
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.resize(cv_image, (320, 240))
            
            if cv_image is None or cv_image.size == 0:
                return
                
        except CvBridgeError as e:
            self.get_logger().error(f"Erro na conversão CV2: {e}")
            return

        # Processamento da Pista
        cte, angle, final_img = self.lane_detector.processImage(cv_image)

        cmd_vel = Twist()

        if cte is not None:
            # --- MODO VISÃO ATIVA ---
            self.lost_lane_counter = 0 # Reseta o contador de falhas
            
            # Cálculo PID
            angular_z = self.Kp * cte + self.Kd * (cte - self.last_cte)
            self.last_cte = cte
            linear_x = self.max_desire_linear_vel
            
            # Saturação (limites de giro)
            angular_z = max(min(angular_z, 1.5), -1.5)
            
        else:
            # --- MODO FAIL-SAFE (IMU/INERCIAL) ---
            self.lost_lane_counter += 1
            
            if self.lost_lane_counter < self.MAX_LOST_FRAMES:
                self.get_logger().warn(f"Pista perdida ({self.lost_lane_counter})! Mantendo curso...")
                
                # O robô tenta seguir reto usando o último erro conhecido amortecido
                # Aqui o Dead Reckoning/IMU ajuda a manter o curso estável
                linear_x = self.max_desire_linear_vel * 0.6 # Reduz velocidade por segurança
                angular_z = self.Kp * self.last_cte * 0.5   # Tenta suavizar a volta
                angular_z = max(min(angular_z, 0.8), -0.8)
            else:
                self.get_logger().error("Pista perdida por muito tempo! EMERGÊNCIA: PARANDO.")
                linear_x = 0.0
                angular_z = 0.0

        # Publica comandos para os motores
        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel)

        # Publica imagem para o Foxglove
        if final_img is not None:
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(final_img, encoding="bgr8")
                self.processed_image_pub.publish(processed_msg)
            except CvBridgeError as e:
                self.get_logger().warn(f"Erro ao publicar imagem: {e}")

    def clean_up(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    lane_detect_follower_object = LaneDetectFollower()
    try:
        rclpy.spin(lane_detect_follower_object)
    except KeyboardInterrupt:
        lane_detect_follower_object.get_logger().info("Encerrando pelo teclado.")
    finally:
        lane_detect_follower_object.clean_up()
        lane_detect_follower_object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()