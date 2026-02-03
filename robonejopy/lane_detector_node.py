import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from .lane_detection import LaneDetection
import cv2
import math
import numpy as np

def euler_from_quaternion(x, y, z, w):
    """
    Converte quatérnio para Euler (apenas Yaw nos interessa aqui)
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class LaneDetectFollower(Node):
    def __init__(self):
        super().__init__('lane_detect_follower_node')
        
        self.lane_detector = LaneDetection()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.processed_image_pub = self.create_publisher(Image, '/camera/processed_image', 1)

        # Subscribers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 1)
        
        # NOVO: Escuta a orientação precisa vinda do Dead Reckoning
        self.pose_sub = self.create_subscription(PoseStamped, '/robot_pose_estimated', self.pose_callback, 1)

        # Parâmetros PID de Linha
        self.declare_parameter('Kp', 0.01)
        self.declare_parameter('Kd', 0.05)
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value

        # Parâmetro Proporcional para Manter o Rumo (Heading Hold)
        self.Kp_heading = 2.0 

        self.max_desire_linear_vel = 0.3
        self.last_cte = 0.0
        
        # Estado de Fail-safe
        self.lost_lane_counter = 0
        self.MAX_LOST_FRAMES = 15
        
        self.current_yaw = 0.0
        self.target_yaw_lock = 0.0 # Rumo a manter quando perder a pista
        
        self.bridge = CvBridge()
        self.counter = 0

    def pose_callback(self, msg):
        # Atualiza o Yaw atual do robô baseado na fusão (Mag + Giro)
        q = msg.pose.orientation
        self.current_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def camera_callback(self, data):
        self.counter += 1
        if self.counter % 3 != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv2.resize(cv_image, (320, 240))
        except CvBridgeError as e:
            self.get_logger().error(f"Erro CV2: {e}")
            return

        cte, angle, final_img = self.lane_detector.processImage(cv_image)
        cmd_vel = Twist()

        if cte is not None:
            # --- MODO VISÃO ATIVA ---
            self.lost_lane_counter = 0
            
            # Enquanto vê a linha, atualizamos o "target_lock" para o rumo atual.
            # Se a linha sumir agora, é ESTE ângulo que vamos manter.
            self.target_yaw_lock = self.current_yaw
            
            angular_z = self.Kp * cte + self.Kd * (cte - self.last_cte)
            self.last_cte = cte
            linear_x = self.max_desire_linear_vel
            angular_z = max(min(angular_z, 1.5), -1.5)
            
        else:
            # --- MODO FAIL-SAFE (HEADING HOLD) ---
            self.lost_lane_counter += 1
            
            if self.lost_lane_counter < self.MAX_LOST_FRAMES:
                self.get_logger().warn(f"Pista perdida! Travando no rumo: {self.target_yaw_lock:.2f}")
                
                # Calcula erro de ângulo (Diferença entre onde quero ir e onde estou)
                yaw_error = self.target_yaw_lock - self.current_yaw
                
                # Normaliza erro para -PI a PI
                yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
                
                # Controlador P simples para corrigir a rotação
                heading_correction = self.Kp_heading * yaw_error
                
                linear_x = self.max_desire_linear_vel * 0.6
                angular_z = heading_correction
                
                # Limita a correção para não dar trancos
                angular_z = max(min(angular_z, 1.0), -1.0)
            else:
                self.get_logger().error("EMERGÊNCIA: Parando.")
                linear_x = 0.0
                angular_z = 0.0

        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel)

        if final_img is not None:
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(final_img, encoding="bgr8")
                self.processed_image_pub.publish(processed_msg)
            except CvBridgeError:
                pass

    def clean_up(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.clean_up()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()