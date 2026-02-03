import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu, MagneticField, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import math

class DeadReckoningFusionNode(Node):
    def __init__(self):
        super().__init__('dead_reckoning_node')
        
        # --- CONFIGURAÇÕES DE CALIBRAÇÃO (AJUSTE AQUI) ---
        # Optical Flow: Quantos metros o robô andou por pixel deslocado?
        # Calibre: Mande andar 1m, veja quantos pixels acumulou e divida 1 / pixels.
        self.pixel_to_meter_ratio = 0.0025 
        
        # Fusão de Velocidade (0.0 a 1.0)
        # 0.7 = 70% Confiança no Optical Flow, 30% no cmd_vel
        self.alpha_velocity = 0.6 
        
        # Fusão de Ângulo (0.0 a 1.0)
        # 0.98 = 98% Giroscópio (rápido), 2% Magnetômetro (correção de drift)
        self.alpha_yaw = 0.98 
        
        # ROI (Região de Interesse) para o Optical Flow [y1:y2, x1:x2]
        # Focamos no centro inferior da imagem (chão logo à frente)
        self.roi_slice = (slice(140, 220), slice(110, 210)) 

        # --- SUBSCRIBERS ---
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/magnetic_field', self.mag_callback, 10)
        self.cam_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1) # 1 frame buffer para baixa latência
        
        # --- PUBLISHERS ---
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose_estimated', 10)
        
        # Variáveis de Estado (Posição Global)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0 # Yaw Fused
        
        # Variáveis de Sensores
        self.gyro_z = 0.0
        self.accel_x, self.accel_y, self.accel_z = 0.0, 0.0, 9.8
        self.mag_x, self.mag_y, self.mag_z = 0.0, 0.0, 0.0
        
        self.linear_v_cmd = 0.0    # Vinda do comando
        self.linear_v_visual = 0.0 # Vinda da câmera
        
        # Controle de Optical Flow
        self.bridge = CvBridge()
        self.prev_gray = None
        
        # Controle de Tempo e Inicialização
        self.has_mag_data = False
        self.initial_mag_yaw = None
        self.last_time = self.get_clock().now()
        
        # Timer principal de Odometria (20Hz)
        self.timer = self.create_timer(0.05, self.update_odometry)
        
        self.get_logger().info("Dead Reckoning Fusion Iniciado: IMU + MAG + Optical Flow")

    def cmd_vel_callback(self, msg):
        self.linear_v_cmd = msg.linear.x

    def imu_callback(self, msg):
        self.gyro_z = msg.angular_velocity.z
        self.accel_x = msg.linear_acceleration.x
        self.accel_y = msg.linear_acceleration.y
        self.accel_z = msg.linear_acceleration.z

    def mag_callback(self, msg):
        self.mag_x = msg.magnetic_field.x
        self.mag_y = msg.magnetic_field.y
        self.mag_z = msg.magnetic_field.z
        self.has_mag_data = True

    def image_callback(self, msg):
        """Calcula a velocidade linear visual usando Optical Flow em um ROI"""
        try:
            # Converte para OpenCV e escala de cinza
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Recorta apenas a região do chão (ROI)
            # Isso acelera absurdamente o processamento
            curr_roi = gray[self.roi_slice]
            
            if self.prev_gray is not None:
                # Calcula Optical Flow Denso (Farneback) no ROI
                # Parâmetros ajustados para chão de galpão (textura de maravalha)
                flow = cv2.calcOpticalFlowFarneback(
                    self.prev_gray, curr_roi, None, 
                    pyr_scale=0.5, levels=3, winsize=15, 
                    iterations=3, poly_n=5, poly_sigma=1.2, flags=0
                )
                
                # A componente Y do fluxo (vertical na imagem) é o movimento para frente/trás
                # Negativo porque Y cresce para baixo na imagem, mas o robô indo pra frente vê o chão "descer"
                flow_y = flow[..., 1]
                avg_pixel_displacement = np.mean(flow_y)
                
                # Conversão: Pixels/frame -> Metros/s
                # Multiplicamos por 30 (FPS estimado) ou usamos o dt real se disponível
                # Aqui usamos uma constante calibrável simples
                self.linear_v_visual = -(avg_pixel_displacement * self.pixel_to_meter_ratio) * 20.0 # aprox 20hz do loop visual

            self.prev_gray = curr_roi

        except CvBridgeError:
            pass
        except Exception as e:
            self.get_logger().warn(f"Erro no Optical Flow: {e}")

    def calculate_tilt_compensated_yaw(self):
        """Calcula o Norte Magnético corrigindo a inclinação do robô"""
        # 1. Roll e Pitch baseados na gravidade (Acelerômetro)
        roll = math.atan2(self.accel_y, self.accel_z)
        pitch = math.atan2(-self.accel_x, math.sqrt(self.accel_y**2 + self.accel_z**2))

        # 2. Rotação dos vetores magnéticos para compensar o tilt
        # Fórmulas padrão de navegação
        Xh = self.mag_x * math.cos(pitch) + self.mag_z * math.sin(pitch)
        Yh = self.mag_x * math.sin(roll) * math.sin(pitch) + \
             self.mag_y * math.cos(roll) - \
             self.mag_z * math.sin(roll) * math.cos(pitch)

        # 3. Cálculo do ângulo
        return math.atan2(Yh, Xh)

    def update_odometry(self):
        if not self.has_mag_data:
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # --- FUSÃO DE ÂNGULO (YAW) ---
        mag_yaw_raw = self.calculate_tilt_compensated_yaw()

        # Define o "Zero" na primeira leitura
        if self.initial_mag_yaw is None:
            self.initial_mag_yaw = mag_yaw_raw
            self.th = 0.0

        # Normaliza o magnético relativo ao início
        mag_yaw_corrected = mag_yaw_raw - self.initial_mag_yaw
        mag_yaw_corrected = math.atan2(math.sin(mag_yaw_corrected), math.cos(mag_yaw_corrected))

        # Filtro Complementar (Giroscópio + Magnetômetro)
        gyro_pred = self.th + (self.gyro_z * dt)
        
        # Diferença considerando a descontinuidade -PI/PI
        diff = mag_yaw_corrected - gyro_pred
        diff = math.atan2(math.sin(diff), math.cos(diff))
        
        self.th = gyro_pred + (1.0 - self.alpha_yaw) * diff
        
        # Normaliza final
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # --- FUSÃO DE VELOCIDADE (LINEAR) ---
        # Lógica de Freio de Mão: Se o comando é ZERO, ignoramos o Optical Flow (ruído visual)
        if abs(self.linear_v_cmd) < 0.01:
            final_v = 0.0
            self.linear_v_visual = 0.0 # Reseta visual para não acumular erro
        else:
            # Média ponderada entre o comando teórico e a visão real
            final_v = (self.alpha_velocity * self.linear_v_visual) + \
                      ((1.0 - self.alpha_velocity) * self.linear_v_cmd)

        # --- INTEGRAÇÃO DA POSIÇÃO ---
        self.x += final_v * math.cos(self.th) * dt
        self.y += final_v * math.sin(self.th) * dt

        # --- PUBLICAÇÃO DA POSE 3D ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = "map"

        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0

        # Para visualização, usamos Roll/Pitch do acelerômetro e Yaw da fusão
        roll = math.atan2(self.accel_y, self.accel_z)
        pitch = math.atan2(-self.accel_x, math.sqrt(self.accel_y**2 + self.accel_z**2))
        
        qx, qy, qz, qw = self.get_quaternion_from_euler(roll, pitch, self.th)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()