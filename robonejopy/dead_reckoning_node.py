import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu
import math

class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__('dead_reckoning_node')
        
        # Subscrições
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose_estimated', 10)

        # Estado do Robô
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.gyro_z = 0.0
        self.linear_v = 0.0
        
        # --- Lógica de Calibração ---
        self.is_calibrated = False
        self.calibration_samples = []
        self.gyro_bias = 0.0
        self.calibration_duration = 2.0 # Segundos
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('INICIANDO CALIBRAÇÃO... MANTENHA O ROBÔ PARADO.')

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update_odometry)

    def cmd_vel_callback(self, msg):
        self.linear_v = msg.linear.x

    def imu_callback(self, msg):
        raw_gyro_z = msg.angular_velocity.z
        
        # Fase 1: Coleta de amostras para calibração
        if not self.is_calibrated:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed < self.calibration_duration:
                self.calibration_samples.append(raw_gyro_z)
            else:
                # Calcula a média do ruído (Bias)
                if self.calibration_samples:
                    self.gyro_bias = sum(self.calibration_samples) / len(self.calibration_samples)
                self.is_calibrated = True
                self.get_logger().info(f'CALIBRAÇÃO CONCLUÍDA! Bias: {self.gyro_bias:.5f} rad/s')
            return

        # Fase 2: Aplicação do Bias e Filtro de Deadzone
        corrected_gyro = raw_gyro_z - self.gyro_bias
        
        # Filtro de Deadzone (Ignora micro-ruídos restantes)
        if abs(corrected_gyro) < 0.005: 
            self.gyro_z = 0.0
        else:
            self.gyro_z = corrected_gyro

    def update_odometry(self):
        if not self.is_calibrated:
            return # Aguarda calibrar antes de começar a integrar

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Integração da Pose
        self.th += self.gyro_z * dt
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        self.x += self.linear_v * math.cos(self.th) * dt
        self.y += self.linear_v * math.sin(self.th) * dt

        # Publicação PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.orientation.z = math.sin(self.th / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.th / 2.0)
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()