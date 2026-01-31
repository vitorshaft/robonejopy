import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class Calibrator(Node):
    def __init__(self):
        super().__init__('calibrador_imu')
        self.subscription = self.create_subscription(
            Imu, '/imu/data', self.listener_callback, 10)
        self.samples_x = []
        self.samples_y = []
        self.samples_z = []
        self.max_samples = 2000
        print("Iniciando coleta de 2000 amostras... NÃO MOVA O ROBÔ!")

    def listener_callback(self, msg):
        if len(self.samples_x) < self.max_samples:
            self.samples_x.append(msg.angular_velocity.x)
            self.samples_y.append(msg.angular_velocity.y)
            self.samples_z.append(msg.angular_velocity.z)
            
            if len(self.samples_x) % 100 == 0:
                print(f"Amostras: {len(self.samples_x)}/{self.max_samples}")
        else:
            self.calculate_stats()
            rclpy.shutdown()

    def calculate_stats(self):
        print("\n--- RESULTADOS DA CALIBRAÇÃO (Offsets) ---")
        print(f"Giroscópio X Bias: {np.mean(self.samples_x):.6f} (Ruído: {np.std(self.samples_x):.6f})")
        print(f"Giroscópio Y Bias: {np.mean(self.samples_y):.6f} (Ruído: {np.std(self.samples_y):.6f})")
        print(f"Giroscópio Z Bias: {np.mean(self.samples_z):.6f} (Ruído: {np.std(self.samples_z):.6f})")
        print("------------------------------------------")
        print("Copie os valores de 'Bias' e subtraia eles no seu imu_node.py!")

def main():
    rclpy.init()
    node = Calibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()