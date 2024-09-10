import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import time

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Inicializa o I2C para a IMU (MPU6050)
        self.bus = smbus.SMBus(1)  # Para Raspberry Pi, geralmente é bus 1
        self.device_address = 0x68
        self.bus.write_byte_data(self.device_address, 0x6B, 0)  # Desativa o modo sleep da IMU

        # Publica os dados da IMU no tópico '/imu/data'
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)

    def read_imu_data(self):
        accel_x = self.read_raw_data(0x3B)
        accel_y = self.read_raw_data(0x3D)
        accel_z = self.read_raw_data(0x3F)

        gyro_x = self.read_raw_data(0x43)
        gyro_y = self.read_raw_data(0x45)
        gyro_z = self.read_raw_data(0x47)

        return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr+1)
        value = ((high << 8) | low)
        if(value > 32768):
            value = value - 65536
        return value

    def publish_imu_data(self):
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.read_imu_data()

        # Cria a mensagem IMU
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Preenche os valores da aceleração e giroscópio
        imu_msg.linear_acceleration.x = accel_x / 16384.0  # Aceleração em G
        imu_msg.linear_acceleration.y = accel_y / 16384.0
        imu_msg.linear_acceleration.z = accel_z / 16384.0
        imu_msg.angular_velocity.x = gyro_x / 131.0  # Velocidade angular em deg/s
        imu_msg.angular_velocity.y = gyro_y / 131.0
        imu_msg.angular_velocity.z = gyro_z / 131.0

        # Publica a mensagem
        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
