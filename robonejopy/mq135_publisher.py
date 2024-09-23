import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import spidev
import time

class MQ135Publisher(Node):

    def __init__(self):
        super().__init__('mq135_publisher')
        self.publisher_ = self.create_publisher(Float32, 'mq135_ammonia_ppm', 10)
        timer_period = 2.0  # Em segundos
        self.timer = self.create_timer(timer_period, self.publish_mq135_data)
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1350000
        self.mq135_channel = 0

    def read_adc(self, channel):
        adc = self.spi.xfer2([1, (8 + channel) << 4, 0])
        data = ((adc[1] & 3) << 8) + adc[2]
        return data

    def convert_to_ppm(self, adc_value):
        # Aqui você pode inserir uma fórmula calibrada para converter o valor lido
        # pelo ADC (0 a 1023) para partes por milhão (ppm) de amônia.
        # Por simplicidade, vamos apenas retornar uma aproximação linear.
        # É necessário calibrar o sensor para obter valores reais em ppm.
        # Este é apenas um exemplo básico:
        voltage = adc_value * (5.0 / 1023.0)
        ppm = voltage * 10  # Substitua por sua fórmula calibrada
        return ppm

    def publish_mq135_data(self):
        adc_value = self.read_adc(self.mq135_channel)
        ammonia_ppm = self.convert_to_ppm(adc_value)
        msg = Float32()
        msg.data = ammonia_ppm
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado: {ammonia_ppm} ppm de amônia')

def main(args=None):
    rclpy.init(args=args)
    mq135_publisher = MQ135Publisher()
    rclpy.spin(mq135_publisher)
    mq135_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
