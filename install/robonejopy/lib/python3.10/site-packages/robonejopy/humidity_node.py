import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure  # Ou sua mensagem personalizada
import Adafruit_DHT

class HumiditySensorNode(Node):
    def __init__(self):
        super().__init__('ambient_humidity_node')
        self.publisher = self.create_publisher(FluidPressure, '/ambient_humidity', 10)
        self.sensor = Adafruit_DHT.DHT11
        self.pin = 17  # GPIO pin where the DHT11 is connected

        # Timer to read sensor data every 3 seconds
        self.timer = self.create_timer(3.0, self.read_sensor)

    def read_sensor(self):
        humidity, temperature = Adafruit_DHT.read(self.sensor, self.pin)
        if humidity is not None and temperature is not None:
            msg = FluidPressure()  # Use sua mensagem personalizada aqui
            msg.fluid_pressure = humidity  # Ajuste conforme sua mensagem
            # msg.temperature = temperature  # Adicione se necessário
            msg.header.stamp = self.get_clock().now().to_msg()  # Adicione timestamp
            msg.header.frame_id = "humidity_sensor"  # Defina o frame_id, se necessário
            self.publisher.publish(msg)
        else:
            self.get_logger().warning('Failed to read from DHT11 sensor')

def main():
    rclpy.init()
    node = HumiditySensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
