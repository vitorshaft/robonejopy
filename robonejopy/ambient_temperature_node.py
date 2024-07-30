import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import Adafruit_DHT

class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('ambient_temperature_node')
        self.publisher = self.create_publisher(Temperature, '/ambient_temperature', 10)
        self.sensor = Adafruit_DHT.DHT11
        self.pin = 4  # GPIO pin where the DHT11 is connected

        # Timer to read sensor data every 5 seconds
        self.timer = self.create_timer(3.0, self.read_sensor)

    def read_sensor(self):
        humidity, temperature = Adafruit_DHT.read(self.sensor, self.pin)
        if temperature is not None:
            msg = Temperature()
            msg.temperature = temperature
            msg.variance = 0.0
            self.publisher.publish(msg)
        else:
            self.get_logger().warning('Failed to read from DHT11 sensor')

def main():
    rclpy.init()
    node = TemperatureSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
