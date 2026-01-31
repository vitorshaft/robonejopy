import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import board
import busio
from adafruit_mlx90614 import MLX90614

class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('temperature_sensor_node')
        self.publisher = self.create_publisher(Temperature, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Initialize I2C and MLX90614 sensor
        print("SCL: ",board.SCL," SDA: ",board.SDA)
        i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
        self.mlx = MLX90614(i2c)

    def timer_callback(self):
        # Read temperatures from the sensor
        ambient_temp = self.mlx.ambient_temperature
        object_temp = self.mlx.object_temperature

        # Create and publish Temperature message
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'temperature_frame'
        msg.temperature = object_temp
        msg.variance = 0.0

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.temperature} °C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
