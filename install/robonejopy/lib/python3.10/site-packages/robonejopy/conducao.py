import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class ConducaoNode(Node):
    def __init__(self):
        super().__init__('conducao_node')
        self.subscription = self.create_subscription(
            Int32,
            'conducao',
            self.conducao_callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
        #self.serial_port = serial.Serial('/dev/serial1', 9600)  # Porta serial onde o Arduino está conectado

    def conducao_callback(self, msg):
        self.get_logger().info('Recebi: %d' % msg.data)
        self.serial_port.write(msg.data.to_bytes(1, 'little'))  # Envia o comando para o Arduino via Serial

def main(args=None):
    rclpy.init(args=args)
    conducao_node = ConducaoNode()
    rclpy.spin(conducao_node)
    conducao_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
