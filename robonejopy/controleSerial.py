import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelToSerialNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1)
        except Exception as e:
            self.get_logger().error(f'Falha ao abrir serial: {e}')
        self.get_logger().info('CmdVelToSerialNode inicializado')

    def cmd_vel_callback(self, msg):
        ang = msg.angular.z
        lin = msg.linear.x
        #self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
        if(ang > 1.0 and lin == 0.0):
            self.serial_port.write(str(0).encode())
        elif(ang > 0.0 and lin > 0.0):
            self.serial_port.write(str(1).encode())
        elif(ang == 0 and lin > 0.0):
            self.serial_port.write(str(2).encode())
        elif(ang < 0.0 and lin > 0.0):
            self.serial_port.write(str(3).encode())
        elif(ang < -1.0 and lin == 0.0):
            self.serial_port.write(str(4).encode())
        elif(ang == 0.0 and lin < 0.0):
            self.serial_port.write(str(6).encode())
            self.serial_port.write(str(2).encode())
            
        else:
            self.serial_port.write(str(6).encode())
            self.serial_port.write(str(5).encode())
        #self.get_logger().info('Resultado: %f', result)
        #self.serial_port.write(f'{result}\n'.encode())
        print("/cmd_vel: ",msg.angular.z, msg.linear.x)
        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_to_serial_node = CmdVelToSerialNode()
    rclpy.spin(cmd_vel_to_serial_node)
    cmd_vel_to_serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
