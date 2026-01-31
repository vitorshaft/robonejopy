import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class AmmoniaSensorNode(Node):
    def __init__(self):
        super().__init__('ammonia_sensor_node')

        # Iniciando a comunicação Serial com o Arduino (ajuste a porta conforme necessário)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.publisher = self.create_publisher(Float32, '/amonia_ppm', 10)

        # Timer para ler dados da Serial e publicar no tópico
        self.timer = self.create_timer(0.5, self.read_and_publish_data)

    def read_and_publish_data(self):
        if self.serial_port.in_waiting > 0:
            try:
                # Lendo dados da Serial e limpando
                line = self.serial_port.readline().decode('utf-8').strip()
                
                # Exemplo de linha esperada: "amonia: 12.34ppm"
                if "amonia:" in line:
                    ppm_str = line.split(":")[1].replace("ppm", "").strip()
                    ppm = float(ppm_str)
                    
                    # Publicando o valor de amônia em ppm no tópico /amonia_ppm
                    msg = Float32()
                    msg.data = ppm
                    self.publisher.publish(msg)

                    self.get_logger().info(f"Amônia (PPM): {ppm}")

            except Exception as e:
                self.get_logger().error(f"Erro na leitura da Serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AmmoniaSensorNode()
    rclpy.spin(node)

    # Encerrando o nó e fechando a porta Serial
    node.serial_port.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
