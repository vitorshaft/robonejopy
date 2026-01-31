import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial

class UltrasonicBridge(Node):
    def __init__(self):
        super().__init__('ultrasonic_bridge')
        
        # Publisher para o tópico de distância
        self.publisher_ = self.create_publisher(Range, '/ultrasonic/range', 10)
        
        # Configuração da Serial
        self.port = '/dev/ttyAMA0'
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            self.get_logger().info(f"Conectado ao ESP32 em {self.port}")
        except Exception as e:
            self.get_logger().error(f"Erro ao abrir serial: {e}")

        # Timer para ler a serial (20Hz para bater com o ESP32)
        self.create_timer(0.05, self.read_serial_callback)

    def read_serial_callback(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Procura pelo valor Filtered na string: "Raw:XX.X,Filtered:YY.YY cm"
                if "Filtered:" in line:
                    parts = line.split("Filtered:")
                    val_str = parts[1].replace(" cm", "")
                    distance_cm = float(val_str)
                    
                    # Criar mensagem Range
                    msg = Range()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'ultrasound_link' # Nome do frame para o TF
                    msg.radiation_type = Range.ULTRASOUND
                    msg.field_of_view = 0.523599 # ~30 graus em radianos
                    msg.min_range = 0.02   # 2 cm em metros
                    msg.max_range = 4.0    # 4 metros
                    msg.range = distance_cm / 100.0 # Converte CM para Metros (Padrão ROS)
                    
                    self.publisher_.publish(msg)
            except Exception as e:
                pass # Ignora linhas mal formadas

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicBridge()
    rclpy.spin(node)
    rclpy.shutdown()
