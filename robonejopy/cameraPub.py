import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import urllib.request

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # URL atualizada com a Porta 81 e o IP correto
        self.url = 'http://192.168.0.101:81/'
        
        # Buffer para acumular os bytes do stream
        self.bytes_buffer = b''
        
        # Timer para processar o stream (15 FPS aproximadamente)
        self.create_timer(0.06, self.timer_callback)
        self.get_logger().info(f"Conectando ao Stream da ESP32-CAM em {self.url}")
        
        try:
            self.stream = urllib.request.urlopen(self.url, timeout=5)
        except Exception as e:
            self.get_logger().error(f"Não foi possível abrir o stream: {e}")

    def timer_callback(self):
        try:
            # Lê um pedaço do stream e adiciona ao buffer
            self.bytes_buffer += self.stream.read(4096)
            
            # Procura os marcadores de início (0xff 0xd8) e fim (0xff 0xd9) do JPEG
            a = self.bytes_buffer.find(b'\xff\xd8')
            b = self.bytes_buffer.find(b'\xff\xd9')
            
            if a != -1 and b != -1:
                # Extrai apenas os bytes da imagem
                jpg_data = self.bytes_buffer[a:b+2]
                # Limpa o buffer para o próximo frame
                self.bytes_buffer = self.bytes_buffer[b+2:]
                
                # Decodifica os bytes para uma imagem OpenCV
                frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                
                if frame is not None:
                    # Redimensiona para manter leve no Foxglove (320x240)
                    frame_resized = cv2.resize(frame, (320, 240))
                    
                    # Converte para mensagem ROS e publica
                    img_msg = self.bridge.cv2_to_imgmsg(frame_resized, encoding='bgr8')
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = 'camera_link'
                    
                    self.publisher_.publish(img_msg)
                    
        except Exception as e:
            # Se a conexão cair, tenta reabrir
            self.get_logger().warn("Conexão perdida com a ESP32-CAM. Reconectando...")
            try:
                self.stream = urllib.request.urlopen(self.url, timeout=2)
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()