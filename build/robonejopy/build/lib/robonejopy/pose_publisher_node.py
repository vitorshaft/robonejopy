import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import math

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        
        # Inscrição no tópico da IMU (seu pacote de sensores)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
            
        # Publisher para o Foxglove
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)
        
        self.get_logger().info("Nó de Pose Atualizada iniciado!")

    def imu_callback(self, msg):
        # Criar a mensagem de PoseStamped
        pose = PoseStamped()
        
        # Sincroniza o timestamp com o da IMU para evitar atrasos no Foxglove
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'map' # Frame de referência global

        # Posição (Estática em 0,0,0 já que IMU sozinha não dá translação precisa)
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0

        # Orientação (Copia o Quatérnio já calculado pela sua fusão sensorial)
        pose.pose.orientation.x = msg.orientation.x
        pose.pose.orientation.y = msg.orientation.y
        pose.pose.orientation.z = msg.orientation.z
        pose.pose.orientation.w = msg.orientation.w

        self.pose_publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()