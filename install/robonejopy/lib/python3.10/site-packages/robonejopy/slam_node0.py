import rclpy
from rclpy.node import Node
import sys
sys.path.append("/home/vitor/ros2_ws/install/lib/python3.10/site-packages/")  # Adjust path if needed
from nav2_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped

import cv2
import orb_slam2

class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)

        # Inicializar o ORB_SLAM2
        self.orb = orb_slam2.System("Vocabulary/ORBvoc.txt", "System.yaml", orb_slam2.ORB_SLAM2.System.MONOCULAR, 0.0)

        # Variáveis para armazenar a pose estimada pelo ORB_SLAM2
        self.pose = None

    def listener_callback(self, msg):
        # Converter a imagem ROS para OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Passar a imagem para o ORB_SLAM2
        self.orb.TrackMonocular(cv_image, msg.header.stamp.to_sec())

        # Obter a pose estimada
        if self.orb.GetPose():
            self.pose = self.orb.GetPose()

            # Criar a mensagem de pose para o AMCL
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = "map"
            # ... (Preencher os campos da pose com base na pose estimada pelo ORB_SLAM2)

            self.publisher_.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    slam_node = SLAMNode()

    rclpy.spin(slam_node)

    # Destroy the node explicitly
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()