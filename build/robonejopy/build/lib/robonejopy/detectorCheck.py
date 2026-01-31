import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            Image,
            '/aves',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        self.get_logger().info('Received image message')

def main():
    rclpy.init()
    node = SubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
