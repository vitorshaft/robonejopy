import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Temperature
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import math
import time
import sqlite3
import signal
import sys

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/aves', 10)

        # Subscribing to the camera topic
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Subscribing to the temperature topic
        self.create_subscription(Temperature, '/temperature', self.temperature_callback, 10)

        # Subscribing to the ambient_temperature topic (environment)
        self.create_subscription(Temperature, '/ambient_temperature', self.ambient_temperature_callback, 10)

        # Model
        self.model = YOLO('/home/vitor/robonejo/best.pt')

        # Object classes
        self.class_names = ["chicken"]

        # SQLite setup
        self.conn = sqlite3.connect('/home/vitor/robonejo/robonejo.db')
        self.cursor = self.conn.cursor()
        self.create_tables()

        # Recover state
        self.route_count, self.bird_count = self.recover_state()
        self.route_start_time = time.time()

        # Variables to store the ambient temperature and detected birds
        self.temperature = None
        self.ambient_temperature = None
        self.detected_birds = {}

    def create_tables(self):
        """Create the necessary tables if they don't exist."""
        self.cursor.execute('''CREATE TABLE IF NOT EXISTS route_summary (
                                id INTEGER PRIMARY KEY AUTOINCREMENT,
                                route_count INTEGER,
                                bird_count INTEGER,
                                route_duration REAL,
                                timestamp TEXT
                              )''')
        self.cursor.execute('''CREATE TABLE IF NOT EXISTS bird_count (
                                id INTEGER PRIMARY KEY AUTOINCREMENT,
                                class TEXT,
                                confidence REAL,
                                timestamp TEXT
                              )''')
        self.conn.commit()

    def recover_state(self):
        """Recover the latest route state from the SQLite database."""
        self.cursor.execute('SELECT route_count, bird_count FROM route_summary ORDER BY id DESC LIMIT 1')
        row = self.cursor.fetchone()
        route_count = row[0] if row else 0
        bird_count = row[1] if row else 0
        return route_count, bird_count

    def ambient_temperature_callback(self, msg):
        self.ambient_temperature = msg.temperature

    def temperature_callback(self, msg):
        self.temperature = msg.temperature

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if img is not None:
            # Resize the image to reduce latency
            img = cv2.resize(img, (320, 240))  # Resize to 640x480 (adjust as needed)

            results = self.model(img, stream=True)

            for r in results:
                boxes = r.boxes

                for box in boxes:
                    # Bounding box
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # Convert to int values

                    # Confidence
                    confidence = math.ceil((box.conf[0] * 100)) / 100
                    if confidence > 0.4:
                        # Class name
                        cls = int(box.cls[0])
                        if (x1 < 160 and x2 > 160):
                            if(y1 < 120 and y2 > 120):
                                if self.class_names[cls] == "chicken" and self.ambient_temperature is not None:
                                    bird_temp = self.temperature
                                    if bird_temp is not None and abs(bird_temp - self.ambient_temperature) < 1.5:
                                        bird_id = f'{x1}_{y1}_{x2}_{y2}'
                                        if bird_id not in self.detected_birds:
                                            self.bird_count += 1
                                            self.detected_birds[bird_id] = time.time()

                                            # Write bird detection to SQLite
                                            self.cursor.execute('''INSERT INTO bird_count (class, confidence, timestamp)
                                                                   VALUES (?, ?, ?)''',
                                                                (self.class_names[cls], confidence, time.strftime('%Y-%m-%d %H:%M:%S')))
                                            self.conn.commit()

                                        # Object details
                                        org = [x1, y1]
                                        font = cv2.FONT_HERSHEY_SIMPLEX
                                        fontScale = 1
                                        color = (255, 0, 0)
                                        thickness = 2

                                        cv2.putText(img, self.class_names[cls], org, font, fontScale, color, thickness)

            # Remove old entries
            current_time = time.time()
            self.detected_birds = {k: v for k, v in self.detected_birds.items() if current_time - v < 10}  # Keep entries for 10 seconds

            # Publish the image with detected objects
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            self.publisher.publish(image_msg)

    def end_route(self):
        route_end_time = time.time()
        route_duration = route_end_time - self.route_start_time
        self.route_count += 1

        # Write route summary to SQLite
        self.cursor.execute('''INSERT INTO route_summary (route_count, bird_count, route_duration, timestamp)
                               VALUES (?, ?, ?, ?)''',
                            (self.route_count, self.bird_count, route_duration, time.strftime('%Y-%m-%d %H:%M:%S')))
        self.conn.commit()

        # Reset for the next route
        self.bird_count = 0
        self.route_start_time = time.time()

    def __del__(self):
        """Ensure the connection to the database is closed properly."""
        self.conn.close()

def signal_handler(sig, frame):
    print('Signal received, shutting down gracefully...')
    node.end_route()  # Ensure the final route data is saved
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

def main():
    rclpy.init()
    global node
    node = ObjectDetectionNode()
    signal.signal(signal.SIGINT, signal_handler)  # Capture Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Capture termination signals

    route_interval = 4 * 60 * 60  # 4 hours in seconds
    start_time = time.time()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_time >= route_interval:
            node.end_route()
            start_time = time.time()  # Reset start time for the next route

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
