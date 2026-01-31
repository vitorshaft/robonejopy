import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Temperature
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time
import sqlite3
import os

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/aves', 10)
        
        # Inscrições
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(Temperature, '/temperature', self.temperature_callback, 10)
        self.create_subscription(Temperature, '/ambient_temperature', self.ambient_temperature_callback, 10)

        # Caminhos
        model_path = '/home/robot/ws/src/robonejopy/robonejopy/best.onnx'
        db_path = '/home/robot/ws/robonejo.db'

        self.model = YOLO(model_path)
        self.class_names = ["chicken"]

        # SQLite
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.cursor = self.conn.cursor()
        self.create_tables()

        # Estado
        self.route_count, self.bird_count = self.recover_state()
        self.route_start_time = time.time()
        
        self.current_temp = None
        self.ambient_temp = None
        self.detected_birds = {}

        # Timer para verificar fim da rota
        self.route_interval = 4 * 60 * 60 
        self.create_timer(60.0, self.check_route_timer)
        self.get_logger().info("Nó de Detecção e SQLite iniciado!")

    def create_tables(self):
        self.cursor.execute('''CREATE TABLE IF NOT EXISTS route_summary (
                                id INTEGER PRIMARY KEY AUTOINCREMENT,
                                route_count INTEGER, bird_count INTEGER,
                                route_duration REAL, timestamp TEXT)''')
        self.cursor.execute('''CREATE TABLE IF NOT EXISTS bird_count (
                                id INTEGER PRIMARY KEY AUTOINCREMENT,
                                class TEXT, confidence REAL, timestamp TEXT)''')
        self.conn.commit()

    def recover_state(self):
        try:
            self.cursor.execute('SELECT route_count, bird_count FROM route_summary ORDER BY id DESC LIMIT 1')
            row = self.cursor.fetchone()
            return (row[0], row[1]) if row else (0, 0)
        except: return 0, 0

    def ambient_temperature_callback(self, msg): self.ambient_temp = msg.temperature
    def temperature_callback(self, msg): self.current_temp = msg.temperature

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame = cv2.resize(frame, (320, 240))

        results = self.model(frame, conf=0.4, verbose=False)

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                if x1 < 160 < x2 and y1 < 120 < y2:
                    if self.class_names[cls] == "chicken" and self.ambient_temp:
                        if self.current_temp and abs(self.current_temp - self.ambient_temp) < 1.5:
                            bird_id = f"{round(x1,-1)}_{round(y1,-1)}"
                            if bird_id not in self.detected_birds:
                                self.bird_count += 1
                                self.detected_birds[bird_id] = time.time()
                                self.save_bird(self.class_names[cls], conf)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

        now = time.time()
        self.detected_birds = {k: v for k, v in self.detected_birds.items() if now - v < 10}
        self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

    def save_bird(self, label, conf):
        self.cursor.execute('INSERT INTO bird_count (class, confidence, timestamp) VALUES (?, ?, ?)',
                            (label, conf, time.strftime('%Y-%m-%d %H:%M:%S')))
        self.conn.commit()

    def check_route_timer(self):
        if time.time() - self.route_start_time >= self.route_interval:
            self.end_route()

    def end_route(self):
        duration = time.time() - self.route_start_time
        self.route_count += 1
        self.cursor.execute('INSERT INTO route_summary (route_count, bird_count, route_duration, timestamp) VALUES (?, ?, ?, ?)',
                            (self.route_count, self.bird_count, duration, time.strftime('%Y-%m-%d %H:%M:%S')))
        self.conn.commit()
        self.get_logger().info(f"Fim da Rota! Total: {self.bird_count}")
        self.bird_count = 0
        self.route_start_time = time.time()

# --- ESTE É O TRECHO QUE FALTAVA ---
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando nó pelo teclado...')
    finally:
        # Tenta salvar o progresso atual antes de morrer
        node.end_route()
        node.conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()