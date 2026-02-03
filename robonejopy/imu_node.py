import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature, MagneticField
import smbus
import time
import math

# --- CONFIGURAÇÃO ---
# Endereços I2C
MPU_ADDR = 0x68
QMC_ADDR = 0x0D

# DECLINAÇÃO MAGNÉTICA (Ajuste para sua cidade!)
# Para Recife é aprox -21.0 graus. Para Lisboa é aprox -1.5 graus.
# Valores negativos = Oeste (West), Positivos = Leste (East).
MAGNETIC_DECLINATION_DEG = -21.0 

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        self.bus = smbus.SMBus(1)
        
        # --- Configuração MPU6050 ---
        try:
            self.bus.write_byte_data(MPU_ADDR, 0x6B, 0)
        except Exception as e:
            self.get_logger().error(f"Erro MPU6050: {e}")

        # --- Configuração Magnetômetro (QMC5883L) ---
        self.use_magnetometer = False
        try:
            self.bus.write_byte_data(QMC_ADDR, 0x09, 0x1D) 
            self.bus.write_byte_data(QMC_ADDR, 0x0B, 0x01)
            self.use_magnetometer = True
            self.get_logger().info("Magnetômetro QMC5883L detetado e ativado!")
        except Exception:
            self.get_logger().warn("Magnetômetro não encontrado. A usar apenas Giroscópio (Drift alto no Yaw).")

        self.alpha = 0.98
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.first_run = True # Para inicializar o Yaw instantaneamente
        self.last_time = self.get_clock().now()

        # CALIBRAÇÃO (Substitua pelos valores que encontrar girando o sensor)
        self.mag_offset_x = 0 
        self.mag_offset_y = 0

        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.temp_publisher = self.create_publisher(Temperature, '/ambient_temperature', 10)
        self.mag_publisher = self.create_publisher(MagneticField, '/magnetic_field', 10)

        self.timer = self.create_timer(0.02, self.publish_imu_data)

    def read_word_2c(self, addr, reg):
        try:
            high = self.bus.read_byte_data(addr, reg)
            low = self.bus.read_byte_data(addr, reg+1)
            val = (high << 8) + low
            return -((65535 - val) + 1) if val >= 0x8000 else val
        except:
            return 0

    def read_qmc_data(self):
        try:
            data = self.bus.read_i2c_block_data(QMC_ADDR, 0x00, 6)
            x = (data[1] << 8) | data[0]
            y = (data[3] << 8) | data[2]
            z = (data[5] << 8) | data[4]
            
            if x > 32768: x -= 65536
            if y > 32768: y -= 65536
            if z > 32768: z -= 65536
            return x, y, z
        except:
            return None, None, None

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0.0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q

    def publish_imu_data(self):
        # 1. Leitura MPU
        ax = self.read_word_2c(MPU_ADDR, 0x3B) / 16384.0
        ay = self.read_word_2c(MPU_ADDR, 0x3D) / 16384.0
        az = self.read_word_2c(MPU_ADDR, 0x3F) / 16384.0
        # Bias is hardcoded from calibrate_imu.py util
        gx = math.radians(self.read_word_2c(MPU_ADDR, 0x43) / 131.0) - 0.007403
        gy = math.radians(self.read_word_2c(MPU_ADDR, 0x45) / 131.0) + 0.006081
        gz = math.radians(self.read_word_2c(MPU_ADDR, 0x47) / 131.0) + 0.003978

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 2. Roll e Pitch (Acelerômetro)
        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

        # Filtro Complementar
        self.roll = self.alpha * (self.roll + gx * dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * accel_pitch

        # 2.1. Leitura da Temperatura (Registrador 0x41)
        raw_temp = self.read_word_2c(MPU_ADDR, 0x41)
        # Fórmula do Datasheet: Temperature in degrees C = (temp_out / 340) + 36.53
        actual_temp = (raw_temp / 340.0) + 36.53
        temp_msg = Temperature()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = 'imu_link'
        temp_msg.temperature = actual_temp
        # Variância (opcional): se você souber a precisão do sensor (ex: 1 grau)
        temp_msg.variance = 1.0 

        self.temp_publisher.publish(temp_msg)
        
        # 3. Magnetómetro e Declinação
        mag_yaw = self.yaw 
        
        if self.use_magnetometer:
            mx, my, mz = self.read_qmc_data()
            if mx is not None:
                mx -= self.mag_offset_x
                my -= self.mag_offset_y
                
                # <--- 3. PUBLICAÇÃO DO CAMPO MAGNÉTICO RAW (PARA O DEAD RECKONING)
                mag_msg = MagneticField()
                mag_msg.header.stamp = current_time.to_msg()
                mag_msg.header.frame_id = 'imu_link'
                
                # Convertendo para Tesla (Aprox: 1 Gauss = 1e-4 Tesla, QMC sensibilidade ~3000LSB/Gauss)
                # O Dead Reckoning usa atan2, então a escala absoluta não afeta o ângulo, 
                # mas para ficar no padrão ROS convertemos aproximadamente.
                scale_factor = 1.0 / 3000.0 * 0.0001 
                mag_msg.magnetic_field.x = mx * scale_factor
                mag_msg.magnetic_field.y = my * scale_factor
                mag_msg.magnetic_field.z = mz * scale_factor
                self.mag_publisher.publish(mag_msg)

                # Compensação de Tilt (Nivelamento matemático)
                Xh = mx * math.cos(self.pitch) + my * math.sin(self.roll) * math.sin(self.pitch) - mz * math.cos(self.roll) * math.sin(self.pitch)
                Yh = my * math.cos(self.roll) + mz * math.sin(self.roll)
                
                mag_yaw = math.atan2(Yh, Xh)
                
                # --- APLICAÇÃO DA DECLINAÇÃO ---
                declination_rad = math.radians(MAGNETIC_DECLINATION_DEG)
                mag_yaw += declination_rad
                
                # Ajuste para manter entre -PI e PI
                if mag_yaw > math.pi: mag_yaw -= 2*math.pi
                if mag_yaw < -math.pi: mag_yaw += 2*math.pi

        # 4. Fusão do Yaw
        gyro_yaw_step = gz * dt
        
        if self.use_magnetometer:
            # Na primeira execução, "saltamos" direto para o valor do magnetómetro
            if self.first_run:
                self.yaw = mag_yaw
                self.first_run = False
            else:
                # Filtro Complementar para suavizar o magnetómetro com o giroscópio
                diff = mag_yaw - self.yaw
                # Corrigir o "salto" de -180 para 180
                while diff > math.pi: diff -= 2*math.pi
                while diff < -math.pi: diff += 2*math.pi
                
                # Confia 98% no Giro (suave), 2% no Magnetómetro (correção de drift)
                self.yaw = self.yaw + gyro_yaw_step + (1.0 - self.alpha) * diff
        else:
            self.yaw += gyro_yaw_step # Apenas giroscópio (vai ter drift)

        # 5. Publicação
        msg = Imu()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'imu_link'

        q = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax*9.81, ay*9.81, az*9.81

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(IMUNode())
    rclpy.shutdown()