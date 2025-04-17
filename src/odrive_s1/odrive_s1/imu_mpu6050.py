import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import time

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 50)
        self.bus = smbus.SMBus(1)
        self.device_address = 0x68
        self.bus.write_byte_data(self.device_address, 0x6B, 0)  # Wake up

        self.timer = self.create_timer(0.02, self.publish_imu_data)
        self.get_logger().info('IMU Starts')

    def read_word(self, reg):
        h = self.bus.read_byte_data(self.device_address, reg)
        l = self.bus.read_byte_data(self.device_address, reg+1)
        val = (h << 8) + l
        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val

    def publish_imu_data(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Gyro (angular velocity)
        gyro_x = self.read_word(0x43) / 131.0
        gyro_y = self.read_word(0x45) / 131.0
        gyro_z = self.read_word(0x47) / 131.0
        imu_msg.angular_velocity.x = math.radians(gyro_x)
        imu_msg.angular_velocity.y = math.radians(gyro_y)
        imu_msg.angular_velocity.z = math.radians(gyro_z)

        # Accelerometer (not used in basic fusion, but can be added)
        # accel_x = self.read_word(0x3B) / 16384.0
        # accel_y = self.read_word(0x3D) / 16384.0
        # accel_z = self.read_word(0x3F) / 16384.0

        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  
    finally:
        node.destroy_node()
        rclpy.shutdown()
