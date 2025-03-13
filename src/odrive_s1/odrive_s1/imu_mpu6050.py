import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import smbus
from math import pi
import time

class IMU_MPU6050_Publisher(Node):
    def __init__(self):
        super().__init__('imu_MPU6050')
        self.publisher_ = self.create_publisher(Imu, '/imu_mpu6050', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize I2C (SMBus)
        self.bus = smbus.SMBus(1)  # For Raspberry Pi, bus=1
        self.address = 0x68       # MPU6050 I2C address
        self.power_mgmt_1 = 0x6b
        self.bus.write_byte_data(self.address, self.power_mgmt_1, 0)

        # Calibration offsets (only for used axes)
        self.gyro_z_offset = 0.2235
        self.accel_x_offset = 0.0116

        # Last known good values (initialized to 0)
        self.last_gyro_zout = 0
        self.last_accel_xout = 0

        # Perform calibration at startup (commented out as per your code)
        # self.calibrate_imu()

        self.get_logger().info("IMU Publisher node (2D) has started.")

    def read_byte(self, adr):
        try:
            value = self.bus.read_byte_data(self.address, adr)
            return value
        except (IOError, OSError) as e:
            self.get_logger().warn(f"Failed to read byte at address {adr}: {str(e)}")
            return 0  # Default fallback (no previous value stored here)

    def read_word(self, adr):
        try:
            high = self.bus.read_byte_data(self.address, adr)
            low = self.bus.read_byte_data(self.address, adr + 1)
            val = (high << 8) + low
            return val
        except (IOError, OSError) as e:
            self.get_logger().warn(f"Failed to read word at address {adr}: {str(e)}")
            return 0  # Default fallback (no previous value stored here)

    def read_word_2c(self, adr):
        try:
            val = self.read_word(adr)
            if val >= 0x8000:
                return -((65535 - val) + 1)
            return val
        except (IOError, OSError) as e:
            self.get_logger().warn(f"Failed to read word_2c at address {adr}: {str(e)}")
            # Return the last known good value based on the address
            if adr == 0x47:  # Gyro Z
                return self.last_gyro_zout
            elif adr == 0x3b:  # Accel X
                return self.last_accel_xout
            return 0  # Default for unexpected address

    def calibrate_imu(self):
        self.get_logger().info("Starting 2D IMU calibration. Keep the sensor stationary...")
        num_samples = 1000
        gyro_z_sum = 0.0
        accel_x_sum = 0.0

        gyro_scale = 131.0
        accel_scale = 16384.0

        for _ in range(num_samples):
            # Read raw data for used axes only
            gyro_zout = self.read_word_2c(0x47)  # Gyro Z
            accel_xout = self.read_word_2c(0x3b)  # Accel X

            # Convert to physical units
            gyro_z = (gyro_zout / gyro_scale) * (pi / 180.0)
            accel_x = (accel_xout / accel_scale) * 9.81

            # Accumulate sums
            gyro_z_sum += gyro_z
            accel_x_sum += accel_x

            # Update last known values (even during calibration)
            self.last_gyro_zout = gyro_zout
            self.last_accel_xout = accel_xout

            time.sleep(0.001)  # Small delay between samples

        # Calculate averages (offsets)
        self.gyro_z_offset = gyro_z_sum / num_samples
        self.accel_x_offset = accel_x_sum / num_samples

        self.get_logger().info(f"Calibration complete. Gyro Z offset: {self.gyro_z_offset:.4f} rad/s")
        self.get_logger().info(f"Accel X offset: {self.accel_x_offset:.4f} m/s²")

    def timer_callback(self):
        # Read raw values for used axes
        gyro_zout = self.read_word_2c(0x47)  # Gyro Z
        accel_xout = self.read_word_2c(0x3b)  # Accel X
        
        # Convert readings and apply offsets
        gyro_scale = 131.0
        gyro_z = ((gyro_zout / gyro_scale) * (pi / 180.0)) - self.gyro_z_offset
        
        accel_scale = 16384.0
        accel_x = ((accel_xout / accel_scale) * 9.81) - self.accel_x_offset

        # Update last known good values
        self.last_gyro_zout = gyro_zout
        self.last_accel_xout = accel_xout

        # Create and populate the Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Explicitly set all fields (unused axes to 0.0)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        # Publish the IMU message
        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMU_MPU6050_Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()