import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Twist # type: ignore
from nav_msgs.msg import Odometry # type: ignore
import odrive

class OdriveNode(Node):
    def __init__(self):
        super().__init__('odrive_node')
        self.get_logger().info("Initializing ODrive S1 nodes...")

        # Declare parameters:
        # 6.5 inches in meters: 6.5 * 0.0254 = 0.1651 m (approximately)
        # circumference = 0.1651 * pi = 0.5186769471076749 m
        self.declare_parameter("wheel_circumference", 0.518677)
        self.declare_parameter("wheel_base", 0.51)
        self.wheel_circumference = self.get_parameter("wheel_circumference").value
        self.wheel_base = self.get_parameter("wheel_base").value

        # Subscriber for /cmd_vel commands
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for odometry from wheel estimates (/odom_wheel)
        self.odom_pub = self.create_publisher(Odometry, '/odom_wheel', 10)

        # Try to connect to the left and right ODrive devices using provided serial numbers.
        try:
            self.leftW = odrive.find_any(serial_number="397534723331")  # left
            self.rightW = odrive.find_any(serial_number="396D346B3331")  # right
            self.leftW.clear_errors()
            self.rightW.clear_errors()
            self.setInputMode(2)
            
            self.setVelRampRate(0.05)
            self.setInertia()
            
            self.get_logger().info("Successfully connected to both ODrive devices")
            self.get_logger().info(f"voltage L: {self.leftW.vbus_voltage} R: {self.rightW.vbus_voltage}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ODrive devices: {e}")
            # Optionally shutdown if connection fails:
            # rclpy.shutdown()
            return

        # Create a timer to periodically send commands and publish odometry
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Controller Ready to be Used")
    
        
    def setInputMode(self, mode=1):
        # 1 = Passthrough
        # 2 = Vel_Ramp
        self.leftW.axis0.controller.config.input_mode = mode
        self.rightW.axis0.controller.config.input_mode = mode
    
    def setVelRampRate(self, val):
        self.leftW.axis0.controller.config.vel_ramp_rate=val
        self.rightW.axis0.controller.config.vel_ramp_rate=val
    
    def setInertia(self, val=0.0368):
        self.leftW.axis0.controller.config.inertia = 0.0368
        self.rightW.axis0.controller.config.inertia = 0.0368
        
    def cmd_vel_callback(self, msg: Twist):
        self.get_logger().info(
            f"Receive: linear={msg.linear.x:.2f} m/s, angular={msg.angular.z:.2f} rad/s"
        )
        """
        Callback for /cmd_vel subscription.
        Stores the latest commanded linear and angular velocity.
        """
        # Convert the commanded Twist into left and right wheel velocities.
        # Differential drive equations:
        #   v_left  = linear.x - (angular.z * wheel_base / 2)
        #   v_right = linear.x + (angular.z * wheel_base / 2)
        v_left_cmd = msg.linear.x - (msg.angular.z * self.wheel_base / 2.0)
        v_right_cmd = msg.linear.x + (msg.angular.z * self.wheel_base / 2.0)
        
        if (v_left_cmd == v_right_cmd == 0):
            self.stop()
        else:
            # CLOSED_LOOP_CONTROL state
            self.leftW.axis0.requested_state = 8
            self.rightW.axis0.requested_state = 8
            # Convert these from m/s to turns/sec using wheel circumference.
            left_cmd_turns = v_left_cmd / self.wheel_circumference
            right_cmd_turns = v_right_cmd / self.wheel_circumference
            # # scale 
            left_cmd_turns = left_cmd_turns * 0.99
            right_cmd_turns = right_cmd_turns * 1.05
            self.get_logger().info(
                f"Command Turns/s: left={left_cmd_turns:.4f} turns/s, right={right_cmd_turns:.4f} turns/s"
            )
            try:
                # Send the velocity commands to the ODrive devices.
                self.leftW.axis0.controller.input_vel = left_cmd_turns
                self.rightW.axis0.controller.input_vel = right_cmd_turns
                
            except Exception as e:
                self.get_logger().error(f"Error sending command to ODrive devices: {e}")
        
    def timer_callback(self):
        """Encoder"""
        # Retrieve the estimated wheel velocities (in turns/sec) from the ODrive devices.
        try:
            v_left_est = self.leftW.axis0.vel_estimate * self.wheel_circumference  # convert to m/s
            v_right_est = self.rightW.axis0.vel_estimate * self.wheel_circumference  # convert to m/s
        except Exception as e:
            self.get_logger().error(f"Error reading velocity estimate: {e}")
            return

        # Compute overall estimated linear and angular velocity from wheel odometry
        linear_est = (v_left_est + v_right_est) / 2.0
        angular_est = (v_right_est - v_left_est) / self.wheel_base

        # Create an Odometry message and publish it on /odom_wheel
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        # The pose is not computed here, so it remains at zero.
        odom_msg.twist.twist.linear.x = linear_est
        odom_msg.twist.twist.angular.z = angular_est
        
        self.odom_pub.publish(odom_msg)

    def stop(self):
        # Idle state
        self.leftW.axis0.requested_state = 1
        self.rightW.axis0.requested_state = 1
    
def main(args=None):
    rclpy.init(args=args)
    node = OdriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stop")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
