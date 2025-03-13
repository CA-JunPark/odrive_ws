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
        
        # Initialize wheel velocities to zero.
        self.v_left_cmd = 0.0 # m/s
        self.v_right_cmd = 0.0 # m/s

        # Subscriber for /odometry/filtered
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.current_yaw = 0.0 # updated every time we receive odometry data
        self.target_yaw = 0.0 # updated when we receive a forward command
        self.move_forward = False # true if we are moving forward, stop or turn return false
        self.yaw_threshold = 0.02 # Tuning 
        self.multiplier = 1.17 # Tuning

        # Subscriber for /cmd_vel commands
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.target_forward_vel = 0.0        

        # Publisher for odometry from wheel estimates (/odom_wheel)
        self.odom_pub = self.create_publisher(Odometry, '/odom_wheel', 10)

        # Try to connect to the left and right ODrive devices using provided serial numbers.
        try:
            self.leftW = odrive.find_any(serial_number="397534723331")  # left
            self.rightW = odrive.find_any(serial_number="396D346B3331")  # right
            self.leftW.clear_errors()
            self.rightW.clear_errors()
            
            self.setInputMode(1)
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
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.odom_wheel_callback)
        self.get_logger().info("Controller Ready to be Used")
        self.input_timer = self.create_timer(timer_period, self.input_timer_callback)
        
        # start with stopped position
        self.stop()
    
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
        """
        Callback for /cmd_vel subscription.
        Stores the latest commanded linear and angular velocity.
        """
        self.get_logger().info(
            f"Receive: linear={msg.linear.x:.2f} m/s, angular={msg.angular.z:.2f} rad/s"
        )
        
        if (msg.angular.z == 0):
            if not self.move_forward:
                if not (self.target_forward_vel == msg.linear.x):
                    self.target_yaw = self.current_yaw
            self.move_forward = True
            self.target_forward_vel = msg.linear.x
        else:
            self.move_forward = False
            self.target_forward_vel = msg.linear.x
        
        # Convert the commanded Twist into left and right wheel velocities.
        # Differential drive equations:
        #   v_left  = linear.x - (angular.z * wheel_base / 2)
        #   v_right = linear.x + (angular.z * wheel_base / 2)
        self.v_left_cmd = msg.linear.x - (msg.angular.z * self.wheel_base / 2.0)
        self.v_right_cmd = msg.linear.x + (msg.angular.z * self.wheel_base / 2.0)
        self.get_logger().info(
                f"Command: left={self.v_left_cmd:.2f} rev/s, right={self.v_right_cmd:.2f} rev/s"
            )
        
    def input_timer_callback(self):
        if (self.v_left_cmd == self.v_right_cmd == 0):
            self.stop()
        else:
            # CLOSED_LOOP_CONTROL state
            self.leftW.axis0.requested_state = 8
            self.rightW.axis0.requested_state = 8
            # Convert these from m/s to turns/sec using wheel circumference.
            left_cmd_turns = self.v_left_cmd / self.wheel_circumference
            right_cmd_turns = self.v_right_cmd / self.wheel_circumference
            
            # scale if yaw is off
            try:
                if self.move_forward:
                    yaw_error = self.current_yaw - self.target_yaw

                    if abs(yaw_error) > self.yaw_threshold:
                        if yaw_error > 0:  # Turn left to correct
                            left_cmd_turns *= self.multiplier 
                        else:  # Turn right to correct
                            right_cmd_turns *= self.multiplier
                        # print(f"Yaw Error: {yaw_error} Left: {left_cmd_turns} Right: {right_cmd_turns}")

                # Send the adjusted velocity commands to the ODrive controllers
                self.leftW.axis0.controller.input_vel = left_cmd_turns
                self.rightW.axis0.controller.input_vel = right_cmd_turns
            except Exception as e:
                self.get_logger().error(f"Error sending command to ODrive devices: {e}")
    
        
    def odom_callback(self, msg: Odometry):
        """Callback for /odometry/filtered subscription."""
        # self.get_logger().info(f"Orientation Z: {orientation_z}")
        self.current_yaw = msg.pose.pose.orientation.z
        # self.get_logger().info(f"Forward {self.move_forward} Current Yaw: {self.current_yaw}")
    
    def odom_wheel_callback(self):
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
        self.move_forward = False
        self.target_forward_vel = 0.0
    
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
