"""
botwheel_encoder_node.py

This node subscribes to '/joint_states' message and estimates the robot's pose (x, y, theta) and velocity (linear x, angular z) based on the wheel encoders' readings.

The estimated pose and velocity are published as an 'Odometry' message.

The node has the following parameters:

- wheel_base: the distance between the two wheels. Default is 0.51 m.
- wheel_radius: the radius of each wheel. Default is 0.0855 m.

The node subscribes to the following topic:

- /joint_states: a 'JointState' message containing the position and velocity of the left and right wheel.

The node publishes the following topic:

- /odom_wheel: an 'Odometry' message containing the estimated pose and velocity of the robot.
"""

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from nav_msgs.msg import Odometry # type: ignore
from sensor_msgs.msg import JointState # type: ignore
from math import cos, sin

class BotwheelEncoderNode(Node):
    def __init__(self):
        """
        Initialize the node.

        The node subscribes to '/joint_states' and publishes an 'Odometry' message.
        """
        super().__init__('botwheel_encoder_node')
        self.get_logger().info("Initializing Botwheel Encoder Node...")

        # Subscribe to '/joint_states'
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        # Publish an 'Odometry' message
        self.odom_pub = self.create_publisher(Odometry, '/odom_wheel', 10)

        # Initialize variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.wheel_base = 0.51 # m
        self.wheel_radius = 0.0855 # m
        self.vel_left = 0.0 # m/s
        self.vel_right = 0.0 # m/s
        self.current_pos_left = None
        self.current_pos_right = None
        self.pos_offset_left = 0.0
        self.pos_offset_right = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Create a timer to publish the 'Odometry' message at a rate of 1 kHz
        self.publishing = self.create_timer(0.001, self.publish_odom)

    def joint_states_callback(self, msg: JointState):
        """
        Callback function for the '/joint_states' subscription.

        This function updates the estimated pose and velocity based on the wheel encoders' readings.
        """
        if self.current_pos_left is None:
            # Initialize the position offset of the left and right wheel
            self.pos_offset_left = msg.position[0] # rad
            self.pos_offset_right = msg.position[1] # rad
            self.current_pos_left = 0.0
            self.current_pos_right = 0.0

        # Calculate the delta theta of the left and right wheel
        pos_left = msg.position[0] - self.pos_offset_left # rad
        pos_right = msg.position[1] - self.pos_offset_right # rad
        delta_theta_L = pos_left - self.current_pos_left
        delta_theta_R = pos_right - self.current_pos_right

        # Update the current position of the left and right wheel
        self.current_pos_left = pos_left
        self.current_pos_right = pos_right

        # Update the estimated pose (x, y, theta)
        self.current_theta = self.current_theta + (self.wheel_radius / self.wheel_base) * (delta_theta_R- delta_theta_L)
        self.current_x = self.current_x + (self.wheel_radius / 2.0) * (delta_theta_R + delta_theta_L) * cos(self.current_theta)
        self.current_y = self.current_y + (self.wheel_radius / 2.0) * (delta_theta_R + delta_theta_L) * sin(self.current_theta)

        # Update the estimated velocity (linear x, angular z)
        self.linear_x = (msg.velocity[1] + msg.velocity[0]) / 2.0
        self.angular_z = (msg.velocity[1] - msg.velocity[0]) / self.wheel_base

    def publish_odom(self):
        """
        Timer callback function to publish the 'Odometry' message.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.current_x
        odom_msg.pose.pose.position.y = self.current_y
        odom_msg.pose.pose.orientation.z = self.current_theta
        # caculate Linear.x and Angular.z with velocities of the wheels
        odom_msg.twist.twist.linear.x = self.linear_x
        odom_msg.twist.twist.angular.z = self.angular_z
        self.odom_pub.publish(odom_msg)

def main(args=None):
    """
    Main function to start the node.
    """
    rclpy.init(args=args)
    node = BotwheelEncoderNode()
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
