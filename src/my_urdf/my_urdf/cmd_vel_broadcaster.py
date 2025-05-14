from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node

# This node subscribes to the topic /cmd_vel and publishes to /botwheel_explorer/cmd_vel
# The purpose is to convert the Twist message to TwistStamped message
class CmdVelBroadcaster(Node):
    def __init__(self):
        super().__init__("cmd_vel_broadcasster")
        # Subscribe to /cmd_vel topic with a queue size of 10
        self.subscription = self.create_subscription(Twist, '/cmd_vel',self.cmd_vel_callback, 10)
        # Publish to /botwheel_explorer/cmd_vel topic with a queue size of 10
        self.publisher = self.create_publisher(TwistStamped, '/botwheel_explorer/cmd_vel', 10)

        self.get_logger().info("Cmd_Vel_Broadcaster is Ready")

    # This function is called when a message is received from the /cmd_vel topic
    def cmd_vel_callback(self, msg:Twist):
        twistStamped = TwistStamped()
        # Set the timestamp of the message to the current time
        twistStamped.header.stamp = self.get_clock().now().to_msg()
        # Set the twist field of the message to the received Twist message
        twistStamped.twist = msg
        # Publish the message to the /botwheel_explorer/cmd_vel topic
        self.publisher.publish(twistStamped)
    
def main(args=None):
    # Initialize the ROS2 client library
    rclpy.init(args=args)
    # Create an instance of the CmdVelBroadcaster node
    node = CmdVelBroadcaster()
    try:
        # Spin the node, this will block until the node is stopped
        rclpy.spin(node)
    except KeyboardInterrupt:
        # If the user presses Ctrl-C, destroy the node and shutdown the ROS2 client library
        node.destroy_node()
        rclpy.shutdown()
    finally:
        # Destroy the node and shutdown the ROS2 client library
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # Call the main function
    main()
