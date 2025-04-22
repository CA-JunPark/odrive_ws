from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node

class CmdVelBroadcaster(Node):
    def __init__(self):
        super().__init__("cmd_vel_broadcasster")
        self.subscription = self.create_subscription(Twist, '/cmd_vel',self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(TwistStamped, '/botwheel_explorer/cmd_vel', 10)

        self.get_logger().info("Cmd_Vel_Broadcaster is Ready")

    def cmd_vel_callback(self, msg:Twist):
        twistStamped = TwistStamped()
        twistStamped.header.stamp = self.get_clock().now().to_msg()
        twistStamped.twist = msg
        self.publisher.publish(twistStamped)
    
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
