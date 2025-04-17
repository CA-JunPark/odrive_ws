import sys
import select
import termios
import tty
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


class BotwheelTeleop(Node):
    def __init__(self):
        super().__init__('botwheel_teleop')
        self.publisher = self.create_publisher(TwistStamped, '/botwheel_explorer/cmd_vel', 10)
        self.get_logger().info('Botwheel Teleop Started')

    def main_loop(self):
        """
        Teleop loop

        Read char from stdin and publish TwistStamped message accordingly.
        'w' : forward
        's' : backward
        'a' : left
        'd' : right
        'q' : quit
        """
        settings = termios.tcgetattr(sys.stdin.fileno())
        try:
            speed = 0.4
            spinSpeed = 1.2
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key == 'w':
                    cmd_vel = TwistStamped()
                    cmd_vel.header.stamp = self.get_clock().now().to_msg()
                    cmd_vel.twist.linear.x = speed
                    cmd_vel.twist.angular.z = 0.0
                    # self.get_logger().info('forward')
                    self.publisher.publish(cmd_vel)
                elif key == 's':
                    cmd_vel = TwistStamped()
                    cmd_vel.header.stamp = self.get_clock().now().to_msg()
                    cmd_vel.twist.linear.x = -speed
                    cmd_vel.twist.angular.z = 0.0
                    # self.get_logger().info('backward')
                    self.publisher.publish(cmd_vel)
                elif key == 'a':
                    cmd_vel = TwistStamped()
                    cmd_vel.header.stamp = self.get_clock().now().to_msg()
                    cmd_vel.twist.linear.x = 0.0
                    cmd_vel.twist.angular.z = spinSpeed
                    # self.get_logger().info('left')
                    self.publisher.publish(cmd_vel)
                elif key == 'd':
                    cmd_vel = TwistStamped()
                    cmd_vel.header.stamp = self.get_clock().now().to_msg()
                    cmd_vel.twist.linear.x = 0.0
                    cmd_vel.twist.angular.z = -spinSpeed
                    # self.get_logger().info('right')
                    self.publisher.publish(cmd_vel)
                elif key == 'q':
                    self.get_logger().info('Quit')
                    break
                else:
                    self.get_logger().info('Quit')
                    break
        finally:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = BotwheelTeleop()
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
