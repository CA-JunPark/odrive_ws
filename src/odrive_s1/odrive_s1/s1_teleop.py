import rclpy
from rclpy.node import Node
import getch
from geometry_msgs.msg import Twist

def mps_to_turns_per_sec(mps):
    """
    Convert linear velocity in meters per second (m/s) to turns per second (turn/s).
    
    Parameters:
        mps (float): Linear velocity in meters per second.

    Returns:
        float: Rotational speed in turns per second.
    """
    # Calculate the circumference (m)
    circumference_m = 0.5186769471076749 # 6.5 inch in meters * pi
    # Rotational speed = linear speed / circumference
    return mps / circumference_m

class S1TeleopPub(Node):
    def __init__(self):
        super().__init__('s1_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('S1 Teleop Started')
        
    def main_loop(self):
        speed =  0.2 #m/s
        spinScale = 3.3
        twist = Twist()
        while rclpy.ok():
            key = getch.getch()

            if key == 'w':
                twist.linear.x = speed
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -speed
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = speed * spinScale
            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -(speed) * spinScale
            elif key == 'q':
                self.get_logger().info('Quit')
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
            self.publisher.publish(twist)
            self.get_logger().info(f'Publishing: linear_x = {twist.linear.x}, angular_z = {twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = S1TeleopPub()
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
