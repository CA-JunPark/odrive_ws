import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

# This node subscribes to the topic /odom_wheel and publishes to /botwheel_explorer/cmd_vel
class CmdVelPlotter(Node):
    def __init__(self):
        super().__init__('odom_wheel_plotter')
        
        # Initialize deques to store last 100 values
        self.linear_x = deque(maxlen=100)
        self.angular_z = deque(maxlen=100)
        
        # Initialize with zeros
        for _ in range(100):
            self.linear_x.append(0.0)
            self.angular_z.append(0.0)
            
        # Define QoS profile to match common odometry publishers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriber with explicit QoS
        # A subscriber is a node that listens to messages from a publisher. 
        # The subscriber is responsible for receiving the messages and 
        # processing them accordingly.
        self.subscription = self.create_subscription(
            Odometry,
            '/odom_wheel',
            self.cmd_vel_callback,
            qos)  # Pass the QoS profile here
        
        # Set up the plot
        plt.ion()  # Turn on interactive mode
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Linear velocity plot
        self.ax1.set_title('Linear X Velocity')
        self.ax1.set_ylim(-0.5, 0.5)  # Adjust as needed
        self.ax1.set_ylabel('Velocity (m/s)')
        self.line_lx, = self.ax1.plot(self.linear_x, label='Linear X', color='blue')
        self.ax1.legend()
        self.ax1.grid(True)
        
        # Angular velocity plot
        self.ax2.set_title('Angular Z Velocity')
        self.ax2.set_ylim(-2.0, 2.0)  # Adjust as needed
        
        self.ax2.set_ylabel('Velocity (rad/s)')
        self.line_az, = self.ax2.plot(self.angular_z, label='Angular Z', color='green')
        self.ax2.legend()
        self.ax2.grid(True)
        
        plt.tight_layout()
        plt.show()

    def cmd_vel_callback(self, msg):
        # Append new values from Odometry message
        # The Odometry message is a ROS message that contains information about the 
        # pose and velocity of a robot. See: https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
        self.linear_x.append(msg.twist.twist.linear.x)
        self.angular_z.append(msg.twist.twist.angular.z)

        # Calculate averages and max
        avg_linear_x = np.mean(list(self.linear_x))
        avg_angular_z = np.mean(list(self.angular_z))
        max_linear_x = np.max(list(self.linear_x))
        max_angular_z = np.max(list(self.angular_z))
        min_linear_x = np.min(list(self.linear_x))
        min_angular_z = np.min(list(self.angular_z))

        # Set y-limits
        self.ax1.set_ylim(min_linear_x - 0.3, max_linear_x + 0.3)
        self.ax2.set_ylim(min_angular_z - 0.3, max_angular_z + 0.3)
        
        # Update plot data
        self.line_lx.set_ydata(self.linear_x)
        self.line_az.set_ydata(self.angular_z)

        self.ax2.set_xlabel(f'Samples lin_x:{msg.twist.twist.linear.x} ang_z: {msg.twist.twist.angular.z}')
        
        # Redraw the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPlotter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    plt.close()

if __name__ == '__main__':
    main()