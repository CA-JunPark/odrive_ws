"""
This launch file is used to run the slam_toolbox node for the first time user of ROS2.
It is recommended to run this launch file in a new terminal window.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('my_urdf'),
        "slam_toolbox_config.yaml")
    
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_file]
        )
    ])