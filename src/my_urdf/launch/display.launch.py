import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    urdf_file_name = 'airobot.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('my_urdf'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rviz_file_name = 'airobot.rviz'
    rviz_config = os.path.join(get_package_share_directory('my_urdf'), rviz_file_name)

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('my_urdf'), 'ekf.yaml'), 
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'frequency': 10.0}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_link_to_laser_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'laser']
        ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
        #     arguments=[urdf]),
        Node(
            package="my_urdf",
            executable="cmd_vel_broadcaster",
            name='cmd_vel_broadcaster',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
        robot_localization_node,
    ])