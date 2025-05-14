import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    # urdf file is a xml file that describes the robot structure
    # (not used anymore since we are using botwheel explorer)
    urdf_file_name = 'airobot.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('my_urdf'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # rviz file is a configuration file that describes what to display in rviz
    rviz_file_name = 'airobot.rviz'
    rviz_config = os.path.join(get_package_share_directory('my_urdf'), rviz_file_name)

    # this node is used to estimate the robot's pose and velocity from the wheel encoders
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
        # this node is used to publish a static transform between the lidar and the base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_link_to_laser_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'laser']
        ),
        # this node is used to publish the robot's state (pose and velocity) 
        # (not used anymore since we are using botwheel explorer)
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
        #     arguments=[urdf]),

        # this node is used to convert the cmd_vel message to a TwistStamped message
        Node(
            package="my_urdf",
            executable="cmd_vel_broadcaster",
            name='cmd_vel_broadcaster',
            output='screen',
        ),
        # this node is used to display the robot's state in rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
        # this node is used to estimate the robot's pose and velocity from the wheel encoders
        robot_localization_node,
    ])