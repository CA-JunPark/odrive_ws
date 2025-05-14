# Odrive_S1_ROS

Additional ROS packages:
- https://github.com/hiwad-aziz/ros2_mpu6050_driver
- https://github.com/odriverobotics/ros_odrive
    - ros_odrive/odrive_botwheel_explorer/description/urdf/diffbot_description.urdf.xacro is modified for our robot model.
    replace diffbot_description.urdf.xacro with our own urdf file in odrive_ws/src folder

Package Descriptions
- my_urdf: For PC (URDF package, display status in rviz)
- odrive_s1: For RP4 (Custom Encoder and teleop)
- ros_odrive: For RP4 (Official Odrive ROS package, only urdf file is modified)
- ros2_mpu6050_driver: For RP4 (MPU6050 ROS package)