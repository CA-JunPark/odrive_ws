from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics',
    maintainer_email='cskoko5786@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher_demo = my_urdf.state_publisher_demo:main',
            'odom_wheel_plotter = my_urdf.odom_wheel_plotter:main',
            'botwheel_teleop_pc = my_urdf.botwheel_teleop_pc:main',
            'cmd_vel_broadcaster = my_urdf.cmd_vel_broadcaster:main',
        ],
    },
)
