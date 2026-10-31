from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_bot')

    return LaunchDescription([
        # Launch Gazebo with empty world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(
                    os.path.join(pkg_dir, 'description', 'robot.urdf.xacro')
                ).read()
            }]
        ),

        # Plant detector node
        Node(
            package='my_bot',
            executable='plant_detector_node.py',
            name='plant_detector',
            output='screen',
            parameters=[],
        ),
    ])
