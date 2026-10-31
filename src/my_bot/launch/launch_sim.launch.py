import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'my_bot'  # your package name
    world_file_name = 'worlds/college.world'  # change to your actual world file name

    # Full path to world file
    world_path = os.path.join(
        get_package_share_directory(package_name),
        world_file_name
    )

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_path}.items()
    )



    # control = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource([
    #     os.path.join(get_package_share_directory(package_name), 'launch', 'control.launch.py')
    # ])
    # )


    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )


    # ✅ Plant Detector Node
    detector_node = Node(
        package='my_bot',
        executable='plant_detector_node.py',
        name='plant_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )



    return LaunchDescription([
        rsp,
        gazebo,
        # control, # add this here
        spawn_entity,
        detector_node,
        
    ])
