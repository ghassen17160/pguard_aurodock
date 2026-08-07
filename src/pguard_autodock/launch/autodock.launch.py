import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('pguard_autodock')

    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'apriltag.launch.py')
        )

    )

    autodock_node = Node(
        package='pguard_autodock',
        executable='autodock_node',
        name='autodock_node',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'autodock_params.yaml'),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        apriltag_launch,
        autodock_node,
    ])
