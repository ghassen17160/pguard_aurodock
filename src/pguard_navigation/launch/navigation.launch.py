import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('pguard_navigation')
    params_file = os.path.join(pkg, 'config', 'pguard_nav_params.yaml')

    prm_nav = Node(
        package='pguard_navigation',
        executable='prm_nav_node',
        name='prm_nav_node',
        output='screen',
        parameters=[params_file],
        remappings=[

            ('/move_base_simple/goal', '/goal_pose'),

        ],
    )

    return LaunchDescription([prm_nav])
