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
            # RViz2 publie l'outil "2D Goal Pose" sur /goal_pose par défaut
            # (contrairement à ROS1 où "2D Nav Goal" publiait sur
            # /move_base_simple/goal, topic écouté en dur dans le node).
            ('/move_base_simple/goal', '/goal_pose'),
            # Décommentez et adaptez si vos autres topics ont un namespace :
            # ('/odom', '/pguard/odom'),
            # ('/scan', '/pguard/scan'),
            # ('/cmd_vel', '/pguard/cmd_vel'),
        ],
    )

    return LaunchDescription([prm_nav])