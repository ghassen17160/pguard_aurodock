#!/usr/bin/env python3
"""
nav2_bringup_pguard.launch.py

Lance la stack Nav2 complete pour la NAVIGATION sur carte fixe deja
construite (pguard_map.pgm/.yaml) :
  - map_server (charge la carte)
  - amcl (localisation)
  - controller_server / planner_server / behavior_server / bt_navigator /
    waypoint_follower (navigation)
  - deux lifecycle_manager (un pour la localisation, un pour la navigation)

Ordre de lancement recommande (3 terminaux, ou un launch parent qui
inclut les trois) :
  1) ton launch gazebo + robot_state_publisher + joint_state_publisher
     (celui que tu as deja)
  2) ton launch pointcloud_to_laserscan
     (SANS slam_toolbox cette fois : la carte est deja construite, on
     n'a plus besoin de SLAM en ligne, seulement de /scan pour amcl et
     l'obstacle_layer)
  3) CE fichier, avec l'argument map pointant vers ta carte, ex:
       ros2 launch pearlguard_description nav2_bringup_pguard.launch.py \\
         map:=/home/ghassen/Desktop/2206/git/pguard1_ws/maps/pguard_map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('pearlguard_description')
    default_params_file = os.path.join(pkg_description, 'config', 'nav2_params.yaml')
    default_map_file = os.path.join(
        os.path.expanduser('~'), 'Desktop', '2206', 'git', 'pguard1_ws',
        'maps', 'pguard_map.yaml'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Chemin vers nav2_params.yaml'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Utiliser /clock de Gazebo'
    )
    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Chemin vers pguard_map.yaml'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])