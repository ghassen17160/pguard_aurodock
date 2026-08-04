import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('pearlguard_description')

    map_yaml = LaunchConfiguration('map')
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.path.expanduser('~'), 'Desktop', '2206', 'git',
            'pguard1_ws', 'maps', 'pguard_map.yaml'
        ),
        description='Chemin vers le fichier yaml de la carte'
    )

    pc2scan = Node(
    package='pointcloud_to_laserscan',
    executable='pointcloud_to_laserscan_node',
    name='pointcloud_to_laserscan',
    remappings=[
        ('cloud_in', '/velodyne_points'),
        ('scan',     '/scan'),
    ],
    parameters=[{
        'target_frame':       'base_link',
        'min_height':          0.1,
        'max_height':          0.35,
        'angle_min':          -3.14159,
        'angle_max':           3.14159,
        'angle_increment':     0.00872,
        'scan_time':           0.1,
        'range_min':           0.8,
        'range_max':          20.0,
        'use_inf':             True,
        'use_sim_time':        True,
    }],
)

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml,
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'amcl_params.yaml')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    rviz_config = os.path.join(pkg, 'config', 'pguard_nav.rviz')
    rviz2 = TimerAction(
        period=2.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )]
    )

    return LaunchDescription([
        declare_map_arg,
        pc2scan,
        map_server,
        amcl,
        lifecycle_manager,
        rviz2,
    ])