import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('pearlguard_description')

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

    slam_localization = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
            {
                'use_sim_time': True,
                'mode': 'localization',
                'map_file_name': '/home/ghassen/Desktop/2206/git/pguard1_ws/maps/pguard_map_serialized',
                'map_start_pose': [0.0, 0.0, 0.0],
            },
        ],
    )

    rviz_config = os.path.join(pkg, 'config', 'pguard_nav.rviz')
    rviz2 = TimerAction(
        period=3.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )]
    )

    return LaunchDescription([pc2scan, slam_localization, rviz2])