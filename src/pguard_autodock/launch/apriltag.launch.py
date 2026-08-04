import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('pguard_autodock')

    # IMPORTANT: adapte ces deux valeurs par defaut aux VRAIS topics publies
    # par le plugin camera de ton xacro (verifie avec `ros2 topic list` une
    # fois Gazebo lance). Exemples frequents selon le plugin utilise :
    #   - libgazebo_ros_camera.so       -> /camera/image_raw, /camera/camera_info
    #   - libgazebo_ros_camera_sensor.so -> /<camera_name>/image_raw, .../camera_info
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='Topic image brute publie par le plugin camera Gazebo du robot')
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/camera_info',
        description='Topic camera_info associe au meme plugin camera')

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_ros',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'apriltag.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[
            ('image_rect', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
        ],
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        apriltag_node,
    ])