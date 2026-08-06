import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('pguard_autodock')

    # IMPORTANT: adapte ces deux valeurs par defaut aux VRAIS topics bruts
    # publies par le plugin camera de ton xacro (verifie avec `ros2 topic
    # list` une fois Gazebo lance).
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/camera/image_raw',
        description='Topic image BRUTE publie par le plugin camera Gazebo du robot')
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/camera/camera_info',
        description='Topic camera_info BRUT associe au meme plugin camera')

    # Contournement d'un comportement de libgazebo_ros_camera.so (Gazebo
    # Classic) : malgre <frameName>camera_link</frameName> correctement
    # renseigne dans le SDF, le plugin publie image_raw/camera_info avec un
    # header.frame_id errone (constate empiriquement : 'base_footprint' --
    # vraisemblablement le lien canonique du modele plutot que le vrai lien
    # camera). Ce relais corrige uniquement le frame_id avant qu'apriltag_ros
    # ne le consomme -- voir scripts/camera_frame_fix.py pour le detail.
    # Necessite l'ajout du lien camera_link_optical dans le xacro du robot
    # (cf. patch_camera_optical_frame.txt).
    camera_frame_fix_node = Node(
        package='pguard_autodock',
        executable='camera_frame_fix.py',
        name='camera_frame_fix',
        output='screen',
        parameters=[{
            'input_image_topic': LaunchConfiguration('image_topic'),
            'input_camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'output_image_topic': '/camera_fixed/image_raw',
            'output_camera_info_topic': '/camera_fixed/camera_info',
            'correct_frame_id': 'camera_link_optical',
            'use_sim_time': True,
        }],
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_ros',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'apriltag.yaml'),
            os.path.join(pkg, 'config', 'tags.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[
            # Pointe desormais sur les topics CORRIGES par camera_frame_fix,
            # pas directement sur /camera/... .
            ('image_rect', '/camera_fixed/image_raw'),
            ('camera_info', '/camera_fixed/camera_info'),
        ],
    )

    return LaunchDescription([
        image_topic_arg,
        camera_info_topic_arg,
        camera_frame_fix_node,
        apriltag_node,
    ])