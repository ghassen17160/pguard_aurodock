import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('pguard_autodock')
    model_path = os.path.join(pkg, 'models', 'pguard_charging_station', 'model.sdf')
    models_dir = os.path.join(pkg, 'models')


    existing_gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_dir + (':' + existing_gazebo_model_path if existing_gazebo_model_path else '')
    )

    x_arg = DeclareLaunchArgument('x', default_value='2.0',
                                   description='Position X de la station dans le monde')
    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                                   description='Position Y de la station dans le monde')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='3.14159',
                                     description=(
                                         'Orientation de la station (rad). Le panneau AprilTag '
                                         'fait face a +X dans le repere du modele : avec yaw=pi, '
                                         'le panneau fait face a -X dans le monde (le robot doit '
                                         'donc approcher par -X). Ajustez selon la disposition '
                                         'reelle souhaitee dans votre carte.'
                                     ))
    entity_name_arg = DeclareLaunchArgument('entity_name', default_value='pguard_charging_station')

    spawn_station = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', model_path,
            '-entity', LaunchConfiguration('entity_name'),
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.0',
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    return LaunchDescription([
        set_model_path,
        x_arg,
        y_arg,
        yaw_arg,
        entity_name_arg,
        spawn_station,
    ])
