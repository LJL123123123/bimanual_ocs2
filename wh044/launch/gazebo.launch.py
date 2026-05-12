from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='wh044.urdf',
        description='URDF file name under share/wh044/urdf/',
    )
    entity_arg = DeclareLaunchArgument(
        'entity',
        default_value='wh044',
        description='Gazebo entity name',
    )

    model = LaunchConfiguration('model')
    entity = LaunchConfiguration('entity')

    urdf_path = PathJoinSubstitution([FindPackageShare('wh044'), 'urdf', model])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']
            )
        )
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen',
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_wh044',
        arguments=['-entity', entity, '-file', urdf_path],
        output='screen',
    )

    return LaunchDescription(
        [
            model_arg,
            entity_arg,
            gazebo_launch,
            static_tf,
            spawn,
        ]
    )
