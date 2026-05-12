from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context, *args, **kwargs):
    logger = get_logger('wh044.display')
    pkg_share = get_package_share_directory('wh044')
    model = LaunchConfiguration('model').perform(context)
    robot_description_topic = LaunchConfiguration('robot_description_topic').perform(context)
    joint_states_topic = LaunchConfiguration('joint_states_topic').perform(context)
    tf_prefix = LaunchConfiguration('tf_prefix').perform(context).strip()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        '1',
        'true',
        'yes',
    )
    use_gui = LaunchConfiguration('use_gui').perform(context).lower() in (
        '1',
        'true',
        'yes',
    )

    urdf_path = os.path.join(pkg_share, 'urdf', model)
    if not os.path.isfile(urdf_path):
        raise RuntimeError(f'URDF not found: {urdf_path}')

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if rviz_config == '':
        rviz_config = os.path.join(pkg_share, 'rviz', 'wh044.rviz')

    base_frame = f'{tf_prefix}/base_link' if tf_prefix != '' else 'base_link'

    static_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'world', base_frame],
        output='screen',
    )

    jsp_nodes = []
    if use_gui:
        try:
            get_package_share_directory('joint_state_publisher_gui')
            jsp_nodes.append(
                Node(
                    package='joint_state_publisher_gui',
                    executable='joint_state_publisher_gui',
                    name='joint_state_publisher_gui',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'robot_description': robot_description},
                    ],
                    remappings=[
                        ('/robot_description', robot_description_topic),
                        ('robot_description', robot_description_topic),
                        ('joint_states', joint_states_topic),
                    ],
                )
            )
        except PackageNotFoundError:
            use_gui = False

    if not use_gui:
        try:
            get_package_share_directory('joint_state_publisher')
            jsp_nodes.append(
                Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    name='joint_state_publisher',
                    output='screen',
                    parameters=[
                        {'use_sim_time': use_sim_time},
                        {'robot_description': robot_description},
                    ],
                    remappings=[
                        ('/robot_description', robot_description_topic),
                        ('robot_description', robot_description_topic),
                        ('joint_states', joint_states_topic),
                    ],
                )
            )
        except PackageNotFoundError:
            logger.warning(
                'Neither joint_state_publisher_gui nor joint_state_publisher is installed; '
                'starting a fallback zero joint state publisher (all joints at 0). '
                'Install joint_state_publisher(_gui) for interactive control.'
            )

            fallback_script = os.path.join(pkg_share, 'scripts', 'zero_joint_state_publisher.py')
            if not os.path.isfile(fallback_script):
                raise RuntimeError(f'Fallback script not found: {fallback_script}')

            python_exe = '/usr/bin/python3' if os.path.isfile('/usr/bin/python3') else 'python3'

            jsp_nodes.append(
                ExecuteProcess(
                    cmd=[
                        python_exe,
                        fallback_script,
                        '--urdf',
                        urdf_path,
                        '--rate',
                        '30',
                        '--topic',
                        joint_states_topic,
                    ],
                    output='screen',
                )
            )

    return [
        static_world_tf,
        *jsp_nodes,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='wh044_robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': use_sim_time},
                {'publish_robot_description': True},
                {'frame_prefix': f'{tf_prefix}/' if tf_prefix != '' else ''},
            ],
            remappings=[
                ('robot_description', robot_description_topic),
                ('joint_states', joint_states_topic),
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('robot_description', robot_description_topic),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'model',
                default_value='wh044.urdf',
                description='URDF file name under share/wh044/urdf/',
            ),
            DeclareLaunchArgument(
                'robot_description_topic',
                default_value='/wh044/robot_description',
                description='Topic for publishing robot_description (avoid collisions with other publishers)',
            ),
            DeclareLaunchArgument(
                'joint_states_topic',
                default_value='/wh044/joint_states',
                description='Topic for publishing / joint_states (avoid collisions with other robots)',
            ),
            DeclareLaunchArgument(
                'tf_prefix',
                default_value='wh044',
                description='Prefix for TF frames to avoid collisions (e.g. wh044 => wh044/base_link)',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use /clock if true',
            ),
            DeclareLaunchArgument(
                'use_gui',
                default_value='true',
                description='Use joint_state_publisher_gui if available',
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value='',
                description='Absolute path to an RViz2 config; empty uses the package default',
            ),
            OpaqueFunction(function=_make_nodes),
        ]
    )

