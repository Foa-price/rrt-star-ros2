#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('youbot_description')

    sdf_dir = os.path.join(pkg_share, 'sdf')
    world_path = os.path.join(sdf_dir, 'world.sdf')
    urdf_path = os.path.join(pkg_share, 'urdf', 'youbot.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'view.rviz')

    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = f"{sdf_dir}:{existing_model_path}" if existing_model_path else sdf_dir

    set_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }],
        remappings=[
            ('joint_states', '/youbot/joint_states_clean')
        ]
    )

    # Noeud relais : retire le prefixe 'youbot::' des noms d'articulations
    # publies par le plugin Gazebo, pour que robot_state_publisher puisse
    # les faire correspondre aux noms definis dans youbot.urdf.
    start_joint_state_debridge_cmd = Node(
        package='youbot_description',
        executable='joint_state_debridge.py',
        name='joint_state_debridge',
        output='screen'
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(set_model_path_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_debridge_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
