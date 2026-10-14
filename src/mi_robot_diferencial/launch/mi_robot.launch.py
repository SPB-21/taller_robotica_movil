import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    pkg = get_package_share_directory('mi_robot_diferencial')

    xacro_path = os.path.join(
        pkg,
        'urdf',
        'mi_robot.urdf.xacro'
    )

    world_path = os.path.join(
        pkg,
        'worlds',
        'pista.sdf'
    )

    bridge_config = os.path.join(
        pkg,
        'config',
        'bridge.yaml'
    )

    rviz_config = os.path.join(
        pkg,
        'config',
        'mi_robot.rviz'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    return LaunchDescription([

        # Gazebo (-r: arranca la simulacion sin pausa)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v4', world_path],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': True
                }
            ]
        ),

        # Bridge ROS 2 <-> Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[
                {
                    'config_file': bridge_config
                }
            ]
        ),

        # Spawn del robot (con retardo para que Gazebo ya este listo)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-topic', 'robot_description',
                        '-name', 'mi_robot',
                        '-world', 'pista',
                        '-x', '0.5',
                        '-y', '0.0',
                        '-z', '0.1',
                        '-Y', '0.0'
                    ],
                    output='screen'
                ),
            ]
        ),

        # Odometria propia (cinematica de las llantas)
        Node(
            package='mi_robot_diferencial',
            executable='odometria_propia.py',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # Comparacion /odom (DiffDrive) vs /odom_propia
        Node(
            package='mi_robot_diferencial',
            executable='comparar_odometrias.py',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])
