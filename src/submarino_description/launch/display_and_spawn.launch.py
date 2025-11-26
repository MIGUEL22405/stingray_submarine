from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('submarino_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'montaje_dron_con_colores.urdf')
    
    # Cargar el contenido del URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Gazebo como proceso directo (MÁS CONFIABLE)
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn del robot
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'montaje_dron',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    # Delay para spawn
    delayed_spawn = TimerAction(
        period=8.0,
        actions=[spawn_robot_node]
    )

    return LaunchDescription([
        gazebo_process,
        robot_state_pub_node,
        rviz_node,
        delayed_spawn
    ])