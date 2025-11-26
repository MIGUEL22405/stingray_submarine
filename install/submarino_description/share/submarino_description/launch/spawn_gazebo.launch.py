from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_robot',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'montaje_dron'
            ],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': '/robot_description'}]
        )
    ])
