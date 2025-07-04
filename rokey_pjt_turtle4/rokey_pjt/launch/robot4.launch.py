from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rokey_pjt',
            executable='robot4_vision',
            name='robot4_vision',
            output='screen',
            remappings=[
                ('/tf', '/robot4/tf'),
                ('/tf_static', '/robot4/tf_static')
            ]
        ),
        Node(
            package='rokey_pjt',
            executable='main_controller',
            name='main_controller',
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='robot4_navigation',
            name='robot4_navigation',
            output='screen'
        )
    ])