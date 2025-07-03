from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rokey_pjt',
            executable='vital',
            name='rppg_chrom_node',
            output='screen'
        )
    ])
