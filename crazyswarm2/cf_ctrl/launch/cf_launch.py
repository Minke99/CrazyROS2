from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cf_ctrl',
            executable='joystick_node',
            name='joystick_node',
            output='screen'
        ),
        Node(
            package='cf_ctrl',
            executable='mocap_ctrl_node',
            name='mocap_ctrl_node',
            output='screen'
        )
    ])
