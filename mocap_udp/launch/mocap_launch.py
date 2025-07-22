import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明配置文件路径的启动参数
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                FindPackageShare('mocap_udp').find('mocap_udp'), 'config', 'mocap_params.yaml'),
            description='配置文件的路径'
        ),
        # 加载参数并启动节点
        Node(
            package='mocap_udp',
            executable='mocap_udp_node',
            name='mocap_udp_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],  # 传递配置文件
        ),
    ])
