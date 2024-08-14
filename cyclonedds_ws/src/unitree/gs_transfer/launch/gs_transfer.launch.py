import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gs_transfer_node',  # 替换为您的包名
            executable='gs_transfer_node',  # 可执行文件名
            name='gs_transfer_node',
            output='screen',
            parameters=[{
                'udp_ip': '127.0.0.1',  # 这里可以修改为您的目标IP
                'udp_port': 12345,      # 这里可以修改为您的目标端口
            }],
        ),
    ])
