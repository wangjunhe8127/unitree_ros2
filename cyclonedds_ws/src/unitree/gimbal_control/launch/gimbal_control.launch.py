from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="gimbal_control_node",
                executable="gimbal_control_node",
                name="gimbal_control_node",
                parameters=[
                    {"rtsp_url_master": "rtsp://127.0.0.1:8554/test"},
                    {"rtsp_url_slave": "rtsp://127.0.0.1:8554/test2"},
                    {"topic_name_master": "vis_image"},
                    {"topic_name_slave": "ir_image"},
                    {"master_enable": False},
                    {"slave_enable": False},
                    {"udp_ip": "127.0.0.1"},
                    {"port": 10025},
                ],
                output="screen",
            )
        ]
    )
