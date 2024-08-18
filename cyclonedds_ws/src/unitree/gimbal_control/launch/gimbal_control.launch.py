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
                    {"rtsp_url_master": "rtsp://192.168.123.25:8554/video1"},
                    {"rtsp_url_slave": "rtsp://192.168.123.25:8554/video2"},
                    {"topic_name_master": "vis_image"},
                    {"topic_name_slave": "ir_image"},
                    {"master_enable": True},
                    {"slave_enable": False},
                    {"udp_ip": "192.168.123.25"},
                    {"port": 37260},
                ],
                output="screen",
            )
        ]
    )
