import argparse
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
##########################################
# 런치 파일 주의 사항 정리
# node code path = /opt/ros/foxy/lib/usb_cam
# namespace는 ros1에서 group이랑 같은 거여서 같은 그룹 노드끼리는 맞춰줘야함.
#############################################


def generate_launch_description():
    ld = LaunchDescription()

############################ camera 1 ##############
    ld.add_action(Node(
        package='bev_package', executable='bev_node', output='screen',
        name='bev_node',
        namespace='integration'
        ))

    return ld