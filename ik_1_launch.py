import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = "in426_motion",
            executable = "ik_1.py",
            parameters = [os.path.join(get_package_share_directory("in426_simu"), "config", "ik_params.yaml")]
        )
    ])