import os, xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory("in426_desc"), "xacro", "Robot_1", "robot_1.xacro")

    return LaunchDescription([
        Node(
            package = "robot_state_publisher",
            executable = "robot_state_publisher",
            parameters = [
                {"robot_description": xacro.process_file(xacro_file).toxml()},
            ]
        ),

        Node(
            package = "joint_state_publisher_gui",
            executable = "joint_state_publisher_gui"
        ),

        Node(
            package = "rviz2",
            executable = "rviz2",
            arguments = ["-d", os.path.join(get_package_share_directory("in426_simu"), "rviz", "config.rviz")]
        )
    ])