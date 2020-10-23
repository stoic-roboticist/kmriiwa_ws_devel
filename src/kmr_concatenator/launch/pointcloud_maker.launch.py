import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import sys
from rclpy.utilities import remove_ros_args
import argparse


def generate_launch_description(argv=sys.argv[1:]):
    return LaunchDescription([
        launch_ros.actions.Node(
            package="kmr_concatenator",
            executable="laserscan_converter_node.py",
            name="laserscan_converter",
            output="screen")])
