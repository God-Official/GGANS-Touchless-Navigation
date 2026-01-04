import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    node_touchlessNav = ExecuteProcess(
        cmd=['ros2', 'run', 'touchless_nav', 'touchlessNav.py'],
        output='screen'
    )

    return LaunchDescription([
        node_touchlessNav,
    ])