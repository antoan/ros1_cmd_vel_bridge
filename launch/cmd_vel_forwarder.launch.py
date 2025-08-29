import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Use a hardcoded, absolute path for reliability
    venv_python_executable = '/home/tony/dev/robotics/shared_ros2/src/ros1_cmd_vel_bridge/.venv/bin/python'
    
    # The entry point script installed by colcon
    entry_point_script = '/home/tony/dev/robotics/shared_ros2/install/ros1_cmd_vel_bridge/lib/ros1_cmd_vel_bridge/cmd_vel_forwarder'

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros1_host',
            default_value='localhost',
            description='ROS 1 host IP or hostname'
        ),
        DeclareLaunchArgument(
            'ros1_port',
            default_value='9090',
            description='ROS 1 rosbridge port'
        ),
        # Use a standard Node action, but force it to use the venv's python interpreter
        Node(
            package='ros1_cmd_vel_bridge',
            executable=entry_point_script,
            name='cmd_vel_forwarder',
            output='screen',
            parameters=[{
                'ros1_host': LaunchConfiguration('ros1_host'),
                'ros1_port': LaunchConfiguration('ros1_port')
            }],
            # This is the key: it forces the node to be executed with the python from the venv
            prefix=[f'exec {venv_python_executable}'],
            shell=True,
        )
    ])