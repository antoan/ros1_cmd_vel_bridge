from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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
        Node(
            package='ros1_cmd_vel_bridge',
            executable='cmd_vel_forwarder',
            name='cmd_vel_forwarder',
            output='screen',
            parameters=[{
                'ros1_host': LaunchConfiguration('ros1_host'),
                'ros1_port': LaunchConfiguration('ros1_port')
            }]
        )
    ])