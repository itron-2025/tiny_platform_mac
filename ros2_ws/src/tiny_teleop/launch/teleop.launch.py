"""Bring up gamepad teleoperation for tiny_platform_mac.

    Pro Controller --bluetooth--> joy_node --/tiny/joy--> joy_teleop
        --/tiny/cmd_vel--> (micro-ROS agent, started separately) --> ESP32

The micro-ROS agent is deliberately NOT launched here. It is a baked image
script (`/root/scripts/start-agent.sh`, or `make agent` from the host) because
it has to release the CH340 RTS line to let the board boot, and because it must
already be listening BEFORE the ESP32 comes up -- ordering a launch file cannot
guarantee.

    ros2 launch tiny_teleop teleop.launch.py

Both nodes go into the /tiny namespace. That is what stops the Tomcat chassis --
same host, same DDS domain, bare /cmd_vel -- from driving this robot.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'tiny'


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('tiny_teleop'), 'config', 'teleop_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params', default_value=params,
        description='YAML with joy_node and joy_teleop parameters.')
    params_file = LaunchConfiguration('params')

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        namespace=NAMESPACE, parameters=[params_file], output='screen',
    )

    teleop_node = Node(
        package='tiny_teleop', executable='joy_teleop', name='joy_teleop',
        namespace=NAMESPACE, parameters=[params_file], output='screen',
    )

    return LaunchDescription([params_arg, joy_node, teleop_node])
