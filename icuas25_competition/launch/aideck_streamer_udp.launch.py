import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    aideck_config_path = os.path.join(get_package_share_directory('icuas25_competition'),
                                                                  'config',
                                                                  'aideck_streamer.yaml')
    launch_description = []
    launch_description.append(Node(
                                package="icuas25_competition",
                                executable="aideck_streamer_udp.py",
                                name="aideck_streamer_udp",
                                parameters=[{"aideck_config_path": aideck_config_path}]))
    return LaunchDescription(launch_description)