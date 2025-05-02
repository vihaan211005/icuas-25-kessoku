import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_params = os.path.join(
        get_package_share_directory('icuas25_competition'),
        'config',
        'aruco_parameters.yaml'
        )
    
    launch_description = []
    for i in range(0, int(os.environ.get('NUM_ROBOTS', '3'))):
        aruco_node = Node(
            package='ros2_aruco',
            namespace = f'cf_{i+1}',
            executable='aruco_node',
            parameters=[aruco_params]
        )
        launch_description.append(aruco_node)

    return LaunchDescription(launch_description)