import os
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=get_package_share_directory(
                'orb_node') + '/config/objects.json'
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare('carla_ros_bridge'),
            '/carla_ros_bridge_with_example_ego_vehicle.launch.py']
        ),
         launch_arguments={
            'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file'),
         }.items()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

    