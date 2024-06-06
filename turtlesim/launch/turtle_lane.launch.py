from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lane_boundary_share = get_package_share_directory("lane_boundary")
    turtlesim_share = get_package_share_directory("turtlesim")

    return LaunchDescription([
        Node(
            package="lane_boundary",
            executable="lane_boundary_node",
            parameters=[{"track_file_path": lane_boundary_share + "/res/KITcar_strecke.csv"}],
        ),
        Node(
            package="turtlesim",
            executable="turtlesim_node",
        )
    ])
