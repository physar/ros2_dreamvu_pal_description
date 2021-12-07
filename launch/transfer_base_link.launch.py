import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    map_tf_node = Node(package = "tf2_ros",
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0.00", "0", "0", "0", "map", "base_link"])

    ld.add_action(map_tf_node)

    return ld


