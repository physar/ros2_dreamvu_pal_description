#!/usr/bin/env python3

# inspired on zed_wrapper package zed_camera.launch.py script,
# published by StereoLabs 2020 under MIT License

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory('dreamvu_pal_camera_description')

    publish_urdf = LaunchConfiguration('publish_urdf')

    xacro_path = LaunchConfiguration('xacro_path')
    rviz2_path = LaunchConfiguration('rviz2_path')

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    base_frame = LaunchConfiguration('base_frame')
    cam_pos_x = LaunchConfiguration('cam_pos_x')
    cam_pos_y = LaunchConfiguration('cam_pos_y')
    cam_pos_z = LaunchConfiguration('cam_pos_z')
    cam_roll = LaunchConfiguration('cam_roll')
    cam_pitch = LaunchConfiguration('cam_pitch')
    cam_yaw = LaunchConfiguration('cam_yaw')

# Path to configuration files 

    default_camera_model = 'pal_usb'
    default_camera_name  = '/dreamvu/pal/'

    default_xacro_path = os.path.join(
        package_dir,
        'urdf',
        default_camera_model + '.urdf.xacro'
    )

    default_rviz2_path = os.path.join(
        package_dir,
        'rviz2',
        default_camera_model + '_with_sensor_subscriptions.rviz'
    )
 
# Declare the launch arguments

    declare_publish_urdf_cmd = DeclareLaunchArgument(
        'publish_urdf',
        default_value='true',
        description='Enable URDF processing and starts Robot State Published to propagate static TF.')

    declare_xacro_path_cmd = DeclareLaunchArgument(
        'xacro_path',
        default_value=default_xacro_path,
        description='Path to the camera URDF file as a xacro file.')

    declare_rviz2_path_cmd = DeclareLaunchArgument(
        'rviz2_path',
        default_value=default_rviz2_path,
        description='Path to the configuration of rviz2 with pal camera model.')

    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value=default_camera_name,
        description='The name of the camera. Handy for a robot with multiple cameras. It can be different from the camera model and it will be used as node `namespace`.')

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=default_camera_model,
        description='The model of the camera. Currently only the `pal`-model implemented based on the `pal_usb`, but the `pal_mini` and `pal_alia` have quite different dimensions and max_depth. Valid models: `pal`, `pal_usb`, `pal_mini`, `pal_alia`.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Name of the base link.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value='0.0',
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value='0.0',
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value='0.06',
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value='0.0',
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value='0.0',
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value='0.0',
        description='Yaw orientation of the camera with respect to the base frame.')

     # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=camera_name,
        executable='robot_state_publisher',
        name='pal_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_name, ' ',
                    'camera_model:=', camera_model, ' ',
                    'base_frame:=', base_frame, ' ',
                    'cam_pos_x:=', cam_pos_x, ' ',
                    'cam_pos_y:=', cam_pos_y, ' ',
                    'cam_pos_z:=', cam_pos_z, ' ',
                    'cam_roll:=', cam_roll, ' ',
                    'cam_pitch:=', cam_pitch, ' ',
                    'cam_yaw:=', cam_yaw
                ])
        }]
    )

    # Rviz2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name,
        executable='rviz2',
        name='dreamvu_pal_rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_path]],
    )


# Define LaunchDescription variable and return it

    ld = LaunchDescription()

    ld.add_action(declare_camera_name_cmd)
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_publish_urdf_cmd)
    ld.add_action(declare_xacro_path_cmd)
    ld.add_action(declare_rviz2_path_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(rsp_node)
    ld.add_action(rviz2_node)

    return ld

