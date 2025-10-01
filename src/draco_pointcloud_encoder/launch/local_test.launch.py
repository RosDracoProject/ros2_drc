#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# ROS_DOMAIN_ID 설정
os.environ['ROS_DOMAIN_ID'] = '10'

def generate_launch_description():
    # Launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/sensing/lidar/top/pointcloud',
        description='Input point cloud topic for compression'
    )
    
    # Get launch configuration
    input_topic = LaunchConfiguration('input_topic')
    
    # Draco Encoder Node (Sender)
    encoder_node = Node(
        package='draco_pointcloud_encoder',
        executable='encoder_node',
        name='draco_encoder_node',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'output_topic': '/lidar_compressed',
        }]
    )
    
    # Draco Decoder Node (Receiver)
    decoder_node = Node(
        package='draco_pointcloud_decoder',
        executable='decoder_node',
        name='draco_decoder_node',
        output='screen',
        parameters=[{
            'input_topic': '/lidar_compressed',
            'output_topic': '/sensing/lidar/points_raw',
        }]
    )
    
    # Log information
    log_info = LogInfo(
        msg=[
            'Starting Local Test Environment\n',
            'Encoder -> Compress -> Decoder -> SLAM Toolbox\n',
            'Input Topic: ', input_topic, '\n',
            'Compressed Topic: /lidar_compressed\n',
            'Output Topic: /sensing/lidar/points_raw\n',
            'This simulates the full pipeline for local testing'
        ]
    )
    
    # ROS_DOMAIN_ID 환경변수 설정
    ros_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='10'
    )
    
    return LaunchDescription([
        ros_domain_id,
        input_topic_arg,
        log_info,
        encoder_node,
        decoder_node,
    ])
