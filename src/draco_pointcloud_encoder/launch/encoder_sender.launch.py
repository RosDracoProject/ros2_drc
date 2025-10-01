#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
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
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/lidar_compressed',
        description='Output compressed data topic'
    )
    
    # Get launch configuration
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    
    # Draco Encoder Node
    encoder_node = Node(
        package='draco_pointcloud_encoder',
        executable='encoder_node',
        name='draco_encoder_node',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'output_topic': output_topic,
        }],
        remappings=[
            # 필요시 토픽 리매핑
        ]
    )
    
    # Log information
    log_info = LogInfo(
        msg=[
            'Starting Draco PointCloud Encoder (Sender PC)\n',
            'Input Topic: ', input_topic, '\n',
            'Output Topic: ', output_topic, '\n',
            'This node compresses PointCloud2 data and publishes compressed ByteMultiArray\n',
            'Ready for TCP/IP transmission to receiver PC'
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
        output_topic_arg,
        log_info,
        encoder_node,
    ])
