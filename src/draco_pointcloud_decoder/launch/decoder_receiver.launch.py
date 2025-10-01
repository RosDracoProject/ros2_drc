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
        default_value='/lidar_compressed',
        description='Input compressed data topic from TCP/IP'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/sensing/lidar/points_raw',
        description='Output decompressed point cloud topic for SLAM'
    )
    
    # Get launch configuration
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    
    # Draco Decoder Node
    decoder_node = Node(
        package='draco_pointcloud_decoder',
        executable='decoder_node',
        name='draco_decoder_node',
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
            'Starting Draco PointCloud Decoder (Receiver PC)\n',
            'Input Topic: ', input_topic, ' (from TCP/IP)\n',
            'Output Topic: ', output_topic, ' (for SLAM Toolbox)\n',
            'This node decompresses ByteMultiArray data and publishes PointCloud2\n',
            'Ready to receive compressed data from sender PC'
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
        decoder_node,
    ])
