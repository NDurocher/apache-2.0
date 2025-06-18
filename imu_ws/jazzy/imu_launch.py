#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('imu_mpu6050')
    
    # Declare launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='IMU data publishing rate in Hz'
    )
    
    i2c_address_arg = DeclareLaunchArgument(
        'i2c_address',
        default_value='104',  # 0x68 in decimal
        description='I2C address of the MPU6050 sensor'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for the IMU data'
    )
    
    # Path to parameter file
    params_file = os.path.join(pkg_share, 'config', 'imu_params.yaml')
    
    # IMU publisher node
    imu_node = Node(
        package='imu_mpu6050',
        executable='imu_publisher',
        name='imu_publisher',
        parameters=[
            params_file,
            {
                'publish_rate': LaunchConfiguration('publish_rate'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'frame_id': LaunchConfiguration('frame_id'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        publish_rate_arg,
        i2c_address_arg,
        frame_id_arg,
        imu_node,
    ])
