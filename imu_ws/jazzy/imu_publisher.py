#!/usr/bin/env python3
import os, sys
sys.path.append(os.path.dirname(__file__))
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import mpu6050


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Declare parameters
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calibration_samples', 100)
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.calibration_samples = self.get_parameter('calibration_samples').get_parameter_value().integer_value
        
        # Initialize the MPU6050 sensor
        try:
            self.mpu6050 = mpu6050.mpu6050(i2c_address)
            self.mpu6050.set_filter_range()
            self.get_logger().info(f'MPU6050 initialized at address 0x{i2c_address:02x}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU6050: {e}')
            return
        
        # Create QoS profile for IMU data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize ROS2 publisher
        self.pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile)
        
        # Store offsets as member variables
        self.linear_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.angular_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
        
        # Perform calibration
        self.get_logger().info('Starting IMU calibration...')
        self.calibrate_sensor()
        self.get_logger().info('IMU calibration complete')
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_data)
        
        self.get_logger().info(f'IMU Publisher started at {self.publish_rate} Hz')

    def read_sensor_data(self):
        """Read raw sensor data from MPU6050"""
        try:
            # Read accelerometer data (m/s²)
            accelerometer_data = self.mpu6050.get_accel_data()
            
            # Read gyroscope data (rad/s)
            gyroscope_data = self.mpu6050.get_gyro_data()
            
            # Read temperature (not used directly here)
            temperature = self.mpu6050.get_temp()
            
            return accelerometer_data, gyroscope_data, temperature
            
        except Exception as e:
            self.get_logger().error(f'Error reading sensor data: {e}')
            return None, None, None

    def calibrate_sensor(self):
        """Calibrate the IMU by collecting offset values"""
        linear_calibration_buffer = []
        angular_calibration_buffer = []
        
        self.get_logger().info(f'Collecting {self.calibration_samples} samples for calibration...')
        
        # Collect samples for calibration
        for i in range(self.calibration_samples):
            accel_data, gyro_data, _ = self.read_sensor_data()
            
            if accel_data is None or gyro_data is None:
                self.get_logger().warn(f'Failed to read sensor data during calibration (sample {i+1})')
                continue
                
            linear_calibration_buffer.append({
                "x": accel_data['x'],
                "y": accel_data['y'],
                "z": accel_data['z']
            })
            
            angular_calibration_buffer.append({
                "x": gyro_data['x'],
                "y": gyro_data['y'],
                "z": gyro_data['z']
            })
            
            # Small delay between samples
            rclpy.spin_once(self, timeout_sec=0.01)
        
        if len(linear_calibration_buffer) < self.calibration_samples * 0.8:
            self.get_logger().error('Insufficient calibration samples collected')
            return
        
        # Compute linear offsets (average of calibration samples)
        self.linear_offset["x"] = sum(sample["x"] for sample in linear_calibration_buffer) / len(linear_calibration_buffer)
        self.linear_offset["y"] = sum(sample["y"] for sample in linear_calibration_buffer) / len(linear_calibration_buffer)
        self.linear_offset["z"] = sum(sample["z"] for sample in linear_calibration_buffer) / len(linear_calibration_buffer)
        
        # Compute angular offsets (average of calibration samples)
        self.angular_offset["x"] = sum(sample["x"] for sample in angular_calibration_buffer) / len(angular_calibration_buffer)
        self.angular_offset["y"] = sum(sample["y"] for sample in angular_calibration_buffer) / len(angular_calibration_buffer)
        self.angular_offset["z"] = sum(sample["z"] for sample in angular_calibration_buffer) / len(angular_calibration_buffer)
        
        self.get_logger().info(f'Linear offsets: x={self.linear_offset["x"]:.4f}, y={self.linear_offset["y"]:.4f}, z={self.linear_offset["z"]:.4f}')
        self.get_logger().info(f'Angular offsets: x={self.angular_offset["x"]:.4f}, y={self.angular_offset["y"]:.4f}, z={self.angular_offset["z"]:.4f}')

    def publish_data(self):
        """Timer callback to read and publish IMU data"""
        # Read sensor data
        accel_data, gyro_data, temperature = self.read_sensor_data()
        
        if accel_data is None or gyro_data is None:
            self.get_logger().warn('Failed to read sensor data, skipping publication')
            return
        
        # Create IMU message
        imu_msg = Imu()
        
        # Set header
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        
        # Apply linear offset correction and set linear acceleration
        imu_msg.linear_acceleration.x = accel_data['x'] - self.linear_offset["x"]
        imu_msg.linear_acceleration.y = accel_data['y'] - self.linear_offset["y"]
        imu_msg.linear_acceleration.z = accel_data['z'] - self.linear_offset["z"]
        
        # Apply angular offset correction and set angular velocity
        imu_msg.angular_velocity.x = gyro_data['x'] - self.angular_offset["x"]
        imu_msg.angular_velocity.y = gyro_data['y'] - self.angular_offset["y"]
        imu_msg.angular_velocity.z = gyro_data['z'] - self.angular_offset["z"]
        
        # Set covariance matrices (unknown/uncalibrated)
        imu_msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        imu_msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
        imu_msg.orientation_covariance = [-1.0] + [0.0] * 8
        
        # Publish the message
        self.pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        imu_publisher = IMUPublisher()
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'imu_publisher' in locals():
            imu_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
