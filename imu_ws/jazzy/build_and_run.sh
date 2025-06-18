#!/bin/bash

# Build and run script for ROS2 IMU MPU6050 package
# This script creates the package structure and builds everything in Docker

set -e

echo "🚀 Setting up ROS2 IMU MPU6050 package..."

# Create directory structure
echo "📁 Creating package structure..."
mkdir -p imu_mpu6050/imu_mpu6050
mkdir -p imu_mpu6050/launch
mkdir -p imu_mpu6050/config
mkdir -p imu_mpu6050/resource

# Create __init__.py
echo "# IMU MPU6050 ROS2 Package" > imu_mpu6050/imu_mpu6050/__init__.py

echo "✅ Package structure created!"

# Build Docker image
echo "🐳 Building Docker image..."
docker build -t ros2-imu-mpu6050 .

echo "✅ Docker image built successfully!"

# Show usage instructions
echo ""
echo "🎉 Build complete! You can now run the IMU publisher with:"
echo ""
echo "Option 1 - Direct Docker run:"
echo "  docker run --device=/dev/i2c-1 --network=host --rm ros2-imu-mpu6050"
echo ""
echo "Option 2 - Using docker-compose:"
echo "  docker-compose up"
echo ""
echo "Option 3 - Interactive shell for debugging:"
echo "  docker run --device=/dev/i2c-1 --network=host --rm -it ros2-imu-mpu6050 bash"
echo ""
echo "💡 Tips:"
echo "  - Make sure your MPU6050 is connected to I2C bus 1 (/dev/i2c-1)"
echo "  - If using I2C bus 0, change --device=/dev/i2c-0"
echo "  - Check I2C connection with: sudo i2cdetect -y 1"
echo "  - Monitor topics with: docker exec -it <container_name> ros2 topic list"
echo ""
