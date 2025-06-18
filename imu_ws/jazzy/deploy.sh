#!/bin/bash

# Deployment script for ROS2 IMU MPU6050
# Handles package creation, building, and running

set -e

PACKAGE_NAME="ros2-imu-mpu6050"
I2C_DEVICE="${I2C_DEVICE:-/dev/i2c-1}"

echo "🚀 ROS2 IMU MPU6050 Deployment Script"
echo "======================================"

# Function to check if I2C device exists
check_i2c() {
    if [ ! -e "$I2C_DEVICE" ]; then
        echo "⚠️  Warning: I2C device $I2C_DEVICE not found!"
        echo "   Make sure I2C is enabled and your device is connected."
        echo "   On Raspberry Pi, run: sudo raspi-config -> Interface Options -> I2C -> Enable"
        echo ""
        read -p "   Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        echo "✅ I2C device $I2C_DEVICE found"
    fi
}

# Function to create package structure
create_structure() {
    echo "📁 Creating package structure..."
    
    # Create main directories
    mkdir -p imu_mpu6050/{imu_mpu6050,launch,config,resource}
    
    # Create __init__.py
    echo "# IMU MPU6050 ROS2 Package" > imu_mpu6050/imu_mpu6050/__init__.py
    
    echo "✅ Package structure created"
}

# Function to build Docker image
build_image() {
    echo "🐳 Building Docker image: $PACKAGE_NAME"
    
    if docker build -t "$PACKAGE_NAME" .; then
        echo "✅ Docker image built successfully"
    else
        echo "❌ Failed to build Docker image"
        exit 1
    fi
}

# Function to run the container
run_container() {
    echo "🏃 Starting IMU publisher container..."
    
    # Stop existing container if running
    if docker ps -q -f name=imu_publisher > /dev/null 2>&1; then
        echo "🛑 Stopping existing container..."
        docker stop imu_publisher > /dev/null 2>&1 || true
        docker rm imu_publisher > /dev/null 2>&1 || true
    fi
    
    # Run new container
    docker run \
        --name imu_publisher \
        --device="$I2C_DEVICE:$I2C_DEVICE" \
        --network=host \
        --restart=unless-stopped \
        -d \
        "$PACKAGE_NAME"
    
    echo "✅ Container started successfully"
    echo "📊 Monitor with: docker logs -f imu_publisher"
}

# Function to show container logs
show_logs() {
    echo "📊 Showing container logs (Ctrl+C to exit)..."
    docker logs -f imu_publisher
}

# Function to stop the container
stop_container() {
    echo "🛑 Stopping IMU publisher..."
    docker stop imu_publisher > /dev/null 2>&1 || true
    docker rm imu_publisher > /dev/null 2>&1 || true
    echo "✅ Container stopped"
}

# Function to show help
show_help() {
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build     - Create package structure and build Docker image"
    echo "  run       - Run the IMU publisher container"
    echo "  logs      - Show container logs"
    echo "  stop      - Stop the container"
    echo "  restart   - Stop and restart the container"
    echo "  shell     - Open interactive shell in container"
    echo "  clean     - Remove Docker image and containers"
    echo ""
    echo "Environment variables:"
    echo "  I2C_DEVICE - I2C device path (default: /dev/i2c-1)"
    echo ""
    echo "Examples:"
    echo "  $0 build"
    echo "  I2C_DEVICE=/dev/i2c-0 $0 run"
    echo "  $0 logs"
}

# Function to open shell
open_shell() {
    echo "🐚 Opening interactive shell..."
    docker run --device="$I2C_DEVICE:$I2C_DEVICE" --network=host --rm -it "$PACKAGE_NAME" bash
}

# Function to clean up
clean_up() {
    echo "🧹 Cleaning up..."
    docker stop imu_publisher > /dev/null 2>&1 || true
    docker rm imu_publisher > /dev/null 2>&1 || true
    docker rmi "$PACKAGE_NAME" > /dev/null 2>&1 || true
    echo "✅ Cleanup complete"
}

# Main script logic
case "${1:-build}" in
    "build")
        check_i2c
        create_structure
        build_image
        echo ""
        echo "🎉 Build complete! Next steps:"
        echo "  Run: $0 run"
        echo "  Logs: $0 logs"
        ;;
    "run")
        check_i2c
        run_container
        echo ""
        echo "💡 Useful commands:"
        echo "  View logs: $0 logs"
        echo "  Stop: $0 stop"
        ;;
    "logs")
        show_logs
        ;;
    "stop")
        stop_container
        ;;
    "restart")
        stop_container
        sleep 2
        run_container
        ;;
    "shell")
        open_shell
        ;;
    "clean")
        clean_up
        ;;
    "help"|"--help"|"-h")
        show_help
        ;;
    *)
        echo "❌ Unknown command: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
