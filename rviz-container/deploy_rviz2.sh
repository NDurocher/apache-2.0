#!/bin/bash

# ROS 2 Jazzy Docker Build and Deploy Script
# Usage: ./deploy_ros2.sh [build|run|stop|clean|shell|rviz|help]

set -e

# Configuration
IMAGE_NAME="ros2-jazzy-rviz"
CONTAINER_NAME="ros2-jazzy-dev"
WORKSPACE_DIR="$(pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker is running
check_docker() {
    if ! docker info > /dev/null 2>&1; then
        log_error "Docker is not running. Please start Docker first."
        exit 1
    fi
}

# Setup X11 forwarding for GUI applications
setup_x11() {
    log_info "Setting up X11 forwarding for GUI applications..."
    
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Linux
        xhost +local:docker > /dev/null 2>&1 || log_warning "Could not set xhost permissions"
        export DISPLAY=${DISPLAY:-:0}
        
        # Create .docker.xauth if it doesn't exist
        if [ ! -f /tmp/.docker.xauth ]; then
            touch /tmp/.docker.xauth
            xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge - 2>/dev/null || true
        fi
        
        log_success "X11 forwarding configured for Linux"
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        log_warning "macOS detected. You may need to install XQuartz and set DISPLAY manually"
        export DISPLAY=${DISPLAY:-host.docker.internal:0}
    else
        log_warning "Unsupported OS for automatic X11 setup. GUI applications may not work."
    fi
}

# Build the Docker image
build_image() {
    log_info "Building ROS 2 Jazzy Docker image..."
    
    if [ ! -f "Dockerfile" ]; then
        log_error "Dockerfile not found in current directory"
        exit 1
    fi
    
    docker build -t $IMAGE_NAME .
    log_success "Docker image '$IMAGE_NAME' built successfully"
}

# Run the container
run_container() {
    log_info "Starting ROS 2 Jazzy container..."
    
    # Stop existing container if running
    if docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
        log_info "Stopping existing container..."
        docker stop $CONTAINER_NAME > /dev/null
    fi
    
    # Remove existing container if it exists
    if docker ps -aq -f name=$CONTAINER_NAME | grep -q .; then
        log_info "Removing existing container..."
        docker rm $CONTAINER_NAME > /dev/null
    fi
    
    setup_x11
    
    # Create src directory if it doesn't exist
    mkdir -p src packages
    
    # Run the container
    docker run -d \
        --name $CONTAINER_NAME \
        --network host \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=/tmp/.docker.xauth \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
        -v $WORKSPACE_DIR/src:/ros2_ws/src:rw \
        -v $WORKSPACE_DIR/packages:/ros2_ws/packages:rw \
        --restart unless-stopped \
        $IMAGE_NAME \
        tail -f /dev/null
    
    log_success "Container '$CONTAINER_NAME' is running"
    log_info "Use './deploy_ros2.sh shell' to access the container"
}

# Stop the container
stop_container() {
    log_info "Stopping ROS 2 container..."
    
    if docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
        docker stop $CONTAINER_NAME > /dev/null
        log_success "Container stopped"
    else
        log_warning "Container is not running"
    fi
}

# Clean up (stop and remove container and image)
clean_all() {
    log_info "Cleaning up Docker resources..."
    
    # Stop and remove container
    if docker ps -aq -f name=$CONTAINER_NAME | grep -q .; then
        docker stop $CONTAINER_NAME > /dev/null 2>&1 || true
        docker rm $CONTAINER_NAME > /dev/null 2>&1 || true
        log_success "Container removed"
    fi
    
    # Remove image
    if docker images -q $IMAGE_NAME | grep -q .; then
        docker rmi $IMAGE_NAME > /dev/null
        log_success "Image removed"
    fi
    
    log_success "Cleanup completed"
}

# Open shell in container
open_shell() {
    if ! docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
        log_error "Container '$CONTAINER_NAME' is not running. Use './deploy_ros2.sh run' first."
        exit 1
    fi
    
    log_info "Opening shell in ROS 2 container..."
    docker exec -it $CONTAINER_NAME bash
}

# Launch RViz in container
launch_rviz() {
    if ! docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
        log_error "Container '$CONTAINER_NAME' is not running. Use './deploy_ros2.sh run' first."
        exit 1
    fi
    
    log_info "Launching RViz2..."
    docker exec -it $CONTAINER_NAME bash -c "
        source /opt/ros/jazzy/setup.bash && 
        if [ -f /ros2_ws/install/setup.bash ]; then 
            source /ros2_ws/install/setup.bash
        fi &&
        rviz2
    "
}

# Run a ROS 2 node in container
run_node() {
    if ! docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
        log_error "Container '$CONTAINER_NAME' is not running. Use './deploy_ros2.sh run' first."
        exit 1
    fi
    
    if [ -z "$2" ] || [ -z "$3" ]; then
        log_error "Usage: $0 node <package_name> <node_name>"
        exit 1
    fi
    
    local package_name=$2
    local node_name=$3
    
    log_info "Running ROS 2 node: $package_name/$node_name"
    docker exec -it $CONTAINER_NAME bash -c "
        source /opt/ros/jazzy/setup.bash && 
        if [ -f /ros2_ws/install/setup.bash ]; then 
            source /ros2_ws/install/setup.bash
        fi &&
        ros2 run $package_name $node_name
    "
}

# Build ROS 2 packages in container
build_packages() {
    if ! docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
        log_error "Container '$CONTAINER_NAME' is not running. Use './deploy_ros2.sh run' first."
        exit 1
    fi
    
    log_info "Building ROS 2 packages in container..."
    
    # Check if there are any packages to build
    if [ ! -d "src" ] || [ -z "$(ls -A src 2>/dev/null)" ]; then
        log_warning "No packages found in src/ directory"
        return 0
    fi
    
    # Build packages
    docker exec -it $CONTAINER_NAME bash -c "
        cd /ros2_ws && 
        source /opt/ros/jazzy/setup.bash && 
        echo 'Building packages...' &&
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release &&
        echo 'Build completed successfully!'
    "
    
    log_success "Packages built successfully"
    log_info "Packages are now available in the container workspace"
}

# Force rebuild packages (clean build)
rebuild_packages() {
    if ! docker ps -q -f name=$CONTAINER_NAME | grep -q .; then
        log_error "Container '$CONTAINER_NAME' is not running. Use './deploy_ros2.sh run' first."
        exit 1
    fi
    
    log_info "Cleaning and rebuilding ROS 2 packages..."
    
    docker exec -it $CONTAINER_NAME bash -c "
        cd /ros2_ws && 
        source /opt/ros/jazzy/setup.bash && 
        rm -rf build/ install/ log/ &&
        echo 'Cleaned workspace, building packages...' &&
        colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release &&
        echo 'Rebuild completed successfully!'
    "
    
    log_success "Packages rebuilt successfully"
}

# Show help
show_help() {
    echo "ROS 2 Jazzy Docker Deployment Script"
    echo
    echo "Usage: $0 [COMMAND]"
    echo
    echo "Commands:"
    echo "  build     Build the Docker image"
    echo "  run       Run the container (stops existing one first)"
    echo "  stop      Stop the running container"
    echo "  clean     Stop container and remove image"
    echo "  shell     Open bash shell in running container"
    echo "  rviz      Launch RViz2 in running container"
    echo "  colcon    Build ROS 2 packages in container"
    echo "  rebuild   Clean and rebuild ROS 2 packages"
    echo "  node      Run a ROS 2 node (usage: node <package> <node>)"
    echo "  status    Show container status"
    echo "  logs      Show container logs"
    echo "  help      Show this help message"
    echo
    echo "Examples:"
    echo "  $0 build && $0 run       # Build and run"
    echo "  $0 shell                 # Access container shell"
    echo "  $0 colcon                # Build your packages"
    echo "  $0 rviz                  # Launch RViz"
    echo "  $0 node my_pkg my_node   # Run specific node"
}

# Show container status
show_status() {
    log_info "Container status:"
    if docker ps -f name=$CONTAINER_NAME --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | grep -q $CONTAINER_NAME; then
        docker ps -f name=$CONTAINER_NAME --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
    else
        echo "Container is not running"
    fi
}

# Show container logs
show_logs() {
    if docker ps -aq -f name=$CONTAINER_NAME | grep -q .; then
        docker logs $CONTAINER_NAME
    else
        log_error "Container does not exist"
    fi
}

# Main script logic
main() {
    check_docker
    
    case "${1:-help}" in
        build)
            build_image
            ;;
        run)
            build_image
            run_container
            ;;
        stop)
            stop_container
            ;;
        clean)
            clean_all
            ;;
        shell)
            open_shell
            ;;
        rviz)
            launch_rviz
            ;;
        colcon|build-packages)
            build_packages
            ;;
        rebuild)
            rebuild_packages
            ;;
        node)
            run_node "$@"
            ;;
        status)
            show_status
            ;;
        logs)
            show_logs
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            log_error "Unknown command: $1"
            show_help
            exit 1
            ;;
    esac
}

# Run main function with all arguments
main "$@"
