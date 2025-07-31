#!/bin/bash
# Enhanced Robotic Arm Container Launch Script
# This script manages the robotic_arm_container with full project setup

# Configuration
CONTAINER_NAME="robotic_arm_container"
IMAGE_NAME="robotic_arm:latest"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "🤖 ========================================== 🤖"
echo "   Robotic Arm Simulation Environment Setup"
echo "🤖 ========================================== 🤖"

# Function to setup X11 forwarding
setup_x11() {
    echo "--- 🔑 Setting up X11 forwarding for GUI applications 🔑 ---"
    export XAUTH=/tmp/.docker.xauth
    touch $XAUTH
    xauth nxlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nxmerge -
    xhost +local:docker
}

# Function to cleanup X11
cleanup_x11() {
    echo "--- 🧹 Cleaning up X11 permissions 🧹 ---"
    xhost -local:docker
}

# Function to check if image exists
check_image() {
    if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
        echo "--- 🏗️ Docker image not found. Building image... 🏗️ ---"
        docker build -t $IMAGE_NAME $PROJECT_DIR
        if [ $? -ne 0 ]; then
            echo "❌ Failed to build Docker image!"
            exit 1
        fi
        echo "✅ Docker image built successfully!"
    else
        echo "✅ Docker image found: $IMAGE_NAME"
    fi
}

# Function to run container
run_container() {
    echo "--- 🚀 Creating and launching robotic arm container... 🚀 ---"
    
    docker run -it \
        --gpus all \
        --name $CONTAINER_NAME \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="XAUTHORITY=$XAUTH" \
        --env="ROS_MASTER_URI=http://localhost:11311" \
        --env="ROS_HOSTNAME=localhost" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$XAUTH:$XAUTH" \
        --volume="$PROJECT_DIR:/root/host_project:ro" \
        --net=host \
        --privileged \
        --workdir="/root/ros_ws" \
        $IMAGE_NAME
}

# Function to restart existing container
restart_container() {
    echo "--- ♻️ Restarting existing container... ♻️ ---"
    docker stop $CONTAINER_NAME > /dev/null 2>&1 || true
    docker start -ai $CONTAINER_NAME
}

# Function to remove container
remove_container() {
    echo "--- 🗑️ Removing existing container... 🗑️ ---"
    docker stop $CONTAINER_NAME > /dev/null 2>&1 || true
    docker rm $CONTAINER_NAME > /dev/null 2>&1 || true
}

# Main execution logic
main() {
    # Setup X11 forwarding
    setup_x11
    
    # Trap to ensure cleanup on exit
    trap cleanup_x11 EXIT
    
    # Parse command line arguments
    case "${1:-}" in
        --rebuild)
            echo "🔄 Rebuilding container from scratch..."
            remove_container
            docker rmi $IMAGE_NAME > /dev/null 2>&1 || true
            check_image
            run_container
            ;;
        --fresh)
            echo "🆕 Creating fresh container (removing existing)..."
            remove_container
            check_image
            run_container
            ;;
        --help|-h)
            echo "Usage: $0 [option]"
            echo "Options:"
            echo "  (no option)  - Start existing container or create new one"
            echo "  --rebuild    - Rebuild Docker image and create fresh container"
            echo "  --fresh      - Remove existing container and create new one"
            echo "  --help, -h   - Show this help message"
            exit 0
            ;;
        *)
            # Default behavior
            if [ "$(docker ps -a -q -f name=^/${CONTAINER_NAME}$)" ]; then
                # Container exists
                if [ "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]; then
                    # Container is running
                    echo "--- 📎 Attaching to running container... 📎 ---"
                    docker exec -it $CONTAINER_NAME /bin/bash
                else
                    # Container exists but is stopped
                    restart_container
                fi
            else
                # Container doesn't exist
                check_image
                run_container
            fi
            ;;
    esac
}

# Run main function
main "$@"