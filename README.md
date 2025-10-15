## ROS 2 Docker Setup for macOS

### Development Workflow
```bash
# Build the image
docker build -t my-ros-rviz .

# Start development container (persistent)
./dev.sh

# Connect to container for development
./connect.sh

# Build ROS workspace
./build_workspace.sh
```

### Quick Start (Original)
```bash
# Start container with RViz2
./run_ros_container.sh

# Connect to running container (new terminal)
./connect.sh
```

### VNC Access
- Install VNC viewer: `brew install --cask tigervnc-viewer`
- Connect to: `localhost:5900`
- No password required

### Custom Commands
```bash
# Start container with bash instead of RViz2
./run_ros_container.sh bash

# Start with custom ROS command
./run_ros_container.sh ros2 topic list
```

### Workspace Structure
- Host: `~/Documents/docker_ros/src` → Container: `/workspace/src`
- Host: `~/Documents/docker_ros/launch` → Container: `/workspace/launch`
- Host: `~/Documents/docker_ros/config` → Container: `/workspace/config`

### Development Setup
1. Use VS Code to edit files in `src/`, `launch/`, and `config/`
2. Changes are automatically synced to the container
3. Build and test inside the container using `./connect.sh`
4. Use VNC for GUI applications like RViz2