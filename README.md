# ROS 2 Docker Environment

## Quick Start
```bash
# 1. Build the Docker image
./build.sh

# 2. Start RViz2
./run_rviz.sh    # Try X11 first
./run_vnc.sh     # Use VNC if X11 doesn't work

# 3. Start RealSense camera
./run_realsense.sh
```

## Available Commands
- `./build.sh` - Build Docker image
- `./dev.sh` - Start development container
- `./connect.sh` - Connect to dev container
- `./run_rviz.sh` - Start RViz2 (X11)
- `./run_vnc.sh` - Start RViz2 (VNC at localhost:5900)
- `./run_realsense.sh` - Start RealSense camera

## Workspace Structure
- `src/` → Your ROS packages
- `launch/` → Launch files  
- `config/` → Configuration files