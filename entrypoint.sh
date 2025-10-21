#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/humble/setup.bash

# Execute command
exec "$@"