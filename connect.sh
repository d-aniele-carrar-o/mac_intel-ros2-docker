#!/bin/bash
# Quick script to connect to running ROS container
docker exec -it ros2_dev bash -c "source /opt/ros/humble/setup.bash && bash"