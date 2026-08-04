#!/bin/bash
# Launcher for the rec_rep2 Control Panel.
# Sources the ROS2 Humble base and the workspace overlay before
# starting the GUI node.  Intended to be called by the .desktop entry.

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
exec ros2 run rec_rep2 gui
