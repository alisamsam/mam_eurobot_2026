#!/bin/bash

# Script to launch RViz with proper X11 forwarding and configuration

source /opt/ros/humble/setup.bash
if [ -f /workspaces/mam_eurobot_2026/install/setup.bash ]; then
  source /workspaces/mam_eurobot_2026/install/setup.bash
fi

# Export X11 forwarding variables for WSL2/Docker
export DISPLAY=${DISPLAY:-:1}
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export GDK_BACKEND=x11
export LIBGL_ALWAYS_SOFTWARE=1

echo "DISPLAY=$DISPLAY"
echo "Starting RViz with X11 backend..."

# Start RViz
rviz2 "$@"
