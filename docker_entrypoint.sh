#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "============================================"
echo " OrbbecSDK_ROS2 Docker Container"
echo "============================================"
echo ""
echo "ROS2 environment sourced."
echo ""
echo "Available launch files:"
ls /ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/*.launch.py 2>/dev/null | xargs -n1 basename
echo ""
echo "To run the capture script:"
echo "  source /opt/capture_venv/bin/activate"
echo "  cd /ros2_ws/src/OrbbecSDK_ROS2"
echo "  python3 orbbec_ros2_capture.py"
echo ""
echo "To launch camera driver:"
echo "  ros2 launch orbbec_camera gemini_330_series.launch.py"
echo ""

exec $@