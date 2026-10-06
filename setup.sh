#!/bin/bash
set -e
cd /workspace
if [ ! -d src/aurora_ndi_ros2_driver ]; then
  git clone -b ros2-package https://github.com/eddrive/aurora_ndi_ros2_driver.git \
    src/aurora_ndi_ros2_driver
fi
cp aurora_overrides/aurora_tracking_config.yaml src/aurora_ndi_ros2_driver/config/driver/
cp aurora_overrides/aurora_tracking.launch.py   src/aurora_ndi_ros2_driver/launch/driver/
source /opt/ros/humble/setup.bash
colcon build
echo "Done. open a new shell or: source install/setup.bash"
