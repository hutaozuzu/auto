#!/bin/bash
set -e
BACKUP_DIR=/home/oaritmi/Documents/ok/backup_before_per_ray_lidar_deskew
WS=/home/oaritmi/Documents/ok/ros2_ws

echo "Restoring files from $BACKUP_DIR..."
cp -a $BACKUP_DIR/config/* $WS/src/s2_wifi_bridge/config/
cp -a $BACKUP_DIR/launch/* $WS/src/s2_wifi_bridge/launch/
cp -a $BACKUP_DIR/s2_wifi_bridge/* $WS/src/s2_wifi_bridge/s2_wifi_bridge/

source /opt/ros/jazzy/setup.bash
cd $WS
colcon build --symlink-install --packages-select s2_wifi_bridge plan_c_kinematic_controller

echo "ROLLBACK COMPLETE!"
