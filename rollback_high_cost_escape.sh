#!/bin/bash
set -e
BACKUP_DIR=/home/oaritmi/Documents/ok/backup_before_high_cost_escape
WS=/home/oaritmi/Documents/ok/ros2_ws

echo "Restoring files from $BACKUP_DIR..."
cp -a $BACKUP_DIR/controller_src/*.cpp $WS/src/plan_c_kinematic_controller/src/
cp -a $BACKUP_DIR/controller_src/*.hpp $WS/src/plan_c_kinematic_controller/include/plan_c_kinematic_controller/
cp -a $BACKUP_DIR/config/* $WS/src/s2_wifi_bridge/config/
cp -a $BACKUP_DIR/launch/* $WS/src/s2_wifi_bridge/launch/

source /opt/ros/jazzy/setup.bash
cd $WS
colcon build --symlink-install --packages-select plan_c_kinematic_controller s2_wifi_bridge

echo "ROLLBACK COMPLETED SUCCESSFULLY!"
