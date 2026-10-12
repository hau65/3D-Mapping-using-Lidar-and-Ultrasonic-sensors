#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source ~/harmonic_ws/install/setup.bash

echo "Killing the foregoers..."
pkill -f '[r]os2 launch mobile_robot'     || true
pkill -f '[g]z sim'                       || true
pkill -f '[p]arameter_bridge'             || true
pkill -f '[r]obot_state_publisher'        || true
pkill -f '[e]kf_node'                     || true
pkill -f '[s]can_to_scan_filter_chain'    || true
pkill -f '[u]ltrasonic_fusion_node'       || true
pkill -f '[s]lam_toolbox'                 || true
pkill -f '[r]viz2'                        || true
sleep 3

#echo "Killing the obstinates..."
#pkill -9 -f '[g]z sim'                    || true
#pkill -9 -f '[u]ltrasonic_fusion_node'    || true
#sleep 2

exec ros2 launch mobile_robot sim_slam_v2.launch.py
#test git