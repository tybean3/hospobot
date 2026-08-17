#!/bin/bash
echo "Saving SLAM map..."
ros2 run nav2_map_server map_saver_cli -f /home/hospobot/hospobot_ws/src/hospobot/src/hospobot_bringup/maps/hospital_map
echo "Map saved successfully to src/hospobot/src/hospobot_bringup/maps/hospital_map.yaml"
