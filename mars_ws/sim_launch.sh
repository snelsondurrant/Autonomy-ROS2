#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches the full simulation stack

sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy

source ~/mars_ws/install/setup.bash
ros2 launch nav2_autonomy autonomy_task.launch.py use_rviz:=True use_mapviz:=True sim_mode:=True
