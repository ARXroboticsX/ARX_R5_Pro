#!/bin/bash
source ~/.bashrc

workspace=$(pwd)

sleep 1

gnome-terminal -t "launcher" -x bash -c "source ~/.bashrc;\
cd ${workspace};\
source ./install/setup.bash;ros2 run arx_r5pro_controller R5ProController;exec bash;"




