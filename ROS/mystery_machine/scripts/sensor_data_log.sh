#!/bin/bash

echo "Starting sensors"
roslaunch mystery_machine data_log.launch

echo "Starting bag file"
gnome-terminal -e rosbag record -a