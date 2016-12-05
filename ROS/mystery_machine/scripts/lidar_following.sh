#!/bin/bash

echo "Beginning LIDAR Following Mode"

sudo chmod r+w /dev/ttyACM0
sudo chmod r+W /dev/ttyACM1

roslaunch mystery_machine lidar_following.launch
