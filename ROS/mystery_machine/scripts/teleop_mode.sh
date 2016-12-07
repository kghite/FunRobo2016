#!/bin/bash

echo "Beginning Telop Mode"

sudo chmod r+W /dev/ttyACM2

roslaunch mystery_machine teleop.launch
