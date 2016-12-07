#!/bin/bash

echo "Beginning Race Mode"

sudo chmod a+rw /dev/ttyACM0
sudo chmod a+rw /dev/ttyACM1

roslaunch mystery_machine race1-3.launch