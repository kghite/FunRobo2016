# FunRobo2016
Code base for the Mystery Machine autonomous race vehicle

---

## Project Description
TODO

---

## Run Instructions for Current Capabilities
### HW 3 Demo

The homework 3 demo can:

* Use the LIDAR to detect obstacles in front of the robot and slow or stop in response
* Use the GPS to find the robot's location in reference to a waypoint and command the robot to turn toward the waypoint
* Use rosserial to pass messages on the /cmd_vel topic from the Midbrain Arbiter (compiles the command velocities from the previous two functions) to the Arduino to move the motors

**To launch this demo run:**

`roslaunch mystery_machine hw3demo.launch`

### LIDAR Visualization

**To initialize and visualize the LIDAR scans on /scan run:**

`sudo chmod a+rw /dev/ttyACM1`

`rosrun hokuyo_node hokuyo_node /dev/ttyACM1`

`rosrun rviz rviz`

Add a LaserScan and set the topic to /scan.  Change fixed frame from map to laser.

### GPS Feed

**To start the GPS feed on /fix run:**

`sudo chmod a+rw /dev/ttyACM0`

`rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyACM0 _baud:=115200`

### Rosserial

If the Arduino is running, it will listen for /cmd_vel.

`rosrun rosserial_python serial_node.py /dev/ttyUSB0`

### Forebrain Controller
We need to write a launch file for this probably

### Midbrain Controller
Same

### Hindbrain Controller
Same