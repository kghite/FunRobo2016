// Midbrain controller
// Take in LIDAR data on /scan
// Output Int16MultArray on /obst/cmd_vel to turn toward waypoint set on /goal

#include <string>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"

ros::Publisher *pub_arb;
sensor_msgs::LaserScan scan;
std_msgs::Int8MultiArray cmd_array;

// Speed to move forward with
int lin_vel = 0;

// Final velocity slider
std::vector<int> final_vel_command;

// Object weights that get passed to the arbiter
const int WALL = 1;

// input lin from -50 to 50
std::vector<int> set_vel_vector(int object_weight, int lin)
{
  std::vector<int> vel(202, 0);
  vel.at(lin+50) = object_weight;
  return vel;
}

void controlSpeed(const sensor_msgs::LaserScan lidar_scan)
{
  // DEBUG
  ROS_INFO("Received Scan");

  // Assign LIDAR scan to global
  scan = lidar_scan;

  long number_of_ranges = lidar_scan.ranges.size();
  float forward_distance = 100;

  // Remove junk values from scan data (0.0 is out of range or no read)
  for(int i=0; i < sizeof(scan.ranges) / sizeof(scan.ranges[0]); i++)
  {
    if(scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max)
    {
      scan.ranges[i] = 0.0;
    }
  }

  // Calculate output array using some portion of scan
  for (long i = number_of_ranges/3; i<2*number_of_ranges/3;i++)
  {
      if (forward_distance > scan.ranges[i] && scan.ranges[i] != 0)
          forward_distance = scan.ranges[i];
  }

  ROS_INFO("Forward distance: %lf", forward_distance);
  if (forward_distance < .3)
  {
    // Move backward
    ROS_INFO("Backward");
    lin_vel = -5;
  }
  else if (forward_distance < .6)
  {
    // Stop
    ROS_INFO("Stop");
    lin_vel = 0;
  }
  else
  {
    // Move forward
    ROS_INFO("Forward");
    lin_vel = 5;
  }

  // Define final_vel_command slider based on given lin_vel
  final_vel_command = set_vel_vector(WALL, lin_vel);

  int* final_vel_arr = &final_vel_command[0];
  cmd_array.data.assign(final_vel_arr, final_vel_arr+202);

  pub_arb->publish(cmd_array);
  cmd_array.data.clear();

  // DEBUG
  ROS_INFO("Publishing Output");

}

int main(int argc, char **argv)
{
  // Define the final_vel_command slider based on given lin_vel
  final_vel_command = set_vel_vector(WALL, lin_vel);

  // Make an array copy of the final_vel_command slider vector
  int* final_vel_arr = &final_vel_command[0];
  cmd_array.data.assign(final_vel_arr, final_vel_arr+202);

  ros::init(argc, argv, "midbrain");

  ros::NodeHandle n;

  ros::Subscriber sub_imu = n.subscribe("/scan", 1000, controlSpeed);

  pub_arb = new ros::Publisher(n.advertise<std_msgs::Int8MultiArray>("obst/cmd_vel", 1000));

  ros::spin();

  return 0;
}
