// Midbrain controller
// Edited: 11/9/16
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
int backward[22] = {0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int stop[22] =     {0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int forward[22] =  {0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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
      if (forward_distance > scan.ranges[i])
          forward_distance = scan.ranges[i];
  }


  ROS_INFO("Forward distance: %lf", forward_distance);
  if (forward_distance < .3)
  {
    // Move backward
    ROS_INFO("Backward");
    cmd_array.data.assign(backward, backward+22);
  }
  else if (forward_distance < .8)
  {
    // Stop
    ROS_INFO("Stop");
    cmd_array.data.assign(stop, stop+22);
  }
  else
  {
    // Move forward
    ROS_INFO("Forward");
    cmd_array.data.assign(forward, forward+22);
    //ROS_INFO_STREAM(cmd_array);
  }

  // for (int i=0; i<22; i++)
  // {
  //   ROS_INFO("%d, ", cmd_array.data.at(i));
  // }

  pub_arb->publish(cmd_array);
  cmd_array.data.clear();

  // DEBUG
  ROS_INFO("Publishing Output");

}

int main(int argc, char **argv)
{
  cmd_array.data.assign(&stop[0], &stop[0]+22);

  ros::init(argc, argv, "midbrain");

  ros::NodeHandle n;

  ros::Subscriber sub_imu = n.subscribe("/scan", 1000, controlSpeed);

  pub_arb = new ros::Publisher(n.advertise<std_msgs::Int8MultiArray>("obst/cmd_vel", 1000));

  ros::spin();

  return 0;
}
