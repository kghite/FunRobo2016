// Midbrain controller
// Edited: 11/9/16
// Take in LIDAR data on /scan
// Output Int16MultArray on /obst/cmd_vel to turn toward waypoint set on /goal

#include <string>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

sensor_msgs::LaserScan scan;
std_msgs::Int16MultiArray cmd_array;

void controlSpeed(const sensor_msgs::LaserScan lidar_scan)
{
  // DEBUG
  ROS_INFO("Received Scan");

  // Assign LIDAR scan to global
  scan = lidar_scan;

  // Remove junk values from scan data (0.0 is out of range or no read)
  for(int i=0; i < sizeof(scan.ranges) / sizeof(scan.ranges[0]); i++)
  {
    if(scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max)
    {
      scan.ranges[i] = 0.0;
    }
  }

  // Calculate output array using some portion of scan
  double forward_distance = scan.ranges[256];
  ROS_INFO("Forward distance: %f", forward_distance);
  if (forward_distance < .5)
  {
    cmd_array.data.at(0) = 0;
  }
  else
  {
    cmd_array.data.at(0) = forward_distance;
  }
  ROS_INFO("%d", cmd_array.data.at(0));

  // DEBUG
  ROS_INFO("Publishing Output");

}

int main(int argc, char **argv)
{
  cmd_array.data.push_back(0);

  ros::init(argc, argv, "midbrain");

  ros::NodeHandle n;

  ros::Subscriber sub_imu = n.subscribe("/scan", 1000, controlSpeed);

  ros::Publisher pub_arb = n.advertise<std_msgs::Int16MultiArray>("wpt/cmd_vel", 1000);

  pub_arb.publish(cmd_array);

  ros::spin();

  return 0;
}
