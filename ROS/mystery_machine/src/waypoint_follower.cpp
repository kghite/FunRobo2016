// Waypoint Follower
// Take in IMU magnetometer data on /imu/mag and GPS in /fix
// Read waypoints from text file
// Output theta value to /goal/theta

#include <iostream>
#include <fstream>
#include <string> // Read waypoint file
#include <stdlib.h> // Convert strings to floats
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h" // GPS data
#include "std_msgs/Float64.h" // Theta output
#include "geometry_msgs/Vector3Stamped.h" // IMU compass data

// ROS Input: gps, compass
sensor_msgs::NavSatFix gps_pos;
geometry_msgs::Vector3Stamped imu_mag;

// File Input: waypoints
float waypoints[10];

// Output: theta
std_msgs::Float64 theta;
ros::Publisher pub_theta;


void getGPS(const sensor_msgs::NavSatFix gps) 
{
  ROS_INFO("Received GPS Data");
  gps_pos = gps;
}


void getCompass(const geometry_msgs::Vector3Stamped imu)
{
  ROS_INFO("Received Compass Data");
  imu_mag = imu;

  // Read in the current waypoint goal

  // Break down lat and long of gps position
  float gps_lat = gps_pos.latitude;
  float gps_long = gps_pos.longitude;

  // Break down IMU mag data
  float compass = imu_mag.vector.x;

  // Calculate angle from robot to waypoint goal
  // theta from east = atan(long_g - long_r, lat_g - lat_r) * (180/pi)
  // translate robot compass data into robot angle from east

  // DEBUG
  ROS_INFO("GPS lat: %f", gps_lat);
  ROS_INFO("GPS long: %f", gps_long);
  ROS_INFO("IMU Mag X: %f", compass);
  ROS_INFO("Publishing Output");
}

void readGoals()
{
  std::ifstream file("race_goals.txt");
  int wp_num = 0;
  std::string str;
  while (std::getline(file, str))
  {
    // Convert string into float
    float coord = atof(str.c_str());

    // Determine if lat or long (lat = even, long = odd)
    waypoints[wp_num] = coord;

    wp_num++;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_follower");
  ros::NodeHandle n;

  // Read in the waypoint goals in order and assign to globals
  readGoals();

  // Subscribe to compass and gps data
  ros::Subscriber sub_compass = n.subscribe("fix", 1000, getGPS);
  ros::Subscriber sub_gps = n.subscribe("imu/mag", 1000, getCompass);

  // Publish angle between robot and the goal
  pub_theta = n.advertise<std_msgs::Float64>("/goal/theta", 1000);

  pub_theta.publish(theta);

  ros::spin();

  return 0;
}
