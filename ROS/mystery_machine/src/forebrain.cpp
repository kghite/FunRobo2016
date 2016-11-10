// Forebrain controller
// Edited: 11/9/16
// Take in IMU magnetometer data on /imu/mag and GPS in /fix
// Output Int16MultArray on /wpt/cmd_vel to turn toward waypoint set on /goal

#include <string>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

sensor_msgs::NavSatFix gps_pos;
geometry_msgs::Vector3Stamped imu_mag;
std_msgs::Int16MultiArray cmd_array;

void turnToGoal(const sensor_msgs::NavSatFix goal_arr)
{
  // DEBUG
  ROS_INFO("Received Goal");

  // Break down lat and long of goal waypoint
  float goal_lat = goal_arr.latitude;
  float goal_long = goal_arr.longitude;

  // Break down lat and long of gps position
  float gps_lat = gps_pos.latitude;
  float gps_long = gps_pos.longitude;

  // Break down IMU mag data
  float imu_mag_x = imu_mag.vector.x;
  float imu_mag_y = imu_mag.vector.y;
  float imu_mag_z = imu_mag.vector.z;

  // Calculate angle from goal waypoint

  // Translate to array for arbiter (set cmd_array)
  

  // DEBUG
  ROS_INFO("Waypoint lat: %f", goal_lat);
  ROS_INFO("Waypoint long: %f", goal_long);
  ROS_INFO("GPS lat: %f", gps_lat);
  ROS_INFO("GPS long: %f", gps_long);
  ROS_INFO("IMU Mag X: %f", imu_mag_x);
  ROS_INFO("IMU Mag Y: %f", imu_mag_y);
  ROS_INFO("IMU Mag Z: %f", imu_mag_z);
  ROS_INFO("Publishing Output");
}

void getGPS(const sensor_msgs::NavSatFix gps) 
{
  gps_pos = gps;
}

void getIMU(const geometry_msgs::Vector3Stamped imu)
{
  imu_mag = imu;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "forebrain");

  ros::NodeHandle n;

  ros::Subscriber sub_goal = n.subscribe("goal", 1000, turnToGoal);

  ros::Subscriber sub_gps = n.subscribe("fix", 1000, getGPS);

  ros::Subscriber sub_imu = n.subscribe("imu/mag", 1000, getIMU);

  ros::Publisher pub_arb = n.advertise<std_msgs::Int16MultiArray>("wpt/cmd_vel", 1000);

  pub_arb.publish(cmd_array);

  ros::spin();

  return 0;
}