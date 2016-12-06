// Forebrain controller
// Edited: 11/9/16
// Take in IMU magnetometer data on /imu/mag and GPS in /fix
// Output Int16MultArray on /wpt/cmd_vel to turn toward waypoint set on /goal

#include <string>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"
#include "sensor_msgs/LaserScan.h"

//sensor_msgs::NavSatFix gps_pos;
//geometry_msgs::Vector3Stamped imu_mag;
std_msgs::Int8MultiArray cmd_array;
sensor_msgs::LaserScan filtered_scan;
sensor_msgs::LaserScan scan;

ros::Publisher pub_filtered_scan;
ros::Publisher pub_arb;

int right[22] =    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0};

int stop[22] =     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0};

int left[22] =     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0};

/*void turnToGoal(const sensor_msgs::NavSatFix goal_arr)
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
} */



void getLIDAR(const sensor_msgs::LaserScan lidar_scan)
{
  //ROS_INFO("Received Scan");

  scan.ranges = lidar_scan.ranges;
  filtered_scan.ranges = lidar_scan.ranges;
  filtered_scan.header.frame_id = lidar_scan.header.frame_id;
  filtered_scan.angle_min = lidar_scan.angle_min;
  filtered_scan.angle_max = lidar_scan.angle_max;
  filtered_scan.angle_increment = lidar_scan.angle_increment;
  filtered_scan.range_max = lidar_scan.range_max;
  filtered_scan.range_min = lidar_scan.range_min;
  std::vector<int> indices;

  long number_of_ranges = lidar_scan.ranges.size();

  float average_range = 0;

  float distance_threshold = 0.0;

  // Remove junk values from scan data (0.0 is out of range or no read)
  for(int i=0; i < number_of_ranges; i++)
  {
    if(filtered_scan.ranges[i] < filtered_scan.range_min || filtered_scan.ranges[i] > filtered_scan.range_max)
        filtered_scan.ranges[i] = 0.0;
  }

    for(int i = 0; i < number_of_ranges/6; i++)
    {
        filtered_scan.ranges[i] = 0;
    }

    for(float i = number_of_ranges/6; i < number_of_ranges / 4; i++)
    {
        if(!isnan(filtered_scan.ranges[i]))
            average_range += filtered_scan.ranges[i];
    }
    
    for(float i = number_of_ranges/4; i < number_of_ranges; i++)
    {
        filtered_scan.ranges[i] = 0;
    }

    average_range /= number_of_ranges/6;

    ROS_INFO("%f",average_range);

    if(average_range > 0.65)
        cmd_array.data.assign(left, left+22);
    else if(average_range < 0.5)
        cmd_array.data.assign(right, right+22);
    else
        cmd_array.data.assign(stop, stop+22);

  pub_filtered_scan.publish(filtered_scan);

  pub_arb.publish(cmd_array);
  cmd_array.data.clear();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forebrain");
    ros::NodeHandle n;

  /* ros::Subscriber sub_goal = n.subscribe("goal", 1000, turnToGoal);

  ros::Subscriber sub_gps = n.subscribe("fix", 1000, getGPS);

  ros::Subscriber sub_imu = n.subscribe("imu/mag", 1000, getIMU); */

  ros::Subscriber sub_lidar = n.subscribe("scan",1000,getLIDAR);

  pub_arb = n.advertise<std_msgs::Int8MultiArray>("wpt/cmd_vel", 1000);

  pub_filtered_scan = n.advertise<sensor_msgs::LaserScan>("filtered_scan",1000);

  pub_arb.publish(cmd_array);

  ros::spin();

  return 0;
}
