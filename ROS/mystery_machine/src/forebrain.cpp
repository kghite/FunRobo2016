// Forebrain controller
// Edited: 11/9/16
// Take in IMU magnetometer data on /imu/mag and GPS in /fix
// Output Int16MultArray on /wpt/cmd_vel to turn toward waypoint set on /goal

#include <string>
#include <vector>
#include <numeric>
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

std::vector<float> average_ranges;
int rolling_length = 5;

int right1[22] =   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0};

int right3[22] =   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int right5[22] =   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int straight[22] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0};

int left[22] =     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0};

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

  float average_range = 0.0;

  float rolling_average_range = 0.0;

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

    int distances_counted = 0;

    for(float i = number_of_ranges/6+42; i < number_of_ranges / 4+42; i++)
    {
        if(!isnan(filtered_scan.ranges[i])) {
            average_range += filtered_scan.ranges[i];
            distances_counted++;
        }
    }
    
    for(float i = number_of_ranges/4; i < number_of_ranges; i++)
    {
        filtered_scan.ranges[i] = 0;
    }

    // Calculate average range
    average_range /= distances_counted;

    // Remove oldest range if vector is of certain size
    if (average_ranges.size() == rolling_length)
    {
      average_ranges.erase(average_ranges.begin(), average_ranges.begin()+1);
    }

    // Add newest range to vector
    average_ranges.push_back(average_range);

    // Calculate running average of ranges
    rolling_average_range = 1.0 * std::accumulate(average_ranges.begin(),	\
      average_ranges.end(), 0.0) / average_ranges.size();

    ROS_INFO("average_range: %f", average_range);
    ROS_INFO("rolling_average_range: %f", rolling_average_range);

    if(rolling_average_range < 0.5)
        cmd_array.data.assign(right5, right5+22);
    else if(rolling_average_range < 0.6)
        cmd_array.data.assign(right3, right3+22);
    else if(rolling_average_range < 1.2)
        cmd_array.data.assign(right1, right1+22);
    else if(rolling_average_range < 1.5)
        cmd_array.data.assign(straight, straight+22);
    else if(rolling_average_range > 1.65)
	cmd_array.data.assign(straight, straight+22);
    else
        cmd_array.data.assign(left, left+22);

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
