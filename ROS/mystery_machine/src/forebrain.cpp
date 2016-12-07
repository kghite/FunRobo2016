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
#include "std_msgs/Int16MultiArray.h"
#include "sensor_msgs/LaserScan.h"

//sensor_msgs::NavSatFix gps_pos;
//geometry_msgs::Vector3Stamped imu_mag;
std_msgs::Int8MultiArray cmd_array;
std_msgs::Int8MultiArray cone_cmd_array;
sensor_msgs::LaserScan filtered_scan;
sensor_msgs::LaserScan scan;

ros::Publisher pub_filtered_scan;
ros::Publisher pub_arb;
ros::Publisher cone_pub_arb;

std::vector<float> average_ranges;
int rolling_length = 5;

std::vector<int> cone_vel_command(202,0);

// Object weights that get passed to the arbiter
const int WALL = 1;
const int CONE = 2;

// input ang from -50 to 50
std::vector<int> set_vel_vector(int object_weight, int ang)
{
  std::vector<int> vel(202, 0);
  vel.at(ang+151) = object_weight;
  return vel;
}

void getLIDAR(const sensor_msgs::LaserScan lidar_scan)
{
  int lin_vel = 0;
  int ang_vel = 0;
  std::vector<int> wall_vel_command;
  std::vector<int> final_vel_command(202,0);

  scan.ranges = lidar_scan.ranges;
  filtered_scan.ranges = lidar_scan.ranges;
  filtered_scan.header.frame_id = lidar_scan.header.frame_id;
  filtered_scan.angle_min = lidar_scan.angle_min;
  filtered_scan.angle_max = lidar_scan.angle_max;
  filtered_scan.angle_increment = lidar_scan.angle_increment;
  filtered_scan.range_max = lidar_scan.range_max;
  filtered_scan.range_min = lidar_scan.range_min;

  long number_of_ranges = lidar_scan.ranges.size();

  float average_range = 0.0;

  float rolling_average_range;

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
    rolling_average_range = 1.0 * std::accumulate(average_ranges.begin(), \
      average_ranges.end(), 0.0) / average_ranges.size();

    ROS_INFO("average_range: %f", average_range);
    ROS_INFO("rolling_average_range: %f", rolling_average_range);
    
    if(rolling_average_range < 0.5)
    {
      ROS_INFO("-3");
      //ang_vel -= 5;
      ang_vel = -15;
    }
    else if(rolling_average_range < 0.6)
    {
      ROS_INFO("-2");
      //ang_vel -= 3;
      ang_vel = -10;
    }
    else if(rolling_average_range < 1.2)
    {
      ROS_INFO("-1");
      //ang_vel -= 1;
      ang_vel = -5;
    }
    else if(rolling_average_range < 1.5)
    {
      ROS_INFO("00");
      //ang_vel += 1;
      ang_vel = 0;
    }
    else if(rolling_average_range < 1.65)
    {
      ROS_INFO("+1");
      //ang_vel += 5;
      ang_vel = 10;
    }
    else
    {
      ROS_INFO("+2");
      ang_vel = 0;
    }

    // Set the wall_vel_command slider for the given ang_vel
    wall_vel_command = set_vel_vector(WALL, ang_vel);

    // Add wall and cone velocity sliders
    for (int i=0; i<wall_vel_command.size(); i++)
    {
      final_vel_command[i] = wall_vel_command[i] + cone_vel_command[i];
    }

    // Make an array copy of the final_vel_command slider vector
    int* final_vel_arr = &final_vel_command[0];
    cmd_array.data.assign(final_vel_arr, final_vel_arr+202);

    pub_filtered_scan.publish(filtered_scan);

    pub_arb.publish(cmd_array);
    cmd_array.data.clear();
}

int cone_height_threshold = 30;
int left_threshold = 160;
int right_threshold = 480;
void cone_callback(const std_msgs::Int8MultiArray cone_array)
{
  std::vector<int> cone_xs;
  std::vector<int> cone_ys;
  std::vector<int> cone_widths;
  std::vector<int> cone_heights;

  int cone_array_length = cone_array.data.size();

  // Track x,y,width,height of cones that are tall enough
  for(int i = 0; i < cone_array_length; i+=4)
  {
    if(cone_array.data[i+3] > cone_height_threshold)
    {
      cone_xs.push_back(cone_array.data[i]);
      cone_ys.push_back(cone_array.data[i+1]);
      cone_widths.push_back(cone_array.data[i+2]);
      cone_heights.push_back(cone_array.data[i+3]);
    }
  }

  // Store the direction we need to end up turning
  int ang_vel = 0;

  for (int i = 0; i < cone_xs.size(); i++)
  {
    //Do we need to turn right away from a cone?
    if(cone_xs[i] > left_threshold && cone_xs[i] < 320)
      //ang_vel = -10;
      ang_vel -= 10;
    //Do we need to turn left away from a cone?
    else if(cone_xs[i] > 320 && cone_xs[i] < right_threshold)      
      //ang_vel = 10;
      ang_vel += 10;
  }

  // Define the cone_vel_command slider based on given ang_vel
  cone_vel_command = set_vel_vector(CONE, ang_vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "forebrain");
    ros::NodeHandle n;

  /* ros::Subscriber sub_goal = n.subscribe("goal", 1000, turnToGoal);

  ros::Subscriber sub_gps = n.subscribe("fix", 1000, getGPS);

  ros::Subscriber sub_imu = n.subscribe("imu/mag", 1000, getIMU); */

  ros::Subscriber sub_lidar = n.subscribe("scan",1000,getLIDAR);

  ros::Subscriber sub_cones = n.subscribe("cone_positions",1000,cone_callback);

  pub_arb = n.advertise<std_msgs::Int8MultiArray>("wpt/cmd_vel", 1000);

  pub_filtered_scan = n.advertise<sensor_msgs::LaserScan>("filtered_scan",1000);

  pub_arb.publish(cmd_array);

  ros::spin();

  return 0;
}
