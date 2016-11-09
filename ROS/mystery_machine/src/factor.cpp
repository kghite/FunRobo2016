#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

void factorPrimes(const std_msgs::Int16 msg)
{
  int n = msg.data;
  std::stringstream ss;
  std::stringstream input;

  input << msg.data;

  while(n % 2 == 0) {
  	int x = 2;
  	ss << 2 << ", ";
  	n = n / 2;
  }

  for (int i = 3; i <= sqrt(n); i = i + 2) {
  	while (n % i == 0){
  		ss << i << ", ";
  		n = n / i;
  	}
  }

  if (n > 2) {
  	ss << n;
  }

  ROS_INFO("Prime Factors of %s: [%s]", input.str().c_str(), ss.str().c_str());
  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "factor");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("factor_input", 1000, factorPrimes);

  ros::spin();

  return 0;
}