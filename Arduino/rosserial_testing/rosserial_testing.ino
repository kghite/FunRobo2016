#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void messageCb( const geometry_msgs::Twist& twist_input ){
  float linear_vel  = twist_input.linear.x;
  float angular_vel = twist_input.angular.z;
  
  twist_notification = String(linear_vel);
  twist_notification += ", " + String(angular_vel);
  
  str_msg.data = twist_notification;
  chatter.publish( &str_msg );
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
