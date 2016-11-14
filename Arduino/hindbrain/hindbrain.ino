#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

//Set up ports and pins to support robot

//Setup for all the neoPixels
const byte LEFT_STRIP  = 2;
const byte RIGHT_STRIP = 3;
const byte LEFT_RING   = 4;
const byte RIGHT_RING  = 5;

Adafruit_NeoPixel left_strip = Adafruit_NeoPixel(8, LEFT_STRIP,NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel right_strip = Adafruit_NeoPixel(8, RIGHT_STRIP,NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel left_ring = Adafruit_NeoPixel(12, LEFT_RING,NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel right_ring = Adafruit_NeoPixel(12, RIGHT_RING,NEO_GRBW + NEO_KHZ800);

uint32_t white = left_strip.Color(0,0,0,128);
uint32_t red   = left_strip.Color(128,0,0,0);
uint32_t off   = left_strip.Color(0,0,0,0);

//Variables for blink timing.
unsigned long current_time;
unsigned long blink_time;
byte blinked = 0;
const int delay_period = 500; //Blinking speed for alive light
char command = 'g'; //Command from midbrain

//Pins and servo objects for Roboclaw control
const byte FORWARD_PIN = 6;
const byte TURN_PIN = 7;

Servo forward_channel;
Servo turn_channel;

//Set up ROS node handling and feedback channel
ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

//Various cariables for ROS workings
int linear_vel = 0;
int angular_vel = 0;
String notification;
byte hindbrain_stopped = 0; //Check for estop from hindbrain
byte midbrain_stopped  = 0; //Check for estop from midbrain

//Callback function for a Twist message TCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCb
void twistCb( const geometry_msgs::Twist& twist_input ){
  
  //Extract velocity data
  //Multiply by 1000 to maintain resolution
  linear_vel  = int(1000 * twist_input.linear.x);
  angular_vel = int(1000 * twist_input.angular.z);
  
  //Set motor speeds based on new command
  update_motors();
  
  //Print the received Twist message
  notification = "Received Twist message!\n";
  notification += "Linear vel: " + String(linear_vel) + "\n";
  notification += "Angular vel: " + String(angular_vel);
  
  chat(notification);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &twistCb );

//SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
void setup(){
  //Setup all the NeoPixels
  left_strip.begin();
  right_strip.begin();
  left_ring.begin();
  right_ring.begin();
  
  left_strip.show();
  right_strip.show();
  left_ring.show();
  right_ring.show();
  
  left_strip.setBrightness(32);
  right_strip.setBrightness(32);
  left_ring.setBrightness(32);
  right_ring.setBrightness(32);
  
  //Attach Servo objects to correct pins
  forward_channel.attach(FORWARD_PIN);
  turn_channel.attach(TURN_PIN);
  
  //Make sure the motors aren't moving
  motor_stop();
  
  //Initialize ROS topics
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  
  //Initialize timing things
  current_time = millis();
  blink_time   = millis();
}

//Run hindbrain loop until commanded to stop LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void loop(){
  //Read Midbrain commands
  
  //This is changed somewhat by ROS
  /* if(Serial.available())
  {
    command = Serial.read();
    Serial.println("Midbrain sent:");
    Serial.println(command);
  } */
  
  //Sense: Read robot sensors
  
  //Think: Run low level cognition and safety code
  if(!hindbrain_stopped && !midbrain_stopped) //If there's an estop, don't go
  {
    blink();
  }
  
  //Act: Run actuators and behavior lights
  
  //Write status data up to midbrain
  if(hindbrain_stopped || midbrain_stopped) //If there's an estop, say so and stop the motors
  {
    notification = "Hindbrain has stopped";
    chat(notification);
    
    //Stop the motors!
    motor_stop();
  }
  
  //Spin!
  nh.spinOnce();
  delay(1);
}

// Hindbrain Helper Functions******************************************************************************

//Writes a String message to the /chatter topic
void chat(String message){
  char charBuf[100];
  message.toCharArray(charBuf,100);    
  str_msg.data = charBuf;
  chatter.publish( &str_msg );
}

//Update motor speeds
void update_motors(){
  forward_channel.writeMicroseconds(1500 - linear_vel);
  turn_channel.writeMicroseconds(1500 - angular_vel);
}

//Stop motors
void motor_stop(){
  forward_channel.writeMicroseconds(1500);
  turn_channel.writeMicroseconds(1500);
}

//Blink all NeoPixels on and off
void blink(){
  current_time = millis();
  
  if(current_time - blink_time > delay_period){
    blinked = !blinked;
    
    //Logic to do turning/stopped lights
    if(blinked){
      if (linear_vel == 0 && angular_vel == 0)
        change_all_colors(red);
      else if (angular_vel > -0.05 && angular_vel < 0) //turning left
        change_left_colors(white);
      else if (angular_vel < 0.05 && angular_vel > 0) //turning right
        change_right_colors(white);
      else
        change_all_colors(white);
    }
    else
      change_all_colors(off);
    
    blink_time = millis();
  }
  
  
}

//Helper function to make all NeoPixels a given color
void change_all_colors(uint32_t color){
  for (int i = 0; i < 8; i++)
  {
    left_strip.setPixelColor(i,color);
    right_strip.setPixelColor(i,color);
  }
  for (int i = 0; i < 12; i++)
  {
    left_ring.setPixelColor(i,color);
    right_ring.setPixelColor(i,color);
  }
  left_strip.show();
  right_strip.show();
  left_ring.show();
  right_ring.show();
}

//Helpre functions for right and left banks of lights
void change_left_colors(uint32_t color){
  for (int i = 0; i < 8; i++)
  {
    left_strip.setPixelColor(i,color);
    right_strip.setPixelColor(i,off);
  }
  for (int i = 0; i < 12; i++)
  {
    left_ring.setPixelColor(i,color);
    right_ring.setPixelColor(i,off);
  }
  left_strip.show();
  right_strip.show();
  left_ring.show();
  right_ring.show();
}
 
void change_right_colors(uint32_t color){
  for (int i = 0; i < 8; i++)
  {
    left_strip.setPixelColor(i,off);
    right_strip.setPixelColor(i,color);
  }
  for (int i = 0; i < 12; i++)
  {
    left_ring.setPixelColor(i,off);
    right_ring.setPixelColor(i,color);
  }
  left_strip.show();
  right_strip.show();
  left_ring.show();
  right_ring.show();
} 
