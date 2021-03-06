#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#include <known_16bit_timers.h>
#include <Adafruit_TiCoServo.h>

//Set up ports and pins to support robot

//Setup for all the neoPixels
const byte LEFT_STRIP  = 3;
const byte RIGHT_STRIP = 2;
const byte LEFT_RING   = 5;
const byte RIGHT_RING  = 4;

Adafruit_NeoPixel left_strip = Adafruit_NeoPixel(8, LEFT_STRIP,NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel right_strip = Adafruit_NeoPixel(8, RIGHT_STRIP,NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel left_ring = Adafruit_NeoPixel(12, LEFT_RING,NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel right_ring = Adafruit_NeoPixel(12, RIGHT_RING,NEO_GRBW + NEO_KHZ800);

uint32_t white = left_strip.Color(0,0,0,255);
uint32_t yellow= left_strip.Color(255,128,0);
uint32_t red   = left_strip.Color(255,0,0,0);
uint32_t off   = left_strip.Color(0,0,0,0);
uint32_t blue  = left_strip.Color(0,64,255,0);
uint32_t purple= left_strip.Color(255,0,255,0);

//Variables for blink timing.
unsigned long current_time;
unsigned long blink_time;
byte blinked = 0;
int delay_period = 500; //Blinking speed for alive light
char command = 'g'; //Command from midbrain

//Pins and servo objects for Roboclaw control
const byte FORWARD_PIN = 6;
const byte TURN_PIN = 7;

Adafruit_TiCoServo forward_channel;
Adafruit_TiCoServo turn_channel;

int linear_vel =  0;
int angular_vel = 0;

int current_linear_vel = 0;
int current_angular_vel = 0;

//Define estop pin
const byte ESTOP_PIN = 42;

//Define IR sensor variables
const byte IR_PIN_1 = A0;
const byte IR_PIN_2 = A1;
//const int ir_high_threshold = 450;
const int ir_low_threshold  = 300;
int ir_estop = 0;

//Set up ROS node handling and feedback channel
ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

std_msgs::Int16 int_msg;
ros::Publisher ir_estop_publisher("ir_estop",&int_msg);

//Various variables for ROS workings
int odroid_estop = 0;
String notification;


//Define LIDAR tilt servo
const byte TILT_PIN = 8;
Adafruit_TiCoServo tilt_servo;
const int middle_tilt_position = 110;
int current_tilt_position = middle_tilt_position;

//Callback function for a Twist message TCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCbTCb
void twistCb( const geometry_msgs::Twist& twist_input ){
  
  //Extract velocity data
  //Multiply by 90 to maintain resolution
  //Ends up giving a signal for write between 0-180
  //Make sure cmd_vel has values that range -1 to 1
  linear_vel  = int(90 * twist_input.linear.x);
  angular_vel = int(90 * twist_input.angular.z);
  
  //Set motor speeds based on new command
  //update_motors();
  
  //Print the received Twist message
  notification = "Received Twist message!\n";
  notification += "Linear vel: " + String(linear_vel) + "\n";
  notification += "Angular vel: " + String(angular_vel);
  
  chat(notification);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &twistCb );

//Callback function for an IR Estop message IRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCbIRCb
void irEstopCallback( const std_msgs::Int16& int_input ){
  //Check and see if the midbrain says it's okay to move
  int estop_message = int_input.data;
  
  if (estop_message == 2){
    ir_estop = 2;
  }
  
}

ros::Subscriber<std_msgs::Int16>ir_estop_sub("ir_estop",&irEstopCallback);

//Callback function for an Odroid Estop message OCbOcbOcbOCbOcbOcbOCbOcbOcbOCbOcbOcbOCbOcbOcbOCbOcbOcbOCbOcbOcbOCbOcbOcb
void odroidEstopCallback( const std_msgs::Int16& int_input ){
  //Just update this estop.. Pretty easy
  int estop_message = int_input.data;
  
  odroid_estop = estop_message;
}

ros::Subscriber<std_msgs::Int16>odroid_estop_sub("odroid_estop",&odroidEstopCallback);

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
  
  left_strip.setBrightness(16);
  right_strip.setBrightness(16);
  left_ring.setBrightness(16);
  right_ring.setBrightness(16);
  
  //Attach Servo objects to correct pins
  forward_channel.attach(FORWARD_PIN);
  turn_channel.attach(TURN_PIN);
  
  //Make sure the motors aren't moving
  //update_motors();
  
  //Initialize ROS topics
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(ir_estop_publisher);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(ir_estop_sub);
  nh.subscribe(odroid_estop_sub);
  
  //Initialize timing things
  current_time = millis();
  blink_time   = millis();
  
  pinMode(ESTOP_PIN,INPUT);
  
  tilt_servo.attach(TILT_PIN);
}

//Run hindbrain loop until commanded to stop LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
void loop(){
  //Read Midbrain commands
  
  
  //Sense: Read robot sensors
  
  //Think: Run low level cognition and safety code TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
  if(digitalRead(ESTOP_PIN))
    delay_period = 500;
  else{
    delay_period = 150;
    //chat("Physical estop pressed");
  }
  
  check_ir_sensors();
  
  //Act: Run actuators and behavior lights
  blink();
  
  //tilt_servo.write(current_tilt_position);
  
  update_motors();
  
  //Write status data up to midbrain
  
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
  //The ir_estop is the only estop the Arduino has to tell itself to stop via software
  if (ir_estop == 1 || odroid_estop == 1){
   forward_channel.write(90);
    turn_channel.write(90);
    current_linear_vel = 0;
    current_angular_vel = 0;
  }
  else{
    forward_channel.write(90 + current_linear_vel);
    turn_channel.write(90 + current_angular_vel);
    if (linear_vel > current_linear_vel)
      current_linear_vel += 1;
    else if (linear_vel < current_linear_vel)
      current_linear_vel -= 1;
      
    if (angular_vel > current_angular_vel)
      current_angular_vel += 1;
    else if (angular_vel < current_linear_vel)
      current_angular_vel -= 1;
  }
}

//See if we need to estop based on IR input
void check_ir_sensors(){
  int ir_reading_1;
  int ir_reading_2;
  ir_reading_1 = analogRead(IR_PIN_1);
  ir_reading_2 = analogRead(IR_PIN_2);
  chat(String(ir_reading_1));
  chat(String(ir_reading_2));
  chat("------");
  
  if (ir_reading_1 < ir_low_threshold || ir_reading_2 < ir_low_threshold){
    if (ir_estop == 0){
      ir_estop = 1;
      int_msg.data = ir_estop;
      ir_estop_publisher.publish(&int_msg);
    }
    
    else if (ir_estop == 2){
      
      if (linear_vel > 0){
        ir_estop = 1;
        int_msg.data = ir_estop;
        ir_estop_publisher.publish(&int_msg);
      }
    }
    
    
  }
  else{
    if (ir_estop != 0){
      ir_estop = 0;
      int_msg.data = ir_estop;
      ir_estop_publisher.publish(&int_msg);
    }
  }
}

//Blink all NeoPixels on and off
void blink(){
  current_time = millis();
  
  if(current_time - blink_time > delay_period){
    blinked = !blinked;
    
    //Logic to do turning/stopped lights
    if(blinked){
      if (linear_vel == 0 && angular_vel == 0 && ir_estop == 1)
        change_all_colors(purple);
      else if (linear_vel == 0 && angular_vel == 0)
        change_all_colors(red);
      else if (ir_estop == 1)
        change_all_colors(blue);
      else if (angular_vel < -3){ //turning left
        if(linear_vel>=0)
          change_left_colors(white);
        else
          change_left_colors(yellow);
      }
      else if (angular_vel > 3){ //turning right
        if(linear_vel>=0)
          change_right_colors(white);
        else
          change_right_colors(yellow);
      }
      else if (linear_vel >= 0)
        change_all_colors(white);
      else
        change_all_colors(yellow);
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

//Helper functions for right and left banks of lights
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
