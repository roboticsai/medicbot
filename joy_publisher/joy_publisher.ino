/* 
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum 
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

bool set_; 

geometry_msgs::Twist joy_data;
ros::Publisher p("/rrbot/mobile_base_controller/cmd_vel", &joy_data);

void setup()
{ 
  nh.initNode();
  nh.advertise(p);
}

float myMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop()
{  
  float xAxis = analogRead(A0);
  float yAxis = analogRead(A1);
   
  if (yAxis < 470) {
    yAxis = myMap(yAxis, 470, 0, 0, -1);
  }
  else if (yAxis > 550) {
    yAxis = myMap(yAxis, 550, 1023, 0, 1);
  }
  else {
    yAxis = 0;
  }
  
  if (xAxis < 470) {
    xAxis = myMap(xAxis, 470, 0, 0, -1);
  }
  else if (xAxis > 550) {
    xAxis = myMap(xAxis, 550, 1023, 0, 1);
  }
  else {
    xAxis = 0;
  }
  
  joy_data.linear.x = xAxis;
  joy_data.linear.y = 0.0;
  joy_data.linear.z = 0.0;  

  joy_data.angular.x = 0.0;
  joy_data.angular.y = 0.0;
  joy_data.angular.z = yAxis;
  
  p.publish(&joy_data);
  nh.spinOnce();
  delay(10);
}
