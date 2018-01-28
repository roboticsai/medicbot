/* 
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum 
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

bool set_; 

geometry_msgs::Twist joy_data;
ros::Publisher p("/twist", &joy_data);

void setup()
{ 
  nh.initNode();
  nh.advertise(p);
}

void loop()
{  
  int xAxis = analogRead(A0);
  int yAxis = analogRead(A1);
   
  if (yAxis < 470) {
    yAxis = map(yAxis, 470, 0, 0, -255);
  }
  else if (yAxis > 550) {
    yAxis = map(yAxis, 550, 1023, 0, 255);
  }
  else {
    yAxis = 0;
  }
  
  if (xAxis < 470) {
    xAxis = map(xAxis, 470, 0, 0, -255);
  }
  else if (xAxis > 550) {
    xAxis = map(xAxis, 550, 1023, 0, 255);
  }
  else {
    xAxis = 0;
  }
  
  joy_data.linear.x = xAxis;
  joy_data.linear.y = yAxis;
  joy_data.linear.z = 0.0;  

  joy_data.angular.x = 0.0;
  joy_data.angular.y = 0.0;
  joy_data.angular.z = 0.0;
  
  p.publish(&joy_data);
  nh.spinOnce();
  delay(10);
}
