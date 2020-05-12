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

void loop()
{  
  int xAxis = analogRead(A0);
  int yAxis = analogRead(A1);
   
  joy_data.linear.x = xAxis;
  joy_data.linear.y = yAxis;
  joy_data.linear.z = 11;  

  joy_data.angular.x = xAxis;
  joy_data.angular.y = yAxis;
  joy_data.angular.z = 11;
  
  p.publish(&joy_data);
  nh.spinOnce();
  delay(10);
}
