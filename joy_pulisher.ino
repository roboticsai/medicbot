/* 
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum 
 */

#include <ros.h>
#include <geometry_msgs/Pose.h>

ros::NodeHandle nh;

bool set_; 

geometry_msgs::Pose joy_data;
ros::Publisher p("pose", &joy_data);

void setup()
{ 
  nh.initNode();
  nh.advertise(p);
}

void loop()
{  
  int xAxis = analogRead(A0);
  int yAxis = analogRead(A1);
  
  joy_data.position.x = xAxis;
  joy_data.position.y = yAxis;
  joy_data.position.z = 0;
  
  p.publish(&joy_data);
  nh.spinOnce();
  delay(10);
}
