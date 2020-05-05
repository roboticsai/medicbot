/* 
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum 
 */

#include <ros.h>
#include <geometry_msgs/Pose.h>

ros::NodeHandle nh;

bool set_; 

geometry_msgs::Pose sum_msg;
ros::Publisher p("pose", &sum_msg);

void setup()
{ 
  nh.initNode();
  nh.advertise(p);
}

void loop()
{  
  sum_msg.position.x = 1;
  sum_msg.position.y = 2;
  sum_msg.position.z = 3;
  
  p.publish(&sum_msg);
  nh.spinOnce();
  delay(10);
}
