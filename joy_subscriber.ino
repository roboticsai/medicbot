 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum
 */
#include <ros.h>
#include <geometry_msgs/Pose.h>

ros::NodeHandle nh;
bool set_;

void messageCb(const geometry_msgs::Pose& msg){
  int x = msg.position.x;
  int y = msg.position.y;
  int z = msg.position.z;
}
ros::Subscriber<geometry_msgs::Pose> s("poses",messageCb);
void setup()
{
  nh.initNode();
  nh.subscribe(s);
}
void loop()
{
  nh.spinOnce();
  delay(10);
}
