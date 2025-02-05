#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 6
#define in4 7
int motorSpeedA = 0;
int motorSpeedB = 0;

void messageCb(const geometry_msgs::Twist& msg){
 
  int x = msg.linear.x;
  int y = msg.linear.y;  

  if (x < 0) {
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    motorSpeedA = -x;
  }
  else{ // Set Motor A forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    motorSpeedA = x;
  }
  
  if (y < 0) {
    // Set Motor A backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    motorSpeedB = -y;
  }
  else{
    // Set Motor A forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    motorSpeedB = y;
  }
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
}

ros::Subscriber<geometry_msgs::Twist> s("twist",messageCb);

void setup()
{
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(s);

  //for Motor Driver inits
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void loop()
{
  nh.spinOnce();
  delay(1);
}
