#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <rosserial_arduino/Adc.h>

ros::NodeHandle nh;

#define y_axis A2
#define x_axis A3


rosserial_arduino::Adc adc_msg;
ros::Publisher p("adc", &adc_msg);


void setup(){

nh.initNode();
nh.advertise(p);

}


void loop() {

  adc_msg.adc0=analogRead(y_axis);
  adc_msg.adc1=analogRead(x_axis);

  p.publish(&adc_msg);
  nh.spinOnce();
}
