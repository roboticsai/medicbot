#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <rosserial_arduino/Adc.h>

ros::NodeHandle nh;

void recvCb(const rosserial_arduino::Adc recv_msg){

}

ros::Subscriber(rosserial_arduino::)

void setup(){
Serial.begin(9600);
}

void loop(){


}
