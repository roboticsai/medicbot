//Receiver code
//under initial testing. 

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Try_RF_RX.h"
RF24 radio(3, 4); // CE, CSN
int recv_Data_x = 0;
int recv_Data_y = 0;
const long int timeout = 3000; //wait time until no data is received
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

const byte address[6] = "00001";

void setup() {
  //Generic inits
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  //for Motor Driver inits
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

}


void loop() {
  //start timer
  currentTime = millis();
  //check if overflown
  if (currentTime - lastReceiveTime > timeout) {
    Serial.println("No Data Recv");
    //enter failsafe function. to stop motor
  }

  if (radio.available()) {

    lastReceiveTime = millis(); // At this moment we have received the data
    radio.read(&recv_Data_x, sizeof(recv_Data_x));
    //    Serial.println(recv_Data_x);
    radio.read(&recv_Data_y, sizeof(recv_Data_y));
    //    Serial.println(recv_Data_y);

  }
  //data received.
  throttle = recv_Data_y;
  steering = recv_Data_x;
  //  Serial.print("Throttle : \t");
  //  Serial.println(throttle);
  //  Serial.println("     ");
  //
  //  Serial.print("SteeringL \t");
  //  Serial.println(steering);
  //
  //  //Motor Control:

  if (throttle < db_y - y_hyst) {
    //15 value tolerance for 0 postion:
    //MOTOR BACK
    // Set Motor A backward
    Serial.println("BACK");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    motorSpeedA = map(throttle, db_y - y_hyst, 0, 0, 255);
    motorSpeedB = map(throttle, db_y - y_hyst, 0, 0, 255);
    //add analogWrite???
  }


  else if (throttle > db_y + y_hyst) {
    //setMotor A forward
    Serial.println("Forward");
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    motorSpeedA = map(throttle, db_y + y_hyst , 255, 0, 255);
    motorSpeedB = map(throttle, db_y + y_hyst , 255, 0, 255);
  }

  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
    Serial.println("STOP");

  }

  //for steering using X
  if (steering < db_x - x_hyst) {
    Serial.println("Right");
    // Convert the declining steering readings from 140 to 255 into increasing 0 to 255 value
    int xMapped = map(steering, db_x - x_hyst, 0, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedA = motorSpeedA - xMapped;
    motorSpeedB = motorSpeedB + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA < 0) {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 255) {
      motorSpeedB = 255;
    }
  }

  if (steering > db_x + x_hyst) {
    Serial.println("Left");
    // Convert the increasing steering readings from 110 to 0 into 0 to 255 value
    int xMapped = map(steering, db_x + x_hyst, 255, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA > 255) {
      motorSpeedA = 255;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
    }
  }
  //lower threshold at holding
  if (motorSpeedA < 20) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 20) {
    motorSpeedB = 0;
  }
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
  Serial.print("PWM A \t");
  Serial.println(motorSpeedA);
  Serial.print("PWM B \t");
  Serial.println(motorSpeedB);


}
