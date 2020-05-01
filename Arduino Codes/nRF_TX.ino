/*
  Send Joystick Values to Bot
  
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";
byte dummy=20;
int adc_value_x=0;
int adc_value_y=0;
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {

  adc_value_y=analogRead(A2);
  adc_value_x=analogRead(A3);
  Serial.println(adc_value_x);
  Serial.println(adc_value_y);
  radio.write(&adc_value_x, sizeof(adc_value_x));
  radio.write(&adc_value_y, sizeof(adc_value_y));
}
