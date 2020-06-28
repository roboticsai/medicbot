String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 6
#define in4 7

void setup()
{
  // initialize serial:
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

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
    // print the string when a newline arrives:
  if (stringComplete) {   
    int joint1_vel = inputString.substring(0,4).toInt();
    int joint2_vel = inputString.substring(4,8).toInt();

    Serial.print(joint1_vel); Serial.print(","); Serial.print(joint2_vel); Serial.print("---");
  
    if (joint1_vel < 0) {
      // Set Motor A backward
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      joint1_vel = -joint1_vel;
    }
    else if (joint1_vel > 0) {
      // Set Motor A forward
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      joint1_vel = joint1_vel;
    }
  
    if (joint2_vel < 0) {
      // Set Motor A backward
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      joint2_vel = -joint2_vel;
    }
    else if (joint2_vel > 0) {
      // Set Motor A forward
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      joint2_vel = joint2_vel;
    }

    Serial.print(joint1_vel); Serial.print(","); Serial.println(joint2_vel);

    analogWrite(enA, joint1_vel); // Send PWM signal to motor A
    analogWrite(enB, joint2_vel); // Send PWM signal to motor B
 
    inputString = "";
    stringComplete = false;
  } 

}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()>0) {
    // get the new byte:
    char inChar = Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
