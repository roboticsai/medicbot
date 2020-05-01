#include <TimerOne.h>

// Controller pins
int CH_1_PIN = A0;
int CH_2_PIN = A1;
// Motor driver pins
int adc_pin = A7;
int buzzer_pin = 4;
//original
const int AIN1_PIN = 6;
const int AIN2_PIN = 7;
const int APWM_PIN = 3;
const int BIN1_PIN = 8;
const int BIN2_PIN = 9;
const int BPWM_PIN = 11;
const int adc_low_th = 520; //10.4Volts
const int adc_off_th = 480;
const int adc_ok_th = 580;

int buzzer_state = LOW;
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)


volatile int timer_flag = 0;
int low_voltage = 0;

void timer_isr() {
  timer_flag = 1;
}

void setup() {
  // Configure pins
  Serial.begin(9600);
  Timer1.initialize(300000);
  Timer1.attachInterrupt(timer_isr);
  Timer1.stop();

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(APWM_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(BPWM_PIN, OUTPUT);
  // Enable motor driver
  pinMode(5, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, LOW);
  TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
}

void toggle_buzzer() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (buzzer_state == LOW) {
      buzzer_state = HIGH;
    } else {
      buzzer_state = LOW;
    }

    digitalWrite(buzzer_pin, buzzer_state);
  }
}

void loop() {
  // Read pulse width from receiver

  int y = pulseIn(CH_2_PIN, HIGH);
//  Serial.print("Y: \t");
//  Serial.println(y);

  int x = pulseIn(CH_1_PIN, HIGH);
//  Serial.print("-----------");
//
//  Serial.print("X: \t");
//  Serial.println(x);

  int z = analogRead(adc_pin);
  // Before Mapping Values
  // 642 12.5V
  //769 14.5
  Serial.println(z);
  /*
    Serial.print(x);
    Serial.print("  ");
    Serial.println(y);
  */
  int PWMX = pulseToPWM(x);
  int PWMY = pulseToPWM(y);
  //  Serial.print(PWMX);
  //  Serial.println(PWMY);
  //  delay(10);
  // Set Speed for A & B
  analogWrite(APWM_PIN, abs(PWMX));
  analogWrite(BPWM_PIN, abs(PWMY));
  // For Motor A

  //VMS//
  if (z < adc_low_th) {
    low_voltage = 1;
  }
  else {
    low_voltage = 0;
    digitalWrite(buzzer_pin, LOW);
  }
  if (z < adc_off_th) {
    digitalWrite(buzzer_pin,HIGH);

     PWMX = 0;
    PWMY = 0;
    analogWrite(APWM_PIN, abs(PWMX));
    analogWrite(BPWM_PIN, abs(PWMY));
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, HIGH);
    while (z <= adc_ok_th) {
      z = analogRead(adc_pin);
        Serial.println(z);

      PWMX = 0;
      PWMY = 0;
      analogWrite(APWM_PIN, abs(PWMX));
      analogWrite(BPWM_PIN, abs(PWMY));
      digitalWrite(AIN1_PIN, HIGH);
      digitalWrite(AIN2_PIN, HIGH);

    }
    low_voltage = 0;
  }

  if (low_voltage) {
    toggle_buzzer();
  }

  if ( PWMX == 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, HIGH);
  } else if ( PWMX > 0 ) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
  }
  if ( PWMY == 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, HIGH);
  } else if ( PWMY > 0 ) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  }


}

int pulseToPWM(int pwm)
{
  if (pwm > 970) {
    pwm = map(pwm, 970, 1989, -255, 255);
  }
  else {
    pwm = 0;
  }
  if ( abs(pwm) <= 10) {
    pwm = 0;
  }
  return pwm;
}
