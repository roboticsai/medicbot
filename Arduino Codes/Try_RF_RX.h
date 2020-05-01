//motordrivers:

#define enA 5  // Arduino pin D9 - CH6 on PCB board - PWM output
#define in1 6  // D8 - CH5 
#define in2 7  // D7 - CH4
#define in3 8  // D6 - CH3 
#define in4 9  // D4 - CH1 
#define enB 10  // D5 - CH2 - PWM output
//motor control params:
int steering, throttle;
int motorSpeedA = 0;
int motorSpeedB = 0;
const int db_x = 508;
const int db_y = 520;
const int y_hyst = 40;
const int x_hyst = 40;
