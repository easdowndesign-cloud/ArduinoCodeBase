// Include required libraries (not used for ArduinoMEGA beta setup).

// #include "Portenta_H7_PWM.h"

// #include <Arduino_PortentaBreakout.h>

//NOTE: PWM DigPin 13 and 4 operate at 980 Hz. All other PWM DigPins operate at 490 Hz.
const int LJYIn = A0;  // Pin number for lefthand joystick y axis input.
const int LJXIn = A1;  // Pin number for lefthand joystick X axis input.
int LJYVal = 0;  // LJXIn read data variable storage.
int LJXVal = 0;  // LJXIn read data variable storage.
const int LJYOut = 11;  // PWM digital pin number to be used for outputting LJYVal values.
const int LJXOut = 12;  // PWM digital pin number to be used for outputting LJXVal values.
constexpr uint16_t TIMER1_TOP = 3999; // 500 Hz => 2000 us period. Timer1 prescaler 8 => 0.5 us tick => TOP = 2000/0.5 - 1 = 3999

/*With Portenta H7, applying an external reference voltage does not need to be determined in the sketch.
0 - 3.1 V IS THE MAXIMUM REFERENCE VOLTAGE FOR PORTENTA H7!!!*/


// setup timer to create the PWM signal
void setupTimer1_500Hz_OC1A_OC1B() {
  pinMode(LJXOut, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Fast PWM, TOP=ICR1 (mode 14), non-inverting OC1A/OC1B, prescaler=8
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS11);

  ICR1  = TIMER1_TOP;
  OCR1A = 0;
  OCR1B = 0;
}


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);  // Initialise serial comms.
  pinMode(LJXIn, INPUT);
  setupTimer1_500Hz_OC1A_OC1B();
}


void loop() {
  // put your main code here, to run repeatedly:
  
  // Lefthand Joystick X Axis.

  LJXVal = analogRead(LJXIn); // Read analogue value in.
  LJYVal = analogRead(LJYIn); // Read analogue value in.
  
  
  
  LJXVal = map(LJXVal, 0, 1023, 0, 4095);  // Mapping input low and high values to output low and high values.
  LJYVal = map(LJXVal, 0, 1023, 0, 4095);  // Mapping input low and high values to output low and high values.



  analogWrite(LJXOut, LJXVal); // PWM output.
  analogWrite(LJYOut, LJYVal); // PWM output.
  delay(50);

// want to serial print every 

  Serial.print("Lefthand Joystick X In: ");  
  Serial.println(LJXVal);
  Serial.println("  |  Lefthand Joystick X Out: ");
  Serial.println(LJXVal);
  Serial.print("Lefthand Joystick Y In: ");  
  Serial.println(LJYVal);
  
  Serial.println("  |  Lefthand Joystick Y Out: ");
  Serial.println(LJYVal);
}
