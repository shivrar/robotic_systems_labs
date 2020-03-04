#include <USBCore.h>    // To fix serial print behaviour bug.
#include "lineSensors.h"

u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)
#define LINE_LEFT_PIN A2
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN A4
#define FORWARD LOW
#define REVERSE HIGH

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15


void setupMotorPins()
{
    // Set our motor driver pins as outputs.
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  // Set initial direction for l and r
  // Which of these is foward, or backward?
  digitalWrite( L_DIR_PIN, FORWARD);
  digitalWrite( R_DIR_PIN, FORWARD);
}
//TODO: Setup a sym link for the the other lib instead of copy and pasting everything

void flash_leds ()
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); 
}

float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Function overload for variable time
void flash_leds (int delay_ms)
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(delay_ms);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(delay_ms); 
}

void play_tone(int ledpin_no, int volume)
{
    analogWrite(ledpin_no, volume);
    delay(1);
    analogWrite(ledpin_no, volume);
}

void BangBang(LineSensor left, LineSensor centre, LineSensor right, int min_power, int max_power, byte &left_power, byte &right_power)
{
  byte sense = 0;
  sense = sense | ( (left.readCalibrated()>200 ? 1 : 0)  << 2 );
  sense = sense | ( (centre.readCalibrated()>200 ? 1 : 0)   << 1 );
  sense = sense | ( (right.readCalibrated()>200 ? 1 : 0)  << 0 );

  switch(sense){
    case 0:
      left_power = min_power;
      right_power = min_power;
    break;
    case 1:
      left_power = max_power;
      right_power = 0;
    break;
    case 3:
      left_power = max_power;
      right_power = min_power;
    break;
    case 4:
      left_power = 0;
      right_power = max_power;
    break;
    case 6:
      left_power = min_power;
      right_power = max_power;
    break;
    case 7:
      left_power = max_power;
      right_power = max_power;
    break;
    default:
    break;
  }
  
}

float weightedPower(LineSensor left, LineSensor centre, LineSensor right, int min_power, int max_power,volatile byte &left_power,volatile byte &right_power)
{
  float total = left.readCalibrated() + centre.readCalibrated() +right.readCalibrated();
  float rightM = right.readCalibrated()/(total);
  float leftM = left.readCalibrated()/(total);
  float m = leftM - rightM;  

  left_power = abs(m*max_power);
  right_power = abs(m*max_power);
  return max(min(m,1),-1);
}

float weightedPower(LineSensor left, LineSensor centre, LineSensor right, int min_power, int max_power)
{
  float total = left.readCalibrated() + centre.readCalibrated() +right.readCalibrated();
  float rightM = right.readCalibrated()/(total);
  float leftM = left.readCalibrated()/(total);
  float m = rightM - leftM;  

  return max(min(m,1),-1);
}

// Routine to setupt timer3 to run 
void setupTimer3(float hz) {
  
  // disable global interrupts
  cli();          

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0

  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  // For a cpu clock precaler of 256:
  // Shift a 1 up to bit CS32 (clock select, timer 3, bit 2)
  // Table 14.5 in manual. 
  TCCR3B = TCCR3B | (1 << CS32);
  
  
  // set compare match register to desired timer count.
  // CPU Clock  = 16000000 (16mhz).
  // Prescaler  = 256
  // Timer freq = 16000000/256 = 62500
  // We can think of this as timer3 counting up to 62500 in 1 second.
  // compare match value = 62500 / 2 (we desire 2hz).
  OCR3A = 62500.00/hz;
  
  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei(); 
  
}

// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the 
// compiler.  It automatically associates with Timer3 in
// CTC mode.
