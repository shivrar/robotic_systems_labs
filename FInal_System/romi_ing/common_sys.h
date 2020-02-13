#include <USBCore.h>    // To fix serial print behaviour bug.
#include "lineSensors.h"

u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

//TODO: Setup a sym link for the the other lib instead of copy and pasting everything

void flash_leds ()
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); 
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

float weightedPower(LineSensor left, LineSensor centre, LineSensor right, int min_power, int max_power, byte &left_power, byte &right_power)
{
  float total = left.readCalibrated() + centre.readCalibrated() +right.readCalibrated();
  float rightM = right.readCalibrated()/(total);
  float leftM = left.readCalibrated()/(total);
  float m = leftM - rightM;  

  left_power = abs(m*max_power);
  right_power = abs(m*max_power);
  return m;
}
