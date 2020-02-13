#include "encoders.h"
#include "lineSensors.h"
#include "common_sys.h"
//#include "pid.h"

#define LINE_LEFT_PIN A2
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN A4
#define FORWARD LOW
#define REVERSE HIGH

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

int max_power = 50;
int min_power = 20;

float leftM, rightM, total, m;

byte l_power;
byte r_power;

//intialise the state

int state = 0;

LineSensor l_sensor(LINE_LEFT_PIN);
LineSensor c_sensor(LINE_CENTRE_PIN);
LineSensor r_sensor(LINE_RIGHT_PIN);


void setup() {

  setupEncoder0();
  setupEncoder1();

  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  digitalWrite( L_DIR_PIN, FORWARD);
  digitalWrite( R_DIR_PIN, FORWARD);
  
  Serial.begin( 9600 );

//  run a calibrationfor the light sensor
  l_sensor.calibrate();
  c_sensor.calibrate();
  r_sensor.calibrate();

  pinMode(6, OUTPUT);
  flash_leds(500);
    // wait for a second
  play_tone(6, 125);
  delay(1000);
  analogWrite(6, 0);
  
}

//TODO: Implement the probablistic nature now for the robot with confidence values.

////N.B. e0 = left wheel, e1 = right wheel
// Remmeber, loop is called again and again.
void loop() 
{


//switch case logic for romi
switch(state){
//Now that we have the sensor lets try to find the line
  case 0:
    if((l_sensor.readCalibrated() + c_sensor.readCalibrated() +r_sensor.readCalibrated())/3 < 400)
    {
      l_power = min(l_power + 4, max_power);
      r_power = min(r_power + 4, max_power);
    }
  
    else
    {
      l_power = 0;
      r_power = 0;
      state = 1;
    }
  break;
  case 1:
      m = weightedPower(l_sensor, c_sensor, r_sensor, min_power, max_power,l_power, r_power);

      if(m >0.2)
      {
        digitalWrite( R_DIR_PIN, FORWARD);
        digitalWrite( L_DIR_PIN, REVERSE);
      }
      else if(m < -0.2)
      {
        digitalWrite( R_DIR_PIN, REVERSE);
        digitalWrite( L_DIR_PIN, FORWARD);
      }
      else
      {
        digitalWrite( L_DIR_PIN, FORWARD);
        digitalWrite( R_DIR_PIN, FORWARD);   
      }
//    Serial.print(m);
//    Serial.print(",");

    l_power = max(min(l_power, max_power),min_power);
    r_power = max(min(r_power, max_power),min_power);
//    Serial.print(l_power);
//    Serial.print(",");
//    Serial.println(r_power);
    break;
}
    // Send power PWM to pins, to motor drivers.
  analogWrite( L_PWM_PIN, l_power );
  analogWrite( R_PWM_PIN, r_power );
  delay(10);
}
