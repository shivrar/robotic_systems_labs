#include "encoders.h"
#include "lineSensors.h"
#include "common_sys.h"
#include "pid.h"

#define LINE_LEFT_PIN A2
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN A4
#define FORWARD LOW
#define REVERSE HIGH

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

//~~~~~~~~~~~~~~~~~~TODO's~~~~~~~~~~~~~~~~~~~~~//
// TODO: Look at the idea of confidence for line following for the robot. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

volatile float confidence;


int max_power = 50;
int min_power = 20;

float leftM, rightM, total, m;

volatile byte l_power;
volatile byte r_power;
volatile bool l_direction;
volatile bool r_direction;

//intialise the state
// NB: -1 is a debug state So use that accordingly
int state = 0;

LineSensor l_sensor(LINE_LEFT_PIN);
LineSensor c_sensor(LINE_CENTRE_PIN);
LineSensor r_sensor(LINE_RIGHT_PIN);


//Interrupt definition here
//volatile boolean DEBUG_LED_STATE;
double hz = 10.0;
volatile double prev_theta_e1 = 0.0;
volatile double prev_theta_e0 = 0.0;


// Variables for speed that are globally acessible
volatile double left_wheel_vel;
volatile double right_wheel_vel;

ISR( TIMER3_COMPA_vect ) {
//  Do speed calcs and odom in here
left_wheel_vel = (theta_e0 - prev_theta_e0)*hz;
right_wheel_vel = (theta_e1 - prev_theta_e1)*hz;

prev_theta_e0 = theta_e0;
prev_theta_e1 = theta_e1;

}

void setup() {

  setupEncoder0();
  setupEncoder1();

  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );

  pinMode(13, OUTPUT);

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

  //setup timer for speed and odom timed calculations
  setupTimer3(hz);  
  
}

//TODO: Implement the probablistic nature now for the robot with confidence values.

////N.B. e0 = left wheel, e1 = right wheel
// Remmeber, loop is called again and again.
void loop() 
{


//switch case logic for romi -> each state should only adjust power and direction of motors for keeping traceability
switch(state){
//Now that we have the sensor lets try to find the line

  // Debug state!!!!!
  case -1:
  l_power = max_power;
  r_power = max_power;
  l_direction = FORWARD;
  r_direction = FORWARD;
  break;
  
  case 0:
    if(l_sensor.readCalibrated()<300 && c_sensor.readCalibrated()<300 && r_sensor.readCalibrated()< 300)
    {
      l_power = min(l_power + 4, min_power);
      l_direction = FORWARD;
      r_power = min(r_power + 4, min_power);
      r_direction = FORWARD;
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

      if(m >0.15)
      {
        r_direction = FORWARD;
        l_direction = REVERSE;
      }
      else if(m < -0.15)
      {
        r_direction = REVERSE;
        l_direction = FORWARD;
      }
      else
      {
        r_direction = FORWARD;
        l_direction = FORWARD;
      }


    l_power = max(min(l_power, max_power),min_power);
    r_power = max(min(r_power, max_power),min_power);
    break;
}
  // Send power PWM to pins, to motor drivers.
  digitalWrite( R_DIR_PIN, r_direction);
  digitalWrite( L_DIR_PIN, l_direction);
  analogWrite( L_PWM_PIN, l_power );
  analogWrite( R_PWM_PIN, r_power );
//  Serial.print(m);
//  Serial.print(",");
//  Serial.print(l_sensor.readCalibrated());
//  Serial.print(",");
//  Serial.print(c_sensor.readCalibrated());
//  Serial.print(",");
//  Serial.print(r_sensor.readCalibrated());
//  Serial.print(",");
//  Serial.println((l_sensor.readCalibrated() + c_sensor.readCalibrated() +r_sensor.readCalibrated())/3);
  delay(5);
}
