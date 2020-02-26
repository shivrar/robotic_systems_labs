#include "encoders.h"
#include "lineSensors.h"
#include "common_sys.h"
#include "pid.h"
#include "SimpleKalmanFilter.h"
#include "kinematics.h"

//~~~~~~~~~~~~~~~~~~TODO's~~~~~~~~~~~~~~~~~~~~~//
// TODO: Look at the idea of confidence for line following for the robot. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

volatile float confidence;

float max_speed = 25.84; //Max reachable speed of the wheels -> output is 255
int max_power = 50; // ~5.06
int min_power = 20;
float max_des_speed = 6.0;

/*20= 2.02, 50 = 5.7, 63.75= 6.8, 100=11.0 , 152 = 17.2, 191.25 = 20.5, 235=24.96  ,255=26.84

speed = 0.1056*(PWM) + 0.287 is an alright linear approximation to convert pwm signal to speed values

*/

float slope = 0.1056, y_int = 0.287;

float right_output = 0.0;
float left_output= 0.0;
float heading_output = 0.0;
//
//PID left_wheel(1.1, 0.085,0.1);
//PID right_wheel(1.1, 0.085,0.1);
//PID left_wheel(1.0, 0.1,0);
//PID right_wheel(1.0, 0.1,0);
PID left_wheel(0.3, 0.0000,1.0);
PID right_wheel(0.3, 0.0000,1.0);
PID heading(0.24,0.0,0.9);


volatile byte l_power;
volatile byte r_power;
volatile bool l_direction;
volatile bool r_direction;
volatile float m;
//intialise the state
// NB: -1, -2, -3 are a debuging state So use that accordingly
int state = 0;

LineSensor l_sensor(LINE_LEFT_PIN);
LineSensor c_sensor(LINE_CENTRE_PIN);
LineSensor r_sensor(LINE_RIGHT_PIN);


//Interrupt definition here
double hz = 250.0;
volatile double prev_theta_e1 = 0.0;
volatile double prev_theta_e0 = 0.0;


// Variables for speed that are globally acessible
volatile double left_wheel_vel;
volatile double left_wheel_est;
volatile double right_wheel_vel;
volatile double right_wheel_est;
SimpleKalmanFilter left_filter(0.3,0.1,0.01);
SimpleKalmanFilter right_filter(0.3,0.1,0.01);

//Control loop timers
unsigned long last_timestamp;
Kinematics2D::Kinematics Romi(0.14, 0.035);
int count = 0;

ISR( TIMER3_COMPA_vect ) {
//  Do speed calcs and odom in here
  left_wheel_vel = (theta_e0 - prev_theta_e0)*hz;
  right_wheel_vel = (theta_e1 - prev_theta_e1)*hz;
  
  left_wheel_est = left_filter.updateEstimate(left_wheel_vel);
  right_wheel_est = right_filter.updateEstimate(right_wheel_vel);
  
  //If the wheels are not moving clip the values at zero
  if(left_wheel_vel == 0)left_wheel_est=0.0;
  if(right_wheel_vel == 0)right_wheel_est=0.0;
  
  Romi.update(left_wheel_vel, right_wheel_vel, millis());
  
  prev_theta_e0 = theta_e0;
  prev_theta_e1 = theta_e1;

}

void setup() {

  left_wheel.setMax(max_des_speed);
  right_wheel.setMax(max_des_speed);
  heading.setMax(1.0);

  
  pinMode(13, OUTPUT);
  Serial.begin( 9600 );

//  run a calibrationfor the light sensor
  l_sensor.calibrate();
  c_sensor.calibrate();
  r_sensor.calibrate();
  
  pinMode(6, OUTPUT);
  flash_leds(500);
  
    // wait for a second
  play_tone(6, 125);
  delay(500);
  analogWrite(6, 0);

  //setup timer for speed and odom timed calculations
  setupMotorPins();
  setupEncoder0();
  setupEncoder1();
  setupTimer3(hz);

  last_timestamp = millis();
  
}

//TODO: Implement the probablistic nature now for the robot with confidence values.

////N.B. e0 = left wheel, e1 = right wheel
// Remmeber, loop is called again and again.
void loop() 
{

  unsigned long time_now = millis();     

  unsigned long elapsed_time = time_now - last_timestamp;
  
//switch case logic for romi -> each state should only adjust power and direction of motors for keeping traceability
switch(state){
//Now that we have the sensor lets try to find the line
 
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
      play_tone(6, 125);
      delay(250);
      play_tone(6, 0);
      last_timestamp = millis();
    }
  break;
  case 1:
    m = weightedPower(l_sensor, c_sensor, r_sensor, min_power, max_power);

    if( elapsed_time >= 40 ) {
      last_timestamp = millis();    
      heading_output = heading.update(0.0, m);
      count++;
      if(count%2==0)
      {
//        right_output = right_wheel.update(heading_output*(0.5*max_des_speed), right_wheel_est);
//        left_output = left_wheel.update(-heading_output*(0.5*max_des_speed), left_wheel_est);
        right_output = right_wheel.update(heading_output*(0.45*max_des_speed), right_wheel_est);
        left_output = left_wheel.update(-heading_output*(0.45*max_des_speed), left_wheel_est);
        count = 0;
        if(heading_output >0.12)
        {
          r_direction = FORWARD;
          l_direction = REVERSE;
        }
        else if(heading_output < -0.12)
        {
          r_direction = REVERSE;
          l_direction = FORWARD;
        }
        else
        {
          r_direction = FORWARD;
          l_direction = FORWARD;
          right_output = max_des_speed/2;
          left_output= max_des_speed/2;
        }
      }
    }
    l_power = max(abs((left_output - y_int)/slope), min_power);
    r_power = max(abs((right_output - y_int)/slope), min_power);
    break;

////   Debug state!!!!!
//  case -1:
//  float left_output = left_wheel.update(max_des_speed-0.5, left_wheel_est);
//  float right_output = right_wheel.update(0.0, right_wheel_est);
////  Serial.print(left_output);
////  Serial.print(",");
//
//  l_power = abs((left_output - y_int)/slope);
//  r_power = abs((right_output - y_int)/slope);
//
//  if(left_output < 0)
//  {
//    l_direction = REVERSE;  
//  }
//  else
//  {
//    l_direction = FORWARD;
//  }
//    
//  if(right_output < 0)
//  {
//    r_direction = REVERSE;  
//  }
//  else
//  {
//    r_direction = FORWARD;
//  }
//  delay(50);
//  break;

/* Original BangBang Controller*/
//  case -2:
//      m = weightedPower(l_sensor, c_sensor, r_sensor, min_power, max_power,l_power, r_power);
//
//      if(m >0.15)
//      {
//        r_direction = FORWARD;
//        l_direction = REVERSE;
//      }
//      else if(m < -0.15)
//      {
//        r_direction = REVERSE;
//        l_direction = FORWARD;
//      }
//      else
//      {
//        r_direction = FORWARD;
//        l_direction = FORWARD;
//      }
//
//
//    l_power = max(min(l_power, max_power),min_power);
//    r_power = max(min(r_power, max_power),min_power);
//    break;

  case -3:
  /*Drive forward a bit so we can figure out if the kinematics are working alright*/
    if(elapsed_time >=500 && count < 8)
    {
    Serial.println("In here");
//      float forward_power = (3.14 - y_int)/slope;
  
      l_power = random(0,50);
      r_power = random(0,50);
      count++;
      last_timestamp = millis();
    }
    else if(elapsed_time >=500 && count >= 8)
    {
      l_power = 0;
      r_power = 0; 
    }
  break;
}
  // Send power PWM to pins, to motor drivers.
  digitalWrite( R_DIR_PIN, r_direction);
  digitalWrite( L_DIR_PIN, l_direction);
  analogWrite( L_PWM_PIN, l_power );
  analogWrite( R_PWM_PIN, r_power );
  Serial.print(Romi.getPose().x, 6);
  Serial.print(",");
  Serial.print(Romi.getPose().y, 6);
  Serial.print(",");
  Serial.println(Romi.getPose().theta, 6);
//  Serial.print(l_power);
//  Serial.print(",");
//  Serial.println(r_power);

}
