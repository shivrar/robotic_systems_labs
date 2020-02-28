#include "encoders.h"
#include "lineSensors.h"
#include "common_sys.h"
#include "pid.h"
#include "SimpleKalmanFilter.h"
#include "kinematics.h"

//~~~~~~~~~~~~~~~~~~TODO's~~~~~~~~~~~~~~~~~~~~~//
// TODO: Look at the idea of confidence for line following for the robot. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

float max_speed = 25.84; //Max reachable speed of the wheels -> output is 255
int max_power = 50; // ~5.06
int min_power = 15;
float max_des_speed = 7.2;

/*20= 2.02, 50 = 5.7, 63.75= 6.8, 100=11.0 , 152 = 17.2, 191.25 = 20.5, 235=24.96  ,255=26.84

speed = 0.1056*(PWM) + 0.287 is an alright linear approximation to convert pwm signal to speed values

*/

float slope = 0.1056, y_int = 0.287;

//
//PID left_wheel(1.1, 0.085,0.1);
//PID right_wheel(1.1, 0.085,0.1);
//PID left_wheel(1.0, 0.1,0);
//PID right_wheel(1.0, 0.1,0);
PID left_wheel(0.3, 0.0000,1.1);
PID right_wheel(0.3, 0.0000,1.1);
PID heading(0.3,0.0,0.9);
PID rth_heading(0.05,0.0,0.0);

/*Passed between states*/
volatile byte l_power;
volatile byte r_power;
volatile bool l_direction;
volatile bool r_direction;
volatile float m;
unsigned long last_timestamp;
int count = 0;
//intialise the state
// NB: -1, -2, -3 are a debuging state So use that accordingly
int state = 0;
bool isClose = false;

/*Sensors*/
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
volatile double confidence = -1.0;

//Control loop timers
Kinematics2D::Kinematics Romi(0.14, 0.035);

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
  if(state == 0 || state ==1){
    if((l_sensor.readCalibrated()+ c_sensor.readCalibrated() + r_sensor.readCalibrated())/3 < 100)
    {
      confidence -= 0.004;
    }
    else
    {
      confidence += 0.004;
    }
    confidence = max(min(1, confidence), -1); 
  }
}

void stateCleanup(void)
{
        r_direction = FORWARD;
        l_direction = FORWARD;
        right_wheel.reset();
        left_wheel.reset();
        rth_heading.reset();
        rth_heading.reset(); 
        l_power = 0.0;
        r_power =0.0;
        last_timestamp = millis(); 
}

void setup() {

  left_wheel.setMax(max_des_speed);
  right_wheel.setMax(max_des_speed);
  heading.setMax(1.0);
  rth_heading.setMax(M_PI);

  
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

void loop() 
{

unsigned long time_now = millis();     

unsigned long elapsed_time = time_now - last_timestamp;
  
//switch case logic for romi -> each state should only adjust power and direction of motors for keeping traceability
switch(state){
//Now that we have the sensor lets try to find the line
 
  case 0:
  {
    if(l_sensor.readCalibrated()<300 && c_sensor.readCalibrated()<300 && r_sensor.readCalibrated()< 300)
    {
      l_power = min(l_power + 5, min_power);
      l_direction = FORWARD;
      r_power = min(r_power + 5, min_power);
      r_direction = FORWARD;
    }
    else
    {
      l_power = 0;
      r_power = 0;
    }
    if(confidence >=-0.2)
    {
      stateCleanup();
      state = 1;
    }
  }
  break;
  case 1:
  {
    m = weightedPower(l_sensor, c_sensor, r_sensor, min_power, max_power);
    float heading_output = 0.0;
    float right_output = 0.0;
    float left_output = 0.0;
    if( elapsed_time >= 50) 
    {
      last_timestamp = millis();    
      heading_output = heading.update(0.0, m);
      count++;
      if(count%2==0)
      {
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
          right_output = max_des_speed/3;
          left_output= max_des_speed/3;
        }
      }
    }
    l_power = max(abs((left_output - y_int)/slope), min_power);
    r_power = max(abs((right_output - y_int)/slope), min_power);
    
    if(confidence <=-1.0)
    {
      stateCleanup();
      state = 2;
      break;  
    }
    }
    break;
    case 2:
    {
    /*Return to home - 
      First lets look at home
    */
      if( elapsed_time >= 20 )
      {
        float right_output = 0.0;
        float left_output = 0.0;
        float alpha = acos((Romi.getPose().x*cos(Romi.getPose().theta) + Romi.getPose().y*sin(Romi.getPose().theta))/sqrt(square(Romi.getPose().x) + square(Romi.getPose().y)));
        float home_heading = ((Romi.getPose().theta>=0 && Romi.getPose().theta<=M_PI)  || (Romi.getPose().theta<=-M_PI && Romi.getPose().theta<=0) ) ? M_PI - alpha : alpha - M_PI;
        float ang_vel = rth_heading.update(0.0, -home_heading)/0.1;
        float head_tol = 0.1;
        if(isClose)
        {
          head_tol = 0.06;
        }
        
        last_timestamp = millis();   
        count++;
        
        if(count%2==0)
        {
          count = 0;
          if(home_heading > head_tol || home_heading < -head_tol)
          {
              float right_wheel_speed, left_wheel_speed;
              Romi.robotVelToWheelVels(0.0, ang_vel, left_wheel_speed, right_wheel_speed);
              // Don't Actually need the PID xD just want the romi to turn until it sees the heding
              right_output = right_wheel_speed;
              left_output = left_wheel_speed;             
              if(left_output < 0)
              {
                l_direction = REVERSE;  
              }
              else
              {
                l_direction = FORWARD;
              }
                
              if(right_output < 0)
              {
                r_direction = REVERSE;  
              }
              else
              {
                r_direction = FORWARD;
              }
          }
          else
          {
              stateCleanup();
              state = 3;
          }
        }
        l_power = max(abs((left_output - y_int)/slope), min_power);
        r_power = max(abs((right_output - y_int)/slope), min_power);
//        Serial.print(head_tol,6);
//        Serial.print(",");
//        Serial.print(-head_tol,6);
//        Serial.print(",");
//        Serial.println(home_heading,6);
      }
    }
    break;
    case 3:
    {
      if( elapsed_time >= 50)
      {
        float right_output = 0.0;
        float left_output = 0.0;
        float alpha = acos((Romi.getPose().x*cos(Romi.getPose().theta) + Romi.getPose().y*sin(Romi.getPose().theta))/sqrt(square(Romi.getPose().x) + square(Romi.getPose().y)));
        float abs_distance = sqrt(square(Romi.getPose().x) + square(Romi.getPose().y));
        float home_heading = ((Romi.getPose().theta>=0 && Romi.getPose().theta<=M_PI)  || (Romi.getPose().theta<=-M_PI && Romi.getPose().theta<=0) ) ? M_PI - alpha : alpha - M_PI;
        float ang_vel = rth_heading.update(0.0, -home_heading);        

        if( !isClose  && abs_distance < 0.1)
        {
          // Re-orient when we are close
          isClose = true;
          stateCleanup();
          state = 2;
          break;
        }
        
        last_timestamp = millis();    
        count++;
        if(count%2==0)
        {
          float right_wheel_speed, left_wheel_speed;
          Romi.robotVelToWheelVels(0.1, ang_vel, left_wheel_speed, right_wheel_speed);
//          right_output = right_wheel.update(right_wheel_speed, right_wheel_est);
//          left_output = left_wheel.update(left_wheel_speed, left_wheel_est);
          right_output = right_wheel_speed;
          left_output = left_wheel_speed;
          count = 0;
          if(abs_distance > 0.01)
          {
              if(left_output < 0)
              {
                l_direction = REVERSE;  
              }
              else
              {
                l_direction = FORWARD;
              }
                
              if(right_output < 0)
              {
                r_direction = REVERSE;  
              }
              else
              {
                r_direction = FORWARD;
              }
          }
          else
          {
              stateCleanup();
              state = 4;
          }
        }
        l_power = max(abs((left_output - y_int)/slope), min_power);
        r_power = max(abs((right_output - y_int)/slope), min_power);

      }
    }
    break;
    case 4:
    {
        l_power = 0.0;
        r_power = 0.0;
        // Finished
    }
    break;
}
  // Send power PWM to pins, to motor drivers.
  digitalWrite( R_DIR_PIN, r_direction);
  digitalWrite( L_DIR_PIN, l_direction);
  analogWrite( L_PWM_PIN, l_power );
  analogWrite( R_PWM_PIN, r_power );

//Serial.print(confidence);
//Serial.print(",");
//Serial.println(state);
}

/*Test states Put them as needed back into the original code*/
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

    
//    case -3:
//    {
//    /*Drive forward a bit so we can figure out if the kinematics are working alright*/
//      if(elapsed_time >=500 && count < 8)
//      {
//        l_power = random(0,50);
//        r_power = random(0,50);
//        count++;
//        last_timestamp = millis();
//      }
//      else if(elapsed_time >=500 && count >= 8)
//      {
//        l_power = 0;
//        r_power = 0; 
//        state = 2;
//      }
//      break;
//    }

//    case -3:
//    {
//    /*Drive forward a bit so we can figure out if the kinematics are working alright*/
//      if(elapsed_time >=500 && count < 8)
//      {
//        l_power = random(0,50);
//        r_power = random(0,50);
////        l_power = 25;
////        r_power = 25;
//        count++;
//        last_timestamp = millis();
//      }
//      else if(elapsed_time >=500 && count >= 8)
//      {
//        stateCleanup();
//        state = 2;
//      }
//      break;
//    }
