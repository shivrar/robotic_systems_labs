#include "encoders.h"
#include "lineSensors.h"
#include "common_sys.h"
#include "pid.h"
#include "SimpleKalmanFilter.h"
#include "kinematics.h"

//~~~~~~~~~~~~~~~~~~TODO's~~~~~~~~~~~~~~~~~~~~~//
// TODO: Look at the idea of confidence for line following for the robot. 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

//float max_speed = 4.2; //Max reachable speed of the wheels -> output is 255
int max_power = 30; // ~5.06
int min_power = 20;
float max_des_speed = 1.5*M_PI;
static float max_ang_vel = 1.5*M_PI;
static float max_linear_vel = 0.08;

/*20= 2.02, 50 = 5.7, 63.75= 6.8, 100=11.0 , 152 = 17.2, 191.25 = 20.5, 235=24.96  ,255=26.84

speed = 0.1056*(PWM) + 0.287 is an alright linear approximation to convert pwm signal to speed values

*/

float slope = 0.1056, y_int = 0.287;

PID left_wheel( 1.3,  0.015, 0.00001);
PID right_wheel(1.3, 0.015,0.00001);
//PID heading(1.4,0.0,0.01);
//PID heading(1.0,0.0,0.035);
PID heading(0.61, 0.001,0.0001);
PID rth_heading(1.0,0.0,0.0);
PID rth_position(0.05, 0.0, 0.0);
//PID rth_heading2(0.05,0.001,1.0);

/*Passed between states*/
volatile byte l_power;
volatile byte r_power;
volatile bool l_direction;
volatile bool r_direction;
volatile float m;
unsigned long last_timestamp;
unsigned long beep_timestamp;

int count = 0;
//intialise the state
// NB: -1, -2, -3, -4 are a debuging state So use that accordingly
int state = 0;
bool isClose = false;
bool shouldBeep = true;
bool direction_chosen = false;
float current_rotation = 0.0;

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
        l_power = 0.0;
        r_power =0.0;
        r_direction = FORWARD;
        l_direction = FORWARD;
        right_wheel.reset();
        left_wheel.reset();
        rth_heading.reset();
        rth_position.reset();
        direction_chosen  = false;
        current_rotation = 0.0;
        last_timestamp = millis(); 
        beep_timestamp = millis();
        shouldBeep = true;
}

void setup() {

  left_wheel.setMax(max_des_speed);
  right_wheel.setMax(max_des_speed);
  heading.setMax(1.0);
  rth_heading.setMax(M_PI);
  rth_position.setMax(0.01);

  pinMode(13, OUTPUT);
  Serial.begin( 9600 );

//  run a calibrationfor the light sensor
  l_sensor.calibrate();
  c_sensor.calibrate();
  r_sensor.calibrate();
  
  pinMode(6, OUTPUT);
  flash_leds(500);
  
    // wait for a second
  analogWrite(6,100);
  delay(250);
  analogWrite(6,0);
  stateCleanup();

  //setup timer for speed and odom timed calculations
  setupMotorPins();
  setupEncoder0();
  setupEncoder1();
  setupTimer3(hz);

  last_timestamp = millis();
  beep_timestamp = millis();
  
}

void loop() 
{

unsigned long time_now = millis();     

unsigned long elapsed_time = time_now - last_timestamp;

unsigned long beep_time = time_now - beep_timestamp;
  
//switch case logic for romi -> each state should only adjust power and direction of motors for keeping traceability
switch(state){
//Now that we have the sensor lets try to find the line
//
//  case -4:
//  {
//      if(elapsed_time >=2000)
//      {
//        count++;
//        l_power = 0.0;
//        l_direction = REVERSE;
////        r_direction = FORWARD;
//        last_timestamp = millis();    
//      }
//      if(beep_time >=100)
//      {
//        float r_speed = (count%2 ==0) ? M_PI : -M_PI;
//        float right_output = right_wheel.update(r_speed, right_wheel_est);
//
//        r_direction = (right_output >= 0)? FORWARD: REVERSE;
//        
//        r_power = (byte)abs((right_output - y_int)/slope); //~2.20
//        Serial.print(right_wheel_est);
//        Serial.print(",");
//        Serial.print(r_speed);
//        Serial.print(",");
//        Serial.println(right_wheel_vel);
//      }
//  }
//  break;
  case 0:
  {
    if((l_sensor.readCalibrated()+ c_sensor.readCalibrated() + r_sensor.readCalibrated())/3 < 100)
    {
      l_power = min(l_power + 5, max_power-5);
      l_direction = FORWARD;
      r_power = min(r_power + 5, max_power-5);
      r_direction = FORWARD;
    }
    else
    {
      l_power = 0;
      r_power = 0;
    }
    if(confidence >=0.4)
    {
      if(shouldBeep)
      {
        shouldBeep = false;
        beep_timestamp = millis();
        analogWrite(6,100);
        digitalWrite(13, HIGH);
        beep_time = 0;  
      }
      if(beep_time >=500)
      {
        analogWrite(6,0);
        digitalWrite(13, LOW);    
        stateCleanup();
        state = 1;
        break;  
      }  
    }
  }
  break;
  
  case 1:
  {
    m = weightedPower(l_sensor, c_sensor, r_sensor, min_power, max_power);
    if( elapsed_time >= 50) 
    {
      float heading_output = 0.0;
      float right_output = 0.0;
      float left_output = 0.0;
      float forward_vel = 0.0;
      float right_vel, left_vel;
      float ang_vel = 0.0;
      last_timestamp = millis();    
      heading_output = heading.update(0.0, m);
      count++;
      if(count%2==0)
      {

          forward_vel = map(confidence, -1.0, 1.0, 0.0, 1.0)*max_linear_vel;
          ang_vel = heading_output*max_ang_vel;
          Romi.robotVelToWheelVels(forward_vel, ang_vel, left_vel, right_vel);
//        right_output = right_wheel.update(right_wheel_vel, right_wheel_est);
//        left_output = left_wheel.update(left_wheel_vel, left_wheel_est);
//
//        left_output= -heading_output*(max_des_speed);
//        right_output = heading_output*(max_des_speed);
        left_output = left_vel;
        right_output = right_vel;
        count = 0;

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
        
        if(abs((left_output - y_int)/slope) > max_power)
        {
          l_power = (byte) max_power;
        }
        else 
        {
          l_power = (byte)abs((left_output - y_int)/slope);
        }

        if(abs((right_output - y_int)/slope)> max_power)
        {
          r_power = (byte) max_power;
        }
        else
        {
          r_power = (byte)abs((right_output - y_int)/slope);
        }
      }

    }    
    if(confidence <=-1.0)
    {
      l_power = 0;
      r_power = 0;
      if(shouldBeep)
      {
        shouldBeep = false;
        beep_timestamp = millis();
        analogWrite(6,100);
        digitalWrite(13, HIGH);
        beep_time = 0;  
      }
      if(beep_time >=1000)
      {
        analogWrite(6,0);
        digitalWrite(13, LOW);    
        stateCleanup();
        state = 4;
        break;  
      } 
    }
    }
    break;

    case 2:
    {
    /*Return to home - 
      First lets look at home
    */
      if( elapsed_time >= 25 )
      {
        float right_output = 0.0;
        float left_output = 0.0;
        float alpha = acos((Romi.getPose().x*cos(Romi.getPose().theta) + Romi.getPose().y*sin(Romi.getPose().theta))/sqrt(square(Romi.getPose().x) + square(Romi.getPose().y)));
        float home_heading = ((Romi.getPose().theta>=0 && Romi.getPose().theta<=M_PI)  || (Romi.getPose().theta<=-M_PI && Romi.getPose().theta<=0) ) ? M_PI - alpha : alpha - M_PI;
//        float ang_vel = max(min((home_heading- Romi.getPose().theta)/(0.025), max_ang_vel), -max_ang_vel);
        if(!direction_chosen)
        {
          direction_chosen = true;
          current_rotation = (home_heading >=0.0) ? 1.0:-1.0;
        }
        float head_tol = M_PI/180.0;
        last_timestamp = millis();   
        count++;
        
        if(count%2==0)
        {
          count = 0;
          if(home_heading > head_tol || home_heading < -head_tol)
          {
              float right_wheel_speed, left_wheel_speed;
              Romi.robotVelToWheelVels(0.0, 0.25*(current_rotation)*max_ang_vel, left_wheel_speed, right_wheel_speed);
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
              state = 4;
              break;
          }
        }
        l_power = min(max(abs((left_output - y_int)/slope), min_power-3), max_power);
        r_power = min(max(abs((right_output - y_int)/slope), min_power-3), max_power);
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
        float ang_vel = rth_heading.update(0.0, home_heading);
//        float lin_vel = 0.05;
        if( !isClose  && abs_distance < 0.2)
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
          // BAng Bang RTH could do some cleaner logic but a working thing right right now
          if(ang_vel>0.125 && !isClose)
          {
            Romi.robotVelToWheelVels(max_linear_vel, max_ang_vel, left_wheel_speed, right_wheel_speed);
          }
          else if(ang_vel<-0.125 && !isClose)
          {
            Romi.robotVelToWheelVels(max_linear_vel,-max_ang_vel, left_wheel_speed, right_wheel_speed);
          }
          else
          {
            Romi.robotVelToWheelVels(max_linear_vel, 0.0, left_wheel_speed, right_wheel_speed);
          }
//          right_output = right_wheel.update(right_wheel_speed, right_wheel_est);
//          left_output = left_wheel.update(left_wheel_speed, left_wheel_est);
          right_output = right_wheel_speed;
          left_output = left_wheel_speed;
          count = 0;
          if(abs_distance > 0.01 && Romi.getPose().x >= 0)
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
              break;
          }
        }
        l_power = min(max(abs((left_output - y_int)/slope), min_power), max_power);
        r_power = min(max(abs((right_output - y_int)/slope), min_power), max_power);
      }
    }
    break;
    
    case 4:
    {
        l_power = 0.0;
        r_power = 0.0;
        Serial.print(Romi.getPose().x);
        Serial.print(",");
        Serial.println(Romi.getPose().y);
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
/*Original directed wheel control*/
//case 1:
//{
//  m = weightedPower(l_sensor, c_sensor, r_sensor, min_power, max_power);
//  if( elapsed_time >= 10) 
//  {
//    float heading_output = 0.0;
//    float right_output = 0.0;
//    float left_output = 0.0;
//    last_timestamp = millis();    
//    heading_output = heading.update(0.0, m);
//    count++;
//    if(count%2==0)
//    {
////        right_output = right_wheel.update(heading_output*(max_des_speed), right_wheel_est);
////        left_output = left_wheel.update(-heading_output*(max_des_speed), left_wheel_est);
//      left_output= -heading_output*(max_des_speed);
//      right_output = heading_output*(max_des_speed);
//      count = 0;
//      if(heading_output >0.35)
//      {
//        r_direction = FORWARD;
//        l_direction = REVERSE;
//        left_output = 0.5*left_output;
//      }
//      else if(heading_output < -0.35)
//      {
//        r_direction = REVERSE;
//        l_direction = FORWARD;
//        right_output = 0.5*right_output;
//      }
//      else
//      {
//        r_direction = FORWARD;
//        l_direction = FORWARD;
//        right_output = map(confidence, -1.0, 1.0, 0.0, 1.0)*max_des_speed/2.0;
//        left_output= map(confidence, -1.0, 1.0, 0.0, 1.0)*max_des_speed/2.0;
//      }
//      if(abs((left_output - y_int)/slope) > max_power)
//      {
//        l_power = (byte) max_power;
//      }
//      else 
//      {
//        l_power = (byte)abs((left_output - y_int)/slope);
//      }
//
//      if(abs((right_output - y_int)/slope)> max_power)
//      {
//        r_power = (byte) max_power;
//      }
//      else
//      {
//        r_power = (byte)abs((right_output - y_int)/slope);
//      }
//    }
//
//  }    
//  if(confidence <=-1.0)
//  {
//    l_power = 0;
//    r_power = 0;
//    if(shouldBeep)
//    {
//      shouldBeep = false;
//      beep_timestamp = millis();
//      analogWrite(6,200);
//      digitalWrite(13, HIGH);
//      beep_time = 0;  
//    }
//    if(beep_time >=2000)
//    {
//      analogWrite(6,0);
//      digitalWrite(13, LOW);    
//      stateCleanup();
//      state = 4;
//      break;  
//    } 
//  }
//  }
//  break;
