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
int max_power = 40; // ~5.06
int min_power = 20;
float max_des_speed = 2.0*M_PI;
static float max_ang_vel = 2.5*M_PI;
static float max_linear_vel = 0.12;

/*20= 2.02, 50 = 5.7, 63.75= 6.8, 100=11.0 , 152 = 17.2, 191.25 = 20.5, 235=24.96  ,255=26.84

speed = 0.1056*(PWM) + 0.287 is an alright linear approximation to convert pwm signal to speed values

*/

float slope = 0.1056, y_int = 0.287;

PID left_wheel( 1.0,  0.02, 0.0000);
PID right_wheel(1.0,  0.02, 0.0000);
//PID heading(1.4,0.0,0.01);
//PID heading(1.0,0.0,0.035);
//PID heading(0.42, 0.005,0.00006);
PID heading(0.45, 0.001,0.004);
PID rth_heading(1.0,0.0,0.0);
PID rth_position(0.05, 0.0, 0.0);
//PID rth_heading2(0.05,0.001,1.0);

/*Passed between states*/
volatile byte l_power;
volatile byte r_power;
volatile bool l_direction;
volatile bool r_direction;
unsigned long last_timestamp;
unsigned long beep_timestamp;
float distance_from_home = 0.0;

int count = 0;
//intialise the state
// NB: -1, -2, -3, -4 are a debuging state So use that accordingly
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int state = 0;
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
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
  if(state == 0 || state ==1 || state == 2){
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
  case 0:
  {
    if((l_sensor.readCalibrated()+ c_sensor.readCalibrated() + r_sensor.readCalibrated())/3 < 100)
    {
      if(elapsed_time >= 50)
      {
        last_timestamp = millis();
        float right_output, left_output;
        right_output = right_wheel.update(max_des_speed, right_wheel_est);
        left_output = left_wheel.update(max_des_speed, left_wheel_est);
        l_direction = FORWARD;
        r_direction = FORWARD;
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
    else
    {
      l_power = 0;
      r_power = 0;
    }
    if(confidence >=0.0)
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
    float m = weightedPower(l_sensor, c_sensor, r_sensor, min_power, max_power);
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

          forward_vel = float_map(confidence, -1.0, 1.0, 0.15, 0.6)*max_linear_vel;
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
    if(confidence <=-0.8)
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
        state = 2;
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
      if( elapsed_time >= 20 )
      {
//      /*BUG HERE!!!!!!! LOOK AT THIS PROPERLY!!!!!!!!*/
        float right_output = 0.0;
        float left_output = 0.0;
        float alpha = acos((Romi.getPose().x*cos(Romi.getPose().theta) + Romi.getPose().y*sin(Romi.getPose().theta))/sqrt(square(Romi.getPose().x) + square(Romi.getPose().y)));
        float home_heading = ((Romi.getPose().theta>=0 && Romi.getPose().theta<=M_PI)  || (Romi.getPose().theta<=-M_PI && Romi.getPose().theta<=0) ) ? M_PI - alpha : alpha - M_PI;
        float ang_vel = (home_heading > 0)? 0.2*max_ang_vel : -0.2*max_ang_vel ;
//        float ang_vel = (home_heading > 0)? float_map(abs(home_heading), 0.0, M_PI, 0.3, 0.6)*max_ang_vel : -float_map(abs(home_heading), 0.0, M_PI, 0.3, 0.6)*max_ang_vel ;
        float head_tol = M_PI/30.0;
//        if(isClose)
//        {
//          ang_vel = (home_heading > 0)? 0.2*max_ang_vel : -0.2*max_ang_vel ;
//          head_tol = M_PI/90.0;
//        }
        
        last_timestamp = millis();   
        count++;
        
        if(count%2==0)
        {
          count = 0;
          if(home_heading > head_tol || home_heading < -head_tol)
          {
              float right_vel, left_vel;
              Romi.robotVelToWheelVels(0.0, ang_vel, left_vel, right_vel);
//              // Don't Actually need the PID xD just want the romi to turn until it sees the heding
              right_output = right_vel;
              left_output = left_vel;             
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
              if(abs((left_output - y_int)/slope) > min_power)
              {
                l_power = (byte) min_power;
              }
              else 
              {
                l_power = (byte)abs((left_output - y_int)/slope);
              }
      
              if(abs((right_output - y_int)/slope)> min_power)
              {
                r_power = (byte) min_power;
              }
              else
              {
                r_power = (byte)abs((right_output - y_int)/slope);
              }
              Serial.print(home_heading,6);
              Serial.print(",");
              Serial.println(Romi.getPose().theta,6);
          }
          else
          {
              distance_from_home = sqrt(square(Romi.getPose().x) + square(Romi.getPose().y));
              stateCleanup();
              state = 4;
              break;
          }
        }
      }
    }
    break;
    /*TODO: Re=implement this entire logic once Iv'e tested the rotation state*/
    case 3:
    {
      
      if( elapsed_time >= 50)
      {
        float abs_distance = sqrt(square(Romi.getPose().x) + square(Romi.getPose().y));
        float right_output = 0.0;
        float left_output = 0.0;
        float alpha = acos((Romi.getPose().x*cos(Romi.getPose().theta) + Romi.getPose().y*sin(Romi.getPose().theta))/sqrt(square(Romi.getPose().x) + square(Romi.getPose().y)));
        float home_heading = ((Romi.getPose().theta>=0 && Romi.getPose().theta<=M_PI)  || (Romi.getPose().theta<=-M_PI && Romi.getPose().theta<=0) ) ? M_PI - alpha : alpha - M_PI;
        float ang_vel = (home_heading > 0)? float_map(abs(home_heading), 0.0, M_PI, 0.0, 0.5)*max_ang_vel : -float_map(abs(home_heading), 0.0, M_PI, 0.0, 0.5)*max_ang_vel ;
//        float head_tol = M_PI/90.0;
        float lin_vel = float_map(abs_distance, 0.0, distance_from_home, 0.5*max_linear_vel,max_linear_vel);
        last_timestamp = millis();    
        count++;
        if(count%2==0)
        {
          float right_vel, left_vel;
          // BAng Bang RTH could do some cleaner logic but a working thing right right now
//          if(home_heading > head_tol || home_heading < -head_tol)
//          {
            Romi.robotVelToWheelVels(lin_vel, ang_vel, left_vel, right_vel);
//          }
//          else
//          {
//            Romi.robotVelToWheelVels(lin_vel, 0.0, left_vel, right_vel);
//          }
//          right_output = right_wheel.update(right_vel, right_wheel_est);
//          left_output = left_wheel.update(left_vel, left_wheel_est);
          right_output = right_vel;
          left_output = left_vel;
          count = 0;
          if(abs_distance > 0.03 && Romi.getPose().x >= 0)
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
          else
          {
              stateCleanup();
              state = 4;
              break;
          }
        }
      }
    }
    break;
    case 4:
    {
        l_power = 0.0;
        r_power = 0.0;
        Serial.print(Romi.getPose().x,6);
        Serial.print(",");
        Serial.print(Romi.getPose().y, 6);
        Serial.print(",");
        Serial.println(Romi.getPose().theta,6);
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
