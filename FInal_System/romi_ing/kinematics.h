#ifndef _Kinematics
#define _Kinematics_h

//You may want to use some/all of these variables
//const float wheel_radius    = ??;
//const float WHEEL_RADIUS      = ??;
//const float WHEEL_SEPERATION  = ??;
//const float GEAR_RATIO        = ??;
//const float COUNTS_PER_SHAFT_REVOLUTION = ??;
//const float COUNTS_PER_WHEEL_REVOLUTION =  ??;
//const float COUNTS_PER_MM               = ??;

namespace Kinematics2D{
  /*Helper struct to help store 2D pose of the robot*/
  struct Pose2D
  {
    float x;
    float y;
    float theta;

    Pose2D(float x_init=0.0, float y_init=0.0, float theta_init=0.0):x(x_init), 
    y(y_init), theta(theta_init)
    {
      
    }
    
  };

  class Kinematics
  {
    public:
      
      Kinematics(float, float);   // Constructor, required.
  
      // Write your method functions:
      // ...
      void update(float, float, float);
      Pose2D getPose();
            
    private:
      //Private variables and methods go here
      Pose2D pose_encoder_;
      float last_update_time_;

      float wheel_separation_;
      float wheel_radius_;
      float last_l_vel_;
      float last_r_vel_;
  };
  
  
  // Required constructor.  Initialise variables.
  Kinematics::Kinematics(float wheel_separation, float wheel_radius)
  :wheel_separation_(wheel_separation),  wheel_radius_(wheel_radius)
  {
  
  
  }
  
  void Kinematics::update(float left_wheel_vel, float right_wheel_vel, float current_time) {

    float delta_time = (current_time - this->last_update_time_)/1000.0; //Assuption millis isbeing used for the timesteps
    
    // Distance turned by wheels
    float sl =  (left_wheel_vel * delta_time)*this->wheel_radius_;
    float sr =  (right_wheel_vel * delta_time)*this->wheel_radius_; 

    float ssum = sl + sr;
    float sdiff = sr - sl;

    float dx = ( ssum ) /2.0 * cos ( pose_encoder_.theta + ( sdiff ) / ( 2.0*wheel_separation_ ) );
    float dy = ( ssum ) /2.0 * sin ( pose_encoder_.theta + ( sdiff ) / ( 2.0*wheel_separation_ ) );
    float dtheta =  ( sdiff ) /wheel_separation_;


    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    last_update_time_ = current_time;
    return;
  }

  Pose2D Kinematics::getPose()
  {
    return pose_encoder_;
  }

}

#endif
