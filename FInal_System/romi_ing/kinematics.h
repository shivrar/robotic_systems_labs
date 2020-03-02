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
      
      Kinematics(float, float);
      void update(float, float, float);
      Pose2D getPose();
      void robotVelToWheelVels(float, float, float&, float&);
      float getRobotLinearX();
      float getRobotAngZ();
            
    private:
      //Private variables and methods go here
      Pose2D pose_encoder_;
      float last_update_time_;

      float wheel_separation_;
      float wheel_radius_;
      float linear_x_;
      float ang_z_;
  };
  
  
  // Required constructor.  Initialise variables.
  Kinematics::Kinematics(float wheel_separation, float wheel_radius)
  :wheel_separation_(wheel_separation),  wheel_radius_(wheel_radius)
  {
    last_update_time_ = 0.0;
    linear_x_ = 0.0;
    ang_z_ = 0.0;
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

    linear_x_ = (left_wheel_vel + right_wheel_vel)*wheel_radius_/2.0;
    ang_z_ = (right_wheel_vel - left_wheel_vel)* wheel_radius_/wheel_separation_;

    //Lets normalise the angle
    pose_encoder_.theta = fmod(pose_encoder_.theta ,2.0*M_PI);

    last_update_time_ = current_time;
    return;
  }

  Pose2D Kinematics::getPose()
  {
    return pose_encoder_;
  }

  void Kinematics::robotVelToWheelVels(float liner_x, float ang_z, float& wheel_speed_L, float& wheel_speed_R)
  {
    /* Given the robot linear x velocity and agular yaw velocity derive how much the wheel velocites are*/

    wheel_speed_L = (liner_x - (ang_z*wheel_separation_/2))/wheel_radius_ ;
    wheel_speed_R = (liner_x + (ang_z*wheel_separation_/2))/wheel_radius_ ;
    
  }

  float Kinematics::getRobotLinearX()
  {
    return linear_x_;
  }

  float Kinematics::getRobotAngZ()
  {
  return ang_z_;
  }

}

#endif
