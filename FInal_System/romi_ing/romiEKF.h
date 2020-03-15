#ifndef _RomiEKF_h
#define _RomiEKF_h

//You may want to use some/all of these variables
//const float wheel_radius    = ??;
//const float WHEEL_RADIUS      = ??;
//const float WHEEL_SEPERATION  = ??;
//const float GEAR_RATIO        = ??;
//const float COUNTS_PER_SHAFT_REVOLUTION = ??;
//const float COUNTS_PER_WHEEL_REVOLUTION =  ??;
//const float COUNTS_PER_MM               = ??;

#include <BasicLinearAlgebra.h>
//Forgive me but these matrix calculations would drive me crazy to do manually

namespace RomiEKF{

  class ekf
  {
    public:
      
      ekf(const float * , const float *);
      void ekfStep(const float*, const float*, float);
      float getStateVar(int);
      void setQ(int, int, float);
      void setP(int, int, float);
      void setR(int, int, float); 
            
    private:
//      float priori_prediction_[3]; // -> hard coded for 3 states, x,y,theta
//      float x_[3]; // -> current state
//      float f_x_[3]; // -> process model
//      float h_x_[4]; // -> obs model
//      float A_k_[3][3]; //-> jacobian for the process model
//      float H_k_[4][3]; //-> jaobian for the obs model
//      float K_[3][4]; //-> Kalman Gain Matrix

      BLA::Matrix<3> x_;
      BLA::Matrix<3> priori_prediction_;
      BLA::Matrix<3,3> A_k_;
      BLA::Matrix<3,4> K_;
      BLA::Matrix<4,3> H_k_;
      BLA::Matrix<3,3> P_;
      BLA::Matrix<3,3> Q_;
      BLA::Matrix<4,4> R_;
//      BLA::Matrix<4> Z_k_;
      BLA::Matrix<4,4> Eye4_;
      BLA::Matrix<3,3> Eye3_;
      

//      // Error matricies here
//      float P_[3][3];  /* prediction error covariance */
//      float Q_[3][3];  /* process noise covariance */
//      float R_[4][4];  /* measurement error covariance */  


      float last_update_time_;

      };
  
  // Required constructor.  Initialise variables.
  ekf::ekf(const float *intial_x, const float *intial_p)
  {
    x_(0) = intial_x[0];
    x_(1) = intial_x[0];
    x_(2) = intial_x[0];

    this->priori_prediction_(0) = intial_x[0];
    this->priori_prediction_(1) = intial_x[0];
    this->priori_prediction_(2) = intial_x[0];

    K_.Fill(0);
    P_.Fill(0);
    R_.Fill(0);
    Q_.Fill(0);
    A_k_.Fill(0);
//    Z_k_.Fill(0);
    A_k_ <<   1,0,0,
              0,1,0,
              0,0,1;
              
    H_k_ << 1,0,0,
              0,1,0,
              0,0,1,
              0,0,1;

    // Assuming guassian noise that are independent of each other
    this->setP(0,0, intial_p[0]);
    this->setP(1,1, intial_p[1]);
    this->setP(2,2, intial_p[2]);

    // We approximate the process noise using a small constant
    this->setQ(0, 0, .00001);
    this->setQ(1, 1, .00001);
    this->setQ(2, 2, .00001);


    // Same for measurement noise
    this->setR(0, 0, .00001);
    this->setR(1, 1, .00001);
    this->setR(2, 2, .00001);
    this->setR(3, 3, .00005);
  }

    /**
   * Sets the specified value of the prediction error covariance. <i>P<sub>i,j</sub> = value</i>
   * @param i row index
   * @param j column index
   * @param value value to set
   */
  void ekf::setP(int i, int j, float value) 
  { 
      this->P_(i,j) = value; 
  }

  /**
   * Sets the specified value of the process noise covariance. <i>Q<sub>i,j</sub> = value</i>
   * @param i row index
   * @param j column index
   * @param value value to set
   */
  void ekf::setQ(int i, int j, float value) 
  { 
      this->Q_(i,j) = value; 
  }

  /**
   * Sets the specified value of the observation noise covariance. <i>R<sub>i,j</sub> = value</i>
   * @param i row index
   * @param j column index
   * @param value value to set
   */
  void ekf::setR(int i, int j, float value) 
  { 
      this->R_(i,j) = value; 
  }

  void ekf::ekfStep(const float *z_k, const float *B_k, float current_time)
  {
    float delta_time = (current_time - this->last_update_time_)/1000.0; //Assuption millis isbeing used for the timesteps
    // Using the current measurment and input command, step the system and update
    // Compute the Kalman gain
    /*
    K =
 
      [ p1/(p1 + r1),            0,                                                                     0,                                                                     0]
      [            0, p2/(p2 + r2),                                                                     0,                                                                     0]
      [            0,            0, (p3*(p3 + r4))/(p3*r3 + p3*r4 + r3*r4) - p3^2/(p3*r3 + p3*r4 + r3*r4), (p3*(p3 + r3))/(p3*r3 + p3*r4 + r3*r4) - p3^2/(p3*r3 + p3*r4 + r3*r4)]
//    */
//    K_[0][0] = P_[0][0]/(P_[0][0] + R_[0][0]);
//    K_[1][1] = P_[1][1]/(P_[1][1] + R_[1][1]);
//    K_[2][2] = (P_[2][2]*(P_[2][2] + R_[3][3])/(P_[2][2]*R_[2][2] + P_[2][2]*R_[3][3] + R_[2][2]*R_[3][3])) - square((P_[2][2]))/(P_[2][2]*R_[2][2] + P_[2][2]*R_[3][3] + R_[2][2]*R_[3][3]);
//    K_[2][3] = (P_[2][2]*(P_[2][2] + R_[2][2])/(P_[2][2]*R_[2][2] + P_[2][2]*R_[3][3] + R_[2][2]*R_[3][3])) - square((P_[2][2]))/(P_[2][2]*R_[2][2] + P_[2][2]*R_[3][3] + R_[2][2]*R_[3][3]);
//      Multiply((A_k_),(A_k_), P_);
      //P_*(~H_k_);

      BLA::Matrix<4> Z_k_;
      BLA::Matrix<4> Z_k_priori;
      Z_k_(0) = z_k[0];
      Z_k_(1) = z_k[1];
      Z_k_(2) = z_k[2];
      Z_k_(3) = z_k[3];

      Z_k_priori(0) = priori_prediction_(0);
      Z_k_priori(1) = priori_prediction_(1);
      Z_k_priori(2) = priori_prediction_(2);
      Z_k_priori(3) = priori_prediction_(2);
      
      K_ = P_*(~H_k_)*((H_k_*P_*(~H_k_) + R_).Inverse());

      x_ = priori_prediction_ + K_*(Z_k_  - Z_k_priori);

      P_ = (Eye3_ - K_*H_k_)*P_;

      float dx = delta_time*B_k[0]*cos(x_(2));
      float dy = delta_time*B_k[0]*sin(x_(2));
  
      priori_prediction_(0) = x_(0) + dx;
      priori_prediction_(1) = x_(1) + dy;
      priori_prediction_(1) = x_(2) + delta_time*B_k[1];;

      A_k_(0,2) = -dy;
      A_k_(1,2) = dx;

      P_ = A_k_*P_*(~A_k_) + Q_;
      
//    //Update measurement
//    /*
//   
//    */
//    x_[0] = priori_prediction_[0] + K_[0][0]*(z_k[0] - priori_prediction_[0]);
//    x_[1] = priori_prediction_[1] + K_[1][1]*(z_k[1] - priori_prediction_[1]);
//    x_[2] = priori_prediction_[0] + K_[2][2]*(z_k[2] - priori_prediction_[2]) + K_[2][3]*(z_k[2] - priori_prediction_[2]);
//    
//    P_[0][0] = P_[0][0]*(1 - K_[0][0]);
//    P_[1][1] = P_[1][1]*(1 - K_[1][1]);
//    P_[2][2] = P_[2][2]*(1 - (K_[2][2] + K_[2][3]));
//
//    float dx = delta_time*B_k[0]*cos(x_[2]);
//    float dy = delta_time*B_k[0]*sin(x_[2]);
//    
//    // predict new states
//    priori_prediction_[0] = x_[0] + dx;
//    priori_prediction_[1] = x_[1] + dy;
//    priori_prediction_[2] = x_[2] + delta_time*B_k[1];

    last_update_time_ = current_time;
    return;
  };

  float ekf::getStateVar(int index)
  {
    return this->x_(index);  
  };


}

#endif
