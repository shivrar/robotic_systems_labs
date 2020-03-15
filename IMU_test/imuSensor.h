#ifndef _IMU_h
#define _IMU_h
#include <LSM6.h>

//Number of readings to take for calibration
//const int NUM_CALIBRATIONS = ????;

/* 
 *  Class to represent a single line sensor
 */
class Gyro
{
  public:

    Gyro(void)
    {
      
    }

    // Suggested functions.
    void calibrate();       //Calibrate
    int readRaw();         //Return the uncalibrated value from the sensor
    int readCalibrated();  //Return the calibrated value from the sensor

    void enable(void);

    int getBias();
    // You may wish to add other functions!
    // ...
    
  private:
  
    LSM6 imu_;
    int bias_;
    /*
     * Add any variables needed for calibration here
     */
    
};


// Returns unmodified reading.
int Gyro::readRaw()
{
  this->imu_.read();
  return this->imu_.g.z;
}

void Gyro::enable(void)
{
  if (!imu_.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu_.enableDefault();  
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
void Gyro::calibrate()
{
// Take several readings and indications while calibrating
   int cal_arr = 0;
   int samples = 20;
   for(int i = 0; i < samples; i++)
   {
     this->imu_.read();
     cal_arr+= this ->readRaw();
     delay(50);
   }

   this->bias_ = cal_arr/samples;
   
}


// Use the above bias values to return a
// compensated ("corrected") sensor reading.
int Gyro::readCalibrated()
{
  /*
   * Write code to return a calibrated reading here
   */
   return this->readRaw() - this->bias_;
}

int Gyro::getBias()
{
  return this->bias_;  
}


#endif
