#ifndef _Line_follow_h
#define _Line_follow_h


//Number of readings to take for calibration
//const int NUM_CALIBRATIONS = ????;

/* 
 *  Class to represent a single line sensor
 */
class LineSensor
{
  public:

    // Required function, class Constructor: 
    // Saves the pin passed in as argument and sets to input
    LineSensor(int line_pin) {
      pin_ = line_pin;
      pinMode(pin_, INPUT);
    }

    // Suggested functions.
    void calibrate();       //Calibrate
    int readRaw();         //Return the uncalibrated value from the sensor
    int readCalibrated();  //Return the calibrated value from the sensor

    int getBias();
    // You may wish to add other functions!
    // ...
    
  private:
  
    int pin_;
    int bias_;
    /*
     * Add any variables needed for calibration here
     */
    
};




// Returns unmodified reading.
int LineSensor::readRaw()
{
  return analogRead(pin_);
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
void LineSensor::calibrate()
{
// Take several readings and indications while calibrating
   int cal_arr = 0;
   int samples = 20;
   for(int i = 0; i < samples; i++)
   {
     cal_arr+= this ->readRaw();
     delay(50);
   }

   this ->bias_ = cal_arr/samples;
   
}


// Use the above bias values to return a
// compensated ("corrected") sensor reading.
int LineSensor::readCalibrated()
{
  /*
   * Write code to return a calibrated reading here
   */
   return analogRead(this->pin_) - this->bias_;
}

int LineSensor::getBias()
{
  return this->bias_;  
}


#endif
