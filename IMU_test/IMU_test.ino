#include <Wire.h>
#include <LSM6.h>
#include "imuSensor.h"

Gyro imu;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  imu.enable();
  imu.calibrate();
}

void loop()
{

//  Serial.print("A: ");
//  Serial.print(imu.a.x);
//  Serial.print(" ");
//  Serial.print(imu.a.y);
//  Serial.print(" ");
//  Serial.print(imu.a.z);
//  Serial.print("G:  ");
//  Serial.print(imu.g.x);
//  Serial.print(" ");
//  Serial.print(imu.g.y);
//  Serial.print(" ");
  Serial.println(((float)imu.readCalibrated() * 8.75/1000)/180.0 * M_PI, 3);

  delay(100);
}
