//! WIP
//* This file is to help organize and clean up the process of writing the Kalman Filter. The goal is to keep the main 
//* and kalman files seperate if possible.
#include <Arduino.h>
#include <KalmanFilter.h> //!Figure this out
#include <Adafruit_MPU6050.h> // MPU library from Adafruit

//Setup Kalman Filter Variables
KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

//kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  //kalRoll = kalmanX.update(accRoll, gyr.XAxis);
  unsigned long lastTime;
double input, output, setPoint;
double errSum, lastErr;
double kp, ki, kd;

void Setup(){
Serial.begin(115200);

}

void Loop(){
delay(1);

}