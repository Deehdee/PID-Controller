//! WIP Entire file set as comment to prevent conflict, tons of unused and undeclared variables
//* This file is to help organize and clean up the process of writing the PID Controller. The goal is to keep the main 
//* and pid files seperate if possible.
#include <Arduino.h>
#include <PID_v1.h> // Custom PID library, in favor of using legacy PID library
/*
void Compute(){
  // How long since last calculated
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  // Compute all working error variables
  double error = setPoint - input;
  errSum += (error * timeChange);
  double dErr = (error - lastErr) / timeChange;

  //Compute PID Output
  output = kp * error + ki * errSum + kd * dErr;  // Initailize I2C
  Wire.begin();

  //Remember some variables for next cycle
  lastErr = error;
  lastTime = now;
  //Print Time that has passed since in seconds
  Serial.print("Time: ");
  Serial.println(now/1000);
  Serial.println("");
}

void setTunings(double Kp, double Ki, double Kd){
  kp = Kp;
  ki = Ki;
  kd = Kd;
}
*/
#ifndef PID_H
#define PID_H

void pidTest() {
  Serial.print("Test successful");
  while(1);
}

#endif