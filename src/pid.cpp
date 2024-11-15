//! WIP
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
  //* PID controller needs further programming. kp, ki, and kd have values
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