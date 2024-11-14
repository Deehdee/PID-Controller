/*
* Notes Oct 14 2024
* MPU6050 was decided to be used for I felt that community support was generally better than the LSM6DS0 (considering the library had a O, not a 0 in the library)
* Formatted some comments so that each section is seperated by a green text header-comment
* Copied the code form the MPU6050 example and assigned each acceleration and gryo value to a double value instead of the called value being printed directly.
* Code currently runs at a delay of 300 mills, idealy, the code will run with a shorter delay for the PID controller to be accurate
* Solved an issue where servos were not communicating with Teensy.
* -When using Arduino IDE, servo behave normally using Servo.h library, but not when using VSCode
* -When using PWMServo.h library, servos behave normally
*/
//TODO: Find the differences between the two MPU variables and their libraries 
//TODO Find out what Compute() function does
//TODO Find out what tsetTunings() function does
//TODO Research how to calculate gains for PID Controller
//TODO Impliment Kalman Filter - one that works properly

// *-------------------Includes------------------- //
#include <Arduino.h> //!Is this one necessary?
#include <PWMServo.h> // Servo library, PWM library works for Teensy in VSCode
#include <PID_v1.h> // Custom PID library, in favor of using legacy PID library
#include <Adafruit_MPU6050.h> // MPU library from Adafruit
#include <Wire.h> //Library for I2C
#include <KalmanFilter.h> //!Figure this out

// *-------------------Variables------------------- //
unsigned long lastTime;
double input, output, setPoint;
double errSum, lastErr;
double kp, ki, kd;

// setup servos 
PWMServo longServo; // Defines longServo object
PWMServo latServo; // Defines latServo object
int longPin = 28;
int latPin = 29;

// Setup gyro/accel
Adafruit_MPU6050 mpu;

//Setup Kalman Filter Variables
KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

// *-------------------Functions------------------- //
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


// *-------------------MPU6050------------------- //
void getMPU(){
/* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
   mpu.getEvent(&a, &g, &temp);
  
  //kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  //kalRoll = kalmanX.update(accRoll, gyr.XAxis);

  // Set values to 
  double ax = a.acceleration.x-0.24;
  double ay = a.acceleration.y+0.19;
  double az = a.acceleration.z+0.46;
  double gx = g.gyro.x;
  double gy = g.gyro.y-0.03;
  double gz = g.gyro.z+0.02;
  
  // Print Acceleration X,Y,Z;
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.print(az);
  Serial.print(", ");
  // Acceleration measured in m/s^2

  //Serial Rotation X,Y,Z;
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.print(gz);
  Serial.print(", ");
  // Rotation measured in rad/s

  //Serial Temperature;
  Serial.print(temp.temperature);
  Serial.println(",");
  //Temperature measured in degrees celsius
  delay(100);
  }
// *-------------------Setup------------------- //
void setup() {
  // Initialize Serial Connection
  Serial.begin(115200); // Initialize Serial

  // Initailize I2C
  Wire.begin();

  // Wait for serial monitor begin 
  /*while (!Serial){
    Serial.println("Waiting...");
    delay(20);}
    */

  // Attach to servos to their respective pins
  longServo.attach(longPin);
  latServo.attach(latPin);
  Serial.println("Long and Lat Servos attached.");
  delay(100);

  //Calls IMU to initialize, freezes program if it isn't successful
  if (!mpu.begin()) {
  Serial.println("Failed to find MPU6050 chip");
  while (1) {
    delay(10);
    }
  }
  else if (mpu.begin()){
    Serial.println("MPU ready!");
    delay(100);
  }
  //Configure the MPU
  //These settings can be changed, look at the example for the different configurations
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU Configured: 8Gs - 500 DEG - 21HZ");

}
// *-------------------Loop------------------- //
void loop() {
  getMPU();
  Compute();
  longServo.write(90);
  delay(1000);
  longServo.write(0);
  delay(1000);
}
