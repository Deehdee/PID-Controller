/*
* Notes Oct 14 2024
* MPU6050 was decided to be used for I felt that community support was generally better than the LSM6DS0 (considering the library had a O, not a 0 in the library)
* Formatted some comments so that each section is seperated by a green text header-comment
* Copied the code form the MPU6050 example and assigned each acceleration and gryo value to a double value instead of the called value being printed directly.
* Code currently runs at a delay of 300 mills, idealy, the code will run with a shorter delay for the PID controller to be accurate

*/
// *-------------------Includes------------------- //
#include <Arduino.h>
#include <Servo.h> // Servo library
#include <PID_v1.h> // Custom PID library, in favor of using legacy PID library
#include <Adafruit_MPU6050.h> // MPU library from Adafruit
#include <Wire.h> //Library for I2C

// *-------------------Variables------------------- //
unsigned long lastTime;
double input, output, setPoint;
double errSum, lastErr;
double kp, ki, kd;

// setup servos 
Servo longServo; // Defines longServo object
Servo latServo; // Defines latServo object
int longPin = 28;
int latPin = 29;

// Setup gyro/accel
//TODO: Find the differences between the two MPU variables and their libraries 
Adafruit_MPU6050 mpu;
// Probably not going to use sensor for now, the example that I'm following didn't utilize it
//Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro; 

// *-------------------Functions------------------- //
//TODO Find out what this function does
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
  Serial.print("Time: ");
  Serial.println(now);
  Serial.println("");
}
//TODO Find out what this function does
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

  // Set values to 
  double ax = a.acceleration.x-0.24;
  double ay = a.acceleration.y+0.19;
  double az = a.acceleration.z+0.46;
  double gx = g.gyro.x;
  double gy = g.gyro.y-0.03;
  double gz = g.gyro.z+0.02;
  
  Serial.print("Acceleration X: ");
  Serial.print(ax);
  Serial.print(", Y: ");
  Serial.print(ay);
  Serial.print(", Z: ");
  Serial.print(az);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(gx);
  Serial.print(", Y: ");
  Serial.print(gy);
  Serial.print(", Z: ");
  Serial.print(gz);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(10);
  }
// *-------------------Setup------------------- //
void setup() {
  // Initialize Serial Connection
  Serial.begin(115200); // Initialize Serial

  // Initailize I2C
  Wire.begin();
  
  // Wait for serial monitor begin 
  while (!Serial){
    Serial.println("Waiting...");
    delay(20);}

  // Attach to servos to their respective pins
  longServo.attach(longPin);
  latServo.attach(latPin);

  //Calls IMU to initialize, freezes program if it isn't successful
  if (!mpu.begin()) {
  Serial.println("Failed to find MPU6050 chip");
  while (1) {
    delay(10);
    }
  }
  else if (mpu.begin()){
    Serial.print("mpu ready!");
  }
  //Configure the MPU
  //These settings can be changed, look at the example for the different configurations
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}
// *-------------------Loop------------------- //
void loop() {
  getMPU();
  Compute();
}
