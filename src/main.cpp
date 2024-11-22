//TODO: Find the differences between the two MPU variables and their libraries 
//TODO Find out what Compute() function does
//TODO Find out what tsetTunings() function does
//TODO Research how to calculate gains for PID Controller
//TODO Impliment Kalman Filter - one that works properly

// *-------------------Includes------------------- //
#include <Arduino.h> //!Is this one necessary?
#include <PWMServo.h> // Servo library, PWM library works for Teensy in VSCode
#include <Adafruit_MPU6050.h> // MPU library from Adafruit
#include <Wire.h> //Library for I2C
#include <pidController.cpp>

// *-------------------Variables------------------- //
#define PI 3.1415926535897932384626433832795
// setup servos 
PWMServo longServo; // Defines longServo object
PWMServo latServo; // Defines latServo object
int longPin = 28;
int latPin = 29;

// Setup gyro/accel
Adafruit_MPU6050 mpu;

// *-------------------Functions------------------- //



// *-------------------MPU6050------------------- //
void getMPU(){
/* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
   mpu.getEvent(&a, &g, &temp);  

  // Set values to 
  double ax = a.acceleration.x-0.24;
  double ay = a.acceleration.y+0.19;
  double az = a.acceleration.z+0.46;
  double gx = (g.gyro.x)*(180/PI); // 180/PI converts radians to degrees
  double gy = (g.gyro.y-0.03)*(180/PI);
  double gz = (g.gyro.z+0.02)*(180/PI);
  
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
  // Rotation measured in degrees/s

  //Serial Temperature;
  Serial.print(temp.temperature);
  Serial.print(",");
  //Temperature measured in degrees celsius
  delay(100);
  double latPosition;
  latPosition = map(ax, -16, 16, 0, 180);
  latServo.write(latPosition);
  Serial.print(latPosition);
  Serial.println(",");
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
  //getMPU();
  pidTest();
}
