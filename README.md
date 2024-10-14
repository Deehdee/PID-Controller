# PID-Controller
This project will allow a microcontroller to provide a desired output based on reading from an IMU, using a Proportional Integral Derivative Controller (PID Controller).

# Technologies
**IDE**: This project was written with ProjectIO for VSCode in mind. Although you can still copy the main.cpp code and paste it into the Arduino IDE with no issues.

**Microcontroller**: The microcontroller I built this around was for the Teensy 4.1, of course you can use any microcontroller that supports I2C. 

**IMU**: The IMU used in this project is the MPU6050. I felt that it was most accessible part to use for this project for having good examples and an excellent library that was easy to read.

# Installation

**VSCode**
1 .Ensure that the PlatformIO extension is installed on VSCode, and that you have the most up to date version.
2. Copy the URL to the github repo [PID-Controller](https://github.com/Deehdee/PID-Controller/tree/main)
3. In PlatformIO, click "Clone Git Project" and paste the link to the repo.
4. When prompted to select a folder to place the repo, go to your platformIO projects folder and place it there.

**Arduino**
1. Ensure Arduino IDE is up to date
2. Copy main.cpp file code (or save as an .ino file) and paste it into your Arduino IDE
3. The following libraries are used for this project and should be installed, or errors will occur:
 * PID 1.2.1 by br3ttb
 * MPU6050 2.2.6 by Adafruit

