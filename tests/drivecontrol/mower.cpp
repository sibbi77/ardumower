#include "mower.h"


MowerLED LED;
MowerMotor Motor;
MowerMotorMow MotorMow;
MowerSettings Settings;
MowerPerimeter Perimeter;
MowerBattery Battery;
MowerBuzzer Buzzer;
MowerSonar Sonar;
MowerButton Button;
MowerTimer Timer;
MowerRobot Robot;


// ------------------------------------------

void MowerSettings::setup(){
  //randomSeed( time(NULL) );
  // --- gear motors ----
  Motor.motorSpeedMaxPwm = 255;
  Motor.motorLeftSwapDir = false;
  Motor.motorRightSwapDir = false;
  Motor.motorSenseRightScale = 9.3;  // motor right sense scale (mA=(ADC-zero) * scale)
  Motor.motorSenseLeftScale  = 9.3;  // motor left sense scale  (mA=(ADC-zero) * scale)
  Motor.motorSpeedMaxRpm = 25;
  Motor.motorEfficiencyMin = 200;
  Motor.enableStallDetection = true;
  Motor.enableErrorDetection = false;
  Motor.odometryTicksPerRevolution = 1060;   // encoder ticks per one full resolution
  Motor.odometryTicksPerCm = 13.49;    // encoder ticks per cm
  Motor.odometryWheelBaseCm = 36;    // wheel-to-wheel distance (cm)
  Motor.motorLeftPID.Kp       = 0.4;    // PID speed controller
  Motor.motorLeftPID.Ki       = 0.0;
  Motor.motorLeftPID.Kd       = 0.0;
  // --- mower motors ----
  MotorMow.motorMowSpeedMaxPwm = 255;
  MotorMow.motorSenseScale = 9.3;
  MotorMow.motorMowPowerMax = 100;
  MotorMow.enableStallDetection = true;
  MotorMow.enableErrorDetection = false;
  Robot.motorMowCircleAbovePower = 70;
  // --- perimeter motors ----
  Perimeter.enable = true;
  Robot.perimeterPID.Kp    = 160;  // perimeter PID controller
  Robot.perimeterPID.Ki    = 4;
  Robot.perimeterPID.Kd    = 50;
  // --- sonar ----
  Sonar.enableCenter = true;
  Sonar.enableLeft = false;
  Sonar.enableRight = false;
  // --- battery ----
  Battery.enableMonitor = true;
  Battery.batFull = 29.4;
  //Battery.batGoHomeIfBelow = 29.39;
  Battery.batGoHomeIfBelow = 23.7;
  Battery.batVoltage = Battery.batFull;
}

// ------------------------------------------

void MowerMotor::driverSetPWM(int leftMotorPWM, int rightMotorPWM){
}

int MowerMotor::driverReadLeftCurrentADC(){
}

int MowerMotor::driverReadRightCurrentADC(){
}


void MowerMotor::readOdometry(){
}

// ------------------------------------------

void MowerMotorMow::driverSetPWM(int pwm){
}

int MowerMotorMow::driverReadCurrentADC(){
}

// ------------------------------------------

int MowerSonar::driverReadCenterDistanceCm(){
  return 1000;
}

// ------------------------------------------

MowerPerimeter::MowerPerimeter(){
}


void MowerPerimeter::run(){
}


bool MowerPerimeter::isInside(char coilIdx){
}

int MowerPerimeter::getMagnitude(char coilIdx){
}

// ------------------------------------------

MowerTimer::MowerTimer(){
}

void MowerTimer::run(){
  TimerControl::run();  
}

// ------------------------------------------

void MowerBattery::read(){
}

// ------------------------------------------

// initializes robot
MowerRobot::MowerRobot(){
}

char MowerRobot::readKey(){
  return 0;
}

void MowerRobot::run(){
  RobotControl::run();  
}

void MowerRobot::processKey(char key){
  RobotControl::processKey(key);
}



// ------------------------------------------




