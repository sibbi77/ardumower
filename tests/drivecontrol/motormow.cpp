#include "common.h"
#include "motormow.h"



MotorMowControl::MotorMowControl(){
  motorMowAccel       = 0.1;  // motor mower acceleration (warning: do not set too high)
  motorMowSpeedMaxPwm   = 255;    // motor mower max PWM
  motorMowPowerMax = 75.0;     // motor mower max power (Watt)

  // MC33926 current:  5V / 1024 ADC counts,  525 mV per A  => 9.3 mA per ADC step
  motorSenseScale  = 9.3;  // motor left sense scale  (mA=(ADC-zero) * scale)
  motorVoltageDC = 24.0;

  motorPWMCurr = 0;
  nextMotorMowTime = 0;
  lastMotorCurrentTime = 0;
  motorSenseCurrent = 0;
  motorSensePower = 0;
  enableErrorDetection = enableStallDetection = true;
}

void MotorMowControl::setup(){
  Console.println(F("MotorMowControl::setup"));
  setSpeedPWM(0);
}

// call this in main loop
void MotorMowControl::run(){
  if (millis() < nextMotorMowTime) return;
  nextMotorMowTime = millis() + 1000;
  readCurrent();
  checkMotorFault();
}

void MotorMowControl::setSpeedPWM(int pwm){
  if (motorStalled) {
    if (pwm != 0) return;
  }
  if (pwm == 0) motorSensePower = 0;
  motorPWMCurr = pwm;
  setDriverPWM(pwm);
}

void MotorMowControl::setDriverPWM(int pwm){
}


void MotorMowControl::print(){
    Console.print(F("  pwm:"));
    Console.print(motorPWMCurr, 0);
    Console.print(",");
    Console.print(F("  mA:"));
    Console.print(motorSenseCurrent, 0);
    Console.print(F("  P:"));
    Console.print(motorSensePower, 1);
}


void MotorMowControl::stopImmediately(){
  setSpeedPWM(0);
}


bool MotorMowControl::hasStopped(){
  return (motorPWMCurr == 0);
}

void MotorMowControl::resetStalled(){
  motorStalled = false;
  Console.println(F("STALL RESET"));
}


void MotorMowControl::checkMotorFault(){
}

void MotorMowControl::resetFault(){
  Console.println(F("MotorMowControl::resetFault"));
  motorError = false;
}


// read motor current
void MotorMowControl::readCurrent(){
}







