/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  
  Private-use only! (you need to ask for a commercial-use)
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
*/

#include "objects.h"
#include "motormow.h"



MotorMowControl::MotorMowControl(){
  motorMowAccel       = 0.1;  // motor mower acceleration (warning: do not set too high)
  motorMowSpeedMaxPwm   = 255;    // motor mower max PWM
  motorMowPowerMax = 75.0;     // motor mower max power (Watt)

  // MC33926 current:  5V / 1024 ADC counts,  525 mV per A  => 9.3 mA per ADC step
  motorSenseScale  = 9.3;  // motor left sense scale  (mA=(ADC-zero) * scale)

  motorPWMCurr = 0;
  nextMotorMowTime = 0;
  lastMotorCurrentTime = 0;
  motorSenseCurrent = 0;
  motorSensePower = 0;
  motorStalled = false;
  enableErrorDetection = enableStallDetection = true;
}

void MotorMowControl::setup(){
  Console.println(F("MotorMowControl::setup"));
  setSpeedPWM(0);
}

// call this in main loop
void MotorMowControl::run(){
  if (millis() < nextMotorMowTime) return;
  nextMotorMowTime = millis() + 50;
  readCurrent();
  checkMotorFault();
  //print();
  //Console.println();
}

void MotorMowControl::setSpeedPWM(int pwm){
  if (motorStalled) {
    if (pwm != 0) return;
  }
  if (pwm == 0) motorSensePower = 0;
  motorPWMCurr = pwm;
  driverSetPWM(pwm);
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
  return (motorPWMCurr < 1);
}

void MotorMowControl::resetStalled(){
  motorStalled = false;
  Console.println(F("STALL RESET"));
}


void MotorMowControl::checkMotorFault(){
  if (enableErrorDetection){
    if ( (!motorError) && (driverReadFault()) ){
      Console.println(F("ERROR: mower motor"));
      motorError = true;
    }
  }
}

void MotorMowControl::resetFault(){
  Console.println(F("MotorMowControl::resetFault"));
  driverResetFault();
  motorError = false;
}


// read motor current
void MotorMowControl::readCurrent(){
    unsigned long TaC = millis() - lastMotorCurrentTime;    // sampling time in millis
    lastMotorCurrentTime = millis();
    if (TaC > 500) TaC = 1;
    double TaS = ((double)TaC) / 1000.0;

    // read current - NOTE: MC33926 datasheets says: accuracy is better than 20% from 0.5 to 6.0 A
    int motorSenseADC = driverReadCurrentADC();

    // compute motor current (mA)
    //double smooth = 0.0;
    double smooth = 0.5;
    motorSenseCurrent = motorSenseCurrent * smooth + ((double)motorSenseADC) * motorSenseScale * (1.0-smooth);

    // compute motor output power (W) by calculating battery voltage, pwm duty cycle and motor current
    // P_output = U_Battery * pwmDuty * I_Motor
    smooth = 0.9;
    motorSensePower = motorSensePower * smooth + (1.0-smooth) * (motorSenseCurrent * Battery.batVoltage * ((double)abs(motorPWMCurr)/255.0)  /1000);

    if (enableStallDetection) {
      if (!motorStalled){
       if ( (abs(motorPWMCurr) > 0) && (motorSensePower > motorMowPowerMax) ) {
         print();
         Console.print(F("  MOW STALL"));
         Console.println();
         motorStalled = true;
         stopImmediately();
       }
     }
    }
}


void MotorMowControl::setState(bool state){
  if (state) setSpeedPWM(motorMowSpeedMaxPwm);
    else setSpeedPWM(0);
}


int MotorMowControl::driverReadCurrentADC(){
}

void MotorMowControl::driverSetPWM(int pwm){
}

void MotorMowControl::driverResetFault(){
}

bool MotorMowControl::driverReadFault(){
}

