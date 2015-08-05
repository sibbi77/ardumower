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

// Ardumower mower motor controller

#ifndef MOTORMOW_H
#define MOTORMOW_H




class MotorMowControl
{
  public:
    float motorPWMCurr;         // current PWM
    float motorMowAccel      ;  // motor mower acceleration (warning: do not set too high)
    float motorMowSpeedMaxPwm;    // motor mower max PWM
    float motorMowPowerMax;     // motor mower max power (Watt)
    float motorSenseScale;   // motor mower sense scale  (mA=(ADC-zero) * scale)

    float motorSenseCurrent;  // current motor current (mA)
    float motorSensePower;    // current motor power (W)

    bool motorError;            // motor error?
    bool motorStalled;          // motor stalled?
    bool enableErrorDetection;  // enable error detection?
    bool enableStallDetection;  // enable stall detection?

    MotorMowControl();
    virtual void setup();
    virtual void run();
    bool hasStopped();
    void setState(bool state);
    void resetStalled();
    void resetFault();
    void stopImmediately();
    void print();
  private:
    virtual void setSpeedPWM(int pwm);
    unsigned long lastMotorCurrentTime;
    void checkMotorFault();
    virtual void readCurrent();
    unsigned long nextMotorMowTime;
    // --- driver ---
    virtual int driverReadCurrentADC();
    virtual bool driverReadFault();
    virtual void driverSetPWM(int pwm);
    virtual void driverResetFault();
};



#endif


