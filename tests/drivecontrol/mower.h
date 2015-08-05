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

// Ardumower Arduino Mega robot classes

#ifndef MOWER_H
#define MOWER_H

#include "common.h"
#include "objects.h"



class MowerSettings : public FactorySettings
{
  public:   
    virtual void setup();
};


class MowerLED : public LEDControl
{
  private:
    virtual void setDriverLED(int LEDidx, bool state);
};

class MowerModelReceiver : public ModelReceiverControl
{
};

class MowerBuzzer : public BuzzerControl
{
  private:
    virtual void driverNoTone();
    virtual void driverTone(int frequencyHz);
};


class MowerSonar : public SonarControl
{
  private:
    virtual int driverReadCenterDistanceCm();
    virtual int driverReadLeftDistanceCm();
    virtual int driverReadRightDistanceCm();    
};


class MowerButton : public ButtonControl
{
  private:
    bool driverButtonPressed();
};


class MowerMotor : public MotorControl
{
  private:
    virtual void driverSetPWM(int leftMotorPWM, int rightMotorPWM);
    virtual int driverReadLeftCurrentADC();
    virtual int driverReadRightCurrentADC();
    virtual bool driverReadRightFault();
    virtual bool driverReadLeftFault();
    virtual void driverResetFault();

};

class MowerMotorMow : public MotorMowControl
{
  private:
    virtual int driverReadCurrentADC();
    virtual bool driverReadFault();
    virtual void driverSetPWM(int pwm);
    virtual void driverResetFault();
};

class MowerTimer : public TimerControl
{
  private:
    virtual bool driverReadRTC(datetime_t &dt);
    virtual bool driverSetRTC(datetime_t &dt);
};

class MowerBattery : public BatteryControl
{
  private:
    virtual void driverSetBatterySwitch(bool state);
    virtual void driverSetChargeRelay(bool state);
    virtual int driverReadBatteryVoltageADC();
    virtual int driverReadChargeVoltageADC();
    virtual int driverReadChargeCurrentADC();
    virtual int driverReadVoltageMeasurementADC();
    
};


class MowerRobot : public RobotControl
{
  public:
    MowerRobot();
    virtual void run();
    virtual void processKey(char key);
    virtual char readKey();
};



#endif

