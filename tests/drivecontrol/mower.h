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
    virtual void setDriverLED(int LEDidx, bool state){};
};


class MowerBuzzer : public BuzzerControl
{
};


class MowerSonar : public SonarControl
{
  private:
    virtual int driverReadCenterDistanceCm();
};


class MowerButton : public ButtonControl
{
};


class MowerMotor : public MotorControl
{
  private:
    virtual void driverSetPWM(int leftMotorPWM, int rightMotorPWM);
    virtual int driverReadLeftCurrentADC();
    virtual int driverReadRightCurrentADC();
    virtual void readOdometry();
};

class MowerMotorMow : public MotorMowControl
{
  private:
    virtual int driverReadCurrentADC();
    virtual void driverSetPWM(int pwm);
};

class MowerTimer : public TimerControl
{
  public:
    MowerTimer();
    virtual void run();    
};

class MowerBattery : public BatteryControl
{
  public:
    virtual void read();
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

