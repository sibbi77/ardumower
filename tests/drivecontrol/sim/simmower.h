#ifndef SIMMOWER_H
#define SIMMOWER_H

/*#include "../types.h"
#include "../led.h"
#include "../motor.h"
#include "../perimeter.h"
#include "../settings.h"
#include "../robot.h"*/

#include "../common.h"


// Ardumower classes implementation for simulator

class SimLED : public LEDControl
{
  private:
    virtual void setDriverLED(int LEDidx, bool state){};
};

class SimMotor : public MotorControl
{
  private:
    virtual void setDriverPWM(int leftMotorPWM, int rightMotorPWM){};
};

class SimPerimeter : public PerimeterControl
{
  public:
    virtual void run(){};
};

class SimSettings : public FactorySettings
{
  public:
    virtual void setup(){
      Motor.motorLeftSwapDir = false;
      Motor.motorRightSwapDir = false;
      Motor.motorSpeedMaxRpm = 10;
      Motor.enableStallDetection = false;
      Motor.enableErrorDetection = false;
      Motor.odometryTicksPerRevolution = 1060;   // encoder ticks per one full resolution
      Motor.odometryTicksPerCm = 13.49;    // encoder ticks per cm
      Motor.odometryWheelBaseCm = 36;    // wheel-to-wheel distance (cm)
      Motor.motorLeftPID.Kp       = 0.1;    // PID speed controller
      Motor.motorLeftPID.Ki       = 0.01;
      Motor.motorLeftPID.Kd       = 0.01;
      Perimeter.enable = false;
    }
};


// simulated robot
class SimRobot : RobotControl
{
  public:
    float distanceToChgStation;
    float leftMotorSpeed; // meter/sec
    float rightMotorSpeed;
    float totalDistance; // meters
    float lastTotalDistance;
    float x; // cm
    float y;
    float orientation; // rad
    float motor_noise; // motor speed 'noise'
    float steering_noise;
    float distance_noise;
    float measurement_noise;
    float timeStep; // sec

    // initializes robot
    SimRobot(){
     distanceToChgStation = 0;
     totalDistance = 0;

     x = y = orientation = 0;
     leftMotorSpeed = 30;
     rightMotorSpeed = 5;

     steering_noise    = 0.0;
     distance_noise    = 0.0;
     measurement_noise = 0.0;
     motor_noise = 10;

     timeStep = 0.01; // one simulation step (seconds)
    }

    virtual void run(){
      float cmPerRound = Motor.odometryTicksPerRevolution / Motor.odometryTicksPerCm;

     // apply noise
     // gauss(mean, std)
     float leftSpeedNoise  = leftMotorSpeed;
     float rightSpeedNoise = rightMotorSpeed;

     rightSpeedNoise = gauss(rightMotorSpeed, motor_noise);
     leftSpeedNoise = gauss(leftMotorSpeed, motor_noise);

     float left_cm = leftSpeedNoise * cmPerRound/60.0 * timeStep;
     float right_cm = rightSpeedNoise * cmPerRound/60.0 * timeStep;

     double avg_cm  = (left_cm + right_cm) / 2.0;
     double wheel_theta = (left_cm - right_cm) / Motor.odometryWheelBaseCm;
     orientation = scalePI( orientation + wheel_theta );
     x = x + (avg_cm * cos(orientation)) ;
     y = y + (avg_cm * sin(orientation)) ;

     totalDistance += fabs(avg_cm/100.0);
    }

    // draw robot on surface
    void draw(cv::Mat &img){
      float r = Motor.odometryWheelBaseCm/2;
      circle( img, cv::Point( x, y), r, cv::Scalar( 0, 0, 0 ), 2, 8 );
      line( img, cv::Point(x, y), cv::Point(x + r * cos(orientation), y + r * sin(orientation)), cv::Scalar(0,0,0), 2, 8);
    }
};



#endif


