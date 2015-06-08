#ifndef SIMMOWER_H
#define SIMMOWER_H

/*#include "../types.h"
#include "../led.h"
#include "../motor.h"
#include "../perimeter.h"
#include "../settings.h"
#include "../robot.h"*/

#include "../common.h"
#include "../objects.h"


// Ardumower classes implementation for simulator

// world size (cm)
#define WORLD_SIZE_X 700
#define WORLD_SIZE_Y 500


class SimSettings : public FactorySettings
{
  public:
    virtual void setup();
};


class SimLED : public LEDControl
{
  private:
    virtual void setDriverLED(int LEDidx, bool state){};
};

class SimMotor : public MotorControl
{
  private:
    virtual void setDriverPWM(int leftMotorPWM, int rightMotorPWM);
    virtual void readOdometry();
    virtual void readCurrent();
};

class SimMotorMow : public MotorMowControl
{
  private:
    virtual void setDriverPWM(int pwm);
};

class SimPerimeter : public PerimeterControl
{
  public:
    cv::Mat imgBfield;
    cv::Mat imgWorld;
    int chgStationX, chgStationY; // cm
    std::vector<point_t> obstacles; // obstacles
    SimPerimeter();
    virtual void run();
    virtual bool isInside(char coilIdx);
    virtual int getMagnitude(char coilIdx);
    bool hitObstacle(int x, int y, int distance, float orientation);
    // return world size (cm)
    int sizeX();
    int sizeY();
    void setLawnMowed(int x, int y);
    void draw();
    // return magnetic field strength at world position
    float getBfield(int x, int y, int resolution=1);
    // lawn mow status
    float lawnMowStatus[WORLD_SIZE_Y][WORLD_SIZE_X];
  private:
    bool drawMowedLawn;
    // magnetic field
    float bfield[WORLD_SIZE_Y][WORLD_SIZE_X];
    int pnpoly(std::vector<point_t> &vertices, float testx, float testy);
};

// simulated timer
class SimTimer : public TimerControl
{
  public:
    bool simStopped;
    bool simFast;
    float simTimeStep; // simulation step (sec)
    float simTimeTotal; // simulation time (sec)
    SimTimer();
    virtual void run();
    unsigned long millis(); // simulation time
};

// simulated battery
class SimBattery : public BatteryControl
{
  public:
    virtual void read();
    virtual bool isCharging();
};


// simulated robot
class SimRobot : public RobotControl
{
  public:
    float distanceToChgStation;
    float simX; // real position (cm)
    float simY;
    float simOrientation; // real orientation (rad)
    float motor_noise; // motor speed 'noise'
    float slope_noise;
    // initializes robot
    SimRobot();
    virtual void run();
    // draw robot on surface
    void draw(cv::Mat &img);
    void move();
};



#endif


