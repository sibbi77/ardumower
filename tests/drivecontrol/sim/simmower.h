// Ardumower simulator classes

#ifndef SIMMOWER_H
#define SIMMOWER_H


#include "../common.h"
#include "../objects.h"


// world size (cm)
#define WORLD_SIZE_X 700
#define WORLD_SIZE_Y 500



// simulator settings
class SimSettings : public FactorySettings
{
  public:
    virtual void setup();
};


// simulated LED
class SimLED : public LEDControl
{
  private:
    virtual void setDriverLED(int LEDidx, bool state){};
};

// simulated buzzer
class SimBuzzer : public BuzzerControl
{
};

// simulated sonar
class SimSonar : public SonarControl
{
  private:
    virtual int driverReadCenterDistanceCm();
};

// simulated button
class SimButton : public ButtonControl
{
};

// simulated gear motors
class SimMotor : public MotorControl
{
  private:
    virtual void driverSetPWM(int leftMotorPWM, int rightMotorPWM);
    virtual int driverReadLeftCurrentADC();
    virtual int driverReadRightCurrentADC();
    virtual void readOdometry();
};

// simulated mower motor
class SimMotorMow : public MotorMowControl
{
  private:
    virtual int driverReadCurrentADC();
    virtual void driverSetPWM(int pwm);
};

class SimPlot
{
  public:
    float vmin;
    float vmax;
    std::string name;
    cv::Scalar color;
    std::vector<float> values;
};

// simulated perimeter
class SimPerimeter : public PerimeterControl
{
  public:
    cv::Mat imgBfield;
    cv::Mat imgWorld;
    cv::Mat plotPerimeter;
    cv::Mat imgPlots;
    int chgStationX, chgStationY; // cm
    float chgStationOrientation; // entrance
    std::vector<point_t> obstacles; // obstacles
    std::vector<SimPlot> simPlots;
    SimPerimeter();
    virtual void run();
    virtual bool isInside(char coilIdx);
    virtual int getMagnitude(char coilIdx);
    bool hitObstacle(int x, int y, int distance, float orientation);
    // return world size (cm)
    int sizeX();
    int sizeY();
    void setLawnMowed(int x, int y);
    bool isLawnMowed(int x, int y);
    bool isLawnAtRobotMowed();
    void draw();
    // return magnetic field strength at world position
    float getBfield(int x, int y, int resolution=1);
  private:
    // lawn mow status
    float lawnMowStatus[WORLD_SIZE_Y][WORLD_SIZE_X];
    bool drawMowedLawn;
    int plotIdx;
    // magnetic field
    float bfield[WORLD_SIZE_Y][WORLD_SIZE_X];
    int pnpoly(std::vector<point_t> &vertices, float testx, float testy);
    void addPlot(int plotIdx, float value);
    void plot();
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
};

// simulated R/C
class SimModelReceiver : public ModelReceiverControl
{
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
    virtual void processKey(char key);
    virtual char readKey();
    // draw robot on surface
    void draw(cv::Mat &img);
    void move();
};



#endif


