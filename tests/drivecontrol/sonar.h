#ifndef SONAR_H
#define SONAR_H

// Ardumower ultrasonic sensor



class SonarControl
{
  public:
    bool enableLeft;
    bool enableRight;
    bool enableCenter;
    int sonarTriggerBelow ;    // ultrasonic sensor trigger distance
    unsigned int sonarDistCenter ;
    unsigned int sonarDistRight ;
    unsigned int sonarDistLeft ;
    unsigned int sonarDistCounter ;
    unsigned long sonarObstacleTimeout ;
    bool triggeredCenter();
    bool triggeredLeft();
    bool triggeredRight();
    bool triggeredAny();
    SonarControl();
    void setup();
    void run();
    virtual void read();
    void print();
  private:
    unsigned long nextSonarTime;
    // ----- driver -----
    virtual int driverReadLeftDistanceCm();
    virtual int driverReadRightDistanceCm();
    virtual int driverReadCenterDistanceCm();
};




#endif

