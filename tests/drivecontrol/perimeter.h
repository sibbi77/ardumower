
#ifndef PERIMETER_H
#define PERIMETER_H



// coil index
enum {
  COIL_LEFT,
  COIL_RIGHT,
};

class PerimeterControl
{
  public:
    PerimeterControl();
    bool enable; // enable ?
    // get perimeter magnitude
    int getMagnitude(char coilIdx);
    int getSmoothMagnitude(char coilIdx);
    // inside perimeter (true) or outside (false)?
    virtual bool isInside(char coilIdx);
    // perimeter signal timed out? (e.g. due to broken wire)
    bool signalTimedOut(char coilIdx);
    int getSignalMin(char coilIdx);
    int getSignalMax(char coilIdx);
    int getSignalAvg(char coilIdx);
    float getFilterQuality(char coilIdx);
    int timedOutIfBelowSmag;
    int timeOutSecIfNotInside;
    virtual void setup();
    virtual void run();
  private:
    unsigned long nextPerimeterTime;
    unsigned long lastInsideTime[2];
    char idxPin[2]; // channel for idx
    int callCounter;
    int mag [2]; // perimeter magnitude per channel
    float smoothMag[2];
    float filterQuality[2];
    int signalMin[2];
    int signalMax[2];
    int signalAvg[2];
    int signalCounter[2];
};


#endif

