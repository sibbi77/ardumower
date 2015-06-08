#ifndef MOTORMOW_H
#define MOTORMOW_H

// Ardumower mower motor controller



class MotorMowControl
{
  public:
    float motorPWMCurr;         // current PWM
    float motorMowAccel      ;  // motor mower acceleration (warning: do not set too high)
    float motorMowSpeedMaxPwm;    // motor mower max PWM
    float motorMowPowerMax;     // motor mower max power (Watt)
    float motorSenseScale;   // motor mower sense scale  (mA=(ADC-zero) * scale)
    float motorVoltageDC;    // battery DC voltage

    float motorSenseCurrent;  // current motor current (mA)
    float motorSensePower;    // current motor power (W)

    bool motorError;            // motor error?
    bool motorStalled;          // motor stalled?
    bool enableErrorDetection;  // enable error detection?
    bool enableStallDetection;  // enable stall detection?

    MotorMowControl();
    virtual void setup();
    virtual void run();
    virtual void setSpeedPWM(int pwm);
    bool hasStopped();
    void resetStalled();
    void resetFault();
    void stopImmediately();
    void print();
  private:
    unsigned long lastMotorCurrentTime;
    void setDriverPWM(int pwm);
    void checkMotorFault();
    void readCurrent();
    unsigned long nextMotorMowTime;
};



#endif


