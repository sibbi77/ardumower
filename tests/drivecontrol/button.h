// Ardumower ON/start button


#ifndef BUTTON_H
#define BUTTON_H


class ButtonControl
{
  public:
    bool pressed;
    int beepCounter;
    ButtonControl();
    void run();
    void setup();
    void resetBeepCounter();
    void setBeepCount(int count);
  private:
    int tempBeepCounter;
    unsigned long nextButtonTime;
    // --- driver ---
    virtual bool driverButtonPressed();
};



#endif

