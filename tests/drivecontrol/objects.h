// Ardumower objects

#ifndef OBJECTS_H
#define OBJECTS_H


#ifdef __AVR__
  // Arduino

#else
  // simulator
  class SimLED;
  class SimMotor;
  class SimMotorMow;
  class SimSettings;
  class SimPerimeter;
  class SimTimer;
  class SimBattery;
  class SimRobot;

  extern SimLED LED;
  extern SimMotor Motor;
  extern SimMotorMow MotorMow;
  extern SimSettings Settings;
  extern SimPerimeter Perimeter;
  extern SimTimer Timer;
  extern SimBattery Battery;
  extern SimRobot Robot;

  #include "led.h"
  #include "motor.h"
  #include "motormow.h"
  #include "perimeter.h"
  #include "timer.h"
  #include "battery.h"
  #include "settings.h"
  #include "robot.h"
  #include "sim/simmower.h"

#endif // __AVR__


#endif



