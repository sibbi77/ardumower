#ifndef CONFIG_H
#define CONFIG_H


#ifdef __AVR__
  // Arduino

#else
  // simulator
  class SimLED;
  class SimMotor;
  class SimSettings;
  class SimPerimeter;
  class SimRobot;

  extern SimLED LED;
  extern SimMotor Motor;
  extern SimSettings Settings;
  extern SimPerimeter Perimeter;
  extern SimRobot Robot;

  #include "led.h"
  #include "motor.h"
  #include "perimeter.h"
  #include "settings.h"
  #include "robot.h"
  #include "sim/simmower.h"

#endif // __AVR__


#endif



