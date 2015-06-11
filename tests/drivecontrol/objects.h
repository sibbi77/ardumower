// Ardumower objects

#ifndef OBJECTS_H
#define OBJECTS_H


#ifdef __AVR__
  // Arduino
  class MowerLED;
  class MowerMotor;
  class MowerMotorMow;
  class MowerSettings;
  class MowerPerimeter;
  class MowerTimer;
  class MowerBattery;
  class MowerButton;
  class MowerSonar;
  class MowerBuzzer;
  class MowerPerimeter;
  class MowerRobot;

  extern MowerLED LED;
  extern MowerMotor Motor;
  extern MowerMotorMow MotorMow;
  extern MowerSettings Settings;
  extern MowerPerimeter Perimeter;
  extern MowerTimer Timer;
  extern MowerSonar Sonar;
  extern MowerBuzzer Buzzer;
  extern MowerButton Button;
  extern MowerBattery Battery;
  extern MowerPerimeter Perimeter;  
  extern MowerRobot Robot;


  #include "led.h"
  #include "motor.h"
  #include "motormow.h"
  #include "perimeter.h"
  #include "timer.h"
  #include "battery.h"
  #include "buzzer.h"
  #include "sonar.h"
  #include "button.h"
  #include "settings.h"
  #include "robot.h"
  
  #include "adcman.h"  
  #include "mowerperimeter.h"

  #include "mower.h"

#else
  // simulator
  class SimLED;
  class SimMotor;
  class SimMotorMow;
  class SimSettings;
  class SimPerimeter;
  class SimTimer;
  class SimBattery;
  class SimButton;
  class SimSonar;
  class SimBuzzer;
  class SimRobot;

  extern SimLED LED;
  extern SimMotor Motor;
  extern SimMotorMow MotorMow;
  extern SimSettings Settings;
  extern SimPerimeter Perimeter;
  extern SimTimer Timer;
  extern SimSonar Sonar;
  extern SimBuzzer Buzzer;
  extern SimButton Button;
  extern SimBattery Battery;
  extern SimRobot Robot;


  #include "led.h"
  #include "motor.h"
  #include "motormow.h"
  #include "perimeter.h"
  #include "timer.h"
  #include "battery.h"
  #include "buzzer.h"
  #include "sonar.h"
  #include "button.h"
  #include "settings.h"
  #include "robot.h"

  #include "sim/simmower.h"

#endif // __AVR__


#endif



