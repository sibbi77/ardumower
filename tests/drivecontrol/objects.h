/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  
  Private-use only! (you need to ask for a commercial-use)
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
*/

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
  class MowerModelReceiver;
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
  extern MowerModelReceiver ModelReceiver;
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
  #include "modelrc.h"
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
  class SimModelReceiver;
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
  extern SimModelReceiver ModelReceiver;
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
  #include "modelrc.h"
  #include "settings.h"
  #include "robot.h"

  #include "sim/simmower.h"

#endif // __AVR__


#endif



