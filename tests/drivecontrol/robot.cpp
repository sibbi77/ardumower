#include "common.h"
#include "objects.h"
#include "robot.h"


RobotControl::RobotControl(){
  loopCounter = 0;
  nextPrintTime = 0;
}


void RobotControl::setup(){
  Console.println(F("-----SETUP-----"));

  LED.setup();
  Motor.setup();
  Perimeter.setup();
  Settings.setup();

  // low-to-high priority
  arbitrator.addBehavior(&driveForwardBehavior);
  arbitrator.addBehavior(&hitObstacleBehavior);
  arbitrator.addBehavior(&hitPerimeterBehavior);

  Console.println(F("-----SETUP completed-----"));
}



void RobotControl::print(){
  Console.print("loopsPerSec=");
  Console.print(((double)loopCounter)/10.0);
  Console.print("  behavior=");
  if (arbitrator.activeBehavior){
    Console.print(arbitrator.activeBehavior->name);
  }
  Console.println();
  loopCounter = 0;
}

// call this in ANY loop!
void RobotControl::run(){
  //Console.println(F("RobotControl::run"));

  arbitrator.monitor();
  Motor.run();
  LED.run();

  if (millis() >= nextPrintTime){
    nextPrintTime = millis() + 10000;
    print();
  }

  //delay(50);
  loopCounter++;
}


  /* program flow:
     a. aribitrator.run (1) calls specific behavior.run (2)
     b. specific behavior.run periodically calls Robot.run (3)
     c. Robot.run processes MotorCtrl, BuzzerCtrl, PfodApp etc. and monitors (4) for behavior suppression (5)
     d. behavior is finsihed if finished or suppressed
     e. arbitrator.run is called again for next behavior


     aribitrator                  behavior                   Robot
     --------------------------------------------------------------
(1) run-->|                           |                         |
          |----------------(2)--run-->|                         |
          |                           |--------------(3)--run-->|
          |                           |                         |
          |                           |                         |
          |<--monitor--(4)--------------------------------------|
          |                           |                         |
          |-----------(5)--suppress-->|                         |
          |                           X                         |
          |                                                     |
          |                                                     |
   */

// this is called over and over from the Arduino loop
void RobotControl::loop(){
  //Console.print(F("RobotControl::loop "));
  //Console.println(loopCounter);
  arbitrator.run();
}




