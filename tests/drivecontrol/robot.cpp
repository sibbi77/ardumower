#include "common.h"
#include "objects.h"
#include "robot.h"


RobotControl::RobotControl(){
  loopCounter = 0;
  nextPrintTime = 0;
}


void RobotControl::setup(){
  Console.println(F("-----SETUP-----"));

  Battery.setup();
  Timer.setup();
  LED.setup();
  Motor.setup();
  MotorMow.setup();
  Perimeter.setup();
  Sonar.setup();
  Button.setup();
  Buzzer.setup();
  Settings.setup();

  // low-to-high priority
  arbitrator.addBehavior(&driveForwardBehavior);
  arbitrator.addBehavior(&circleBehavior);
  arbitrator.addBehavior(&hitObstacleBehavior);
  arbitrator.addBehavior(&hitPerimeterBehavior);
  arbitrator.addBehavior(&trackingBehavior);
  arbitrator.addBehavior(&chargingBehavior);

  Console.println(F("-----SETUP completed-----"));
}

char RobotControl::readKey(){
  if (Console.available()){
    return ((char)Console.read());
  } else return 0;
}

void RobotControl::processKey(char key){
    switch (key){
      // simulate button press
      case '1': Button.setBeepCount(1); break;
      case '2': Button.setBeepCount(2); break;
      case '3': Button.setBeepCount(3); break;
      case '4': Button.setBeepCount(4); break;
      case '5': Button.setBeepCount(5); break;
      case 'm':
        Motor.enableSpeedControl = false;
        Motor.setSpeedPWM(127, 127);
        break;
      case 'p':
        //motorPIDTest();
        break;
      case 's':
        // simulate sonar trigger
        Sonar.sonarDistLeft = Sonar.sonarTriggerBelow-1;
        break;
      case 'e':
        // simulate motor error
        Motor.motorLeftError = true;
        break;
      case 'h':
        // simulate low battery voltage
        Battery.batVoltage = 18;
        break;
      case 'b':
        // simulate charger connection
        if (Battery.chgVoltage > 5){
          Battery.chgVoltage = 0;
          Battery.batVoltage = 24;
        } else {
          Battery.chgVoltage = 27;
          Battery.batVoltage = 24;
        }
        break;
      case 't':
        // force tracking behavior
        //arbitrator.setNextBehavior(&trackingBehavior);
        break;
    }
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
  Timer.run();
  Battery.run();
  Buzzer.run();
  Button.run();
  Sonar.run();
  Motor.run();
  MotorMow.run();
  Perimeter.run();
  LED.run();

  if (millis() >= nextPrintTime){
    nextPrintTime = millis() + 10000;
    print();
  }

  char key = readKey();
  processKey(key);

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




