#include "led.h"
#include "behavior.h"
#include "config.h"



DriveForwardBehavior::DriveForwardBehavior() : Behavior() {
  name = "DriveForwardBehavior";
}

bool DriveForwardBehavior::takeControl(){
  return true;
}

void DriveForwardBehavior::action(){
  suppressed = false;

  LED.playSequence(LED_SEQ_GREEN_ON);

  // forward
  Motor.setSpeedRpm(Motor.motorSpeedMaxRpm, Motor.motorSpeedMaxRpm);

  while ( (!suppressed) && (Motor.motion != MOTION_STOP) ){
    Robot.run();
  }
}



