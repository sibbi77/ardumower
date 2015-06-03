#include "behavior.h"
#include "objects.h"



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

// ---------------------------------

HitObstacleBehavior::HitObstacleBehavior()  : Behavior(){
  name = "HitObstacleBehavior";
}

bool HitObstacleBehavior::takeControl(){
  return (  //(MotorCtrl.motion != MOTION_STOP) &&
         (Motor.motorRightStalled) || (Motor.motorLeftStalled) || (!Perimeter.isInside(0)) );
}

void HitObstacleBehavior::action(){
  suppressed = false;
  Motor.resetStalled();
  bool rotateRight = Motor.motorLeftStalled;
  //if (!Buzzer.isPlaying()) Buzzer.play(BC_LONG_SHORT_SHORT);

  // reverse
  Motor.travelLineDistance(-30, Motor.motorSpeedMaxRpm);
  while ( (!suppressed) && (!Motor.hasStopped()) ) {
    Robot.run();
  }

  // rotate
  if (rotateRight){
    Motor.rotate(-PI/2, Motor.motorSpeedMaxRpm);
  } else {
    Motor.rotate(+PI/2, Motor.motorSpeedMaxRpm);
  }
  // wait until motion stop
  while ( (!suppressed) && (!Motor.hasStopped()) ){
    Robot.run();
  }
}

