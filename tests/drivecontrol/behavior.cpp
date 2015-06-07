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

HitPerimeterBehavior::HitPerimeterBehavior()  : Behavior(){
  name = "HitPerimeterBehavior";
}

bool HitPerimeterBehavior::takeControl(){
  return ( (Perimeter.enable) && (!Perimeter.isInside(0)) );
}

void HitPerimeterBehavior::action(){
  suppressed = false;
  Motor.stopImmediately();
  bool rotateLeft = ((rand() % 2) == 0);
  float angle = ((float)random(90, 180)) / 180.0 * PI;
  //float angle = PI;
  if (rotateLeft) angle *= -1;

  /*Motor.rotate(angle, Motor.motorSpeedMaxRpm/2);
  while ( (!suppressed) && (!Motor.hasStopped()) ) {
    Robot.run();
    if (Perimeter.isInside(0)) {
      Motor.stopImmediately();
      break;
    }
  }
  return;*/

  //float angle = PI;
  //if (!Buzzer.isPlaying()) Buzzer.play(BC_SHORT_SHORT);

  // reverse
  //Motor.setSpeedRpm(-Motor.motorSpeedMaxRpm, -Motor.motorSpeedMaxRpm);
  Motor.travelLineDistance(-30, Motor.motorSpeedMaxRpm);
  while ( (!suppressed) && (!Motor.hasStopped()) ) {
    if (Perimeter.isInside(0)) {
      Motor.stopImmediately();
      break;
    }
    Robot.run();
  }

  // rotate
  Motor.rotate(angle, Motor.motorSpeedMaxRpm/2);

  // wait until motion stop
  while ( (!suppressed) && (!Motor.hasStopped()) ){
    Robot.run();
  }
}



// ---------------------------------

HitObstacleBehavior::HitObstacleBehavior()  : Behavior(){
  name = "HitObstacleBehavior";
}

bool HitObstacleBehavior::takeControl(){
  return ( (Motor.motorRightStalled) || (Motor.motorLeftStalled) );
  //return ( Perimeter.hitObstacle(Robot.simX, Robot.simY, Motor.odometryWheelBaseCm/2+8) );
}

void HitObstacleBehavior::action(){
  suppressed = false;
  //Motor.stopImmediately();
  Motor.resetStalled();
  bool rotateLeft = ((rand() % 2) == 0);
  float angle = ((float)random(90, 180)) / 180.0 * PI;
  if (rotateLeft) angle *= -1;
  //float angle = PI;
  //if (!Buzzer.isPlaying()) Buzzer.play(BC_SHORT_SHORT);

  // reverse
  //Motor.setSpeedRpm(-Motor.motorSpeedMaxRpm, -Motor.motorSpeedMaxRpm);
  Motor.travelLineDistance(-30, Motor.motorSpeedMaxRpm);
  while ( (!suppressed) && (!Motor.hasStopped()) ) {
    /*if (Perimeter.isInside(0)) {
      Motor.stopImmediately();
      break;
    }*/
    Robot.run();
  }

  // rotate
  Motor.rotate(angle, Motor.motorSpeedMaxRpm/2);

  // wait until motion stop
  while ( (!suppressed) && (!Motor.hasStopped()) ){
    Robot.run();
  }
}

