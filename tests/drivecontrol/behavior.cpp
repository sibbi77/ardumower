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

  MotorMow.setSpeedPWM(MotorMow.motorMowSpeedMaxPwm);

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

// ---------------------------------

TrackingBehavior::TrackingBehavior()  : Behavior(){
  name = "TrackingBehavior";
}

bool TrackingBehavior::takeControl(){

  return (   (Battery.robotShouldGoHome()) &&
             (Perimeter.enable) && (!Perimeter.isInside(0))  );
}

void TrackingBehavior::action(){
  suppressed = false;
  //Motor.stopImmediately();
  MotorMow.setSpeedPWM(0);
  Motor.rotate(PI, Motor.motorSpeedMaxRpm/2);
  while ( (!suppressed) && (!Motor.hasStopped()) ) {
    Robot.run();
    if (Perimeter.isInside(0)) {
      Motor.stopImmediately();
      break;
    }
  }
  // reverse
  //Motor.setSpeedRpm(-Motor.motorSpeedMaxRpm, -Motor.motorSpeedMaxRpm);
  //Motor.travelLineDistance(-30, Motor.motorSpeedMaxRpm);
  PID pidTrack;
  pidTrack.Kp    = 160;  // perimeter PID controller
  pidTrack.Ki    = 4;
  pidTrack.Kd    = 50;

  Motor.enableSpeedControl = false;
  unsigned long nextControlTime = 0;
  while ( !suppressed) {
    if (millis() >= nextControlTime){
      nextControlTime = millis() + 50;
      int mag = Perimeter.getMagnitude(0);
      if (mag < 0) pidTrack.x = -1;
        else if (mag > 0) pidTrack.x = 1;
        else pidTrack.x = 0;
      pidTrack.w = 0;
      pidTrack.y_min = -Motor.motorSpeedMaxPwm;
      pidTrack.y_max = Motor.motorSpeedMaxPwm;
      pidTrack.max_output = Motor.motorSpeedMaxPwm;
      pidTrack.compute();
      //printf("%d, %.3f\n", mag, pidTrack.y);
      Motor.setSpeedPWM( Motor.motorSpeedMaxPwm/2 - pidTrack.y,
                         Motor.motorSpeedMaxPwm/2 + pidTrack.y );
    }
    Robot.run();
  }
  Motor.enableSpeedControl = true;
}

// ----------------------------------------

ChargingBehavior::ChargingBehavior() : Behavior() {
  name = "ChargingBehavior";
}

bool ChargingBehavior::takeControl(){
  return ( Battery.isCharging() );
}

void ChargingBehavior::action(){
  suppressed = false;

  Motor.stopImmediately();
  //LED.playSequence(LED_OFF);
  //Buzzer.play(BC_SHORT_SHORT_SHORT);

  // wait until some other behavior was activated
  while ( (!suppressed ) && (Battery.isCharging()) ) {
    Robot.run();
  }
}
