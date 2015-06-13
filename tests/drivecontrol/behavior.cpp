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

  MotorMow.setState(true);

  // forward
  Motor.travelLineSpeedRpm(Motor.motorSpeedMaxRpm);

  while ( (!suppressed) && (Motor.motion != MOTION_STOP) ){
    Robot.run();

    if (MotorMow.motorStalled){
      MotorMow.resetStalled();
      MotorMow.setState(true);
    }
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
  //Motor.travelLineSpeedRpm(-Motor.motorSpeedMaxRpm, -Motor.motorSpeedMaxRpm);
  Console.println("REV");
  Motor.travelLineDistance(-60, Motor.motorSpeedMaxRpm);
  while ( (!suppressed) && (!Motor.hasStopped()) ) {
    if (Perimeter.isInside(0)) {
      Motor.stopImmediately();
      break;
    }
    Robot.run();
  }

  // rotate
  Motor.resetStalled();
  Console.println("ROT");
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
  return ( (Motor.motorRightStalled) || (Motor.motorLeftStalled)  || (Sonar.triggeredAny()) );
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
  //Motor.travelLineSpeedRpm(-Motor.motorSpeedMaxRpm, -Motor.motorSpeedMaxRpm);
  Motor.travelLineDistance(-30, Motor.motorSpeedMaxRpm);
  while ( (!suppressed) && (!Motor.hasStopped()) ) {
    /*if (Perimeter.isInside(0)) {
      Motor.stopImmediately();
      break;
    }*/
    Robot.run();
  }

  // rotate
  Motor.resetStalled();
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
  //MotorMow.setState(false);
  Motor.resetStalled();
  Motor.rotate(PI, Motor.motorSpeedMaxRpm/2);
  while ( (!suppressed) && (!Motor.hasStopped()) ) {
    Robot.run();
    if (Perimeter.isInside(0)) {
      Motor.stopImmediately();
      break;
    }
  }
  // reverse
  //Motor.travelLineSpeedRpm(-Motor.motorSpeedMaxRpm, -Motor.motorSpeedMaxRpm);
  //Motor.travelLineDistance(-30, Motor.motorSpeedMaxRpm);

  Motor.enableSpeedControl = false;
  Motor.enableStallDetection = false;
  unsigned long nextControlTime = 0;
  while (  (!suppressed) && (!Motor.hasStopped())  ) {
    if (millis() >= nextControlTime){
      nextControlTime = millis() + 50;
      int mag = Perimeter.getMagnitude(0);
      if (mag <= 0) Robot.perimeterPID.x = -1;
        else if (mag > 0) Robot.perimeterPID.x = 1;
      Robot.perimeterPID.w = 0;
      Robot.perimeterPID.y_min = -Motor.motorSpeedMaxPwm;
      Robot.perimeterPID.y_max = Motor.motorSpeedMaxPwm;
      Robot.perimeterPID.max_output = Motor.motorSpeedMaxPwm;
      Robot.perimeterPID.compute();
      //printf("%d, %.3f\n", mag, pidTrack.y);
      Motor.setSpeedPWM( Motor.motorSpeedMaxPwm/2 - Robot.perimeterPID.y,
                         Motor.motorSpeedMaxPwm/2 + Robot.perimeterPID.y );
    }
    Robot.run();
  }
  Motor.enableSpeedControl = true;
  Motor.enableStallDetection = true;
}

// ----------------------------------------

ChargingBehavior::ChargingBehavior() : Behavior() {
  name = "ChargingBehavior";
}

bool ChargingBehavior::takeControl(){
  return ( Battery.chargerConnected() );
}

void ChargingBehavior::action(){
  suppressed = false;

  Motor.stopImmediately();
  MotorMow.setState(false);
  //LED.playSequence(LED_OFF);
  //Buzzer.play(BC_SHORT_SHORT_SHORT);

  // wait until some other behavior was activated
  while ( (!suppressed ) && (Battery.chargerConnected()) ) {
    Robot.run();
  }
}

// ---------------------------------

CircleBehavior::CircleBehavior() : Behavior() {
  name = "CircleBehavior";
}

bool CircleBehavior::takeControl(){
  return (MotorMow.motorSensePower > Robot.motorMowCircleAbovePower);
}

void CircleBehavior::action(){
  suppressed = false;

  MotorMow.setState(true);
  Motor.setSpeedRpm(Motor.motorSpeedMaxRpm, Motor.motorSpeedMaxRpm);

  unsigned long startTime = millis();
  float ratio = 0.0;
  bool circleLeft = ((rand() % 2) == 0);

  while ( (!suppressed) && (Motor.motion != MOTION_STOP) && (ratio < 1.0) ){
    unsigned long duration (millis() - startTime);
    ratio = ((float)duration) / 30000.0;
    if (circleLeft) Motor.setSpeedRpm(Motor.motorSpeedMaxRpm*ratio, Motor.motorSpeedMaxRpm);
      else Motor.setSpeedRpm(Motor.motorSpeedMaxRpm, Motor.motorSpeedMaxRpm*ratio);

    Robot.run();

    if (MotorMow.motorStalled){
      MotorMow.resetStalled();
      MotorMow.setState(true);
    }
  }
}

// ---------------------------------
