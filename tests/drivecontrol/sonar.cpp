#include "objects.h"
#include "sonar.h"


#define NO_ECHO 0
// ultrasonic sensor max echo time (WARNING: do not set too high, it consumes CPU time!)
#define MAX_ECHO_TIME 3000
//#define MIN_ECHO_TIME 350
#define MIN_ECHO_TIME 0


SonarControl::SonarControl(){
  nextSonarTime = 0;
  enableLeft = false;
  enableRight = false;
  enableCenter = false;
  sonarTriggerBelow = 900;    // ultrasonic sensor trigger distance
  sonarDistCenter = sonarDistLeft = sonarDistRight = 0;
  sonarDistCounter  = 0;
}

void SonarControl::setup(){
  Console.println(F("SonarControl::setup"));
  // sonar
}

bool SonarControl::triggeredLeft(){
  return ((sonarDistLeft != NO_ECHO) &&  (sonarDistLeft < sonarTriggerBelow));
}

bool SonarControl::triggeredRight(){
  return ((sonarDistRight != NO_ECHO) &&  (sonarDistRight < sonarTriggerBelow));
}

bool SonarControl::triggeredCenter(){
  return ((sonarDistCenter != NO_ECHO) &&  (sonarDistCenter < sonarTriggerBelow));
}

// call this in main loop
void SonarControl::run(){
  if (millis() < nextSonarTime) return;
  nextSonarTime = millis() + 1000;

  //if (enableRight)  sonarDistRight  = readHCSR04(pinSonarRightTrigger,  pinSonarRightEcho);
  //if (enableLeft)   sonarDistLeft   = readHCSR04(pinSonarLeftTrigger,   pinSonarLeftEcho);
  //if (enableCenter) sonarDistCenter = readHCSR04(pinSonarCenterTrigger, pinSonarCenterEcho);

  if ( (triggeredRight()) || (triggeredLeft()) || (triggeredCenter()) ){
    sonarDistCounter++;
    print();
  }
}


void SonarControl::print(){
  Console.print(F("sonar L,C,R="));
  Console.print(sonarDistLeft);
  Console.print(sonarDistCenter);
  Console.print(sonarDistRight);
  Console.println();
}


void SonarControl::read(){
}


