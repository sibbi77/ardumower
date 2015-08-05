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

bool SonarControl::triggeredAny(){
  return ( (triggeredCenter()) || (triggeredLeft()) || (triggeredRight())  );
}

// call this in main loop
void SonarControl::run(){
  if (millis() < nextSonarTime) return;
  nextSonarTime = millis() + 1000;

  read();

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
  if (enableRight)  sonarDistRight  = driverReadRightDistanceCm();
  if (enableLeft)   sonarDistLeft   = driverReadLeftDistanceCm();
  if (enableCenter) sonarDistCenter = driverReadCenterDistanceCm();
}

int SonarControl::driverReadLeftDistanceCm(){
}

int SonarControl::driverReadRightDistanceCm(){
}

int SonarControl::driverReadCenterDistanceCm(){
}


