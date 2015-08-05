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

#include "common.h"
#include "objects.h"
#include "modelrc.h"



void ModelReceiverControl::setup(){
  Console.println(F("ModelReceiver::setup"));
}

ModelReceiverControl::ModelReceiverControl(){
  enable=false;
}

void ModelReceiverControl::run(){
  if (!enable) return;
  // control motor by R/C receiver
  int steer = ((double)Motor.motorSpeedMaxRpm/2) * (((double)remoteSteer)/100.0);
  if (remoteSpeed < 0) steer *= -1;
    
  int motorLeftSpeedRpmSet  = ((double)Motor.motorSpeedMaxRpm) * (((double)remoteSpeed)/100.0) - steer; 
  int motorRightSpeedRpmSet = ((double)Motor.motorSpeedMaxRpm) * (((double)remoteSpeed)/100.0) + steer; 
  Motor.setSpeedRpm(motorLeftSpeedRpmSet, motorRightSpeedRpmSet);  
}

void ModelReceiverControl::print(){
   Console.print(F("RC:"));
   Console.print(remoteSpeed);
   Console.print(",");
   Console.print(remoteSteer);
   Console.print(",");
   Console.print(remoteMow);
}


  
