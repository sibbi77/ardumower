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


  
