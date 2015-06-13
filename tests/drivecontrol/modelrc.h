// model R/C receiver

#ifndef MODELRC_H
#define MODELRC_H


#include "common.h"

class ModelReceiverControl 
{
  public:
    bool enable;
    int remoteSteer ;  // range -100..100
    int remoteSpeed ;  // range -100..100      
    int remoteMow;  // range -100..100      
    ModelReceiverControl();    
    void setup();
    void run();
    void print();
};


#endif


