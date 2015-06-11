
// differential wheels drive controller experiments 
// Requires: Ardumower Mini, Arduino Mega


#include <EEPROM.h>  
#include <Wire.h>  

#include "objects.h"


void setup(){
  Robot.setup();      
}


void loop(){  
  Robot.loop();
    
}



