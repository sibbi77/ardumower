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


