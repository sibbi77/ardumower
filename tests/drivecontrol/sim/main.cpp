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


// Ardumower simulator
// Requires: CodeBlocks and OpenCV  
// (IDE package can be downloaded here: 
//  http://wiki.ardumower.de/index.php?title=Sensor_fusion#SLAM_simulation )

// Sim     -- simulator environment (simulation time step, speed etc.)
// World   -- simulator world (garden with perimeter loop etc.)
// Robot   -- simulator robot

//#include <stdio.h>
#include "../objects.h"


void setup(){
  Robot.setup();
}

void loop(){
  Robot.loop();
}

int main()
{
  setup();
  while (true){
    //char key = cvWaitKey( 10 );
    //if (key == 27) return 0;
    loop();
  }
}


