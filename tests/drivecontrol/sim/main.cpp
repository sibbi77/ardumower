
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


