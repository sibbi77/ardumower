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

#include "arbitrator.h"

#define MAX_BEHAVIORS 30

// stored in increasing priority order
Behavior *behaviors[MAX_BEHAVIORS];    

    
Arbitrator::Arbitrator(){
  behaviorCount = 0;
  activeBehavior = nextBehavior = NULL;
  activeBehaviorIdx = nextBehaviorIdx = -1;
}  
 
void Arbitrator::addBehavior(Behavior *behavior){
  if (behaviorCount >= MAX_BEHAVIORS) {
    Console.println(F("ERROR: Arbitrator::add"));
    return;
  }
  Console.print(F("Arbitrator::addBehavior "));  
  Console.print(behaviorCount);
  Console.print(": ");  
  Console.println(behavior->name);
  behaviors[behaviorCount] = behavior;
  behaviorCount++;
}  
   
void Arbitrator::run(){
  Console.println(F("Arbitrator::run"));
  if (activeBehavior){
    activeBehavior->action();  
    Console.print(F("COMPLETED: "));
    Console.println(activeBehavior->name);    
  }
  activeBehavior = NULL;
  activeBehaviorIdx = -1;
  while (nextBehavior == NULL) {
    // no next behavior (no supression) => find out next behavior    
    Console.println(F("NO SUPPRESSION"));
    monitor();
  }
  activeBehaviorIdx=nextBehaviorIdx;    
  activeBehavior = nextBehavior;
  nextBehavior = NULL;    
  nextBehaviorIdx = -1;
  Console.print(F("-----CHANGED activeBehavior Idx: "));
  Console.print(activeBehaviorIdx);
  Console.print("  ");
  Console.print(activeBehavior->name);
  Console.println(F("-----"));
}
   
void Arbitrator::monitor() {
  //Console.println("Arbitrator::monitor");
  for (int idx=behaviorCount-1; idx >= 0; idx--) {   
    Behavior *behavior = behaviors[idx];
    
    /*Console.print("takeControl ");
    Console.print(idx);    
    Console.print("  ");    
    Console.println(behavior->name);    */
    
    if ( (behavior->enabled) && (idx > activeBehaviorIdx) && (behavior->takeControl()) ){
      setNextBehavior(behavior);
      //if (activeBehavior != NULL){    
      //  Console.println(F("Arbitrator::monitor suppressing"));
      //  activeBehavior->suppress(); 
      //}
      //nextBehavior = behavior;
      //nextBehaviorIdx = idx;             
      break;
    }
  }    
}

void Arbitrator::setNextBehavior(Behavior *behavior){
  //Console.print(F("Arbitrator::setNextBehavior "));
  //Console.println(behavior->name);
  if (activeBehavior != NULL){    
    Console.println(F("Arbitrator::monitor suppressing"));
    activeBehavior->suppress(); 
  }
  for (int idx=0; idx < behaviorCount; idx++){
    if (behaviors[idx] == behavior){
      nextBehavior = behavior;
      nextBehaviorIdx = idx;            
      break;
    }
  }  
}

