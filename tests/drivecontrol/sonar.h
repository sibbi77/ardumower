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

// Ardumower ultrasonic sensor

#ifndef SONAR_H
#define SONAR_H



class SonarControl
{
  public:
    bool enableLeft;
    bool enableRight;
    bool enableCenter;
    int sonarTriggerBelow ;    // ultrasonic sensor trigger distance
    unsigned int sonarDistCenter ;
    unsigned int sonarDistRight ;
    unsigned int sonarDistLeft ;
    unsigned int sonarDistCounter ;
    unsigned long sonarObstacleTimeout ;
    bool triggeredCenter();
    bool triggeredLeft();
    bool triggeredRight();
    bool triggeredAny();
    SonarControl();
    void setup();
    void run();
    virtual void read();
    void print();
  private:
    unsigned long nextSonarTime;
    // ----- driver -----
    virtual int driverReadLeftDistanceCm();
    virtual int driverReadRightDistanceCm();
    virtual int driverReadCenterDistanceCm();
};




#endif

