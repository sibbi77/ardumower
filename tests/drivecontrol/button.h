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

// Ardumower ON/start button


#ifndef BUTTON_H
#define BUTTON_H


class ButtonControl
{
  public:
    bool pressed;
    int beepCounter;
    ButtonControl();
    void run();
    void setup();
    void resetBeepCounter();
    void setBeepCount(int count);
  private:
    int tempBeepCounter;
    unsigned long nextButtonTime;
    // --- driver ---
    virtual bool driverButtonPressed();
};



#endif

