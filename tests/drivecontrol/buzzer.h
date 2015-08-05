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

// Ardumower buzzer (play 'beep codes')


#ifndef BUZZER_H
#define BUZZER_H



// beep types ('tones')
enum {
  BEEP_MUTE,
  BEEP_SHORT,
  BEEP_LONG
};

// beep codes ('melodies')
enum {
  BC_SILENCE,
  BC_SHORT,
  BC_SHORT_SHORT,
  BC_SHORT_SHORT_SHORT,
  BC_LONG,
  BC_LONG_SHORT_SHORT,
  BC_LONG_LONG,
  BC_LONG_SHORT_LONG,
};

class BuzzerControl
{
  public:
    BuzzerControl();
    // play a 'melody' (non-blocking)
    void play(int beepCode);
    bool isPlaying();
    void setup();
    void run();
  private:
    unsigned long nextBeepTime;
    int beepCodeToneIdx;
    int beepCodeIdx;
    // --- driver ---
    virtual void driverNoTone();
    virtual void driverTone(int frequencyHz);
};




#endif

