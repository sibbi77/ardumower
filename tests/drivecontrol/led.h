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

// Ardumower LEDs  (play LED 'sequences')

#ifndef LED_H
#define LED_H


// playable LED sequences
enum {
  LED_SEQ_OFF,
  LED_SEQ_GREEN_ON,
  LED_SEQ_ORANGE_ON,
  LED_SEQ_ORANGE_BLINK,
  LED_SEQ_RED_BLINK,
  LED_SEQ_RED_ON,
};

// LED types
enum {
  LED_ARDUINO,
  LED_DUAL_RED,
  LED_DUAL_GREEN,
};

/*
LED Anzeigen - Hardware:
Led 3V grün - Betriebsanzeige Versorgungsspannung
LED 24V grün - Betriebsanzeige Versorgungsspannung R2 für 12 V anpassen
LED 5V grün - Betriebsanzeige Versorgungsspannung
LED Station - gelb - Statusanzeige ob Mover in Ladestation

LED Anzeigen - Software:
Dual LED z.B:
Dual LED grün. Dauerlicht - Mover innerhalb Perimeter
Dual LED grün+rot = Orange - Dauerlicht Mover außerhalb Perimeter
Dual LED grün+rot = Orange - Blinkt Mover findet Perimeter nicht oder sucht danach
Dual LED rot = Blinkt Fehler kurzseitig Überlast Antriebmotor Treibe oder Mähmotor.
Dual LED rot - Dauerlicht - Fehler muß für weiterfahren durch drücken
*/



class LEDControl
{
  public:
    LEDControl();
    void playSequence(int sequenceIdx);
    void run();
    void setup();
  private:
    unsigned long nextLEDTime;
    int ledSequenceIdx;
    bool onState;
    // ---- driver ---
    virtual void driverSetLED(int LEDidx, bool state);
};


#endif


