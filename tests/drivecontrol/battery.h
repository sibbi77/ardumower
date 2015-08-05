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

// Ardumower battery/charger management
// * battery, charging voltage, charging current
// * charging relay, battery switch
// * statistics

#ifndef BATTERY_H
#define BATTERY_H

#include "common.h"


class BatteryControl
{
  public:
    bool enableMonitor;
    float batFactor       ;     // battery conversion factor
    float batGoHomeIfBelow;     // drive home voltage (Volt)
    float batSwitchOffIfBelow;  // switch off battery if below voltage (Volt)
    int batSwitchOffIfIdle ;      // switch off battery if idle (minutes)
    float batChgFactor       ;     // battery conversion factor
    float batFull         ;      // battery reference Voltage (fully charged)
    float batChargingCurrentMax ; // maximum current your charger can devliver
    float batFullCurrent   ; // current flowing when battery is fully charged
    float startChargingIfBelow; // start charging if battery Voltage is below
    unsigned long chargingTimeoutMinutes; // safety timer for charging (minutes)
    int batADC;
    float chgFactor       ;     // charge current conversion factor
    float batVoltage ;  // battery voltage (Volt)
    float batRefFactor ;
    float batCapacity ; // battery capacity (mAh)
    float chgVoltage ;  // charge voltage (Volt)
    float chgCurrent ;  // charge current  (Ampere)
    int batteryReadCounter;
    bool chargeRelayEnabled;
    int idleTimeSec;    // number of seconds robot is idle (no user-interaction and no mowing)
    unsigned long chargingStartTimeMinutes; // start time of charging
    BatteryControl();
    void setup();
    virtual void run();
    virtual void read();  // read battery/charger voltage, current
    void print();
    bool chargerConnected();
    bool isCharging();
    int getChargingTimeMinutes();
    bool robotShouldGoHome();
    bool robotShouldCharge();
    void setBatterySwitch(bool state);
    void enableChargingRelay(bool state);
    bool robotShouldSwitchOff();
  private:
    unsigned long nextBatteryTime;
    // ----- driver -----
    virtual void driverSetBatterySwitch(bool state);
    virtual void driverSetChargeRelay(bool state);
    virtual int driverReadBatteryVoltageADC();
    virtual int driverReadChargeVoltageADC();
    virtual int driverReadChargeCurrentADC();
    virtual int driverReadVoltageMeasurementADC();
};




#endif

