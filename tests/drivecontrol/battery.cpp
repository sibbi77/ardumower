#include "common.h"
#include "objects.h"
#include "battery.h"



BatteryControl::BatteryControl(){
  nextBatteryTime = batteryReadCounter = 0;
  chargingStartTimeMinutes = 0;
  batVoltage = 24;
  chgVoltage = 0;
  idleTimeSec = 0;
  enableMonitor = false;
  chargeRelayEnabled = false;
  batFactor       = 0.495;      // battery conversion factor  / 10 due to arduremote bug, can be removed after fixing (look in robot.cpp)
  batChgFactor    = 0.495;      // battery conversion factor  / 10 due to arduremote bug, can be removed after fixing (look in robot.cpp)
  batFull          =29.4;      // battery reference Voltage (fully charged) PLEASE ADJUST IF USING A DIFFERENT BATTERY VOLTAGE! FOR a 12V SYSTEM TO 14.4V
  batChargingCurrentMax =1.6;  // maximum current your charger can devliver
  batFullCurrent  = 0.3;      // current flowing when battery is fully charged
  batGoHomeIfBelow = 23.7;     // drive home voltage (Volt)
  startChargingIfBelow = 27.0; // start charging if battery Voltage is below
  chargingTimeoutMinutes = 210; // safety timer for charging (minutes)
  chgFactor       = 39;         // charge current conversion factor   - Empfindlichkeit nimmt mit ca. 39/V Vcc ab
}

void BatteryControl::setup(){
  Console.println(F("BatteryControl::setup"));

  setBatterySwitch(HIGH);
}

void BatteryControl::setBatterySwitch(bool state){
  Console.print(F("BatteryControl::setBatterySwitch "));
  Console.println(state);
  driverSetBatterySwitch(state);
}

// call this in main loop
void BatteryControl::run(){
  if (millis() < nextBatteryTime) return;
  nextBatteryTime = millis() + 1000;
  read();
  if (batteryReadCounter == 10) {
    batteryReadCounter = 0;
    print();
  }
  if ( (chargerConnected()) && (robotShouldCharge()) ) {
    enableChargingRelay(true);
  } else {
    enableChargingRelay(false);
  }
  idleTimeSec++;
}

bool BatteryControl::isCharging(){
  return chargeRelayEnabled;
}

bool BatteryControl::robotShouldGoHome(){
  return ((enableMonitor) &&  (batVoltage < batGoHomeIfBelow));
}

bool BatteryControl::robotShouldSwitchOff(){
  return (     (enableMonitor)
           &&  (  (idleTimeSec > batSwitchOffIfIdle * 60) || (batVoltage < batSwitchOffIfBelow) )
         );
}

bool BatteryControl::robotShouldCharge(){
  return (     (enableMonitor) &&  (batVoltage < startChargingIfBelow)
               && (getChargingTimeMinutes() < chargingTimeoutMinutes)
         );
}

// read battery/charger voltage, current
void BatteryControl::read(){
  batteryReadCounter++;

  if ((abs(chgCurrent) > 0.04) && (chgVoltage > 5)){
    // charging
    batCapacity += (chgCurrent / 36.0);
    // convert to double
    batADC = driverReadBatteryVoltageADC();
    double batvolt = (double)batADC * batFactor / 10;  // / 10 due to arduremote bug, can be removed after fixing
    int chgADC = driverReadChargeVoltageADC();
    double chgvolt = (double)chgADC * batChgFactor / 10;  // / 10 due to arduremote bug, can be removed after fixing
    // low-pass filter
    double accel = 0.01;
    if (abs(batVoltage-batvolt)>5)   batVoltage = batvolt; else batVoltage = (1.0-accel) * batVoltage + accel * batvolt;
    if (abs(chgVoltage-chgvolt)>5)   chgVoltage = chgvolt; else chgVoltage = (1.0-accel) * chgVoltage + accel * chgvolt;

    // Berechnung fÃ¼r Ladestromsensor INA169 board              wenn chgSelection =2
    double current = ((double)((int)(driverReadChargeCurrentADC())));
    chgCurrent = max(0, (current * 5) / 1023 / (10 * 0.1)  );                               // Ampere berechnen RL = 10k    Is = (Vout x 1k) / (RS x RL)
  }
}

bool BatteryControl::chargerConnected(){
  return ((chgVoltage > 5.0)  && (batVoltage > 8));
}

int BatteryControl::getChargingTimeMinutes(){
  if (!chargeRelayEnabled) return 0;
  return (Timer.powerTimeMinutes - chargingStartTimeMinutes);
}

void BatteryControl::enableChargingRelay(bool state){
  if (state == chargeRelayEnabled) return;
  Console.print(F("BatteryControll::enableChargingRelay "));
  Console.println(state);
  chargeRelayEnabled = state;
  driverSetChargeRelay(state);
  if (state) chargingStartTimeMinutes = Timer.powerTimeMinutes;
}


void BatteryControl::print(){
  Console.print(F("batVoltage="));
  Console.print(batVoltage);
  Console.print(F("  chgVoltage="));
  Console.print(chgVoltage);
  Console.print(F("  chgCurrent="));
  Console.print(chgCurrent);
  Console.print(F("  chargerConnected="));
  Console.print(chargerConnected());
  Console.print(F("  chargeRelayEnabled="));
  Console.print(chargeRelayEnabled);
  Console.print(F("  chargingTimeMinutes="));
  Console.print(getChargingTimeMinutes());
  Console.println();
}


void BatteryControl::driverSetBatterySwitch(bool state){
}

void BatteryControl::driverSetChargeRelay(bool state){
}

int BatteryControl::driverReadBatteryVoltageADC(){
}

int BatteryControl::driverReadChargeVoltageADC(){
}

int BatteryControl::driverReadChargeCurrentADC(){
}

int BatteryControl::driverReadVoltageMeasurementADC(){
}


