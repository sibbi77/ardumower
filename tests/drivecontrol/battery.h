#ifndef BATTERY_H
#define BATTERY_H

// Ardumower battery/charger management

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
  private:
    void setBatterySwitch(bool state);
    void enableChargingRelay(bool state);
    bool robotShouldSwitchOff();
    bool robotShouldCharge();
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

