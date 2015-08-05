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

// Ardumower Arduino Mega robot classes

#include "mower.h"


// ------ pins---------------------------------------
// Warning: If you change pinout, interrupt configuration (odometry, R/C, rpm) need to be changed too!

#define pinMotorEnable  37         // EN motors enable
#define pinMotorLeftPWM 5          // M1_IN1 left motor PWM pin
#define pinMotorLeftDir 31         // M1_IN2 left motor Dir pin
#define pinMotorLeftSense A1       // M1_FB  left motor current sense
#define pinMotorLeftFault 25       // M1_SF  left motor fault
                                                             
#define pinMotorRightPWM  3        // M2_IN1 right motor PWM pin
#define pinMotorRightDir 33        // M2_IN2 right motor Dir pin
#define pinMotorRightSense A0      // M2_FB  right motor current sense
#define pinMotorRightFault 27      // M2_SF  right motor fault
                                    
#define pinMotorMowPWM 2           // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
#define pinMotorMowDir 29          // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
#define pinMotorMowSense A3        // M1_FB  mower motor current sense  
#define pinMotorMowFault 26        // M1_SF  mower motor fault   (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowEnable 28       // EN mower motor enable      (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowRpm A11
    
#define pinBumperLeft 39           // bumper pins
#define pinBumperRight 38

#define pinDropLeft 45           // drop pins                                                                                          Dropsensor - Absturzsensor
#define pinDropRight 23          // drop pins                                                                                          Dropsensor - Absturzsensor

#define pinSonarCenterTrigger 24   // ultrasonic sensor pins
#define pinSonarCenterEcho 22
#define pinSonarRightTrigger 30    
#define pinSonarRightEcho 32
#define pinSonarLeftTrigger 34         
#define pinSonarLeftEcho 36
#define pinPerimeterRight A4       // perimeter
#define pinPerimeterLeft A5

#define pinLED 13                  // LED
#define pinLEDDuoRed 7            
#define pinLEDDuoGreen 6            
#define pinBuzzer 53               // Buzzer
#define pinTilt 35                 // Tilt sensor (required for TC-G158 board)
#define pinButton 51               // digital ON/OFF button
#define pinBatteryVoltage A2       // battery voltage sensor
#define pinBatterySwitch 4         // battery-OFF switch   
#define pinChargeVoltage A9        // charging voltage sensor
#define pinChargeCurrent A8        // charge current sensor
#define pinChargeRelay 50          // charge relay
#define pinRemoteMow 12            // remote control mower motor
#define pinRemoteSteer 11          // remote control steering 
#define pinRemoteSpeed 10          // remote control speed
#define pinRemoteSwitch 52         // remote control switch
#define pinVoltageMeasurement A7   // test pin for your own voltage measurements
#ifdef __AVR__
  #define pinOdometryLeft A12      // left odometry sensor
  #define pinOdometryLeft2 A13     // left odometry sensor (optional two-wire)
  #define pinOdometryRight A14     // right odometry sensor 
  #define pinOdometryRight2 A15    // right odometry sensor (optional two-wire)  
#else
  #define pinOdometryLeft DAC0     // left odometry sensor
  #define pinOdometryLeft2 DAC1    // left odometry sensor (optional two-wire)
  #define pinOdometryRight CANRX   // right odometry sensor  
  #define pinOdometryRight2 CANTX  // right odometry sensor (optional two-wire)  
#endif
#define pinLawnFrontRecv 40        // lawn sensor front receive
#define pinLawnFrontSend 41        // lawn sensor front sender 
#define pinLawnBackRecv 42         // lawn sensor back receive
#define pinLawnBackSend 43         // lawn sensor back sender 
#define pinUserSwitch1 46          // user-defined switch 1
#define pinUserSwitch2 47          // user-defined switch 2
#define pinUserSwitch3 48          // user-defined switch 3
#define pinRain 44                 // rain sensor
// RTC, IMU (compass/gyro/accel): I2C  (SCL, SDA) 
// GPS: Serial3 (TX3, RX3)
// Bluetooth: Serial2 (TX2, RX2)
// WLAN: Serial1 (TX1, RX1) 

// ------- baudrates---------------------------------
#define BAUDRATE 19200            // serial output baud rate
#define PFOD_BAUDRATE 19200       // pfod app serial output baud rate
#define PFOD_PIN 1234             // Bluetooth pin


// ------------------------------------------

void MowerSettings::setup(){  
  //randomSeed( time(NULL) );
  Wire.begin();
  Console.begin(BAUDRATE);    
  // --------------------------- config begin ----------------------------------------  
  // --- gear motors ----
  Motor.motorSpeedMaxPwm = 255;
  Motor.motorLeftSwapDir = false;
  Motor.motorRightSwapDir = false;
  Motor.motorSenseRightScale = 9.3;  // motor right sense scale (mA=(ADC-zero) * scale)
  Motor.motorSenseLeftScale  = 9.3;  // motor left sense scale  (mA=(ADC-zero) * scale)
  Motor.motorSpeedMaxRpm = 25;
  Motor.motorEfficiencyMin = 200;
  Motor.enableStallDetection = true;
  Motor.enableErrorDetection = false;
  Motor.odometryTicksPerRevolution = 1060;   // encoder ticks per one full resolution
  Motor.odometryTicksPerCm = 13.49;    // encoder ticks per cm
  Motor.odometryWheelBaseCm = 36;    // wheel-to-wheel distance (cm)
  Motor.motorLeftPID.Kp       = 0.4;    // PID speed controller
  Motor.motorLeftPID.Ki       = 0.0;
  Motor.motorLeftPID.Kd       = 0.0;
  // --- mower motors ----
  MotorMow.motorMowSpeedMaxPwm = 255;
  MotorMow.motorSenseScale = 9.3;
  MotorMow.motorMowPowerMax = 100;
  MotorMow.enableStallDetection = true;
  MotorMow.enableErrorDetection = false;
  Robot.motorMowCircleAbovePower = 70;
  // --- perimeter motors ----
  Perimeter.enable = true;
  Robot.perimeterPID.Kp    = 160;  // perimeter PID controller
  Robot.perimeterPID.Ki    = 4;
  Robot.perimeterPID.Kd    = 50;
  // --- sonar ----
  Sonar.enableCenter = true;
  Sonar.enableLeft = false;
  Sonar.enableRight = false;
  // --- battery ----
  Battery.enableMonitor = true;
  Battery.batFull = 29.4;
  //Battery.batGoHomeIfBelow = 29.39;
  Battery.batGoHomeIfBelow = 23.7;
  Battery.batVoltage = Battery.batFull;  
  // ------------------------------------------ config end ----------------------------------------------
 
     
// keep battery switched ON
  pinMode(pinBatterySwitch, OUTPUT);
  digitalWrite(pinBatterySwitch, HIGH);
  
  // LED, buzzer, battery
  pinMode(pinLED, OUTPUT);    
  pinMode(pinBuzzer, OUTPUT);    
  digitalWrite(pinBuzzer,0);    
  pinMode(pinBatteryVoltage, INPUT);        
  pinMode(pinChargeCurrent, INPUT);          
  pinMode(pinChargeVoltage, INPUT);            
  pinMode(pinChargeRelay, OUTPUT);  
  
  // left wheel motor
  pinMode(pinMotorEnable, OUTPUT);  
  digitalWrite(pinMotorEnable, HIGH);
  pinMode(pinMotorLeftPWM, OUTPUT);
  pinMode(pinMotorLeftDir, OUTPUT);   
  pinMode(pinMotorLeftSense, INPUT);     
  pinMode(pinMotorLeftFault, INPUT);    
  
  // right wheel motor
  pinMode(pinMotorRightPWM, OUTPUT);
  pinMode(pinMotorRightDir, OUTPUT); 
  pinMode(pinMotorRightSense, INPUT);       
  pinMode(pinMotorRightFault, INPUT);  
  
  // mower motor
  pinMode(pinMotorMowDir, OUTPUT); 
  pinMode(pinMotorMowPWM, OUTPUT);     
  pinMode(pinMotorMowSense, INPUT);     
  pinMode(pinMotorMowRpm, INPUT);    
  pinMode(pinMotorMowEnable, OUTPUT);
  digitalWrite(pinMotorMowEnable, HIGH);  
  pinMode(pinMotorMowFault, INPUT);      
    
  // lawn sensor
  pinMode(pinLawnBackRecv, INPUT);
  pinMode(pinLawnBackSend, OUTPUT);
  pinMode(pinLawnFrontRecv, INPUT);
  pinMode(pinLawnFrontSend, OUTPUT);  
  
  // perimeter
  pinMode(pinPerimeterRight, INPUT);    
  pinMode(pinPerimeterLeft, INPUT);        
  
  // button
  pinMode(pinButton, INPUT);
  pinMode(pinButton, INPUT_PULLUP);

  // bumpers
  pinMode(pinBumperLeft, INPUT);
  pinMode(pinBumperLeft, INPUT_PULLUP);
  pinMode(pinBumperRight, INPUT);
  pinMode(pinBumperRight, INPUT_PULLUP);
 
 // drops
  pinMode(pinDropLeft, INPUT);                                                                                                         // Dropsensor - Absturzsensor - Deklariert als Eingang
  pinMode(pinDropLeft, INPUT_PULLUP);                                                                                                  // Dropsensor - Absturzsensor - Intern Pullab Widerstand aktiviert (Auslösung erfolgt gegen GND)
  pinMode(pinDropRight, INPUT);                                                                                                        // Dropsensor - Absturzsensor - Deklariert als Eingang 
  pinMode(pinDropRight, INPUT_PULLUP);                                                                                                 // Dropsensor - Absturzsensor - Intern Pullab Widerstand aktiviert (Auslösung erfolgt gegen GND)
  
  // sonar
  pinMode(pinSonarCenterTrigger, OUTPUT); 
  pinMode(pinSonarCenterEcho, INPUT); 
  pinMode(pinSonarLeftTrigger, OUTPUT); 
  pinMode(pinSonarLeftEcho, INPUT); 
  pinMode(pinSonarRightTrigger, OUTPUT); 
  pinMode(pinSonarRightEcho, INPUT); 
  
  // rain
  pinMode(pinRain, INPUT);
        
  // R/C
  pinMode(pinRemoteMow, INPUT);
  pinMode(pinRemoteSteer, INPUT);
  pinMode(pinRemoteSpeed, INPUT); 
  pinMode(pinRemoteSwitch, INPUT);       
  
  // R/C interrupt configuration
  PCICR |= (1<<PCIE0);
  PCMSK0 |= (1<<PCINT4);
  PCMSK0 |= (1<<PCINT5);
  PCMSK0 |= (1<<PCINT6);
  PCMSK0 |= (1<<PCINT1);    

  // odometry
  pinMode(pinOdometryLeft, INPUT_PULLUP);  
  pinMode(pinOdometryLeft2, INPUT_PULLUP);    
  pinMode(pinOdometryRight, INPUT_PULLUP);
  pinMode(pinOdometryRight2, INPUT_PULLUP);  
  
   // odometry interrupt configuration
  PCICR |= (1<<PCIE2);
  PCMSK2 |= (1<<PCINT20);
  PCMSK2 |= (1<<PCINT22);            
  
  // user switches
  pinMode(pinUserSwitch1, OUTPUT);
  pinMode(pinUserSwitch2, OUTPUT);
  pinMode(pinUserSwitch3, OUTPUT);   
  
  // other
  pinMode(pinVoltageMeasurement, INPUT);

  TCCR3B = (TCCR3B & 0xF8) | 0x02;    // set PWM frequency 3.9 Khz (pin2,3,5)

  // ADC
  ADCMan.setup();
  ADCMan.setCapture(pinChargeCurrent, 1, true);//Aktivierung des LaddeStrom Pins beim ADC-Managers      
  ADCMan.setCapture(pinMotorMowSense, 1, true);
  ADCMan.setCapture(pinMotorLeftSense, 1, true);
  ADCMan.setCapture(pinMotorRightSense, 1, true);
  ADCMan.setCapture(pinBatteryVoltage, 1, false);
  ADCMan.setCapture(pinChargeVoltage, 1, false);  
  ADCMan.setCapture(pinVoltageMeasurement, 1, false);    
  Perimeter.setPins(pinPerimeterLeft, pinPerimeterRight);    
}

// ------------------------------------------

volatile static boolean odometryLeftLastState;
volatile static boolean odometryRightLastState;       
volatile static unsigned long odometryLeftLastHighTime = 0;
volatile static unsigned long odometryRightLastHighTime = 0;
volatile static unsigned long odometryLeftTickTime = 0;
volatile static unsigned long odometryRightTickTime = 0;


// odometry interrupt handler
ISR(PCINT2_vect, ISR_NOBLOCK){
//  ISR(PCINT2_vect){
  unsigned long timeMicros = micros();
  boolean odometryLeftState = digitalRead(pinOdometryLeft);
  boolean odometryRightState = digitalRead(pinOdometryRight);      
  if (odometryLeftState != odometryLeftLastState){    
    if (odometryLeftState){ // pin1 makes LOW->HIGH transition
      if (timeMicros > odometryLeftLastHighTime) odometryLeftTickTime = timeMicros - odometryLeftLastHighTime;
      if (Motor.motorLeftPWMCurr >=0) Motor.odometryLeftTicks ++; else Motor.odometryLeftTicks --;           
      odometryLeftLastHighTime = timeMicros;      
    } else {
      //odometryLeftLowTime = timeMicros;
    }
    odometryLeftLastState = odometryLeftState;
  } 
  if (odometryRightState != odometryRightLastState){
    if (odometryRightState){ // pin1 makes LOW->HIGH transition
      if (timeMicros > odometryRightLastHighTime) odometryRightTickTime = timeMicros - odometryRightLastHighTime;
      if (Motor.motorRightPWMCurr >=0) Motor.odometryRightTicks ++; else Motor.odometryRightTicks --;    
      odometryRightLastHighTime = timeMicros;
    } else {
      //odometryRightLowTime = timeMicros;
    }
    odometryRightLastState = odometryRightState;
  }  
}

static unsigned long remoteSteerLastTime ;
static unsigned long remoteSpeedLastTime ;
static unsigned long remoteMowLastTime ;        
static boolean remoteSteerLastState ;
static boolean remoteSpeedLastState ;        
static boolean remoteMowLastState ;


// RC remote control helper
// convert ppm time (us) to percent (-100..+100)
// ppmtime: zero stick pos: 1500 uS 		
//          right stick pos: 2000 uS 		
//          left stick pos: 1000 uS
int rcValue(int ppmTime){
  int value = (int) (((double)((ppmTime) - 1500)) / 3.4);
  if ((value < 5) && (value > -5)) value = 0;  //  ensures exact zero position
  return value;
}

// remote control (RC) ppm signal change interrupt
ISR(PCINT0_vect){   
  unsigned long timeMicros = micros();
  boolean remoteSpeedState = digitalRead(pinRemoteSpeed);
  boolean remoteSteerState = digitalRead(pinRemoteSteer);
  boolean remoteMowState = digitalRead(pinRemoteMow);    
  if (remoteSpeedState != remoteSpeedLastState) {    
    remoteSpeedLastState = remoteSpeedState;
    if (remoteSpeedState) remoteSpeedLastTime = timeMicros; else ModelReceiver.remoteSpeed = rcValue(timeMicros - remoteSpeedLastTime);
  }
  if (remoteSteerState != remoteSteerLastState) {    
    remoteSteerLastState = remoteSteerState;
    if (remoteSteerState) remoteSteerLastTime = timeMicros; else ModelReceiver.remoteSteer = rcValue(timeMicros - remoteSteerLastTime);
  }      
  if (remoteMowState != remoteMowLastState) {    
    remoteMowLastState = remoteMowState;
    if (remoteMowState) remoteMowLastTime = timeMicros; else ModelReceiver.remoteMow = rcValue(timeMicros - remoteMowLastTime);
  }
}

// ---- I2C helpers --------------------------------------------------------------
void I2CwriteTo(uint8_t device, uint8_t address, uint8_t val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

void I2CwriteTo(uint8_t device, uint8_t address, int num, uint8_t buff[]) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   for (int i=0; i < num; i++){
     Wire.write(buff[i]);        // send value to write
   }
   Wire.endTransmission(); //end transmission
}

int I2CreadFrom(uint8_t device, uint8_t address, uint8_t num, uint8_t buff[], int retryCount) {
  int i = 0;
  for (int j=0; j < retryCount+1; j++){
    i=0;
    Wire.beginTransmission(device); //start transmission to device 
    Wire.write(address);        //sends address to read from
    Wire.endTransmission(); //end transmission
  
    //Wire.beginTransmission(device); //start transmission to device (initiate again)
    Wire.requestFrom(device, num);    // request 6 bytes from device
  
    while(Wire.available())    //device may send less than requested (abnormal)
    {  
      buff[i] = Wire.read(); // receive a byte
      i++;
    }
    //Wire.endTransmission(); //end transmission
    if (num == i) return i;
    if (j != retryCount) delay(3);
  }
  return i;
}


// MC33926 motor driver
// Check http://forum.pololu.com/viewtopic.php?f=15&t=5272#p25031 for explanations.
//(8-bit PWM=255, 10-bit PWM=1023)
// IN1 PinPWM         IN2 PinDir
// PWM                L     Forward
// nPWM               H     Reverse
void setMC33926(int pinDir, int pinPWM, int speed){
  if (speed < 0){
    digitalWrite(pinDir, HIGH) ;
    analogWrite(pinPWM, 255-((byte)abs(speed)));
  } else {
    digitalWrite(pinDir, LOW) ;
    analogWrite(pinPWM, ((byte)speed));
  }
}

void MowerMotor::driverSetPWM(int leftMotorPWM, int rightMotorPWM){
  setMC33926(pinMotorLeftDir, pinMotorLeftPWM, leftMotorPWM);
  setMC33926(pinMotorRightDir, pinMotorRightPWM, rightMotorPWM);
}

int MowerMotor::driverReadLeftCurrentADC(){
  return ADCMan.read(pinMotorLeftSense);
}

int MowerMotor::driverReadRightCurrentADC(){
  return ADCMan.read(pinMotorRightSense);
}

bool MowerMotor::driverReadRightFault(){
  return (digitalRead(pinMotorLeftFault) == LOW);
}

bool MowerMotor::driverReadLeftFault(){
  return (digitalRead(pinMotorRightFault) == LOW);
}

void MowerMotor::driverResetFault(){
  digitalWrite(pinMotorEnable, LOW);
  digitalWrite(pinMotorEnable, HIGH);  
}

// ------------------------------------------

void MowerMotorMow::driverSetPWM(int pwm){
  setMC33926(pinMotorMowDir, pinMotorMowPWM, pwm);
}

int MowerMotorMow::driverReadCurrentADC(){
  return ADCMan.read(pinMotorMowSense);
}

bool MowerMotorMow::driverReadFault(){
  return (digitalRead(pinMotorMowFault)==LOW);
}

void MowerMotorMow::driverResetFault(){
  digitalWrite(pinMotorMowEnable, LOW);
  digitalWrite(pinMotorMowEnable, HIGH);
}


// ------------------------------------------

#define NO_ECHO 0
// ultrasonic sensor max echo time (WARNING: do not set too high, it consumes CPU time!)
#define MAX_ECHO_TIME 3000
//#define MIN_ECHO_TIME 350                  
#define MIN_ECHO_TIME 0


// HC-SR04 ultrasonic sensor driver
unsigned int readHCSR04(int triggerPin, int echoPin){
  unsigned int uS;  
  digitalWrite(triggerPin, LOW);   
  delayMicroseconds(2); 
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(triggerPin, LOW);
  // ultrasonic sensor max echo time (WARNING: do not set too high, it consumes CPU time!)
  uS = pulseIn(echoPin, HIGH, MAX_ECHO_TIME + 1000);  
  if (uS > MAX_ECHO_TIME) uS = NO_ECHO;
    else if (uS < MIN_ECHO_TIME) uS = NO_ECHO;
  return (uS/2) / 29.1; // convert uS to cm
}

int MowerSonar::driverReadCenterDistanceCm(){
  return readHCSR04(pinSonarCenterTrigger, pinSonarCenterEcho);
}

int MowerSonar::driverReadLeftDistanceCm(){
  return readHCSR04(pinSonarLeftTrigger, pinSonarLeftEcho);
}

int MowerSonar::driverReadRightDistanceCm(){
  return readHCSR04(pinSonarRightTrigger, pinSonarRightEcho);
}

// ------------------------------------------

#define DS1307_ADDRESS B1101000

// DS1307 real time driver
boolean readDS1307(datetime_t &dt){
  byte buf[8];  
  if (I2CreadFrom(DS1307_ADDRESS, 0x00, 8, buf, 3) != 8) {
    Console.println(F("ERROR: DS1307 comm"));    
    return false;
  }      
  if (   ((buf[0] >> 7) != 0) || ((buf[1] >> 7) != 0) || ((buf[2] >> 7) != 0) || ((buf[3] >> 3) != 0) 
      || ((buf[4] >> 6) != 0) || ((buf[5] >> 5) != 0) || ((buf[7] & B01101100) != 0) ) {    
    Console.println(F("ERROR: DS1307 data1"));    
    return false;
  }
  datetime_t r;
  r.time.minute    = 10*((buf[1] >>4) & B00000111) + (buf[1] & B00001111);
  r.time.hour      = 10*((buf[2] >>4) & B00000111) + (buf[2] & B00001111);
  r.date.dayOfWeek = (buf[3] & B00000111);
  r.date.day       = 10*((buf[4] >>4) & B00000011) + (buf[4] & B00001111);
  r.date.month     = 10*((buf[5] >>4) & B00000001) + (buf[5] & B00001111);
  r.date.year      = 10*((buf[6] >>4) & B00001111) + (buf[6] & B00001111);
  if (    (r.time.minute > 59) || (r.time.hour > 23) || (r.date.dayOfWeek > 6)  
       || (r.date.month > 12)  || (r.date.day > 31)  || (r.date.day < 1)         
       || (r.date.month < 1)   || (r.date.year > 99) ){
    Console.println(F("ERROR: DS1307 data2"));    
    return false;
  }  
  r.date.year      += 2000;
  dt = r;
  return true;
}

boolean setDS1307(datetime_t &dt){
  byte buf[7];
  if (I2CreadFrom(DS1307_ADDRESS, 0x00, 7, buf, 3) != 7){
    Console.println(F("ERROR: DS1307 comm"));    
    return false;
  }
  buf[0] = buf[0] & B01111111; // enable clock
  buf[1] = ((dt.time.minute / 10) << 4) | (dt.time.minute % 10);
  buf[2] = ((dt.time.hour   / 10) << 4) | (dt.time.hour   % 10);
  buf[3] = dt.date.dayOfWeek;
  buf[4] = ((dt.date.day    / 10) << 4) | (dt.date.day    % 10);
  buf[5] = ((dt.date.month  / 10) << 4) | (dt.date.month  % 10);
  buf[6] = ((dt.date.year % 100  / 10) << 4) | (dt.date.year % 10);
  I2CwriteTo(DS1307_ADDRESS, 0x00, 7, buf);
  return true;
}


bool MowerTimer::driverReadRTC(datetime_t &dt){
  return readDS1307(dt); 
}

bool MowerTimer::driverSetRTC(datetime_t &dt){
  return setDS1307(dt);
}

// ------------------------------------------

void MowerBattery::driverSetBatterySwitch(bool state){
  digitalWrite(pinBatterySwitch, state);
}

void MowerBattery::driverSetChargeRelay(bool state){
  digitalWrite(pinChargeRelay, state);  
}

int MowerBattery::driverReadBatteryVoltageADC(){
  return ADCMan.read(pinBatteryVoltage);
}
 
int MowerBattery::driverReadChargeVoltageADC(){
  return ADCMan.read(pinChargeVoltage);  
}

int MowerBattery::driverReadChargeCurrentADC(){
  return ADCMan.read(pinChargeCurrent);    
}
 
int MowerBattery::driverReadVoltageMeasurementADC(){
  return ADCMan.read(pinVoltageMeasurement);  
}

// ------------------------------------------

void MowerBuzzer::driverNoTone(){
  noTone(pinBuzzer);
}

void MowerBuzzer::driverTone(int frequencyHz){
  tone(pinBuzzer, frequencyHz);
}

// ------------------------------------------

bool MowerButton::driverButtonPressed(){
  return (digitalRead(pinButton == LOW));
}

// ------------------------------------------

// initializes robot
MowerRobot::MowerRobot(){
}

char MowerRobot::readKey(){
  if (Console.available()) return ((char)Console.read());
  return 0;
}

void MowerRobot::run(){
  RobotControl::run();  
}

void MowerRobot::processKey(char key){
  RobotControl::processKey(key);
}


// ------------------------------------------

void MowerLED::setDriverLED(int LEDidx, bool state){
  switch (LEDidx){
    case LED_ARDUINO:    digitalWrite(pinLED, state); break;
    case LED_DUAL_RED:   digitalWrite(pinLEDDuoRed, state); break;
    case LED_DUAL_GREEN: digitalWrite(pinLEDDuoGreen, state); break;
  }
}



MowerLED LED;
MowerMotor Motor;
MowerMotorMow MotorMow;
MowerSettings Settings;
MowerPerimeter Perimeter;
MowerBattery Battery;
MowerBuzzer Buzzer;
MowerSonar Sonar;
MowerButton Button;
MowerTimer Timer;
MowerModelReceiver ModelReceiver;
MowerRobot Robot;


