#include "common.h"
#include "led.h"


LEDControl::LEDControl(){
  nextLEDTime = 0;
  ledSequenceIdx = LED_SEQ_OFF;
  onState = 0;
}

void LEDControl::setup(){
  Console.println(F("LEDControl::setup"));
}

void LEDControl::run(){
  if (millis() < nextLEDTime) return;
  nextLEDTime = millis() + 500;
  switch (ledSequenceIdx){
    case LED_SEQ_OFF:
      driverSetLED(LED_ARDUINO, LOW);
      driverSetLED(LED_DUAL_RED, LOW);
      driverSetLED(LED_DUAL_GREEN, LOW);
      break;
    case LED_SEQ_GREEN_ON:
      driverSetLED(LED_ARDUINO, HIGH);
      driverSetLED(LED_DUAL_RED, LOW);
      driverSetLED(LED_DUAL_GREEN, HIGH);
      break;
    case LED_SEQ_ORANGE_ON:
      driverSetLED(LED_DUAL_RED, HIGH);
      driverSetLED(LED_DUAL_GREEN, HIGH);
      break;
    case LED_SEQ_ORANGE_BLINK:
      onState = !onState;
      driverSetLED(LED_DUAL_RED, onState);
      driverSetLED(LED_DUAL_GREEN, onState);
      break;
    case LED_SEQ_RED_BLINK:
      onState = !onState;
      driverSetLED(LED_ARDUINO, onState);
      driverSetLED(LED_DUAL_RED, onState);
      driverSetLED(LED_DUAL_GREEN, LOW);
      break;
    case LED_SEQ_RED_ON:
      driverSetLED(LED_DUAL_RED, HIGH);
      driverSetLED(LED_DUAL_GREEN, LOW);
      break;
  };
}

void LEDControl::driverSetLED(int LEDidx, bool state){
}

void LEDControl::playSequence(int sequenceIdx){
  ledSequenceIdx = sequenceIdx;
  nextLEDTime = 0;
}


