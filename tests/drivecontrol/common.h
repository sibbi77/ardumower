#ifndef COMMON_H
#define COMMON_H


#ifdef __AVR__
  // Arduino
  #include <Arduino.h>
  #include <EEPROM.h>  
  #include <Wire.h>  
  #define Console Serial
  #define Bluetooth Serial2  
  
// ---------- EEPROM helpers ----------------------------------

template <class T> int eewrite(int &ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int eeread(int &ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}

template <class T> int eereadwrite(boolean readflag, int &ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
    { 
       if (readflag) *p++ = EEPROM.read(ee++);
         else EEPROM.write(ee++, *p++);
    }
    return i;
}

  
#else  // __AVR__
  // simulator
  #include <opencv2/core/core.hpp>
  #include <opencv/cv.h>
  #include <opencv2/legacy/legacy.hpp>
  #include <opencv2/legacy/compat.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/imgproc/imgproc.hpp>
  #include <opencv2/features2d/features2d.hpp>

  #include "sim/simarduino.h"

#endif // __AVR__



struct point_t {
  float x;
  float y;
};

typedef struct point_t point_t;


float random1();
float gaussRandom();
float gauss(float mean, float std_dev);
float gaussian(float mu, float sigma, float x);
float distance(float x1, float y1, float x2, float y2);
float scalePI(float v);
float distancePI(float x, float w);



#endif
