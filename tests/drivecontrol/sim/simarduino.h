// Arduino classes for simulator

#ifndef SIMARDUINO_H
#define SIMARDUINO_H



#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// some libraries and sketches depend on this
// AVR stuff, assuming Arduino.h or WProgram.h
// automatically includes it...
//#include "avr/pgmspace.h"
//#include "avr/interrupt.h"

#include "binary.h"
#include "itoa.h"


#ifdef __cplusplus
extern "C"{
#endif


#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2


#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif // abs


#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif // min

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif // max


#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

// flash string macro
#define F(x) x


typedef unsigned int word;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
//typedef bool boolean;
typedef uint8_t byte;



unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int us);
void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
void tone(uint8_t _pin, unsigned int frequency, unsigned long duration);
void noTone(uint8_t _pin);


#ifdef __cplusplus
} // extern "C"


#include "WString.h"
#include "WMath.h"
//#include <iostream>
#include "Stream.h"
#include <opencv2/highgui/highgui.hpp>

class AConsole : public Stream
{
    int key;
  public:
    virtual int available(){ key = cvWaitKey( 1 ); return (key != 0); }
    virtual int read(){ return key; }
    virtual int peek(){ return key; }
    virtual void flush(){}
    virtual size_t write(const uint8_t c){
      printf("%c", ((char)c));
      return 0;
    }
    using Print::write; // pull in write(str) and write(buf, size) from Print
};

extern AConsole Console;

#endif // __cplusplus



#endif
