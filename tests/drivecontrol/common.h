#ifndef COMMON_H
#define COMMON_H


#ifdef __AVR__
  // Arduino
  #include <Arduino.h>
#else
  // simulator
  #include "sim/simarduino.h"

  #include <opencv2/core/core.hpp>
  #include <opencv/cv.h>
  #include <opencv2/legacy/legacy.hpp>
  #include <opencv2/legacy/compat.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/imgproc/imgproc.hpp>
  #include <opencv2/features2d/features2d.hpp>


#endif // __AVR__



struct point_t {
  float x;
  float y;
};

typedef struct point_t point_t;


float random();
float gaussRandom();
float gauss(float mean, float std_dev);
float gaussian(float mu, float sigma, float x);
float distance(float x1, float y1, float x2, float y2);
float scalePI(float v);
float distancePI(float x, float w);



#endif
