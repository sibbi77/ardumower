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

// Ardumower RTC/timer
// *RTC time/date
// *time schedule table for timer mowing 

#ifndef TIMER_H
#define TIMER_H


#include "common.h"

#define MAX_TIMERS 5

// ---------- date time --------------------------------------

struct timehm_t {
  byte hour;
  byte minute;
};

typedef struct timehm_t timehm_t;

struct date_t {
  byte dayOfWeek;
  byte day;
  byte month;
  short year;
};

typedef struct date_t date_t;

struct datetime_t {
  timehm_t time;
  date_t date;
};

typedef struct datetime_t datetime_t;

// ---------- timers --------------------------------------
struct ttimer_t {
  boolean active;
  timehm_t startTime;
  timehm_t stopTime;
  byte daysOfWeek;
};

typedef struct ttimer_t ttimer_t;



class TimerControl
{
  public:
    bool enable;
    unsigned long powerTimeMinutes;  // power-ON time (minutes)
    ttimer_t timer[MAX_TIMERS];  // timers
    datetime_t datetime;         // current date & time
    bool stopTimerTriggered;     // has stop timer triggered?
    bool startTimerTriggered;    // has start timer triggered?
    TimerControl();
    void setup();
    bool rtcError;              // RTC communication error?
    void run();
    void print();
    void writeRTCDateTime();  // write datetime to RTC
    int getDayOfWeek(int month, int day, int year, int CalendarSystem);
    String date2str(date_t date);
    int time2minutes(timehm_t time);
    String time2str(timehm_t time);
    void minutes2time(int minutes, timehm_t &time);
  private:
    void checkTimer();
    void setDefaultTime();
    unsigned long nextTimerTime;
    // ------ driver ----------------
    virtual bool driverReadRTC(datetime_t &dt);
    virtual bool driverSetRTC(datetime_t &dt);
};




#endif

