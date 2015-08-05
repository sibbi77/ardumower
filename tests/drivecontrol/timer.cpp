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

#include "common.h"
#include "timer.h"



char *dayOfWeek[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};



TimerControl::TimerControl(){
  enable = false;
  powerTimeMinutes = 0;
  nextTimerTime = 60000;
  stopTimerTriggered = true;
  startTimerTriggered = false;
}

void TimerControl::setup(){
  Console.println(F("TimerControl::setup"));
  setDefaultTime();
  rtcError = false;
}

// call this in main loop
void TimerControl::run(){
  if (!enable) return;
  if (millis() < nextTimerTime) return;
  nextTimerTime = millis() + 60000;
  powerTimeMinutes++;
  driverReadRTC(datetime);
  print();
  checkTimer();
}

// Returns the day of week (0=Sunday, 6=Saturday) for a given date
int TimerControl::getDayOfWeek(int month, int day, int year, int CalendarSystem)
{
     // CalendarSystem = 1 for Gregorian Calendar, 0 for Julian Calendar
     if (month < 3)
     {
           month = month + 12;
           year = year - 1;
     }
     return (
             day
             + (2 * month)
             + int(6 * (month + 1) / 10)
             + year
             + int(year / 4)
             - int(year / 100)
             + int(year / 400)
             + CalendarSystem
            ) % 7;
}


int TimerControl::time2minutes(timehm_t time){
  return (time.hour * 60 + time.minute);
}

void TimerControl::minutes2time(int minutes, timehm_t &time){
  time.hour   = minutes / 60;
  time.minute = minutes % 60;
}

String TimerControl::time2str(timehm_t time){
  String s = String(time.hour/10);
  s += (time.hour%10);
  s += ":";
  s += (time.minute/10);
  s += (time.minute%10);
  return s;
}

String TimerControl::date2str(date_t date){
  String s = dayOfWeek[date.dayOfWeek];
  s += " ";
  s += date.day / 10;
  s += date.day % 10;
  s += ".";
  s += date.month / 10;
  s += date.month % 10;
  s += ".";
  s += date.year;
  return s;
}


void TimerControl::setDefaultTime(){
  datetime.time.hour = 12;
  datetime.time.minute = 0;
  datetime.date.dayOfWeek = 0;
  datetime.date.day = 1;
  datetime.date.month = 1;
  datetime.date.year = 2013;
  timer[0].active = false;
  timer[0].daysOfWeek= B01111110;
  timer[0].startTime.hour = 9;
  timer[0].stopTime.hour = 11;
}

void TimerControl::print(){
  Console.print(F("powerTimeMinutes: "));
  Console.print(powerTimeMinutes);
  Console.print(F("  RTC date: "));
  Console.println(date2str(datetime.date));
}

void TimerControl::writeRTCDateTime(){
  Console.print(F("RTC date set: "));
  Console.println(date2str(datetime.date));
  driverSetRTC(datetime);
}


// check timer
void TimerControl::checkTimer(){
  stopTimerTriggered = true;
  startTimerTriggered = false;

  for (int i=0; i < MAX_TIMERS; i++){
      if (timer[i].active){
        if  ( (timer[i].daysOfWeek & (1 << datetime.date.dayOfWeek)) != 0) {
          int startmin = time2minutes(timer[i].startTime);
          int stopmin =  time2minutes(timer[i].stopTime);
          int currmin =  time2minutes(datetime.time);
          if ((currmin >= startmin) && (currmin < stopmin)){
            // start timer triggered
            stopTimerTriggered = false;
            Console.println(F("timer start triggered"));
            startTimerTriggered = true;
          }
        }
      }
  }
}

bool TimerControl::driverReadRTC(datetime_t &dt){
  return true;
}

bool TimerControl::driverSetRTC(datetime_t &dt){
 return true;
}


