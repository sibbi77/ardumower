/*
 * boards.h
 *
 *  Created on: 12.08.2015
 *      Author: sebas_000
 */

#ifndef BOARDS_H_
#define BOARDS_H_

#ifdef __AVR__
  // Arduino Mega
  #include <EEPROM.h>
#else
  // Arduino Due
  #include "due.h"
#endif




#endif /* BOARDS_H_ */
