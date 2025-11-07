/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Timer.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Timer implementation
 *
 * \par Purpose
 *      
 *
 * $LastChangedRevision: 1905 $  embeddeers revision number
 *
 ******************************************************************************
 *
 * \endcond
 */

/* --- Includes, Defines, Local Types  -------------------------------------- */
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "config.h"
#include "Timer.h"

/* --- Local Variables ------------------------------------------------------ */

/* --- Prototypes ----------------------------------------------------------- */

static void (*isrCallback)();

/* --- Global Functions ----------------------------------------------------- */

Timer::Timer()
{
}

Timer::~Timer()
{
}

// \par Description: Module initialization
// \param[in] uint8_t Hz: the ISR frequency, range 1Hz to 255 Hz, 0Hz is not advisable
void Timer::init(uint8_t Hz)
{
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 62500 / Hz;       // compare match register 16MHz/256/Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

// \par Description: Attaches a function to the timer ISR
// \param[in] void(*): the ISR function
void Timer::attachIsrFunc(void(*isr)())
{
  isrCallback = isr;
}

// \par Description: clears the attached ISR function
void Timer::detachIsrFunc()
{
  isrCallback = nullptr; 
}

/* --- Local Functions ----------------------------------------------------- */

// AVR timer 1 ISR
ISR(TIMER1_COMPA_vect)
{
  // run the attached ISR function
  if (nullptr != isrCallback)
  {
    isrCallback();
  }
}
