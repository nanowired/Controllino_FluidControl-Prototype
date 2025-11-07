/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Timer.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Timer API
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

#ifndef EMB_TIMER_H_
#define EMB_TIMER_H_

// Timer class
class Timer
{
public:
   Timer();
   virtual ~Timer();

   void init(uint8_t Hz);

   void attachIsrFunc(void(*isr)());
   void detachIsrFunc();   
private:

};

#endif /* EMB_TIMER_H_ */
