/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Adc.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  ADC API
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

#ifndef EMB_ADC_H_
#define EMB_ADC_H_

#include <Wire.h>
#include <Adafruit_ADS1015.h>

// ADC class
class Adc
{
public:
  Adc();
  virtual ~Adc();

  void init();
   
private:
  Adafruit_ADS1115 ads; 
};

#endif /* EMB_ADC_H_ */
