/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Max31865.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  MAX31865 Temperature measurement API
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

#ifndef EMB_MAX31865_H_
#define EMB_MAX31865_H_

#include <stdint.h>

#include "config.h"

#define MAX31865_CHANNELS (3)

// Temperaure chip class
class Max31865
{
public:
  Max31865();
  virtual ~Max31865();

  void init(uint8_t chipNum, uint8_t wireNum);
  float readTemp(uint8_t ch);
  uint8_t getFault(uint8_t ch);
  float cyclic100ms(uint8_t ch);
  
private:
  uint8_t csPins[MAX31865_CHANNELS] = {SPI_PT_CS1, SPI_PT_CS2, SPI_PT_CS3};
  float pt100[MAX31865_CHANNELS] = {};
  uint8_t mode[MAX31865_CHANNELS] = {};
private:

};




#endif /* EMB_MAX31865_H_ */
