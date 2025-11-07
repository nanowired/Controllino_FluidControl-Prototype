/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Max31865.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  MAX31865 Temperature measurement implementation
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

#include "Emb_Max31865.h"

#include "port.h"
#include "config.h"

#include "Max31865.h"

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define MAX31865_RREF      (430.0)
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define MAX31865_RNOMINAL  (100.0)

/* --- Local Variables ------------------------------------------------------ */

Emb_Max31865 max31865 = Emb_Max31865(CONTROLLINO_PIN_HEADER_DIGITAL_OUT_12, CONTROLLINO_PIN_HEADER_DIGITAL_OUT_13, CONTROLLINO_PIN_HEADER_DIGITAL_IN_03, CONTROLLINO_PIN_HEADER_DIGITAL_OUT_14);

/* --- Global Functions ----------------------------------------------------- */

Max31865::Max31865()
{

}

Max31865::~Max31865()
{

}

// \par Description: Module initialization
void Max31865::init(uint8_t chipNum, uint8_t wireNum)
{
  max31865.setCsPin(csPins[chipNum]);
  max31865.begin(wireNum);
  max31865.autoConvert(true);  
  max31865.enableBias(true);
}

float Max31865::readTemp(uint8_t ch)
{
  float pt100 = 0.0f;
  ch = (ch < MAX31865_CHANNELS) ? ch : 0;
  max31865.setCsPin(csPins[ch]);
  pt100 = max31865.temperature(MAX31865_RNOMINAL, MAX31865_RREF);
  return pt100;
}

uint8_t Max31865::getFault(uint8_t ch)
{
  ch = (ch < MAX31865_CHANNELS) ? ch : 0;
  max31865.setCsPin(csPins[ch]);
  uint8_t fault = max31865.readFault();
  max31865.clearFault();
  return fault;
}

// State machine to read the temperatures
float Max31865::cyclic100ms(uint8_t ch)
{
  ch = (ch < MAX31865_CHANNELS) ? ch : 0;
  max31865.setCsPin(csPins[ch]);
  
  switch (mode[ch])
  {
    case 0:
      max31865.clearFault();
      max31865.enableBias(true);
      break;
    case 1:
      max31865.startReadRTD();
      break;
    case 2:
      pt100[ch] = max31865.getTemperature(MAX31865_RNOMINAL, MAX31865_RREF);
  }

  mode[ch] = (mode[ch] <= 2) ? mode[ch] + 1 : 0;

  return pt100[ch];
}
