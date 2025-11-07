/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Ds18b20.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Temperature measurement implementation
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

#include "Ds18b20.h"

#define DS18B20_TEMPERATURE_PRECISION (12)

/* --- Local Variables ------------------------------------------------------ */

/* --- Global Functions ----------------------------------------------------- */

Ds18b20::Ds18b20()
{

}

Ds18b20::~Ds18b20()
{

}

// \par Description: Module initialization
void Ds18b20::init(uint8_t pinNum)
{
  oneWire.begin(pinNum);
  sensors.setOneWire(&oneWire);
  sensors.begin();
  sensorNum = sensors.getDeviceCount();

  for (uint8_t n = 0; n < sensorNum; n++)
  {
    sensors.getAddress(sensorAddr[n], n);
    sensors.setResolution(sensorAddr[n], DS18B20_TEMPERATURE_PRECISION);
  }

  sensors.setWaitForConversion(false);
}

// \par Description: Start temperatur measurement
void Ds18b20::startConversion()
{
  sensors.requestTemperatures();
}

// \par Description: Get Temperature of a Sensor
// \param[in] uint8_t num: sensor number
// \return float: the measured frequency
float Ds18b20::getTemp(uint8_t num)
{
  num = (num < sensorNum) ? num : 0;
  //sensors.getAddress(sensorAddr, num);
  return sensors.getTempC(sensorAddr[num]);
}

// \par Description: Returns the number of rekognized sensors
// \return uint8_t: number of rekognized sensors
uint8_t Ds18b20::getSensornum()
{
  return sensorNum;
}

/* --- Local Functions ----------------------------------------------------- */
