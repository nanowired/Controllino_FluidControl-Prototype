/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Ds18b20.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Temperature measurement API
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

#ifndef EMB_DS18B20_H_
#define EMB_DS18B20_H_

#include <OneWire.h>
#include <DallasTemperature.h>

enum
{
  DS18B20_SENSOR_1,
  DS18B20_SENSOR_2,
  DS18B20_SENSOR_3,

  DS18B20_SENSOR_NUM
};

// Temperaure sensor class
class Ds18b20
{
public:
  Ds18b20();
  virtual ~Ds18b20();

  void init(uint8_t pinNum);
  void startConversion();
  float getTemp(uint8_t num);
  uint8_t getSensornum();
   
private:
  uint8_t sensorNum = 0;

  OneWire oneWire;
  DallasTemperature sensors;
  DeviceAddress sensorAddr[3] = {};

};

#endif /* EMB_DS18B20_H_ */
