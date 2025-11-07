/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file sensor.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Sensor measurement implementation
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

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include "sensor.h"

Adafruit_ADS1115 ads;

#define SENSOR_CUR_DIVIDER_GAIN_TWO         (1599.0f)
#define SENSOR_VOL_DIVIDER_GAIN_TWOTHIRDS   (5330.0f)
#define SENSOR_VOL_DIVIDER_GAIN_ONE         (8000.0f)
#define SENSOR_UTF_OFFSET                   (25.0f)
#define SENSOR_UTF_MULTIPLIER               (76.351f)

static bool chipOK = false;

// \par Description: Module initialization
void sensor_init()
{  
  if (sensor_chipOK())
  {
    ads.begin(); 
  }
}

bool sensor_chipOK()
{
  // Test it chip exists
  Wire.beginTransmission(ADS1X15_ADDRESS );
  chipOK = Wire.endTransmission() ? false : true;
  return chipOK ; 
}

// \par Description: returns the current of 4-20mA Sensor A raw value
int16_t sensor_getRawCurA()
{
  int16_t val = 0;
  if (chipOK)
  {
    ads.setGain(GAIN_TWO);
    val = ads.readADC_SingleEnded(3);
  }

  return val;
}

// \par Description: returns the current of 4-20mA Sensor A
float sensor_getCurA()
{
  return (float)sensor_getRawCurA() / SENSOR_CUR_DIVIDER_GAIN_TWO;
}

// \par Description: returns the current channel B raw value
int16_t sensor_getRawCurB()
{
  int16_t val = 0;
  if (chipOK)
  {
    ads.setGain(GAIN_TWO);
    val = ads.readADC_SingleEnded(2);
  }

  return val;
}

// \par Description: returns the current channel B
float sensor_getCurB()
{
  return (float)sensor_getRawCurA() / SENSOR_CUR_DIVIDER_GAIN_TWO;
}

// \par Description: returns the Electroplating Reference Voltage raw value
int16_t sensor_getRawDiffA()
{
  int16_t val = 0;
  if (chipOK)
  {
    ads.setGain(GAIN_TWOTHIRDS);
    val = ads.readADC_Differential_0_1();
  }

  return val;
}

// \par Description: returns the Electroplating Reference Voltage
float sensor_getDiffA()
{
  return (float)(sensor_getRawDiffA()) / SENSOR_VOL_DIVIDER_GAIN_TWOTHIRDS;;
}

// \par Description: returns the voltage of channel 0 to 3 raw voltage
int16_t sensor_getRawU(uint8_t channel)
{
  int16_t val = 0;
  if (chipOK)
  {
    ads.setGain(GAIN_ONE);
    val = ads.readADC_SingleEnded(channel);
  }

  return val;
}
  
// \par Description: returns the voltage of channel 0 to 3
float sensor_getU(uint8_t channel)
{
  return (float)sensor_getRawU(channel) / SENSOR_VOL_DIVIDER_GAIN_ONE;
}

// \par Description: returns the ultrasonic sensor in mm raw value
int16_t sensor_getRawUFP()
{
  int16_t val = 0;
  if (chipOK)
  {
    ads.setGain(GAIN_ONE);  
    val = ads.readADC_SingleEnded(3);
  }

  return val;
}

// \par Description: returns the ultrasonic sensor in mm
float sensor_getUFP()
{
  return SENSOR_UTF_OFFSET + ((float)sensor_getRawUFP() / SENSOR_UTF_MULTIPLIER);
}

/* --- Local Functions ----------------------------------------------------- */
