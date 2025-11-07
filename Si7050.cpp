/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Si7050.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Si7050 Temperature measurement implementation
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
#include <Wire.h>

#include "Si7050.h"

// SI7050 I2C address is 0x40(64)
#define Addr 0x40

/* --- Local Variables ------------------------------------------------------ */

/* --- Global Functions ----------------------------------------------------- */

Si7050::Si7050()
{

}

Si7050::~Si7050()
{

}

// \par Description: Module initialization
void Si7050::init()
{
    // Initialise I2C communication as MASTER
  Wire.begin();

  // Start I2C transmission
  Wire.beginTransmission(Addr);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);
  startConversion();
}

// \par Description: Start temperatur measurement
void Si7050::startConversion()
{
    // Start I2C transmission
    Wire.beginTransmission(Addr);
    // Send temperature measurement command, NO HOLD MASTER
    Wire.write(0xF3);
    // Stop I2C transmission
    Wire.endTransmission();
}

// \par Description: Get Temperature of a Sensor
// \return float: the measured frequency
float Si7050::getTemp()
{
      // Request 2 bytes of data
    Wire.requestFrom(Addr, 2);

    // Read 2 bytes of data
    // temp msb, temp lsb
    if(Wire.available() == 2)
    {
      *ptrL = Wire.read();
      *ptrH = Wire.read();
    }

    startConversion();

    return ((175.72f * (float)buffer) / 65536.0f) - 46.85f;
}


/* --- Local Functions ----------------------------------------------------- */
