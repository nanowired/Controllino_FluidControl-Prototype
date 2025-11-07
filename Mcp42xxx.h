/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Mcp42xxx.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Poti API
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

#ifndef EMB_MCP42XXX_H_
#define EMB_MCP42XXX_H_


#include <Adafruit_SPIDevice.h>
#include "config.h"

class Mcp42xxx
{
public:
  Mcp42xxx();
  virtual ~Mcp42xxx();

  void set(uint8_t num, uint8_t val);

private:
  Adafruit_SPIDevice spi2 = Adafruit_SPIDevice(SPI2_SS, SPI2_SCK, SPI2_MISO, SPI2_MOSI, SPI2_SPEED, SPI_BITORDER_MSBFIRST, SPI_MODE0);
};

#endif /* EMB_MCP42XXX_H_ */
