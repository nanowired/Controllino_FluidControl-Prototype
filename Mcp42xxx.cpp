/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Mcp42xxx.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Poti implementation
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

#include "Mcp42xxx.h"

#define MCP42XXX_SELECT_POTI_0        (1)
#define MCP42XXX_SELECT_POTI_1        (2)
#define MCP42XXX_CMD_WRITE_POTI_0     (0x10 | MCP42XXX_SELECT_POTI_0)
#define MCP42XXX_CMD_WRITE_POTI_1     (0x10 | MCP42XXX_SELECT_POTI_1)

/* --- Global Functions ----------------------------------------------------- */

Mcp42xxx::Mcp42xxx()
{
  if (!spi2.begin()) 
  {
    Serial.println("Could not initialize SPI device");
  }
}

Mcp42xxx::~Mcp42xxx()
{
  
}

// \par Description: Sets a poti
// \param[in] uint8_t num: poti number
// \param[in] uint8_t val: poti value to be set
void Mcp42xxx::set(uint8_t num, uint8_t val)
{
  uint8_t buffer[2];

  buffer[0] = (0 ==num) ? MCP42XXX_CMD_WRITE_POTI_0 : MCP42XXX_CMD_WRITE_POTI_1;
  buffer[1] = val;
  
  spi2.write(buffer, 2);
}
