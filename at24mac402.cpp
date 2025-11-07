/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file at24mac402.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  AT24MAC402 read serial number 
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
#include <Arduino.h>

#define AT24MAC402_I2C_ADDR     (0x5F)
#define AT24MAC402_MAC_POS      (0x9A)
#define AT24MAC402_UID_POS      (0x80)

#define AT24MAC402_MAC_SIZE     (6)
#define AT24MAC402_UID_SIZE     (8)
#define AT24MAC402_UID_STR_SIZE (AT24MAC402_UID_SIZE * 2 + 1)
#define AT24MAC402_BIN2HEX(x) ((x > 9) ? (x + 'A' - 10) : (x + '0'))

/* --- Local Variables ------------------------------------------------------ */

static uint8_t mac[AT24MAC402_MAC_SIZE] = {0xFC, 0xC2, 0x3D, 0 , 8, 0x15};
static uint8_t uid[AT24MAC402_UID_SIZE] = {1,2,3,4,5,6,7,8};
static char uidStr[AT24MAC402_UID_STR_SIZE] = {};

/* --- Prototypes ----------------------------------------------------------- */

static void at24mac402_read(uint8_t *buf, uint8_t memPos, uint8_t len);

/* --- Global Functions ----------------------------------------------------- */

char* at24mac402_readMac()
{  
  at24mac402_read(mac, AT24MAC402_MAC_POS, AT24MAC402_MAC_SIZE);

  return mac;
}

char* at24mac402_readUid()
{
  // read 8 byte MAC instead of UID
  at24mac402_read(uid, AT24MAC402_MAC_POS, AT24MAC402_UID_SIZE);

  // write UID string
  for (uint8_t n = 0; n < AT24MAC402_UID_SIZE; n++)
  {
    uint8_t h = uid[AT24MAC402_UID_SIZE - n - 1] >> 4;
    uint8_t l = uid[AT24MAC402_UID_SIZE - n - 1] & 0x0F;

    uidStr[n * 2]     = AT24MAC402_BIN2HEX(l);
    uidStr[n * 2 + 1] = AT24MAC402_BIN2HEX(h);   
  }

  uidStr[AT24MAC402_UID_STR_SIZE - 1] = 0;
  
  return uid;
}

char* at24mac402_getUidStr()
{
  return uidStr;
}

/* --- Local Functions ----------------------------------------------------- */

static void at24mac402_read(uint8_t *buf, uint8_t memPos, uint8_t len)
{
  bool success;
  Wire.begin();
  Wire.beginTransmission(AT24MAC402_I2C_ADDR);
  Wire.write(memPos);
  success = (0 == Wire.endTransmission());

  if (success)
  {
    Wire.requestFrom((int)AT24MAC402_I2C_ADDR, (int)len); 
  
    for (uint8_t n = 0; n < len; n++) 
    {
      buf[n] = Wire.read();
    }
    Wire.end();
  }
}
