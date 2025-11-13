/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file port.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2025
 ******************************************************************************
 *
 * \brief  Port access implementation 
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

#include <Adafruit_MCP23X17.h>

#include "config.h"
#include "port.h"

#define MCP23017_ADPT_ADDR (0x27)
#define PORT_DHCP_EN (0)

enum
{
  PORT_CFG_JMP_SERVICE,
  PORT_CFG_JMP_ETH_DHCP_EN,
  PORT_CFG_JMP_CTRL_MODE,  // device 0: COC 1: PWS

  PORT_CFG_JMP_NUM
};

// LED names
enum
{
  PORT_ADPT_LED1_G = 8,
  PORT_ADPT_LED2_B,
  PORT_ADPT_LED3_G,
  PORT_ADPT_LED4_R,
  PORT_ADPT_LED5_B,
  PORT_ADPT_LED6_R,
  PORT_ADPT_LED7_R
};
enum
{
  PORT_PANEL_LED_G = CONTROLLINO_AI6,
  PORT_PANEL_LED_Y = CONTROLLINO_AI7,
  PORT_PANEL_LED_B = CONTROLLINO_AI8,
  PORT_PANEL_LED_R = CONTROLLINO_AI9,
};

Adafruit_MCP23X17 mcp;
bool port_MCPexists = false; // true: MCP exists, false: MCP not found
uint8_t configJumperState = 0;

// the ADPT MCP 
Adafruit_MCP23X17 mcpAdpt;
bool port_MCPAdpd_exists = false; // true: MCP exists, false: MCP not found

void port_init(const CFG_ports_T *ports, uint8_t portNum)
{
  for (uint8_t p = 0; p < portNum; p++)
  {
    pinMode(ports[p].portNum, ports[p].direction);
  }
}

// initialization of the digital output ports
void port_initDoutPorts(const CFG_DoutPorts_T *ports, uint8_t portNum)
{
  for (uint8_t p = 0; p < portNum; p++)
  {
    pinMode(ports[p].portNum, OUTPUT);
  }
}
// reset of the digital output ports
void port_resetDoutPorts(const CFG_DoutPorts_T *ports, uint8_t portNum)
{
  for (uint8_t p = 0; p < portNum; p++)
  {
    digitalWrite(ports[p].portNum, ports[p].defaultState);
  }
}

// Sets a Port to output with state (HIGH or LOW)
void port_setOutput(uint8_t portNum, uint8_t state)
{
  pinMode(portNum, OUTPUT);
  digitalWrite(portNum, state);
}

void port_initMcp()
{
    // test if MCP exists
  Wire.begin();
  Wire.beginTransmission(MCP23XXX_ADDR);
  byte error = Wire.endTransmission();
  // no error -> MCP exists
  if (0 == error)
  {
    port_MCPexists = true;
  
    mcp.begin_I2C();
  
    mcp.pinMode(CFG_SERVICE, INPUT_PULLUP);
    mcp.pinMode(CFG_ETH_DHCP_EN, INPUT_PULLUP);
    mcp.pinMode(CFG_CTRL_MODE, INPUT_PULLUP);
    mcp.pinMode(ADC_ALERT, INPUT_PULLUP);
        
    mcp.pinMode(TEST_LED1, OUTPUT);
    mcp.digitalWrite(TEST_LED1, LOW);
    mcp.pinMode(TEST_LED2, OUTPUT);
    mcp.digitalWrite(TEST_LED2, LOW);
    mcp.pinMode(TEST_LED3, OUTPUT);
    mcp.digitalWrite(TEST_LED3, LOW);
    mcp.pinMode(TEST_LED4, OUTPUT);
    mcp.digitalWrite(TEST_LED4, LOW);
  
    mcp.pinMode(SPI_PT_CS1, OUTPUT);
    mcp.digitalWrite(SPI_PT_CS1, HIGH);
    mcp.pinMode(SPI_PT_CS2, OUTPUT);
    mcp.digitalWrite(SPI_PT_CS2, HIGH);
    mcp.pinMode(SPI_PT_CS3, OUTPUT);
    mcp.digitalWrite(SPI_PT_CS3, HIGH);

    // get service jumper state
    configJumperState = !mcp.digitalRead(CFG_CTRL_MODE)   << PORT_CFG_JMP_CTRL_MODE |
                        !mcp.digitalRead(CFG_ETH_DHCP_EN) << PORT_CFG_JMP_ETH_DHCP_EN |
                        !mcp.digitalRead(CFG_SERVICE)     << PORT_CFG_JMP_SERVICE;
#if PORT_DHCP_EN                      
    configJumperState |= 1 << PORT_CFG_JMP_ETH_DHCP_EN;
#endif
  }
}

void port_writeMcp(uint8_t pinNum, uint8_t value)
{
  if (port_MCPexists)
  {
    mcp.digitalWrite(pinNum, value);
  }
}

uint8_t port_readMcp(uint8_t pinNum)
{
  uint8_t value = 0;
  if (port_MCPexists)
  {
    value = mcp.digitalRead(pinNum);
  }

  return value;
}

// returns the module type COC, GAL or SNS
uint8_t port_getModuleType()
{
  return 3;//0x3
  return port_readMcpAdpdCfg();;
}

// returns the jumper state: service 
uint8_t port_getCfgJmpService()
{
  return (configJumperState & (1 << CFG_SERVICE)) ? 1 : 0;
}

// returns the jumper state: ETH DHCP enable 
uint8_t port_getCfgJmpEthDhcpEn()
{
   return (configJumperState & (1 << PORT_CFG_JMP_ETH_DHCP_EN)) ? 1 : 0;
}

// returns the jumper state: device 
uint8_t port_getCfgJmpDevice()
{
  return (configJumperState & (1 << PORT_CFG_JMP_CTRL_MODE)) ? 1 : 0;
}

// returns the jumper states
uint8_t port_getCfgJmpStates()
{
  return configJumperState;
}

//---------------------------------------------------------------------------------------
// ADPT MCP

void port_initMcpAdpt()
{
  // test if MCP exists
  Wire.begin();
  Wire.beginTransmission(MCP23017_ADPT_ADDR);
  byte error = Wire.endTransmission();
  // no error -> MCP exists
  if (0 == error)
  {
    port_MCPAdpd_exists = true;
    mcpAdpt.begin_I2C(MCP23017_ADPT_ADDR);

    // configure LED pins, all LEDs on
    for (uint8_t n = 8; n < 15; n++)
    {
      mcpAdpt.pinMode(n, OUTPUT);
      mcpAdpt.digitalWrite(n, 0);
    }

    // configure CFG pins
    for (uint8_t n = 0; n < 4; n++)
    {
      mcpAdpt.pinMode(n, INPUT_PULLUP);
    }

    delay(1000);
    // show configuration pins by LEDs
    uint8_t cfg = port_readMcpAdpdCfg() | 16;
    port_writeAdpd_LEDs(cfg);
  }
}

void port_writeMcpAdpd(uint8_t pinNum, uint8_t value)
{
  if (port_MCPAdpd_exists)
  {
    mcpAdpt.digitalWrite(pinNum, value ? 1 : 0);
  }
  else{
    //Serial.println(F("nono"));
  }
}

uint8_t port_readMcpAdpd(uint8_t pinNum)
{
  uint8_t value = 0;
  if (port_MCPAdpd_exists)
  {
    value = mcpAdpt.digitalRead(pinNum);
  }

  return value;
}

uint8_t port_readMcpAdpdCfg()
{
  uint8_t cfg = 0xFF;
  
  if (port_MCPAdpd_exists)
  {
    cfg = ~(uint8_t)mcpAdpt.readGPIOAB();
    cfg &= 0x0F;
  }
  
  return cfg;
}

void port_writeAdpd_LED_green(uint8_t value)
{
  digitalWrite(PORT_PANEL_LED_G, value ? 1 : 0);
  port_writeMcpAdpd(PORT_ADPT_LED1_G, value ? 0 : 1);
}

void port_toggleAdpd_LED_green()
{
  auto led  = digitalRead(PORT_PANEL_LED_G);
  digitalWrite(PORT_PANEL_LED_G, led ? 1 : 0);
  port_writeMcpAdpd(PORT_ADPT_LED1_G, !port_readMcpAdpd(PORT_ADPT_LED1_G));
}

void port_writeAdpd_LED_blue(uint8_t value)
{
  digitalWrite(PORT_PANEL_LED_B, value ? 1 : 0);
  port_writeMcpAdpd(PORT_ADPT_LED2_B, value ? 0 : 1);
}

void port_writeAdpd_LED_yellow(uint8_t value)
{
  digitalWrite(PORT_PANEL_LED_Y, value ? 1 : 0);
  port_writeMcpAdpd(PORT_ADPT_LED3_G, value ? 0 : 1);
  port_writeMcpAdpd(PORT_ADPT_LED4_R, value ? 0 : 1);
}

void port_writeAdpd_LED_magenta(uint8_t value)
{
  digitalWrite(PORT_PANEL_LED_Y, value ? 1 : 0);
  port_writeMcpAdpd(PORT_ADPT_LED5_B, value ? 0 : 1);
  port_writeMcpAdpd(PORT_ADPT_LED6_R, value ? 0 : 1);
}

void port_writeAdpd_LED_red(uint8_t value)
{
  digitalWrite(PORT_PANEL_LED_R, value ? 1 : 0);
  port_writeMcpAdpd(PORT_ADPT_LED7_R, value ? 0 : 1);
}
void port_toggleAdpd_LED_red()
{
  auto led  = digitalRead(PORT_PANEL_LED_R);
  digitalWrite(PORT_PANEL_LED_R, led ? 0 : 1);
}

void port_writeAdpd_LEDs(uint8_t value)
{
    digitalWrite(PORT_PANEL_LED_B, value ? 1 : 0);
    digitalWrite(PORT_PANEL_LED_G, value ? 1 : 0);
    digitalWrite(PORT_PANEL_LED_Y, value ? 1 : 0);
    digitalWrite(PORT_PANEL_LED_R, value ? 1 : 0);
    
    port_writeAdpd_LED_green(value & 16);
    port_writeAdpd_LED_blue(value & 8);
    port_writeAdpd_LED_yellow(value & 4);
    port_writeAdpd_LED_magenta(value & 2);
    port_writeAdpd_LED_red(value & 1);
}
