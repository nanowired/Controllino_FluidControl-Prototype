/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file app_gal.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Galvanik Contoller Application  implementation
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

#include "port.h"
#include "config.h"

#include "app_gal.h"

/* --- Local Variables ------------------------------------------------------ */

// GAL digital output port configuration
static const CFG_DoutPorts_T app_gal_DoutPortCfg[] = 
{
    // Relays
  {CONTROLLINO_R0, "R0"},
  {CONTROLLINO_R1, "R1"},
  {CONTROLLINO_R2, "R2"},
  {CONTROLLINO_R3, "R3"},
  {CONTROLLINO_R4, "R4"},
  {CONTROLLINO_R5, "R5"},
  {CONTROLLINO_R6, "R6"},
  {CONTROLLINO_R7, "R7"},
  {CONTROLLINO_R8, "R8"},
  {CONTROLLINO_R9, "R9"},

  // D0 .. D7
  {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_00, "DO0"},
  {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_01, "DO1"},
  {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_02, "DO2"},
  {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_03, "DO3"},
  {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_04, "DO4"},
  {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_05, "DO5"},
  {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_06, "DO6"},
  {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_07, "DO7"},
};

// GAL digital and analog input port configuration
static const CFG_inputPorts_T app_gal_inputPortCfg[] = 
{
  {CONTROLLINO_AI0,  CFG_DIGITAL, "AI00"},
  {CONTROLLINO_AI1,  CFG_DIGITAL, "AI01"},
  {CONTROLLINO_AI2,  CFG_DIGITAL, "AI02"},
  {CONTROLLINO_AI3,  CFG_DIGITAL, "AI03"},
  {CONTROLLINO_AI4,  CFG_DIGITAL, "AI04"},
  {CONTROLLINO_AI5,  CFG_DIGITAL, "AI05"},
  {CONTROLLINO_AI6,  CFG_DIGITAL, "AI06"},
  {CONTROLLINO_AI7,  CFG_DIGITAL, "AI07"},
  {CONTROLLINO_AI8,  CFG_DIGITAL, "AI08"},
  {CONTROLLINO_AI9,  CFG_DIGITAL, "AI09"},
  {CONTROLLINO_AI10, CFG_DIGITAL, "AI10"},
  {CONTROLLINO_AI11, CFG_DIGITAL, "AI11"},
  {CONTROLLINO_AI12, CFG_ANALOG,  "AI12"},
  {CONTROLLINO_AI13, CFG_ANALOG,  "AI13"},
  {CONTROLLINO_DI0,  CFG_DIGITAL, "DI0"},
  {CONTROLLINO_DI1,  CFG_DIGITAL, "DI1"},
  {CONTROLLINO_DI2,  CFG_DIGITAL, "DI2"},
  {CONTROLLINO_DI3,  CFG_DIGITAL, "DI3"}  
};

static uint8_t cnt100ms = 0;
static bool ledFlag = false;
void app_gal_loop_tests();

/* --- Global Functions ----------------------------------------------------- */

// \par Description: Module initialization
void app_gal_init()
{
  // digital output port initialization
  port_initDoutPorts(app_gal_DoutPortCfg, sizeof(app_gal_DoutPortCfg) / sizeof(CFG_DoutPorts_T));
}

// 100ms task (GAL -> PC)
void app_gal_task100ms(JsonDocument &doc)
{
  // Get temperatures all 1 seconds, time-shifted
  switch (cnt100ms)
  {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
  }

  cnt100ms = (cnt100ms < 2) ? cnt100ms + 1 : 0;
  
  // Read analog and digital input ports
  for (uint8_t n = 0; n < sizeof(app_gal_inputPortCfg) / sizeof(CFG_inputPorts_T); n++)
  {
    char *name = app_gal_inputPortCfg[n].name;
    uint8_t mode = app_gal_inputPortCfg[n].mode;
    uint8_t portNum = app_gal_inputPortCfg[n].portNum;
    
    doc[name] = (CFG_ANALOG == mode) ? analogRead(portNum) : digitalRead(portNum);
  }
  
  doc["vers"] = SW_VERSION;   // firmware version info
}

// 1s task
void app_gal_task1s()
{
  // flash Test LED 1 of MCP23017
  port_writeMcp(TEST_LED1, ledFlag);
  ledFlag = !ledFlag;
#if 0
  app_gal_loop_tests();
#endif
}

// sets the JSON values (PC -> GAL)
void app_gal_setValues(JsonDocument &doc)
{
  // Write digital output ports
  for (uint8_t n = 0; n < sizeof(app_gal_DoutPortCfg) / sizeof(CFG_DoutPorts_T); n++)
  {
    char *name = app_gal_DoutPortCfg[n].name;
    uint8_t portNum = app_gal_DoutPortCfg[n].portNum;
    
    digitalWrite(portNum, doc[name]);
  }

  port_writeMcp(TEST_LED4, LOW);
}

// Enter save state, reset outputs
void app_gal_enterSaveState()
{
  // Reset digital output ports
  for (uint8_t n = 0; n < sizeof(app_gal_DoutPortCfg) / sizeof(CFG_DoutPorts_T); n++)
  {
    uint8_t portNum = app_gal_DoutPortCfg[n].portNum;
    
    digitalWrite(portNum, LOW);
  }
}

// Tests in the main loop
void app_gal_loop_tests()
{

}
