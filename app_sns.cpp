/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file app_sns.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Sensor Application  implementation
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

#include "app_sns.h"

// COC port configuration
static const CFG_ports_T app_sns_PortCfg[] = 
{ // port name          direction
  // terminal output ports
  {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_00, OUTPUT}
};

/* --- Global Functions ----------------------------------------------------- */

// \par Description: Module initialization
void app_sns_init()
{
  // port initialization
  port_init(app_sns_PortCfg, sizeof(app_sns_PortCfg) / sizeof(CFG_ports_T));
}

// 100ms task (COC -> PC)
void app_sns_task100ms(JsonDocument &doc)
{
  doc["AI00"] = digitalRead(CONTROLLINO_AI0);
  doc["AI01"] = digitalRead(CONTROLLINO_AI1);
  doc["AI02"] = digitalRead(CONTROLLINO_AI2);
  doc["AI03"] = digitalRead(CONTROLLINO_AI3);
  doc["AI04"] = digitalRead(CONTROLLINO_AI4);
  doc["AI05"] = digitalRead(CONTROLLINO_AI5);
  doc["AI06"] = digitalRead(CONTROLLINO_AI6);
  doc["AI07"] = digitalRead(CONTROLLINO_AI7);
  doc["AI08"] = digitalRead(CONTROLLINO_AI8);
  doc["AI09"] = digitalRead(CONTROLLINO_AI9);
  doc["AI10"] = digitalRead(CONTROLLINO_AI10);
  doc["AI11"] = digitalRead(CONTROLLINO_AI11);
  doc["AI12"] = analogRead(CONTROLLINO_AI12);
  doc["AI13"] = analogRead(CONTROLLINO_AI13);

  doc["DI00"] = digitalRead(CONTROLLINO_SCREW_TERMINAL_DIGITAL_IN_00);
  doc["DI01"] = digitalRead(CONTROLLINO_SCREW_TERMINAL_DIGITAL_IN_01);
  doc["DI02"] = digitalRead(CONTROLLINO_SCREW_TERMINAL_DIGITAL_IN_02);
  doc["DI03"] = digitalRead(CONTROLLINO_SCREW_TERMINAL_DIGITAL_IN_03);

  doc["IN00"] = digitalRead(CONTROLLINO_IN0);
  doc["IN01"] = digitalRead(CONTROLLINO_IN1);

  doc["vers"] = SW_VERSION;   // firmware version info
}

// 1s task
void app_sns_task1s()
{
 
}

// sets the values (PC -> COC)
void app_sns_setValues(JsonDocument &doc)
{
  digitalWrite(CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_00, doc["DO0"]);
}

// Enter save state, reset outputs
void app_sns_enterSaveState()
{
  digitalWrite(CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_00, LOW);
}
