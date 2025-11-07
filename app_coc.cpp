/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file app_coc.cpp
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Containment Contoller Application  implementation
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

#include "Emb_Max31865.h"
#include "Max31865.h"
#include "sensor.h"
#include "port.h"
#include "config.h"

#include "app_coc.h"

/* --- Local Variables ------------------------------------------------------ */

// COC digital output port configuration
static const CFG_DoutPorts_T app_coc_DoutPortCfg[] =
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

// COC digital and analog input port configuration
static const CFG_inputPorts_T app_coc_inputPortCfg[] =
    {
        {CONTROLLINO_AI0, CFG_ANALOG, "AI00"},
        {CONTROLLINO_AI1, CFG_ANALOG, "AI01"},
        {CONTROLLINO_AI2, CFG_ANALOG, "AI02"},
        {CONTROLLINO_AI3, CFG_DIGITAL, "AI03"},
        {CONTROLLINO_AI4, CFG_DIGITAL, "AI04"},
        {CONTROLLINO_AI5, CFG_DIGITAL, "AI05"},
        {CONTROLLINO_AI6, CFG_DIGITAL, "AI06"},
        {CONTROLLINO_AI7, CFG_DIGITAL, "AI07"},
        {CONTROLLINO_AI8, CFG_DIGITAL, "AI08"},
        {CONTROLLINO_AI9, CFG_ANALOG, "AI09"},
        {CONTROLLINO_AI10, CFG_ANALOG, "AI10"},
        {CONTROLLINO_AI11, CFG_ANALOG, "AI11"},
        {CONTROLLINO_AI12, CFG_ANALOG, "AI12"},
        {CONTROLLINO_AI13, CFG_ANALOG, "AI13"},
        {CONTROLLINO_DI0, CFG_DIGITAL, "DI0"},
        {CONTROLLINO_DI1, CFG_DIGITAL, "DI1"},
        {CONTROLLINO_DI2, CFG_DIGITAL, "DI2"},
        {CONTROLLINO_DI3, CFG_DIGITAL, "DI3"}};

Max31865 thermo;
float temp[3] = {};
static uint8_t cnt100ms = 0;
static bool ledFlag = false;
volatile uint32_t app_coc_int0_cnt = 0;

// IN0 counter
void app_coc_loop_tests();
// IN0 interrrupt handler
void app_coc_int0_isr()
{
  app_coc_int0_cnt++;
}

/* --- Global Functions ----------------------------------------------------- */

// \par Description: Module initialization
void app_coc_init()
{
  // digital output port initialization
  port_initDoutPorts(app_coc_DoutPortCfg, sizeof(app_coc_DoutPortCfg) / sizeof(CFG_DoutPorts_T));

  // terminal input ports with pull up resistor
  pinMode(CONTROLLINO_DI0, INPUT_PULLUP); // Leckagesensorband - leakage
  pinMode(CONTROLLINO_DI1, INPUT_PULLUP); // Leckagesensorband - Disconnection
  // install IN0 interrupt
  pinMode(CONTROLLINO_IN0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CONTROLLINO_IN0), app_coc_int0_isr, FALLING);

  // PT100 measurement initialization
  thermo.init(0, MAX31865_2WIRE);
  thermo.init(1, MAX31865_2WIRE);
  thermo.init(2, MAX31865_3WIRE);
}

// 100ms task (COC -> PC)
void app_coc_task100ms(JsonDocument &doc)
{
  // Get temperatures all 1 seconds, time-shifted
  switch (cnt100ms)
  {
  case 0:
    temp[0] = thermo.cyclic100ms(0);
    break;
  case 1:
    temp[1] = thermo.cyclic100ms(1);
    break;
  case 2:
    temp[2] = thermo.cyclic100ms(2);
    break;
  }

  cnt100ms = (cnt100ms < 2) ? cnt100ms + 1 : 0;

  doc["ET1"] = temp[0];
  doc["ET2"] = temp[1];
  doc["ET3"] = temp[2];

  if (sensor_chipOK())
  {
    doc["E21"] = sensor_getRawDiffA();
    doc["E20"] = sensor_getRawCurA(); // durchflusssensor
    doc["E19"] = sensor_getRawCurB(); // drucksensor
  }

  doc["IN0"] = app_coc_int0_cnt;

  // Read analog and digital input ports
  for (uint8_t n = 0; n < sizeof(app_coc_inputPortCfg) / sizeof(CFG_inputPorts_T); n++)
  {
    char *name = app_coc_inputPortCfg[n].name;
    uint8_t mode = app_coc_inputPortCfg[n].mode;
    uint8_t portNum = app_coc_inputPortCfg[n].portNum;

    doc[name] = (CFG_ANALOG == mode) ? analogRead(portNum) : digitalRead(portNum);
  }

  doc["vers"] = SW_VERSION; // firmware version info
}

// 1s task
void app_coc_task1s()
{
  // flash Test LED 1 of MCP23017
  port_writeMcp(TEST_LED1, ledFlag);
  ledFlag = !ledFlag;
#if 0
  app_coc_loop_tests();
#endif
}

// sets the values (PC -> COC)
void app_coc_setValues(JsonDocument &doc)
{
  // Write digital output ports
  for (uint8_t n = 0; n < sizeof(app_coc_DoutPortCfg) / sizeof(CFG_DoutPorts_T); n++)
  {
    char *name = app_coc_DoutPortCfg[n].name;
    uint8_t portNum = app_coc_DoutPortCfg[n].portNum;

    digitalWrite(portNum, doc[name]);
  }

  port_writeMcp(TEST_LED4, LOW);
}

// Enter save state, reset outputs
void app_coc_enterSaveState()
{
  // Set digital output ports
  for (uint8_t n = 0; n < sizeof(app_coc_DoutPortCfg) / sizeof(CFG_DoutPorts_T); n++)
  {
    uint8_t portNum = app_coc_DoutPortCfg[n].portNum;

    digitalWrite(portNum, LOW);
  }
}

// Tests in the main loop
void app_coc_loop_tests()
{
  // float temp[3] = {thermo.readTemp(0), thermo.readTemp(1), thermo.readTemp(2)};

  String str = "PT100: " + String(temp[0]) + " " + String(temp[1]) + " " + String(temp[2]);
  str += " UFP: " + String(sensor_getUFP()) + "mm ";
  str += " AI0 " + String((uint16_t)analogRead(CONTROLLINO_AI0));

  Serial.println(str);
}
