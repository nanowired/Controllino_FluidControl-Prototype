/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file app_ptt.cpp
 ******************************************************************************
 * Copyright (C) NWI GmbH, 2025
 ******************************************************************************
 *
 * \brief  PTT Contoller Application  implementation
 *
 * \par Purpose
 *
 *
 ******************************************************************************
 *
 * \endcond
 */

/* --- Includes, Defines, Local Types  -------------------------------------- */

#include <Arduino.h>
#include "sensor.h"
#include "port.h"
#include "app_ptt.h"

/* --- Local Variables ------------------------------------------------------ */
// PTT digital output port configuration
static const CFG_DoutPorts_T app_ptt_DoutPortCfg[] =
    {
        // Relays
        {CONTROLLINO_R0, "R0", LOW}, // HE01
        {CONTROLLINO_R1, "R1", LOW}, // PV01
        {CONTROLLINO_R2, "R2", LOW}, // LM01
        {CONTROLLINO_R3, "R3", LOW},
        {CONTROLLINO_R4, "R4", LOW},
        {CONTROLLINO_R5, "R5", LOW},
        {CONTROLLINO_R6, "R6", LOW},
        {CONTROLLINO_R7, "R7", LOW},
        {CONTROLLINO_R8, "R8", LOW},
        {CONTROLLINO_R9, "R9", LOW},

        // D0 .. D7
        {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_00, "DO0", LOW},
        {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_01, "DO1", LOW},
        {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_02, "DO2", LOW},
        {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_03, "DO3", LOW},
        {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_04, "DO4", LOW},
        {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_05, "DO5", LOW},
        {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_06, "DO6", LOW},
        {CONTROLLINO_SCREW_TERMINAL_DIGITAL_OUT_07, "DO7", LOW},
        {CONTROLLINO_AO0, "AO0", LOW},
        {CONTROLLINO_AO1, "AO1", LOW},

        // {CONTROLLINO_AI0, "AI00"}, // BUZZER
};

// PTT port configuration
static const CFG_ports_T app_ptt_PortCfg[] =
    {
        // port name          direction
        // terminal output ports
        {CONTROLLINO_IN0, INPUT_PULLUP},
        {CONTROLLINO_IN1, INPUT_PULLUP},

        // terminal input ports with pull up resistor
        {CONTROLLINO_DI0, INPUT_PULLUP},
        {CONTROLLINO_DI1, INPUT_PULLUP},
        {CONTROLLINO_DI2, INPUT_PULLUP},
        {CONTROLLINO_DI3, INPUT_PULLUP},
 };

// PTT digital and analog input port configuration
static const CFG_inputPorts_T app_ptt_inputPortCfg[] =
    {
        {CONTROLLINO_AI0, CFG_ANALOG, "AI00"},
        {CONTROLLINO_AI1, CFG_ANALOG, "AI01"},
        {CONTROLLINO_AI2, CFG_ANALOG, "AI02"},
        {CONTROLLINO_AI3, CFG_ANALOG, "AI03"},
        {CONTROLLINO_AI4, CFG_ANALOG, "AI04"},
        {CONTROLLINO_AI5, CFG_ANALOG, "AI05"},
        {CONTROLLINO_AI6, CFG_ANALOG, "AI06"},
        {CONTROLLINO_AI7, CFG_ANALOG, "AI07"},
        {CONTROLLINO_AI8, CFG_ANALOG, "AI08"},
        {CONTROLLINO_AI9, CFG_ANALOG, "AI09"},
        {CONTROLLINO_AI10, CFG_ANALOG, "AI10"},
        {CONTROLLINO_AI11, CFG_ANALOG, "AI11"},

        {CONTROLLINO_AI12, CFG_ANALOG, "AI12"},
        {CONTROLLINO_AI13, CFG_ANALOG, "AI13"},

        {CONTROLLINO_DI0, CFG_DIGITAL, "DI0"},
        {CONTROLLINO_DI1, CFG_DIGITAL, "DI1"},
        {CONTROLLINO_DI2, CFG_DIGITAL, "DI2"},

        {CONTROLLINO_IN0, CFG_DIGITAL, "IN0"},
        {CONTROLLINO_IN1, CFG_DIGITAL, "IN1"},
};

// char *paramArr[] = {"STM01_cpos", "STM01_dis2go", "STM02_cpos", "STM02_dis2go", "STM03_cpos", "STM03_dis2go", "STM04_cpos", "STM04_dis2go"};
// Max31865 thermo_;
// float temp_[3] = {};
// static uint8_t cnt100ms = 0;
// static bool ledFlag = false;
volatile uint32_t app_ptt_int0_cnt = 0;
volatile uint32_t app_ptt_int1_cnt = 0;

// IN0 counter
// void app_ptt_loop_tests();

// IN0 interrrupt handler
void app_ptt_int0_isr()
{
  app_ptt_int0_cnt++;

  port_resetDoutPorts(app_ptt_DoutPortCfg, sizeof(app_ptt_DoutPortCfg) / sizeof(CFG_DoutPorts_T));
}
// IN1 interrrupt handler
void app_ptt_int1_isr()
{
  app_ptt_int1_cnt++;
}

/* --- Global Functions ----------------------------------------------------- */

// \par Description: Module initialization
void app_ptt_init()
{
  // digital output port initialization
  port_initDoutPorts(app_ptt_DoutPortCfg, sizeof(app_ptt_DoutPortCfg) / sizeof(CFG_DoutPorts_T));
  // reset digital output ports
  port_resetDoutPorts(app_ptt_DoutPortCfg, sizeof(app_ptt_DoutPortCfg) / sizeof(CFG_DoutPorts_T));
  // port initialization
  port_init(app_ptt_PortCfg, sizeof(app_ptt_PortCfg) / sizeof(CFG_ports_T));

  // install IN0 interrupt
  attachInterrupt(digitalPinToInterrupt(CONTROLLINO_IN0), app_ptt_int0_isr, FALLING);
  // install IN1 interrupt
  attachInterrupt(digitalPinToInterrupt(CONTROLLINO_IN1), app_ptt_int1_isr, FALLING);

  // PT100 measurement initialization
  // thermo_.init(0, MAX31865_3WIRE);
  // thermo_.init(1, MAX31865_2WIRE);
  // thermo_.init(2, MAX31865_3WIRE);
}

// 100ms task (HSH -> PC)
void app_ptt_task100ms(JsonDocument &doc)
{
  // to send dis2go & cpos to server
  // for (const auto &element : doc_rcv_slave.as<JsonObject>())
  // {
  //   const char *key = element.key().c_str();
  //   const long value = element.value();
  //   int ikey = atoi(key);
  //   doc[paramArr[ikey]] = value;
  // }
  //  String str_snd = "";
  //  serializeJson(doc, str_snd);
  //  Serial.println(str_snd);

  // Read analog and digital input ports
  for (uint8_t n = 0; n < sizeof(app_ptt_inputPortCfg) / sizeof(CFG_inputPorts_T); n++)
  {
    char *name = app_ptt_inputPortCfg[n].name;
    uint8_t mode = app_ptt_inputPortCfg[n].mode;
    uint8_t portNum = app_ptt_inputPortCfg[n].portNum;

    doc[name] = (CFG_ANALOG == mode) ? analogRead(portNum) : digitalRead(portNum);
  }
  // doc["AI12"] = map(doc["AI12"], 0, 1023, 0, 300);
  doc["heartbeat"] =  millis() / 1000;
  doc["swver"] = SW_VERSION; // firmware version info
}

// 1s task
void app_ptt_task1s()
{
  // flash Test LED 1 of MCP23017
  //  port_writeMcp(TEST_LED1, ledFlag);
  //  ledFlag = !ledFlag;
  //  app_ptt_loop_tests();
}

// sets the values (PC -> HSH)
void app_ptt_setValues(JsonDocument &doc)
{
  // Write digital output ports
  for (uint8_t n = 0; n < sizeof(app_ptt_DoutPortCfg) / sizeof(CFG_DoutPorts_T); n++)
  {
    uint8_t portNum = app_ptt_DoutPortCfg[n].portNum;
    char *name = app_ptt_DoutPortCfg[n].name;

    digitalWrite(portNum, doc[name]);
  }

  // port_writeMcp(TEST_LED4, LOW);
}

// Enter save state, reset outputs
void app_ptt_enterSaveState()
{
  // Set digital output ports
  for (uint8_t n = 0; n < sizeof(app_ptt_DoutPortCfg) / sizeof(CFG_DoutPorts_T); n++)
  {
    uint8_t portNum = app_ptt_DoutPortCfg[n].portNum;

    digitalWrite(portNum, LOW);
  }
}

// Tests in the main loop
void app_ptt_loop_tests()
{
  // float temp_[3] = {thermo_.readtemp_(0), thermo_.readtemp_(1), thermo_.readtemp_(2)};

  // String str = "PT100: " + String(temp_[0]) + " " + String(temp_[1]) + " " + String(temp_[2]);
  // str += " UFP: " + String(sensor_getUFP()) + "mm ";
  // str += " AI0 " + String((uint16_t)analogRead(CONTROLLINO_AI0));

  // Serial.println(str);
}

int app_ptt_loop_ReadPort(char *portName)
{
  for (uint8_t n = 0; n < sizeof(app_ptt_inputPortCfg) / sizeof(CFG_inputPorts_T); n++)
  {
    char *name = app_ptt_inputPortCfg[n].name;
    if (name != portName)
      continue;

    uint8_t mode = app_ptt_inputPortCfg[n].mode;
    uint8_t portNum = app_ptt_inputPortCfg[n].portNum;

    return (CFG_ANALOG == mode) ? analogRead(portNum) : digitalRead(portNum);
  }
  return 0;
}