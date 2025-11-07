/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file app_hsh.cpp
 ******************************************************************************
 * Copyright (C) NWI GmbH, 2023
 ******************************************************************************
 *
 * \brief  Shunt Contoller Application  implementation
 *
 * \par Purpose
 *
 *
 *
 *
 ******************************************************************************
 *
 * \endcond
 */

/* --- Includes, Defines, Local Types  -------------------------------------- */

#include <Arduino.h>

// #include "Emb_Max31865.h"
#//include "Max31865.h"
#include "sensor.h"
#include "port.h"
#include "app_hsh.h"

/* --- Local Variables ------------------------------------------------------ */
// HSH digital output port configuration
static const CFG_DoutPorts_T app_hsh_DoutPortCfg[] =
    {
        // Relays
        {CONTROLLINO_R0, "R0"}, // HE01
        {CONTROLLINO_R1, "R1"}, // PV01
        {CONTROLLINO_R2, "R2"}, // LM01
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

        // {CONTROLLINO_AI0, "AI00"}, // BUZZER
};

// HSH port configuration
static const CFG_ports_T app_hsh_PortCfg[] =
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
        {CONTROLLINO_AI0, INPUT_PULLUP}, // IS01
        {CONTROLLINO_AI1, INPUT_PULLUP}, // IS02
        {CONTROLLINO_AI2, INPUT_PULLUP}, // IS03
        {CONTROLLINO_AI3, INPUT_PULLUP}, // IS04
        {CONTROLLINO_AI4, INPUT_PULLUP}, // IS05
        {CONTROLLINO_AI5, INPUT_PULLUP}, // IS06
        {CONTROLLINO_AI6, INPUT_PULLUP}, // IS07
        {CONTROLLINO_AI7, INPUT_PULLUP}, // IS08
};

// HSH digital and analog input port configuration
static const CFG_inputPorts_T app_hsh_inputPortCfg[] =
    {
        {CONTROLLINO_AI0, CFG_DIGITAL, "AI00"},
        {CONTROLLINO_AI1, CFG_DIGITAL, "AI01"},
        {CONTROLLINO_AI2, CFG_DIGITAL, "AI02"},
        {CONTROLLINO_AI3, CFG_DIGITAL, "AI03"},
        {CONTROLLINO_AI4, CFG_DIGITAL, "AI04"},
        {CONTROLLINO_AI5, CFG_DIGITAL, "AI05"},
        {CONTROLLINO_AI6, CFG_DIGITAL, "AI06"},
        {CONTROLLINO_AI7, CFG_DIGITAL, "AI07"},
        {CONTROLLINO_AI8, CFG_ANALOG, "AI08"},
        {CONTROLLINO_AI9, CFG_ANALOG, "AI09"},
        {CONTROLLINO_AI10, CFG_ANALOG, "AI10"},
        {CONTROLLINO_AI11, CFG_ANALOG, "AI11"},

        {CONTROLLINO_AI12, CFG_ANALOG, "AI12"}, // HS01
        {CONTROLLINO_AI13, CFG_ANALOG, "AI13"},

        {CONTROLLINO_DI0, CFG_DIGITAL, "DI0"},
        {CONTROLLINO_DI1, CFG_DIGITAL, "DI1"},
        {CONTROLLINO_DI2, CFG_DIGITAL, "DI2"},
        {CONTROLLINO_DI3, CFG_DIGITAL, "DI3"}};

char *paramArr[] = {"STM01_cpos", "STM01_dis2go", "STM02_cpos", "STM02_dis2go", "STM03_cpos", "STM03_dis2go", "STM04_cpos", "STM04_dis2go"};
// Max31865 thermo_;
float temp_[3] = {};
static uint8_t cnt100ms = 0;
static bool ledFlag = false;
volatile uint32_t app_hsh_int0_cnt = 0;
volatile uint32_t app_hsh_int1_cnt = 0;

// IN0 counter
// void app_hsh_loop_tests();

// IN0 interrrupt handler
void app_hsh_int0_isr()
{
  app_hsh_int0_cnt++;
}
// IN1 interrrupt handler
void app_hsh_int1_isr()
{
  app_hsh_int1_cnt++;
}

/* --- Global Functions ----------------------------------------------------- */

// \par Description: Module initialization
void app_hsh_init()
{
  // digital output port initialization
  port_initDoutPorts(app_hsh_DoutPortCfg, sizeof(app_hsh_DoutPortCfg) / sizeof(CFG_DoutPorts_T));
  // port initialization
  port_init(app_hsh_PortCfg, sizeof(app_hsh_PortCfg) / sizeof(CFG_ports_T));

  // install IN0 interrupt
  attachInterrupt(digitalPinToInterrupt(CONTROLLINO_IN0), app_hsh_int0_isr, FALLING);
  // install IN1 interrupt
  attachInterrupt(digitalPinToInterrupt(CONTROLLINO_IN1), app_hsh_int1_isr, FALLING);

  // PT100 measurement initialization
  // thermo_.init(0, MAX31865_3WIRE);
  // thermo_.init(1, MAX31865_2WIRE);
  // thermo_.init(2, MAX31865_3WIRE);
}

// 100ms task (HSH -> PC)
void app_hsh_task100ms(JsonDocument &doc)
{
  // Get temperatures all 1 seconds, time-shifted
  // temp_[0] = thermo_.cyclic100ms(0);
  // switch (cnt100ms)
  // {
  // case 0:
  //   temp_[0] = thermo_.cyclic100ms(0);
  //   break;
  // case 1:
  //   temp_[1] = thermo_.cyclic100ms(1);
  //   break;
  // case 2:
  //   temp_[2] = thermo_.cyclic100ms(2);
  //   break;
  // }

  // cnt100ms = (cnt100ms < 2) ? cnt100ms + 1 : 0;

  // doc["ET1"] = temp_[0];
  //  doc["ET2"] = temp_[1];
  //  doc["ET3"] = temp_[2];

  // if (sensor_chipOK())
  // {
  //   doc["E21"] = sensor_getRawDiffA();
  //   doc["E20"] = sensor_getRawCurA(); // durchflusssensor
  //   doc["E19"] = sensor_getRawCurB(); // drucksensor
  // }

  // concatinate keys
  // for (const auto &element : doc_rcv_slave.as<JsonObject>())
  // {
  //   const char *key = element.key().c_str();
  //   // const char *value = element.value().as<const char *>();
  //   const long value = element.value();
  //   doc[key] = value;
  // }
  // to send dis2go & cpos to server
  for (const auto &element : doc_rcv_slave.as<JsonObject>())
  {
    const char *key = element.key().c_str();
    const long value = element.value();
    int ikey = atoi(key);
    doc[paramArr[ikey]] = value;
  }
  //  String str_snd = "";
  //  serializeJson(doc, str_snd);
  //  Serial.println(str_snd);

  // Read analog and digital input ports
  for (uint8_t n = 0; n < sizeof(app_hsh_inputPortCfg) / sizeof(CFG_inputPorts_T); n++)
  {
    char *name = app_hsh_inputPortCfg[n].name;
    uint8_t mode = app_hsh_inputPortCfg[n].mode;
    uint8_t portNum = app_hsh_inputPortCfg[n].portNum;

    doc[name] = (CFG_ANALOG == mode) ? analogRead(portNum) : digitalRead(portNum);
  }
  // doc["AI12"] = map(doc["AI12"], 0, 1023, 0, 300);
  doc["IN0"] = app_hsh_int0_cnt;
  doc["vers"] = SW_VERSION; // firmware version info
}

// 1s task
void app_hsh_task1s()
{
  // flash Test LED 1 of MCP23017
  //  port_writeMcp(TEST_LED1, ledFlag);
  //  ledFlag = !ledFlag;
  //  app_hsh_loop_tests();
}

// sets the values (PC -> HSH)
void app_hsh_setValues(JsonDocument &doc)
{
  // Write digital output ports
  for (uint8_t n = 0; n < sizeof(app_hsh_DoutPortCfg) / sizeof(CFG_DoutPorts_T); n++)
  {
    uint8_t portNum = app_hsh_DoutPortCfg[n].portNum;
    char *name = app_hsh_DoutPortCfg[n].name;

    digitalWrite(portNum, doc[name]);
  }

  // run stepper motors
  stepper_process(doc);
  // run servo motors
  // servo_process(doc);

  // port_writeMcp(TEST_LED4, LOW);
}

// Enter save state, reset outputs
void app_hsh_enterSaveState()
{
  // Set digital output ports
  for (uint8_t n = 0; n < sizeof(app_hsh_DoutPortCfg) / sizeof(CFG_DoutPorts_T); n++)
  {
    uint8_t portNum = app_hsh_DoutPortCfg[n].portNum;

    digitalWrite(portNum, LOW);
  }
}

// Tests in the main loop
void app_hsh_loop_tests()
{
  // float temp_[3] = {thermo_.readtemp_(0), thermo_.readtemp_(1), thermo_.readtemp_(2)};

  // String str = "PT100: " + String(temp_[0]) + " " + String(temp_[1]) + " " + String(temp_[2]);
  // str += " UFP: " + String(sensor_getUFP()) + "mm ";
  // str += " AI0 " + String((uint16_t)analogRead(CONTROLLINO_AI0));

  // Serial.println(str);
}

int app_hsh_loop_ReadPort(char *portName)
{
  for (uint8_t n = 0; n < sizeof(app_hsh_inputPortCfg) / sizeof(CFG_inputPorts_T); n++)
  {
    char *name = app_hsh_inputPortCfg[n].name;
    if (name != portName)
      continue;

    uint8_t mode = app_hsh_inputPortCfg[n].mode;
    uint8_t portNum = app_hsh_inputPortCfg[n].portNum;

    return (CFG_ANALOG == mode) ? analogRead(portNum) : digitalRead(portNum);
  }
  return 0;
}