/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file motor_stepper.cpp
 ******************************************************************************
 * Copyright (C) NWI GmbH, 2023
 ******************************************************************************
 *
 * \brief  stepper motors implementation
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
#include "motor_stepper.h"

StaticJsonDocument<300> doc_recv_stepper;
StaticJsonDocument<300> doc_rcv_slave;

/* --- Local Variables ------------------------------------------------------ */

char *steperMotors[] = {"STM01", "STM02", "STM03", "STM04"};
int size_of_steppers = sizeof(steperMotors) / (sizeof(steperMotors[0]));
String str_snd_slave = "";
String str_rcv_slave = "";
String wire_rcv_slave; // received character from the Slave

static CFG_stepperMotorVal_T app_hsh_stepperMotorCfg[] =
    {
        {"STM01", 0, 0, 0, 0, 0, 0, 0},
        {"STM02", 0, 0, 0, 0, 0, 0, 0},
        {"STM03", 0, 0, 0, 0, 0, 0, 0},
        {"STM04", 0, 0, 0, 0, 0, 0, 0},
};
static CFG_stepperMotorVal_T app_hsh_old_stepperMotorCfg[] =
    {
        {"STM01", 0, 0, 0, 0, 0, 0, 0},
        {"STM02", 0, 0, 0, 0, 0, 0, 0},
        {"STM03", 0, 0, 0, 0, 0, 0, 0},
        {"STM04", 0, 0, 0, 0, 0, 0, 0},
};
static CFG_stepperMotorSensor_T app_hsh_stepperMotorSensorCfg[] =
    {
        {CONTROLLINO_AI0, CONTROLLINO_AI1, 1, 1, 1, 1},
        {CONTROLLINO_AI2, CONTROLLINO_AI3, 1, 1, 0, 0},
        {CONTROLLINO_AI4, CONTROLLINO_AI5, 1, 1, 0, 0},
        {CONTROLLINO_AI6, CONTROLLINO_AI7, 1, 1, 0, 0},
};

void stepper_init()
{
  stepper_initWire();
};

// init I2C connection;
void stepper_initWire()
{
  Wire.begin(); // STP_WIRE_ADDR       // Join the i2c bus (pin 4 , 5 connect together in master and slave )
  // Wire.onReceive(wireReceiveEvent); // register I2C event
}

void stepper_process(JsonDocument &doc)
{
  for (uint8_t n = 0; n < size_of_steppers; n++)
  {
    // get jsonString value from corresponding stepper in doc[]
    str_snd_slave = (const char *)doc[(app_hsh_stepperMotorCfg[n].devName)];
    // if (val.isNull() || val == nullptr)
    if (str_snd_slave == "" || str_snd_slave == nullptr)
      continue;

    DeserializationError error = deserializeJson(doc_recv_stepper, str_snd_slave);
    // set values
    if (error)
    {
      Serial.println("Error in doc_recv_stepper deserializeJson ....");
      continue;
    }
    if (doc_recv_stepper.isNull())
      continue;
    doc_recv_stepper[app_stepperMotorParamNameCfg.devName] = app_hsh_stepperMotorCfg[n].devName;
    app_hsh_stepperMotorCfg[n].valStep = doc_recv_stepper[app_stepperMotorParamNameCfg.Step];
    app_hsh_stepperMotorCfg[n].valPos = doc_recv_stepper[app_stepperMotorParamNameCfg.Pos];
    app_hsh_stepperMotorCfg[n].valDir = doc_recv_stepper[app_stepperMotorParamNameCfg.Dir];
    app_hsh_stepperMotorCfg[n].valMaxSpeed = doc_recv_stepper[app_stepperMotorParamNameCfg.Speed];
    app_hsh_stepperMotorCfg[n].valSpeed = doc_recv_stepper[app_stepperMotorParamNameCfg.Speed];
    app_hsh_stepperMotorCfg[n].valAccl = doc_recv_stepper[app_stepperMotorParamNameCfg.Accl];
    app_hsh_stepperMotorCfg[n].valAsync = doc_recv_stepper[app_stepperMotorParamNameCfg.Async];
    app_hsh_stepperMotorCfg[n].valSetpnt = doc_recv_stepper[app_stepperMotorParamNameCfg.Setpnt]; // set Home position for motors

    check_limit(n);
    sndDataToSlave(n);
  }
  // each 100ms slave device will send data
  rcvDataFromSlave();
};

#pragma region Local_Functions

void wireReceiveEvent(int NumberOfBytes)
{
  char c = 0;
  wire_rcv_slave = "";
  // loop through all received character from the Slave
  while (Wire.available())
  {
    c = Wire.read(); // receive byte as a character
    wire_rcv_slave += c;
  }
  Serial.println(wire_rcv_slave);
}

void wireSendData(String datatosend, int address)
{
  // Start I2C transmission
  Wire.beginTransmission(address); // STM01_ADDR
  // Send temperature measurement command, NO HOLD MASTER
  Wire.write(datatosend.c_str());
  // Stop I2C transmission
  Wire.endTransmission();
}

void rcvDataFromSlave()
{
  // Serial.println("reading value");
  if (Serial1.available())
  {
    // str_rcv_slave = "";
    String str_rcv_slave = Serial1.readStringUntil('\n');
    // String str_rcv_slave = "";
    //  char c = 0;
    //  do
    //  {
    //    c = Serial1.read();
    //    str_rcv_slave += c;
    //  } while ((c != '\n') && (c != -1));
    //  str_rcv_slave = str_rcv_slave.substring(0, str_rcv_slave.length() - 1);

    if (!str_rcv_slave.startsWith("{"))
      return false;

    // Serial.println("Received from slave : " + str_rcv_slave);
    DeserializationError error = deserializeJson(doc_rcv_slave, str_rcv_slave);

    if (error)
    {
      Serial.println("ERROR  in doc_rcv_slave deserializeJson ...." + str_rcv_slave);
      return;
    }
    // Serial.println(str_rcv_slave);

    // char inputString[] = "value1,value2,value3,value4";
    // const char delimiter[] = ",";

    // char *token = strtok(inputString, delimiter);
    // while (token != NULL)
    // {
    //   Serial.println(token);           // Print each value
    //   token = strtok(NULL, delimiter); // Find the next token
    // }

    // Serial.println(str_rcv_slave);
  }
}

void sndDataToSlave(int8_t n)
{
  // check whether there is a new command to send data to slave
  if (app_hsh_stepperMotorCfg[n].valDir == -1 || app_hsh_stepperMotorCfg[n].valStep == -1)
    return;
  if (
      app_hsh_stepperMotorCfg[n].valStep != 0 ||
      // app_hsh_stepperMotorCfg[n].valStep != app_hsh_old_stepperMotorCfg[n].valStep ||
      app_hsh_stepperMotorCfg[n].valSetpnt != app_hsh_old_stepperMotorCfg[n].valSetpnt ||
      app_hsh_stepperMotorCfg[n].valAccl != app_hsh_old_stepperMotorCfg[n].valAccl ||
      app_hsh_stepperMotorCfg[n].valSpeed != app_hsh_old_stepperMotorCfg[n].valSpeed ||
      app_hsh_stepperMotorCfg[n].valDir != app_hsh_old_stepperMotorCfg[n].valDir ||
      (abs(app_hsh_stepperMotorCfg[n].valPos) != abs(app_hsh_old_stepperMotorCfg[n].valPos)))
  {
    // replace the old values of motor params with already new one
    app_hsh_old_stepperMotorCfg[n].valDir = app_hsh_stepperMotorCfg[n].valDir;
    app_hsh_old_stepperMotorCfg[n].valPos = app_hsh_stepperMotorCfg[n].valPos;
    app_hsh_old_stepperMotorCfg[n].valStep = app_hsh_stepperMotorCfg[n].valStep;
    app_hsh_old_stepperMotorCfg[n].valSpeed = app_hsh_stepperMotorCfg[n].valSpeed;
    app_hsh_old_stepperMotorCfg[n].valAccl = app_hsh_stepperMotorCfg[n].valAccl;
    app_hsh_old_stepperMotorCfg[n].valAsync = app_hsh_stepperMotorCfg[n].valAsync;
    app_hsh_old_stepperMotorCfg[n].valSetpnt = app_hsh_stepperMotorCfg[n].valSetpnt; // if dir == 0 && setpnt == 1 this causes set home-position for the current motor

    str_snd_slave = "";
    serializeJson(doc_recv_stepper, str_snd_slave);
    // Serial.println(str_snd_slave);
    Serial1.println(str_snd_slave);
  }
}

// Since backend has some delay to get data from redis , we implemented a pre real time controll for sensors!
void check_limit(int8_t n)
{
  app_hsh_stepperMotorSensorCfg[n].backwSensorData = digitalRead(app_hsh_stepperMotorSensorCfg[n].backwPinMaped);
  app_hsh_stepperMotorSensorCfg[n].forwSensorData = digitalRead(app_hsh_stepperMotorSensorCfg[n].forwPinMaped);
  // default value if not enabled
  if (!app_hsh_stepperMotorSensorCfg[n].isenBackw)
    app_hsh_stepperMotorSensorCfg[n].backwSensorData = 1;

  // default value if not enabled
  if (!app_hsh_stepperMotorSensorCfg[n].isenForw)
    app_hsh_stepperMotorSensorCfg[n].forwSensorData = 1;

  if (
      (app_hsh_stepperMotorSensorCfg[n].backwSensorData == 0 && app_hsh_stepperMotorCfg[n].valDir == 2) || (app_hsh_stepperMotorSensorCfg[n].forwSensorData == 0 && app_hsh_stepperMotorCfg[n].valDir == 1) // ignor untill sensore device is prepared !!!
  )
  {
    app_hsh_stepperMotorCfg[n].valDir = 0;
    app_hsh_stepperMotorCfg[n].valSetpnt = 0;
    app_hsh_stepperMotorCfg[n].valStep = 0;
    doc_recv_stepper[app_stepperMotorParamNameCfg.Dir] = 0;
    doc_recv_stepper[app_stepperMotorParamNameCfg.Setpnt] = 0;
    doc_recv_stepper[app_stepperMotorParamNameCfg.Step] = 0;
  }
}
#pragma endregion Local_Functions
