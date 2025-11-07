/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file motor_stepper.h
 ******************************************************************************
 * Copyright (C) NWI GmbH, 2023
 ******************************************************************************
 *
 * \brief  stepper motors API
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

#ifndef NWI_MOTOR_STEPPER_H_
#define NWI_MOTOR_STEPPER_H_

#include "config.h"
#include "AccelStepper.h"
#include <ArduinoJson.h>
#include <MultiStepper.h>
#include <Wire.h>


extern StaticJsonDocument<300> doc_recv_stepper;
extern StaticJsonDocument<300> doc_rcv_slave;

extern int size_of_steppers;
extern CFG_stepperMotorVal_T app_hsh_stepperMotorCfg[];
extern CFG_stepperMotorVal_T app_hsh_old_stepperMotorCfg[];
extern CFG_stepperMotorSensor_T app_hsh_stepperMotorSensorCfg[];
extern String str_rcv_slave;

void stepper_process(JsonDocument &doc);
void stepper_init();
void stepper_initWire();
void rcvDataFromSlave();
void sndDataToSlave(int8_t n);
void wireReceiveEvent(int NumberOfBytes);
void wireSendData(String datatosend, int address);
void check_limit(int8_t n);

#endif // NWI_MOTOR_STEPPER_H_
