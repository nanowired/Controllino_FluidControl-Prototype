/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file app_hsh.h
 ******************************************************************************
 * Copyright (C) NWI GmbH, 2023
 ******************************************************************************
 *
 * \brief  Shunt Contoller Application API
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

#ifndef NWI_APP_HSH_H_
#define NWI_APP_HSH_H_

#include <ArduinoJson.h>
#include "motor_stepper.h"
#include "motor_servo.h"

void app_hsh_init();
void app_hsh_task100ms(JsonDocument &doc);
void app_hsh_task1s();
void app_hsh_setValues(JsonDocument &doc);
void app_hsh_enterSaveState();
void app_hsh_loop_tests();

#endif // NWI_APP_HSH_H_
