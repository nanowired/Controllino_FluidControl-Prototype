/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file motor_servo.h
 ******************************************************************************
 * Copyright (C) NWI GmbH, 2023
 ******************************************************************************
 *
 * \brief  servo motors API
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

#ifndef NWI_MOTOR_SERVO_H_
#define NWI_MOTOR_SERVO_H_

#include <ArduinoJson.h>
#include <Controllino.h>
#include "config.h"

void servo_init();
void servo_process(JsonDocument &doc);

#endif // NWI_MOTOR_SERVO_H_
