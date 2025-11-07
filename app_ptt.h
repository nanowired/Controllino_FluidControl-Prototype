/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file app_ptt.h
 ******************************************************************************
 * Copyright (C) NWI GmbH, 2025
 ******************************************************************************
 *
 * \brief  Shunt Contoller Application API
 *
 * \par Purpose
 *
 *
 ******************************************************************************
 *
 * \endcond
 */

#ifndef NWI_APP_ptt_H_
#define NWI_APP_ptt_H_

#include <ArduinoJson.h>

void app_ptt_init();
void app_ptt_task100ms(JsonDocument &doc);
void app_ptt_task1s();
void app_ptt_setValues(JsonDocument &doc);
void app_ptt_enterSaveState();
void app_ptt_loop_tests();

#endif // NWI_APP_ptt_H_
