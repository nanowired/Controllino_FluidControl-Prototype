/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file app_sns.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Sensor Application API
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

#ifndef EMB_APP_SNS_H_
#define EMB_APP_SNS_H_

#include <ArduinoJson.h>

void app_sns_init();
void app_sns_task100ms(JsonDocument &doc);
void app_sns_task1s();
void app_sns_setValues(JsonDocument &doc);
void app_sns_enterSaveState();

#endif // EMB_APP_SNS_H_
