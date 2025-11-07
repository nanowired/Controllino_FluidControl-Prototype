/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file app_gal.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Galvanik Contoller Application API
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

#ifndef EMB_APP_GAL_H_
#define EMB_APP_GAL_H_

#include <ArduinoJson.h>

void app_gal_init();
void app_gal_task100ms(JsonDocument &doc);
void app_gal_task1s();
void app_gal_setValues(JsonDocument &doc);
void app_gal_enterSaveState();


#endif // EMB_APP_GAL_H_
