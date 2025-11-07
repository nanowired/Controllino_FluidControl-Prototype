/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file app_coc.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Containment Contoller Application API
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

#ifndef EMB_APP_COC_H_
#define EMB_APP_COC_H_

#include <ArduinoJson.h>

void app_coc_init();
void app_coc_task100ms(JsonDocument &doc);
void app_coc_task1s();
void app_coc_setValues(JsonDocument &doc);
void app_coc_enterSaveState();


#endif // EMB_APP_COC_H_
