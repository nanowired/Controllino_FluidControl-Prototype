/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file sensor.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Sensor measurement API
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

#ifndef EMB_SENSOR_H_
#define EMB_SENSOR_H_

void sensor_init();
bool sensor_chipOK();

int16_t sensor_getRawCurA();
int16_t sensor_getRawCurB();
int16_t sensor_getRawDiffA();
int16_t sensor_getRawU(uint8_t channel);
int16_t sensor_getRawUFP();

float sensor_getCurA();
float sensor_getCurB();
float sensor_getDiffA();
float sensor_getU(uint8_t channel);
float sensor_getUFP();

#endif /* EMB_SENSOR_H_ */
