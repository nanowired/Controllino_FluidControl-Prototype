/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Si7050.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Si7050 Temperature measurement API
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

#ifndef EMB_SI7050_H_
#define EMB_SI7050_H_

// Temperaure sensor class
class Si7050
{
public:
  Si7050();
  virtual ~Si7050();

  void init();
  void startConversion();
  float getTemp();
  
private:
  uint16_t buffer = 0;
  uint8_t *ptrH = (uint8_t*)&buffer;
  uint8_t *ptrL = ptrH + 1;
};

#endif /* EMB_SI7050_H_ */
