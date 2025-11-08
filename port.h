/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file port.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  Port API
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

#ifndef EMB_PORT_H_
#define EMB_PORT_H_

#include <Controllino.h>

#include "config.h"

void port_init(const CFG_ports_T *ports, uint8_t portNum);
void port_initDoutPorts(const CFG_DoutPorts_T *ports, uint8_t portNum);
void port_resetDoutPorts(const CFG_DoutPorts_T *ports, uint8_t portNum);

void port_initMcp();
void port_setOutput(uint8_t portNum, uint8_t state);
void port_writeMcp(uint8_t pinNum, uint8_t value);
uint8_t port_readMcp(uint8_t pinNum);
uint8_t port_getCfgJmpService();
uint8_t port_getModuleType();
uint8_t port_getCfgJmpEthDhcpEn();
uint8_t port_getCfgJmpDevice();
uint8_t port_getCfgJmpStates();
void port_initMcpAdpt();
void port_writeMcpAdpd(uint8_t pinNum, uint8_t value);
uint8_t port_readMcpAdpd(uint8_t pinNum);
uint8_t port_readMcpAdpdCfg();
void port_writeAdpd_LED_green(uint8_t value);
void port_toggleAdpd_LED_green();
void port_writeAdpd_LED_blue(uint8_t value);
void port_writeAdpd_LED_yellow(uint8_t value);
void port_writeAdpd_LED_magenta(uint8_t value);
void port_writeAdpd_LED_red(uint8_t value);
void port_writeAdpd_LEDs(uint8_t value);

#endif // EMB_PORT_H_
