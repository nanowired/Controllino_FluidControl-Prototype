/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file config
 ******************************************************************************
 * Copyright (C) NWI GmbH, 2023
 ******************************************************************************
 *
 * \brief  Configuration
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

#ifndef NWI_CONFIG_H_
#define NWI_CONFIG_H_

#define CONTROLLINO_MAXI_AUTOMATION
#include <Controllino.h>

// hardware info
#define HW_INFO "Controllino Maxi Controller"
#define SW_VERSION "1.3"

// module type MCP jumper settings
#define CFG_MODULE_PTT      (3)          // PTT Controller, no Extension Box

#define SPI1_SS             (53)         // SPI1

#define SPI2_MISO           (22)         // SPI2
#define SPI2_MOSI           (23)         // SPI2
#define SPI2_SCK            (25)         // SPI2
#define SPI2_SS             (27)         // SPI2
#define SPI2_SPEED          (1000000)    // SPI2

#define TEST_PIN            (46)         // output

//      Pin                 definitions  of MCP23017
#define CFG_SERVICE         (0)          // input
#define CFG_ETH_DHCP_EN     (1)          // input
#define CFG_CTRL_MODE       (2)          // input
#define ADC_ALERT           (3)          // input
#define TEST_LED1           (7)          // output
#define TEST_LED2           (6)          // output
#define TEST_LED3           (5)          // output
#define TEST_LED4           (4)          // output
#define SPI_PT_CS1          (10)         // output
#define SPI_PT_CS2          (9)          // output
#define SPI_PT_CS3          (8)          // output

//      I2C                 addresses
#define INA260_VCP_IN_ADDR  (0x40)
#define INA260_VCP_OUT_ADDR (0x41)

#define SERIAL_BUFFER_SIZE  (128)
//      #define             BUZZER       (CONTROLLINO_AI0)
#define BUZZER (CONTROLLINO_R9)
enum
{
  CFG_DIGITAL,
  CFG_ANALOG
};

// port definition
typedef struct
{
  uint8_t portNum;
  uint8_t direction; // Bit 7 is port state HIGH
} CFG_ports_T;

// digital output port definition
typedef struct
{
  uint8_t portNum;
  char *name;
} CFG_DoutPorts_T;

// digital and analog input port definition
typedef struct
{
  uint8_t portNum;
  uint8_t mode; //  CFG_DIGITAL or CFG_ANALOG
  char *name;
} CFG_inputPorts_T;

// digital and analog input port definition
#endif /* NWI_CONFIG_H_ */
