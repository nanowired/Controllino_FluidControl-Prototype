/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file Galvanic-NT-Arduino-uno.ino
 ******************************************************************************
 * Copyright (C) NWI GmbH, 2025
 ******************************************************************************
 *
 * \brief
 *
 * used libraries:  
                    ArduinoJson 7.4.2
                    Controllino 3.0.7
                    OneWire 2.3.6
                    DallasTemperature 3.9.0
                    // Adafruit_SPIDevice
                    Adafruit_MCP23017 2.0.2
                    Adafruit_ADS1X15 2.4.0
                    Adafruit_INA260 1.5.0
                    Adafruit_Max31865 1.3.0
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

/* --- Includes, Defines, Local Types  -------------------------------------- */

#pragma region Includes_Defines_Local_Types
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h>
#include "at24mac402.h"
#include "Timer.h"
#include "port.h"
#include "sensor.h"

#include "app_gal.h"
#include "app_coc.h"
#include "app_sns.h"
#include "app_ptt.h"

#include "config.h"

#define ETH_WATCHDOG_COUNTER_TRIGGER_VAL (10) // in seconds
#define BROKER_ACCESS_KEY F("123456789")
/* --- Local Variables ------------------------------------------------------ */
// Set the static IP address to use if the DHCP fails to assign
// IPAddress gal_static_ip(192, 168, 35, 20);    // remove
// IPAddress coc_static_ip(192, 168, 35, 21);    // remove
// IPAddress sns_static_ip(192, 168, 35, 22);    // remove
// IPAddress hsh_static_ip(192, 168, 0, 105);

// J
// IPAddress server_static_ip(169, 254, 212, 188);
// IPAddress hsh_static_ip(169, 254, 212, 190);
// IPAddress static_dns(169, 254, 212, 200);
// IPAddress server_ip(169, 254, 212, 188);

// B
IPAddress server_static_ip(192, 168, 35, 10);
IPAddress ptt_static_ip(192, 168, 35, 23);
IPAddress static_dns(192, 168, 0, 1);
IPAddress server_ip(192, 168, 35, 10);

// initialize the library instance:
EthernetClient client;

unsigned long lastConnectionTime      = 0;   // last time you connected to the server, in milliseconds
const unsigned long postingInterval   = 100; // delay between updates, in milliseconds
unsigned long packet_counter          = 0;   // give every request a unique id
unsigned int ethernet_watchDogCounter = 0;   // error counter for ethernet watchdog -> safe state if broker communication is interrupted
static bool saveStateFlag             = false;

StaticJsonDocument<500> doc_send_cmd;
// received broker commands
StaticJsonDocument<500> doc_recv_cmd;

Timer timer;

bool flag                      = true;
bool flag100ms                 = false;
bool flag1s                    = false;
bool httpRequestflag           = false;
String req_str                 = "";
static String jsonString       = "";
long last_millis               = 0;
long _now                      = millis();
int _cyctime                   = last_millis == 0 ? 0: _now - last_millis;
const char moduleTypeStr[4][5] = {"GAL-", "COC-", "SNS-", "PTT-"};
String _host_ip;
String _device_name;
// volatile byte state = LOW;

#pragma endregion Includes_Defines_Local_Types

/* --- Global Functions ----------------------------------------------------- */
#pragma region Global_Functions
void Show_Cycle_Time(char *pre = "")
{
  _now        = millis();
  _cyctime    = last_millis == 0 ? 0 : _now - last_millis;
  last_millis = _now;
  Serial.print(pre);
  Serial.print(" cycle time is: ");
  Serial.print(_cyctime);
  Serial.println(" ms");
}

// System iniialization
void setup()
{
  delay(1000);
  // Serial communication
  Serial.begin(115200);
  Serial.println();
  Serial.print(HW_INFO);
  Serial.print(F(" Firmware: V"));
  Serial.println(F(SW_VERSION));
  // Serial1.begin(115200);

  // chk [mcp,sensors] & init serial
  initHardware();

  // scan all ports
  i2c_scanner();

  // select application
  switch (port_getModuleType())
  {
  case CFG_MODULE_PTT: // remove the ther modules from the declaration
    app_ptt_init();
    break;
  default:
    break;
  }
  setup_network();

  // Timer initialitation
  timer.init(100);
  timer.attachIsrFunc(timerFunc);

  // buzzer for endup setup
  for (size_t i = 0; i < 3; i++)
  {
    delay(150);
    digitalWrite(BUZZER, HIGH);
    delay(150);
    digitalWrite(BUZZER, LOW);
  }
}
// Main loop
void loop()
{
  // Show_Cycle_Time("main ");
  loop_network();
}

#pragma endregion Global Functions

/* --- Local Functions ----------------------------------------------------- */

#pragma region Local_Functions
// Timer function, called cyclic 100ms
void timerFunc()
{
  // Serial.println(F("Inside timerfunction"));
  static uint8_t cnt = 0;
  // schduler controller
  // 0.1 seconds task
  if (!(cnt % 10))
  {
    flag100ms = true;
    // timerFunc_stepper();

    // 1 seconds task
    if (!(cnt % 100))
    {
      flag1s = true;

      // Manage network watchdog
      if (ethernet_watchDogCounter < (ETH_WATCHDOG_COUNTER_TRIGGER_VAL - 1))
      {
        ethernet_watchDogCounter++;
      }
      else
      {
        saveStateFlag = true;
      }
    }
  }

  cnt++;
  if (cnt >= 100)
  {
    cnt = 0;
  }
}

// Hardware initialization
void initHardware()
{
  //  init MCP23017 ports
  //  port_initMcp();
  //  port_initMcpAdpt();
  //  sensor_init();
  //  Serial.println();
}

// Scans the I2C-Bus and prints the found addresses, only for tests
void i2c_scanner()
{
  int nDevices = 0;

  Serial.print(F("I2C-Scanning -> "));
  Wire.begin();
  for (byte address = 1; address < 127; ++address)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0)
    {
      // Serial.print(F("I2C device found at address 0x"));
      if (address < 16)
      {
        Serial.print(F("0"));
      }
      Serial.print(address, HEX);
      Serial.print(" ");

      ++nDevices;
    }
  }

  if (nDevices == 0)
  {
    Serial.println(F("No I2C devices found\n"));
  }
  else
  {
    Serial.println(F("device addresse(s) found"));
  }
}

// setup network
void setup_network()
{
  // read UID
  (void)at24mac402_readUid();

  _host_ip     = server_ip[0] + String(".") + String(server_ip[1]) + String(".") + String(server_ip[2]) + String(".") + String(server_ip[3]);
  _device_name = moduleTypeStr[port_getModuleType()];
  _device_name += at24mac402_getUidStr();

  // print serial number
  Serial.print(F("SN: "));
  Serial.print(moduleTypeStr[port_getModuleType()]);
  Serial.println(at24mac402_getUidStr());

  // start the Ethernet connection:
  if (port_getCfgJmpEthDhcpEn())
  {
    Ethernet.begin((uint8_t *)at24mac402_readMac());
    Serial.print(F("Initialize Ethernet with DHCP: "));
  }
  else
  {
    server_ip = server_static_ip;

    // try to congifure using static IP address:
    switch (port_getModuleType())
    {
    case CFG_MODULE_PTT:
      Ethernet.begin((uint8_t *)at24mac402_readMac(), ptt_static_ip, static_dns); //,
      break;
    default:
      break;
    }

    Serial.println(F("Initialize Ethernet with static IP: "));
  }

  Serial.print(F("localIP: "));
  Serial.println(Ethernet.localIP());
  Serial.print(F("Server Address: "));
  Serial.println(server_ip);

  //** printout the MAC address */
  byte macBuffer[6];              // create a buffer to hold the MAC address
  Ethernet.MACAddress(macBuffer); // fill the buffer
  Serial.print(F("MAC address: "));
  for (byte octet = 0; octet < 6; octet++)
  {
    Serial.print(macBuffer[octet], HEX);
    if (octet < 5)
    {
      Serial.print('-');
    }
  }

  if (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println(F("Ethernet cable is not connected."));
  }

  Serial.print(F("ETH-Chip: "));
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println(F("Ethernet shield was not found.  Sorry, can't run without hardware. :("));
    while (true)
    {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  else if (Ethernet.hardwareStatus() == EthernetW5100)
  {
    Serial.println(F("W5100"));
  }
  else if (Ethernet.hardwareStatus() == EthernetW5200)
  {
    Serial.println(F("W5200"));
  }
  else if (Ethernet.hardwareStatus() == EthernetW5500)
  {
    Serial.println(F("W5500"));
  }

  // give the Ethernet shield a second to initialize:
  delay(1000);
  // switch LEDs off
  port_writeAdpd_LEDs(0);
}

// Main loop .....
void loop_network()
{
  // if (rcvDataFromEth())
  //   return;
  rcvDataFromEth();
  scheduler();
  sndDataToEth();
}
// receive data from lan
bool rcvDataFromEth()
{
  if (client.available())
  {
    // Serial.println(F("client available"));
    // req_str = client.readStringUntil('\n');
    req_str = "";
    char c = 0;
    // sample a line
    do
    {
      c = client.read();

      req_str += c;
    } while ((c != '\n') && (c != -1));

    // reset watchdog counter and reset save state flag
    ethernet_watchDogCounter = 0;
    saveStateFlag = false;

    if (!req_str.startsWith("{"))
      return false;

    // cut JSON string
    req_str = req_str.substring(0, req_str.length() - 1);
    DeserializationError error = deserializeJson(doc_recv_cmd, req_str);

    if (error)
    {
      Serial.println("Error in doc_recv_cmd  deserializeJson ...." + req_str);
      return false;
    }
    if ((doc_recv_cmd["status"] | -1) != 1)
      return false;
    // Serial.println(req_str);

    // port_writeAdpd_LED_yellow(1);
    //  select application for set value
    switch (port_getModuleType())
    {
    case CFG_MODULE_PTT:
      app_ptt_setValues(doc_recv_cmd);
      break;
    default:
      break;
    }
    // port_writeAdpd_LED_yellow(0);
    // clear request string
    req_str = "";
    return true;
  }
  return false;
}

// scheduler for timer tasks
void scheduler()
{
  // scheduler PIN -> DOC
  if (flag100ms)
  {
    // select application
    switch (port_getModuleType())
    {
    case CFG_MODULE_PTT:
      app_ptt_task100ms(doc_send_cmd);
      break;
    default:
      break;
    }
    httpRequestflag = true;
    flag100ms = false;
  }

  if (flag1s)
  {
    // select application
    switch (port_getModuleType())
    {
    case CFG_MODULE_PTT:
      if (saveStateFlag)
        app_ptt_enterSaveState();
      app_ptt_task1s();
      break;
    default:
      break;
    }

    // port_toggleAdpd_LED_green();
    flag1s = false;
  }
}

// send data to lan
void sndDataToEth()
{
  if (httpRequestflag)
  {
    httpRequest();
    httpRequestflag = false;
  }
}

// this method makes a HTTP connection to the server:
void httpRequest()
{
  packet_counter++;
  //  close any connection before send a new request.
  //  This will free the socket on the WiFi shield
  client.stop();
  // if there's a successful connection:
  if (client.connect(server_ip, 3333))
  {
    // Serial.println(F("connected"));
    if (!(doc_send_cmd.isNull()))
    {
      // port_writeAdpd_LED_blue(1);

      serializeJson(doc_send_cmd, jsonString);
      //Serial.println(jsonString);

      // send the HTTP GET request:
      client.println("POST /device/data HTTP/1.1");

      client.print("Host: "); // Replace with your server's IP address
      client.println(_host_ip);

      client.print("device-name: ");
      client.println(_device_name);

      client.print("device-key: ");
      client.println(BROKER_ACCESS_KEY);

      client.print("device-req-id: ");
      client.println(packet_counter);

      client.println("Content-Type: application/json");
      client.println("Connection: close");

      client.print("Content-Length: ");
      client.println(jsonString.length());

      client.println();
      // Send the JSON string
      client.println(jsonString);
      jsonString = "";
      // port_writeAdpd_LED_blue(0);
    }
  }
  else
  {
    Serial.print(F("server not connected "));
    Serial.println(packet_counter);
  }
}

#pragma endregion Local_Functions