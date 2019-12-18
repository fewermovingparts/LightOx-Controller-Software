#ifndef SYSTEMHEADERS_H
#define SYSTEMHEADERS_H

#include <Arduino.h>

#ifdef __GNUC__
// Avoid tons of warnings from library code
#pragma GCC system_header
#endif



#include <BlockDriver.h>
#include <Ch376msc.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <SdFat.h>  //SD card library
#include <SdFatConfig.h>
#include <SysCall.h>
#include <sdios.h>

#include <FT_NHD_43CTP_SHIELD.h>
//#include <FT_VM801B43.h>
//#include <FT_VM801P43_50.h>
//#include "FT_NHD_43RTP_SHIELD.h"  //new screen
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <OneWire.h>  //control devices (from Dallas Semiconductor) that use the One Wire protocol
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Wire.h>  //Allows communication with I2C devices via SDA (data line) and SCL (clock line)
#include <stdio.h>
#include "Adafruit_VEML6070.h"  //UV sensor

#endif
