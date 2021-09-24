#ifndef ANAIRE_WIFI
#define ANAIRE_WIFI
#include <Arduino.h>
#include <Print.h>

// WiFi
#include <ESP8266WiFi.h>                          // Wifi ESP8266
extern "C" {
#include "user_interface.h"
#include "wpa2_enterprise.h"
#include "c_types.h"
}

#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>                         // to be reached on anaire_device_id.local in the local network

//EEPROM:
#include "eeprom.h"
#include "global-vars.h"
#include "config.h"
#include "../sensors/sensors.h"

void Connect_WiFi(void);
void Print_WiFi_Status(void);
void Check_WiFi_Server(void);

#endif